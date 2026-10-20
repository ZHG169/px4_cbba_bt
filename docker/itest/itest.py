#!/usr/bin/env python3
"""整合測試：只看話題和網路，不看 log，換成別人寫的 CBBA Node／BT 也能用（介面寫在 config.json）。

  python3 docker/itest/itest.py comm     CBBA 之間的通訊（不開 PX4、BT）：每台在自己的容器啟動 CBBA Node，
                                         位置用假的 robot_state，測分配、完成、失敗交回、停止送狀態、節點停止、
                                         重開機、掉包 30%、斷網
  python3 docker/itest/itest.py uav      UAV 整串（PX4 SITL＋Gazebo＋CBBA Node＋BT）：起飛、單一任務、火警、
                                         誘餌、三台分配與避碰、PX4 斷線接手
  python3 docker/itest/itest.py demo     展示：demo_site 開 Gazebo 畫面（錄成 gazebo.mp4），全部起飛後才開始 CBBA，3 個固定任務；出發後火點出現在某台的
                                         航線上，飛行途中發現、發布地面任務（給 V60，先不開狗）、繼續飛；結束後畫俯視動畫
                                         demo.mp4（要主機的 ffmpeg）。
                                         --seed N 重現同一個情境、--speed 3（m/s）、--hold 10、--no-record
  python3 docker/itest/itest.py loss     第 5 週弱網量測（不開 PX4、BT）：掉包梯度 0／10／30／50／70%（延遲 50±30 ms），
                                         每個掉包率試驗 N 次、每次三台各建立一個任務（多源插單），記錄收斂時間、
                                         5 s 內收斂率、重複認領、完成後撤下的時間、mesh 頻寬（trials.csv、levels.csv）

在主機執行（要能用 docker），容器要先用 docker/start.sh 開好、套件要先編譯好。
  --config 檔案       換成別的設定（預設同目錄的 config.json）
  --only C1,C3        只跑指定的項目（前置步驟照跑）
  --kill-existing     容器裡已經有在跑的節點、PX4、Gazebo 時先關掉（預設會停下來，避免打斷你正在做的事）
  --levels 0,10,30    loss：要測的掉包率（%）；--trials 20 每個掉包率試驗幾次；--delay 50 --jitter 30（ms）

結果：每一項 PASS／FAIL／INFO，寫到 docker/itest/out/<時間>/report.md（各節點的輸出也在那個目錄）。
"""
import argparse
import json
import math
import os
import random
import re
import select
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


# =============================================================================
# 設定、容器
# =============================================================================
class Env:
    def __init__(self, cfg_path):
        with open(cfg_path, encoding="utf-8") as f:
            self.cfg = json.load(f)
        self.cfg_path = os.path.abspath(cfg_path)
        p = self.cfg["paths"]
        self.host_repo = os.path.normpath(os.path.join(os.path.dirname(self.cfg_path), p["host_repo"]))
        self.repo = p["container_repo"]
        self.ids = self.cfg["vehicles"]["ids"]

    def ns(self, vid):
        return self.cfg["vehicles"]["ns"].format(id=vid)

    def sim(self):
        return self.cfg["containers"]["sim"]

    def uav(self, vid):
        return self.cfg["containers"]["uav"].format(id=vid)

    def to_container(self, host_path):
        rel = os.path.relpath(os.path.abspath(host_path), self.host_repo)
        if rel.startswith(".."):
            raise SystemExit(f"{host_path} 不在 repo 裡，容器看不到")
        return os.path.join(self.repo, rel)

    def prefix(self):
        return (f"source /opt/ros/jazzy/setup.bash; source {self.cfg['paths']['ros_setup']}; "
                f"export PATH={self.repo}/docker/scripts:$PATH; ")

    def exec(self, container, script, detach=False, log=None, env=None, check=False):
        """在容器裡用 ncrl 執行 bash script（已經 source ROS、PATH 有 docker/scripts）"""
        if log:
            script = f"{{ {script} ; }} > {log} 2>&1"
        cmd = ["docker", "exec"] + (["-d"] if detach else []) + ["-u", "ncrl"]
        for k, v in (env or {}).items():
            cmd += ["-e", f"{k}={v}"]
        cmd += [container, "bash", "-c", self.prefix() + script]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if check and r.returncode != 0:
            raise RuntimeError(f"{container}: {script}\n{r.stderr.strip()}")
        return r

    def pkill(self, container, pattern):
        subprocess.run(["docker", "exec", container, "pkill", "-f", pattern], capture_output=True)

    def pkill_hard(self, container, pattern, wait=3.0):
        """先 SIGTERM，wait 秒後還在就 SIGKILL（Gazebo 伺服器卡住時會不理 SIGTERM，2026-10-12 遇到：
        殘留的伺服器讓之後的 px4_sitl.sh 不開視窗、也讀不到世界）；回傳最後是否關掉"""
        self.pkill(container, pattern)
        deadline = time.time() + wait
        while time.time() < deadline:
            if not self.running(container, pattern):
                return True
            time.sleep(0.3)
        subprocess.run(["docker", "exec", container, "pkill", "-9", "-f", pattern], capture_output=True)
        time.sleep(0.5)
        return not self.running(container, pattern)

    def running(self, container, pattern):
        r = subprocess.run(["docker", "exec", container, "pgrep", "-f", pattern], capture_output=True, text=True)
        return r.returncode == 0

    def mesh_bytes(self, vid):
        """這台 mesh 網卡累計收送的位元組"""
        script = ('IF=$(ip -o -4 addr | awk -v ip="$MESH_IP" \'{split($4,a,"/"); if (a[1]==ip) print $2}\'); '
                  'IF=${IF%%@*}; awk -v i="$IF:" \'$1==i{print $2+$10}\' /proc/net/dev')
        r = subprocess.run(["docker", "exec", self.uav(vid), "bash", "-c", script], capture_output=True, text=True)
        try:
            return int(r.stdout.strip())
        except ValueError:
            return None


def fill(obj, values):
    """把設定裡的 "{名稱}" 換成數值（整個字串是一個佔位符時保留數值型別）"""
    if isinstance(obj, dict):
        return {k: fill(v, values) for k, v in obj.items() if not k.startswith("_")}
    if isinstance(obj, list):
        return [fill(v, values) for v in obj]
    if isinstance(obj, str):
        if obj.startswith("{") and obj.endswith("}") and obj[1:-1] in values:
            return values[obj[1:-1]]
        return obj.format(**values)
    return obj


# =============================================================================
# 和 ros_bridge 的連線：commands.jsonl（寫）、events.jsonl（讀）
# =============================================================================
class Run:
    def __init__(self, env, out_dir, with_px4):
        self.env = env
        self.out = out_dir
        self.cout = env.to_container(out_dir)
        self.next_id = 1
        self.events = []
        self.pos = 0
        self.assigned = {}       # uav -> (task_id, version, t)
        self.history = []        # (t, uav, task_id, version)：每次 assigned 改變
        self.positions = {}      # uav -> (x, y, z, t)
        self.armed = {}
        self.cmd_file = os.path.join(out_dir, "commands.jsonl")
        open(self.cmd_file, "w").close()
        open(os.path.join(out_dir, "events.jsonl"), "w").close()
        env.exec(env.sim(), f"exec python3 {env.repo}/docker/itest/ros_bridge.py --config {env.to_container(env.cfg_path)}"
                 f" --dir {self.cout}" + (" --px4" if with_px4 else ""), detach=True, log=f"{self.cout}/ros_bridge.log")
        if not self.wait(lambda e: e["kind"] == "ready", 20):
            raise SystemExit(f"ros_bridge 沒有啟動，看 {out_dir}/ros_bridge.log")

    # ---------- 事件 ----------
    def read(self):
        with open(os.path.join(self.out, "events.jsonl"), "rb") as f:
            f.seek(self.pos)
            chunk = f.read()
        complete = chunk.rfind(b"\n") + 1     # 只處理完整的行（ros_bridge 可能寫到一半）
        self.pos += complete
        new = [json.loads(l) for l in chunk[:complete].decode("utf-8").splitlines() if l.strip()]
        f_id, f_ver = self.env.cfg["fields"]["task_id"], self.env.cfg["fields"]["version"]
        for e in new:
            if e["kind"] == "assigned":
                tid = e["msg"].get(f_id)
                ver = e["msg"].get(f_ver) if f_ver else None
                self.assigned[e["uav"]] = (tid, ver, e["t"])
                self.history.append((e["t"], e["uav"], tid, ver))
            elif e["kind"] == "pos":
                self.positions[e["uav"]] = (e["x"], e["y"], e["z"], e["t"])
            elif e["kind"] == "status":
                self.armed[e["uav"]] = e["armed"]
            elif e["kind"] == "error":
                print(f"  [ros_bridge] 錯誤：{e['error']}")
        self.events.extend(new)
        return new

    def wait(self, pred, timeout, step=0.05):
        """等到某個新事件符合 pred，回傳那個事件；逾時回傳 None"""
        deadline = time.time() + timeout
        while True:
            for e in self.read():
                if pred(e):
                    return e
            if time.time() >= deadline:
                return None
            time.sleep(step)

    def until(self, cond, timeout, step=0.05):
        """等到 cond()（看目前的狀態）成立；回傳成立的時刻，逾時回傳 None"""
        deadline = time.time() + timeout
        while True:
            self.read()
            if cond():
                return time.time()
            if time.time() >= deadline:
                return None
            time.sleep(step)

    def sleep(self, seconds):
        self.until(lambda: False, seconds)

    # ---------- 指令 ----------
    def send(self, op, wait_ack=True, **kw):
        cid = self.next_id
        self.next_id += 1
        with open(self.cmd_file, "a", encoding="utf-8") as f:
            f.write(json.dumps(dict(kw, op=op, id=cid), ensure_ascii=False) + "\n")
        if wait_ack:
            ev = self.wait(lambda e: e.get("id") == cid and e["kind"] in ("ack", "error"), 10)
            if ev is None or ev["kind"] == "error":
                raise RuntimeError(f"ros_bridge 沒有完成 {op}：{ev}")
            return ev
        return None

    def topic(self, kind, vid):
        spec = self.env.cfg["topics"][kind]
        return dict(topic=spec["name"].format(ns=self.env.ns(vid), id=vid), type=spec["type"],
                    reliable=spec.get("reliable", True), transient_local=spec.get("transient_local", False))

    def publish(self, kind, vid, fields):
        return self.send("pub", fields=fields, **self.topic(kind, vid))

    def quit(self):
        try:
            self.send("quit")
        except RuntimeError:
            pass

    # ---------- 查詢 ----------
    def holders(self, tid):
        return sorted(u for u, (t, _, _) in self.assigned.items() if t == tid)

    def ever_held(self, tid, since=0.0):
        return sorted({u for (t, u, task, _) in self.history if task == tid and t >= since})


# =============================================================================
# 測試項目
# =============================================================================
class Results:
    def __init__(self):
        self.rows = []

    def add(self, code, name, status, detail, **metrics):
        self.rows.append(dict(code=code, name=name, status=status, detail=detail, metrics=metrics))
        mark = {"PASS": "✓", "FAIL": "✗", "INFO": "·", "SKIP": "-"}[status]
        print(f"  {mark} {code} {name}：{status}　{detail}")

    def write(self, out_dir, title):
        lines = [f"# {title}", "", f"{time.strftime('%Y-%m-%d %H:%M:%S')}", "",
                 "| 項目 | 名稱 | 結果 | 說明 |", "|---|---|---|---|"]
        for r in self.rows:
            lines.append(f"| {r['code']} | {r['name']} | {r['status']} | {r['detail']} |")
        with open(os.path.join(out_dir, "report.md"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        with open(os.path.join(out_dir, "report.json"), "w", encoding="utf-8") as f:
            json.dump(self.rows, f, ensure_ascii=False, indent=2)
        n_fail = sum(r["status"] == "FAIL" for r in self.rows)
        n_pass = sum(r["status"] == "PASS" for r in self.rows)
        print(f"\n  通過 {n_pass}、失敗 {n_fail}；報告：{out_dir}/report.md")
        return n_fail


class Suite:
    def __init__(self, env, run, only):
        self.env = env
        self.run = run
        self.cfg = env.cfg
        self.only = only
        self.res = Results()
        self.seq = 0x2000 + int(time.time()) % 0x5000   # 流水號：和之前的執行、BT 的火警（0x8000～）錯開
        self.creator = env.ids[0]

    def want(self, code):
        return not self.only or code in self.only

    def new_task(self, x, y, duration=5.0, creator=None, value=None):
        creator = creator or self.creator
        self.seq += 1
        tid = creator * 65536 + self.seq
        fields = fill(self.cfg["messages"]["new_task"],
                      dict(task_id=tid, x=float(x), y=float(y), duration=float(duration)))
        if value is not None:
            fields["value"] = float(value)
        self.run.publish("new_task", creator, fields)
        return tid, time.time()

    def report(self, vid, tid, success):
        _, ver, _ = self.run.assigned.get(vid, (None, None, None))
        key = "result_success" if success else "result_failure"
        fields = fill(self.cfg["messages"][key], dict(task_id=tid, version=ver if ver is not None else 0))
        if not self.cfg["fields"]["version"]:
            fields.pop("assignment_version", None)
        self.run.publish("result", vid, fields)

    def wait_holder(self, tid, timeout, exclude=()):
        """等到有人（不在 exclude 裡）拿到 tid；回傳 (機號, 時刻) 或 (None, None)"""
        t = self.run.until(lambda: [u for u in self.run.holders(tid) if u not in exclude], timeout)
        if t is None:
            return None, None
        return [u for u in self.run.holders(tid) if u not in exclude][0], t

    def wait_released(self, vid, tid, timeout):
        return self.run.until(lambda: self.run.assigned.get(vid, (None,))[0] != tid, timeout)

    def finish(self, tid, timeout=10.0):
        """收尾：讓目前持有者回報完成（失敗的項目也不要留下沒做完的任務干擾後面）"""
        h, _ = self.wait_holder(tid, timeout)
        if h is not None:
            self.report(h, tid, True)
            self.wait_released(h, tid, 5.0)


# -----------------------------------------------------------------------------
# 通訊（comm）
# -----------------------------------------------------------------------------
class CommSuite(Suite):
    def positions(self):
        return {int(k): v for k, v in self.cfg["comm"]["positions"].items()}

    def nearest(self, x, y):
        return min(self.positions().items(), key=lambda kv: math.hypot(kv[1][0] - x, kv[1][1] - y))[0]

    def state_on(self, vid):
        x, y = self.positions()[vid]
        fields = fill(self.cfg["messages"]["robot_state"], dict(id=vid, x=float(x), y=float(y)))
        self.run.send("periodic", key=f"rs{vid}", rate=10, fields=fields, **self.run.topic("robot_state", vid))

    def state_off(self, vid):
        self.run.send("stop_periodic", key=f"rs{vid}")

    def start_node(self, vid):
        c = self.cfg["comm"]
        self.env.exec(self.env.uav(vid), "exec " + c["start"].format(id=vid, ns=self.env.ns(vid)), detach=True,
                      log=f"{self.run.cout}/node_uav{vid}.log")

    def stop_node(self, vid):
        self.env.pkill(self.env.uav(vid), self.cfg["comm"]["stop_pattern"].format(id=vid))

    def tc(self, loss, delay=50, jitter=30):
        for vid in self.env.ids:
            if loss is None:
                self.env.exec(self.env.uav(vid), "mesh_tc.sh clear")
            else:
                self.env.exec(self.env.uav(vid), f"mesh_tc.sh set {loss} {delay} {jitter}", check=True)

    def start_all(self):
        """每台送假的 robot_state、啟動 CBBA Node，等到各台的 assigned 都收到；回傳是否成功"""
        ids = self.env.ids
        for vid in ids:
            self.state_on(vid)
        for vid in ids:
            self.start_node(vid)
        return self.run.until(lambda: all(v in self.run.assigned for v in ids), self.cfg["comm"]["warmup"] + 15)

    def run_all(self):
        env, run, res = self.env, self.run, self.res
        ids = env.ids
        for vid in ids:
            self.state_on(vid)
        for vid in ids:
            self.start_node(vid)
        # C0：節點上線（assigned_task 是 transient_local，開起來就會有一則「沒有任務」）
        t = run.until(lambda: all(v in run.assigned for v in ids), self.cfg["comm"]["warmup"] + 15)
        missing = [v for v in ids if v not in run.assigned]
        res.add("C0", "節點上線、話題看得到", "PASS" if t else "FAIL",
                "各台的 assigned 都收到" if t else f"收不到 uav{missing} 的 assigned（節點沒起來或話題名稱不對）")
        if not t:
            return
        run.sleep(self.cfg["comm"]["warmup"])
        b0, t0 = env.mesh_bytes(ids[0]), time.time()
        run.sleep(3.0)
        b1 = env.mesh_bytes(ids[0])
        if b0 is not None and b1 is not None:
            res.add("M", "mesh 流量（待命、沒有任務）", "INFO", f"uav{ids[0]} 約 {(b1 - b0) / (time.time() - t0) / 1024:.1f} kB/s")

        if self.want("C1"):
            x, y = 18.0, 2.0
            tid, t0 = self.new_task(x, y)
            h, th = self.wait_holder(tid, 5.0)
            if h is None:
                res.add("C1", "任務傳到全隊、只有一台得標", "FAIL", "5 s 內沒有任何一台拿到")
            else:
                run.sleep(2.0)
                ever = run.ever_held(tid, t0)
                ok = run.holders(tid) == [h] and ever == [h]
                near = self.nearest(x, y)
                res.add("C1", "任務傳到全隊、只有一台得標", "PASS" if ok else "FAIL",
                        f"uav{h} 得標，建立 → 指派 {(th - t0) * 1000:.0f} ms" + ("" if ok else f"；拿過的：{ever}") +
                        ("" if h == near else f"（最近的是 uav{near}）"), latency=th - t0)
                if self.want("C2"):
                    t1 = time.time()
                    self.report(h, tid, True)
                    tr = self.wait_released(h, tid, 3.0)
                    run.sleep(2.0)
                    again = [u for u in run.ever_held(tid, t1) if u != h]
                    ok = tr is not None and not again and not run.holders(tid)
                    res.add("C2", "回報完成後全隊不再派這個任務", "PASS" if ok else "FAIL",
                            (f"{(tr - t1) * 1000:.0f} ms 後撤下" if tr else "3 s 內沒有撤下") +
                            (f"；之後又派給 {again}" if again else ""))
                else:
                    self.finish(tid)

        if self.want("C3"):
            tid, _ = self.new_task(18.0, 2.0)
            h, _ = self.wait_holder(tid, 5.0)
            if h is None:
                res.add("C3", "失敗交回、別台接手", "FAIL", "沒有人拿到任務")
            else:
                t1 = time.time()
                self.report(h, tid, False)
                h2, t2 = self.wait_holder(tid, 5.0, exclude=(h,))
                res.add("C3", "失敗交回、別台接手", "PASS" if h2 else "FAIL",
                        f"uav{h} 失敗 → uav{h2}，{(t2 - t1) * 1000:.0f} ms" if h2 else f"uav{h} 失敗後 5 s 沒人接")
                self.finish(tid)

        if self.want("C4"):
            x, y = self.positions()[ids[1]]
            tid, _ = self.new_task(x, y + 5.0)
            h, _ = self.wait_holder(tid, 5.0)
            if h is None:
                res.add("C4", "停止送狀態（等同 PX4 斷線）→ 別台接手", "FAIL", "沒有人拿到任務")
            else:
                t1 = time.time()
                self.state_off(h)
                h2, t2 = self.wait_holder(tid, 10.0, exclude=(h,))
                res.add("C4", "停止送狀態（等同 PX4 斷線）→ 別台接手", "PASS" if h2 else "FAIL",
                        f"uav{h} 停送 → uav{h2}，{t2 - t1:.1f} s" if h2 else f"uav{h} 停送後 10 s 沒人接")
                self.state_on(h)
                self.finish(tid)

        if self.want("C5"):
            x, y = self.positions()[ids[-1]]
            tid, _ = self.new_task(x + 5.0, y - 3.0)
            h, _ = self.wait_holder(tid, 5.0)
            if h is None:
                res.add("C5", "節點整個停止 → 別台接手", "FAIL", "沒有人拿到任務")
            else:
                t1 = time.time()
                self.stop_node(h)
                run.assigned.pop(h, None)      # 停掉的節點不會再更新，從目前狀態拿掉
                h2, t2 = self.wait_holder(tid, 40.0, exclude=(h,))
                res.add("C5", "節點整個停止 → 別台接手", "PASS" if h2 else "FAIL",
                        f"uav{h} 停止 → uav{h2}，{t2 - t1:.1f} s" if h2 else
                        f"uav{h} 停止後 40 s 沒人接（失聯接手：火警 10 s、巡檢 30 s，見 CLAUDE.md；還沒實作時會失敗）")
                self.start_node(h)
                run.until(lambda: h in run.assigned, 15)
                self.finish(tid)

        if self.want("C6"):
            v = ids[1]
            self.stop_node(v)
            run.assigned.pop(v, None)
            run.sleep(2.0)
            self.start_node(v)
            up = run.until(lambda: v in run.assigned, 15)
            run.sleep(3.0)
            x, y = self.positions()[v]
            tid, _ = self.new_task(x + 2.0, y + 2.0)
            h, _ = self.wait_holder(tid, 5.0)
            run.sleep(1.5)
            ok = up and h == v and run.holders(tid) == [v]
            res.add("C6", "重開機後重新加入", "PASS" if ok else "FAIL",
                    f"uav{v} 重開後拿到身邊的任務" if ok else f"重開後：上線 {bool(up)}、持有者 {run.holders(tid)}")
            self.finish(tid)

        if self.want("C7"):
            self.tc(30)
            try:
                tid, t0 = self.new_task(18.0, 2.0)
                h, th = self.wait_holder(tid, 10.0)
                run.sleep(2.0)
                ever = run.ever_held(tid, t0)
                if h is None:
                    res.add("C7", "掉包 30%（延遲 50±30 ms）", "FAIL", "10 s 內沒人拿到")
                else:
                    t1 = time.time()
                    self.report(h, tid, True)
                    tr = self.wait_released(h, tid, 10.0)
                    ok = ever == [h] and tr is not None
                    res.add("C7", "掉包 30%（延遲 50±30 ms）", "PASS" if ok else "FAIL",
                            f"建立 → 指派 {th - t0:.2f} s、完成 → 撤下 " + (f"{tr - t1:.2f} s" if tr else "逾時") +
                            ("" if ever == [h] else f"；重複認領：{ever}"), latency=th - t0)
                    self.finish(tid)
            finally:
                self.tc(None)

        if self.want("C8"):
            far = ids[-1]
            x, y = self.positions()[far]
            self.tc(100)
            try:
                tid, t0 = self.new_task(x + 1.0, y + 1.0)
                h, _ = self.wait_holder(tid, 5.0)
                run.sleep(1.0)
                cut_ok = h == self.creator and run.ever_held(tid, t0) == [self.creator]
            finally:
                self.tc(None)
            run.sleep(8.0)
            ever = run.holders(tid)
            ok = cut_ok and len(ever) <= 1
            res.add("C8", "斷網：只有建立者知道（協調確實走 mesh）；恢復後不重複", "PASS" if ok else "FAIL",
                    (f"斷網時只有 uav{self.creator} 拿到（身邊的 uav{far} 不知道）" if cut_ok else
                     f"斷網時持有者 {run.ever_held(tid, t0)}（應該只有建立者）") + f"；恢復後持有者 {ever}")
            self.finish(tid)


# -----------------------------------------------------------------------------
# 第 5 週弱網量測（loss）
# -----------------------------------------------------------------------------
class LossSuite(CommSuite):
    """掉包梯度：每個掉包率試驗 trials 次，每次三台各建立一個任務（多源插單），任務放在別台身邊
    （建立者不是最適合的那台，要靠協商）。只看話題：
      收斂     每個任務恰好一台持有、各台持有的任務不同（current_task 只看得到路徑第一個，所以任務數 = 台數、
               各放在不同台身邊，最佳解是一台一個）；收斂時間從建立算起
      重複認領 同一個任務在這次試驗裡被兩台以上拿過（指派過又被換掉也算）
      5 s 收斂 收斂時間 ≤ 5 s 而且沒有重複認領（週計畫：30% 掉包 ≥ 95%）
      完成     持有者回報完成 → 全部撤下的時間；之後沒有再派給別台
      頻寬     每個掉包率從第一次試驗到最後一次，各台 mesh 網卡收送的平均 kB/s（tc 只限制送出方向，
               所以實際的單向掉包率就是設定值）
    每次試驗從「三台都沒有任務、再等 idle_gap 秒」開始（掉包時確認完成要 1～3 s）；收尾把這次的任務全部做完
    （含排在路徑後面的）。開始時不乾淨的試驗不計；收不了尾時停止這個掉包率（2026-10-12 第一次量測：
    只收拾看得到持有者的任務，排在後面的留下來變成執行中、沒人回報，之後每一次都被它卡住）
    """

    def __init__(self, env, run, only, levels, trials, delay, jitter, idle_gap=3.0):
        super().__init__(env, run, only)
        self.levels, self.trials, self.delay, self.jitter = levels, trials, delay, jitter
        self.idle_gap = idle_gap

    def busy(self):
        """目前拿著任務的載具"""
        none = self.cfg["fields"]["no_task_id"]
        return sorted(u for u, (t, _, _) in self.run.assigned.items() if t not in (None, none))

    def settle(self):
        """等到三台都沒有任務，再等 idle_gap 秒（掉包時全隊確認完成要 1～3 s，路徑裡可能還留著上一次的），
        之後還是都沒有任務才算乾淨；回傳是否乾淨"""
        if self.run.until(lambda: not self.busy(), 10.0) is None:
            return False
        self.run.sleep(self.idle_gap)
        return not self.busy()

    def close_all(self, tasks, timeout=30.0):
        """收尾：這次試驗的每個任務（含排在路徑後面、之後才輪到的）都等到有人拿到就回報完成，直到全部撤下。
        回傳（第一輪全部撤下的時刻或 None, 做不完的任務數）"""
        run = self.run
        reported = {}                         # 任務 → 回報過的持有者
        first_release = None
        deadline = time.time() + timeout
        while True:
            run.read()
            pending = 0
            for t in tasks:
                held = run.holders(t)
                for h in held:
                    if reported.get(t) != h:
                        self.report(h, t, True)
                        reported[t] = h
                if held or t not in reported:
                    pending += 1
            if pending == 0:
                if first_release is None:
                    first_release = time.time()
                run.sleep(0.5)                    # 撤下之後沒有再派出去才算做完
                if not any(run.holders(t) for t in tasks):
                    return first_release, 0
                continue
            if time.time() >= deadline:
                return first_release, pending
            run.sleep(0.1)

    def trial(self, level, k):
        run, ids = self.run, self.env.ids
        pos = self.positions()
        clean = self.settle()                    # 上一次的任務還沒全部結束時，這一次不算進統計
        t0 = time.time()
        tasks = []
        for i, creator in enumerate(ids):
            near = ids[(i + 1 + k) % len(ids)]          # 放在別台身邊，每次輪換
            x, y = pos[near]
            tid, _ = self.new_task(x + 2.0, y + 2.0, creator=creator)
            tasks.append(tid)

        def converged():
            held = [run.holders(t) for t in tasks]
            return all(len(h) == 1 for h in held) and len({h[0] for h in held}) == len(tasks)

        tc = run.until(converged, 10.0)
        conv = tc - t0 if tc is not None else None
        if tc is not None:
            run.sleep(1.0)                               # 收斂之後 1 s 不變才算
        holders = {t: run.holders(t) for t in tasks}
        dup = sum(len(run.ever_held(t, t0)) > 1 for t in tasks)

        t1 = time.time()
        released, leftover = self.close_all(tasks)
        reassigned = sum(bool(set(run.ever_held(t, t1)) - set(holders[t])) for t in tasks)
        row = dict(loss=level, trial=k + 1, clean=int(clean), conv_s=round(conv, 3) if conv is not None else "",
                   converged_5s=int(conv is not None and conv <= 5.0 and dup == 0), duplicates=dup,
                   release_s=round(released - t1, 3) if released else "", reassigned_after_done=reassigned,
                   leftover=leftover)
        mark = f"{conv:.2f} s" if conv is not None else "未收斂"
        print(f"    {level:>3}% #{k + 1:<3} 收斂 {mark}　重複 {dup}　撤下 " +
              (f"{released - t1:.2f} s" if released else "逾時") + (f"　完成後又派 {reassigned}" if reassigned else "") +
              ("" if clean else "　（開始時還有任務，不計）") + (f"　做不完 {leftover}" if leftover else ""))
        return row

    def run_all(self):
        env, run, res = self.env, self.run, self.res
        if not self.start_all():
            res.add("L0", "節點上線", "FAIL", "收不到各台的 assigned")
            return
        run.sleep(self.cfg["comm"]["warmup"])
        rows, levels = [], []
        for level in self.levels:
            self.tc(level, self.delay, self.jitter) if level > 0 else self.tc(None)
            try:
                run.sleep(2.0)
                b0 = {v: env.mesh_bytes(v) for v in env.ids}
                tb = time.time()
                lv = []
                for k in range(self.trials):
                    lv.append(self.trial(level, k))
                    if lv[-1]["leftover"]:            # 收不了尾：後面的試驗會被留下的任務干擾，這個掉包率停止
                        print(f"    {level}% 第 {k + 1} 次有 {lv[-1]['leftover']} 個任務做不完，停止這個掉包率")
                        break
                dt = time.time() - tb
                b1 = {v: env.mesh_bytes(v) for v in env.ids}
            finally:
                self.tc(None)
            rows += lv
            aborted = lv[-1]["leftover"] > 0
            lv = [r for r in lv if r["clean"] and not r["leftover"]]   # 開始時不乾淨、收不了尾的不計
            if not lv:
                res.add(f"L{level}", f"掉包 {level}%", "FAIL", "沒有可以統計的試驗（每次開始時都還有任務、或收不了尾）")
                break
            kbs = [(b1[v] - b0[v]) / dt / 1024 for v in env.ids if b0[v] is not None and b1[v] is not None]
            convs = sorted(r["conv_s"] for r in lv if r["conv_s"] != "")
            rels = sorted(r["release_s"] for r in lv if r["release_s"] != "")
            q = lambda xs, p: xs[min(len(xs) - 1, int(p * len(xs)))] if xs else None
            summary = dict(loss=level, trials=len(lv), rate_5s=sum(r["converged_5s"] for r in lv) / len(lv),
                           conv_median=q(convs, 0.5), conv_p90=q(convs, 0.9), not_converged=len(lv) - len(convs),
                           duplicates=sum(r["duplicates"] for r in lv), release_median=q(rels, 0.5),
                           reassigned=sum(r["reassigned_after_done"] for r in lv),
                           mesh_kBps=round(sum(kbs) / len(kbs), 2) if kbs else None)
            levels.append(summary)
            fmt = lambda v: f"{v:.2f} s" if v is not None else "—"
            ok = (level != 30 or summary["rate_5s"] >= 0.95) and not aborted
            res.add(f"L{level}", f"掉包 {level}%（延遲 {self.delay}±{self.jitter} ms）",
                    ("PASS" if ok else "FAIL") if level == 30 or aborted else "INFO",
                    f"5 s 內收斂 {summary['rate_5s'] * 100:.0f}%（{len(lv)} 次）、收斂中位數 {fmt(summary['conv_median'])}、"
                    f"P90 {fmt(summary['conv_p90'])}、未收斂 {summary['not_converged']}、重複認領 {summary['duplicates']}、"
                    f"完成→撤下 {fmt(summary['release_median'])}、完成後又派 {summary['reassigned']}、"
                    f"mesh {summary['mesh_kBps']} kB/s/台" + ("；**有任務做不完，提早停止**" if aborted else ""), **summary)
            if aborted:
                break
        for name, data in (("trials.csv", rows), ("levels.csv", levels)):
            with open(os.path.join(run.out, name), "w", encoding="utf-8") as f:
                f.write(",".join(data[0].keys()) + "\n")
                for r in data:
                    f.write(",".join("" if v is None else str(v) for v in r.values()) + "\n")
        print(f"  畫圖：python3 docker/itest/plot_loss.py {run.out}")


# -----------------------------------------------------------------------------
# UAV 整串（uav）
# -----------------------------------------------------------------------------
class UavSuite(Suite):
    def dist_xy(self, vid, x, y):
        p = self.run.positions.get(vid)
        return math.hypot(p[0] - x, p[1] - y) if p else float("inf")

    def results_of(self, tid, vid, since, success_only=False):
        f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]
        return [e for e in self.run.events if e["kind"] == "result" and e["uav"] == vid and e["t"] >= since
                and e["msg"].get(f_id) == tid and (not success_only or e["msg"].get(f_ok) is True)]

    def ground_tasks(self, since):
        f_type = self.cfg["fields"]["task_type"]
        return [e for e in self.run.events if e["kind"] == "new_task" and e["t"] >= since
                and e["msg"].get(f_type) == self.cfg["fields"]["ground_type"]]

    def do_task(self, x, y, reach_timeout=60.0):
        """建立任務 → 等指派 → 等飛到 → 等回報；回傳 (持有者, 到達時刻, 回報事件, 說明)"""
        dur = self.cfg["uav"]["task_duration"]
        tid, t0 = self.new_task(x, y, dur)
        h, _ = self.wait_holder(tid, 5.0)
        if h is None:
            return tid, None, None, None, "5 s 內沒有人拿到"
        ta = self.run.until(lambda: self.dist_xy(h, x, y) <= 1.5, reach_timeout)
        if ta is None:
            return tid, h, None, None, f"uav{h} 在 {reach_timeout:.0f} s 內沒飛到（剩 {self.dist_xy(h, x, y):.1f} m）"
        self.run.until(lambda: self.results_of(tid, h, t0), dur + 25.0)
        r = self.results_of(tid, h, t0)
        return tid, h, ta, (r[0] if r else None), ""

    def launch(self, headless, world=None, stack_env=None, on_world=None, together=False, takeoff_gate=None):
        """每台開 cbba_uav.sh，sim 開 PX4（第 1 台開場景）、相機；等全部起飛。回傳是否全部起飛。
        on_world：場景開好（第 1 台 PX4 起來）之後呼叫一次，例如開始錄影。
        together：場景開好就接著開其他台（間隔 2 s，原本每台固定等 12 s），畫面上三台一起出現。
        takeoff_gate：cbba_uav.sh 的 CBBA_TAKEOFF_GATE（BT 等這個檔案才啟動）；全部 PX4 都在送位置之後才建立，一起起飛"""
        env, run, res, u = self.env, self.run, self.res, self.cfg["uav"]
        ids = env.ids
        world = world or u.get("world")
        # 無人機的 BT：bt（備用）或 task_manager（照機器狗端骨架，2026-10-13）
        stack_env = {**(stack_env or {}), "EXECUTOR": env.executor}
        for vid in ids:
            env.exec(env.uav(vid), u["start"].format(id=vid, ns=env.ns(vid)), detach=True, env=stack_env,
                     log=f"{run.cout}/stack_uav{vid}.log")
        run.sleep(8.0)
        for i, vid in enumerate(ids):
            senv = {"HEADLESS": "1"} if headless else {}
            if i == 0 and world:
                senv["CBBA_WORLD"] = world
            env.exec(env.sim(), u["px4_start"].format(id=vid), detach=True, env=senv,
                     log=f"{run.cout}/px4_{vid}.log")
            if i == 0 and together and world:
                # 等場景開好（有 /world/<名稱>/clock）就接著開其他台
                for _ in range(60):
                    if env.exec(env.sim(), f"gz topic -l 2>/dev/null | grep -q '^/world/{world}/clock'").returncode == 0:
                        break
                    time.sleep(1.0)
                run.sleep(3.0)
            else:
                run.sleep(2.0 if together else (20.0 if i == 0 else 12.0))
            if i == 0 and on_world:
                on_world()
        if takeoff_gate:
            # 全部 PX4 都在送位置（開好、加入場景）才讓 BT 啟動、一起起飛
            ok = run.until(lambda: all(v in run.positions for v in ids), 90)
            print(f"  {'全部 PX4 上線' if ok else '90 s 內沒有全部上線'}，同時起飛")
            for vid in ids:
                env.exec(env.uav(vid), f"touch {takeoff_gate}")
        if u.get("camera_start"):
            env.exec(env.sim(), u["camera_start"], detach=True, log=f"{run.cout}/camera.log")

        alt = u["altitude"]
        t = run.until(lambda: all(run.armed.get(v) and run.positions.get(v, (0, 0, 0))[2] >= alt - 0.6 for v in ids), 150)
        up = [v for v in ids if run.armed.get(v) and run.positions.get(v, (0, 0, 0))[2] >= alt - 0.6]
        res.add("U1", "起飛到巡檢高度", "PASS" if t else "FAIL", f"已起飛：{up}" + ("" if t else f"（全部 {ids}）"))
        if t:
            run.sleep(5.0)    # 起飛後 CBBA 開始參與出價
        return t is not None

    def run_all(self):
        env, run, res, u = self.env, self.run, self.res, self.cfg["uav"]
        ids = env.ids
        if not self.launch(u.get("headless")):
            return

        if self.want("U2"):
            tid, h, ta, r, why = self.do_task(10.0, 5.0)
            ok = r is not None and r["msg"].get(self.cfg["fields"]["success"]) is True
            if ok:
                run.until(lambda: run.assigned.get(h, (None,))[0] != tid, 5)
            res.add("U2", "單一任務：指派 → 飛到 → 回報完成", "PASS" if ok else "FAIL",
                    why or (f"uav{h} 完成" if ok else f"uav{h} 到了但沒有回報成功"))

        if self.want("U3") and u.get("fire"):
            t0 = time.time()
            tid, h, ta, r, why = self.do_task(u["fire"]["x"], u["fire"]["y"])
            g = self.ground_tasks(t0)
            if g:
                p = g[0]["msg"].get("position", {})
                err = math.hypot(p.get("x", 0) - u["fire"]["x"], p.get("y", 0) - u["fire"]["y"])
                ok = err <= 1.5
                res.add("U3", "火警：到達後建立地面處置任務", "PASS" if ok else "FAIL",
                        f"uav{g[0]['uav']} 建立，到達後 {g[0]['t'] - ta:.2f} s，位置誤差 {err:.2f} m" if ta else
                        f"uav{g[0]['uav']} 建立，位置誤差 {err:.2f} m")
            else:
                res.add("U3", "火警：到達後建立地面處置任務", "FAIL", why or "沒有地面處置任務（相機有開嗎？）")

        if self.want("U4") and u.get("decoy"):
            t0 = time.time()
            tid, h, ta, r, why = self.do_task(u["decoy"]["x"], u["decoy"]["y"])
            run.sleep(3.0)
            g = self.ground_tasks(t0)
            ok = ta is not None and not g
            res.add("U4", "誘餌：看得到但不觸發火警", "PASS" if ok else "FAIL",
                    why or ("沒有建立地面任務" if ok else f"建立了 {len(g)} 個地面任務"))

        if self.want("U5"):
            pts = [(20.0, 10.0), (-15.0, 12.0), (5.0, -18.0)][:len(ids)]
            t0 = time.time()
            tids = [self.new_task(x, y, u["task_duration"])[0] for x, y in pts]
            min_d = [float("inf")]

            def all_done():
                for a in ids:
                    for b in ids:
                        if a < b and a in run.positions and b in run.positions:
                            pa, pb = run.positions[a], run.positions[b]
                            min_d[0] = min(min_d[0], math.dist(pa[:3], pb[:3]))
                f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]
                return all(any(e["kind"] == "result" and e["msg"].get(f_id) == t and e["msg"].get(f_ok) is True
                               and e["t"] >= t0 for e in run.events) for t in tids)
            done = run.until(all_done, 120.0, step=0.2)
            who = [run.ever_held(t, t0) for t in tids]
            distinct = len({w[0] for w in who if w}) == len(tids)
            ok = done is not None and min_d[0] >= 1.5
            res.add("U5", "多台分配與避碰", "PASS" if ok else "FAIL",
                    f"{'全部完成' if done else '120 s 內沒全部完成'}；持有者 {who}" +
                    ("" if distinct else "（沒有分給不同台）") + f"；機間最近 {min_d[0]:.2f} m", min_distance=min_d[0])

        if self.want("U6"):
            tid, t0 = self.new_task(18.0, -20.0, u["task_duration"])
            h, _ = self.wait_holder(tid, 5.0)
            if h is None:
                res.add("U6", "飛行中 PX4 斷線 → 別台接手完成", "FAIL", "沒有人拿到任務")
            else:
                run.sleep(2.0)
                t1 = time.time()
                env.pkill(env.sim(), u["px4_stop_pattern"].format(id=h))
                h2, t2 = self.wait_holder(tid, 15.0, exclude=(h,))
                ok = False
                if h2:
                    run.until(lambda: self.results_of(tid, h2, t1, True), 90.0)
                    ok = bool(self.results_of(tid, h2, t1, True))
                res.add("U6", "飛行中 PX4 斷線 → 別台接手完成", "PASS" if ok else "FAIL",
                        (f"uav{h} 斷線 → uav{h2}，{t2 - t1:.1f} s" + ("，完成" if ok else "，但沒完成")) if h2 else
                        f"uav{h} 斷線後 15 s 沒人接")


# -----------------------------------------------------------------------------
# 展示（demo）：Gazebo 畫面（錄影）＋俯視動畫（MP4）
# -----------------------------------------------------------------------------
class DemoSuite(UavSuite):
    """demo_site（展示用的小場景）開 Gazebo 畫面（地面網格、沒有陰影，docker/gz/gui/cbba_gui.config）。
    流程（fire_mode point，週計畫，2026-10-13）：
      1. 三台一起載入、一起起飛；都到巡檢高度後才同時啟動 cbba_node 和假狗（CBBA_TAKEOFF_GATE、CBBA_START_GATE）
      2. 依 --seed 挑一個 R 點起火（紅色方塊＋tag 101），和 G1～G3 一起建立巡檢任務，CBBA 分給三台
      3. 飛到 R 點的那台懸停時看到 tag 101 → 發布地面處置任務（GROUND_INTERVENTION）→ 假狗得標、走過去、處理、回報
      4. 那台懸停監控到地面任務結束（task_closed）才完成 R 點；全部做完後各自飛回起飛點上空
    fire_mode route（2026-10-12）：出發後在某台的航線上起火，看飛行途中發現、發布地面任務、繼續飛。
    fire_mode side（2026-10-14）：一開始就在 side_points（CSV 的點，例如 G3 航線旁的 R1）起火，不建成任務；
      飛過的那台看到 → 發布地面任務 → 轉向火點上空懸停監控 → 假狗處理完 → 回航線、完成原本的巡檢點（D4 檢查這個順序）。
    Gazebo 畫面：場景裡的俯瞰攝影機（overview_camera）由伺服器錄成 gazebo.mp4；全部完成後停 hold 秒，
    再把 events.jsonl＋demo_plan.json 畫成俯視動畫（animate_demo.py，sim 容器畫每一格、主機的 ffmpeg 合成 demo.mp4）"""

    def __init__(self, env, run, only, hold, seed, speed, record, fire_mode=None, scenario=None,
                 interactive=False, area=None, with_patrol=True):
        super().__init__(env, run, only)
        self.hold, self.seed, self.speed, self.record = hold, seed, speed, record
        self.fire_mode = fire_mode
        self.scenario = scenario      # --scenario 的內容（dict）；有的話 fire_mode = custom
        self.interactive = interactive    # --interactive：terminal 手動插入（也是 custom）
        self.with_patrol = with_patrol    # --interactive 時先建立 demo.patrol（G1～G3）；--no-patrol 不建
        d = self.cfg["demo"]
        self.area = area or d.get("area", [-22.0, 24.0, -14.0, 11.0])   # 手動、隨機插入的範圍 [x0, x1, y0, y1]
        self.min_gap = d.get("min_gap", 3.0)
        self.rng = random.Random(seed)
        self.console_done = not interactive
        self.n_task = self.n_fire = 0

    def points(self):
        path = os.path.join(self.env.host_repo, "docker", "gz", "worlds", f"{self.cfg['demo']['world']}_points.csv")
        pts = {}
        with open(path, encoding="utf-8") as f:
            rows = [l.strip() for l in f if l.strip() and not l.startswith("#")]
        for line in rows[1:]:
            name, x, y, tag = line.split(",")
            pts[name] = (float(x), float(y), int(tag))
        return pts

    def gz_record(self, start, path=""):
        """場景裡的俯瞰攝影機（overview_camera，CameraVideoRecorder 系統）錄影，Gazebo 伺服器自己錄、不用開視窗。
        2026-10-13 前用視窗的 VideoRecorder 按鈕（/gui/record_video）：gz-sim 8＋MinimalScene 沒有人提供這個服務，一直錄不到"""
        svc = f"/{self.cfg['demo']['world']}/overview/record_video"
        req = ('start: true, format: "mp4", save_filename: "%s"' % path) if start else 'stop: true'
        r = self.env.exec(self.env.sim(), f"gz service -s {svc} --reqtype gz.msgs.VideoRecord "
                          f"--reptype gz.msgs.Boolean --timeout 5000 --req '{req}'")
        return "data: true" in (r.stdout or "")

    # ------------------------------------------------------------------ 自訂情境（--scenario，2026-10-14）
    def scenario_items(self, sc, pts):
        """情境檔的 tasks、fires → 待插入的項目（座標用 point 查 CSV，或直接給 x、y）"""
        items = []
        for kind in ("task", "fire"):
            for i, it in enumerate(sc.get(kind + "s", [])):
                if "point" in it:
                    if it["point"] not in pts:
                        raise SystemExit(f"情境檔：{kind} 的 point {it['point']} 不在 {self.cfg['demo']['world']}_points.csv")
                    x, y, _ = pts[it["point"]]
                else:
                    x, y = float(it["x"]), float(it["y"])
                name = it.get("name") or it.get("point") or f"{kind[0].upper()}{i + 1}"
                items.append(dict(kind=kind, name=name, x=x, y=y, known_point="point" in it,
                                  duration=float(it.get("duration", 5.0)),
                                  at=float(it.get("at", 0.0)), after_arrival=it.get("after_arrival"),
                                  after_done=it.get("after_done"), delay=float(it.get("delay", 0.0)),
                                  creator=it.get("creator")))
        names = [it["name"] for it in items if it["kind"] == "task"]
        for it in items:
            for k in ("after_arrival", "after_done"):
                if it[k] and it[k] not in names:
                    raise SystemExit(f"情境檔：{it['name']} 的 {k} = {it[k]}，沒有這個任務（任務：{', '.join(names)}）")
        return items

    def insert_due(self, items, plan, t0):
        """條件達成的項目插入：at（開始後幾秒）、after_arrival（那個任務的執行者到點、1 m 內）、
        after_done（那個任務回報完成），再等 delay 秒"""
        run = self.run
        f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]
        now = time.time()
        by_name = {p["point"]: p for p in plan["patrol"]}
        for it in items:
            if it.get("done"):
                continue
            if "ready_t" not in it:
                ref = it["after_arrival"] or it["after_done"]
                p = by_name.get(ref) if ref else None
                if ref and not p:
                    continue
                if it["after_arrival"]:
                    if not any(v in run.positions and math.hypot(run.positions[v][0] - p["x"],
                                                                 run.positions[v][1] - p["y"]) < 1.0
                               for v in run.holders(p["task_id"])):
                        continue
                elif it["after_done"]:
                    if not any(e["kind"] == "result" and e["msg"].get(f_id) == p["task_id"]
                               and e["msg"].get(f_ok) is True for e in run.events):
                        continue
                elif now - t0 < it["at"]:
                    continue
                it["ready_t"] = now
            if now - it["ready_t"] < it["delay"]:
                continue
            self.insert_now(it, plan, t0)

    def insert_now(self, it, plan, t0):
        """插入一個任務或起火點。任務點不是場景裡的點（自訂 x、y）時生成藍色平台（models/task_pad）"""
        env, d = self.env, self.cfg["demo"]
        now = time.time()
        it["done"] = True
        scen = {"CBBA_SCENARIO": d["world"]}
        if it["kind"] == "task":
            if not it.get("known_point"):
                env.exec(env.sim(), f"patrol_scenario.sh mark-at {it['x']} {it['y']} {it['name']}", env=scen)
            creator = it["creator"] or env.ids[len(plan["patrol"]) % len(env.ids)]
            tid, _ = self.new_task(it["x"], it["y"], it["duration"], creator=creator)
            plan["patrol"].append(dict(point=it["name"], task_id=tid, x=it["x"], y=it["y"], t=now,
                                       duration=it["duration"]))
            self.reply(f"  {now - t0:5.1f} s  插入任務 {it['name']} ({it['x']:g}, {it['y']:g})、停留 {it['duration']:g} s")
        else:
            env.exec(env.sim(), f"patrol_scenario.sh fire-at {it['x']} {it['y']} {it['name']}", env=scen)
            plan["fires"].append(dict(name=it["name"], x=it["x"], y=it["y"], custom=True, spawned=now))
            self.reply(f"  {now - t0:5.1f} s  起火 {it['name']} ({it['x']:g}, {it['y']:g})")

    # ------------------------------------------------------------------ 手動插入（--interactive，2026-10-14）
    CONSOLE_HELP = (
        "  手動插入（另開一個 terminal：python3 docker/itest/insert.py；或直接在這裡輸入後按 Enter）：\n"
        "    t [x y] [d=秒]    任務點（藍色平台）；不給 x y = 範圍內隨機\n"
        "    t G2 [d=秒]       用場景的點名\n"
        "    tf [x y] [d=秒]   任務點、而且那裡起火（飛到時看到火 → 發布地面任務 → 監控到狗處理完）\n"
        "    f [x y]           只起火、不建任務（無人機路過或到附近時發現）\n"
        "    l 列出  a 範圍  h 說明  q 不再插入（做完就結束）")

    # 第二個 terminal（insert.py）和 itest 之間的檔案：insert.py 把指令一行一行加到 commands.txt，
    # itest 讀新的行、處理，回覆寫到 replies.txt（insert.py 讀回來印）；session.json 表示有一個 --interactive 在跑
    IA_DIR = os.path.join(HERE, "out", "interactive")

    def ia_open(self):
        os.makedirs(self.IA_DIR, exist_ok=True)
        for name in ("commands.txt", "replies.txt"):
            open(os.path.join(self.IA_DIR, name), "w", encoding="utf-8").close()
        self.ia_pos = 0
        self.ia_session(ready=False)

    def ia_session(self, ready):
        with open(os.path.join(self.IA_DIR, "session.json"), "w", encoding="utf-8") as f:
            json.dump(dict(pid=os.getpid(), out=self.run.out, area=self.area, ready=ready,
                           world=self.cfg["demo"]["world"]), f, ensure_ascii=False)

    def ia_close(self):
        try:
            os.remove(os.path.join(self.IA_DIR, "session.json"))
        except OSError:
            pass

    def reply(self, text):
        """回覆：印在這個 terminal，--interactive 時也寫給第二個 terminal（insert.py）"""
        print(text)
        if self.interactive:
            with open(os.path.join(self.IA_DIR, "replies.txt"), "a", encoding="utf-8") as f:
                f.write(text + "\n")

    def console(self, items, plan, t0, pts):
        """非阻塞地讀指令（第二個 terminal 的 insert.py 寫的 commands.txt，或這個 terminal 的輸入），插入任務或起火點"""
        if self.console_done:
            return
        lines = []
        path = os.path.join(self.IA_DIR, "commands.txt")
        try:
            with open(path, encoding="utf-8") as f:
                f.seek(self.ia_pos)
                chunk = f.read()
            done = chunk.rfind("\n") + 1           # 只處理完整的行
            self.ia_pos += len(chunk[:done].encode("utf-8"))
            lines += chunk[:done].splitlines()
        except OSError:
            pass
        if select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line:
                lines.append(line)
        for line in lines:
            if not self.console_done:
                self.handle_cmd(line, items, plan, t0, pts)

    def handle_cmd(self, line, items, plan, t0, pts):
        words = line.split()
        if not words:
            return
        self.reply(f"> {line.strip()}")
        cmd, args = words[0].lower(), words[1:]
        dur = self.cfg["demo"]["duration"]
        rest = []
        for a in args:
            if a.startswith("d="):
                try:
                    dur = float(a[2:])
                except ValueError:
                    self.reply(f"  停留秒數不是數字：{a}")
                    return
            else:
                rest.append(a)
        if cmd == "q":
            self.console_done = True
            self.reply("  不再插入；目前的任務做完就結束")
        elif cmd == "h":
            self.reply(self.CONSOLE_HELP)
        elif cmd == "a":
            x0, x1, y0, y1 = self.area
            self.reply(f"  範圍：x {x0:g}～{x1:g}、y {y0:g}～{y1:g}（隨機點離其他點 ≥ {self.min_gap:g} m）")
        elif cmd == "l":
            f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]
            for p in plan["patrol"]:
                ok = any(e["kind"] == "result" and e["msg"].get(f_id) == p["task_id"] and e["msg"].get(f_ok) is True
                         for e in self.run.events)
                who = self.run.holders(p["task_id"])
                self.reply(f"  {p['point']:>4} ({p['x']:6.1f}, {p['y']:6.1f})  " +
                      ("完成" if ok else f"uav{who[0]} 執行中" if who else "等待分配"))
            for f in plan["fires"]:
                self.reply(f"  {f['name']:>4} ({f['x']:6.1f}, {f['y']:6.1f})  起火")
        elif cmd in ("t", "tf", "f"):
            xy = self.console_xy(rest, plan, pts)
            if xy is None:
                return
            x, y, known = xy
            if cmd in ("t", "tf"):
                f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]
                gap = self.min_gap
                busy = [p for p in plan["patrol"] if math.hypot(p["x"] - x, p["y"] - y) < gap and not any(
                    e["kind"] == "result" and e["msg"].get(f_id) == p["task_id"] and e["msg"].get(f_ok) is True
                    for e in self.run.events)]
                if busy:
                    self.reply(f"  ({x:g}, {y:g}) {gap:g} m 內已經有還沒做完的任務 {busy[0]['point']}，不再插入"
                               f"（同一個位置兩個任務，CBBA 會當成兩件事分配）")
                    return
            if cmd in ("tf", "f"):
                self.n_fire += 1
                self.insert_now(dict(kind="fire", name=f"F{self.n_fire}", x=x, y=y), plan, t0)
            if cmd in ("t", "tf"):
                self.n_task += 1
                name = known or f"T{self.n_task}"
                self.insert_now(dict(kind="task", name=name, x=x, y=y, known_point=bool(known), duration=dur,
                                     creator=None), plan, t0)
            items.append(dict(kind="manual", name=line.strip(), done=True))
        else:
            self.reply(f"  不認得：{line.strip()}（h 看說明）")

    def console_xy(self, rest, plan, pts):
        """指令的位置：點名、x y、或沒給（範圍內隨機）。回傳 (x, y, 點名或 None)；不合法回 None"""
        x0, x1, y0, y1 = self.area
        if len(rest) == 1:
            name = rest[0].upper()
            if name not in pts:
                self.reply(f"  沒有 {rest[0]} 這個點（{', '.join(sorted(pts))}）")
                return None
            return pts[name][0], pts[name][1], name
        if len(rest) == 2:
            try:
                x, y = float(rest[0]), float(rest[1])
            except ValueError:
                self.reply(f"  座標不是數字：{' '.join(rest)}")
                return None
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                self.reply(f"  ({x:g}, {y:g}) 在範圍外：x {x0:g}～{x1:g}、y {y0:g}～{y1:g}")
                return None
            return x, y, None
        if rest:
            self.reply("  位置要嘛給點名、要嘛給 x y、要嘛不給（隨機）")
            return None
        # 隨機：離場景的點、已插入的點、起飛點都 ≥ min_gap
        spacing = self.cfg["vehicles"]["spawn_spacing"]
        taken = [(p[0], p[1]) for p in pts.values()] + [(p["x"], p["y"]) for p in plan["patrol"]] + \
            [(f["x"], f["y"]) for f in plan["fires"]] + [(0.0, (v - 1) * spacing) for v in self.env.ids]
        for _ in range(500):
            x, y = round(self.rng.uniform(x0, x1), 1), round(self.rng.uniform(y0, y1), 1)
            if all(math.hypot(x - a, y - b) >= self.min_gap for a, b in taken):
                return x, y, None
        self.reply(f"  範圍內找不到離其他點 ≥ {self.min_gap:g} m 的位置")
        return None

    # ------------------------------------------------------------------ CBF（D7，2026-10-14）
    def safe_radius(self, exec_args):
        """cbf.safe_radius：EXEC_ARGS 有給就用它，否則讀 uav_bt.yaml"""
        m = re.search(r"cbf\.safe_radius:=([0-9.]+)", exec_args or "")
        if m:
            return float(m.group(1))
        path = os.path.join(self.env.host_repo, "ros2_ws", "src", "uav_px4_bt", "config", "uav_bt.yaml")
        try:
            with open(path, encoding="utf-8") as f:
                for line in f:
                    m = re.match(r"\s*cbf\.safe_radius:\s*([0-9.]+)", line)
                    if m:
                        return float(m.group(1))
        except OSError:
            pass
        return 1.2

    def check_cbf(self, t_from, t_to, plan):
        """D7：任務開始到結束（含回起飛點）在空中的兩台之間的最近水平距離，和 BT 印的每次接近（「避碰：這次最接近鄰機」）"""
        run, res = self.run, self.res
        safe = self.safe_radius(plan.get("exec_args"))
        alt = self.cfg["uav"]["altitude"]
        last, best = {}, None
        for e in run.events:
            if e["kind"] != "pos" or not t_from <= e["t"] <= t_to or e["z"] < alt - 1.5:
                continue
            last[e["uav"]] = (e["x"], e["y"], e["t"])
            for o, (x, y, t) in last.items():
                if o != e["uav"] and abs(t - e["t"]) < 0.3:
                    dd = math.hypot(x - e["x"], y - e["y"])
                    if best is None or dd < best[0]:
                        best = (dd, e["t"] - t_from, min(o, e["uav"]), max(o, e["uav"]))
        close = []
        for v in self.env.ids:
            try:
                with open(os.path.join(run.out, f"stack_uav{v}.log"), encoding="utf-8", errors="replace") as f:
                    close += [(v, float(m.group(1))) for m in re.finditer(r"避碰：這次最接近鄰機 ([0-9.]+) m", f.read())]
            except OSError:
                pass
        plan["cbf"] = dict(safe_radius=safe, min_distance=best[0] if best else None, encounters=close)
        if best is None:
            res.add("D7", "CBF 避碰：空中兩台的最近水平距離", "INFO", "沒有兩台同時在空中的資料")
            return
        ok = best[0] >= 0.9 * safe
        res.add("D7", "CBF 避碰：空中兩台的最近水平距離", "PASS" if ok else "FAIL",
                f"最近 {best[0]:.2f} m（uav{best[2]}–uav{best[3]}，開始後 {best[1]:.1f} s；safe_radius {safe:g} m，"
                f"門檻 0.9 × safe_radius）；BT 記到的接近 {len(close)} 次" +
                (f"：{'、'.join(f'uav{v} {x:.2f} m' for v, x in close)}" if close else
                 "（BT 只在鄰機進到 1.5 × safe_radius 內才記；CBF 在更遠處就繞開時這裡是 0，要看影片的路徑）"))

    def check_side_divert(self, f, tc, plan):
        """D4（side）：發現的那台轉向到火點上空（1 m 內）、懸停監控到地面任務結束（離火點 < 2 m），
        之後回航線、完成發現時手上的那個巡檢任務"""
        run, res = self.run, self.res
        f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]
        v, t_seen = f.get("reported_by"), f.get("reported_t")
        if not v or not tc:
            res.add("D4", "路過發現 → 轉向火點上空監控 → 狗處理完 → 回航線完成原任務", "FAIL",
                    "沒有發現火點或地面任務沒有結束")
            return
        ps = [e for e in run.events if e["kind"] == "pos" and e["uav"] == v and t_seen <= e["t"] <= tc]
        arrive = next((e["t"] for e in ps if math.hypot(e["x"] - f["x"], e["y"] - f["y"]) < 1.0), None)
        drift = max((math.hypot(e["x"] - f["x"], e["y"] - f["y"]) for e in ps if arrive and e["t"] >= arrive),
                    default=None)
        held = [tid for t, u, tid, _ in run.history if u == v and t <= t_seen and tid]
        patrol = {p["task_id"]: p["point"] for p in plan["patrol"]}
        task = held[-1] if held and held[-1] in patrol else None
        tr = [e["t"] for e in run.events if task and e["kind"] == "result" and e["msg"].get(f_id) == task
              and e["msg"].get(f_ok) is True]
        ok = bool(arrive and drift is not None and drift < 2.0 and tr and tr[0] >= tc - 0.5)
        detail = (f"uav{v} 發現後 {arrive - t_seen:.1f} s 到火點上空、監控期間離火點最遠 {drift:.2f} m" if arrive
                  else f"uav{v} 沒有飛到火點上空（1 m 內）")
        detail += (f"；狗處理完後 {tr[0] - tc:.1f} s 完成原本的 {patrol[task]}" if tr else
                   f"；沒有看到完成原本的巡檢任務（{patrol.get(task, '不知道是哪個')}）")
        res.add("D4", "路過發現 → 轉向火點上空監控 → 狗處理完 → 回航線完成原任務", "PASS" if ok else "FAIL", detail)

    def run_all(self):
        env, run, res = self.env, self.run, self.res
        d = self.cfg["demo"]
        dog = self.cfg.get("dog")
        mode = "custom" if self.scenario or self.interactive else (self.fire_mode or d.get("fire_mode", "point"))
        sc = self.scenario or {}
        ids = env.ids
        rng = random.Random(self.seed)
        pts = self.points()
        spacing = self.cfg["vehicles"]["spawn_spacing"]
        alt = self.cfg["uav"]["altitude"]
        plan = dict(seed=self.seed, world=d["world"], speed=self.speed, mode=mode,
                    points={k: dict(x=v[0], y=v[1], tag=v[2]) for k, v in pts.items()
                            if k in d["patrol"] or k in d.get("fire_points", []) or k in d.get("side_points", [])},
                    patrol=[], fires=[dict(name=f"F{i + 1}") for i in range(d["fire_count"])], dog=None)
        if mode == "custom":
            plan["fires"] = []
            plan["scenario"] = sc
            if self.scenario:
                print(f"  自訂情境（{sc.get('_file', '--scenario')}）：{len(sc.get('tasks', []))} 個任務、"
                      f"{len(sc.get('fires', []))} 個起火點")
            if self.interactive:
                self.ia_open()
                print(f"  手動插入：另開一個 terminal 執行 python3 docker/itest/insert.py，CBBA 上線後開始接受指令"
                      f"（範圍 x {self.area[0]:g}～{self.area[1]:g}、y {self.area[2]:g}～{self.area[3]:g}）")
        elif mode == "side":
            plan["fires"] = [dict(name=n) for n in d.get("side_points", [])]
            print(f"  航線旁起火（{', '.join(d.get('side_points', []))}）：路過發現 → 轉向火點上空監控 → 狗處理完 → 回航線")
        elif mode == "point":
            print(f"  種子 {self.seed}：週計畫流程（火點在某個巡檢點上；無人機懸停監控到狗處理完、全部回起飛點）")
        else:
            print(f"  種子 {self.seed}：出發後在 {d['fire_count']} 台的航線上起火")
        # 錄影從場景開好就開始（影片看得到起飛）
        gz_video = f"{run.cout}/gazebo.mp4"
        rec = {"on": False}

        def start_recording():
            if not self.record:
                return
            for _ in range(10):
                rec["on"] = self.gz_record(True, gz_video)
                if rec["on"]:
                    rec["t0"] = time.time()   # 下視相機的畫面照這個時間和 gazebo.mp4 對齊（demo_cameras.mp4）
                    print("  Gazebo 開始錄影（俯瞰攝影機）")
                    return
                time.sleep(2.0)
            env.exec(env.sim(), "gz service -l", log=f"{run.cout}/gz_services.log")
            print(f"  Gazebo 錄影沒有開始（俯瞰攝影機的服務沒回應；服務清單在 {run.out}/gz_services.log）；照樣繼續")
        # CBBA 等全隊都起飛到同一個高度才開始：cbba_uav.sh 的 cbba_node 等 gate 檔案出現才啟動（BT、PX4 照常起飛）
        gate, takeoff = "/tmp/cbba_demo_go", "/tmp/cbba_demo_takeoff"
        for vid in ids:
            env.exec(env.uav(vid), f"rm -f {gate} {takeoff}")
        # 展示用的 BT 參數（config 的 demo.exec_args＋情境檔的 exec_args，例如速度 3 m/s 時 CBF 的 influence_radius）
        exec_args = " ".join(a for a in (d.get("exec_args", ""), sc.get("exec_args", "")) if a)
        stack_env = {"CBBA_SPEED": str(self.speed), "CBBA_START_GATE": gate, "CBBA_TAKEOFF_GATE": takeoff}
        if exec_args:
            stack_env["EXEC_ARGS"] = exec_args
            print(f"  BT 參數：{exec_args}")
        plan["exec_args"] = exec_args
        if not self.launch(headless=False, world=d["world"], stack_env=stack_env,
                           on_world=start_recording, together=True, takeoff_gate=takeoff):
            return
        # 各台的下視相機（掃 AprilTag 的那台）也錄下來，最後和 gazebo.mp4 並排成 demo_cameras.mp4（2026-10-19）
        if self.record:
            env.exec(env.sim(), f"exec python3 {env.repo}/docker/itest/record_cameras.py {run.cout} "
                     f"--ids {' '.join(str(v) for v in ids)}", detach=True, log=f"{run.cout}/record_cameras.log")
        # 假狗：Gazebo 替身＋執行者在 sim、cbba_node 在 dog.container（要在 mesh 上），和無人機的 cbba_node 同時開始
        if dog and (mode in ("point", "side") or (mode == "custom" and (sc.get("fires") or self.interactive))):
            # 2026-10-20 起可以多隻（dog.units，分散在場地各處，看狗之間對火警的競標）；紀錄檔 fake_dog_<ns>.log、dog_cbba_<ns>.log
            units = dog.get("units") or [dict(ns="v60", id=50, start_xy=dog["start_xy"])]
            plan["dog"] = []
            for u in units:
                dx, dy = u["start_xy"]
                env.exec(env.sim(), dog["sim_start"].format(repo=env.repo, world=d["world"], x=dx, y=dy, ns=u["ns"],
                                                            id=u["id"], speed=dog["speed"]),
                         detach=True, log=f"{run.cout}/fake_dog_{u['ns']}.log")
                env.exec(dog["container"], dog["start"].format(ns=u["ns"], id=u["id"], speed=dog["speed"]), detach=True,
                         log=f"{run.cout}/dog_cbba_{u['ns']}.log")
                plan["dog"].append(dict(ns=u["ns"], id=u["id"], x=dx, y=dy, speed=dog["speed"]))
            print(f"  假狗 {len(units)} 隻：" + "、".join(f"{u['ns']}（{u['id']}）在 ({u['start_xy'][0]:g}, {u['start_xy'][1]:g})"
                                                     for u in units))
        print("  全部起飛到巡檢高度；懸停 3 s 後同時啟動三台的 cbba_node" + ("和假狗" if plan["dog"] else ""))
        run.sleep(3.0)
        for vid in ids:
            env.exec(env.uav(vid), f"touch {gate}")
        up = run.until(lambda: all(v in run.assigned for v in ids), 20.0)
        print(f"  CBBA {'上線' if up else '20 s 內沒有全部上線'}；等 {d['start_delay']:.0f} s（互相連上）後建立固定任務")
        run.sleep(d["start_delay"])
        recording = rec["on"]

        t0 = time.time()
        plan["t0"] = t0
        names = list(d["patrol"])
        if mode == "point":
            # 週計畫：火點在某個巡檢點上（紅色方塊＋tag 101），那個點也是巡檢任務；無人機飛到、懸停時看到
            f = plan["fires"][0]
            name = rng.choice(d["fire_points"])
            x, y, _ = pts[name]
            env.exec(env.sim(), f"patrol_scenario.sh fire-at {x} {y} {name}", env={"CBBA_SCENARIO": d["world"]})
            f.update(x=x, y=y, point=name, spawned=time.time())
            plan["fires"] = [f]
            names.append(name)
            print(f"  巡檢點 {name} ({x}, {y}) 起火；巡檢任務 {', '.join(names)}")
        elif mode == "side":
            # 航線旁的點起火（不建成任務）：建立巡檢任務之前就點好，飛過的那台路上看得到
            for f in plan["fires"]:
                x, y, _ = pts[f["name"]]
                env.exec(env.sim(), f"patrol_scenario.sh fire-at {x} {y} {f['name']}", env={"CBBA_SCENARIO": d["world"]})
                f.update(x=x, y=y, side=f["name"], spawned=time.time())
            where = ", ".join("%s (%g, %g)" % (f["name"], f["x"], f["y"]) for f in plan["fires"])
            print(f"  航線旁起火：{where}；巡檢任務 {', '.join(names)}")
        if mode == "custom":
            # 手動插入（沒有情境檔）時，原本的巡檢任務（demo.patrol：G1～G3）照樣一開始就建立、要做完，
            # 插入的點是加在它們之上（2026-10-15）；--no-patrol 不建；有情境檔時照情境檔
            names = list(d["patrol"]) if self.interactive and not self.scenario and self.with_patrol else []
            if names:
                print(f"  原本的巡檢任務：{', '.join(names)}（停留 " +
                      ", ".join(f"{n} {d.get('durations', {}).get(n, d['duration']):g} s" for n in names) + "）")
            items = self.scenario_items(sc, pts)     # 情境檔的任務、起火點（依條件插入）
        for i, name in enumerate(names):
            x, y, _ = pts[name]
            # 每個點的停留時間（config 的 demo.durations，沒有就 demo.duration）：G1 停久一點，CBBA 不會把 G2 排在同一台後面
            dur = d.get("durations", {}).get(name, d["duration"])
            tid, _ = self.new_task(x, y, dur, creator=ids[i % len(ids)])
            plan["patrol"].append(dict(point=name, task_id=tid, x=x, y=y, t=time.time(), duration=dur))
        f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]

        def done(t):
            return any(e["kind"] == "result" and e["msg"].get(f_id) == t and e["msg"].get(f_ok) is True
                       and e["t"] >= t0 for e in run.events)

        def closed_at(t):
            """地面任務結束（cbba_node 的 task_closed，status DONE）的最早時刻；沒有 → None"""
            ts = [e["t"] for e in run.events if e["kind"] == "closed" and e["msg"].get(f_id) == t
                  and e["msg"].get("status") == 1]
            return min(ts) if ts else None

        departed = {}     # 固定任務 → 出發時那台的位置

        def ahead(p):
            """（航線模式）這個固定任務的執行者已經出發（飛了 fire_after 以上）、離目的地還有 fire_min_ahead 以上時，
            回傳（執行者, 目前位置, 剩下的距離），否則 None"""
            h = run.holders(p["task_id"])
            if not h or h[0] not in run.positions or h[0] in used:
                return None
            v = h[0]
            if p["task_id"] not in departed:
                departed[p["task_id"]] = run.positions[v][:2]
            sx, sy = departed[p["task_id"]]
            total = math.hypot(p["x"] - sx, p["y"] - sy)
            px, py = run.positions[v][:2]
            left = math.hypot(p["x"] - px, p["y"] - py)
            if total < 1.0 or (total - left) / total < d["fire_after"] or left < d["fire_min_ahead"]:
                return None
            return v, (px, py), left

        pending = [f for f in plan["fires"] if "spawned" not in f]
        used = set()
        deadline = t0 + d["timeout"]
        finished = None
        settled_since = None
        if mode == "custom":
            deadline = t0 + sc.get("timeout", d.get("interactive_timeout", 600.0) if self.interactive else d["timeout"])
            if self.interactive:
                self.ia_session(ready=True)
                self.reply(f"  CBBA 上線，開始接受指令（最長 {deadline - t0:.0f} s；模擬電池約 12 分鐘從 100% 降到 20%、"
                           f"之後無人機會返航，長時間測試要注意）")
                print(self.CONSOLE_HELP)
        while time.time() < deadline:
            run.read()
            if mode == "custom":
                self.insert_due(items, plan, t0)
                self.console(items, plan, t0, pts)
            cands = [(p, a) for p in plan["patrol"] for a in [ahead(p)] if a] if pending else []
            if cands:
                p, (v, (px, py), dist) = rng.choice(cands)
                dx, dy = p["x"] - px, p["y"] - py
                f = pending.pop(0)
                k = rng.uniform(*d["fire_along"])
                off = rng.uniform(-d["fire_offset"], d["fire_offset"])
                x = round(px + dx * k - dy / dist * off, 2)
                y = round(py + dy * k + dx / dist * off, 2)
                env.exec(env.sim(), f"patrol_scenario.sh fire-at {x} {y} {f['name']}",
                         env={"CBBA_SCENARIO": d["world"]})
                f.update(x=x, y=y, on_route_of=v, route_to=p["point"], spawned=time.time())
                used.add(v)
                print(f"  {f['spawned'] - t0:5.1f} s  {f['name']} 在 uav{v}（往 {p['point']}）的航線上起火：({x}, {y})")
            g = self.ground_tasks(t0)
            all_done = all(done(p["task_id"]) for p in plan["patrol"])
            dog_done = not plan["dog"] or all(closed_at(e["msg"][f_id]) for e in g)
            waiting = mode == "custom" and (any(not it.get("done") for it in items) or not self.console_done)
            # 自訂情境、手動插入（2026-10-15）：任務都做完、狗也處理完之後，還沒被發現的火再等 fire_grace 秒；
            # 還是沒人發現（不在任何航線、懸停點附近）就照常結束，D2 標「沒有發現」，不卡到 timeout
            settled = not pending and not waiting and all_done and dog_done
            if not settled:
                settled_since = None
            elif settled_since is None:
                settled_since = time.time()
            fires_ok = len(g) >= len(plan["fires"]) or (
                mode == "custom" and settled_since is not None and time.time() - settled_since >= d.get("fire_grace", 15.0))
            if settled and fires_ok:
                finished = time.time()
                break
            if pending and all_done:
                print("  固定任務都做完了，沒有一台在航線上符合起火條件（fire_after、fire_min_ahead）")
                break
            run.sleep(0.2)
        plan["fires"] = [f for f in plan["fires"] if "spawned" in f]
        if self.interactive:
            self.reply("  不再接受指令（回起飛點、產生報告）")
            self.ia_close()

        # 做完之後回到起飛點上空（BT 的 return_home_delay）
        homes = {v: (0.0, (v - 1) * spacing) for v in ids}

        def at_home(v):
            p = run.positions.get(v)
            return p is not None and math.hypot(p[0] - homes[v][0], p[1] - homes[v][1]) < 1.0 and abs(p[2] - alt) < 0.8
        if finished:
            run.until(lambda: all(at_home(v) for v in ids), d.get("return_wait", 40.0))
            # 剛進到 1 m 內就記錄的話，報告的距離會是還在靠近的 ~0.99 m（2026-10-19）；再等最多 5 s 讓它停好，
            # 報告的才是最後停的位置。通過的條件照舊是 1 m
            def settled(v):
                p = run.positions.get(v)
                return p is not None and math.hypot(p[0] - homes[v][0], p[1] - homes[v][1]) < 0.3
            run.until(lambda: all(settled(v) for v in ids), 5.0)

        who = {p["task_id"]: run.ever_held(p["task_id"], t0) for p in plan["patrol"]}
        missing = [it["name"] for it in items if it["kind"] == "task" and not it.get("done")] if mode == "custom" else []
        if mode == "custom" and self.interactive:
            plan["manual"] = [it["name"] for it in items if it["kind"] == "manual"]
        res.add("D1", f"{len(plan['patrol'])} 個巡檢任務：CBBA 分配 → 飛到 → 回報完成",
                "PASS" if all(done(p["task_id"]) for p in plan["patrol"]) and not missing else "FAIL",
                "；".join(f"{p['point']}→uav{'/'.join(map(str, who[p['task_id']])) or '?'}" +
                         ("" if done(p["task_id"]) else "（沒完成）") for p in plan["patrol"]) +
                (f"；條件沒達成、沒插入：{', '.join(missing)}" if missing else ""),
                elapsed=(finished - t0) if finished else None)
        g = self.ground_tasks(t0)
        rows = []
        for f in plan["fires"]:
            near = [e for e in g if math.hypot(e["msg"]["position"]["x"] - f["x"],
                                               e["msg"]["position"]["y"] - f["y"]) <= 2.0]
            where = (f"巡檢點 {f['point']}" if "point" in f else
                     f"航線旁 {f['side']}" if "side" in f else
                     f"自訂 ({f['x']}, {f['y']})" if f.get("custom") else f"uav{f['on_route_of']} 的航線上")
            if near:
                e = near[0]
                err = math.hypot(e["msg"]["position"]["x"] - f["x"], e["msg"]["position"]["y"] - f["y"])
                rows.append(f"{f['name']}（{where}）：uav{e['uav']} 起火後 {e['t'] - f['spawned']:.1f} s 發現、"
                            f"發布地面任務 {e['msg'][f_id]:08X}（誤差 {err:.2f} m）")
                f["ground_task"] = e["msg"][f_id]
                f["reported_by"] = e["uav"]
                f["reported_t"] = e["t"]
            else:
                rows.append(f"{f['name']}（{where}）：沒有發現")
        want = (len(d.get("side_points", [])) if mode == "side" else
                len(plan["fires"]) if mode == "custom" else d["fire_count"])
        ok = bool(plan["fires"]) and len(plan["fires"]) == want and \
            all("沒有" not in r for r in rows) and len(g) == len(plan["fires"])
        if plan["fires"] or mode != "custom":      # 沒有起火點的自訂情境不檢查火警
            res.add("D2", "看到火警（tag 101）→ 發布地面處置任務", "PASS" if ok else "FAIL",
                    "；".join(rows) + (f"（地面任務共 {len(g)} 個）" if rows else "沒有起火"))

        if plan["dog"]:
            # 每個起火點的地面任務都要由狗處理完（2026-10-14：原本只看第一個）
            rows3, ok3 = [], bool(plan["fires"])
            for fx in plan["fires"]:
                gx = fx.get("ground_task")
                tx = closed_at(gx) if gx else None
                took = gx is not None and any(e["kind"] == "assigned" and e["msg"].get(f_id) == gx for e in run.events)
                # 哪隻狗得標（2026-10-20，多隻狗）：最後一次拿到這個地面任務的
                dogs = [e["dog"] for e in run.events if e["kind"] == "dog_assigned" and gx is not None
                        and e["msg"].get(f_id) == gx]
                ok3 = ok3 and bool(tx) and not took
                rows3.append(f"{fx['name']}：" +
                             (f"地面任務 {gx:08X} 由狗 {dogs[-1]} 處理，發布後 {tx - fx['reported_t']:.1f} s 結束" if tx and dogs else
                              f"地面任務 {gx:08X} 在發布後 {tx - fx['reported_t']:.1f} s 結束" if tx else
                              ("地面任務沒有結束（看 fake_dog_*.log、dog_cbba_*.log）" if gx else "沒有地面任務")) +
                             ("；有無人機被指派地面任務（不對）" if took else ""))
            res.add("D3", "狗（假的 V60）得標、走過去、處理完（地面任務結束）", "PASS" if ok3 else "FAIL",
                    "；".join(rows3) if rows3 else "沒有起火")
            f = plan["fires"][0] if plan["fires"] else {}
            gt = f.get("ground_task")
            tc = closed_at(gt) if gt else None
            if mode == "side":
                self.check_side_divert(f, tc, plan)
        if plan["dog"] and mode == "point":
            f = plan["fires"][0] if plan["fires"] else {}
            gt = f.get("ground_task")
            tc = closed_at(gt) if gt else None
            # 發現的那台懸停監控到地面任務結束才完成那個巡檢點、離開
            pt = next((p for p in plan["patrol"] if p["point"] == f.get("point")), None)
            done_by = [(e["t"], e.get("uav")) for e in run.events if pt and e["kind"] == "result"
                       and e["msg"].get(f_id) == pt["task_id"] and e["msg"].get(f_ok) is True]
            tr = [t for t, _ in done_by]
            executor = done_by[0][1] if done_by else None
            v = f.get("reported_by")
            drift = None
            if v and tc:
                ps = [e for e in run.events if e["kind"] == "pos" and e["uav"] == v and f["reported_t"] <= e["t"] <= tc]
                drift = max((math.hypot(e["x"] - f["x"], e["y"] - f["y"]) for e in ps), default=None)
            watched = drift is not None and drift < 2.0
            if v and executor is not None and v != executor:
                # 先看到火的是別台（例如待命、飛回起飛點途中，2026-10-19）：那台監控到狗處理完；被派去巡檢點的那台
                # 照規則不重複監控（火已經有人報、有人盯），做完巡檢就走，所以巡檢點可以在狗處理完之前完成
                ok = bool(tr and tc and watched)
                detail = (f"uav{v} 先看到火、監控到狗處理完（離火點最遠 {drift:.2f} m）；巡檢點由 uav{executor} 完成"
                          f"（火已有人盯，不重複監控）" if tr and tc and drift is not None else "沒有看到監控到狗處理完")
            else:
                ok = bool(tr and tc and tr[0] >= tc - 0.5 and watched)
                detail = (f"uav{v} 在狗處理完後 {tr[0] - tc:.1f} s 回報完成；監控期間離火點最遠 {drift:.2f} m"
                          if tr and tc and drift is not None else "沒有看到「狗處理完 → 巡檢點完成」的順序")
            res.add("D4", "無人機懸停監控火點，狗處理完才完成那個巡檢點（別台先發現時：發現的那台監控到狗處理完）",
                    "PASS" if ok else "FAIL", detail)
        home_rows = []
        for v in ids:
            p = run.positions.get(v)
            home_rows.append(f"uav{v} " + (f"離起飛點 {math.hypot(p[0] - homes[v][0], p[1] - homes[v][1]):.2f} m、"
                                           f"高度 {p[2]:.1f} m" if p else "沒有位置"))
        res.add("D5", "全部做完後回到起飛點上空", "PASS" if all(at_home(v) for v in ids) else "FAIL",
                "；".join(home_rows))
        self.check_cbf(t0, time.time(), plan)

        print(f"  停 {self.hold:.0f} s 讓你看 Gazebo 畫面")
        run.sleep(self.hold)
        rec_end = time.time()
        env.pkill(env.sim(), "record_cameras.py")
        if recording:
            self.gz_record(False)
            run.sleep(3.0)                      # 存檔要一點時間
        with open(os.path.join(run.out, "demo_plan.json"), "w", encoding="utf-8") as f:
            json.dump(plan, f, ensure_ascii=False, indent=2)

        # 俯視動畫：sim 容器畫每一格（PNG），主機的 ffmpeg 合成 MP4
        frames = os.path.join(run.out, "frames")
        env.exec(env.sim(), f"python3 {env.repo}/docker/itest/animate_demo.py {run.cout} --frames {run.cout}/frames",
                 log=f"{run.cout}/animate.log")
        mp4 = os.path.join(run.out, "demo.mp4")
        if os.path.isdir(frames) and os.listdir(frames):
            fps = int(open(os.path.join(frames, "fps.txt")).read().strip())
            r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i",
                                os.path.join(frames, "%05d.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                                "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", mp4], capture_output=True, text=True) \
                if shutil.which("ffmpeg") else None
            if r is not None and r.returncode == 0:
                shutil.rmtree(frames)
            elif r is None:
                print(f"  主機沒有 ffmpeg（sudo apt install ffmpeg），每一格留在 {frames}；裝好後：\n"
                      f"    ffmpeg -framerate {fps} -i {frames}/%05d.png -c:v libx264 -pix_fmt yuv420p {mp4}")
        gz_ok = os.path.exists(os.path.join(run.out, "gazebo.mp4"))
        cams_mp4 = self.compose_cameras(run, rec.get("t0"), rec_end) if gz_ok else None
        res.add("D6", "影片", "INFO" if os.path.exists(mp4) and gz_ok else "FAIL",
                ("俯視動畫 demo.mp4" if os.path.exists(mp4) else f"俯視動畫沒有合成（看 {run.out}/animate.log、ffmpeg）") +
                ("；Gazebo 畫面 gazebo.mp4" if gz_ok else "；Gazebo 畫面沒有錄到") +
                ("；Gazebo＋下視相機 demo_cameras.mp4" if cams_mp4 else
                 ("；下視相機沒有合成（看 compose_cameras.log、record_cameras.log）" if gz_ok and self.record else "")))

    def compose_cameras(self, run, t0, t1):
        """下視相機的畫面（record_cameras.py）照時間拼成三台直排（compose_cameras.py，sim 容器），主機的 ffmpeg
        再和 gazebo.mp4 左右並排成 demo_cameras.mp4（2026-10-19）。gazebo.mp4 用模擬時間錄、相機用牆上時間對齊，
        模擬比實際慢（real time factor < 1）時兩邊會慢慢差開。回傳輸出的路徑，失敗時 None"""
        env = self.env
        cams = os.path.join(run.out, "cameras")
        if t0 is None or not os.path.isdir(cams) or not shutil.which("ffmpeg"):
            return None
        # 總寬 1280（2026-10-20 使用者：1280＋相機欄的 1600 × 720 太大）：gazebo.mp4 縮到 1024 × 576、相機一欄 256 × 576
        height = 576
        env.exec(env.sim(), f"python3 {env.repo}/docker/itest/compose_cameras.py {run.cout} --start {t0:.3f} "
                 f"--end {t1:.3f} --height {height}", log=f"{run.cout}/compose_cameras.log")
        frames = os.path.join(run.out, "camframes")
        if not os.path.isdir(frames) or not os.listdir(frames):
            return None
        fps = open(os.path.join(frames, "fps.txt")).read().strip()
        out = os.path.join(run.out, "demo_cameras.mp4")
        r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", os.path.join(run.out, "gazebo.mp4"),
                            "-framerate", fps, "-i", os.path.join(frames, "%05d.jpg"), "-filter_complex",
                            f"[0:v]scale=-2:{height}[g];[1:v]scale=-2:{height},fps=15[c];[g][c]hstack=inputs=2",
                            "-shortest", "-c:v", "libx264", "-pix_fmt", "yuv420p", out], capture_output=True, text=True)
        if r.returncode != 0:
            print(f"  下視相機合成失敗：{r.stderr.strip()[-300:]}")
            return None
        shutil.rmtree(frames, ignore_errors=True)
        shutil.rmtree(cams, ignore_errors=True)
        return out


# =============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("suite", choices=["comm", "uav", "loss", "demo"])
    ap.add_argument("--config", default=os.path.join(HERE, "config.json"))
    ap.add_argument("--only", default="")
    ap.add_argument("--kill-existing", action="store_true")
    ap.add_argument("--levels", default="0,10,30,50,70")
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--delay", type=int, default=50)
    ap.add_argument("--jitter", type=int, default=30)
    ap.add_argument("--hold", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--interactive", action="store_true",
                    help="demo：起飛後在 terminal 手動插入任務點、起火點（t、tf、f；h 看說明）")
    ap.add_argument("--no-patrol", action="store_true",
                    help="demo --interactive：不建立原本的巡檢任務（G1～G3），全部由手動插入")
    ap.add_argument("--area", type=float, nargs=4, default=None, metavar=("X0", "X1", "Y0", "Y1"),
                    help="demo：手動、隨機插入的範圍（預設 config 的 demo.area）")
    ap.add_argument("--scenario", default=None,
                    help="demo：自訂情境檔（JSON，任務、起火點、插入時機；範例在 docker/itest/scenarios/）")
    ap.add_argument("--fire-mode", choices=["point", "route", "side"], default=None,
                    help="demo 的起火方式（預設照 config.json 的 demo.fire_mode）：point 巡檢點上、route 航線上、"
                         "side 航線旁的固定點（demo.side_points）")
    ap.add_argument("--executor", choices=["bt", "task_manager"], default="task_manager",
                    help="uav、demo：無人機的 BT（task_manager = 照機器狗端骨架的 uav_task_manager_node，預設；bt = 備用 uav_bt_node）")
    args = ap.parse_args()
    env = Env(args.config)
    env.executor = args.executor
    only = {s.strip() for s in args.only.split(",") if s.strip()}

    # 已經有在跑的東西時先停下來（避免打斷使用者正在做的事）
    busy = []
    for vid in env.ids:
        for pat in (env.cfg["comm"]["stop_pattern"].format(id=vid), env.cfg["uav"]["stop_pattern"]):
            if env.running(env.uav(vid), pat):
                busy.append((env.uav(vid), pat))
    for pat in ("bin/px4 -i", "gz[ ]sim", "parameter_bridge", "ros_bridge.py", "fake_dog.py", "record_cameras.py"):
        if env.running(env.sim(), pat):
            busy.append((env.sim(), pat))
    dog = env.cfg.get("dog")
    if dog and env.running(dog["container"], dog["stop_pattern"]):
        busy.append((dog["container"], dog["stop_pattern"]))
    if busy and not args.kill_existing:
        print("容器裡已經有在跑的節點（加 --kill-existing 會先關掉）：")
        for c, p in busy:
            print(f"  {c}：{p}")
        return 2
    for c, p in busy:
        if not env.pkill_hard(c, p):
            print(f"  {c}：{p} 關不掉（SIGKILL 也沒用），試試 docker restart {c}")
            return 2
    if busy:
        time.sleep(2)

    out_dir = os.path.join(HERE, "out", time.strftime("%Y%m%d_%H%M%S") + "_" + args.suite)
    os.makedirs(out_dir, exist_ok=True)
    print(f"[itest] {args.suite}：{len(env.ids)} 台，輸出 {out_dir}" +
          (f"，BT {args.executor}" if args.suite in ("uav", "demo") else ""))
    run = Run(env, out_dir, with_px4=args.suite in ("uav", "demo"))
    if args.suite == "loss":
        suite = LossSuite(env, run, only, [int(x) for x in args.levels.split(",") if x.strip()],
                          args.trials, args.delay, args.jitter)
    elif args.suite == "demo":
        seed = args.seed if args.seed is not None else int(time.time()) % 10000
        scenario = None
        if args.scenario:
            path = args.scenario if os.path.exists(args.scenario) else os.path.join(HERE, "scenarios", args.scenario)
            if not os.path.exists(path) and not path.endswith(".json"):
                path += ".json"
            with open(path, encoding="utf-8") as f:
                scenario = json.load(f)
            scenario["_file"] = os.path.basename(path)
        suite = DemoSuite(env, run, only, args.hold, seed, args.speed, not args.no_record, args.fire_mode, scenario,
                          args.interactive, args.area, not args.no_patrol)
    else:
        suite = (CommSuite if args.suite == "comm" else UavSuite)(env, run, only)
    try:
        suite.run_all()
    finally:
        run.quit()
        for vid in env.ids:
            env.pkill(env.uav(vid), env.cfg["comm"]["stop_pattern"].format(id=vid))
            if args.suite in ("uav", "demo"):
                env.pkill(env.uav(vid), env.cfg["uav"]["stop_pattern"])
        if args.suite in ("uav", "demo"):
            dog = env.cfg.get("dog")
            if dog:
                env.pkill(dog["container"], dog["stop_pattern"])
                env.pkill(env.sim(), dog["sim_stop_pattern"])
            # px4_start 的 stdin（tail -f /dev/null）：不關的話每跑一次留一個（2026-10-13 前用 sleep infinity，
            # 一直累積；而且容器的主程序也是 sleep infinity，不能用它比對）
            for pat in ("bin/px4 -i", "gz[ ]sim", "parameter_bridge", "tail -f /dev/null", "record_cameras.py"):
                env.pkill_hard(env.sim(), pat)
        if args.suite in ("comm", "loss"):
            for vid in env.ids:
                env.exec(env.uav(vid), "mesh_tc.sh clear")
    title = {"comm": "CBBA 之間的通訊", "uav": "UAV 整串", "loss": "弱網量測（掉包梯度）",
             "demo": "展示（Gazebo＋GIF）"}[args.suite]
    return 1 if suite.res.write(out_dir, f"整合測試：{title}") else 0


if __name__ == "__main__":
    sys.exit(main())

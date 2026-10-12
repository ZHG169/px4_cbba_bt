#!/usr/bin/env python3
"""整合測試：只看話題和網路，不看 log，換成別人寫的 CBBA Node／BT 也能用（介面寫在 config.json）。

  python3 docker/itest/itest.py comm     CBBA 之間的通訊（不開 PX4、BT）：每台在自己的容器啟動 CBBA Node，
                                         位置用假的 robot_state，測分配、完成、失敗交回、停止送狀態、節點停止、
                                         重開機、掉包 30%、斷網
  python3 docker/itest/itest.py uav      UAV 整串（PX4 SITL＋Gazebo＋CBBA Node＋BT）：起飛、單一任務、火警、
                                         誘餌、三台分配與避碰、PX4 斷線接手

在主機執行（要能用 docker），容器要先用 docker/start.sh 開好、套件要先編譯好。
  --config 檔案       換成別的設定（預設同目錄的 config.json）
  --only C1,C3        只跑指定的項目（前置步驟照跑）
  --kill-existing     容器裡已經有在跑的節點、PX4、Gazebo 時先關掉（預設會停下來，避免打斷你正在做的事）

結果：每一項 PASS／FAIL／INFO，寫到 docker/itest/out/<時間>/report.md（各節點的輸出也在那個目錄）。
"""
import argparse
import json
import math
import os
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

    def new_task(self, x, y, duration=5.0):
        self.seq += 1
        tid = self.creator * 65536 + self.seq
        fields = fill(self.cfg["messages"]["new_task"],
                      dict(task_id=tid, x=float(x), y=float(y), duration=float(duration)))
        self.run.publish("new_task", self.creator, fields)
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

    def tc(self, loss):
        for vid in self.env.ids:
            if loss is None:
                self.env.exec(self.env.uav(vid), "mesh_tc.sh clear")
            else:
                self.env.exec(self.env.uav(vid), f"mesh_tc.sh set {loss}", check=True)

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

    def run_all(self):
        env, run, res, u = self.env, self.run, self.res, self.cfg["uav"]
        ids = env.ids
        for vid in ids:
            env.exec(env.uav(vid), u["start"].format(id=vid, ns=env.ns(vid)), detach=True,
                     log=f"{run.cout}/stack_uav{vid}.log")
        run.sleep(8.0)
        for i, vid in enumerate(ids):
            senv = {"HEADLESS": "1"} if u.get("headless") else {}
            if i == 0 and u.get("world"):
                senv["CBBA_WORLD"] = u["world"]
            env.exec(env.sim(), u["px4_start"].format(id=vid), detach=True, env=senv,
                     log=f"{run.cout}/px4_{vid}.log")
            run.sleep(20.0 if i == 0 else 12.0)
        if u.get("camera_start"):
            env.exec(env.sim(), u["camera_start"], detach=True, log=f"{run.cout}/camera.log")

        alt = u["altitude"]
        t = run.until(lambda: all(run.armed.get(v) and run.positions.get(v, (0, 0, 0))[2] >= alt - 0.6 for v in ids), 150)
        up = [v for v in ids if run.armed.get(v) and run.positions.get(v, (0, 0, 0))[2] >= alt - 0.6]
        res.add("U1", "起飛到巡檢高度", "PASS" if t else "FAIL", f"已起飛：{up}" + ("" if t else f"（全部 {ids}）"))
        if not t:
            return
        run.sleep(5.0)    # 起飛後 CBBA 開始參與出價

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


# =============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("suite", choices=["comm", "uav"])
    ap.add_argument("--config", default=os.path.join(HERE, "config.json"))
    ap.add_argument("--only", default="")
    ap.add_argument("--kill-existing", action="store_true")
    args = ap.parse_args()
    env = Env(args.config)
    only = {s.strip() for s in args.only.split(",") if s.strip()}

    # 已經有在跑的東西時先停下來（避免打斷使用者正在做的事）
    busy = []
    for vid in env.ids:
        for pat in (env.cfg["comm"]["stop_pattern"].format(id=vid), env.cfg["uav"]["stop_pattern"]):
            if env.running(env.uav(vid), pat):
                busy.append((env.uav(vid), pat))
    for pat in ("bin/px4 -i", "gz[ ]sim", "parameter_bridge", "ros_bridge.py"):
        if env.running(env.sim(), pat):
            busy.append((env.sim(), pat))
    if busy and not args.kill_existing:
        print("容器裡已經有在跑的節點（加 --kill-existing 會先關掉）：")
        for c, p in busy:
            print(f"  {c}：{p}")
        return 2
    for c, p in busy:
        env.pkill(c, p)
    if busy:
        time.sleep(3)

    out_dir = os.path.join(HERE, "out", time.strftime("%Y%m%d_%H%M%S") + "_" + args.suite)
    os.makedirs(out_dir, exist_ok=True)
    print(f"[itest] {args.suite}：{len(env.ids)} 台，輸出 {out_dir}")
    run = Run(env, out_dir, with_px4=args.suite == "uav")
    suite = (CommSuite if args.suite == "comm" else UavSuite)(env, run, only)
    try:
        suite.run_all()
    finally:
        run.quit()
        for vid in env.ids:
            env.pkill(env.uav(vid), env.cfg["comm"]["stop_pattern"].format(id=vid))
            if args.suite == "uav":
                env.pkill(env.uav(vid), env.cfg["uav"]["stop_pattern"])
        if args.suite == "uav":
            for pat in ("bin/px4 -i", "gz[ ]sim", "parameter_bridge"):
                env.pkill(env.sim(), pat)
        if args.suite == "comm":
            for vid in env.ids:
                env.exec(env.uav(vid), "mesh_tc.sh clear")
    title = "CBBA 之間的通訊" if args.suite == "comm" else "UAV 整串"
    return 1 if suite.res.write(out_dir, f"整合測試：{title}") else 0


if __name__ == "__main__":
    sys.exit(main())

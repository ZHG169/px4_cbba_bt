#!/usr/bin/env python3
"""整合測試：只看話題和網路，不看 log，換成別人寫的 CBBA Node／BT 也能用（介面寫在 config.json）。

  python3 docker/itest/itest.py comm     CBBA 之間的通訊（不開 PX4、BT）：每台在自己的容器啟動 CBBA Node，
                                         位置用假的 robot_state，測分配、完成、失敗交回、停止送狀態、節點停止、
                                         重開機、掉包 30%、斷網（C8 只有建立者知道、C9 100% 斷網後各自完成與恢復）
  python3 docker/itest/itest.py uav      UAV 整串（PX4 SITL＋Gazebo＋CBBA Node＋BT）：起飛、單一任務、火警、
                                         誘餌、三台分配與避碰、PX4 斷線接手
  python3 docker/itest/itest.py demo     展示：demo_site 開 Gazebo 畫面（錄成 gazebo.mp4），全部起飛後才開始 CBBA，3 個固定任務；出發後火點出現在某台的
                                         航線上，飛行途中發現、發布地面任務（給 V60，先不開狗）、繼續飛；結束後畫俯視動畫
                                         demo.mp4（要主機的 ffmpeg）。
                                         --seed N 重現同一個情境、--speed 3（m/s，預設；2026-10-23 改成 0.5、10-24 改回 3，CBF 參數照速度自動算）、--hold 10、--no-record
  python3 docker/itest/itest.py loss     第 5 週弱網量測（不開 PX4、BT）：掉包梯度 0／10／30／50／70%（延遲 50±30 ms），
                                         每個掉包率試驗 N 次、每次建立 9 個任務（多源插單，每台身邊 3 個），記錄
                                         A 送達（每台知道幾個任務）、B 分配（每台分到幾個、和理論比）、收斂時間、
                                         5 s 內收斂率、重複認領、完成後撤下的時間、mesh 頻寬（trials.csv、levels.csv）

在主機執行（要能用 docker），容器要先用 docker/start.sh 開好、套件要先編譯好。
  --config 檔案       換成別的設定（預設同目錄的 config.json）
  --only C1,C3        只跑指定的項目（前置步驟照跑）
  --kill-existing     容器裡已經有在跑的節點、PX4、Gazebo 時先關掉（預設會停下來，避免打斷你正在做的事）
  --levels 0,10,30    loss：要測的掉包率（%）；--trials 20 每個掉包率試驗幾次；--delay 50 --jitter 30（ms）；
                      --tasks 9 每次試驗的任務數（理論上每台分到 tasks／台數 個）
  --node-param K=V    comm、loss：每台 cbba_node 多加的參數，可以給多次（例如 resend_gap=0.5、completion_retry=0.5）

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
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dog_patrol  # noqa: E402  狗巡檢點的進度（2026-11-07）


# =============================================================================
# 設定、容器
# =============================================================================
def load_equipment(path, skip=(), cover=()):
    """場景的設備（2026-11-05，隨機挑點考慮設備）：讀 <場景>_env.sdf.inc 每個 model 的位置、box／cylinder 大小。
    回傳 (設備, 遮蔽)：設備 = [dict(name, x, y, hx, hy | r, top)]，名稱開頭在 skip 的不算（圍牆、起降板…）；
    遮蔽 = cover 的 model（例如鋼構棚頂：底下的點下視相機看不到），只用來排除"""
    text = open(path, encoding="utf-8").read()
    equip, covers = [], []
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.S):
        name, body = m.group(1), m.group(2)
        pose = re.search(r"<pose>([^<]+)</pose>", body)
        if not pose:
            continue
        x, y, z = (float(v) for v in pose.group(1).split()[:3])
        e = dict(name=name, x=x, y=y)
        box = re.search(r"<box>\s*<size>([^<]+)</size>", body)
        cyl = re.search(r"<cylinder>\s*<radius>([^<]+)</radius>\s*<length>([^<]+)</length>", body)
        if box:
            sx, sy, sz = (float(v) for v in box.group(1).split())
            e.update(hx=sx / 2, hy=sy / 2, top=z + sz / 2, h=sz)
        elif cyl:
            e.update(r=float(cyl.group(1)), top=z + float(cyl.group(2)) / 2, h=float(cyl.group(2)))
        else:
            continue
        if any(name.startswith(c) for c in cover):
            covers.append(e)
        elif not any(name.startswith(k) for k in skip):
            equip.append(e)
    return equip, covers


def seg_dist(px, py, a, b):
    """點 (px, py) 到線段 a–b 的距離"""
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    k = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + k * dx), py - (ay + k * dy))


def in_footprint(e, x, y, margin=0.0):
    """(x, y) 在設備 e 的投影範圍（往外 margin m；margin 負數 = 往內縮）裡"""
    if "r" in e:
        return math.hypot(x - e["x"], y - e["y"]) <= e["r"] + margin
    return abs(x - e["x"]) <= e["hx"] + margin and abs(y - e["y"]) <= e["hy"] + margin


def cbf_speed_args(speed, safe_radius=1.0, max_accel=1.5):
    """巡航速度改了時，避碰跟著要改的參數（yaml 是照 0.5 m/s 定的，≤ 0.6 m/s 不給）。
    2026-10-24 起預設二階 CBF（cbf.second_order：CBF 管加速度、h 用剎車距離），剎車由 CBF 自己處理，
    guard_radius、γ、emergency_speed 不用跟著改（一階時 3 m/s 要 guard 4 m，起降板上三台互相減速）：
      miss_clearance  safe_radius + 0.5 + 0.2 V：讓的那台繞到預計最近距離這麼遠（3 m/s → 2.1 m；一階時是 4.5 m，繞太大）
      yield_exit      miss_clearance + 0.5 m"""
    if speed <= 0.6:
        return ""
    miss = safe_radius + 0.5 + 0.2 * speed
    return f"-p cbf.miss_clearance:={miss:.1f} -p cbf.yield_exit:={miss + 0.5:.1f}"


class Env:
    def __init__(self, cfg_path):
        with open(cfg_path, encoding="utf-8") as f:
            self.cfg = json.load(f)
        self.cfg_path = os.path.abspath(cfg_path)
        p = self.cfg["paths"]
        self.host_repo = os.path.normpath(os.path.join(os.path.dirname(self.cfg_path), p["host_repo"]))
        self.repo = p["container_repo"]
        self.ids = self.cfg["vehicles"]["ids"]
        self.spawn_origin = tuple(self.cfg["vehicles"].get("spawn_origin", [0.0, 0.0]))
        self.bat_drain = None      # 模擬電池續航（s）；None = 腳本預設 900 s
        self.uav_ids = list(self.ids)
        self.dogs = {}             # comm、loss 的狗（--dogs N，2026-11-09）：機號 → config 的 comm.dogs 那一筆

    def add_dogs(self, n):
        """comm、loss 加 n 隻狗（2026-11-09，第 5 週「空地」CBBA）：每隻狗在自己的容器 cbba_dogK（docker/.env 的
        DOG_COUNT）跑 cbba_node（agent_type UGV），有自己的 mesh 網卡，空地之間每條連線都經過 tc。
        加進 self.ids（機號 50、51…），容器、namespace、tc、頻寬都照機號對應；self.uav_ids 只有無人機"""
        units = (self.cfg["comm"].get("dogs") or [])[:n]
        if len(units) < n:
            raise SystemExit(f"--dogs {n}：config 的 comm.dogs 只有 {len(units)} 隻")
        self.dogs = {u["id"]: u for u in units}
        self.ids = self.uav_ids + [u["id"] for u in units]

    def is_dog(self, vid):
        return vid in self.dogs

    def dog_containers(self):
        """demo 的狗跑 cbba_node、BT 的容器（去重複）：dog.units 每隻的 container（2026-11-09 起 cbba_dogK，自己的
        mesh 網卡），沒寫時用 dog.container（原本借 uav1）"""
        dog = self.cfg.get("dog") or {}
        units = dog.get("units") or []
        return list(dict.fromkeys([u.get("container") or dog.get("container") for u in units] or
                                  ([dog["container"]] if dog.get("container") else [])))

    def dog_container(self, unit):
        dog = self.cfg.get("dog") or {}
        return unit.get("container") or dog.get("container")

    def node_pattern(self, vid):
        """pkill／pgrep 這台 cbba_node 用的樣式（狗的節點名稱是 <ns>_cbba_node）"""
        if self.is_dog(vid):
            return self.cfg["comm"]["dog_stop_pattern"].format(id=vid, ns=self.ns(vid))
        return self.cfg["comm"]["stop_pattern"].format(id=vid)

    def name(self, vid):
        """報告、log、圖上的名稱：uav1、dog1（狗照機號排序叫 dog1～dogN，和 2D 動畫、容器 cbba_dogK 一致，2026-11-09
        使用者：不用機號 dog50）"""
        if self.is_dog(vid):
            return f"dog{sorted(self.dogs).index(vid) + 1}"
        return f"uav{vid}"

    def apply_world(self, name):
        """demo 換場景（--world，2026-10-24）：config 的 worlds.<場景> 蓋掉 demo 的同名欄位、巡檢高度、出生原點、
        假狗的出生點。沒列在 worlds 的場景（demo_site、fire_site…）照 config 原本的值"""
        self.cfg["demo"]["world"] = name
        w = self.cfg.get("worlds", {}).get(name, {})
        self.cfg["demo"].update(w.get("demo", {}))
        if "altitude" in w:
            self.cfg["uav"]["altitude"] = w["altitude"]
        if "spawn_origin" in w:
            self.spawn_origin = tuple(w["spawn_origin"])
        if "spawn_spacing" in w:
            self.cfg["vehicles"]["spawn_spacing"] = w["spawn_spacing"]
        self.bat_drain = w.get("bat_drain")
        for u in (self.cfg.get("dog") or {}).get("units", []):
            if u["ns"] in w.get("dog_start", {}):
                u["start_xy"] = w["dog_start"][u["ns"]]

    def home(self, vid):
        """第 vid 台的出生點（map ENU）：出生原點 + (0, (vid − 1) × spawn_spacing)，和 px4_sitl.sh、cbba_uav.sh 一樣"""
        return (self.spawn_origin[0], self.spawn_origin[1] + (vid - 1) * self.cfg["vehicles"]["spawn_spacing"])

    def spawn_env(self):
        """給 px4_sitl.sh、cbba_uav.sh 的 UAV_SPAWN_ORIGIN（兩邊要一樣，節點才換算得對）"""
        out = {"UAV_SPAWN_ORIGIN": f"{self.spawn_origin[0]:g},{self.spawn_origin[1]:g}",
               "UAV_SPAWN_SPACING": f"{self.cfg['vehicles']['spawn_spacing']:g}"}
        if self.bat_drain:
            out["SIM_BAT_DRAIN"] = f"{self.bat_drain:g}"    # px4_sitl.sh 設模擬電池、cbba_uav.sh 算能量模型，兩邊要一樣
        return out

    def ns(self, vid):
        if self.is_dog(vid):
            return self.dogs[vid]["ns"]
        return self.cfg["vehicles"]["ns"].format(id=vid)

    def sim(self):
        return self.cfg["containers"]["sim"]

    def uav(self, vid):
        """這台的容器（狗：cbba_dogK，2026-11-09）"""
        if self.is_dog(vid):
            return self.dogs[vid]["container"]
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


# gdb 批次模式：跑到結束；收到訊號（SIGSEGV 等）停下時印出當機的位置、所有執行緒的 backtrace，然後結束。
# SIGINT、SIGTERM 照常交給節點（itest 用 pkill 關節點），不讓 gdb 攔下來
GDB_PREFIX = ("gdb -q -batch -ex 'handle SIGINT SIGTERM nostop noprint pass' -ex run -ex bt "
              "-ex 'thread apply all bt' --args")


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
        self.known = {}          # uav -> {task_id: 第一次知道的時刻}（task_announced，2026-10-27）
        self.paths = {}          # uav -> [task_id…]（cbba_path：這台目前的 bundle，2026-10-27）
        self.closed = {}         # uav -> {task_id: 收到 task_closed 的時刻}（這台確認任務結束，2026-10-27）
        self.tx = {}             # uav -> [(t, 送出位元組累計, 其中重傳, 其中轉送)]（cbba_tx，2026-11-09）
        self.cmd_file = os.path.join(out_dir, "commands.jsonl")
        open(self.cmd_file, "w").close()
        open(os.path.join(out_dir, "events.jsonl"), "w").close()
        cfg_path = env.cfg_path
        if env.dogs:
            # 有狗時（comm、loss 的 --dogs）：ros_bridge 也要訂狗的話題 → 寫一份加了狗的設定（vehicles.ids、ns_map）
            cfg = json.loads(json.dumps(env.cfg))
            cfg["vehicles"]["ids"] = list(env.ids)
            cfg["vehicles"]["ns_map"] = {str(v): env.ns(v) for v in env.ids}
            cfg_path = os.path.join(out_dir, "config_effective.json")
            with open(cfg_path, "w", encoding="utf-8") as f:
                json.dump(cfg, f, ensure_ascii=False, indent=1)
        env.exec(env.sim(), f"exec python3 {env.repo}/docker/itest/ros_bridge.py --config {env.to_container(cfg_path)}"
                 f" --dir {self.cout}" + (" --px4" if with_px4 else ""), detach=True, log=f"{self.cout}/ros_bridge.log",
                 env=env.spawn_env())     # PX4 的位置換成 map 要出生原點
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
            elif e["kind"] == "announced":
                self.known.setdefault(e["uav"], {}).setdefault(e["msg"].get(f_id), e["t"])
            elif e["kind"] == "path":
                self.paths[e["uav"]] = list(e["msg"].get("data") or [])
            elif e["kind"] == "tx":
                self.tx.setdefault(e["uav"], []).append((e["t"], *e["d"]))
            elif e["kind"] == "closed":
                self.closed.setdefault(e["uav"], {}).setdefault(e["msg"].get(f_id), e["t"])
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

    def tx_at(self, vid, t):
        """這台在 t 時（之前最後一筆）的送出位元組累計 (全部, 重傳, 轉送)；沒有資料 → None"""
        best = None
        for row in self.tx.get(vid, []):
            if row[0] > t:
                break
            best = row
        return best[1:] if best else None

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

    def in_paths(self, tid):
        """路徑（bundle）裡有 tid 的載具"""
        return sorted(u for u, p in self.paths.items() if tid in p)


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

    def new_task(self, x, y, duration=5.0, creator=None, value=None, task_type=None):
        creator = creator or self.creator
        self.seq += 1
        tid = creator * 65536 + self.seq
        fields = fill(self.cfg["messages"]["new_task"],
                      dict(task_id=tid, x=float(x), y=float(y), duration=float(duration)))
        if value is not None:
            fields["value"] = float(value)
        if task_type is not None:            # 1 空中偵察、2 地面處置（火警）、3 巡邏（2026-11-09，loss 的空地混合任務）
            fields["type"] = int(task_type)
        self.run.publish("new_task", creator, fields)
        return tid, time.time()

    def report(self, vid, tid, success):
        _, ver, _ = self.run.assigned.get(vid, (None, None, None))
        key = "result_success" if success else "result_failure"
        fields = fill(self.cfg["messages"][key], dict(task_id=tid, version=ver if ver is not None else 0))
        if not self.cfg["fields"]["version"]:
            fields.pop("assignment_version", None)
        return self.run.publish("result", vid, fields)   # ros_bridge 的 ack：t = 實際發出的時刻

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
        pos = {int(k): v for k, v in self.cfg["comm"]["positions"].items()}
        pos.update({vid: u["pos"] for vid, u in self.env.dogs.items()})      # --dogs（2026-11-09）
        return pos

    def nearest(self, x, y, ground=False):
        """離 (x, y) 最近、能做這種任務的那台：空中任務只看無人機、地面任務只看狗（types.hpp 的相容表）"""
        ok = [v for v in self.env.ids if self.env.is_dog(v) == ground]
        pos = self.positions()
        return min(ok, key=lambda v: math.hypot(pos[v][0] - x, pos[v][1] - y))

    def state_on(self, vid):
        x, y = self.positions()[vid]
        fields = fill(self.cfg["messages"]["robot_state"], dict(id=vid, x=float(x), y=float(y)))
        self.run.send("periodic", key=f"rs{vid}", rate=10, fields=fields, **self.run.topic("robot_state", vid))

    def state_off(self, vid):
        self.run.send("stop_periodic", key=f"rs{vid}")

    def start_node(self, vid):
        c = self.cfg["comm"]
        # 狗（--dogs，2026-11-09）：cbba_v60.yaml（agent_type UGV，和 demo 的狗一樣），在自己的容器 cbba_dogK
        cmd = c["dog_start" if self.env.is_dog(vid) else "start"].format(id=vid, ns=self.env.ns(vid))
        # --node-param resend_gap=0.5（2026-11-04，第 5 週比較補發、完成確認的參數）：每台的 cbba_node 都加
        cmd += "".join(f" -p {k}:={v}" for k, v in getattr(self.env, "node_params", []))
        if getattr(self.env, "gdb", False):
            # --gdb（2026-10-27，追 cbba_node 的 Segmentation fault）：在 gdb 底下跑，當掉時把 backtrace 印到 node_uavN.log
            # （要 RelWithDebInfo 編譯才有行號；映像要有 gdb，robot.Dockerfile）
            cmd = cmd.replace("ros2 run ", f'ros2 run --prefix "{GDB_PREFIX}" ', 1)
        tag = getattr(self, "log_tag", "")     # loss 每個掉包率重新啟動時分開記（node_uavN_L30.log），當機的 log 不被蓋掉
        self.env.exec(self.env.uav(vid), "exec " + cmd, detach=True,
                      log=f"{self.run.cout}/node_{self.env.name(vid)}{tag}.log")

    def stop_node(self, vid):
        self.env.pkill(self.env.uav(vid), self.env.node_pattern(vid))

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
                "各台的 assigned 都收到" if t else
                f"收不到 {[env.name(v) for v in missing]} 的 assigned（節點沒起來或話題名稱不對）")
        if not t:
            return
        run.sleep(self.cfg["comm"]["warmup"])
        b0, t0 = env.mesh_bytes(ids[0]), time.time()
        run.sleep(3.0)
        b1 = env.mesh_bytes(ids[0])
        if b0 is not None and b1 is not None:
            res.add("M", "mesh 流量（待命、沒有任務）", "INFO", f"uav{ids[0]} 約 {(b1 - b0) / (time.time() - t0) / 1024:.1f} kB/s")
        # C1～C8 是無人機之間的空中任務（狗不出價）：有 --dogs 時狗只當旁觀的鄰居（照樣收送封包、經過 tc），
        # 挑「某一台」時只挑無人機；C9 的 blackout 另外用全部的 agent（2026-11-09）
        ids = env.uav_ids

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

        if self.want("C9"):
            self.blackout()

    def blackout(self, cut_sec=6.0):
        """C9：100% 斷網（週計畫第 5 週「100% 斷網 → 單機貪婪、安全完成手上的任務」，2026-11-04）。
        每台身邊一個任務（別台建立）、各自拿著 → 全部斷網 cut_sec 秒（> lost_timeout 1.5 s、< takeover_patrol 30 s）：
          ① 失聯後手上的任務照樣拿著（已接受的鎖定，不釋放）
          ② 斷網中各自回報完成 → 本機確認結束（要確認的鄰居都失聯了，不用等）
          ③ 斷網中各台自己建一個新任務 → 只有自己知道、自己接（單機貪婪）
        恢復後：① 的完成全隊都知道（task_closed）、③ 的任務全隊都知道、各在原本那台的路徑裡；
        沒有重複認領（③ 從斷網到結束只有建立者拿過）、完成的沒有再派"""
        run, ids, res = self.run, self.env.ids, self.res
        pos = self.positions()
        pre = {}                                  # 斷網前的任務 → 該拿著的那台
        # 有狗時（--dogs，2026-11-09）：狗身邊的是地面任務（斷網前的是火警、斷網中自己建的是巡邏），空地各自退化成單機
        for i, v in enumerate(ids):
            x, y = pos[v]
            tid, _ = self.new_task(x + 1.5, y + 1.0, creator=ids[(i + 1) % len(ids)],
                                   task_type=2 if self.env.is_dog(v) else None)
            pre[tid] = v
        if run.until(lambda: all(run.holders(t) == [v] for t, v in pre.items()), 10.0) is None:
            res.add("C9", "100% 斷網：各自做完手上的、自己建的自己接；恢復後一致", "FAIL",
                    "斷網前 10 s 內沒有各自拿到身邊的任務：" + "、".join(f"{t:08X}→{run.holders(t)}" for t in pre))
            for t in pre:
                self.finish(t)
            return
        new, notes = {}, []
        self.tc(100)
        try:
            t_cut = time.time()
            run.sleep(2.5)                        # > lost_timeout：互相判定失聯
            kept = all(run.holders(t) == [v] for t, v in pre.items())
            t_done = time.time()
            for t, v in pre.items():
                self.report(v, t, True)
            closed_local = run.until(lambda: all(t in run.closed.get(v, {}) for t, v in pre.items()), 3.0)
            for v in ids:
                x, y = pos[v]
                tid, _ = self.new_task(x - 1.5, y - 1.0, creator=v, task_type=3 if self.env.is_dog(v) else None)
                new[tid] = v
            local = run.until(lambda: all(run.holders(t) == [v] for t, v in new.items()), 3.0)
            run.sleep(max(0.0, cut_sec - (time.time() - t_cut)))
        finally:
            self.tc(None)
        t_back = time.time()

        def synced():
            return (all(t in run.closed.get(u, {}) for t in pre for u in ids) and
                    all(t in run.known.get(u, {}) for t in new for u in ids) and
                    all(run.in_paths(t) == [v] for t, v in new.items()))
        t_sync = run.until(synced, 20.0)
        run.sleep(1.0)
        dup = {t: run.ever_held(t, t_cut) for t, v in new.items() if run.ever_held(t, t_cut) != [v]}
        redone = sorted({task for (tt, _, task, _) in run.history if task in pre and tt > t_done + 0.5})
        if not kept:
            notes.append("失聯後手上的任務被釋放：" + "、".join(f"{t:08X}→{run.ever_held(t, t_cut)}" for t in pre))
        if closed_local is None:
            notes.append("斷網中回報完成後，本機 3 s 內沒有確認結束")
        if local is None:
            notes.append("斷網中自己建的任務 3 s 內沒有自己接")
        if t_sync is None:
            miss_close = [f"{t:08X}@{self.env.name(u)}" for t in pre for u in ids if t not in run.closed.get(u, {})]
            miss_know = [f"{t:08X}@{self.env.name(u)}" for t in new for u in ids if t not in run.known.get(u, {})]
            where = [f"{t:08X}→{run.in_paths(t)}" for t, v in new.items() if run.in_paths(t) != [v]]
            notes.append("恢復後 20 s 內沒有一致：" + "；".join(x for x in (
                ("沒收到完成 " + "、".join(miss_close)) if miss_close else "",
                ("不知道新任務 " + "、".join(miss_know)) if miss_know else "",
                ("路徑不對 " + "、".join(where)) if where else "") if x))
        if dup:
            notes.append("重複認領：" + "、".join(f"{t:08X}→{h}" for t, h in dup.items()))
        if redone:
            notes.append("完成後又派：" + "、".join(f"{t:08X}" for t in redone))
        ok = not notes
        res.add("C9", "100% 斷網：各自做完手上的、自己建的自己接；恢復後一致", "PASS" if ok else "FAIL",
                (f"斷網 {cut_sec:.0f} s：{len(pre)} 台" + (f"（含狗 {len(self.env.dogs)} 隻）" if self.env.dogs else "") +
                 f"手上的任務照樣拿著、回報完成後本機確認結束 "
                 f"{closed_local - t_done:.2f} s、自己建的任務自己接；恢復後 {t_sync - t_back:.2f} s 全隊一致"
                 f"（完成的全隊都知道、新任務都知道而且在原本那台），沒有重複認領、完成後沒有再派")
                if ok else "；".join(notes), resync=(t_sync - t_back) if t_sync else None)
        for t in new:
            self.finish(t)


# -----------------------------------------------------------------------------
# 第 5 週弱網量測（loss）
# -----------------------------------------------------------------------------
class LossSuite(CommSuite):
    """掉包梯度：每個掉包率試驗 trials 次，每次建立 tasks 個任務（預設 9 個、3 台，2026-10-27；原本一台一個），
    多源插單：每個任務放在某一台（理論上的執行者）身邊 2.5 m，由**別台**建立（要靠協商）。理論上每台該分到
    「放在它身邊的那幾個」（9 個 → 每台 3 個；別台的任務遠 20 m，出價一定較低）。只看話題：
      A 送達   每台知道幾個任務（task_announced）：理論上每台都知道全部；記全部送到的時間、截止時送到的比例
      B 分配   每台分到幾個任務（cbba_path 的 bundle）：理論上每台 tasks／台數 個、而且是自己身邊的；
               記每台分到的數量、和最近分配不同的任務數（不在理論執行者的路徑裡、或在兩台以上的路徑裡）。
               「理論執行者」＝離任務最近的那台，**不是 CBBA 保證的結果**：出價看邊際成本，順路的任務幾乎不多花，
               掉包時建立者先搶到、assign_hold 後鎖定，就會分給別台（2026-11-03 分析 20261103_163735_loss）
      覆蓋     每個任務都有一台拿著（收斂）、全部做完（實驗規劃的「任務完成覆蓋率 100%」；週計畫、實驗規劃
               都沒有「每台最少分到幾個」的標準，每台最少～最多只列出來）
      收斂     每個任務恰好在一台的路徑裡（不管對不對）；收斂時間從建立算起。「正確」另外要求是理論上的那台
      重複認領 同一個任務在這次試驗裡被兩台以上當成 current_task 拿過（真的派給兩台的 BT）
      5 s 收斂 收斂時間 ≤ 5 s 而且沒有重複認領（週計畫：30% 掉包 ≥ 95%）
      完成     持有者回報完成 → 全部撤下的時間；回報完成的任務之後沒有再出現在任何一台的 current_task
      頻寬     每個掉包率從第一次試驗到最後一次，各台 mesh 網卡收送的平均 kB/s（tc 只限制送出方向，
               所以實際的單向掉包率就是設定值）
    每次試驗從「三台都沒有任務、再等 idle_gap 秒」開始（掉包時確認完成要 1～3 s）；收尾把這次的任務全部做完
    （含排在路徑後面的）。開始時不乾淨的試驗不計；收不了尾時停止這個掉包率（2026-10-12 第一次量測：
    只收拾看得到持有者的任務，排在後面的留下來變成執行中、沒人回報，之後每一次都被它卡住）
    """

    def __init__(self, env, run, only, levels, trials, delay, jitter, idle_gap=3.0, tasks=9, window=10.0, fresh=True):
        super().__init__(env, run, only)
        self.fresh = fresh            # 每個掉包率重新啟動 cbba_node（2026-10-27）
        self.levels, self.trials, self.delay, self.jitter = levels, trials, delay, jitter
        self.idle_gap = idle_gap
        self.n_tasks = tasks
        self.window = window          # 每次試驗等收斂、送達的上限（s）
        self.tx_window = 10.0         # 頻寬的量測區間：任務注入後固定 10 s（2026-11-09）

    def params_tag(self):
        """--node-param 的設定（寫進 trials.csv、levels.csv，plot_loss.py 依它分成不同的線；空 = 預設值）"""
        return " ".join(f"{k}={v}" for k, v in getattr(self.env, "node_params", []))

    def busy(self):
        """目前拿著任務（current_task）或路徑裡還有任務的載具"""
        none = self.cfg["fields"]["no_task_id"]
        return sorted({u for u, (t, _, _) in self.run.assigned.items() if t not in (None, none)} |
                      {u for u, p in self.run.paths.items() if p})

    def layout(self, k):
        """第 k 次試驗的任務：[(理論執行者, 建立者, x, y, 任務種類)]。第 j 個任務放在 ids[j % n] 身邊 2.5 m（角度每次輪換），
        建立者是別台（輪換），所以每台理論上分到 tasks／n 個（除不盡時前幾台多一個）。
        空地混合（--dogs，2026-11-09，週計畫第 5 週「無人機火警＋地面巡檢點」）：無人機身邊的是空中任務（1）；狗身邊的
        輪流是火警（2，地面處置，建立者是無人機——無人機看到火才發布）和巡檢點（3，建立者照輪換）。
        能做這種任務的其他台都在 20 m 外，理論執行者 = 能力相符的最近那台"""
        ids, pos, uavs = self.env.ids, self.positions(), self.env.uav_ids
        n = len(ids)
        out = []
        for j in range(self.n_tasks):
            oi = j % n
            owner = ids[oi]
            creator = ids[(oi + 1 + (k + j // n) % (n - 1)) % n] if n > 1 else owner
            ttype = 1
            if self.env.is_dog(owner):
                ttype = 2 if (j // n + k) % 2 == 0 else 3
                if ttype == 2:
                    creator = uavs[(j + k) % len(uavs)]
            a = 2 * math.pi * (j // n) / max(1, math.ceil(self.n_tasks / n)) + 0.7 * k
            x, y = pos[owner]
            out.append((owner, creator, x + 2.5 * math.cos(a), y + 2.5 * math.sin(a), ttype))
        return out

    def restart_all(self):
        """三台 cbba_node 全部關掉再開（全新的 session、不記得之前的任務），等到都上線"""
        run, ids = self.run, self.env.ids
        for v in ids:
            self.stop_node(v)
        run.until(lambda: not any(self.env.running(self.env.uav(v), self.env.node_pattern(v))
                                  for v in ids), 10.0)
        for v in ids:                         # 舊節點的狀態不算數；新節點的 transient_local 話題會重新送來
            run.assigned.pop(v, None)
            run.paths.pop(v, None)
        for v in ids:
            self.start_node(v)
        run.until(lambda: all(v in run.assigned for v in ids), self.cfg["comm"]["warmup"] + 15)
        run.sleep(self.cfg["comm"]["warmup"])

    def all_closed(self, tasks):
        """這些任務每一台都確認結束了（都發了 task_closed）"""
        return all(t in self.run.closed.get(v, {}) for v in self.env.ids for t in tasks)

    def settle(self, timeout=30.0):
        """試驗隔離（2026-10-27 加強）：等到上一次的任務**全隊都確認結束**（每台的 task_closed），而且三台都沒有任務
        （current_task、路徑都空），再等 idle_gap 秒還是如此才算乾淨；回傳是否乾淨。
        原本只等 current_task 空 10 s：高掉包時完成確認要好幾秒，上一次的任務還開著就開始下一次，70% 時 19 次有 11 次不計"""
        prev = getattr(self, "prev_tasks", [])
        if self.run.until(lambda: self.all_closed(prev) and not self.busy(), timeout) is None:
            return False
        self.run.sleep(self.idle_gap)
        return self.all_closed(prev) and not self.busy()

    def close_all(self, tasks, timeout=30.0):
        """收尾：這次試驗的每個任務（含排在路徑後面、之後才輪到的）都等到有人拿到（current_task）就回報完成，
        直到每個任務都**全隊確認結束**（每台都發了 task_closed；2026-10-27 起，原本只看 current_task 撤下）。
        記錄每個任務：done_at 回報完成、released 執行那台撤下、closed 全隊確認。回傳做不完（時限內沒有全隊確認）的任務數"""
        run, ids = self.run, self.env.ids
        reported = {}                         # 任務 → 回報過的持有者
        self.done_at, self.released, self.closed_at = {}, {}, {}
        deadline = time.time() + timeout
        while True:
            run.read()
            for t in tasks:
                held = run.holders(t)
                for h in held:
                    if reported.get(t) != h:      # 換了持有者（失聯接手等）也要回報
                        ack = self.report(h, t, True)
                        reported[t] = h
                        # 回報時刻 = ros_bridge 實際發出 task_result 的時刻（ack 的 t，和事件同一個時鐘）。
                        # 2026-10-27 用迴圈開頭的時刻：比後面任務的派出還早，「完成後又派」84 次（實際 2 次）；
                        # 10-28 用 report() 回來的時刻：等 ack 期間 0% 時全隊已經確認完，「→全隊確認」變成 −0.02 s
                        self.done_at.setdefault(t, ack["t"] if ack and "t" in ack else time.time())
                if t in reported and not held:
                    self.released.setdefault(t, time.time())
                if t not in self.closed_at and all(t in run.closed.get(v, {}) for v in ids):
                    self.closed_at[t] = max(run.closed[v][t] for v in ids)
            now = time.time()
            if len(self.closed_at) == len(tasks):
                return 0
            if now >= deadline:
                return len(tasks) - len(self.closed_at)
            run.sleep(0.1)

    def trial(self, level, k):
        run, ids = self.run, self.env.ids
        clean = self.settle()                    # 上一次的任務還沒全部結束時，這一次不算進統計
        t0 = time.time()
        owner = {}                               # 任務 → 理論上的執行者
        ttype = {}                               # 任務 → 種類（1 空中、2 火警、3 巡檢）
        for o, creator, x, y, ty in self.layout(k):
            tid, _ = self.new_task(x, y, creator=creator, task_type=ty if self.env.dogs else None)
            owner[tid] = o
            ttype[tid] = ty
        tasks = list(owner)
        expect = {v: sum(o == v for o in owner.values()) for v in ids}

        def n_known(v):
            return sum(t in run.known.get(v, {}) for t in tasks)

        def all_known():
            return all(n_known(v) == len(tasks) for v in ids)

        def partition():                         # 每個任務恰好在一台的路徑裡
            return all(len(run.in_paths(t)) == 1 for t in tasks)

        def correct():
            return all(run.in_paths(t) == [owner[t]] for t in tasks)

        # 等到「全部送到而且分配正確」或 window 秒；途中記下第一次收斂、第一次正確的時刻
        t_conv = t_corr = None
        deadline = t0 + self.window
        while True:
            run.read()
            now = time.time()
            if t_conv is None and partition():
                t_conv = now
            if t_corr is None and correct():
                t_corr = now
            if (t_corr is not None and all_known()) or now >= deadline:
                break
            run.sleep(0.05)
        if t_conv is not None:
            run.sleep(1.0)                               # 收斂之後 1 s 再看最後的分配
            if not partition():                          # 又變了：不算收斂（之後還會再動）
                t_conv = t_corr = None
        # 頻寬（2026-11-09，週計畫第 5 週）：任務注入後固定 tx_window 秒（10 s）的送出位元組，每個掉包率用同樣的區間；
        # 區間結束前不收尾（不然收斂快的掉包率會把完成宣告算進來，不公平）
        run.sleep(max(0.0, t0 + self.tx_window + 0.3 - time.time()))
        run.read()
        tx0 = {v: run.tx_at(v, t0) for v in ids}
        tx1 = {v: run.tx_at(v, t0 + self.tx_window) for v in ids}
        tx_ok = [v for v in ids if tx0[v] is not None and tx1[v] is not None]
        tx_sum = [sum(tx1[v][k] - tx0[v][k] for v in tx_ok) for k in range(3)] if tx_ok else [None] * 3
        # A 送達：每台知道幾個、全部送到的時間（ros_bridge 收到 task_announced 的時刻）
        known = {v: n_known(v) for v in ids}
        last_known = [run.known[v][t] for v in ids for t in tasks if t in run.known.get(v, {})]
        known_s = max(last_known) - t0 if all_known() and last_known else None
        # B 分配：每台路徑裡有幾個這次的任務、幾個和最近分配不同
        got = {v: sum(t in run.paths.get(v, []) for t in tasks) for v in ids}
        wrong = sum(run.in_paths(t) != [owner[t]] for t in tasks)
        dup = sum(len(run.ever_held(t, t0)) > 1 for t in tasks)
        # 跨能力（2026-11-09）：空中任務出現在狗、地面任務出現在無人機的路徑或 current_task（應該永遠是 0）
        cross = sum(any(self.env.is_dog(u) == (ttype[t] == 1) for u in set(run.ever_held(t, t0)) | set(run.in_paths(t)))
                    for t in tasks)
        conv = t_conv - t0 if t_conv is not None else None
        corr = t_corr - t0 if t_corr is not None else None

        t1 = time.time()
        leftover = self.close_all(tasks, timeout=30.0 + 5.0 * len(tasks))   # 每台依序做完路徑上的幾個
        self.prev_tasks = tasks
        med = lambda xs: sorted(xs)[len(xs) // 2] if xs else None
        rel = [self.released[t] - self.done_at[t] for t in tasks if t in self.released and t in self.done_at]
        clo = [self.closed_at[t] - self.done_at[t] for t in tasks if t in self.closed_at and t in self.done_at]
        rel_s, close_s = med(rel), med(clo)
        batch_s = max(self.closed_at.values()) - t1 if len(self.closed_at) == len(tasks) else None
        # 完成後又派：回報完成之後，這個任務又出現在某台的 current_task（2026-10-27 修正：原本和量測當下的路徑比，
        # 完成後其他任務重新規劃、換台是正常的，被算成「又派」）
        reassigned = sum(any(task == t and tt > self.done_at[t] for (tt, _, task, _) in run.history)
                         for t in tasks if t in self.done_at)
        fmt_counts = lambda d: "/".join(str(d[v]) for v in ids)
        row = dict(loss=level, trial=k + 1, node_params=self.params_tag(), clean=int(clean), tasks=len(tasks),
                   known_s=round(known_s, 3) if known_s is not None else "",
                   known_pct=round(100.0 * sum(known.values()) / (len(tasks) * len(ids)), 1),
                   known_per_agent=fmt_counts(known),
                   conv_s=round(conv, 3) if conv is not None else "",
                   correct_s=round(corr, 3) if corr is not None else "",
                   expected_per_agent=fmt_counts(expect), got_per_agent=fmt_counts(got), wrong=wrong,
                   converged_5s=int(conv is not None and conv <= 5.0 and dup == 0),
                   correct_5s=int(corr is not None and corr <= 5.0 and dup == 0), duplicates=dup,
                   release_s=round(rel_s, 3) if rel_s is not None else "",
                   close_s=round(close_s, 3) if close_s is not None else "",
                   close_max_s=round(max(clo), 3) if clo else "",
                   batch_s=round(batch_s, 3) if batch_s is not None else "", reassigned_after_done=reassigned,
                   cross_type=cross, ground_tasks=sum(ty != 1 for ty in ttype.values()),
                   agents="/".join(self.env.name(v) for v in ids),
                   tx_window_s=self.tx_window, tx_vehicles=len(tx_ok),
                   tx_total_bytes=tx_sum[0] if tx_sum[0] is not None else "",
                   tx_retry_bytes=tx_sum[1] if tx_sum[1] is not None else "",
                   tx_relay_bytes=tx_sum[2] if tx_sum[2] is not None else "",
                   # 每台平均 TX（kB/s）＝ Σ 送出位元組 ／（台數 × 區間）
                   tx_kBps=round(tx_sum[0] / (len(tx_ok) * self.tx_window) / 1024, 3) if tx_ok else "",
                   retry_kBps=round(tx_sum[1] / (len(tx_ok) * self.tx_window) / 1024, 3) if tx_ok else "",     # *_per_agent 的順序（plot_tasks.py 的標籤）
                   leftover=leftover)
        mark = f"{conv:.2f} s" if conv is not None else "未收斂"
        print(f"    {level:>3}% #{k + 1:<3} 送達 " + (f"{known_s:.2f} s" if known_s is not None else
                                                    f"{row['known_pct']:.0f}%（{fmt_counts(known)}）") +
              f"　分配 {fmt_counts(got)}（理論 {fmt_counts(expect)}" + (f"，非最近 {wrong}" if wrong else "") + "）" +
              f"　收斂 {mark}　重複 {dup}　完成→撤下 " + (f"{rel_s:.2f} s" if rel_s is not None else "—") +
              "、→全隊確認 " + (f"{close_s:.2f} s" if close_s is not None else "—") +
              (f"　完成後又派 {reassigned}" if reassigned else "") + (f"　跨能力 {cross}" if cross else "") +
              ("" if clean else "　（開始時還有任務，不計）") + (f"　做不完 {leftover}" if leftover else ""))
        return row

    def run_all(self):
        env, run, res = self.env, self.run, self.res
        if not self.start_all():
            res.add("L0", "節點上線", "FAIL", "收不到各台的 assigned")
            return
        run.sleep(self.cfg["comm"]["warmup"])
        rows, levels = [], []
        crashed = []
        for li, level in enumerate(self.levels):
            if self.fresh and li > 0:
                # 試驗隔離（2026-10-27）：每個掉包率重新啟動三台 cbba_node，前一個掉包率的任務（已結束的會一直記著）
                # 不帶到下一個；量測只看這個掉包率自己的任務
                self.log_tag = f"_L{level}"
                self.restart_all()
            self.prev_tasks = []
            self.tc(level, self.delay, self.jitter) if level > 0 else self.tc(None)
            try:
                run.sleep(2.0)
                b0 = {v: env.mesh_bytes(v) for v in env.ids}
                tb = time.time()
                lv = []
                for k in range(self.trials):
                    lv.append(self.trial(level, k))
                    dead = [v for v in env.ids if not env.running(env.uav(v), self.env.node_pattern(v))]
                    if dead:                          # 節點當掉（2026-10-27 遇過 Segmentation fault）：看 node_uavN.log
                        crashed += [(level, k + 1, v) for v in dead]
                        print(f"    {level}% 第 {k + 1} 次之後 {[env.name(v) for v in dead]} 的 cbba_node 不在了"
                              f"（看 node_*.log），停止這個掉包率")
                        lv[-1]["leftover"] = max(lv[-1]["leftover"], 1)
                        break
                    if lv[-1]["leftover"]:            # 收不了尾：後面的試驗會被留下的任務干擾，這個掉包率停止
                        print(f"    {level}% 第 {k + 1} 次有 {lv[-1]['leftover']} 個任務做不完，停止這個掉包率")
                        break
                dt = time.time() - tb
                b1 = {v: env.mesh_bytes(v) for v in env.ids}
            finally:
                self.tc(None)
            rows += lv
            aborted = lv[-1]["leftover"] > 0
            n_all = len(lv)
            covered = sum(r["conv_s"] != "" for r in lv)          # 每個任務恰好一台拿著
            finished = sum(r["leftover"] == 0 for r in lv)        # 這次的任務全部做完、全隊確認
            lv = [r for r in lv if r["clean"] and not r["leftover"]]   # 開始時不乾淨、收不了尾的不計
            if not lv:
                res.add(f"L{level}", f"掉包 {level}%", "FAIL", "沒有可以統計的試驗（每次開始時都還有任務、或收不了尾）")
                break
            kbs = [(b1[v] - b0[v]) / dt / 1024 for v in env.ids if b0[v] is not None and b1[v] is not None]
            convs = sorted(r["conv_s"] for r in lv if r["conv_s"] != "")
            corrs = sorted(r["correct_s"] for r in lv if r["correct_s"] != "")
            knows = sorted(r["known_s"] for r in lv if r["known_s"] != "")
            rels = sorted(r["release_s"] for r in lv if r["release_s"] != "")
            clos = sorted(r["close_s"] for r in lv if r["close_s"] != "")
            txs = sorted(r["tx_kBps"] for r in lv if r.get("tx_kBps", "") != "")
            rts = sorted(r["retry_kBps"] for r in lv if r.get("retry_kBps", "") != "")
            q = lambda xs, p: xs[min(len(xs) - 1, int(p * len(xs)))] if xs else None
            # 每台平均分到幾個（理論值每次都一樣，取第一次的）
            got = [[int(c) for c in r["got_per_agent"].split("/")] for r in lv]
            got_mean = "/".join(f"{sum(g[i] for g in got) / len(got):.1f}" for i in range(len(env.ids)))
            got_min = "/".join(str(min(g[i] for g in got)) for i in range(len(env.ids)))
            got_max = "/".join(str(max(g[i] for g in got)) for i in range(len(env.ids)))
            uneven = sum(r["got_per_agent"] != r["expected_per_agent"] for r in lv)
            summary = dict(loss=level, node_params=self.params_tag(), trials=len(lv), tasks=self.n_tasks,
                           known_rate=sum(r["known_s"] != "" for r in lv) / len(lv),
                           known_median=q(knows, 0.5), known_p90=q(knows, 0.9),
                           known_pct_mean=round(sum(r["known_pct"] for r in lv) / len(lv), 1),
                           expected_per_agent=lv[0]["expected_per_agent"], got_per_agent_mean=got_mean,
                           got_per_agent_min=got_min, got_per_agent_max=got_max, uneven=uneven,
                           coverage_rate=covered / n_all if n_all else None,
                           finished_rate=finished / n_all if n_all else None,
                           correct_rate=sum(r["wrong"] == 0 for r in lv) / len(lv),
                           wrong=sum(r["wrong"] for r in lv), correct_5s=sum(r["correct_5s"] for r in lv) / len(lv),
                           correct_median=q(corrs, 0.5),
                           rate_5s=sum(r["converged_5s"] for r in lv) / len(lv),
                           conv_median=q(convs, 0.5), conv_p90=q(convs, 0.9), not_converged=len(lv) - len(convs),
                           duplicates=sum(r["duplicates"] for r in lv), release_median=q(rels, 0.5),
                           close_median=q(clos, 0.5), close_p90=q(clos, 0.9),
                           reassigned=sum(r["reassigned_after_done"] for r in lv),
                           cross_type=sum(r["cross_type"] for r in lv), dogs=len(env.dogs),
                           mesh_kBps=round(sum(kbs) / len(kbs), 2) if kbs else None,
                           tx_kBps_median=q(txs, 0.5), tx_kBps_p90=q(txs, 0.9),
                           retry_kBps_median=q(rts, 0.5), retry_kBps_p90=q(rts, 0.9))
            levels.append(summary)
            fmt = lambda v: f"{v:.2f} s" if v is not None else "—"
            ok = (level != 30 or summary["rate_5s"] >= 0.95) and not aborted and summary["cross_type"] == 0
            n_ground = lv[0].get("ground_tasks", 0)
            name = f"掉包 {level}%（延遲 {self.delay}±{self.jitter} ms，{self.n_tasks} 個任務" + \
                (f"（空中 {self.n_tasks - n_ground}＋地面 {n_ground}），{len(env.uav_ids)} 架無人機＋{len(env.dogs)} 隻狗"
                 if env.dogs else "") + \
                (f"，{self.params_tag()}" if self.params_tag() else "") + "）"
            res.add(f"L{level}A", name + "：送達（每台知道幾個任務）", "INFO",
                    f"全部送到 {summary['known_rate'] * 100:.0f}%（{len(lv)} 次）、平均送到 {summary['known_pct_mean']}%、"
                    f"送到時間中位數 {fmt(summary['known_median'])}、P90 {fmt(summary['known_p90'])}", **summary)
            res.add(f"L{level}B", name + "：分配（每台分到幾個任務）", "INFO",
                    f"理論 {summary['expected_per_agent']}、平均分到 {summary['got_per_agent_mean']}、"
                    f"每台最少 {summary['got_per_agent_min']}、最多 {summary['got_per_agent_max']}"
                    f"（和理論不同 {uneven}／{len(lv)} 次）；任務覆蓋：每個任務都有人拿 "
                    f"{summary['coverage_rate'] * 100:.0f}%、全部做完 {summary['finished_rate'] * 100:.0f}%（{n_all} 次）；"
                    f"和最近分配一致 {summary['correct_rate'] * 100:.0f}%（不一致的任務共 {summary['wrong']} 個；"
                    f"理論＝離任務最近的那台，不是 CBBA 保證的結果）、"
                    f"5 s 內一致 {summary['correct_5s'] * 100:.0f}%、一致時間中位數 {fmt(summary['correct_median'])}")
            res.add(f"L{level}", name + "：收斂（無衝突）",
                    ("PASS" if ok else "FAIL") if level == 30 or aborted or summary["cross_type"] else "INFO",
                    f"5 s 內收斂 {summary['rate_5s'] * 100:.0f}%（{len(lv)} 次；每個任務恰好一台拿著，不管分得對不對，對不對看 B）、"
                    f"收斂中位數 {fmt(summary['conv_median'])}、"
                    f"P90 {fmt(summary['conv_p90'])}、未收斂 {summary['not_converged']}、重複認領 {summary['duplicates']}、"
                    f"完成→本機撤下 {fmt(summary['release_median'])}、完成→全隊確認 {fmt(summary['close_median'])}"
                    f"（P90 {fmt(summary['close_p90'])}）、完成後又派 {summary['reassigned']}、"
                    f"網卡收＋送 {summary['mesh_kBps']} kB/s/台（參考；tc 丟掉的不算）" +
                    (f"、跨能力 {summary['cross_type']}（空中任務給狗、地面任務給無人機，應該是 0）" if env.dogs else "") +
                    (f"；每台 TX（CBBA，UDP 14600，注入後 10 s）中位數 {summary['tx_kBps_median']:.2f} kB/s、"
                     f"P90 {summary['tx_kBps_p90']:.2f}，其中重傳 {summary['retry_kBps_median']:.2f}／"
                     f"{summary['retry_kBps_p90']:.2f} kB/s" if summary["tx_kBps_median"] is not None else "") +
                    ("；**有任務做不完，提早停止**" if aborted else ""))
            if aborted and not self.fresh:        # 沒有重新啟動時，留下的任務會干擾之後的掉包率
                break
        if crashed:
            res.add("LX", "cbba_node 當掉", "FAIL", "、".join(f"{lv}% 第 {k} 次之後 uav{v}" for lv, k, v in crashed) +
                    "（node_uavN.log；--gdb 會印 backtrace）")
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
        # 巡檢高度、出生原點一定要給（場景決定，2026-10-24）；速度沒給時用 uav.speed（uav 這組照舊 5 m/s）
        stack_env = {"CBBA_ALTITUDE": f"{u['altitude']:g}", **({"CBBA_SPEED": f"{u['speed']:g}"} if "speed" in u else {}),
                     **(stack_env or {}), **env.spawn_env(), "EXECUTOR": env.executor}
        for vid in ids:
            env.exec(env.uav(vid), u["start"].format(id=vid, ns=env.ns(vid)), detach=True, env=stack_env,
                     log=f"{run.cout}/stack_uav{vid}.log")
        run.sleep(8.0)
        for i, vid in enumerate(ids):
            senv = {**env.spawn_env(), **({"HEADLESS": "1"} if headless else {})}
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
        if not t:
            self.dds_diag(ids)
        if t:
            run.sleep(5.0)    # 起飛後 CBBA 開始參與出價
        return t is not None

    def dds_diag(self, ids):
        """U1 失敗時的機內 DDS 診斷（2026-11-09：demo 時 sim 的 ros_bridge 只收到 uav3 的 PX4 話題，uav1、uav2 的 PX4
        明明在飛）。topic list 會列出 ros_bridge 自己訂閱的話題，分不出有沒有人在發，所以改看：
          ① sim 裡新開的程式看到 /uavN/fmu/out/vehicle_status_v1 有幾個發布者（Publisher count）
          ② sim 裡新開的程式能不能真的收到一筆（echo --once）
          ③ 那台自己的容器能不能收到一筆（PX4 → agent 這段正常嗎）
        ①② 有、ros_bridge 沒有 → 只有 ros_bridge 這個程式沒配對；②沒有、③有 → 那台到 sim 的 DDS 不通；③也沒有 → PX4／agent。
        寫到 dds_diag.txt"""
        env, lines = self.env, []
        got = sorted({e.get("uav") for e in self.run.events if e["kind"] == "pos"})
        lines.append(f"ros_bridge 收到位置的：{['uav%d' % v for v in got if v]}")
        for v in ids:
            topic = f"/uav{v}/fmu/out/vehicle_status_v1"
            info = env.exec(env.sim(), f"timeout 20 ros2 topic info --no-daemon --spin-time 8 {topic} 2>&1 | "
                                       "grep -E 'Publisher count|Subscription count'").stdout.strip().replace("\n", "、")
            echo = "timeout 15 ros2 topic echo --once --qos-reliability best_effort --field timestamp " + topic
            in_sim = env.exec(env.sim(), echo).stdout.strip()
            in_uav = env.exec(env.uav(v), echo).stdout.strip()
            lines.append(f"== uav{v}：sim 看到 {info or '（沒有資料）'}；sim 收到一筆：{'有' if in_sim else '沒有'}；"
                         f"uav{v} 容器自己收到一筆：{'有' if in_uav else '沒有'}")
        path = os.path.join(self.run.out, "dds_diag.txt")
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        print(f"  機內 DDS 診斷寫在 {path}")
        for line in lines:
            print("    " + line)

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
        # 下視相機看得到的半徑（2026-10-24 從固定 6.5 m 改成看高度）：水平視角 1.74 rad → 半寬 ≈ 1.19 × 高度；
        # 5 m 高時 6.5 m（照舊），yard 8 m 高時 9.6 m。用在「火只讓被指派的那台看到」
        self.view_keep = max(6.5, 1.2 * self.cfg["uav"]["altitude"])
        # 火和任務點（G、M、T）之間至少這麼遠（2026-11-05 使用者：任務點暫時不要和火重疊）：任務點離火 6 m（AirRecon 的
        # view_radius）內時，做那個任務的無人機會移到火點上空監控，和先發現、正在監控的那台搶同一點 → 避碰來回繞圈。
        # 預設 = 下視相機看得到的半徑 view_keep；demo.fire_task_gap 可以改
        self.fire_gap = d.get("fire_task_gap", self.view_keep)
        self.rng = random.Random(seed)
        # 設備（2026-11-05，config 的 demo.equipment）：隨機的任務點、火點照 demo.placement 的比例落在設備旁、設備頂上
        # （只有火）或空地；沒設定的場景照舊用 demo.keep_out 排除障礙物
        self.equip, self.covers = [], []
        eq = d.get("equipment")
        if eq:
            self.equip, self.covers = load_equipment(os.path.join(self.env.host_repo, eq["file"]),
                                                     eq.get("skip", []), eq.get("cover", []))
        self.spot_info = {}       # (x, y) → dict(where, equip, z)：挑點時的類別（印出來、記在 demo_plan.json、火的高度）
        # 狗也看得到火（2026-11-05，模擬狗的雲台相機；demo.dog_detect）：itest 照狗的位置判斷，狗的 BT 不用改
        self.dog_seen = {}        # (火名, 狗機號) → 第一次在範圍內的時刻
        self.dog_last = {}        # 狗機號 → (x, y)
        self.dog_ev_pos = 0
        self.dog_patrol_segments = []   # 巡邏中的狗的巡檢線（--dog bt 且場景有 dog_patrol 時；火的涵蓋範圍用）
        self.console_done = not interactive
        self.n_task = self.n_fire = self.n_manual = 0

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
        for kind in ("task", "fire", "manual"):
            # count N（2026-10-24）：同一筆展開成 N 個（名稱後面加 1～N），位置要 random；at_range [a, b]：開始後幾秒在這之間隨機
            entries = []
            for it in sc.get(kind + "s", []):
                if int(it.get("count", 1)) > 1:
                    base = it.get("name") or kind[0].upper()
                    entries += [{**it, "name": f"{base}{k + 1}", "count": 1} for k in range(int(it["count"]))]
                else:
                    entries.append(it)
            for i, it in enumerate(entries):
                if "at_range" in it:
                    it = {**it, "at": round(self.rng.uniform(*it["at_range"]), 1)}
                if it.get("random") or it.get("pair") or it.get("behind"):
                    x = y = None              # 插入時才在範圍內隨機（2026-10-20，使用者：隨機才測得到 CBBA）；對調時才知道
                elif "point" in it:
                    if it["point"] not in pts:
                        raise SystemExit(f"情境檔：{kind} 的 point {it['point']} 不在 {self.cfg['demo']['world']}_points.csv")
                    x, y, _ = pts[it["point"]]
                else:
                    x, y = float(it["x"]), float(it["y"])
                name = it.get("name") or it.get("point") or f"{kind[0].upper()}{i + 1}"
                # delay_range [a, b]：等待秒數在這之間隨機（種子固定就重現）
                delay = self.rng.uniform(*it["delay_range"]) if "delay_range" in it else float(it.get("delay", 0.0))
                items.append(dict(kind=kind, name=name, x=x, y=y, known_point="point" in it and not it.get("random"),
                                  duration=float(it.get("duration", 5.0)),
                                  at=float(it.get("at", 0.0)), after_arrival=it.get("after_arrival"),
                                  after_done=it.get("after_done"), after_manual=it.get("after_manual"),
                                  delay=round(delay, 1), creator=it.get("creator"),
                                  target=it.get("target"), fire=it.get("fire"),
                                  pair=it.get("pair"), min_dist=float(it.get("min_dist", 15.0)),
                                  wait=float(it.get("wait", 30.0)), behind=it.get("behind")))
        names = [it["name"] for it in items if it["kind"] == "task"]
        manuals = [it["name"] for it in items if it["kind"] == "manual"]
        for it in items:
            for k in ("after_arrival", "after_done"):
                if it[k] and it[k] not in names:
                    raise SystemExit(f"情境檔：{it['name']} 的 {k} = {it[k]}，沒有這個任務（任務：{', '.join(names)}）")
            if it["after_manual"] and it["after_manual"] not in manuals:
                raise SystemExit(f"情境檔：{it['name']} 的 after_manual = {it['after_manual']}，沒有這個人工指派")
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
                if it["after_manual"]:
                    m = next(m for m in items if m["kind"] == "manual" and m["name"] == it["after_manual"])
                    if not m.get("done"):
                        continue
                elif it["kind"] == "manual" and it["target"] == "busy" and (
                        now - t0 < it["at"] or not self.busy_uavs(plan)):
                    continue        # 要等到有一台正在做固定任務（G）
                elif it["after_arrival"]:
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
            if it["kind"] == "manual":
                (self.manual_pair_now if it.get("pair") else self.manual_now)(it, plan, t0, items)
            else:
                self.insert_now(it, plan, t0, items)

    def insert_now(self, it, plan, t0, items=()):
        """插入一個任務或起火點。任務點不是場景裡的點（自訂 x、y）時生成藍色平台（models/task_pad）"""
        env, d = self.env, self.cfg["demo"]
        now = time.time()
        it["done"] = True
        scen = {"CBBA_SCENARIO": d["world"]}
        if it.get("x") is None and it.get("behind"):
            xy = self.behind_spot(it, plan, items)
            if xy is None:
                if now - it.get("ready_t", now) < it.get("wait", 30.0):
                    it["done"] = False          # 等下一輪（還沒有懸停中的人工指派、或沒有閒著的無人機）
                    return
                self.reply(f"  {now - t0:5.1f} s  {it['name']}：等了 {it.get('wait', 30.0):g} s 找不到懸停的無人機後方的位置，"
                           f"改成隨機位置")
            else:
                it["x"], it["y"] = xy
        if it.get("x") is None:
            # 任務點同時起火：不放在別台現在的航線、位置附近（下視相機看得到的範圍 view_keep 內），否則別台路過先看到、
            # 停到火點上空監控，CBBA 又把同一點的任務派給另一台 → 兩台搶同一點、避碰來回晃（2026-11-05 SITL，最近 0.45 m；
            # 使用者：先在模擬避免，BT 之後再處理）
            routes = self.uav_routes(plan, items) if it["kind"] == "task" and it.get("fire") is True else ()
            xy = self.random_spot(plan, items, min_from_uavs=0.0, kind="fire" if it["kind"] == "fire" else "task",
                                  avoid_routes=routes)
            if xy is None and it["kind"] == "fire" and it["name"].startswith("F_"):
                self.reply(f"  {now - t0:5.1f} s  {it['name']}：找不到離任務點 ≥ {self.fire_gap:.1f} m 的位置，這次不起火")
                return
            if xy is None:
                it["done"] = False
                return
            it["x"], it["y"] = xy
        if it["kind"] == "task" and it.get("fire") is True:
            # 任務點同時起火（2026-10-24，情境檔 tasks 的 "fire": true；同互動的 tf）：接到這個任務的那台到點時看到火
            # 2026-11-05：任務點不要和火重疊（同一點時別台路過先看到、停在上面，兩台搶同一點繞圈）→ 另外一個隨機火，
            # 離所有任務點 ≥ fire_gap；火要靠移動中的無人機、狗看到才會被發現，所以只放在有人會經過的範圍（fire_in_coverage）
            self.n_fire += 1
            self.insert_now(dict(kind="fire", name=f"F_{it['name']}", x=None, y=None), plan, t0, items)
        where = self.where_text(it["x"], it["y"])
        z = (self.spot_info.get((it["x"], it["y"])) or {}).get("z", 0.0)   # 設備頂上：標記、火點生成在頂面
        if it["kind"] == "task":
            # T 的 "fire": true 2026-11-05 起是另一個隨機火點（不在同一點），T 照常放黃色圓盤
            if not it.get("known_point"):
                env.exec(env.sim(), f"patrol_scenario.sh mark-at {it['x']} {it['y']} {it['name']} T {z:g}", env=scen)
            creator = it["creator"] or env.ids[len(plan["patrol"]) % len(env.ids)]
            tid, _ = self.new_task(it["x"], it["y"], it["duration"], creator=creator)
            plan["patrol"].append(dict(point=it["name"], task_id=tid, x=it["x"], y=it["y"], t=now,
                                       duration=it["duration"], kind="G" if it.get("known_point") else "T",
                                       where=where, z=z))
            self.reply(f"  {now - t0:5.1f} s  插入任務 {it['name']} ({it['x']:g}, {it['y']:g}"
                       + (f"，{where}" if where else "") + f")、停留 {it['duration']:g} s")
        else:
            info = self.spot_info.get((it["x"], it["y"])) or {}
            e = next((q for q in self.equip if q["name"] == info.get("equip")), None) if info.get("where") == "top" else None
            if e:
                # 設備頂上起火（2026-11-05 使用者）：那個設備變紅（外殼大 2 cm）、tag 101 貼在頂面；設備旁、空地照舊是紅色圓盤
                cz = e["top"] - e["h"] / 2
                geo = (f"cyl {e['x']} {e['y']} {cz:.3f} {e['r'] + 0.02:.2f} {e['h'] + 0.02:.2f}" if "r" in e else
                       f"box {e['x']} {e['y']} {cz:.3f} {2 * e['hx'] + 0.04:.2f} {2 * e['hy'] + 0.04:.2f} {e['h'] + 0.02:.2f}")
                env.exec(env.sim(), f"patrol_scenario.sh fire-on {it['name']} {it['x']} {it['y']} {geo}", env=scen)
            else:
                env.exec(env.sim(), f"patrol_scenario.sh fire-at {it['x']} {it['y']} {it['name']} {z:g}", env=scen)
            plan["fires"].append(dict(name=it["name"], x=it["x"], y=it["y"], z=z, custom=True, spawned=now, where=where))
            self.reply(f"  {now - t0:5.1f} s  起火 {it['name']} ({it['x']:g}, {it['y']:g}" +
                       (f"，{where}" if where else "") + ")")

    def dog_watch(self, plan, t0):
        """狗也看得到火（2026-11-05，使用者：先模擬狗也能偵測火災點）：狗離還沒有人回報的火 ≤ demo.dog_detect.radius m、
        持續 dwell 秒 → 用狗的身分發布地面任務（/<ns>/new_task，task_id 建立者 = 狗的機號，內容和無人機的 fire_task 一樣），
        之後照常由 CBBA 在狗之間競標（地面任務只有狗能出價）；無人機的 BT 看 task_announced 知道已經有人報，不重複發布。
        只用狗的位置判斷（模擬雲台相機，不算遮蔽），狗的 BT（fake_dog.py、伯宇的 task_manager）都不用改。
        同時寫一筆 new_task 事件到 events.jsonl（uav = "dogN"），D2、2D 動畫、label_gazebo.py 都看得到是狗發現的"""
        cfg = self.cfg["demo"].get("dog_detect") or {}
        radius = cfg.get("radius", 0.0)
        if radius <= 0 or not plan.get("dog"):
            return
        ev = self.run.events
        for e in ev[self.dog_ev_pos:]:
            if e["kind"] == "dog":
                self.dog_last[e["dog"]] = (e["x"], e["y"])
        self.dog_ev_pos = len(ev)
        reported = [(e["msg"]["position"]["x"], e["msg"]["position"]["y"]) for e in self.ground_tasks(t0)]
        names = {d["id"]: f"dog{i + 1}" for i, d in enumerate(sorted(plan["dog"], key=lambda d: d["id"]))}
        ns_of = {d["id"]: d["ns"] for d in plan["dog"]}
        now = time.time()
        for f in plan["fires"]:
            if "x" not in f or not f.get("spawned") or f.get("dog_reported"):
                continue
            if any(math.hypot(f["x"] - a, f["y"] - b) <= 3.0 for a, b in reported):
                continue                    # 已經有人報了（無人機或別隻狗）
            for dog_id, (x, y) in sorted(self.dog_last.items()):
                key = (f["name"], dog_id)
                if math.hypot(x - f["x"], y - f["y"]) > radius:
                    self.dog_seen.pop(key, None)
                    continue
                first = self.dog_seen.setdefault(key, now)
                if now - first < cfg.get("dwell", 1.0):
                    continue
                self.seq += 1
                tid = dog_id * 65536 + self.seq
                fields = fill(self.cfg["messages"]["new_task"],
                              dict(task_id=tid, x=float(f["x"]), y=float(f["y"]), duration=float(cfg.get("duration", 20.0))))
                fields.update(type=self.cfg["fields"]["ground_type"], value=float(cfg.get("value", 100.0)),
                              deadline_sec=float(cfg.get("deadline", 120.0)))
                fields["position"]["z"] = float(f.get("z") or 0.0)
                spec = self.cfg["topics"]["new_task"]
                self.run.send("pub", fields=fields, topic=spec["name"].format(ns=ns_of[dog_id], id=dog_id),
                              type=spec["type"], reliable=spec.get("reliable", True),
                              transient_local=spec.get("transient_local", False))
                msg = dict(task_id=tid, type=fields["type"],
                           position=dict(x=float(f["x"]), y=float(f["y"]), z=fields["position"]["z"]))
                with open(os.path.join(self.run.out, "events.jsonl"), "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(dict(kind="new_task", uav=names[dog_id], dog=dog_id, msg=msg, t=time.time(),
                                             source="dog_detect"), ensure_ascii=False) + "\n")
                f["dog_reported"] = names[dog_id]
                self.reply(f"  {now - t0:5.1f} s  {names[dog_id]}（機號 {dog_id}）看到 {f['name']}（距離 "
                           f"{math.hypot(x - f['x'], y - f['y']):.1f} m）→ 發布地面任務 {tid:08X}")
                break

    def uav_routes(self, plan, items):
        """每台無人機現在的航線（目前位置 → 手上的任務或人工指派的點；沒有目的地時只有目前位置）"""
        by_id = {p["task_id"]: (p["x"], p["y"]) for p in plan["patrol"]}
        manual = {i.get("uav"): (i["x"], i["y"]) for i in items
                  if i["kind"] == "manual" and i.get("done") and i.get("x") is not None and not self.manual_finished(i)}
        out = []
        for v in self.env.ids:
            if v not in self.run.positions:
                continue
            here = self.run.positions[v][:2]
            goal = manual.get(v) or by_id.get(self.run.assigned.get(v, (0,))[0])
            out.append((here, goal or here))
        return out

    # ------------------------------------------------------------------ 情境檔的人工指派（2026-10-20）
    def busy_uavs(self, plan):
        """正在做固定任務（plan 的巡檢任務、還沒完成）的無人機 → {uav: 點名}"""
        f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]
        done = {e["msg"].get(f_id) for e in self.run.events if e["kind"] == "result" and e["msg"].get(f_ok) is True}
        out = {}
        for p in plan["patrol"]:
            if p["task_id"] in done:
                continue
            for v in self.run.holders(p["task_id"]):
                if v in self.run.positions:
                    out[v] = p["point"]
        return out

    def placement_spot(self, kind):
        """照 demo.placement[kind]（沒有時用 placement.default）的比例抽一個候選點（2026-11-05）：kind = "fire" 或 "task"。
        回傳 (x, y, info)；info = dict(where = beside／top／open, equip = 設備名稱, z = 火的高度（設備頂上時是頂面）)。
        beside：隨機一個設備，外緣往外 beside_gap m；top：設備頂面往內縮 0.6 m（tag 1 m 見方要整個貼得上去）；
        open：範圍內隨機、離每個設備 ≥ open_clear m（和「設備旁」分得開）"""
        pl = self.cfg["demo"].get("placement", {})
        weights = pl.get(kind) or pl.get("default") or {"open": 1.0}
        cats = [c for c in ("beside", "top", "open") if weights.get(c, 0) > 0 and (c == "open" or self.equip)]
        cat = self.rng.choices(cats, weights=[weights[c] for c in cats])[0]
        x0, x1, y0, y1 = self.area
        if cat == "open":
            clear = pl.get("open_clear", 2.0)
            for _ in range(200):
                x, y = self.rng.uniform(x0, x1), self.rng.uniform(y0, y1)
                if not any(in_footprint(e, x, y, clear) for e in self.equip):
                    return round(x, 1), round(y, 1), dict(where="open", equip="", z=0.0)
            return None
        e = self.rng.choice(self.equip)
        if cat == "top":
            if "r" in e:
                a, r = self.rng.uniform(0, 2 * math.pi), (max(0.0, e["r"] - 0.8)) * math.sqrt(self.rng.random())
                x, y = e["x"] + r * math.cos(a), e["y"] + r * math.sin(a)
            else:
                x = e["x"] + self.rng.uniform(-1, 1) * max(0.0, e["hx"] - 0.6)
                y = e["y"] + self.rng.uniform(-1, 1) * max(0.0, e["hy"] - 0.6)
            return round(x, 1), round(y, 1), dict(where="top", equip=e["name"], z=round(e["top"], 2))   # 標記、火點的高度
        g = self.rng.uniform(*pl.get("beside_gap", [0.5, 2.0]))
        if "r" in e:
            a = self.rng.uniform(0, 2 * math.pi)
            x, y = e["x"] + (e["r"] + g) * math.cos(a), e["y"] + (e["r"] + g) * math.sin(a)
        else:   # 隨機一邊，沿著那一邊均勻
            side = self.rng.randrange(4)
            t = self.rng.uniform(-1, 1)
            if side < 2:
                x, y = e["x"] + (1 if side == 0 else -1) * (e["hx"] + g), e["y"] + t * (e["hy"] + g)
            else:
                x, y = e["x"] + t * (e["hx"] + g), e["y"] + (1 if side == 2 else -1) * (e["hy"] + g)
        return round(x, 1), round(y, 1), dict(where="beside", equip=e["name"], z=0.0)

    def where_text(self, x, y):
        """挑點時記下的類別 → 「TB3 旁」「tank_1 頂上」「空地」；不是 placement 挑的點回傳空字串"""
        info = self.spot_info.get((x, y))
        if not info:
            return ""
        return {"beside": f"{info['equip']} 旁", "top": f"{info['equip']} 頂上", "open": "空地"}[info["where"]]

    def random_spot(self, plan, items, min_from_uavs=0.0, near=None, keep_away=(), kind="task", avoid_routes=(),
                    fire_gap=True, exempt=None):
        """範圍內隨機一點：離場景的點、已插入的任務／火點、起飛點都 ≥ min_gap，離每台現在的位置 ≥ min_from_uavs，
        離 keep_away 的點 ≥ view_keep（下視相機看得到的範圍：火不要被別台順路看到）、不在障礙物範圍（demo.keep_out）內。
        near = (x, y, r1, r2)：改成在那一點周圍 r1～r2 m 內隨機。找不到回傳 None"""
        x0, x1, y0, y1 = self.area
        use_placement = near is None and (self.equip or self.cfg["demo"].get("placement"))
        taken = [(p["x"], p["y"]) for p in plan["points"].values()] + [(p["x"], p["y"]) for p in plan["patrol"]] + \
            [(f["x"], f["y"]) for f in plan["fires"] if "x" in f] + [self.env.home(v) for v in self.env.ids] + \
            [(it["x"], it["y"]) for it in items if it.get("x") is not None and it.get("done")]
        uavs = [self.run.positions[v][:2] for v in self.env.ids if v in self.run.positions]
        # 火 ↔ 任務點保持 fire_gap（2026-11-05）：挑火時避開所有任務點，挑任務點時避開所有火；M 和 M 的火不檢查
        # （fire_gap=False，照原本的做法）
        task_pts = [(p["x"], p["y"]) for p in plan["patrol"]] + \
            [(m["x"], m["y"]) for m in (plan.get("manual") or []) if not m.get("clear")] + \
            [(i["x"], i["y"]) for i in items if i.get("x") is not None and i["kind"] in ("task", "manual")]
        fire_pts = [(f["x"], f["y"]) for f in plan["fires"] if "x" in f]
        apart = (task_pts if kind == "fire" else fire_pts) if fire_gap else []   # M 的火不檢查（照原本的做法）
        if exempt is not None:      # 那個任務自己不算
            apart = [q for q in apart if math.hypot(q[0] - exempt[0], q[1] - exempt[1]) > 0.05]
        # 火只放在有人會經過的範圍（2026-11-05，demo.fire_in_coverage）：狗的巡檢線 dog_detect.radius 內、
        # 或無人機現在航線的下視相機範圍內。M 的火（fire_gap=False）照原本的位置規則，不檢查
        coverage = []
        if kind == "fire" and fire_gap and near is None and self.cfg["demo"].get("fire_in_coverage"):
            r_dog = (self.cfg["demo"].get("dog_detect") or {}).get("radius", 0.0)
            coverage = [(a, b, r_dog) for a, b in self.dog_patrol_segments] if r_dog > 0 else []
            coverage += [(a, b, self.view_keep) for a, b in self.uav_routes(plan, items) if a != b]
        for _ in range(1000):
            info = None
            if near:
                a, r = self.rng.uniform(0, 2 * math.pi), self.rng.uniform(near[2], near[3])
                x, y = round(near[0] + r * math.cos(a), 1), round(near[1] + r * math.sin(a), 1)
                gap = 1.0     # 火在 M 旁：只要不疊在別的點上
            elif use_placement:
                got = self.placement_spot(kind)
                if got is None:
                    continue
                x, y, info = got
                gap = self.min_gap
            else:
                x, y = round(self.rng.uniform(x0, x1), 1), round(self.rng.uniform(y0, y1), 1)
                gap = self.min_gap
            on_top = info is not None and info["where"] == "top"
            if not (x0 <= x <= x1 and y0 <= y <= y1) or (self.blocked(x, y) and not on_top) or \
                    any(in_footprint(c, x, y, 0.5) for c in self.covers):
                continue
            if all(math.hypot(x - a, y - b) >= gap for a, b in taken) and \
                    all(math.hypot(x - a, y - b) >= min_from_uavs for a, b in uavs) and \
                    all(math.hypot(x - a, y - b) >= self.view_keep for a, b in keep_away) and \
                    all(seg_dist(x, y, a, b) >= self.view_keep for a, b in avoid_routes) and \
                    all(math.hypot(x - a, y - b) >= self.fire_gap for a, b in apart) and \
                    (not coverage or any(seg_dist(x, y, a, b) <= r for a, b, r in coverage)):
                if info:
                    self.spot_info[(x, y)] = info
                return x, y
        return None

    def blocked(self, x, y):
        """(x, y) 不能放點：有 demo.equipment 時是設備本體（往外 0.3 m）或遮蔽物（鋼構棚頂）底下（2026-11-05：使用者
        「點不在設備本體裡面、不在鋼構棚底下」）；沒有時照舊看 config 的 demo.keep_out（[x0, x1, y0, y1]）"""
        if self.equip:
            return any(in_footprint(e, x, y, 0.3) for e in self.equip) or \
                any(in_footprint(c, x, y, 0.5) for c in self.covers)
        return any(a <= x <= b and c <= y <= d for a, b, c, d in self.cfg["demo"].get("keep_out", []))

    def manual_now(self, it, plan, t0, items):
        """情境檔的人工指派 M（2026-10-20，使用者的新測試流程 2-2）：
        a. target "busy"：從正在做固定任務（G）的無人機隨機挑一台；M 的位置隨機（離每台 ≥ 8 m，要飛一段）→ 那台交還 G、去 M
        b. fire：同時在那台飛往 M 的路上（線段 route 比例處）或 M 旁（near m 內）隨機起火，離其他無人機、其他任務點 ≥ 6.5 m，
           只有它會看到（≥ view_keep）→ 照火警 BT 發布地面任務、監控到狗處理完，M 延遲（BT 印「人工指派 … 延遲」）"""
        env, d = self.env, self.cfg["demo"]
        now = time.time()
        busy = self.busy_uavs(plan)
        if it["target"] == "busy" and not busy:
            it.pop("ready_t", None)      # 等的時候 G 都做完了：等下一台開始做 G
            return
        if it["target"] == "busy":
            v = self.rng.choice(sorted(busy))
        else:
            v = int(str(it["target"]).replace("uav", ""))
        if v not in self.run.positions:
            return
        px, py = self.run.positions[v][:2]
        others = [self.run.positions[u][:2] for u in env.ids if u != v and u in self.run.positions]
        open_pts = [(p["x"], p["y"]) for p in plan["patrol"] if p["point"] != busy.get(v)]
        if it["x"] is None:
            xy = None
            for _ in range(50):
                xy = self.random_spot(plan, items, min_from_uavs=7.0, fire_gap=False)
                if xy and math.hypot(xy[0] - px, xy[1] - py) >= 8.0:
                    break
                xy = None
            if xy is None:
                return      # 下一輪再試
            it["x"], it["y"] = xy
        mx, my = it["x"], it["y"]
        fire = None
        fc = it.get("fire")
        if fc:
            route = fc.get("route", [0.4, 0.8])
            near = fc.get("near", 3.0)
            keep = others + open_pts + [env.home(u) for u in env.ids]
            # M 的火照原本的做法（2026-11-05 使用者）：放在 M 的路上或 M 旁，不檢查和任務點的距離 fire_gap
            # （同一台先處理火、再做 M；fire_gap 只用在隨機插入的任務 T 和火之間）
            use_route = self.rng.random() < fc.get("route_prob", 0.5) and math.hypot(mx - px, my - py) >= 8.0
            if use_route:
                L = math.hypot(mx - px, my - py)
                for _ in range(100):
                    k, off = self.rng.uniform(*route), self.rng.uniform(-0.5, 0.5)
                    fx = round(px + (mx - px) * k - (my - py) / L * off, 2)
                    fy = round(py + (my - py) * k + (mx - px) / L * off, 2)
                    if not self.blocked(fx, fy) and all(math.hypot(fx - a, fy - b) >= self.view_keep for a, b in keep):
                        fire = (fx, fy, f"uav{v} 往 {it['name']} 的路上（{k:.0%}）")
                        break
            if fire is None:
                xy = self.random_spot(plan, items, near=(mx, my, 1.5, near), keep_away=keep, fire_gap=False)
                if xy:
                    fire = (xy[0], xy[1], f"{it['name']} 旁 {math.hypot(xy[0] - mx, xy[1] - my):.1f} m")
        it["done"] = True
        it["uav"], it["from"], it["t"] = v, busy.get(v), now
        held = self.run.assigned.get(v, (0, 0, 0))[0]
        it["held"] = held
        if v in busy:
            self.reply(f"  {now - t0:5.1f} s  uav{v} 原本在做 {busy[v]}，人工指派後交還")
        self.manual_cmd([f"uav{v}", str(mx), str(my)], it["duration"], plan, t0, {})
        label = plan["manual"][-1]["label"] if plan.get("manual") else it["name"]
        it["label"] = label
        mz = (self.spot_info.get((mx, my)) or {}).get("z", 0.0)
        env.exec(env.sim(), f"patrol_scenario.sh mark-at {mx} {my} {label} M {mz:g}", env={"CBBA_SCENARIO": d["world"]})
        it["n"] = self.n_manual
        if plan.get("manual"):
            plan["manual"][-1].update(name=it["name"], held=held, held_point=busy.get(v), where=self.where_text(mx, my),
                                      z=mz)
            if not plan["manual"][-1]["ok"]:
                it["finished"] = True      # 被拒絕：不用等它做完（M1 會標 FAIL）
        if fire:
            fx, fy, where = fire
            fname = f"F_{it['name']}"
            env.exec(env.sim(), f"patrol_scenario.sh fire-at {fx} {fy} {fname}", env={"CBBA_SCENARIO": d["world"]})
            plan["fires"].append(dict(name=fname, x=fx, y=fy, custom=True, spawned=time.time(), manual=it["name"],
                                      where=where))
            it["fire_xy"] = (fx, fy)
            self.reply(f"  {time.time() - t0:5.1f} s  起火 {fname} ({fx:g}, {fy:g})：{where}")
        elif fc:
            self.reply(f"  找不到放火的位置（離其他無人機、任務點都 ≥ {self.view_keep:.1f} m），這次 M 沒有火")

    def manual_pair_now(self, it, plan, t0, items):
        """對調（2026-10-24，使用者：要看到航線交叉時的避碰）：兩台用人工指派互相飛到對方「現在」的位置，一定正面交會。
        pair：["uav1", "uav3"]，或 "auto" = 當下距離最遠、不在人工指派中的兩台；兩台要相距 ≥ min_dist（預設 15 m），
        否則等下一輪；等超過 wait 秒（預設 30 s）就略過這次（2026-10-24：排在大家都做完、回到起降板之後，最遠只有 4 m，
        一直等到時限，看起來像卡住）。被指派的那台手上有 CBBA 任務時照人工指派的規則交還（PREEMPTED_BY_MANUAL）"""
        env = self.env

        def give_up(why):
            if time.time() - it.get("ready_t", time.time()) < it["wait"]:
                return
            it.update(done=True, finished=True, skipped=why)
            self.reply(f"  {time.time() - t0:5.1f} s  對調 {it['name']} 略過：{why}（等了 {it['wait']:g} s）")
        # 只挑停著的（水平速度 < 0.3 m/s）：指派要 3～4 s 才送到，飛行中的這段時間會跑掉 10 m 以上，兩條航線就不交會了
        # （2026-10-24 SITL：記下位置 4 s 後才出發，最近 20 m，看不到 CBF）
        pos = {v: self.run.positions[v][:2] for v in env.ids
               if v in self.run.positions and self.uav_speed(v) < 0.3}
        in_manual = set()
        for i in items:
            if i["kind"] == "manual" and i.get("done") and not self.manual_finished(i):
                in_manual |= {i.get("uav"), i.get("uav_b")}
        if it["pair"] == "auto":
            free = sorted(v for v in pos if v not in in_manual)
            pairs = [(math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1]), a, b)
                     for k, a in enumerate(free) for b in free[k + 1:]]
            if not pairs:
                give_up("沒有兩台停著、不在人工指派中的無人機")
                return
            d, a, b = max(pairs)
        else:
            a, b = (int(str(x).replace("uav", "")) for x in it["pair"])
            if a not in pos or b not in pos or a in in_manual or b in in_manual:
                give_up(f"uav{a}、uav{b} 沒有位置或在人工指派中")
                return
            d = math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
        if d < it["min_dist"]:
            give_up(f"最遠的兩台只相距 {d:.1f} m（< {it['min_dist']:g} m）")
            return
        (ax, ay), (bx, by) = pos[a], pos[b]
        now = time.time()
        it.update(done=True, uav=a, uav_b=b, t=now, x=round(bx, 1), y=round(by, 1), xb=round(ax, 1), yb=round(ay, 1),
                  dist=d)
        jobs = [self.manual_prepare([f"uav{a}", str(it["x"]), str(it["y"])], it["duration"], plan, {}),
                self.manual_prepare([f"uav{b}", str(it["xb"]), str(it["yb"])], it["duration"], plan, {})]
        self.reply(f"  {now - t0:5.1f} s  對調 {it['name']}：uav{a} ↔ uav{b}（{jobs[0]['label']}、{jobs[1]['label']}，"
                   f"相距 {d:.1f} m，互相飛到對方的位置、同時出發，正面交會）")
        # 兩個指派同時送（一台一台送時第二台晚 3～4 s 出發，航線就錯開了）
        threads = [threading.Thread(target=self.manual_send, args=(j,)) for j in jobs]
        for th in threads:
            th.start()
        for th in threads:
            th.join()
        for j, key in zip(jobs, ("n", "n_b")):
            self.manual_record(j, plan, t0)
            it[key] = j["n"]
            plan["manual"][-1].update(name=j["label"], pair=it["name"])
            env.exec(env.sim(), f"patrol_scenario.sh mark-at {j['x']} {j['y']} {j['label']}",
                     env={"CBBA_SCENARIO": self.cfg["demo"]["world"]})

    def behind_spot(self, it, plan, items):
        """「behind": "manual"（2026-10-24，使用者：找會讓路徑交叉的任務點，不用對調）：任務放在「正在人工指派點懸停」的
        那台 H 後方——閒著的 I 往 H 的延長線上、H 後方 6～10 m（左右偏 0～1 m）。CBBA 的出價看距離，兩台在飛行中交叉的
        分配幾乎不會出現（交叉的話對調一定更近），所以改成讓 I 的航線經過不會讓開的 H（人工指派中、不參與 CBBA，不會把任務
        搶走；任務懸停、路權比較高）→ I 要用 VO 繞過 H。條件：I 到 T 的距離 < 0.8 × 其他每一台（閒著的用目前位置；正在做任務的
        用它目前任務點的位置——CBBA 會把新任務排在它手上的任務後面，2026-10-24 SITL：只比閒著的，T1 被做 G2 的 uav1 接走、
        沒經過 H），在範圍內、不在障礙物裡、離其他任務點和火 ≥ min_gap。回傳 (x, y)，記下 it["via"] = (H, I)；找不到回傳 None"""
        pos = {v: self.run.positions[v][:2] for v in self.env.ids if v in self.run.positions}
        hovering = [i.get("uav") for i in items if i["kind"] == "manual" and i.get("done") and not i.get("pair")
                    and not self.manual_finished(i)]
        hovering = [v for v in hovering if v in pos and self.uav_speed(v) < 0.3]
        busy_manual = set(hovering) | {i.get("uav") for i in items if i["kind"] == "manual" and i.get("done")
                                       and not self.manual_finished(i)}
        idle = [v for v in pos if v not in busy_manual and self.run.assigned.get(v, (0,))[0] in (0, None)]
        # 其他可能接 T 的：不在人工指派中的每一台；正在做任務的用任務點的位置（CBBA 從那裡算新任務的距離）
        task_xy = {p["task_id"]: (p["x"], p["y"]) for p in plan["patrol"]}
        start = {}
        for v in pos:
            if v in busy_manual:
                continue
            tid = self.run.assigned.get(v, (0,))[0]
            start[v] = task_xy.get(tid, pos[v]) if tid not in (0, None) else pos[v]
        x0, x1, y0, y1 = self.area
        taken = [(p["x"], p["y"]) for p in plan["patrol"]] + [(f["x"], f["y"]) for f in plan["fires"] if "x" in f]
        for h in hovering:
            for i in sorted(idle, key=lambda v: -math.hypot(pos[v][0] - pos[h][0], pos[v][1] - pos[h][1])):
                hx, hy = pos[h]
                ix, iy = pos[i]
                L = math.hypot(hx - ix, hy - iy)
                if L < 8.0:
                    continue                    # 太近：I 一出發就在 H 旁邊，看不出繞行
                ux, uy = (hx - ix) / L, (hy - iy) / L
                for dist in (8.0, 6.0, 10.0):
                    for lat in (0.0, 0.5, -0.5, 1.0, -1.0):
                        x, y = round(hx + ux * dist - uy * lat, 1), round(hy + uy * dist + ux * lat, 1)
                        if not (x0 <= x <= x1 and y0 <= y <= y1) or self.blocked(x, y):
                            continue
                        if any(math.hypot(x - a, y - b) < self.min_gap for a, b in taken):
                            continue
                        di = math.hypot(x - ix, y - iy)
                        if any(math.hypot(x - start[j][0], y - start[j][1]) * 0.8 <= di for j in start if j != i):
                            continue            # 別台（閒著的或做完手上任務後）比較近，CBBA 會給它
                        it["via"] = (h, i)
                        return x, y
        return None

    def uav_speed(self, v):
        """uav v 最近的水平速度（events 裡最後兩筆位置）"""
        last = []
        for e in reversed(self.run.events):
            if e["kind"] == "pos" and e["uav"] == v:
                last.append(e)
                if len(last) == 2:
                    break
        if len(last) < 2 or last[0]["t"] - last[1]["t"] <= 0:
            return math.inf
        return math.hypot(last[0]["x"] - last[1]["x"], last[0]["y"] - last[1]["y"]) / (last[0]["t"] - last[1]["t"])

    def pair_report(self, it, t0):
        """對調的紀錄：兩台之間的最近距離（events 的位置）、BT 記到的避碰（誰讓、往哪邊）"""
        a, b = it["uav"], it["uav_b"]
        track = {a: [], b: []}
        for e in self.run.events:
            if e["kind"] == "pos" and e["uav"] in track and e["t"] >= it["t"]:
                track[e["uav"]].append((e["t"], e["x"], e["y"]))
        best = None
        j = 0
        tb = track[b]
        for t, x, y in track[a]:
            while j + 1 < len(tb) and abs(tb[j + 1][0] - t) <= abs(tb[j][0] - t):
                j += 1
            if tb and abs(tb[j][0] - t) <= 0.3:
                d = math.hypot(x - tb[j][1], y - tb[j][2])
                if best is None or d < best[0]:
                    best = (d, t)
        notes = []
        for v in (a, b):
            path = os.path.join(self.run.out, f"stack_uav{v}.log")
            if not os.path.exists(path):
                continue
            for line in open(path, encoding="utf-8", errors="replace"):
                m = re.search(r"\[(\d+\.\d+)\] \[[^\]]+\]: (避碰：(預測|和).*|警告：避碰.*)", line)
                if m and float(m.group(1)) >= it["t"] and (f"uav{b if v == a else a}" in m.group(2) or "警告" in m.group(2)):
                    notes.append(f"uav{v} {float(m.group(1)) - t0:.1f} s {m.group(2)}")
        return best, notes

    def manual_log(self, v):
        """uav v 的 BT log 裡跟人工指派有關的行：[(牆上時間, 內容)]"""
        out = []
        path = os.path.join(self.run.out, f"stack_uav{v}.log")
        if not os.path.exists(path):
            return out
        for line in open(path, encoding="utf-8", errors="replace"):
            if "人工指派" not in line and "平板任務" not in line and "地面站" not in line:
                continue
            m = re.search(r"\[(\d+\.\d+)\] \[[^\]]+\]: (.*)", line)
            if m:
                out.append((float(m.group(1)), m.group(2).strip()))
        return out

    def manual_finished(self, it):
        """M 做完了沒（BT 的「平板任務 N 完成」）；log 每 2 s 才讀一次"""
        if it.get("finished"):
            return True
        if not it.get("done") or "n" not in it:
            return False
        if time.time() - it.get("checked", 0.0) >= 2.0:
            it["checked"] = time.time()
            jobs = [(it["uav"], it["n"])] + ([(it["uav_b"], it["n_b"])] if it.get("pair") and "n_b" in it else [])
            it["finished"] = all(any(f"平板任務 {n} 完成" in text for _, text in self.manual_log(v)) for v, n in jobs)
        return bool(it.get("finished"))

    def ground_station_watch(self, plan, t0):
        """地面站／平板的即時通知（2026-10-20 使用者：人工指派延遲、交還這些現象要在 terminal 警告使用者，不是錯誤）：
        - 被人工指派的無人機的 BT log（stack_uavN.log）裡「人工指派／地面站／平板任務」的行（延遲、繼續、完成）
        - task_result 的 PREEMPTED_BY_MANUAL（交還 CBBA 任務）→ 之後別台拿到那個任務時印「改由 uavK 接手」
        --interactive 時也寫給 insert.py 的 terminal。每 0.5 s 看一次"""
        now = time.time()
        if now - getattr(self, "_gs_t", 0.0) < 0.5:
            return
        self._gs_t = now
        off = self.__dict__.setdefault("_gs_off", {})
        handover = self.__dict__.setdefault("_gs_handover", {})    # task_id → 交還的那台
        names = {p["task_id"]: p["point"] for p in plan["patrol"]}
        f_id = self.cfg["fields"]["task_id"]
        events = self.run.events
        for e in events[self.__dict__.get("_gs_ev", 0):]:
            tid = e.get("msg", {}).get(f_id) if isinstance(e.get("msg"), dict) else None
            if e["kind"] == "result" and "PREEMPTED_BY_MANUAL" in (e["msg"].get("detail") or ""):
                handover[tid] = e["uav"]
                self.reply(f"  ⚠ {e['t'] - t0:5.1f} s  地面站通知：uav{e['uav']} 被人工指派，交還 "
                           f"{names.get(tid, f'{tid:08X}')}，重新競標")
            elif e["kind"] == "assigned" and tid in handover and e["uav"] != handover[tid]:
                self.reply(f"  ℹ {e['t'] - t0:5.1f} s  地面站通知：{names.get(tid, f'{tid:08X}')} 改由 uav{e['uav']} 接手")
                del handover[tid]
        self._gs_ev = len(events)
        for v in sorted({int(m["target"][3:]) for m in plan.get("manual", [])
                         if m.get("ok") and str(m.get("target", "")).startswith("uav")}):
            path = os.path.join(self.run.out, f"stack_uav{v}.log")
            if not os.path.exists(path):
                continue
            with open(path, encoding="utf-8", errors="replace") as f:
                f.seek(off.get(v, 0))
                lines = f.readlines()
                off[v] = f.tell()
            for line in lines:
                if ("人工指派" not in line and "平板任務" not in line and "地面站" not in line) or "交還 CBBA" in line:
                    continue      # 交還由上面的 task_result 印
                m = re.search(r"\[(\d+\.\d+)\] \[[^\]]+\]: (.*)", line)
                if not m:
                    continue
                text = m.group(2).strip()
                mark = "⚠" if text.startswith("警告") else "ℹ"
                self.reply(f"  {mark} {float(m.group(1)) - t0:5.1f} s  地面站通知 uav{v}：{text}")

    def manual_done_t(self, m):
        """人工指派 m（plan["manual"] 的一筆）做完的時刻（牆上時間）；還沒做完、被拒絕、解除 → None。
        無人機：BT log「平板任務 N 完成」；狗（2026-11-07）：robot_state 的 mode 在指派後變成 MANUAL、又回到 AUTO
        （假狗、伯宇的 BT 都是做完自動解除接管）"""
        label, target = str(m.get("label", "")), str(m.get("target", ""))
        if m.get("clear") or not m.get("ok") or not label[1:].isdigit():
            return None
        if target.startswith("uav"):
            return next((t for t, x in self.manual_log(int(target[3:]))
                         if f"平板任務 {label[1:]} 完成" in x and t >= m["t"]), None)
        if target.startswith("dog") and target[3:].isdigit():
            dog_id, manual_seen = int(target[3:]), False
            for e in self.run.events:
                if e["kind"] != "dog" or e.get("dog") != dog_id or e["t"] < m["t"] or "mode" not in e:
                    continue
                if e["mode"] == 1:
                    manual_seen = True
                elif e["mode"] == 0 and manual_seen:
                    return e["t"]
        return None

    def marker_watch(self, plan, t0, force=False):
        """任務做完時刪掉 Gazebo 的標記（2026-11-07 使用者：做完的任務在 Gazebo 上消失；2D 動畫的旗子變淺灰）：
        - G、T：task_result 成功 → G 刪場景的 pad_<點>、tag_<點>；T 刪 task_<名稱>
        - M（人工指派）：manual_done_t（無人機看 BT log、狗看 mode 回到 AUTO）→ task_<M 編號>
        - 火：3 m 內的地面任務 task_closed（status DONE）→ fire_<名稱>、tag_fire_<名稱>
        gazebo_labeled.mp4 的標籤照舊變灰、打勾（label_gazebo.py）。每 1 s 看一次"""
        now = time.time()
        if not force and now - getattr(self, "_mk_t", 0.0) < 1.0:
            return
        self._mk_t = now
        gone = self.__dict__.setdefault("_mk_gone", set())
        f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]
        ev = self.run.events
        ok_ids = {e["msg"].get(f_id) for e in ev if e["kind"] == "result" and e["msg"].get(f_ok) is True and e["t"] >= t0}
        closed_xy = []
        ground = {e["msg"].get(f_id): e["msg"]["position"] for e in self.ground_tasks(t0)}
        for e in ev:
            if e["kind"] == "closed" and e["msg"].get("status") == 1 and e["msg"].get(f_id) in ground:
                p = ground[e["msg"][f_id]]
                closed_xy.append((p["x"], p["y"]))
        remove = []
        for p in plan["patrol"]:
            key = ("task", p["task_id"])
            if key in gone or p["task_id"] not in ok_ids:
                continue
            gone.add(key)
            remove += [f"task_{p['point']}"] if p.get("kind") == "T" else [f"pad_{p['point']}", f"tag_{p['point']}"]
        for m in plan.get("manual") or []:
            key = ("manual", str(m.get("label", "")))
            if key in gone or self.manual_done_t(m) is None:
                continue
            gone.add(key)
            remove.append(f"task_{m['label']}")
        for f in plan["fires"]:
            key = ("fire", f["name"])
            if key in gone or not f.get("spawned") or "x" not in f:
                continue
            if any(math.hypot(f["x"] - x, f["y"] - y) <= 3.0 for x, y in closed_xy):
                gone.add(key)
                remove += [f"fire_{f['name']}", f"tag_fire_{f['name']}"]
        # 狗的日常巡檢點（2026-11-07 使用者）：做完（雲台檢查完、往下一點走）刪掉，下一圈狗再出發去它時重新生成；
        # 判斷照狗 BT 的 log（dog_patrol.py，和 2D 動畫、gazebo_labeled.mp4 同一套）
        spawn = []
        p_state = self.__dict__.setdefault("_mk_patrol", {})
        for u in plan.get("dog") or []:
            pts = u.get("patrol") or []
            log = os.path.join(self.run.out, f"dog_bt_{u['ns']}.log")
            if not pts or not os.path.exists(log):
                continue
            with open(log, encoding="utf-8", errors="replace") as fh:
                ivs = dog_patrol.done_intervals(dog_patrol.parse(fh), [q["name"] for q in pts])
            for q in pts:
                is_done = dog_patrol.is_done(ivs.get(q["name"], []), now)
                if p_state.get(q["name"], False) == is_done:
                    continue
                p_state[q["name"]] = is_done
                if is_done:
                    remove.append(f"task_{q['name']}")
                else:
                    spawn.append(f"patrol_scenario.sh mark-at {q['x']:g} {q['y']:g} {q['name']} P")
        if spawn:
            self.env.exec(self.env.sim(), " ; ".join(spawn), detach=True, env={"CBBA_SCENARIO": self.cfg["demo"]["world"]})
        if remove:
            # 背景執行（gz model --list＋remove 要 1～2 s，不擋主迴圈）
            self.env.exec(self.env.sim(), "patrol_scenario.sh remove " + " ".join(remove), detach=True,
                          env={"CBBA_SCENARIO": self.cfg["demo"]["world"]})
            self.reply(f"  {now - t0:5.1f} s  做完，Gazebo 刪掉標記：{' '.join(remove)}")
        if spawn:
            self.reply(f"  {now - t0:5.1f} s  狗下一圈出發，Gazebo 重新生成巡檢點：" +
                       " ".join(c.split()[4] for c in spawn))

    # ------------------------------------------------------------------ 100% 斷網（情境檔的 blackout，2026-11-09）
    def blackout_cfg(self):
        return (self.scenario or {}).get("blackout")

    def blackout_open(self):
        """斷網還沒開始或還在斷：主迴圈不能結束"""
        b = getattr(self, "_bo", None)
        return bool(self.blackout_cfg()) and (b is None or "t1" not in b)

    def holders_now(self, plan):
        """現在手上有 CBBA 任務的載具：{名稱: task_id}（無人機看 current_task、狗看 dog_assigned 的最後一筆）"""
        none = self.cfg["fields"]["no_task_id"]
        out = {f"uav{v}": t for v, (t, _, _) in self.run.assigned.items() if t not in (None, none)}
        last = {}
        for e in self.run.events:
            if e["kind"] == "dog_assigned":
                last[e["dog"]] = e["msg"].get(self.cfg["fields"]["task_id"], 0)
        dog_name = {d["id"]: f"dog{i + 1}" for i, d in enumerate(sorted(plan.get("dog") or [], key=lambda d: d["id"]))}
        out.update({dog_name.get(d, f"dog{d}"): t for d, t in last.items() if t not in (None, none, 0)})
        return out

    def blackout_watch(self, plan, t0):
        """週計畫第 5 週「100% 斷網 → 兩機退化成本機單機、安全完成手頭任務」的 demo 版（使用者 2026-11-09：只驗證任務退化，
        避碰是 BT 的事——斷網時位置廣播也斷，無人機看不到彼此）。情境檔 blackout：
          after "dog_busy"（預設）：有一隻狗在做 CBBA 的地面任務、而且有無人機在做任務時才斷（空地手上都有任務）；
                 等到開始後 max_wait 秒還沒有就照樣斷；"at"：開始後 at 秒斷
          sec：斷多久（所有無人機、狗的容器 mesh_tc.sh set 100）"""
        cfg = self.blackout_cfg()
        if not cfg:
            return
        env, now = self.env, time.time()
        b = getattr(self, "_bo", None)
        conts = list(dict.fromkeys([env.uav(v) for v in env.ids] + (env.dog_containers() if plan.get("dog") else [])))
        if b is None:
            held = self.holders_now(plan)
            after = cfg.get("after", "dog_busy")
            due = now - t0 >= cfg.get("at", 0.0) and (
                after == "at" or (any(k.startswith("dog") for k in held) and any(k.startswith("uav") for k in held)) or
                now - t0 >= cfg.get("max_wait", 240.0))
            if not due:
                return
            t_cut = time.time()          # 6 個容器逐一 docker exec 要 1～2 s：時刻記開始斷的那一刻
            for c in conts:
                env.exec(c, "mesh_tc.sh set 100", check=False)
            self._bo = b = dict(t0=t_cut, t0_done=time.time(), sec=float(cfg.get("sec", 20.0)), held=held,
                                containers=conts)
            plan["blackout"] = b
            self.reply(f"  {b['t0'] - t0:5.1f} s  ⚡ 100% 斷網 {b['sec']:g} s（{len(conts)} 個容器）；手上的任務："
                       + ("、".join(f"{k} {t:08X}" for k, t in sorted(held.items())) or "沒有"))
            return
        if "t1" not in b and now - b["t0"] >= b["sec"]:
            b["t1"] = time.time()        # 恢復的時刻記開始清的那一刻（原本記清完，第一個容器已經恢復約 1 s，B3 出現負數）
            for c in b["containers"]:
                env.exec(c, "mesh_tc.sh clear")
            b["t1_done"] = time.time()
            self.reply(f"  {b['t1'] - t0:5.1f} s  ⚡ 恢復連線")

    def dog_reports(self, plan):
        """狗的 cbba_node 收到 BT 回報成功的紀錄：{task_id: [(牆上時間, dogN)]}（dog_cbba_<ns>.log 的
        「task_result XXXXXXXX：success 1」；ros_bridge 沒有訂狗的 task_result，2026-11-09）"""
        out = {}
        for i, d in enumerate(sorted(plan.get("dog") or [], key=lambda d: d["id"])):
            path = os.path.join(self.run.out, f"dog_cbba_{d['ns']}.log")
            if not os.path.exists(path):
                continue
            for line in open(path, encoding="utf-8", errors="replace"):
                m = re.search(r"\[(\d+\.\d+)\] \[[^\]]+\]: task_result ([0-9A-F]{8})：success 1", line)
                if m:
                    out.setdefault(int(m.group(2), 16), []).append((float(m.group(1)), f"dog{i + 1}"))
        return out

    def check_blackout(self, plan, t0):
        """B1～B4（情境檔有 blackout 時）"""
        b, res, run = plan.get("blackout"), self.res, self.run
        if not b:
            if self.blackout_cfg():
                res.add("B0", "100% 斷網", "FAIL", "沒有斷網（條件一直沒達成，也沒到 max_wait）")
            return
        f_id, f_ok, none = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"], self.cfg["fields"]["no_task_id"]
        cut, back = b["t0"], b.get("t1", time.time())
        dog_name = {d["id"]: f"dog{i + 1}" for i, d in enumerate(sorted(plan.get("dog") or [], key=lambda d: d["id"]))}
        held_by = {}                                     # task → [(開始拿著的時刻, 名稱)]
        for (t, v, task, _) in run.history:
            if task not in (None, none) and t >= cut - 0.5:
                held_by.setdefault(task, []).append((t, f"uav{v}"))
        for e in run.events:
            if e["kind"] == "dog_assigned" and e["t"] >= cut - 0.5:
                task = e["msg"].get(f_id, 0)
                if task not in (None, none, 0):
                    held_by.setdefault(task, []).append((e["t"], dog_name.get(e["dog"], f"dog{e['dog']}")))
        for k, task in b["held"].items():
            held_by.setdefault(task, []).insert(0, (cut, k))
        ground_ids = {e["msg"].get(f_id) for e in self.ground_tasks(t0)}
        reports = self.dog_reports(plan)                 # 狗回報成功：{task: [(t, dogN)]}
        for e in run.events:
            if e["kind"] == "result" and e["msg"].get(f_ok) is True:
                reports.setdefault(e["msg"].get(f_id), []).append((e["t"], f"uav{e['uav']}"))
        closed_t = {}
        for e in run.events:
            if e["kind"] == "closed" and e["msg"].get("status") == 1:
                closed_t.setdefault(e["msg"].get(f_id), e["t"])
        # B1 手上的任務由原本那台做完
        lines, bad = [], []
        for k, task in sorted(b["held"].items()):
            # 原本那台自己回報成功的時刻（狗原本用全隊的 task_closed，會算到別隻狗做完的時間）
            tdone = min((t for t, who in reports.get(task, []) if who == k and t >= cut - 0.5), default=None)
            when = ("斷網中" if tdone is not None and tdone <= back else "恢復後") if tdone is not None else None
            lines.append(f"{k} {task:08X} " + (f"{when}做完（{tdone - cut:.1f} s）" if when else "沒做完"))
            if tdone is None:
                bad.append(k)
        res.add("B1", "100% 斷網：手上的任務由原本那台做完", "FAIL" if bad else "PASS",
                f"斷網 {b['sec']:g} s，" + ("；".join(lines) if lines else "斷網時沒有人手上有任務"))
        # B2 斷網中沒有重複認領（持有者失聯 takeover_fire 10 s 後別台接手火警是設計：火警寧可重複，列 INFO）
        dup_bad, dup_design = [], []
        for task, hs in held_by.items():
            names = list(dict.fromkeys(n for _, n in hs))
            if len(names) < 2:
                continue
            second = min(t for t, n in hs if n != names[0])
            if not (cut - 0.5 <= second <= back + 5.0):
                continue
            done_by = list(dict.fromkeys(who for t, who in sorted(reports.get(task, [])) if t >= cut - 0.5))
            text = (f"{task:08X}：派給 {'、'.join(names)}（第二台在斷網後 {second - cut:.1f} s 拿到），"
                    f"做完的 {'、'.join(done_by) or '沒有'}" + (f"（重複執行 {len(done_by) - 1} 次）" if len(done_by) > 1 else ""))
            (dup_design if task in ground_ids and second - cut >= 10.0 else dup_bad).append(text)
        res.add("B2", "100% 斷網：沒有重複認領", "FAIL" if dup_bad else "PASS",
                ("重複：" + "；".join(dup_bad) if dup_bad else "斷網中、恢復後 5 s 內每個任務只有一台拿過") +
                ("；失聯接手（設計：火警的持有者失聯 10 s 後別台接手，寧可重複）：" + "；".join(dup_design)
                 if dup_design else ""))
        # B3 恢復後一致：斷網中做完的任務，恢復後每台無人機都收到 task_closed
        # 斷網中有人回報做完的任務（無人機、狗）：恢復後多久每台無人機都收到 task_closed
        done_in = {task: min(t for t, _ in rs) for task, rs in reports.items() if any(cut <= t <= back for t, _ in rs)}
        late = []
        for task in done_in:
            ts = [run.closed.get(v, {}).get(task) for v in self.env.ids]
            if any(t is None for t in ts):
                late.append(f"{task:08X}（{sum(t is None for t in ts)} 台不知道）")
        sync = {task: max(run.closed[v][task] for v in self.env.ids) - back for task in done_in
                if all(task in run.closed.get(v, {}) for v in self.env.ids)}
        res.add("B3", "100% 斷網：恢復後全隊一致（斷網中做完的，每台都知道）", "FAIL" if late else "PASS",
                (f"斷網中做完 {len(done_in)} 個；" if done_in else "斷網中沒有做完的任務；") +
                ("沒有同步：" + "、".join(late) if late else
                 "；".join(f"{task:08X} 恢復後 {max(dt, 0.0):.1f} s 每台都收到 task_closed" +
                          ("（斷網中做完的回報版本較舊、被拒收，等接手的那台做完才結束）"
                           if dt > 5.0 and len(reports.get(task, [])) > 1 else "")
                          for task, dt in sync.items())),
                resync=max(sync.values()) if sync else None)
        # B4 斷網中發現的火（新的地面任務）：恢復後由狗接走、處理完
        found = [e for e in self.ground_tasks(t0) if cut <= e["t"] <= back]
        lines = []
        for e in found:
            task = e["msg"].get(f_id)
            first_dog = min((t for t, n in held_by.get(task, []) if n.startswith("dog")), default=None)
            lines.append(f"{task:08X}（{e['uav'] if isinstance(e['uav'], str) else 'uav%d' % e['uav']} 發現）：" +
                         (f"狗在斷網後 {first_dog - cut:.1f} s 接走" if first_dog else "沒有狗接") +
                         (f"、處理完" if task in closed_t else "、沒處理完"))
        res.add("B4", "100% 斷網：斷網中發現的火，恢復後由狗接走", "INFO",
                "；".join(lines) if lines else "斷網中沒有發現新的火")

    def check_manual(self, items, plan, t0, closed_at):
        """新的模擬端測試（2026-10-20）的紀錄：M1 做 G 的那台轉去 M、G 交還由誰接手；M2 去 M 時看到火 → 火警 BT、M 延遲多久；
        M3 之後插入的任務由誰做、當時在做什麼。**都是 INFO**（使用者：延遲、交還是要通知地面站的現象，不是錯誤；
        跑的時候 ground_station_watch 已經在 terminal 印警告），只列出發生了什麼"""
        run, res = self.run, self.res
        f_id, f_ok = self.cfg["fields"]["task_id"], self.cfg["fields"]["success"]
        names = {p["task_id"]: p["point"] for p in plan["patrol"]}

        def results(tid, ok):
            return [e for e in run.events if e["kind"] == "result" and e["msg"].get(f_id) == tid and e["msg"].get(f_ok) is ok]

        finished = {}      # 人工指派 → 完成的時刻（BT 的「平板任務 N 完成」）
        for it in (i for i in items if i["kind"] == "manual"):
            if it.get("pair"):
                if "uav" not in it:
                    res.add("X1", f"對調 {it['name']}", "INFO", f"沒有下指派（{it.get('skipped', '時限內條件沒有成立')}）")
                    continue
                best, notes = self.pair_report(it, t0)
                done = "都完成" if self.manual_finished(it) else "沒有都完成"
                res.add("X1", f"對調 {it['name']}：兩台互相飛到對方的位置（正面交會）→ 避碰", "INFO",
                        f"{it['t'] - t0:.1f} s uav{it['uav']} ({it['xb']:g}, {it['yb']:g}) ↔ uav{it['uav_b']} "
                        f"({it['x']:g}, {it['y']:g})，相距 {it['dist']:.1f} m；" +
                        (f"兩台最近 {best[0]:.2f} m（{best[1] - t0:.1f} s）" if best else "沒有位置紀錄") +
                        f"；{done}" + ("；" + "／".join(notes[:6]) if notes else "；BT 沒有記到避碰"))
                continue
            if "uav" not in it:
                res.add("M1", f"人工指派 {it['name']}", "INFO", "沒有下指派（等不到有一台在做固定任務）")
                continue
            v, held, hp, n = it["uav"], it.get("held"), it.get("from"), it.get("n")
            rec = next((m for m in plan.get("manual", []) if m.get("name") == it["name"]), {})
            back = [e for e in results(held, False) if e["uav"] == v] if held else []
            why = back[0]["msg"].get("detail", "") if back else ""
            by = [e["uav"] for e in results(held, True)] if held else []
            ok1 = bool(rec.get("ok")) and (not held or ("PREEMPTED_BY_MANUAL" in why and bool(by) and by[0] != v))
            text = (f"{it['t'] - t0:.1f} s uav{v}" + (f"（在做 {hp}）" if hp else "（手上沒有 CBBA 任務）") +
                    f" ← {it['name']} ({it['x']:g}, {it['y']:g})、停留 {it['duration']:g} s：" +
                    ("接受" if rec.get("ok") else f"拒絕（{rec.get('message', '沒有回應')}）"))
            if held:
                text += (f"；{names.get(held, f'{held:08X}')} 交還（{why or '沒看到交還'}）→ " +
                         (f"uav{by[0]} 完成" if by else "沒有完成"))
            res.add("M1", f"人工指派 {it['name']}：做 G 的那台交還 G、轉去 M；G 重新競標", "INFO",
                    text + ("" if ok1 else "（和預期不同，看 log）"))

            logs = self.manual_log(v)
            delay = [x for _, x in logs if f"manual_{n} 延遲" in x]
            resume = [x for _, x in logs if f"manual_{n} 繼續" in x or f"manual_{n} 完成" in x]
            fin = [t for t, x in logs if f"平板任務 {n} 完成" in x]
            if fin:
                finished[it["name"]] = (v, fin[0])
            fire = next((f for f in plan["fires"] if f.get("manual") == it["name"]), None)
            if fire:
                gt, who = fire.get("ground_task"), fire.get("reported_by")
                tc = closed_at(gt) if gt else None
                ok2 = who == v and bool(tc) and bool(delay) and bool(fin) and fin[0] >= tc - 0.5
                text = (f"火 {fire['name']}（{fire.get('where', '')}）：" +
                        (f"{who if isinstance(who, str) else f'uav{who}'} 發現、發布地面任務 {gt:08X}" if gt else "沒有人發現") +
                        (f"（不是被指派的 uav{v}）" if who and who != v else "") +
                        (f"；狗處理完 {tc - t0:.1f} s" if tc else "；地面任務沒有結束") +
                        (f"；{it['name']} 完成 {fin[0] - t0:.1f} s（指派後 {fin[0] - it['t']:.1f} s）" if fin else
                         f"；{it['name']} 沒有完成") +
                        f"；延遲訊息：{delay[0] if delay else '沒有'}" + (f"／{resume[-1]}" if resume else ""))
                res.add("M2", f"去 {it['name']} 時看到火警 → 火警 BT（發布地面任務、監控到狗處理完）→ {it['name']} 延遲", "INFO",
                        text + ("" if ok2 else "（和預期不同，看 log）"))
            else:
                res.add("M2", f"{it['name']} 完成", "INFO",
                        "這次沒有放火；" + (f"{it['name']} 完成 {fin[0] - t0:.1f} s" if fin else f"{it['name']} 沒有完成"))

        for it in items:
            if it["kind"] != "task" or not it.get("via") or not it.get("done"):
                continue
            p = next((p for p in plan["patrol"] if p["point"] == it["name"]), None)
            h, i = it["via"]
            ok_runs = results(p["task_id"], True) if p else []
            who = ok_runs[0]["uav"] if ok_runs else i
            best, notes = self.pair_report(dict(uav=who, uav_b=h, t=p["t"] if p else it.get("ready_t", t0)), t0)
            res.add("X1", f"{it['name']} 放在懸停的 uav{h} 後方：航線經過它 → 避碰", "INFO",
                    f"{(p['t'] if p else t0) - t0:.1f} s 插入 {it['name']}，預期 uav{i} 接、經過 uav{h}；" +
                    (f"uav{who} 完成" if ok_runs else f"{it['name']} 沒有完成") + "；" +
                    (f"兩台最近 {best[0]:.2f} m（{best[1] - t0:.1f} s）" if best else "沒有位置紀錄") +
                    ("；" + "／".join(notes[:6]) if notes else "；BT 沒有記到避碰"))

        for it in items:
            if it["kind"] != "task" or not it.get("after_manual") or not it.get("done"):
                continue
            p = next((p for p in plan["patrol"] if p["point"] == it["name"]), None)
            ok_runs = results(p["task_id"], True) if p else []
            if not ok_runs:
                res.add("M3", f"插入 {it['name']}", "INFO", f"{it['name']} 沒有完成")
                continue
            u, ins = ok_runs[0]["uav"], p["t"]
            # T 由沒任務的那台直接做，或排在手上的任務（鎖定中、同級不搶占）後面、做完才開始（2026-10-20 使用者）：
            # 看那台的 current_task 換成 T 的那一刻，前一個任務已經回報完成、而且不在人工指派中
            hist = [h for h in run.history if h[1] == u]
            k = next((i for i, h in enumerate(hist) if h[2] == p["task_id"]), None)
            start_t = hist[k][0] if k is not None else None
            before = next((h[2] for h in reversed(hist[:k]) if h[2] not in (0, p["task_id"])), 0) if k else 0
            before_done = before == 0 or any(e["t"] <= start_t + 0.5 for e in results(before, True))
            at_ins = [h for h in hist if h[0] <= ins]
            cur = at_ins[-1][2] if at_ins else 0
            cur_done = cur == 0 or any(e["t"] <= ins for e in results(cur, True))
            m = next(((fv, ft) for _, (fv, ft) in finished.items() if fv == u), None)
            manual_busy = start_t is not None and any(
                i["kind"] == "manual" and i.get("uav") == u and i.get("t", math.inf) <= start_t and
                not (m and m[1] <= start_t) for i in items)
            state = ("閒著" if cur == 0 else f"{names.get(cur, f'{cur:08X}')} 已完成" if cur_done else
                     f"還在做 {names.get(cur, f'{cur:08X}')}，T 排在後面")
            ok3 = start_t is not None and before_done and not manual_busy
            res.add("M3", f"插入 {it['name']}：由沒任務的那台做，或排在手上的任務後面、做完才開始（不打斷）", "INFO",
                    f"{ins - t0:.1f} s 插入 {it['name']} ({p['x']:g}, {p['y']:g}) → uav{u}；插入時 {state}" +
                    (f"；{start_t - t0:.1f} s 開始做 {it['name']}" if start_t else "；沒看到開始") +
                    ("" if before_done else f"（{names.get(before, f'{before:08X}')} 還沒完成就換成 T）") +
                    ("；開始時還在人工指派中" if manual_busy else "") + "，完成" +
                    ("" if ok3 else "（和預期不同，看 log）"))

    # ------------------------------------------------------------------ 手動插入（--interactive，2026-10-14）
    CONSOLE_HELP = (
        "  手動插入（另開一個 terminal：python3 docker/itest/insert.py；或直接在這裡輸入後按 Enter）：\n"
        "    t [x y] [d=秒]    任務點（藍色平台）；不給 x y = 範圍內隨機\n"
        "    t G2 [d=秒]       用場景的點名\n"
        "    tf [x y] [d=秒]   任務點、而且那裡起火（飛到時看到火 → 發布地面任務 → 監控到狗處理完）\n"
        "    f [x y]           只起火、不建任務（無人機路過或到附近時發現）\n"
        "    m uav2 [x y] [d=秒]   人工指派（地面站／平板）：指定那台去那裡停留，期間不參與 CBBA；做完自動回到 CBBA\n"
        "    m dog51 R3 [d=秒]     狗也可以（dog50～52 或 v60、v60_1…）；m uav2 clear 提早解除\n"
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
            for m in plan.get("manual", []):
                self.reply(f"  {m['target']:>6} " + ("解除人工指派" if m["clear"] else
                           f"人工指派 ({m['x']:6.1f}, {m['y']:6.1f}) 停留 {m['duration']:g} s") +
                           ("" if m["ok"] else f"（拒絕：{m['message']}）"))
        elif cmd == "m":
            self.manual_cmd(rest, dur, plan, t0, pts)
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

    def manual_cmd(self, rest, dur, plan, t0, pts):
        """人工指派（2026-10-20）：m <uavN|dogN|ns> [x y | 點名] [d=秒]，或 m <…> clear。在 sim 容器呼叫
        /<ns>/manual_override（swarm_interfaces/ManualOverride；無人機由 uav_task_manager_node、狗由 fake_dog.py 提供），
        和伯宇 BT 的平板接管同一個介面：被指派的那台交還手上的 CBBA 任務（PREEMPTED_BY_MANUAL）、mode MANUAL、做完恢復。
        每個指派編號 M1、M2…（2026-10-24 使用者：畫面上用 Mx 表示，不直接寫座標）"""
        job = self.manual_prepare(rest, dur, plan, pts)
        if job:
            self.manual_send(job)
            self.manual_record(job, plan, t0)

    def manual_prepare(self, rest, dur, plan, pts):
        """解析 m 指令 → 要送的請求（還沒送）；不合法回 None"""
        if not rest:
            self.reply("  用法：m uav2 [x y | 點名] [d=秒]，或 m uav2 clear")
            return None
        who, rest = rest[0].lower(), rest[1:]
        units = (self.cfg.get("dog") or {}).get("units") or []
        ns = None
        if who.startswith("uav") and who[3:].isdigit() and int(who[3:]) in self.env.ids:
            ns = self.cfg["vehicles"]["ns"].format(id=int(who[3:]))
        else:
            for u in units:
                if who in (u["ns"], f"dog{u['id']}"):
                    ns, who = u["ns"], f"dog{u['id']}"
        if ns is None:
            self.reply(f"  不認得 {who}：要是 " + "、".join([f"uav{v}" for v in self.env.ids] +
                                                           [f"dog{u['id']}" for u in units]))
            return None
        clear = bool(rest) and rest[0].lower() == "clear"
        job = dict(who=who, ns=ns, clear=clear, duration=dur, x=0.0, y=0.0, label="")
        if clear:
            job["req"] = "{clear: true}"
        else:
            xy = self.console_xy(rest, plan, pts)
            if xy is None:
                return None
            job["x"], job["y"] = xy[0], xy[1]
            self.n_manual += 1
            job["n"] = self.n_manual
            job["label"] = f"M{self.n_manual}"
            job["req"] = (f"{{task: {{task_id: {self.n_manual}, position: {{x: {job['x']}, y: {job['y']}, z: 0.0}}, "
                          f"duration_sec: {dur}}}, clear: false}}")
        return job

    def manual_send(self, job):
        """送出（ros2 service call，約 3～4 s；可以在不同執行緒同時送）"""
        r = self.env.exec(self.env.sim(), f"timeout 15 ros2 service call /{job['ns']}/manual_override "
                          f"swarm_interfaces/srv/ManualOverride \"{job['req']}\"")
        out = r.stdout or ""
        job["ok"] = "accepted=True" in out
        m = re.search(r"message='([^']*)'", out)
        job["message"] = m.group(1) if m else ((r.stderr or out).strip().splitlines() or ["沒有回應"])[-1]
        job["t"] = time.time()

    def manual_record(self, job, plan, t0):
        plan.setdefault("manual", []).append(dict(t=job["t"], target=job["who"], ns=job["ns"], clear=job["clear"],
                                                  x=job["x"], y=job["y"], duration=job["duration"], ok=job["ok"],
                                                  message=job["message"], label=job["label"]))
        self.reply(f"  {job['t'] - t0:5.1f} s  {job['who']} " +
                   ("解除人工指派" if job["clear"] else f"← {job['label']}（人工指派，停留 {job['duration']:g} s）") +
                   ("：" + job["message"] if job["ok"] else f"：拒絕／沒有回應（{job['message']}）"))

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
        # 隨機：離場景的點、已插入的點、起飛點都 ≥ min_gap，不在障礙物範圍內
        taken = [(p[0], p[1]) for p in pts.values()] + [(p["x"], p["y"]) for p in plan["patrol"]] + \
            [(f["x"], f["y"]) for f in plan["fires"]] + [self.env.home(v) for v in self.env.ids]
        for _ in range(500):
            x, y = round(self.rng.uniform(x0, x1), 1), round(self.rng.uniform(y0, y1), 1)
            if not self.blocked(x, y) and all(math.hypot(x - a, y - b) >= self.min_gap for a, b in taken):
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
        # 展示用的 BT 參數（config 的 demo.exec_args＋情境檔的 exec_args，例如 --speed 調快時 CBF 的 influence_radius）
        # 速度不是 yaml 的 0.5 m/s 時，CBF 的 γ、influence_radius、miss_clearance 照速度算（cbf_speed_args）；
        # config、情境檔的 exec_args 接在後面，要手動蓋掉時寫在那裡
        exec_args = " ".join(a for a in (cbf_speed_args(self.speed), d.get("exec_args", ""), sc.get("exec_args", ""))
                             if a)
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
        if dog and (mode in ("point", "side") or (mode == "custom" and (
                sc.get("fires") or self.interactive or any(m.get("fire") for m in sc.get("manuals", [])) or
                any(t.get("fire") is True for t in sc.get("tasks", []))))):
            # 2026-10-20 起可以多隻（dog.units，分散在場地各處，看狗之間對火警的競標）；紀錄檔 fake_dog_<ns>.log、dog_cbba_<ns>.log
            units = dog.get("units") or [dict(ns="v60", id=50, start_xy=dog["start_xy"])]
            plan["dog"] = []
            plan["dog_detect"] = d.get("dog_detect")     # 偵測半徑（2D 動畫、gazebo_labeled.mp4 畫狗的偵測範圍，2026-11-09）
            bt = getattr(env, "dog_mode", "fake") == "bt"
            for u in units:
                dx, dy = u["start_xy"]
                f = dict(repo=env.repo, world=d["world"], x=dx, y=dy, ns=u["ns"], id=u["id"], speed=dog["speed"],
                         ptz=dog.get("ptz_sec", 5.0), mission=f"{env.repo}/docker/dog/idle_mission.yaml")
                route = (env.cfg.get("worlds", {}).get(d["world"], {}).get("dog_patrol") or {}).get(u["ns"])
                patrol = []
                if bt and route:
                    # 巡邏（2026-11-05）：照伯宇的巡檢線產生任務檔（他的格式，config/MISSION_FORMAT.md），loop 一直巡。
                    # 日常巡檢的內容（2026-11-07 使用者：狗也要有任務定義，Live Agent Status 才看得到變化）：每個巡檢點
                    # 到點後 ptz_inspect 雲台看最近的設備頂面、拍 1 張（伯宇的 BT 唯一的到點動作；方塊身體的假雲台等
                    # dog.ptz_sec 秒回 OK）。沒有設備清單時照舊只走路
                    name = f"dog_mission_{u['ns']}.yaml"
                    ptz_sec = float(dog.get("ptz_sec", 5.0))
                    # 巡檢點名稱每隻狗不同（2026-11-07 使用者）：照機號排序 dog1 → A1～A3、dog2 → B1～、dog3 → C1～
                    # （原本都叫 P1～P3，Live Agent Status 看起來像同一組點）
                    letter = "ABCDEFGH"[sorted(q["id"] for q in units).index(u["id"]) % 8]
                    wps = []
                    for k, (px, py) in enumerate(route):
                        e = min(self.equip, key=lambda q: math.hypot(q["x"] - px, q["y"] - py)) if self.equip else None
                        tasks = ("    tasks: []\n" if e is None else
                                 "    tasks:\n      - action: ptz_inspect\n"
                                 f"        target: {{ x: {e['x']:g}, y: {e['y']:g}, z: {e['top']:g} }}\n"
                                 f"        snapshots: 1\n        timeout: {ptz_sec + 10:g}\n")
                        wps.append(f"  - id: {letter}{k + 1}\n    goto: {{ x: {px:g}, y: {py:g}, yaw: 0.0 }}\n" + tasks)
                        patrol.append(dict(name=f"{letter}{k + 1}", x=px, y=py, inspect=e["name"] if e else None))
                    with open(os.path.join(run.out, name), "w", encoding="utf-8") as fh:
                        fh.write(f'mission: "模擬 巡檢 {u["ns"]}"\ncontext:\n  map: "{d["world"]}"\n  frame_id: "map"\n'
                                 "policy:\n  on_nav_fail: \"skip_waypoint\"\n  on_task_fail: \"skip_task\"\n"
                                 "  mission_timeout: 3600\n  loop: true\nwaypoints:\n" + "".join(wps))
                    f["mission"] = f"{run.cout}/{name}"
                    # 巡檢點在 Gazebo 標出來（2026-11-07 使用者：圖上、Gazebo 都標狗的任務）：紫色小圓盤 task_<A1>，
                    # 巡邏一直繞，做完不刪；背景一次生成，不拖慢啟動
                    env.exec(env.sim(), " ; ".join(f"patrol_scenario.sh mark-at {q['x']:g} {q['y']:g} {q['name']} P"
                                                   for q in patrol), detach=True, env={"CBBA_SCENARIO": d["world"]})
                    loop = [tuple(u["start_xy"])] + [tuple(p) for p in route]   # 不能叫 pts（上面場景點位的變數）
                    self.dog_patrol_segments += list(zip(loop, loop[1:])) + [(loop[-1], loop[1])]
                if bt:
                    # --dog bt（2026-10-27）：sim 放方塊身體（假 Nav2、假雲台、TF），dog.container 跑伯宇的 task_manager；
                    # 紀錄檔 dog_body_<ns>.log、dog_bt_<ns>.log
                    env.exec(env.sim(), dog["bt_sim_start"].format(**f), detach=True,
                             log=f"{run.cout}/dog_body_{u['ns']}.log")
                    env.exec(env.dog_container(u), dog["bt_start"].format(**f), detach=True,
                             log=f"{run.cout}/dog_bt_{u['ns']}.log")
                else:
                    env.exec(env.sim(), dog["sim_start"].format(**f), detach=True, log=f"{run.cout}/fake_dog_{u['ns']}.log")
                env.exec(env.dog_container(u), dog["start"].format(**f), detach=True,
                         log=f"{run.cout}/dog_cbba_{u['ns']}.log")
                # patrol：日常巡檢點（A1、A2…，2D 動畫的 Live Agent Status 用；沒有巡邏時是空的）
                plan["dog"].append(dict(ns=u["ns"], id=u["id"], x=dx, y=dy, speed=dog["speed"], patrol=patrol))
            print(f"  {'伯宇的狗 BT＋方塊身體' if bt else '假狗'} {len(units)} 隻" +
                  ("（照巡檢線巡邏）" if self.dog_patrol_segments else "（沒任務時停在原地）") + "：" +
                  "、".join(f"{u['ns']}（{u['id']}）在 ({u['start_xy'][0]:g}, {u['start_xy'][1]:g})" for u in units))
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
        deadline = t0 + max(d["timeout"], d.get("timeout_min", 0.0))
        finished = None
        settled_since = None
        if mode == "custom":
            deadline = t0 + max(sc.get("timeout", d.get("interactive_timeout", 600.0) if self.interactive else d["timeout"]),
                                d.get("timeout_min", 0.0))     # 大場景（yard）的情境檔時限至少 timeout_min
            if self.interactive:
                self.ia_session(ready=True)
                self.reply(f"  CBBA 上線，開始接受指令（最長 {deadline - t0:.0f} s；模擬電池約 12 分鐘從 100% 降到 20%、"
                           f"之後無人機會返航，長時間測試要注意）")
                print(self.CONSOLE_HELP)
        while time.time() < deadline:
            run.read()
            self.dog_watch(plan, t0)
            self.marker_watch(plan, t0)
            self.blackout_watch(plan, t0)
            if mode == "custom":
                self.insert_due(items, plan, t0)
                self.ground_station_watch(plan, t0)
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
            waiting = mode == "custom" and (any(not it.get("done") for it in items) or not self.console_done or
                                            any(it["kind"] == "manual" and not self.manual_finished(it) for it in items))
            # 自訂情境、手動插入（2026-10-15）：任務都做完、狗也處理完之後，還沒被發現的火再等 fire_grace 秒；
            # 還是沒人發現（不在任何航線、懸停點附近）就照常結束，D2 標「沒有發現」，不卡到 timeout
            settled = not pending and not waiting and all_done and dog_done and not self.blackout_open()
            if not settled:
                settled_since = None
            elif settled_since is None:
                settled_since = time.time()
            fires_ok = len(g) >= len(plan["fires"]) or (
                mode == "custom" and settled_since is not None and time.time() - settled_since >= d.get("fire_grace", 15.0))
            if settled and fires_ok:
                finished = time.time()
                # 最後一個任務做完就離開迴圈，1 s 一次的 marker_watch 還沒看到它（2026-11-07：T5 的標記沒刪）
                run.read()
                self.marker_watch(plan, t0, force=True)
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
        homes = {v: env.home(v) for v in ids}

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
                who = e["uav"] if isinstance(e["uav"], str) else f"uav{e['uav']}"
                rows.append(f"{f['name']}（{where}）：{who} 起火後 {e['t'] - f['spawned']:.1f} s 發現、"
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
            res.add("D2", "看到火警（無人機 tag 101，或狗的模擬偵測）→ 發布地面處置任務", "PASS" if ok else "FAIL",
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
            dog_found = isinstance(v, str)
            if dog_found:
                # 狗先看到（模擬偵測，2026-11-05）：去巡檢點的那台到了看到同一個火已經有人報 → 照 BT 監控那個地面任務，
                # 檢查的對象換成它
                v = executor
            drift = None
            if v and tc:
                start = f["reported_t"]
                if dog_found:             # 從它飛到火點上空（1 m 內）開始算
                    start = next((e["t"] for e in run.events if e["kind"] == "pos" and e["uav"] == v and
                                  e["t"] >= f["reported_t"] and math.hypot(e["x"] - f["x"], e["y"] - f["y"]) < 1.0), tc)
                ps = [e for e in run.events if e["kind"] == "pos" and e["uav"] == v and start <= e["t"] <= tc]
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
        if mode == "custom" and any(it["kind"] == "manual" for it in items):
            self.check_manual(items, plan, t0, closed_at)
        home_rows = []
        for v in ids:
            p = run.positions.get(v)
            home_rows.append(f"uav{v} " + (f"離起飛點 {math.hypot(p[0] - homes[v][0], p[1] - homes[v][1]):.2f} m、"
                                           f"高度 {p[2]:.1f} m" if p else "沒有位置"))
        res.add("D5", "全部做完後回到起飛點上空", "PASS" if all(at_home(v) for v in ids) else "FAIL",
                "；".join(home_rows))
        self.check_cbf(t0, time.time(), plan)
        self.check_blackout(plan, t0)
        if plan.get("blackout"):
            # 斷網的時間軸圖（2026-11-09，plot_blackout.py）要 demo_plan.json：先寫一次（結尾會再寫完整的）
            with open(os.path.join(run.out, "demo_plan.json"), "w", encoding="utf-8") as f:
                json.dump(plan, f, ensure_ascii=False, indent=2)
            env.exec(env.sim(), f"python3 {env.repo}/docker/itest/plot_blackout.py {run.cout}",
                     log=f"{run.cout}/plot_blackout.log")

        print(f"  停 {self.hold:.0f} s 讓你看 Gazebo 畫面")
        run.sleep(self.hold)
        rec_end = time.time()
        env.pkill(env.sim(), "record_cameras.py")
        if recording:
            self.gz_record(False)
            run.sleep(3.0)                      # 存檔要一點時間
        # 錄影的開始、結束（牆上時間）：label_gazebo.py 用來把事件對到 gazebo.mp4（2026-11-05）
        plan["record"] = dict(t0=rec.get("t0"), t1=rec_end) if recording else None
        # 人工指派實際做完的時刻（2026-11-07）：2D 動畫的 Live Agent Status、旗子變灰用，不再用距離猜
        # （無人機：BT log「平板任務 N 完成」；狗：mode 從 MANUAL 回到 AUTO）
        for m in plan.get("manual") or []:
            if not m.get("clear") and m.get("ok"):
                m["finished_t"] = self.manual_done_t(m)
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
        # 出價表（2026-10-20）：各台 cbba_node 的「出價表」log → bids.md、bids.csv（主機、只用標準庫）
        r = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "bid_table.py"),
                            run.out], capture_output=True, text=True)
        print(f"  出價表：{(r.stdout or r.stderr).strip()}")
        gz_ok = os.path.exists(os.path.join(run.out, "gazebo.mp4"))
        cams_mp4 = self.compose_cameras(run, rec.get("t0"), rec_end) if gz_ok else None
        # Gazebo 影片加標籤（2026-11-05，label_gazebo.py：主機的 Pillow＋ffmpeg）→ gazebo_labeled.mp4
        labeled = None
        if gz_ok and shutil.which("ffmpeg"):
            r = subprocess.run([sys.executable, os.path.join(HERE, "label_gazebo.py"), run.out],
                               capture_output=True, text=True)
            if r.returncode == 0 and os.path.exists(os.path.join(run.out, "gazebo_labeled.mp4")):
                labeled = os.path.join(run.out, "gazebo_labeled.mp4")
            else:
                print(f"  Gazebo 影片加標籤失敗：{(r.stderr or r.stdout).strip()[-300:]}")
        res.add("D6", "影片", "INFO" if os.path.exists(mp4) and gz_ok else "FAIL",
                ("俯視動畫 demo.mp4" if os.path.exists(mp4) else f"俯視動畫沒有合成（看 {run.out}/animate.log、ffmpeg）") +
                ("；Gazebo 畫面 gazebo.mp4" if gz_ok else "；Gazebo 畫面沒有錄到") +
                ("；加標籤 gazebo_labeled.mp4" if labeled else "") +
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
    ap.add_argument("--tasks", type=int, default=None,
                    help="loss：每次試驗的任務數（平均放在各台身邊，理論上每台分到 tasks／台數 個；每台最多 max_bundle 5 個）。"
                         "預設 9；有 --dogs 時 2 ×（無人機＋狗）")
    ap.add_argument("--dogs", type=int, default=0,
                    help="comm、loss：加幾隻狗（2026-11-09，第 5 週空地 CBBA）：每隻在自己的容器 cbba_dogK 跑 cbba_node（UGV），"
                         "loss 的任務改成空中＋地面（火警、巡檢）混合；要先在 docker/.env 設 DOG_COUNT 並 ./start.sh")
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
    ap.add_argument("--world", default=None,
                    help="demo 的場景（預設 config 的 demo.world＝demo_site）；yard = 伯宇的變電所（出生在起降板、巡檢高度 8 m，"
                         "設定在 config 的 worlds.yard）")
    ap.add_argument("--executor", choices=["bt", "task_manager"], default="task_manager",
                    help="uav、demo：無人機的 BT（task_manager = 照機器狗端骨架的 uav_task_manager_node，預設；bt = 備用 uav_bt_node）")
    ap.add_argument("--no-restart", action="store_true",
                    help="loss：每個掉包率不重新啟動 cbba_node（2026-10-27 起預設重新啟動；要重現長時間累積的問題時用）")
    ap.add_argument("--gdb", action="store_true",
                    help="comm、loss：cbba_node 在 gdb 底下跑，當掉時 backtrace 印在 node_uavN.log（2026-10-27；要 RelWithDebInfo 編譯）")
    ap.add_argument("--node-param", action="append", default=[], metavar="KEY=VALUE",
                    help="comm、loss：每台 cbba_node 多加的參數（可以給多次），例如 --node-param resend_gap=0.5 "
                         "--node-param completion_retry=0.5；記在 trials.csv、levels.csv 的 node_params，"
                         "plot_loss.py 給多個目錄時依它分成不同的線（2026-11-04）")
    ap.add_argument("--dog", choices=["fake", "bt"], default="fake",
                    help="demo 的狗：fake = fake_dog.py（直線、BT 邏輯是我們寫的，預設）；bt = 伯宇的狗 BT（task_manager）＋"
                         "方塊身體 fake_nav_body.py（假 Nav2、假雲台），要先在 uav1 容器 dog_bt.sh build（2026-10-27）")
    args = ap.parse_args()
    env = Env(args.config)
    env.executor = args.executor
    env.dog_mode = args.dog
    env.gdb = args.gdb
    env.node_params = []
    for kv in args.node_param:
        k, sep, v = kv.partition("=")
        if not sep or not k.strip() or not v.strip():
            print(f"--node-param 要寫成 KEY=VALUE：{kv}")
            return 2
        env.node_params.append((k.strip(), v.strip()))
    if args.dogs:
        if args.suite not in ("comm", "loss"):
            print("--dogs 只用在 comm、loss（demo 的狗照 --dog）")
            return 2
        env.add_dogs(args.dogs)
    if args.tasks is None:
        args.tasks = 2 * len(env.ids) if env.dogs else 9
    if args.suite == "demo":
        env.apply_world(args.world or env.cfg["demo"]["world"])
    only = {s.strip() for s in args.only.split(",") if s.strip()}

    # 容器沒開時 docker exec 直接失敗，連紀錄檔都不會有（2026-10-24：容器停了 33 小時，只看到「ros_bridge 沒有啟動」）
    need = [env.sim()] + [env.uav(v) for v in env.ids] + (env.dog_containers() if args.suite == "demo" else [])
    down = [c for c in dict.fromkeys(need)
            if subprocess.run(["docker", "inspect", "-f", "{{.State.Running}}", c],
                              capture_output=True, text=True).stdout.strip() != "true"]
    if down:
        print(f"容器沒有在跑：{'、'.join(down)}。先在主機執行 docker/start.sh（會照 .env 開 sim、uav1～N、dog1～K、gcs；"
              f"狗的容器數是 .env 的 DOG_COUNT）")
        return 2

    # 已經有在跑的東西時先停下來（避免打斷使用者正在做的事）
    busy = []
    for vid in env.ids:
        for pat in (env.node_pattern(vid), env.cfg["uav"]["stop_pattern"]):
            if env.running(env.uav(vid), pat):
                busy.append((env.uav(vid), pat))
    for pat in ("bin/px4 -i", "gz[ ]sim", "parameter_bridge", "ros_bridge.py", "fake_dog.py", "fake_nav_body.py",
                "record_cameras.py"):
        if env.running(env.sim(), pat):
            busy.append((env.sim(), pat))
    dog = env.cfg.get("dog")
    for pat in ((dog["stop_pattern"], dog.get("bt_stop_pattern")) if dog else ()):
        for dc in env.dog_containers():
            if pat and env.running(dc, pat):
                busy.append((dc, pat))
    if args.suite == "demo" and args.dog == "bt" and dog and dog.get("bt_check") and \
            env.exec(env.dog_containers()[0], dog["bt_check"]).returncode != 0:
        print(f"--dog bt：{env.dog_containers()[0]} 還沒編譯伯宇的 task_manager，先在那個容器執行 dog_bt.sh build"
              "（dog_ws 在 repo 裡，所有容器共用，編一次就好）")
        return 2
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
    print(f"[itest] {args.suite}：{len(env.uav_ids)} 台" + (f"＋狗 {len(env.dogs)} 隻" if env.dogs else "") +
          f"，輸出 {out_dir}" +
          (f"，BT {args.executor}" if args.suite in ("uav", "demo") else "") +
          ("，cbba_node 參數 " + " ".join(f"{k}={v}" for k, v in env.node_params) if env.node_params else ""))
    run = Run(env, out_dir, with_px4=args.suite in ("uav", "demo"))
    if args.suite == "loss":
        suite = LossSuite(env, run, only, [int(x) for x in args.levels.split(",") if x.strip()],
                          args.trials, args.delay, args.jitter, tasks=args.tasks, fresh=not args.no_restart)
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
            env.pkill(env.uav(vid), env.node_pattern(vid))
            if args.suite in ("uav", "demo"):
                env.pkill(env.uav(vid), env.cfg["uav"]["stop_pattern"])
        if args.suite in ("uav", "demo"):
            dog = env.cfg.get("dog")
            if dog:
                for dc in env.dog_containers():
                    env.pkill(dc, dog["stop_pattern"])
                env.pkill(env.sim(), dog["sim_stop_pattern"])
                if dog.get("bt_stop_pattern"):           # --dog bt（2026-10-27）
                    for dc in env.dog_containers():
                        env.pkill(dc, dog["bt_stop_pattern"])
                    env.pkill(env.sim(), dog["bt_sim_stop_pattern"])
            # px4_start 的 stdin（tail -f /dev/null）：不關的話每跑一次留一個（2026-10-13 前用 sleep infinity，
            # 一直累積；而且容器的主程序也是 sleep infinity，不能用它比對）
            for pat in ("bin/px4 -i", "gz[ ]sim", "parameter_bridge", "tail -f /dev/null", "record_cameras.py"):
                env.pkill_hard(env.sim(), pat)
        if args.suite in ("comm", "loss"):
            for vid in env.ids:
                env.exec(env.uav(vid), "mesh_tc.sh clear")
        if args.suite == "demo" and (getattr(suite, "scenario", None) or {}).get("blackout"):
            # 情境檔的 100% 斷網（2026-11-09）：中途停掉時也要恢復，不然下一次測試的網路是斷的
            for c in dict.fromkeys([env.uav(v) for v in env.ids] + env.dog_containers()):
                env.exec(c, "mesh_tc.sh clear")
    title = {"comm": "CBBA 之間的通訊", "uav": "UAV 整串", "loss": "弱網量測（掉包梯度）",
             "demo": "展示（Gazebo＋GIF）"}[args.suite]
    return 1 if suite.res.write(out_dir, f"整合測試：{title}") else 0


if __name__ == "__main__":
    sys.exit(main())

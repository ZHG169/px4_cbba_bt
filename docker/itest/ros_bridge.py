#!/usr/bin/env python3
"""整合測試的 ROS 端（在 sim 容器執行，itest.py 會自動啟動）。

sim 容器經線材網路連得到每一台 uav 的 DDS，所以在這裡：
  - 訂閱每台的 assigned、result、new_task（和 PX4 的位置、狀態），每則寫一行到 <dir>/events.jsonl
  - 讀 <dir>/commands.jsonl（itest.py 寫入），發話題、開始／停止週期性的假 robot_state
  - 假身體（2026-11-16，itest group 的動畫；不用真的 BT）：收到 current_task 就走向任務點（速度由 itest 給；
    2026-11-22 起繞開這個平台不能進的區域：site-config/yard/zones.geojson 的 no_entry，代替真的狗的 Nav2），
    到了發 {"kind": "arrived"}；位置寫成 robot_state（10 Hz）與 {"kind": "pos"}（5 Hz）。task_id = 0 或 CANCELLED → 停下。
    body_hold 讓它停在原地（模擬當機）、body_mode 改 robot_state 的 mode（MANUAL 時 cbba_node 不出價）、
    note 寫一筆說明給動畫（{"kind": "note"}）

只看話題，不看任何節點的 log；話題名稱、型別、QoS 都由 config.json 決定，換成別人的實作時不用改這裡。
時間用 time.time()（主機和容器共用核心的時鐘），itest.py 才能和自己的時間比較。
"""
import argparse
import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message


# ---------- 假身體繞開禁入區（2026-11-22：原本直線走，狗會穿過禁狗區）----------
ZONES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "site-config", "yard", "zones.geojson")
ZONE_MARGIN = 1.0          # 離區域邊緣至少這麼遠
CORNER_MARGIN = 1.3        # 繞過角落的點（比 ZONE_MARGIN 大一點，沿著邊走不會被當成穿過）
PLATFORM_KEY = {"dog": "quadruped", "uav": "uav"}


def load_blocks():
    """{平台: [(x0, x1, y0, y1)]}：每個平台不能進的區域（外框；yard 的區域都是矩形）"""
    try:
        with open(ZONES, encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return {}
    out = {}
    for feat in doc.get("features", []):
        ring = feat.get("geometry", {}).get("coordinates", [[]])[0]
        if not ring:
            continue
        box = (min(p[0] for p in ring), max(p[0] for p in ring), min(p[1] for p in ring), max(p[1] for p in ring))
        for plat, key in PLATFORM_KEY.items():
            if key in feat.get("properties", {}).get("no_entry", []):
                out.setdefault(plat, []).append(box)
    return out


def _crosses(a, b, box, m):
    """線段 a→b 有沒有穿過外擴 m 的矩形內部（Liang–Barsky 裁切）"""
    x0, x1, y0, y1 = box[0] - m, box[1] + m, box[2] - m, box[3] + m
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if abs(p) < 1e-12:
            if q <= 0:
                return False
        else:
            r = q / p
            if p < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
            if t0 >= t1:
                return False
    return t1 - t0 > 1e-6


def _inside(pt, box, m):
    return box[0] - m < pt[0] < box[1] + m and box[2] - m < pt[1] < box[3] + m


def plan_path(start, goal, blocks):
    """start → goal 繞開 blocks 的最短路（角落的可視圖＋Dijkstra）；回傳要經過的點（含 goal）。
    起點或終點本來就在某個區域裡（剛重設、或規劃器給錯）就不管那個區域，免得走不出來"""
    active = [b for b in blocks if not _inside(start, b, ZONE_MARGIN) and not _inside(goal, b, ZONE_MARGIN)]
    if not any(_crosses(start, goal, b, ZONE_MARGIN) for b in active):
        return [goal]
    nodes = [start, goal]
    for b in active:
        for x in (b[0] - CORNER_MARGIN, b[1] + CORNER_MARGIN):
            for y in (b[2] - CORNER_MARGIN, b[3] + CORNER_MARGIN):
                if not any(_inside((x, y), o, ZONE_MARGIN) for o in active):
                    nodes.append((x, y))
    n = len(nodes)
    dist, prev, done = [float("inf")] * n, [-1] * n, [False] * n
    dist[0] = 0.0
    for _ in range(n):
        u = min((i for i in range(n) if not done[i]), key=lambda i: dist[i], default=None)
        if u is None or dist[u] == float("inf"):
            break
        done[u] = True
        if u == 1:
            break
        for v in range(n):
            if done[v] or any(_crosses(nodes[u], nodes[v], b, ZONE_MARGIN) for b in active):
                continue
            d = dist[u] + ((nodes[u][0] - nodes[v][0]) ** 2 + (nodes[u][1] - nodes[v][1]) ** 2) ** 0.5
            if d < dist[v]:
                dist[v], prev[v] = d, u
    if prev[1] == -1:
        return [goal]       # 找不到路：照直線（不應該發生，閘道與 CBBA 不會把任務放在走不到的地方）
    path, i = [], 1
    while i != 0:
        path.append(nodes[i])
        i = prev[i]
    return path[::-1]


def qos_of(spec, depth=20):
    return QoSProfile(
        depth=spec.get("depth", depth),
        reliability=ReliabilityPolicy.RELIABLE if spec.get("reliable", True) else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if spec.get("transient_local") else DurabilityPolicy.VOLATILE)


class Bridge(Node):
    def __init__(self, cfg, out_dir, with_px4):
        super().__init__("itest_bridge")
        self.cfg = cfg
        self.out = open(os.path.join(out_dir, "events.jsonl"), "a", buffering=1, encoding="utf-8")
        self.cmd_path = os.path.join(out_dir, "commands.jsonl")
        self.cmd_pos = 0
        self.lock = threading.Lock()
        self.pubs = {}
        self.periodic = {}          # key -> (publisher, msg, period, next)
        self.pending = []           # 等訂閱者出現的一次性訊息：(publisher, msg, id, deadline)
        self.last_pos = {}          # 位置每台最多 5 Hz 寫一次
        self.bodies = {}            # 假身體：機號 -> {x, y, speed, task, target, arrived, frozen, pub, msg, next, platform, path}
        self.blocks = load_blocks() # 每個平台不能進的區域（假身體繞開，2026-11-22）
        self.last_body = time.time()
        ids = cfg["vehicles"]["ids"]
        topics = cfg["topics"]
        # announced（task_announced：這台知道的任務）、path（cbba_path：這台的 bundle）給弱網測試算「每台該收到幾個任務」
        # （2026-10-27）；設定裡沒有就不訂（換成別人的 CBBA Node 時）
        # teammates（2026-11-16，itest group：CB-4 的同組角色與 gate）
        watch = ["assigned", "result", "new_task"] + \
            [k for k in ("closed", "announced", "path", "tx", "teammates") if k in topics] + \
            (["px4_position", "px4_status"] if with_px4 else []) + \
            (["px4_battery"] if with_px4 and "px4_battery" in topics else [])
        self.subs = []
        for vid in ids:
            # ns_map（2026-11-09）：comm、loss 的 --dogs 時 itest 寫的，狗的 namespace 是 v60、v60_1…
            ns = cfg["vehicles"].get("ns_map", {}).get(str(vid)) or cfg["vehicles"]["ns"].format(id=vid)
            for kind in watch:
                spec = topics[kind]
                name = spec["name"].format(ns=ns, id=vid)
                msg_type = get_message(spec["type"])
                self.subs.append(self.create_subscription(
                    msg_type, name, lambda m, k=kind, v=vid: self.on_msg(k, v, m), qos_of(spec)))
        # 機器狗（2026-10-13，demo 的假狗；2026-10-20 起可以多隻，dog.units）：
        #   robot_state  → {"kind": "dog", "dog": 機號, "x", "y", "mode"}，每隻最多 5 Hz
        #   current_task → {"kind": "dog_assigned", "dog": 機號, "msg": …}（誰拿到哪個地面任務）
        dog = cfg.get("dog", {})
        units = dog.get("units") or ([{"ns": "v60", "id": 50}] if dog.get("state_topic") else [])
        for u in units:
            if "robot_state" in topics and dog.get("state_topic"):
                self.subs.append(self.create_subscription(
                    get_message(topics["robot_state"]["type"]), dog["state_topic"].format(ns=u["ns"], id=u["id"]),
                    lambda m, d=u["id"]: self.on_dog(d, m), qos_of(topics["robot_state"])))
            if dog.get("current_topic"):
                self.subs.append(self.create_subscription(
                    get_message(topics["assigned"]["type"]), dog["current_topic"].format(ns=u["ns"], id=u["id"]),
                    lambda m, d=u["id"]: self.emit({"kind": "dog_assigned", "dog": d, "msg": message_to_ordereddict(m)}),
                    qos_of(topics["assigned"])))
        # 模擬時鐘（2026-11-05，uav、demo）：{"kind": "clock", "sim": 秒}，每 0.5 s 一筆。gazebo.mp4 用模擬時間錄，
        # 模擬比實際慢、而且快慢會變，label_gazebo.py 用這個把事件的牆上時間換成影片時間（/clock 由 gz_bridge.sh 送進 ROS）
        self.last_clock = 0.0
        if with_px4:
            self.subs.append(self.create_subscription(get_message("rosgraph_msgs/msg/Clock"), "/clock", self.on_clock,
                                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
        self.create_timer(0.02, self.poll)
        self.emit({"kind": "ready", "vehicles": ids, "px4": with_px4})

    def on_clock(self, msg):
        now = time.time()
        if now - self.last_clock >= 0.5:
            self.last_clock = now
            self.emit({"kind": "clock", "sim": msg.clock.sec + msg.clock.nanosec * 1e-9})

    def emit(self, ev):
        ev["t"] = time.time()
        with self.lock:
            self.out.write(json.dumps(ev, ensure_ascii=False) + "\n")

    def on_msg(self, kind, vid, msg):
        if kind == "px4_position":
            now = time.time()
            if now - self.last_pos.get(vid, 0.0) < 0.2:
                return
            self.last_pos[vid] = now
            spacing = float(os.environ.get("UAV_SPAWN_SPACING", self.cfg["vehicles"]["spawn_spacing"]))
            # NED（以出生點為原點）→ map ENU，和 px4_state_bridge 相同；出生原點由 itest 用 UAV_SPAWN_ORIGIN 給
            # （2026-10-24 yard 的起降板在 (−40, −30)；原本當成 (0, 0)，位置整個偏 50 m）
            ox, oy = (float(v) for v in os.environ.get("UAV_SPAWN_ORIGIN", "0,0").split(","))
            self.emit({"kind": "pos", "uav": vid,
                       "x": ox + msg.y, "y": oy + (vid - 1) * spacing + msg.x, "z": -msg.z})
            return
        if kind == "px4_battery":
            # 電量（2026-10-14，demo 動畫顯示）：每台最多 1 Hz；remaining 0～1，沒接上 / 不知道時不記
            now = time.time()
            if now - self.last_pos.get(("bat", vid), 0.0) < 1.0 or not msg.connected or msg.remaining < 0.0:
                return
            self.last_pos[("bat", vid)] = now
            self.emit({"kind": "battery", "uav": vid, "pct": round(float(msg.remaining) * 100.0, 1)})
            return
        if kind == "px4_status":
            self.emit({"kind": "status", "uav": vid, "armed": msg.arming_state == 2, "nav": msg.nav_state})
            return
        if kind == "tx":
            # cbba_node 送出位元組的累計 [全部, 重傳, 轉送]（2026-11-09，第 5 週頻寬圖），每 0.2 s 一筆，只記數字
            self.emit({"kind": "tx", "uav": vid, "d": [int(x) for x in msg.data]})
            return
        if kind == "assigned" and vid in self.bodies:
            b = self.bodies[vid]
            tid = int(msg.task_id)
            if tid == 0 or int(msg.status) == 2:
                b.update(task=0, target=None)
            else:
                target = (float(msg.position.x), float(msg.position.y))
                if tid != b["task"] or target != b["target"]:   # 新任務，或同一個任務改版換了位置
                    b.update(task=tid, target=target, arrived=False,
                             path=plan_path((b["x"], b["y"]), target, self.blocks.get(b["platform"], [])))
        self.emit({"kind": kind, "uav": vid, "msg": message_to_ordereddict(msg)})

    def on_dog(self, dog_id, msg):
        now = time.time()
        if now - self.last_pos.get(("dog", dog_id), 0.0) < 0.2:
            return
        self.last_pos[("dog", dog_id)] = now
        # mode（2026-11-07）：平板指派給狗的 M 做完 = MANUAL 回到 AUTO（假狗、伯宇的 BT 都這樣發），itest 用來刪標記
        self.emit({"kind": "dog", "dog": dog_id, "x": msg.position.x, "y": msg.position.y, "mode": int(msg.mode)})

    def publisher(self, topic, type_name, spec):
        key = (topic, type_name)
        if key not in self.pubs:
            self.pubs[key] = self.create_publisher(get_message(type_name), topic, qos_of(spec))
        return self.pubs[key]

    def build(self, type_name, fields):
        msg = get_message(type_name)()
        set_message_fields(msg, fields)
        return msg

    def poll(self):
        self.flush_pending()
        if not os.path.exists(self.cmd_path):
            self.tick_periodic()
            return
        with open(self.cmd_path, encoding="utf-8") as f:
            f.seek(self.cmd_pos)
            lines = f.readlines()
            self.cmd_pos = f.tell()
        for line in lines:
            if line.strip():
                self.run(json.loads(line))
        self.tick_periodic()

    def run(self, c):
        try:
            op = c["op"]
            if op == "pub":
                # 一次性的訊息要等訂閱者出現（剛建立的 publisher 還沒配對），最多 3 s。
                # 不能在這裡 sleep：會卡住 executor，其他話題的事件時間就不準
                pub = self.publisher(c["topic"], c["type"], c)
                self.pending.append((pub, self.build(c["type"], c["fields"]), c["id"], time.time() + 3.0))
                self.flush_pending()
            elif op == "periodic":
                pub = self.publisher(c["topic"], c["type"], c)
                self.periodic[c["key"]] = (pub, self.build(c["type"], c["fields"]), 1.0 / c["rate"], 0.0)
                self.emit({"kind": "ack", "id": c["id"]})
            elif op == "stop_periodic":
                self.periodic.pop(c["key"], None)
                self.emit({"kind": "ack", "id": c["id"]})
            elif op == "body":
                pub = self.publisher(c["topic"], c["type"], c)
                msg = self.build(c["type"], c["fields"])
                self.bodies[c["vid"]] = dict(x=float(c["x"]), y=float(c["y"]), z=float(c.get("z", 0.0)),
                                             speed=float(c["speed"]), task=0, target=None, arrived=False,
                                             frozen=False, pub=pub, msg=msg, next=0.0,
                                             platform=c.get("platform", ""), path=[])
                self.emit({"kind": "body", "uav": c["vid"], "x": c["x"], "y": c["y"], "speed": c["speed"],
                           "platform": c.get("platform", "")})
                self.emit({"kind": "ack", "id": c["id"]})
            elif op == "body_hold":
                self.bodies[c["vid"]]["frozen"] = bool(c["hold"])
                self.emit({"kind": "body_hold", "uav": c["vid"], "hold": bool(c["hold"])})
                self.emit({"kind": "ack", "id": c["id"]})
            elif op == "body_mode":
                self.bodies[c["vid"]]["msg"].mode = int(c["mode"])
                self.emit({"kind": "body_mode", "uav": c["vid"], "mode": int(c["mode"])})
                self.emit({"kind": "ack", "id": c["id"]})
            elif op == "body_off":
                self.bodies.pop(c["vid"], None)
                self.emit({"kind": "ack", "id": c["id"]})
            elif op == "note":
                self.emit({k: v for k, v in c.items() if k not in ("op", "id")} | {"kind": "note"})
                self.emit({"kind": "ack", "id": c["id"]})
            elif op == "quit":
                self.emit({"kind": "ack", "id": c["id"]})
                raise SystemExit(0)
        except SystemExit:
            raise
        except Exception as e:     # 設定寫錯（型別、欄位）時回報給 itest.py，不要整個停掉
            self.emit({"kind": "error", "id": c.get("id"), "error": repr(e)})

    def flush_pending(self):
        now = time.time()
        keep = []
        for pub, msg, cid, deadline in self.pending:
            n = pub.get_subscription_count()
            if n > 0 or now >= deadline:
                pub.publish(msg)
                self.emit({"kind": "ack", "id": cid, "subscribers": n})
            else:
                keep.append((pub, msg, cid, deadline))
        self.pending = keep

    def tick_bodies(self):
        now = time.time()
        dt = min(0.2, now - self.last_body)
        self.last_body = now
        for vid, b in self.bodies.items():
            if b["target"] is not None and not b["frozen"]:
                # 沿著繞路的點走（最後一個是任務點）；這一步走完一個點就接著走下一個
                left = b["speed"] * dt
                path = b.get("path") or [b["target"]]
                while left > 1e-9:
                    tx, ty = path[0]
                    dx, dy = tx - b["x"], ty - b["y"]
                    d = (dx * dx + dy * dy) ** 0.5
                    step = min(d, left)
                    if d > 1e-6:
                        b["x"] += dx / d * step
                        b["y"] += dy / d * step
                    left -= step
                    if d - step <= 1e-6 and len(path) > 1:
                        path.pop(0)
                    else:
                        break
                b["path"] = path
                tx, ty = b["target"]
                d, step = ((tx - b["x"]) ** 2 + (ty - b["y"]) ** 2) ** 0.5, 0.0
                if d - step <= 0.3 and not b["arrived"]:
                    b["arrived"] = True
                    self.emit({"kind": "arrived", "uav": vid, "task_id": b["task"], "x": b["x"], "y": b["y"]})
            if now >= b["next"]:
                b["next"] = now + 0.1
                b["msg"].position.x = b["x"]
                b["msg"].position.y = b["y"]
                b["pub"].publish(b["msg"])
            if now - self.last_pos.get(("body", vid), 0.0) >= 0.2:
                self.last_pos[("body", vid)] = now
                self.emit({"kind": "pos", "uav": vid, "x": round(b["x"], 3), "y": round(b["y"], 3), "z": b["z"]})

    def tick_periodic(self):
        self.tick_bodies()
        now = time.time()
        for key, (pub, msg, period, nxt) in list(self.periodic.items()):
            if now >= nxt:
                pub.publish(msg)
                self.periodic[key] = (pub, msg, period, now + period)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--px4", action="store_true", help="也訂閱 PX4 的位置與狀態（UAV 整串測試）")
    args = ap.parse_args()
    with open(args.config, encoding="utf-8") as f:
        cfg = json.load(f)
    rclpy.init()
    node = Bridge(cfg, args.dir, args.px4)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

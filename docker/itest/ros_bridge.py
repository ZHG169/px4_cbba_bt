#!/usr/bin/env python3
"""整合測試的 ROS 端（在 sim 容器執行，itest.py 會自動啟動）。

sim 容器經線材網路連得到每一台 uav 的 DDS，所以在這裡：
  - 訂閱每台的 assigned、result、new_task（和 PX4 的位置、狀態），每則寫一行到 <dir>/events.jsonl
  - 讀 <dir>/commands.jsonl（itest.py 寫入），發話題、開始／停止週期性的假 robot_state
  - 假身體（2026-11-16，itest group 的動畫；不用真的 BT）：收到 current_task 就直線走向任務點（速度由 itest 給），
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
        self.bodies = {}            # 假身體：機號 -> {x, y, speed, task, target, arrived, frozen, pub, msg, next}
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
                    b.update(task=tid, target=target, arrived=False)
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
                                             frozen=False, pub=pub, msg=msg, next=0.0)
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
                tx, ty = b["target"]
                dx, dy = tx - b["x"], ty - b["y"]
                d = (dx * dx + dy * dy) ** 0.5
                step = min(d, b["speed"] * dt)
                if d > 1e-6:
                    b["x"] += dx / d * step
                    b["y"] += dy / d * step
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

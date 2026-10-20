#!/usr/bin/env python3
"""整合測試的 ROS 端（在 sim 容器執行，itest.py 會自動啟動）。

sim 容器經線材網路連得到每一台 uav 的 DDS，所以在這裡：
  - 訂閱每台的 assigned、result、new_task（和 PX4 的位置、狀態），每則寫一行到 <dir>/events.jsonl
  - 讀 <dir>/commands.jsonl（itest.py 寫入），發話題、開始／停止週期性的假 robot_state

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
        depth=depth,
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
        ids = cfg["vehicles"]["ids"]
        topics = cfg["topics"]
        watch = ["assigned", "result", "new_task"] + (["closed"] if "closed" in topics else []) + \
            (["px4_position", "px4_status"] if with_px4 else []) + \
            (["px4_battery"] if with_px4 and "px4_battery" in topics else [])
        self.subs = []
        for vid in ids:
            ns = cfg["vehicles"]["ns"].format(id=vid)
            for kind in watch:
                spec = topics[kind]
                name = spec["name"].format(ns=ns, id=vid)
                msg_type = get_message(spec["type"])
                self.subs.append(self.create_subscription(
                    msg_type, name, lambda m, k=kind, v=vid: self.on_msg(k, v, m), qos_of(spec)))
        # 機器狗（2026-10-13，demo 的假狗；2026-10-20 起可以多隻，dog.units）：
        #   robot_state  → {"kind": "dog", "dog": 機號, "x", "y"}，每隻最多 5 Hz
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
        self.create_timer(0.02, self.poll)
        self.emit({"kind": "ready", "vehicles": ids, "px4": with_px4})

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
            spacing = self.cfg["vehicles"]["spawn_spacing"]
            # NED（以出生點為原點）→ map ENU，和 px4_state_bridge 相同
            self.emit({"kind": "pos", "uav": vid,
                       "x": msg.y, "y": (vid - 1) * spacing + msg.x, "z": -msg.z})
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
        self.emit({"kind": kind, "uav": vid, "msg": message_to_ordereddict(msg)})

    def on_dog(self, dog_id, msg):
        now = time.time()
        if now - self.last_pos.get(("dog", dog_id), 0.0) < 0.2:
            return
        self.last_pos[("dog", dog_id)] = now
        self.emit({"kind": "dog", "dog": dog_id, "x": msg.position.x, "y": msg.position.y})

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

    def tick_periodic(self):
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

#!/usr/bin/env python3
"""swarm_zenoh 的單機自我測試（2026-10-21）：不用 GCS、不用別台、不用 CBBA，一台機器（無人機、狗、筆電都可以）就能測。

  ros2 run swarm_zenoh selftest            # 預設：domain 90（當載具）＋ 91（當 GCS），埠 17447
  ros2 run swarm_zenoh selftest --count 50 # 每一項送 50 次

做法：同一台開兩個 bridge，用不同的 ROS domain 假裝是兩台機器（domain 之間 DDS 不通，只能經過 Zenoh）：
  載具 bridge（domain 90、namespace selftest）── Zenoh tcp/127.0.0.1:17447 ── GCS bridge（domain 91，router）
檢查：
  1. GCS → 載具：白名單的 /selftest/emergency_stop（std_msgs/Bool）送得到，量延遲
  2. 載具 → GCS：白名單的 /selftest/task_orphaned（只測路由：用 std_msgs/Bool 代替真正的 Task 型別）送得到
  3. 白名單外的 /selftest/not_whitelisted：**送不到**
  4. 別的 namespace 的 /other/emergency_stop：載具只轉自己的 namespace，**送不到**
中繼（2026-10-25，--relay）：再測一次「GCS ← 中繼 ← 載具」——中繼 bridge（domain 92、namespace relay1、router，往上連
GCS、在 埠＋1 聽），載具改連中繼。某一台當內網、但它是 client 時會單向或不通；中繼是 router 時兩個方向都要通：
  5. GCS → 經過中繼的載具、6. 經過中繼的載具 → GCS、7. GCS ↔ 中繼自己、8. 別的 namespace 送不到
全部符合印 PASS、回傳 0。bridge 的輸出在 /tmp/swarm_zenoh_selftest/。
預設只測直接連 GCS（2026-10-26 使用者：每台都直接連 GCS，不經過別台轉）；中繼要測時加 --only relay 或 --all
"""
import argparse
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

HERE = os.path.dirname(os.path.abspath(__file__))


def bridge_cmd():
    """zenoh_bridge（安裝後）或同目錄的 zenoh_bridge.py（原始碼）"""
    for name in ("zenoh_bridge", "zenoh_bridge.py"):
        p = os.path.join(HERE, name)
        if os.path.exists(p):
            return [sys.executable, p]
    raise SystemExit("找不到 zenoh_bridge")


class Side:
    """一個 ROS domain（假裝是一台機器）"""

    def __init__(self, domain, name):
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=domain)
        self.node = rclpy.create_node(f"swarm_zenoh_selftest_{name}", context=self.ctx)
        self.exe = SingleThreadedExecutor(context=self.ctx)
        self.exe.add_node(self.node)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            self.exe.spin_once(timeout_sec=0.01)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


def run_case(src, dst, sides, topic, count, expect):
    """src 端發 count 次，dst 端收；回傳（收到幾次, 延遲 ms 清單）"""
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    got, lat = [], []
    pub = src.node.create_publisher(Bool, topic, qos)
    sent = {}

    def cb(m):
        got.append(time.time())
    sub = dst.node.create_subscription(Bool, topic, cb, qos)
    # 等兩邊的 bridge 互相看到（新的話題要經過 DDS 探索＋Zenoh 宣告）
    t_end = time.time() + 8.0
    while time.time() < t_end and pub.get_subscription_count() == 0:
        for s in sides:
            s.spin(0.05)
    for s in sides:
        s.spin(1.0)
    for i in range(count):
        n = len(got)
        sent[i] = time.time()
        pub.publish(Bool(data=bool(i % 2)))
        t_end = time.time() + 1.0
        while time.time() < t_end and len(got) == n:
            for s in sides:
                s.spin(0.005)
        if len(got) > n:
            lat.append((got[-1] - sent[i]) * 1000.0)
    for s in sides:
        s.spin(0.5)
    src.node.destroy_publisher(pub)
    dst.node.destroy_subscription(sub)
    return len(got), lat


def start_bridges(specs, out):
    """specs：[(名稱, zenoh_bridge 的參數)]；回傳 Popen 清單（起不來回 None）"""
    procs = []
    for name, argv in specs:
        log = open(os.path.join(out, f"{name}.log"), "w")
        procs.append(subprocess.Popen(bridge_cmd() + argv + ["--out-dir", os.path.join(out, name)],
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    time.sleep(2.5)
    if any(p.poll() is not None for p in procs):
        stop_bridges(procs)
        return None
    return procs


def stop_bridges(procs):
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    for p in procs:
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


def run_cases(title, cases, sides, count):
    rows, ok = [f"[{title}]"], True
    for label, src, dst, topic, expect in cases:
        n, lat = run_case(src, dst, sides, topic, count if expect else 5, expect)
        good = (n >= count * 0.95) if expect else n == 0
        ok = ok and good
        detail = (f"收到 {n}/{count}，延遲中位數 {statistics.median(lat):.1f} ms、最大 {max(lat):.1f} ms"
                  if expect and lat else f"收到 {n} 次" + ("（應該是 0）" if not expect else ""))
        rows.append(f"  {'✓' if good else '✗'} {label}：{topic}　{detail}")
    return rows, ok


def main():
    ap = argparse.ArgumentParser(description="swarm_zenoh 單機自我測試")
    ap.add_argument("--vehicle-domain", type=int, default=90)
    ap.add_argument("--gcs-domain", type=int, default=91)
    ap.add_argument("--relay-domain", type=int, default=92)
    ap.add_argument("--port", type=int, default=17447)
    ap.add_argument("--count", type=int, default=20)
    ap.add_argument("--dds-ip", default=None, help="bridge 的 DDS 網卡（預設照 zenoh_bridge 的規則）")
    ap.add_argument("--only", choices=["direct", "relay"], default=None, help="只測其中一種拓撲（預設 direct）")
    ap.add_argument("--all", action="store_true", help="兩種拓撲都測（直接連 GCS、經過中繼）")
    args = ap.parse_args()
    if not shutil.which("zenoh-bridge-ros2dds"):
        print("FAIL：找不到 zenoh-bridge-ros2dds（先安裝）")
        return 1
    out = "/tmp/swarm_zenoh_selftest"
    os.makedirs(out, exist_ok=True)
    dds = ["--dds-ip", args.dds_ip] if args.dds_ip else []
    p0, p1 = str(args.port), str(args.port + 1)
    gcs_spec = ("gcs", ["--role", "gcs", "--domain", str(args.gcs_domain), "--listen-ip", "127.0.0.1", "--port", p0] + dds)
    topologies = {
        # 載具直接連 GCS
        "direct": [gcs_spec,
                   ("vehicle", ["--role", "vehicle", "--ns", "selftest", "--domain", str(args.vehicle_domain),
                                "--gcs-ip", "127.0.0.1", "--port", p0] + dds)],
        # GCS ← 中繼（router）← 載具
        "relay": [gcs_spec,
                  ("relay", ["--role", "vehicle", "--ns", "relay1", "--relay", "--domain", str(args.relay_domain),
                             "--gcs-ip", "127.0.0.1", "--port", p0, "--listen-ip", "127.0.0.1", "--listen-port", p1] + dds),
                  ("vehicle_via_relay", ["--role", "vehicle", "--ns", "selftest", "--domain", str(args.vehicle_domain),
                                         "--gcs-ip", "127.0.0.1", "--port", p1] + dds)],
    }
    all_rows, all_ok = [], True
    for topo in (["direct", "relay"] if args.all else [args.only or "direct"]):
        procs = start_bridges(topologies[topo], out)
        if procs is None:
            print(f"FAIL：{topo} 的 bridge 沒有起來，看 {out}/*.log")
            return 1
        try:
            veh, gcs = Side(args.vehicle_domain, f"vehicle_{topo}"), Side(args.gcs_domain, f"gcs_{topo}")
            sides = [veh, gcs]
            cases = [
                ("GCS → 載具 白名單", gcs, veh, "/selftest/emergency_stop", True),
                ("載具 → GCS 白名單", veh, gcs, "/selftest/task_orphaned", True),
                ("白名單外的話題", gcs, veh, "/selftest/not_whitelisted", False),
                ("別的 namespace", gcs, veh, "/other/emergency_stop", False),
            ]
            relay = None
            if topo == "relay":
                relay = Side(args.relay_domain, "relay")
                sides.append(relay)
                cases += [("GCS → 中繼自己", gcs, relay, "/relay1/emergency_stop", True),
                          ("中繼自己 → GCS", relay, gcs, "/relay1/task_orphaned", True)]
            rows, ok = run_cases("直接連 GCS" if topo == "direct" else "GCS ← 中繼 ← 載具", cases, sides, args.count)
            all_rows += rows
            all_ok = all_ok and ok
            for side in sides:
                side.close()
        finally:
            stop_bridges(procs)
    print("\n".join(all_rows))
    print("PASS" if all_ok else f"FAIL（bridge 的 log 在 {out}/）")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

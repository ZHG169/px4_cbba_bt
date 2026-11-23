#!/usr/bin/env python3
"""假的 VLM（itest group，L5＋L6 C 批，2026-11-23）：在 cbba_gcs 跑，回覆 event_manager 的驗證請求。

訂 /gcs/vlm_request（swarm_interfaces/VlmRequest）→ 發 /gcs/vlm_result（VlmResult）。回什麼由 --overrides 的 JSON 決定
（每次收到請求都重讀，測試可以中途改）：
    {"perimeter_intrusion": {"verdict": "uncertain", "confidence": 0.5}, "*": {...}}
依事件類型找，沒有時用 "*"，再沒有就回 confirmed 0.95。每個請求印一行（itest 從 log 查哪個事件送過 VLM）。
VLM 模型本身不在這次範圍（docs/l5_l6_plan.md 第 5 節）；這裡只是讓流程（含 2-1 案例 3 的中信心）測得到。
"""
import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from swarm_interfaces.msg import VlmRequest, VlmResult


class FakeVlm(Node):
    def __init__(self, overrides, ns):
        super().__init__("fake_vlm")
        self.overrides = overrides
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(VlmResult, f"{ns}/vlm_result", qos)
        self.create_subscription(VlmRequest, f"{ns}/vlm_request", self.on_request, qos)

    def answer(self, event_type):
        try:
            with open(self.overrides, encoding="utf-8") as f:
                table = json.load(f)
        except (OSError, ValueError):
            table = {}
        a = table.get(event_type) or table.get("*") or {}
        return a.get("verdict", "confirmed"), float(a.get("confidence", 0.95))

    def on_request(self, req):
        verdict, conf = self.answer(req.event_type)
        m = VlmResult()
        m.event_id = req.event_id
        m.verdict = verdict
        m.confidence = conf
        m.evidence = f"fake_vlm {req.prompt_id}"
        self.pub.publish(m)
        print(f"fake_vlm {req.event_id} {req.event_type} -> {verdict} {conf:.2f}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--overrides", default="")
    ap.add_argument("--ns", default="/gcs")
    args = ap.parse_args()
    rclpy.init()
    node = FakeVlm(args.overrides, args.ns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()

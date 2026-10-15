#!/usr/bin/env python3
"""假的 V60（2026-10-13）：代替伯宇的 task_manager，讓「無人機發布地面處置任務 → 狗競標 → 狗走過去處理 → 任務結束」
在 Gazebo 裡跑得完。在 sim 容器執行（要看得到狗的 cbba_node 的 DDS，也要能呼叫 Gazebo 的服務）。

  ROS（和伯宇的 task_manager 相同的介面，在 /<ns> 底下）
    發 robot_state   5 Hz：位置（map ENU）、電量、mode = AUTO
    收 current_task  task_id ≠ 0：往任務點走（cruise 公尺／秒，直線、平地），到了停留 duration_sec，回報 task_result 成功；
                     中途換任務就改走新的（搶占）；task_id = 0：原地停下
    發 task_result   成功（reason NONE）
  Gazebo
    生成一個橘色方塊（約 V60 大小）當替身，每 0.1 s 用 /world/<world>/set_pose 移到目前位置

狗的 cbba_node 另外啟動（fake_dog.sh 在 uav 容器啟動，機號 50、agent_type UGV，namespace v60）。
用法：python3 fake_dog.py --world demo_site --start -4 0 [--speed 1.0] [--ns v60]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from swarm_interfaces.msg import RobotState, Task, TaskResult

try:
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node as GzNode
except ImportError:   # 沒有 Gazebo 的 python（不在 sim 容器）：只跑 ROS 的部分
    GzNode = None

BOX_SDF = """<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>
<link name="body"><visual name="v"><geometry><box><size>0.9 0.45 0.5</size></box></geometry>
<material><ambient>1 0.45 0 1</ambient><diffuse>1 0.45 0 1</diffuse></material></visual></link></model></sdf>"""


class FakeDog(Node):
    def __init__(self, args):
        super().__init__("v60_fake_dog")
        self.args = args
        self.x, self.y = args.start
        self.yaw = 0.0
        self.task = None          # 目前的 current_task
        self.arrived_at = None    # 到達任務點的時刻
        ns = "/" + args.ns
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(RobotState, ns + "/robot_state", 10)
        self.result_pub = self.create_publisher(TaskResult, ns + "/task_result", 20)
        self.create_subscription(Task, ns + "/current_task", self.on_task, latched)
        self.create_timer(0.1, self.step)
        self.create_timer(0.2, self.publish_state)
        self.gz = None
        if GzNode is not None and args.world:
            self.gz = GzNode()
            self.spawn()
        self.get_logger().info(f"假 V60：{ns}，起點 ({self.x:.1f}, {self.y:.1f})，{args.speed:.1f} m/s" +
                               ("" if self.gz else "（沒有 Gazebo 替身）"))

    # ---------------------------------------------------------- Gazebo
    def spawn(self):
        req = EntityFactory()
        req.sdf = BOX_SDF.format(name=self.args.model)
        req.name = self.args.model
        req.allow_renaming = False
        req.pose.position.x, req.pose.position.y, req.pose.position.z = self.x, self.y, 0.25
        ok, rep = self.gz.request(f"/world/{self.args.world}/create", req, EntityFactory, Boolean, 3000)
        if not (ok and rep.data):
            self.get_logger().warn("Gazebo 替身沒有生成（可能已經有了）")

    def move_model(self):
        if self.gz is None:
            return
        req = Pose()
        req.name = self.args.model
        req.position.x, req.position.y, req.position.z = self.x, self.y, 0.25
        req.orientation.z, req.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.gz.request(f"/world/{self.args.world}/set_pose", req, Pose, Boolean, 200)

    # ---------------------------------------------------------- ROS
    def on_task(self, msg):
        if msg.task_id == 0:
            if self.task is not None:
                self.get_logger().info(f"任務 {self.task.task_id:08X} 撤銷：停下")
            self.task, self.arrived_at = None, None
            return
        if self.task is not None and self.task.task_id == msg.task_id:
            return
        if self.task is not None:
            self.get_logger().info(f"中斷 {self.task.task_id:08X}，改去 {msg.task_id:08X}")
        self.task, self.arrived_at = msg, None
        d = math.hypot(msg.position.x - self.x, msg.position.y - self.y)
        self.get_logger().info(f"接到 {msg.task_id:08X}：往 ({msg.position.x:.1f}, {msg.position.y:.1f})，"
                               f"{d:.1f} m，停留 {msg.duration_sec:.0f} s")

    def publish_state(self):
        m = RobotState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.agent_id = self.args.id
        m.position.x, m.position.y = self.x, self.y
        m.battery = float(self.args.battery)
        m.mode = RobotState.AUTO
        self.state_pub.publish(m)

    def step(self):
        dt = 0.1
        t = self.task
        if t is not None:
            dx, dy = t.position.x - self.x, t.position.y - self.y
            d = math.hypot(dx, dy)
            if self.arrived_at is None:
                if d <= 0.3:
                    self.arrived_at = time.monotonic()
                    self.get_logger().info(f"到達 {t.task_id:08X}，處置 {t.duration_sec:.0f} s")
                else:
                    s = min(self.args.speed * dt, d)
                    self.x += dx / d * s
                    self.y += dy / d * s
                    self.yaw = math.atan2(dy, dx)
            elif time.monotonic() - self.arrived_at >= t.duration_sec:
                r = TaskResult()
                r.task_id = t.task_id
                r.success = True
                r.reason = TaskResult.NONE
                self.result_pub.publish(r)
                self.get_logger().info(f"完成 {t.task_id:08X}：回報 task_result")
                self.task, self.arrived_at = None, None
        self.move_model()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="v60")
    ap.add_argument("--id", type=int, default=50)
    ap.add_argument("--start", type=float, nargs=2, default=[-4.0, 0.0])
    ap.add_argument("--speed", type=float, default=1.0, help="和狗的 cbba_node 的 cruise_speed 一致")
    ap.add_argument("--battery", type=float, default=100.0)
    ap.add_argument("--world", default="", help="Gazebo 世界名稱（空的 = 不放替身）")
    ap.add_argument("--model", default="v60_standin")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = FakeDog(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

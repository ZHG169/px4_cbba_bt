#!/usr/bin/env python3
"""狗的身體（2026-10-27）：用 Gazebo 的方塊代替 V60＋Nav2＋雲台，讓伯宇的狗 BT（task_manager）照原樣跑。
和 fake_dog.py 不同：這裡**沒有 BT 的邏輯**（current_task、task_result、robot_state、manual_override 都由 task_manager 處理），
只提供它要的三樣東西。在 sim 容器執行（要能呼叫 Gazebo 的服務；DDS 和 uav1 的 task_manager 互通）。

  NavigateToPose action  /<ns>/navigate_to_pose（task_manager 的 nav_action）：收到目標就直線走過去（--speed，平地、不繞障礙），
                         0.2 m 內回成功；cancel 立刻停下回 CANCELED；新目標蓋掉舊的（舊的回 ABORTED）
  PtzInspect action      /<ns>/ptz_inspect（ptz_action）：等 --ptz-sec 秒回 OK（不拍照、image_paths 空的）；cancel 回 CANCELLED
  TF                     map → <ns>/base_link，20 Hz（task_manager 用 TF 讀位置、發 robot_state，base_frame 要設成 <ns>/base_link）
  Gazebo                 橘色方塊（和 fake_dog.py 同一個），每 0.1 s set_pose

只驗證流程與 BT（中斷、回報、平板接管、急停）；Nav2 的規劃、繞障礙、走路的時間不在這裡（見 docs/nav2_integration_plan.md）。
用法：python3 fake_nav_body.py --world demo_site --ns v60 --start -4 0 [--speed 1.5] [--ptz-sec 5]
要 source ros2_ws/install 和 dog_ws/install（task_manager_interfaces 的 PtzInspect，docker/scripts/dog_bt.sh build 產生）。
"""
import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from task_manager_interfaces.action import PtzInspect
from tf2_msgs.msg import TFMessage

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

REACH = 0.2   # 到達的距離（m）


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FakeNavBody(Node):
    def __init__(self, args):
        super().__init__(f"{args.ns}_fake_body")
        self.args = args
        self.lock = threading.Lock()
        self.x, self.y = args.start
        self.yaw = 0.0
        self.target = None        # (x, y, yaw)：目前的導航目標；None = 停著
        self.nav_seq = 0          # 每個新目標 + 1，舊的 execute 看到不一樣就結束（被新目標蓋掉）
        self.frame = f"{args.ns}/base_link"
        cb = ReentrantCallbackGroup()
        ns = "/" + args.ns
        self.nav = ActionServer(self, NavigateToPose, ns + "/navigate_to_pose", execute_callback=self.nav_execute,
                                goal_callback=lambda _: GoalResponse.ACCEPT,
                                cancel_callback=self.on_cancel, callback_group=cb)
        self.ptz = ActionServer(self, PtzInspect, ns + "/ptz_inspect", execute_callback=self.ptz_execute,
                                goal_callback=lambda _: GoalResponse.ACCEPT,
                                cancel_callback=self.on_cancel, callback_group=cb)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.create_timer(0.1, self.step)
        self.create_timer(0.05, self.publish_tf)
        self.gz = None
        if GzNode is not None and args.world:
            self.gz = GzNode()
            self.spawn()
        self.get_logger().info(f"狗的身體（方塊）：{ns}/navigate_to_pose、{ns}/ptz_inspect、TF map → {self.frame}，"
                               f"起點 ({self.x:.1f}, {self.y:.1f})，{args.speed:.1f} m/s，雲台 {args.ptz_sec:.0f} s" +
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

    # ---------------------------------------------------------- 身體
    def step(self):
        dt = 0.1
        with self.lock:
            if self.target is not None:
                tx, ty, tyaw = self.target
                dx, dy = tx - self.x, ty - self.y
                d = math.hypot(dx, dy)
                if d > REACH:
                    s = min(self.args.speed * dt, d)
                    self.x += dx / d * s
                    self.y += dy / d * s
                    self.yaw = math.atan2(dy, dx)
                else:
                    self.yaw = tyaw      # 到了轉向目標的方向（task_manager 的外部點面向目標）
        self.move_model()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = self.frame
        with self.lock:
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation.z, t.transform.rotation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.tf_pub.publish(TFMessage(transforms=[t]))

    # ---------------------------------------------------------- action
    def on_cancel(self, _goal):
        self.get_logger().info(f"收到 cancel（{time.time():.3f}）")   # 對照 task_manager 的「★ halt → cancel」
        return CancelResponse.ACCEPT

    def nav_execute(self, gh):
        p = gh.request.pose.pose
        with self.lock:
            self.nav_seq += 1
            my = self.nav_seq
            self.target = (p.position.x, p.position.y, yaw_of(p.orientation))
            d0 = math.hypot(p.position.x - self.x, p.position.y - self.y)
        self.get_logger().info(f"導航 #{my}：往 ({p.position.x:.1f}, {p.position.y:.1f})，{d0:.1f} m")
        result = NavigateToPose.Result()
        last_fb = 0.0
        while rclpy.ok():
            with self.lock:
                superseded = self.nav_seq != my
                d = math.hypot(self.target[0] - self.x, self.target[1] - self.y) if self.target else 0.0
                if gh.is_cancel_requested and not superseded:
                    self.target = None
            if superseded:               # 新目標蓋掉（Nav2 也是讓舊的結束）
                result.error_msg = "preempted by a new goal"
                gh.abort()
                return result
            if gh.is_cancel_requested:
                self.get_logger().info(f"導航 #{my} 取消：停在 ({self.x:.1f}, {self.y:.1f})")
                gh.canceled()
                return result
            if d <= REACH:
                with self.lock:
                    self.target = None
                self.get_logger().info(f"導航 #{my} 到達")
                gh.succeed()
                return result
            now = time.monotonic()
            if now - last_fb >= 0.5:
                fb = NavigateToPose.Feedback()
                fb.distance_remaining = float(d)
                gh.publish_feedback(fb)
                last_fb = now
            time.sleep(0.05)
        gh.abort()
        return result

    def ptz_execute(self, gh):
        g = gh.request
        wait = self.args.ptz_sec if g.timeout <= 0.0 else min(self.args.ptz_sec, float(g.timeout))
        self.get_logger().info(f"雲台：看 ({g.target.point.x:.1f}, {g.target.point.y:.1f})，{wait:.0f} s")
        result = PtzInspect.Result()
        end = time.monotonic() + wait
        while rclpy.ok() and time.monotonic() < end:
            if gh.is_cancel_requested:
                result.error_code = PtzInspect.Result.CANCELLED
                gh.canceled()
                return result
            time.sleep(0.05)
        result.error_code = PtzInspect.Result.OK
        self.get_logger().info("雲台：完成")
        gh.succeed()
        return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="v60")
    ap.add_argument("--start", type=float, nargs=2, default=[-4.0, 0.0])
    ap.add_argument("--speed", type=float, default=1.5, help="和狗的 cbba_node 的 cruise_speed 一致")
    ap.add_argument("--ptz-sec", type=float, default=5.0, help="雲台檢查（到點後的處置）要幾秒")
    ap.add_argument("--world", default="", help="Gazebo 世界名稱（空的 = 不放替身）")
    ap.add_argument("--model", default="v60_standin")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = FakeNavBody(args)
    ex = MultiThreadedExecutor(num_threads=4)   # 兩個 action 的 execute 會一直跑，要多執行緒
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

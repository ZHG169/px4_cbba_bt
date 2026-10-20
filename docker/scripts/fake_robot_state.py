#!/usr/bin/env python3
"""假的 robot_state（手動測試用，2026-10-19）：代替狗的 BT 送狀態，mode 從檔案讀，切換時不用重開。

用 `ros2 topic pub -r 2` 切換 mode 要重開 publisher（1～2 s），空檔超過 cbba_node 的 state_timeout（1.5 s），
會變成「逾時釋放」而不是「MANUAL 釋放」，所以 mode 改從檔案讀（和 swarm_cbba/test/g2_e2e.sh 的 rs.py 同一種做法）。

用法（和 cbba_node 同一個容器，source 過 ros2_ws/install/setup.bash）：
  python3 fake_robot_state.py --ns v60 --id 50 --pos 0 -10            # mode 檔預設 /tmp/fake_mode_v60
  echo 1 > /tmp/fake_mode_v60     # MANUAL（平板接管）
  echo 2 > /tmp/fake_mode_v60     # ESTOP（急停）
  echo 0 > /tmp/fake_mode_v60     # AUTO
"""
import argparse

import rclpy
from swarm_interfaces.msg import RobotState


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="v60")
    ap.add_argument("--id", type=int, default=50)
    ap.add_argument("--pos", type=float, nargs=2, default=[0.0, 0.0], metavar=("X", "Y"))
    ap.add_argument("--battery", type=float, default=90.0)
    ap.add_argument("--mode-file", default="", help="預設 /tmp/fake_mode_<ns>")
    ap.add_argument("--rate", type=float, default=2.0)
    args = ap.parse_args()
    mode_file = args.mode_file or f"/tmp/fake_mode_{args.ns}"
    try:
        open(mode_file, "x").write("0\n")   # 沒有就建一個 AUTO 的
    except FileExistsError:
        pass

    rclpy.init()
    node = rclpy.create_node(f"fake_robot_state_{args.ns}")
    pub = node.create_publisher(RobotState, f"/{args.ns}/robot_state", 10)
    names = {0: "AUTO", 1: "MANUAL", 2: "ESTOP"}
    last = [None]

    def tick():
        try:
            mode = int(open(mode_file).read().strip() or 0)
        except (OSError, ValueError):
            mode = 0
        if mode != last[0]:
            print(f"[fake_robot_state] /{args.ns}/robot_state mode = {mode}（{names.get(mode, '?')}）", flush=True)
            last[0] = mode
        m = RobotState()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.agent_id = args.id
        m.position.x, m.position.y = args.pos
        m.battery = args.battery
        m.mode = mode
        pub.publish(m)

    node.create_timer(1.0 / args.rate, tick)
    print(f"[fake_robot_state] /{args.ns}/robot_state：agent {args.id}、位置 ({args.pos[0]}, {args.pos[1]})、"
          f"mode 檔 {mode_file}", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""看 PX4 的 vehicle_status 裡和「接管／失效保護」有關的欄位，只在改變時印一行（2026-10-17）。

用途：確認被遙控器／QGC 接管和 PX4 失效保護分得出來（nav_state_user_intention、failsafe、
failsafe_and_user_took_over），之後 BT 被接管時回報 PREEMPTED_BY_MANUAL、故障時回報 STOPPED。

用法（uav 容器，source 過 ros2_ws/install/setup.bash）：
  python3 /home/ncrl/CBBA_BT/docker/scripts/watch_px4_status.py 1      # uav1
"""
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleStatus

NAV = {v: k[len("NAVIGATION_STATE_"):] for k, v in vars(VehicleStatus).items()
       if k.startswith("NAVIGATION_STATE_") and isinstance(v, int)}


def main():
    uav = sys.argv[1] if len(sys.argv) > 1 else "1"
    topic = f"/uav{uav}/fmu/out/vehicle_status_v1"
    rclpy.init()
    node = rclpy.create_node(f"watch_px4_status_{uav}")
    last = [None]
    t0 = time.monotonic()

    def on_status(m):
        key = (m.arming_state, m.nav_state, m.nav_state_user_intention, m.failsafe,
               m.failsafe_and_user_took_over, m.executor_in_charge)
        if key == last[0]:
            return
        last[0] = key
        print(f"{time.monotonic() - t0:7.2f} s  armed={int(m.arming_state == VehicleStatus.ARMING_STATE_ARMED)}"
              f"  nav_state={NAV.get(m.nav_state, m.nav_state):<12}"
              f"  user_intention={NAV.get(m.nav_state_user_intention, m.nav_state_user_intention):<12}"
              f"  failsafe={int(m.failsafe)}  user_took_over={int(m.failsafe_and_user_took_over)}"
              f"  executor={m.executor_in_charge}", flush=True)

    node.create_subscription(VehicleStatus, topic, on_status, qos_profile_sensor_data)
    print(f"訂閱 {topic}（只在改變時印；Ctrl-C 結束）", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

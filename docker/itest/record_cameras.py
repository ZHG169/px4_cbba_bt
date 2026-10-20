#!/usr/bin/env python3
"""錄下各台無人機的下視相機畫面（掃 AprilTag 的那台，/uavN/camera/image_raw），給 itest demo 合成 demo_cameras.mp4（2026-10-19）。

在 sim 容器跑（gz_bridge.sh 在那裡把 Gazebo 的相機轉成 ROS 話題；容器裡有 PIL、沒有 OpenCV）：
  python3 record_cameras.py <itest 輸出目錄> --ids 1 2 3 [--fps 5] [--width 426]
每台每秒最多存 --fps 張，縮成 --width 寬的 JPEG：<輸出目錄>/cameras/uavN/<收到的時間 ms>.jpg。
檔名用收到的牆上時間（和 itest 記的 Gazebo 錄影開始、結束時間同一個時鐘），合成時照時間對齊。Ctrl-C／SIGTERM 結束。
"""
import argparse
import os
import signal
import time

import numpy as np
import rclpy
from PIL import Image
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage

CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def to_pil(msg):
    ch = CHANNELS.get(msg.encoding)
    if ch is None:
        return None
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    a = a.reshape(msg.height, msg.width, ch) if ch > 1 else a.reshape(msg.height, msg.width)
    if msg.encoding.startswith("bgr"):
        a = a[:, :, [2, 1, 0]]
    elif ch == 4:
        a = a[:, :, :3]
    return Image.fromarray(np.ascontiguousarray(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--width", type=int, default=426)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("itest_record_cameras")
    last = {}
    counts = {v: 0 for v in args.ids}

    def make_cb(v):
        folder = os.path.join(args.out_dir, "cameras", f"uav{v}")
        os.makedirs(folder, exist_ok=True)

        def cb(msg):
            now = time.time()
            if now - last.get(v, 0.0) < 1.0 / args.fps:
                return
            img = to_pil(msg)
            if img is None:
                return
            last[v] = now
            h = round(img.height * args.width / img.width)
            img.resize((args.width, h)).save(os.path.join(folder, f"{int(now * 1000):013d}.jpg"), quality=80)
            counts[v] += 1
        return cb

    for v in args.ids:
        node.create_subscription(RosImage, f"/uav{v}/camera/image_raw", make_cb(v), qos_profile_sensor_data)
    print(f"[record_cameras] 錄 {', '.join(f'/uav{v}/camera/image_raw' for v in args.ids)}，每秒 {args.fps:g} 張", flush=True)

    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    try:
        while rclpy.ok() and not stop["now"]:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    print(f"[record_cameras] 結束：" + "、".join(f"uav{v} {counts[v]} 張" for v in args.ids), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""由 worlds/patrol_site_points.csv 產生 worlds/patrol_site.sdf（巡檢＋隨機插入測試場景，2026-10-10）。

綠色平台（G）一開始就貼著 tag；灰色方塊（R，2026-10-12 前是紅色）先不貼，插入時由 patrol_scenario.sh 用 Gazebo 的
/world/patrol_site/create 生成 tag（Gazebo 執行中不容易切換模型的顯示，生成、刪除比較簡單）。
tag 的高度要和 patrol_scenario.sh 的 TAG_Z_RED 一致。
火警是另外生成的紅色方塊（models/fire_block）＋tag 101，起火時才出現（patrol_scenario.sh fire-at），場景裡沒有紅色。

用法：python3 docker/gz/gen_patrol_site.py              patrol_site（預設）
      python3 docker/gz/gen_patrol_site.py demo_site    worlds/demo_site_points.csv → worlds/demo_site.sdf
                                                        （展示用的小場景：點都在 ±22 m 內，鏡頭拉近時無人機看得清楚）
"""
import argparse
import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent

GREEN = "0.1 0.7 0.2 1"
GRAY = "0.55 0.55 0.55 1"
PAD_H = 0.05      # 綠色平台高度
BOX_H = 0.3       # 灰色方塊高度（tag 貼在頂面上方 2 mm，避免 z-fighting）


def read_points(path):
    with path.open(encoding="utf-8") as f:
        rows = [line for line in f if line.strip() and not line.startswith("#")]
    return list(csv.DictReader(rows))


def block(name, x, y, size, height, color):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {height / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size} {size} {height}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{size} {size} {height}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse><specular>0.1 0.1 0.1 1</specular></material>
        </visual>
      </link>
    </model>
"""


def tag(name, tag_id, x, y, z):
    return f"""    <include>
      <uri>model://tag36h11_{tag_id}</uri>
      <name>{name}</name>
      <pose>{x} {y} {z:.3f} 0 0 0</pose>
    </include>
"""


# 俯瞰錄影攝影機（2026-10-13）：視窗的 VideoRecorder 按鈕在 gz-sim 8＋MinimalScene 沒有人提供 /gui/record_video 服務，
# 錄不到。改成場景裡的固定攝影機＋CameraVideoRecorder 系統，由伺服器自己錄（不用開視窗也能錄）：
#   gz service -s /<world>/overview/record_video --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean \
#     --timeout 5000 --req 'start: true, format: "mp4", save_filename: "/某處/gazebo.mp4"'     （停止：stop: true）
# 位置和視窗的俯瞰鏡頭相同（px4_sitl.sh 的 CAMERA_POSE：西南方往東北看）
# demo_site 2026-10-14 改成正上方往下看（俯視，影像上方 = 北 +y、右邊 = 東 +x）：pitch π/2 往下、yaw π/2 讓影像上方朝北；
# 30 m 高、水平視角 1.4 rad（80°）約看 50 m × 28 m（x −24～26、y −15.7～12.7），點都在 ±22 m 內；x500 約 18 px
# 2026-10-19 使用者：拉高到 35 m，約看 59 m × 33 m（x −28.5～30.5、y −18～15），x500 約 15 px
# 2026-10-19 同日：透視的視差讓 5 m 高的無人機看起來沒停在點上（偏移 ≈ 離畫面中心的水平距離 × 5 ／（鏡頭高度 − 5）：
#   35 m 時 R4 處 1.1 m、場地邊緣 3.3 m）。改成 150 m 高＋窄視角（像望遠鏡頭），拍到的範圍一樣 59 m × 33 m，
#   視差降到 R4 處 0.22 m、邊緣 0.7 m。視角 = 2 × atan(29.5 / 150) = 0.389 rad
OVERVIEW_POSE = {"demo_site": "1 -1.5 150 0 1.5708 1.5708", "patrol_site": "-60 -55 55 0 0.57 0.76"}
OVERVIEW_FOV = {"demo_site": 0.389}

# 地面網格（2026-10-19）：視窗的網格是 GUI 外掛（docker/gz/gui/cbba_gui.config 的 GridConfig），只畫在視窗裡，
# 伺服器的錄影鏡頭看不到。要錄到網格，改成場景裡貼地的細長方塊（只有 visual、沒有 collision，不影響物理）。
# 間距、範圍和視窗的網格一樣：2 m 一格、往四個方向各 30 格。線寬 6 cm：35 m 高俯視時約 1 px 寬
# （太細會時有時無），無人機 5 m 高的下視相機看到的是細灰線，不會被當成 AprilTag。
# 場景名稱 → (格子邊長 m, 每個方向幾格, 線寬 m)；沒列的場景不加
GRID = {"demo_site": (2.0, 30, 0.06)}
GRID_COLOR = "0.35 0.35 0.35 1"


def grid_lines(world):
    if world not in GRID:
        return ""
    cell, count, width = GRID[world]
    length = 2 * cell * count
    visuals = []
    for i in range(-count, count + 1):
        c = i * cell
        for axis, x, y, sx, sy in (("x", c, 0.0, width, length), ("y", 0.0, c, length, width)):
            visuals.append(f"""        <visual name="grid_{axis}{i + count}">
          <pose>{x:g} {y:g} 0 0 0 0</pose>
          <geometry><box><size>{sx:g} {sy:g} 0.002</size></box></geometry>
          <material><ambient>{GRID_COLOR}</ambient><diffuse>{GRID_COLOR}</diffuse></material>
        </visual>
""")
    return f"""    <model name="ground_grid">
      <static>true</static>
      <pose>0 0 0.005 0 0 0</pose>
      <link name="link">
{"".join(visuals)}      </link>
    </model>
"""


def overview_camera(world):
    pose = OVERVIEW_POSE.get(world, "-30 -30 30 0 0.6 0.785")
    return f"""    <model name="overview_camera">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <sensor name="overview" type="camera">
          <always_on>true</always_on>
          <update_rate>15</update_rate>
          <topic>/{world}/overview/image</topic>
          <camera>
            <horizontal_fov>{OVERVIEW_FOV.get(world, 1.05)}</horizontal_fov>
            <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>500</far></clip>
          </camera>
          <plugin filename="gz-sim-camera-video-recorder-system" name="gz::sim::systems::CameraVideoRecorder">
            <service>/{world}/overview/record_video</service>
            <use_sim_time>true</use_sim_time>
            <fps>15</fps>
            <bitrate>4000000</bitrate>
          </plugin>
        </sensor>
      </link>
    </model>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world", nargs="?", default="patrol_site")
    world = ap.parse_args().world
    points_csv = HERE / "worlds" / f"{world}_points.csv"
    out = HERE / "worlds" / f"{world}.sdf"
    points = read_points(points_csv)
    body = []
    for p in points:
        name, x, y, tag_id = p["point"], p["x"], p["y"], p["tag"]
        if name.startswith("G"):
            body.append(block(f"pad_{name}", x, y, 1.5, PAD_H, GREEN))
            body.append(tag(f"tag_{name}", tag_id, x, y, PAD_H + 0.002))
        else:
            body.append(block(f"box_{name}", x, y, 1.4, BOX_H, GRAY))
    listing = "\n".join(f"    {p['point']}  ({p['x']}, {p['y']})  tag {p['tag']}" for p in points)
    out.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<!--
  巡檢＋隨機插入測試場景（docs/sim_scenario_plan.md）。由 docker/gz/gen_patrol_site.py 從
  {world}_points.csv 產生，不要直接改這個檔案。
  綠色平台 G＝一開始就知道的巡檢任務（tag 一直貼著）；灰色方塊 R＝隨機插入的候選（插入時 patrol_scenario.sh 生成 tag）。
  火警（紅色方塊＋tag 101）起火時才由 patrol_scenario.sh fire-at 生成。
{listing}
  座標 = map ENU。用法：在 sim 容器 CBBA_WORLD={world} px4_sitl.sh 1
-->
<sdf version="1.9">
  <world name="{world}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.62 0.78 0.93 1</background>
      <shadows>false</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.8 0.8 0.8 1</specular></material>
        </visual>
      </link>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

{overview_camera(world)}{grid_lines(world)}{"".join(body)}  </world>
</sdf>
""", encoding="utf-8")
    print(f"[gen_patrol_site] {len(points)} 個點 -> {out}")


if __name__ == "__main__":
    main()

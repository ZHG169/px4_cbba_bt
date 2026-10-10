#!/usr/bin/env python3
"""由 worlds/patrol_site_points.csv 產生 worlds/patrol_site.sdf（巡檢＋隨機插入測試場景，2026-10-10）。

綠色平台（G）一開始就貼著 tag；紅色方塊（R）先不貼，插入時由 patrol_scenario.sh 用 Gazebo 的
/world/patrol_site/create 生成 tag（Gazebo 執行中不容易切換模型的顯示，生成、刪除比較簡單）。
tag 的高度要和 patrol_scenario.sh 的 TAG_Z_RED 一致。

用法：python3 docker/gz/gen_patrol_site.py
"""
import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent
POINTS = HERE / "worlds" / "patrol_site_points.csv"
OUT = HERE / "worlds" / "patrol_site.sdf"

GREEN = "0.1 0.7 0.2 1"
RED = "0.8 0.1 0.1 1"
PAD_H = 0.05      # 綠色平台高度
BOX_H = 0.3       # 紅色方塊高度（tag 貼在頂面上方 2 mm，避免 z-fighting）


def read_points():
    with POINTS.open(encoding="utf-8") as f:
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


def main():
    points = read_points()
    body = []
    for p in points:
        name, x, y, tag_id = p["point"], p["x"], p["y"], p["tag"]
        if name.startswith("G"):
            body.append(block(f"pad_{name}", x, y, 1.5, PAD_H, GREEN))
            body.append(tag(f"tag_{name}", tag_id, x, y, PAD_H + 0.002))
        else:
            body.append(block(f"box_{name}", x, y, 1.4, BOX_H, RED))
    listing = "\n".join(f"    {p['point']}  ({p['x']}, {p['y']})  tag {p['tag']}" for p in points)
    OUT.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<!--
  巡檢＋隨機插入測試場景（docs/sim_scenario_plan.md）。由 docker/gz/gen_patrol_site.py 從
  patrol_site_points.csv 產生，不要直接改這個檔案。
  綠色平台 G＝一開始就知道的巡檢任務（tag 一直貼著）；紅色方塊 R＝隨機插入的候選（插入時 patrol_scenario.sh 生成 tag）。
{listing}
  座標 = map ENU。用法：在 sim 容器 CBBA_WORLD=patrol_site px4_sitl.sh 1
-->
<sdf version="1.9">
  <world name="patrol_site">
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
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
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
      <cast_shadows>true</cast_shadows>
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

{"".join(body)}  </world>
</sdf>
""", encoding="utf-8")
    print(f"[gen_patrol_site] {len(points)} 個點 -> {OUT}")


if __name__ == "__main__":
    main()

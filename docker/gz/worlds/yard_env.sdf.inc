<!--
  yard（變電所）的靜態物件，gen_patrol_site.py yard 會把這段原樣插進 worlds/yard.sdf（2026-10-24）。
  來源：伯宇 2026-10-12 的 ros2_ws/src/world/yard.sdf（狗端的世界，他用 scripts/make_yard.py 從 yard_layout.yaml 產生）；
  只取圍牆、控制室、變壓器三排、管架、鋼構棚、閥門站、起降板、儲槽，座標、尺寸、顏色照原樣。和原檔不同的地方：
    - 地面、燈光、物理、外掛、spherical_coordinates 用我們場景的共用設定（gen_patrol_site.py；物理 ode 4 ms，
      原檔 dart 2 ms 是狗的 WBC 要 500 Hz，我們只有假狗）
    - 拿掉固定的火情 tag 101（30, −10）：火改成 itest 起火時才生成（yard_points.csv 的 R1 就在那裡）
    - 拿掉閥門 tag 102（直立在棚內，給狗的相機；下視相機看不到）
  他更新場景時：把原檔 fence_n 到 tank_2 的 model 換進來。
-->
    <model name="fence_n"><static>true</static><pose>0 40 1.0 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>100 0.3 2.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>100 0.3 2.0</size></box></geometry>
        <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material></visual>
    </link></model>
    <model name="fence_s_w"><static>true</static><pose>-30 -40 1.0 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>40 0.3 2.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>40 0.3 2.0</size></box></geometry>
        <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material></visual>
    </link></model>
    <model name="fence_s_e"><static>true</static><pose>30 -40 1.0 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>40 0.3 2.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>40 0.3 2.0</size></box></geometry>
        <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material></visual>
    </link></model>
    <model name="fence_w"><static>true</static><pose>-50 0 1.0 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>0.3 80 2.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.3 80 2.0</size></box></geometry>
        <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material></visual>
    </link></model>
    <model name="fence_e"><static>true</static><pose>50 0 1.0 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>0.3 80 2.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.3 80 2.0</size></box></geometry>
        <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material></visual>
    </link></model>
    <model name="control_room"><static>true</static><pose>-38 28 2.0 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>12 8 4.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>12 8 4.0</size></box></geometry>
        <material><ambient>0.75 0.67 0.55 1</ambient><diffuse>0.75 0.67 0.55 1</diffuse></material></visual>
    </link></model>
    <model name="TA1"><static>true</static><pose>-30 15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TA2"><static>true</static><pose>-20 15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TA3"><static>true</static><pose>-10 15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TA4"><static>true</static><pose>0 15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TA5"><static>true</static><pose>10 15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TB1"><static>true</static><pose>-30 0 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TB2"><static>true</static><pose>-20 0 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TB3"><static>true</static><pose>-10 0 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TB4"><static>true</static><pose>0 0 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TB5"><static>true</static><pose>10 0 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TC1"><static>true</static><pose>-30 -15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TC2"><static>true</static><pose>-20 -15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TC3"><static>true</static><pose>-10 -15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TC4"><static>true</static><pose>0 -15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TC5"><static>true</static><pose>10 -15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.47 0.51 0.63 1</ambient><diffuse>0.47 0.51 0.63 1</diffuse></material></visual>
    </link></model>
    <model name="TC6_fire"><static>true</static><pose>30 -15 1.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 3 3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 3 3</size></box></geometry>
        <material><ambient>0.78 0.35 0.27 1</ambient><diffuse>0.78 0.35 0.27 1</diffuse></material></visual>
    </link></model>
    <model name="pipe_rack"><static>true</static><pose>0 -28 2.0 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>60 1.5 4.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>60 1.5 4.0</size></box></geometry>
        <material><ambient>0.59 0.59 0.59 1</ambient><diffuse>0.59 0.59 0.59 1</diffuse></material></visual>
    </link></model>
    <model name="shed_col_1"><static>true</static><pose>15 7 2.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>0.4 0.4 5.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.4 0.4 5.0</size></box></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material></visual>
    </link></model>
    <model name="shed_col_2"><static>true</static><pose>25 7 2.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>0.4 0.4 5.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.4 0.4 5.0</size></box></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material></visual>
    </link></model>
    <model name="shed_col_3"><static>true</static><pose>35 7 2.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>0.4 0.4 5.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.4 0.4 5.0</size></box></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material></visual>
    </link></model>
    <model name="shed_col_4"><static>true</static><pose>15 17 2.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>0.4 0.4 5.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.4 0.4 5.0</size></box></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material></visual>
    </link></model>
    <model name="shed_col_5"><static>true</static><pose>25 17 2.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>0.4 0.4 5.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.4 0.4 5.0</size></box></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material></visual>
    </link></model>
    <model name="shed_col_6"><static>true</static><pose>35 17 2.5 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>0.4 0.4 5.0</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.4 0.4 5.0</size></box></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material></visual>
    </link></model>
    <model name="shed_roof"><static>true</static><pose>25 12 5.15 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>20.8 10.8 0.3</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>20.8 10.8 0.3</size></box></geometry>
        <material><ambient>0.55 0.55 0.6 1</ambient><diffuse>0.55 0.55 0.6 1</diffuse></material></visual>
    </link></model>
    <model name="valve_skid"><static>true</static><pose>25 10 0.6 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>2 2 1.2</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>2 2 1.2</size></box></geometry>
        <material><ambient>0.5 0.7 0.5 1</ambient><diffuse>0.5 0.7 0.5 1</diffuse></material></visual>
    </link></model>
    <model name="uav_pad"><static>true</static><pose>-40 -30 0.01 0 0 0.0</pose><link name="link">
      <collision name="c"><geometry><box><size>4 4 0.02</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 4 0.02</size></box></geometry>
        <material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material></visual>
    </link></model>
    <model name="tank_1"><static>true</static><pose>40 25 3.0 0 0 0</pose><link name="link">
      <collision name="c"><geometry><cylinder><radius>4.0</radius><length>6.0</length></cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder><radius>4.0</radius><length>6.0</length></cylinder></geometry>
        <material><ambient>0.67 0.75 0.82 1</ambient><diffuse>0.67 0.75 0.82 1</diffuse></material></visual>
    </link></model>
    <model name="tank_2"><static>true</static><pose>40 33 3.0 0 0 0</pose><link name="link">
      <collision name="c"><geometry><cylinder><radius>4.0</radius><length>6.0</length></cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder><radius>4.0</radius><length>6.0</length></cylinder></geometry>
        <material><ambient>0.67 0.75 0.82 1</ambient><diffuse>0.67 0.75 0.82 1</diffuse></material></visual>
    </link></model>

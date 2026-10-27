# swarm_zenoh：GCS ↔ 載具的白名單話題（Zenoh）

2026-10-21。實驗規劃（`docs/reports/…_7.實驗規劃.pdf`）的「ROS 2 中間件避開 multicast」：GCS（地面站、平板）要看載具的狀態、
下急停和人工指派，但 ROS 話題不能整個上 mesh（DDS 的探索、PX4 的話題會吃掉頻寬，10/12 量到每台 69 kB/s）。
這個套件用 Eclipse 的 `zenoh-bridge-ros2dds`，**只讓白名單上的話題、service 經過 mesh**，其他照樣只在機內。

| 通訊 | 走什麼 | 在哪裡 |
|---|---|---|
| CBBA（載具 ↔ 載具） | 自訂 UDP 14600 | `cbba_core`、`swarm_cbba`（**不經過這裡**） |
| 位置廣播（UAV ↔ UAV，CBF） | 自訂 UDP 14601 | `uav_px4_bt` 的 `flight_beacon`（**不經過這裡**） |
| GCS ↔ 載具 | Zenoh（這個套件） | 白名單 `config/whitelist.yaml` |
| 機內（PX4、BT、cbba_node） | DDS | 不出機器 |

**獨立的套件**：不依賴 cbba_core、swarm_cbba、BT，只要 `python3-yaml`、`rclpy`、`std_msgs` 和 bridge 執行檔。
無人機、狗（伯宇的機器）、GCS 都可以單獨裝、單獨測。

## 安裝

```bash
# bridge 執行檔（docker 的 robot.Dockerfile 已經裝好；實機自己裝一次）
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update && sudo apt install zenoh-bridge-ros2dds=1.10.1
# 這個套件
colcon build --packages-select swarm_zenoh && source install/setup.bash
```
改了 `scripts/` 要**重新編譯**：安裝時改了檔名（`zenoh_bridge.py` → `zenoh_bridge`），`--symlink-install` 也是複製、不是連結。

## 單機自我測試（不用 GCS、不用別台）

```bash
ros2 run swarm_zenoh selftest
```
同一台開兩個 bridge，用 ROS domain 90、91 假裝兩台機器（domain 之間 DDS 不通，只能經過 Zenoh），檢查：
GCS → 載具的白名單話題送得到（量延遲）、載具 → GCS 送得到、白名單外的話題**送不到**、別的 namespace **送不到**。
全部符合印 `PASS`。bridge 的 log 在 `/tmp/swarm_zenoh_selftest/`。

## 正式使用

```bash
# GCS（router，聽 mesh 的網卡）
ros2 run swarm_zenoh zenoh_bridge --role gcs --listen-ip <GCS 的 mesh IP>
# 每台載具（client，連 GCS）
ros2 run swarm_zenoh zenoh_bridge --role vehicle --ns uav1 --gcs-ip <GCS 的 mesh IP> --dds-ip <機內的網卡 IP>
# 只看產生的設定：加 --dry-run
```

| 參數 | 說明 | 預設 |
|---|---|---|
| `--role` | `vehicle` 或 `gcs` | 必填 |
| `--ns` | 載具要轉的 namespace，可以多個（模擬時 uav1 容器也替假狗 `v60` 轉） | 載具必填 |
| `--gcs-ip`、`--listen-ip`、`--port` | Zenoh 的連線 | 環境變數 `GCS_IP`／`MESH_IP`、7447 |
| `--transport` | `tcp`（可靠）、`udp`（不保證送達）、`quic` | tcp |
| `--dds-ip` | bridge 的 DDS 用哪張網卡：**機內那張，不能是 mesh**，否則 DDS 又漏到 mesh | `WIRE_IP`，沒有就 `LOCAL_IPS` 第一個，再沒有就 127.0.0.1 |
| `--dds-peers` | DDS 用 unicast 探索的對象 | `LOCAL_IPS`＋`PEER_IPS`（和 `docker/fastdds` 的設定一樣） |
| `--domain` | ROS domain | `ROS_DOMAIN_ID` |
| `--whitelist` | 白名單 | `share/swarm_zenoh/config/whitelist.yaml` |

連法：GCS 是 Zenoh router，每台載具是 client，只連 GCS（載具之間的資料走 CBBA 的 UDP，不用 Zenoh 互連）；
關掉 Zenoh 的 multicast scouting。bridge 內部是 CycloneDDS，和機內的 Fast DDS 用標準 RTPS 互通。

## 模擬（docker）

主機上一次開／關全部的 bridge（GCS ＋ uav1～3 ＋ 3 隻假狗）：
```bash
python3 docker/itest/itest.py demo --interactive --no-patrol   # 先讓整套跑著、待命
docker/scripts/zenoh_sim.sh start      # 開
docker/scripts/zenoh_sim.sh status     # 各容器幾個 bridge
docker/scripts/zenoh_sim.sh logs       # 各 bridge 的 log 最後幾行
docker/scripts/zenoh_sim.sh stop       # 關（itest 結束時不會幫你關）
```
假狗的 bridge 放在 uav1 容器（sim 不在 mesh 上；uav1 的 DDS 看得到 sim 裡的 `fake_dog.py`），每隻一個 bridge，共用 uav1 的 mesh 連線。

## 白名單（`config/whitelist.yaml`）

| 方向 | 名稱 | 用途 |
|---|---|---|
| 載具 → GCS | `robot_state`（限 2 Hz）、`current_task`、`task_closed`、`task_orphaned` | 監看 |
| GCS → 載具 | `emergency_stop`、`manual_override`（service） | 急停、人工指派 |

改白名單只要改這份 yaml。不要把 CBBA、位置廣播、PX4（`fmu/*`）、影像加進來。

## 中繼（選用，2026-10-25；目前不用）

2026-10-26 使用者：**每台 agent 都直接連 GCS**（星狀），不會經過 A 轉到 B，所以正式使用不需要中繼。
如果之後有載具連不到 GCS、要經過某一台轉，那一台用 `--relay`（Zenoh router：往上連 GCS、自己也聽）。**client 不替別人轉送**，
那一台是 client 時，連到它的載具會單向或不通。
```bash
ros2 run swarm_zenoh zenoh_bridge --role vehicle --ns uav1 --relay --gcs-ip <GCS> --listen-ip <uav1 的 mesh IP>
ros2 run swarm_zenoh zenoh_bridge --role vehicle --ns uav2 --gcs-ip <uav1 的 mesh IP>     # uav2 連 uav1
```
`ros2 run swarm_zenoh selftest` 預設只測直接連 GCS；`--only relay` 測經過中繼、`--all` 兩種都測。

## 還沒做

- 全體急停（現在是每台 `/<ns>/emergency_stop`）、人工指派延遲的通知話題（現在只有 BT 的 log）。
- 掉包下的延遲、送達率（之後併進 itest 的掉包測試）；實機要量 DTC 上 tcp／udp 的差別。
- 伯宇的狗：`emergency_stop` 是全域的 `/emergency_stop`，不在 namespace 底下，要和他確認名稱。

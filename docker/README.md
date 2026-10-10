# CBBA_BT 分散式模擬環境

```
sim ─┬─ sim_uav1_net（線）─ uav1 ─┐
     ├─ sim_uav2_net（線）─ uav2 ─┼─ mesh_net（無線電，加 tc）─ gcs
     └─ ...                       ┘
```

| 容器 | 代表 | 主要內容 |
|---|---|---|
| sim | 真實世界＋所有飛控 | Gazebo、每台無人機一個 PX4 SITL 實例 |
| uav1、uav2… | 每台無人機的樹莓派 | XRCE Agent、BT、CBBA |
| gcs | 地面站 | 任務注入、監控、RViz |

每台無人機有**自己的線材網路**，無人機之間沒有線，只能走 mesh，所以 tc 會作用在無人機之間的所有流量上。

## 設定：只改 `.env`

| 變數 | 說明 |
|---|---|
| `UAV_COUNT` | 無人機數量（1~9）。改完執行 `./start.sh` 就會生效 |
| `NET_BASE` | IP 前綴，預設 `172.30`，其他 IP 依規則自動算出 |
| `PX4_SIM_MODEL` | 機型，例如 `gz_x500_mono_cam_down`、`gz_x500` |

IP 規則（`NET_BASE=172.30`）：

| | 線材 | mesh |
|---|---|---|
| uavN | 172.30.**N**.11 | 172.30.0.**(10+N)** |
| sim | 172.30.**N**.2（每台各一個） | — |
| gcs | — | 172.30.0.100 |

`docker-compose.yaml` 是 `gen_compose.py` 依 `.env` **自動產生**的，不要手動改。

## 檔案說明

| 檔案 | 用途 | 修改後需要 |
|---|---|---|
| `.env` | 唯一的設定來源 | `./start.sh` |
| `gen_compose.py` | 依 `.env` 產生 `docker-compose.yaml` | `./start.sh` |
| `entrypoint.sh` | UID 對應、產生 Fast DDS XML | 重啟容器 |
| `bashrc_cbba.sh` | ROS 環境、PATH | 重開 terminal |
| `fastdds/profile.xml.template` | 所有容器共用的 Fast DDS 設定 | 重啟容器 |
| `scripts/` | 容器內的工具（已在 PATH） | 立即生效 |
| `*.Dockerfile`、`entrypoint_stub.sh`、`bashrc_loader.sh` | 映像內容 | `./build.sh` |

`entrypoint.sh` 和 `bashrc_cbba.sh` 是從掛載的專案目錄讀取的，修改它們**不需要重新 build**。

## 從舊版（單一 uav 容器）升級

舊版的 `cbba_uav` 容器和 `sim_uav_net` 網路會和新版衝突，先清掉再 build：

```bash
cd ~/CBBA_BT/docker
docker rm -f cbba_sim cbba_uav cbba_gcs 2>/dev/null
docker network rm cbba_bt_sim_uav_net cbba_bt_mesh_net 2>/dev/null
./build.sh        # base 有改，PX4 會重新編譯一次
./start.sh
```

## 使用流程（以 2 台為例，`.env` 設 `UAV_COUNT=2`）

**每台無人機**：Agent 要先開，再開對應的 PX4。

| Terminal | 容器 | 指令 |
|---|---|---|
| 1 | `./enter.sh uav1` | `xrce_agent.sh` |
| 2 | `./enter.sh uav2` | `xrce_agent.sh` |
| 3 | `./enter.sh sim` | `px4_sitl.sh 1`（同時開啟 Gazebo） |
| 4 | `./enter.sh sim` | `px4_sitl.sh 2`（加入同一個 Gazebo，出生點錯開） |
| 5 | `./enter.sh uav1` | 下指令、跑你的節點 |
| 6 | `./enter.sh gcs` | 監控 |

terminal 太多可以用 tmux：`Ctrl+O` 再按 `h`（左右分割）/ `v`（上下分割）。

**topic 都有命名空間**：`/uav1/fmu/...`、`/uav2/fmu/...`

## 驗收步驟

**① px4_msgs（任一台 uav，只做一次，所有容器共用）**
```bash
setup_px4_msgs.sh
cd ~/CBBA_BT/ros2_ws
colcon build --symlink-install --packages-select px4_msgs
source install/setup.bash
```
⚠ 所有容器共用同一份編譯結果（ros2_ws/build、install），**不要在兩個容器同時編譯**。

**② 收到飛控資料（uav1）**
```bash
ros2 topic echo /uav1/fmu/out/vehicle_status_v1 --once
```

**③ mesh 看得到所有無人機（gcs）**
```bash
ros2 topic list | grep fmu/out/vehicle_status
# 應該看到 /uav1/... 和 /uav2/...
```

**④ 起飛（sim 裡對應那台的 pxh>）**
```
commander takeoff
```

**⑤ tc 只影響 mesh（每台 uav 和 gcs 都設）**
```bash
mesh_tc.sh set 30
```
gcs 執行 `ros2 topic hz /uav1/fmu/out/vehicle_odometry` 頻率下降，但 Gazebo 裡的無人機仍穩定懸停。
結束後每個容器執行 `mesh_tc.sh clear`。

## 常用指令

| 指令 | 在哪 | 說明 |
|---|---|---|
| `px4_sitl.sh [N] [機型]` | sim | 啟動第 N 台 PX4 |
| `xrce_agent.sh` | uavN | 啟動 Agent |
| `colcon build --symlink-install` | uav1（在 ros2_ws 裡） | 編譯 ros2_ws，所有容器共用 |
| `mesh_tc.sh set/show/clear` | uavN / gcs | 控制 mesh 丟包 |
| `cbba_uav.sh` | uavN | 一次啟動 XRCE Agent、task_executor、px4_state_bridge、cbba_node（步驟見 `cbba_core/doc/sitl_test.md`） |

每台機號只能跑一個 cbba_node（程序鎖 `agentN.lock`）：所有容器把主機的 `/tmp/cbba_locks` 掛在 `/run/cbba`（`CBBA_LOCK_DIR`），
entrypoint 會把它改成使用者可寫。改了 `gen_compose.py` 之後要重新 `./start.sh` 才生效。
| `cbba_task.sh new/done/fail/watch` | uavN | 發任務、手動回報結果、看目前指派 |
| `gz_bridge.sh` | sim | 把 /clock 送進 ROS 2 |
| `./enter.sh <容器>` | host | 進入容器（`uav` = `uav1`） |
| `./stop.sh` | host | 刪除所有容器 |
| `docker compose -f docker-compose.yaml stop` | host | 只暫停，保留容器 |

## 模擬時的 PX4 參數

沒有接 QGC 和遙控器，offboard 控制時可能觸發 failsafe，在對應的 `pxh>` 設定：
```
param set NAV_DLL_ACT 0
param set COM_RC_IN_MODE 4
param set COM_RCL_EXCEPT 4
```
**只適用於模擬**，實機要保留這些安全檢查。

## 換成實機

每台機上設定兩個環境變數，`fastdds/profile.xml.template` 不用改：
- `LOCAL_IPS`：自己的網卡 IP（例如無線網卡）
- `PEER_IPS`：其他機器的 IP

Agent 改用序列埠：`MicroXRCEAgent serial --dev /dev/ttyXXX -b 921600`

## 之後加入機器狗

在 `gen_compose.py` 加一個 `dog` 服務（自己的 `sim_dog_net`，mesh IP 用 `172.30.0.50`），並把它的 mesh IP 加進各容器的 `PEER_IPS`。

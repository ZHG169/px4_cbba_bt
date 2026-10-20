# Gazebo 視角調整（2026-10-19）

模擬裡有**兩個鏡頭**，設定在不同地方，改的時候不要搞混：

| 鏡頭 | 用在哪 | 設定在哪 | 改完怎麼生效 |
|---|---|---|---|
| **錄影鏡頭**（場景裡的 `overview_camera`） | `itest.py demo` 的 `gazebo.mp4`、手動錄影 | `docker/gz/gen_patrol_site.py` 的 `OVERVIEW_POSE`、`OVERVIEW_FOV` → 產生 `docker/gz/worlds/<場景>.sdf` | 重新產生 sdf，**關掉 Gazebo 再開** |
| **視窗鏡頭**（Gazebo GUI 打開時的初始視角） | 只影響螢幕畫面，不影響錄影 | `docker/scripts/px4_sitl.sh` 的 `CAMERA_POSE`（依場景） | 重開 Gazebo 視窗（`px4_sitl.sh 1`） |

建議流程：**先在視窗裡用滑鼠找到喜歡的角度 → 讀出鏡頭位置 → 填進兩個地方 → 重開確認**。

---

## 一、pose 的六個數字

兩個地方的格式都是 `x y z roll pitch yaw`：

| 數字 | 意思 | 單位 |
|---|---|---|
| `x y z` | 鏡頭的位置，map ENU（x 往東、y 往北、z 往上） | 公尺 |
| `roll` | 鏡頭左右傾斜，一般填 0 | 弧度 |
| `pitch` | 往下看多少：0 = 水平，1.5708（π/2）= 正下方 | 弧度 |
| `yaw` | 朝哪個方向：0 = 朝東，0.785 = 朝東北，1.5708 = 朝北，3.1416 = 朝西 | 弧度 |

錄影鏡頭另外有 **`OVERVIEW_FOV`**：水平視角（弧度）。越大看得越廣、東西越小；畫面固定 1280×720（16:9）。

### 現在的值

| 場景 | 錄影鏡頭（`OVERVIEW_POSE`、`OVERVIEW_FOV`） | 視窗鏡頭（`CAMERA_POSE`） |
|---|---|---|
| `demo_site`（展示用，±22 m） | `1 -1.5 150 0 1.5708 1.5708`：正上方 150 m 往下看，影像上方朝北；視角 0.389 rad（約 22°），約拍到 59 m × 33 m（x −28.5～30.5、y −18～15）。2026-10-19 從 30 → 35 → 150 m（見「視差」） | `-22 -24 20 0 0.52 0.78`：西南方斜看 |
| `patrol_site`（±40 m） | `-60 -55 55 0 0.57 0.76`；視角 1.05 rad | `-60 -55 55 0 0.57 0.76` |
| `fire_site` | 沒有錄影鏡頭（不是 `gen_patrol_site.py` 產生的） | `-28 -30 28 0 0.61 0.77` |
| 其他 | `-30 -30 30 0 0.6 0.785`（`gen_patrol_site.py` 的預設） | `-30 -30 30 0 0.6 0.785` |

### 視差：無人機看起來沒停在點上（2026-10-19）

錄影鏡頭是透視鏡頭。無人機飛在 5 m 高，比地面的方塊更靠近鏡頭，畫面上會被往外推（遠離畫面中心）：

```
畫面上的偏移 ≈ 離畫面中心（鏡頭正下方）的水平距離 × 無人機高度 ／（鏡頭高度 − 無人機高度）
```

35 m 高、視角 1.4 rad 時，R4（離中心 6.5 m）上空懸停的無人機看起來偏南 1.1 m，場地邊緣（20 m）偏 3.3 m。
**鏡頭拉高、視角縮小**（拍到的範圍不變）視差就變小：

| 鏡頭高度 | 視角（拍 59 m 寬） | 離中心 6.5 m | 離中心 20 m |
|---|---|---|---|
| 35 m | 1.4 rad | 1.1 m | 3.3 m |
| 100 m | 0.574 rad | 0.34 m | 1.1 m |
| 150 m（現在） | 0.389 rad | 0.22 m | 0.7 m |

視角 = 2 × atan(拍到的寬度的一半 ／ 鏡頭高度)。畫面範圍、無人機大小、網格線在畫面上的粗細都不變。
Gazebo 視窗可以切成正交投影（沒有視差）看；伺服器的錄影鏡頭能不能用正交投影沒有確認過，所以用拉高的方式。

### 自己算：從鏡頭位置對準場地中心

鏡頭放在 (x, y, z)，要對準地面的 (cx, cy)：

```
yaw   = atan2(cy − y, cx − x)
pitch = atan(z ／ 水平距離)，水平距離 = √((cx − x)² + (cy − y)²)
```

例：鏡頭在 (−25, −30, 25)、對準 (0, −1.5)：水平距離 ≈ 38 m，yaw ≈ atan2(28.5, 25) ≈ 0.85，pitch ≈ atan(25 / 38) ≈ 0.58
→ `-25 -30 25 0 0.58 0.85`。

正上方往下看（俯視）：pitch = 1.5708；yaw = 1.5708 時影像上方朝北、右邊朝東。

---

## 二、最省事：在視窗裡調好，再讀出數字

### 1. 開 Gazebo 視窗

照 `docs/sim_launch_guide.md` 開模擬，或只開場景（sim 容器）：

```bash
CBBA_WORLD=demo_site px4_sitl.sh 1
```

滑鼠操作：左鍵拖曳平移、右鍵拖曳（或滾輪按住）旋轉、滾輪縮放。

### 2. 讀出目前視窗鏡頭的位置（sim 容器，另開一個 terminal）

視窗的 `CameraTracking` 外掛會把鏡頭位置發在 `/gui/camera/pose`：

```bash
gz topic -e -t /gui/camera/pose -n 1
```

印出 `position { x y z }` 和 `orientation { x y z w }`（四元數；值是 0 的欄位不會印出來）。
直接換成 `x y z roll pitch yaw`：

```bash
gz topic -e -t /gui/camera/pose -n 1 | python3 -c '
import math, re, sys
t = sys.stdin.read()
def get(block, k):
    m = re.search(block + r"\s*\{([^}]*)\}", t)
    v = re.search(r"\b" + k + r":\s*([-0-9.eE+]+)", m.group(1)) if m else None
    return float(v.group(1)) if v else 0.0
x, y, z = (get("position", k) for k in "xyz")
qx, qy, qz, qw = (get("orientation", k) for k in "xyzw")
roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
print(f"{x:.1f} {y:.1f} {z:.1f} {roll:.2f} {pitch:.2f} {yaw:.2f}")'
```

輸出例：`-22.0 -24.0 20.0 0.00 0.52 0.78`。roll 不是 0 的話改成 0（錄影畫面才不會歪）。

- 什麼都沒印出來：用 `gz topic -l | grep camera` 看話題名稱；還是沒有的話，先載入 PX4 的 gz 環境
  （`px4_sitl.sh` 用的 `gz_env.sh`，在 PX4 的 rootfs 裡）再試。
- 視窗鏡頭的視角和錄影鏡頭的 `OVERVIEW_FOV` 不一定一樣，同一個 pose 拍到的範圍可能略有不同，第四節錄一小段確認。

---

## 三、填進設定

### 錄影鏡頭（`docker/gz/gen_patrol_site.py`）

```python
OVERVIEW_POSE = {"demo_site": "1 -1.5 150 0 1.5708 1.5708", "patrol_site": "-60 -55 55 0 0.57 0.76"}
OVERVIEW_FOV = {"demo_site": 0.389}
```

改對應場景的值，然後**重新產生場景**（主機，`~/CBBA_BT`）：

```bash
python3 docker/gz/gen_patrol_site.py demo_site      # → docker/gz/worlds/demo_site.sdf
python3 docker/gz/gen_patrol_site.py                # patrol_site
```

- **不要直接改 `.sdf`**：下次執行 `gen_patrol_site.py` 會被蓋掉。
- 改完要**關掉 Gazebo 再開**，sdf 只在 Gazebo 啟動時讀一次。殘留的 Gazebo 伺服器可能不理 SIGTERM，
  sim 容器裡用 `pkill -9 -f "gz sim"`（不要 `pkill -f "sleep infinity"`，會把容器停掉）。

### 視窗鏡頭（`docker/scripts/px4_sitl.sh`）

```bash
case "${CBBA_WORLD}" in
    patrol_site) CAMERA_POSE="-60 -55 55 0 0.57 0.76" ;;
    demo_site)   CAMERA_POSE="-22 -24 20 0 0.52 0.78" ;;
    fire_site)   CAMERA_POSE="-28 -30 28 0 0.61 0.77" ;;
    *)           CAMERA_POSE="-30 -30 30 0 0.6 0.785" ;;
esac
```

改對應場景那一行，重開 `px4_sitl.sh 1` 的視窗就生效（`px4_sitl.sh` 每次開視窗時把它填進 `docker/gz/gui/cbba_gui.config` 的 `@CAMERA_POSE@`）。
想讓視窗打開時就和錄影同一個角度，兩個地方填一樣的值。

### 錄影裡的網格（`GRID`，2026-10-19）

視窗的網格是 GUI 外掛（`docker/gz/gui/cbba_gui.config` 的 GridConfig），**只畫在視窗裡，錄影鏡頭看不到**。
要錄到網格，`gen_patrol_site.py` 在場景裡加了貼地的細線模型 `ground_grid`（只有外觀、沒有碰撞，不影響物理）：

```python
GRID = {"demo_site": (2.0, 30, 0.06)}   # 場景 → (格子邊長 m, 每個方向幾格, 線寬 m)
GRID_COLOR = "0.35 0.35 0.35 1"
```

- 間距、範圍和視窗的網格一樣（2 m 一格、往四個方向各 30 格），所以視窗裡兩組網格會重疊在一起。
- 線寬 6 cm：35 m 高俯視時約 1 個像素。太細的話會時有時無，錄影高度改高時要跟著加粗。
- 只有 `demo_site` 有；其他場景要加，在 `GRID` 加一行。改完一樣要重新產生 sdf、重開 Gazebo。
- 無人機的下視相機也看得到這些線（細灰線，不是 AprilTag 的黑白方格），應該不影響火點判定；如果 itest 的火警判定變差，先把 `GRID` 拿掉比對。

---

## 四、確認錄影的畫面

不用跑整個 itest demo，Gazebo 開著時直接錄一小段（sim 容器）：

```bash
# 開始
gz service -s /demo_site/overview/record_video --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'start: true, format: "mp4", save_filename: "/home/ncrl/CBBA_BT/view_test.mp4"'
# 幾秒後停止
gz service -s /demo_site/overview/record_video --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'stop: true'
```

檔案在主機的 `~/CBBA_BT/view_test.mp4`（看完刪掉，不要 commit）。

也可以在視窗加 **Image Display** 外掛（右上角選單 → 搜尋 Image Display），話題選 `/demo_site/overview/image`，即時看錄影鏡頭的畫面。

### 錄影範圍的參考

點都要在畫面內才拍得到；`demo_site` 的點在 x −20～2.5、y −14～12（`docker/gz/worlds/demo_site_points.csv`），
itest 手動插入的範圍預設 x −22～24、y −14～11（`docker/itest/config.json` 的 `demo.area`，是照目前的錄影範圍設的）。
錄影範圍改小的話，`demo.area` 也要跟著縮小，否則插入的點會拍不到。

---

## 五、常見問題

| 狀況 | 原因 | 處理 |
|---|---|---|
| 改了 `OVERVIEW_POSE`，錄影沒變 | 沒重新產生 sdf，或 Gazebo 沒重開 | `gen_patrol_site.py <場景>`，`pkill -9 -f "gz sim"` 後重開 |
| 改了 `CAMERA_POSE`，視窗沒變 | 視窗沒重開，或場景名稱對不到 `case` | 確認 `CBBA_WORLD`，重開 `px4_sitl.sh 1` |
| 畫面是歪的 | roll 不是 0 | roll 填 0 |
| 畫面上下顛倒或方向不對 | 正上方往下看時 yaw 決定影像的「上方」 | yaw = 1.5708：上方朝北；0：上方朝東 |
| 什麼都看不到（全黑或只有天空） | pitch 太小（往上看）或位置在地面下 | pitch 取 0.4～1.57、z 要大於 0 |
| 無人機太小 | 太高或視角太大 | 降低 z，或減小 `OVERVIEW_FOV` |

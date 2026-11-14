# 註冊表（2-1b REG-1，schema 1.0，2026-11-14）

2-1b 的原則：**一份模組註冊表是唯一來源**，規劃器、檢查器、VLM 詞彙、CBBA 的可行性、BT 分派都讀它；
新增動作或機器人只改註冊表與 adapter。這裡先放 CBBA 要的三份；`actions.yaml`、`objects.yaml`、`event_types.yaml`
由規劃器（PLN-1、PLN-2）、事件偵測（EVT-1）一起定，CBBA 不讀。

| 檔案 | 內容 | 誰讀 |
|---|---|---|
| `tools.yaml` | 能力標籤：name、**bit**（CBBA 封包裡能力遮罩的位元位置）、description、deprecated | cbba_node（檢查和程式一致）、規劃器、檢查器 |
| `platforms.yaml` | 平台：name、**code**（封包裡的平台代碼）、成本模型（速度、耗電、存量、續航、max_bundle、must_return） | cbba_node（M0 只檢查；M1 拿來出價） |
| `fleet.yaml` | 機隊：每台的 id（機號）、name、platform、namespace、capabilities；gateways（閘道的機號） | cbba_node（自己的平台與能力、閘道的機號） |

## 規則

- 每份檔案開頭 `schema_version: "1.0"`；版本不對、缺欄位、型別不對時 cbba_node 拒絕啟動。
- **bit、code 只增不改**：已經用過的不改、不重用；不要的標籤標 `deprecated: true` 留著。bit 0～63、code 1～255（0 保留給
  「任務不限平台」和「閘道」）。
- `tools.yaml` 要包含程式（`ros2_ws/src/cbba_core/include/cbba_core/capabilities.hpp`）認得的全部標籤、bit 一樣；
  註冊表可以比程式新（多出程式不認得的標籤，cbba_node 印警告、宣告了也用不到）。新增標籤時兩邊一起加。
- 機號 1～254，載具和閘道之間也不能重複。能力要在 `tools.yaml`、不能是 deprecated；平台要在 `platforms.yaml`。
- 能力故障（E_HW）在執行期由 cbba_node 扣掉，不改這份。

## cbba_node 怎麼用

```bash
ros2 run swarm_cbba cbba_node --ros-args ... -p registry_dir:=$HOME/CBBA_BT/registry
# 或環境變數 CBBA_REGISTRY_DIR=$HOME/CBBA_BT/registry
```

- 載具：從 `fleet.yaml` 找自己的機號，平台要和 `agent_type` 一致（uav = UAV、quadruped = UGV），能力取代 `capabilities` 參數；
  不在清單裡的機號照參數（印警告）。
- 閘道（`participate: never`）：自己的機號應該在 `gateways`；全隊的 `gateways` 參數（判斷閘道在不在線）改用 `fleet.yaml` 的。
- 沒給 `registry_dir`：照舊用參數（`capabilities`、`gateways`），行為和之前一樣。

檢查：`swarm_cbba` 的單元測試 `test_registry` 讀這個目錄，和 `capabilities.hpp` 比對。

## 目前的內容

模擬的機隊（docker：uav1～3、dog1～3、gcs）。能力照節點原本的預設（UAV：mobility_air、rgb_camera；狗：mobility_ground、
rgb_camera），所以給不給 `registry_dir` 出價都一樣；實機（V60 的雲台變焦、熱像等）的能力待和伯宇確認。
`platforms.yaml` 的成本參數和現在 `swarm_cbba/config/cbba_uav.yaml`、`cbba_v60.yaml`（及程式預設）一樣，M1 改成從這裡讀之後，
那兩份的對應參數就拿掉。

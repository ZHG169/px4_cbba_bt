# docs：跨套件的文件

給團隊看的文件：怎麼開模擬、目前進度、測試情境、對外的通知。各套件的技術細節在套件自己的 `doc/`
（CBBA 的協定、參數、狗端的測試在 `ros2_ws/src/cbba_core/doc/`，目錄見那裡的 `README.md`）。

| 文件 | 給誰 | 內容 | 狀態 |
|---|---|---|---|
| [`sim_launch_guide.md`](sim_launch_guide.md) | 自己、隊友 | **模擬測試的開啟方式**：基本分配、火情偵測、巡檢＋隨機插入、弱網、離線模擬、自動整合測試 | 有效 |
| [`progress.md`](progress.md) | 自己、隊友、老師 | **目前進度**：週進度對照、最近的測試結果、缺失清單、已定案還沒實作的 | 持續更新 |
| [`cbba_comm_status_20261012.md`](cbba_comm_status_20261012.md) | 簡報、隊友 | **CBBA 機間通訊進度**：架構、協定版本 2、已完成功能、測試結果、量測數據、介面對齊、修掉的問題、還沒做的 | 2026-10-12 快照 |
| [`sim_scenario_plan.md`](sim_scenario_plan.md) | 自己 | 巡檢＋隨機插入、電池權重測試：當初的分析、使用者的決定、實作與第一次結果 | 已實作 |
| [`protocol_v2_notice.md`](protocol_v2_notice.md) | 狗端（伯宇）、規格作者 | 機間通訊協定版本 2 的變更通知（不相容於規格書） | 有效，還沒送給規格作者 |
| [`nav2_integration_plan.md`](nav2_integration_plan.md) | 自己、狗端（伯宇） | 模擬裡導入 Nav2：伯宇的 task_manager＋Nav2 取代假狗（dog 容器、Gazebo V60、座標、假雲台、itest） | 草案，還沒實作 |
| [`task_insertion_guide.md`](task_insertion_guide.md) | 自己 | **插入任務點的使用方法**：itest demo 的 `--interactive`（terminal 手動、隨機插入）、`--scenario`（情境檔）、手動 SITL 的腳本 | 2026-10-14 |
| [`reports/`](reports/) | 對外 | 某一天的進度快照（PDF），不再修改 | 快照 |
| [`archive/`](archive/) | 查歷史 | 已經做完或被取代的規劃 | 封存 |

封存的文件：

| 文件 | 為什麼封存 |
|---|---|
| `archive/generic_cbba_node_plan.md` | 通用 cbba_node 的規劃，2026-10-09 已實作 |
| `archive/week4_6_status.md` | 2026-10-10 的第 4～6 週差異，被 `progress.md` 取代 |
| `archive/cpf_20261014/` | 無人機避碰 CPF（人工位勢場）的原始碼、測試與當時的 uav_bt.yaml；2026-10-14 換成 CBF |

其他入口：專案總覽 `../README.md`、模擬環境 `../docker/README.md`、自動整合測試 `../docker/itest/README.md`。

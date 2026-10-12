# 整合測試（itest）

只看**話題和網路**，不看任何節點的 log，所以換成別人寫的 CBBA Node／BT 也能用：
介面（話題名稱、訊息型別、欄位、QoS、啟動指令）都寫在 `config.json`，對方的介面不同時只改這份。

```
主機：itest.py ──docker exec──► 各容器啟動／關掉節點、PX4、tc
         │  commands.jsonl ▲ events.jsonl（docker/itest/out/<時間>/，主機和容器都掛載）
         ▼                 │
   sim 容器：ros_bridge.py（經線材網路連得到每一台的 DDS）：收各台的話題、幫主機發話題
```

## 用法

容器要先開好（`docker/start.sh`）、要測的套件要先編譯好，在**主機**執行：

```bash
cd ~/CBBA_BT
python3 docker/itest/itest.py comm     # CBBA 之間的通訊（不開 PX4、BT），約 2 分鐘
python3 docker/itest/itest.py uav      # UAV 整串（PX4 SITL＋Gazebo＋CBBA Node＋BT），約 8～10 分鐘
python3 docker/itest/itest.py comm --only C1,C3     # 只跑某幾項
```

容器裡已經有在跑的節點、PX4、Gazebo 時會停下來，避免打斷你正在做的事；加 `--kill-existing` 先關掉再跑。
結果在 `docker/itest/out/<時間>_<組>/report.md`，各節點的輸出（`node_uavN.log`、`stack_uavN.log`、`px4_N.log`）也在那裡。

## 測試項目

**comm：CBBA 之間的通訊**（每台在自己的容器啟動 CBBA Node，位置由測試發假的 robot_state）

| 項目 | 測什麼 | 通過的條件 |
|---|---|---|
| C0 | 節點上線 | 每台的 assigned 話題都收得到 |
| M | mesh 流量 | 只記錄（待命時每秒多少） |
| C1 | 任務傳到全隊、只有一台得標 | 5 s 內恰好一台拿到，之後 2 s 沒有別台拿過 |
| C2 | 回報完成 | 3 s 內撤下，之後不再派給別台 |
| C3 | 失敗交回 | 5 s 內別台接手 |
| C4 | 停止送 robot_state（等同 PX4 斷線） | 10 s 內別台接手 |
| C5 | 節點整個停止 | 40 s 內別台接手（失聯接手：火警 10 s、巡檢 30 s） |
| C6 | 重開機後重新加入 | 重開的那台拿到身邊的新任務，不重複 |
| C7 | 掉包 30%、延遲 50±30 ms | 10 s 內恰好一台拿到；完成後撤下 |
| C8 | 斷網 | 斷網時只有建立者拿到（證明協調走 mesh）；恢復後不重複 |

**uav：UAV 整串**（`fire_site` 場景）

| 項目 | 測什麼 | 通過的條件 |
|---|---|---|
| U1 | 起飛 | 每台 armed、到巡檢高度（150 s 內） |
| U2 | 單一任務 | 指派 → 飛到 1.5 m 內 → task_result 成功 |
| U3 | 火警 | 到達火警點後出現地面處置任務（new_task），位置誤差 ≤ 1.5 m |
| U4 | 誘餌 | 到達誘餌點後**沒有**地面任務 |
| U5 | 多台分配與避碰 | 3 個任務全部成功、機間最近距離 ≥ 1.5 m |
| U6 | 飛行中 PX4 斷線 | 15 s 內別台接手並完成 |

## 換成別人的 CBBA Node／BT 時

| config.json | 改什麼 |
|---|---|
| `topics` | 話題名稱、訊息型別、QoS（`{ns}` = 命名空間） |
| `fields` | assigned 訊息裡任務編號、指派版本的欄位名稱；沒有版本就把 `version` 設成 `""` |
| `messages` | 測試發的 new_task、task_result、robot_state 的內容（`{task_id}` 等會換成數值） |
| `comm.start`、`comm.stop_pattern` | 在 uav 容器啟動 CBBA Node 的指令、關掉時 `pkill -f` 比對的字串 |
| `uav.start`、`uav.stop_pattern` | 每台的 CBBA Node＋BT＋轉接（要在背景一直跑） |
| `paths.ros_setup` | 容器裡要 source 的環境 |

訊息型別要有 `swarm_interfaces` 的定義才收得到；對方用別的訊息套件時，`ros_setup` 要包含那個套件。

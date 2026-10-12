# cbba_core/doc：CBBA 的技術文件

跨套件的文件（模擬的開啟方式、進度）在 repo 的 `docs/`。

| 文件 | 內容 | 狀態 |
|---|---|---|
| [`protocol.md`](protocol.md) | **機間通訊協定版本 2**：封包總覽、接收端處理、分批、正式指派與鎖定、和規格書不同的地方、流量 | 有效 |
| [`parameters.md`](parameters.md) | **所有參數**：意義、預設值、為什麼是這個值、要和誰一致、已知限制 | 有效 |
| [`sitl_test.md`](sitl_test.md) | PX4 SITL＋BT 的**手動**測試步驟（看 log 除錯用；自動版是 `docker/itest`） | 有效 |
| [`ugv_test.md`](ugv_test.md) | 機器狗端的執行方式、介面、測試資料 T1～T8 | 有效 |
| [`ugv_tuning.md`](ugv_tuning.md) | 機器狗的能量模型怎麼量、怎麼改 | 有效（數值是佔位） |
| [`formula/`](formula/) | 出價公式（電量成本）的設計理由：`formula_design.md`、`.pdf` 和公式圖 | 有效（公式沒變） |
| [`experiments/`](experiments/) | 電池權重的實驗結果：`cbba_parameters.pdf`（`../cbba_parameters.md` 的 PDF）和結果圖 | 2026-10-05 的結果 |
| [`spec/`](spec/) | 外部規格：`機間通訊封包規格_20261006.pdf`（版本 1 照它，版本 2 不相容）、`ugv_interface.png`（機器狗端的介面：RobotState、TaskResult、ManualOverride） | 參考 |
| [`archive/`](archive/) | 已被取代：`cbba_messages.md`／`.pdf`、`task_msg_table.png`（2026-10-05 的 ROS 訊息介面 v1.0：CBBAMessage、Bid、AgentStamp，機間通訊改成協定版本 2 後不再使用） | 封存 |

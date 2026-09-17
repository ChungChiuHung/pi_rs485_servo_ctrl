# servo_comm_shihlin_unified（籌備中）

這個資料夾的目標：把 `servo_comm_shihlin/` 與 `servo_comm_shihlin_50W/`
合併成一份程式碼，透過 JSON 設定檔（`motor_profiles.json`）切換要控制
哪一顆馬達，取代目前「兩份程式碼手動同步」的做法。

完整設計文件：`docs/servo_comm_shihlin_merge_design.md`

## 目前狀態

**通訊底層 + 一個防呆檢查已就緒，主控邏輯（`servo_control.py`）尚未搬入**：

* ✅ 已從 `servo_comm_shihlin_50W/`（決議中的基準版本）複製通訊底層：
  `modbus_ascii_client.py`、`modbus_response.py`、`modbus_command_code.py`、
  `modbus_utils.py`、`servo_control_registers.py`、`serial_port_manager.py`、
  `servo_p_register.py`。
* ✅ `servo_p_register.py` 新增 `PA.ABS`（PA28）暫存器定義。
* ✅ 新增 `absolute_mode_check.py`：讀取 PA28、在 log 中清楚回報絕對模式
  狀態的防呆檢查（讀值 1／0／預期外數值／通訊失敗，四種情況都有對應
  log 與明確的回傳值語意，`None` 代表「不知道」，不會被誤判成安全）。
  搭配 `test_absolute_mode_check.py`（7 個測試，已跑過、全過）。
* ✅ 新增 `check_pa28.py`：**可直接在樹莓派上對真實硬體執行**的獨立
  診斷腳本，純讀取、不會啟動/移動/寫入馬達，跑完會印出 PA28 是否為 1。
* ✅ §2.2 第 1、2、3、5、6 項：已確認做法
* ✅ Encoder overflow：已決定採用方案 B（改讀 `PA32`+`PA33`）
* ⏸ **卡點：需要你在樹莓派上實際執行 `check_pa28.py`，確認 PA28 是否為 1**
  —— 沒有這個確認，PA32/PA33 讀出來的位置資料無效，§2.2 第 4、7 項
  也無法定案。
* ⏸ §2.2 第 4、7 項（角度追蹤演算法、180 度保護）、`servo_control.py`
  主控邏輯本體，待 PA28 確認後才會搬入/實作。

## 如何執行 PA28 檢查

在樹莓派上、驅動器已接妥的狀態下：

```bash
cd servo_comm_shihlin_unified/
python3 check_pa28.py
```

只會讀取，不會對馬達做任何啟動/移動/寫入動作。結果會印在畫面上，
exit code 0 代表確認為絕對模式（PA28=1），1 代表不是，2 代表讀不到。

## 下一步

1. 在樹莓派上執行 `check_pa28.py`，把結果回報回來。
2. 依結果把 §2.2 第 4、7 項定案。
3. 搬入/重寫 `servo_control.py`（含 `motor_profiles.json` 的 config 切換
   邏輯），繼續 TDD Red/Green 階段。
4. `servo_comm_shihlin/`、`servo_comm_shihlin_50W/` 在這份合併版本
   通過真實硬體驗證（兩個 motor profile 都測過）之前，**不會**被刪除
   或標記為棄用。

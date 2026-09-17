# servo_comm_shihlin 與 servo_comm_shihlin_50W 合併設計文件（草案）

狀態：**設計討論中，主控邏輯（`servo_control.py`）尚未動手改**——第 4、7
項（角度追蹤演算法、180 度保護）與 `motor_profiles.json` config 切換邏輯
都還在暫緩／待決策。但通訊底層與一個獨立的 PA28 防呆檢查已經先落地在
`servo_comm_shihlin_unified/`（見 §2.4、§4 第 2、4 項），不是完全零實作。

## 1. 目標
把 `servo_comm_shihlin/` 與 `servo_comm_shihlin_50W/` 合併成一份程式碼，
透過一個 JSON 設定檔切換「要控制哪一顆馬達 / 哪種情境（scenario）」，
而不是像現在這樣維護兩份幾乎相同、只能「手動同步」的資料夾。

這其實就是 `main.py` 當初「暫停中」的計畫（見 CLAUDE.md §5）的一個縮小
版實作——只先處理 Shihlin 系列這兩個變種，還不牽涉 `servo_communication`
（Type 1 品牌）那邊。

## 2. 差異總表

### 2.1 純參數差異（可以直接放進 JSON config，沒有爭議）

| 項目 | servo_comm_shihlin | servo_comm_shihlin_50W | 建議 config key |
|---|---|---|---|
| Baud rate | 115200 | 9600 | `baud_rate` |
| 齒輪比換算（pulse/degree） | 349525.333... | 116508.444445 | ~~`base_pulse_per_degree`~~ → 改存 `gear_ratio`（見下方說明） |
| 原點絕對位置 `abs_home_pos` | 62369153 | 1184347 | `abs_home_pos` |
| P 暫存器 STA/STB/JOG（加減速時間、Jog 速度） | 無 | 有 | ~~`extra_p_registers`~~ → 兩邊都保留（見下方說明） |
| P 暫存器 PE03/PE04（PATH#1 definition/data） | 無 | 有 | ~~`extra_p_registers`~~ → 兩邊都保留（見下方說明） |

> **更新（已確認）：** 兩種馬達用的驅動器是同一款，所以 STA/STB/JOG、
> PE03/PE04 這幾個暫存器不是「50W 專屬」，而是驅動器本身就有的暫存器
> ——兩個 profile 都應該定義它們，不需要用 `extra_p_registers` 這種
> per-profile 開關來切換。原本 `servo_comm_shihlin`（原版）程式碼裡沒有
> 定義，單純是因為當初沒寫到，不代表原版馬達不支援。合併後這幾個暫存器
> 會直接寫進共用的 `servo_p_register.py`，兩個 profile 都能用，不放進
> JSON config。

這幾項單純是「同一份程式碼、代入不同數字」，適合做成
`motor_profiles.json` 裡的欄位，兩顆馬達各一組 profile。

> **`base_pulse_per_degree` 拆解確認：** 對照程式碼裡的註解、
> `docs/SDE_English_manual_UL_v107.pdf`（p.349「22-bit (4,194,304 pulses/rev)
> high resolution encoder」、p.7829「Calculation of electronic gear
> ratio」段落）與 git log，驗證你說的推導方式完全吻合——`base_pulse_per_degree`
> 是「編碼器解析度（encoder pulse/rev）× 減速比（gear ratio）÷ 360」算出來的：
>
> | | 編碼器解析度（manual 確認） | 減速比 | 輸出軸 pulse/rev | ÷360 = base_pulse_per_degree |
> |---|---|---|---|---|
> | 400W（原版） | 4,194,304（22-bit） | **30** | 125,829,120 | 349525.333... |
> | 50W | 4,194,304（22-bit） | **10** | 41,943,040 | 116508.444445 |
>
> （50W 的減速比 10:1 也對得上 git log 裡的 commit `d41dd5f change other
> gear ratio 10:1`，交叉驗證沒問題。）
>
> **修正說明：** 這份文件較早版本曾誤植編碼器解析度為 1,048,576（2^20），
> 雖然當時算出來的減速比（120/40）在數學上也剛好整除、看起來合理，但對照
> manual 明確寫的「22-bit (4,194,304 pulses/rev)」才發現算錯——兩個
> 2 的次方數字都能整除同一組 pulse/rev 總數是巧合，光靠程式碼裡的數字反推
> 無法唯一決定，必須對照原廠規格書才能確認。已更正為 4,194,304 / 30 / 10。
>
> 另外也對照了 `servo_p_register.py` 裡 PA06(CMX)、PA07(CDV) 電子齒輪比
> 暫存器的預設值皆為 1（且程式碼中沒有覆寫它們），確認電子齒輪比維持
> 1:1、沒有額外疊加——上面表格的減速比是純機械減速機的比例，不是電子
> 齒輪比設定值。
>
> 兩邊的編碼器解析度（4,194,304，22-bit）完全相同——這跟你之前確認「驅動器
> 都相同」是一致的，編碼器解析度是驅動器/馬達本體的固定規格，不是每個
> profile 各自不同的數字。真正因馬達/減速機而異的只有**減速比
> （gear_ratio：30 vs 10）**。
>
> 所以 JSON 不應該直接存目前這種手算好、看不出物理意義的浮點數
> （`349525.3333333333`），而應該存：
> * 一個**共用常數**：`encoder_pulses_per_rev: 4194304`（不放進 per-profile，
>   兩個 profile 共用同一個值）
> * 各 profile 各自的 `gear_ratio`（400W = 30，50W = 10）
>
> 程式啟動時用 `base_pulse_per_degree = encoder_pulses_per_rev * gear_ratio / 360`
> 動態算出來，不再把算好的浮點數字寫死在程式碼或 config 裡。這樣以後
> 減速比若有變動（或要支援第三顆馬達），只需要改 `gear_ratio` 這一個
> 有物理意義、可以直接對照驅動器/減速機規格表的數字，不用重新手算浮點數。

### 2.2 邏輯/功能差異（無法只靠參數切換，需要你逐項決定）

以下每一項都需要你確認「要保留哪一種行為」，或是「兩種都要保留、變成
可選功能」。**我不會自己選一邊當標準**——请在每一項後面標記你的決定。

| # | 差異項目 | `servo_comm_shihlin`（原版）行為 | `servo_comm_shihlin_50W` 行為 | 你的決定 |
|---|---|---|---|---|
| 1 | 取消連續讀取的安全機制 | **沒有** `cancel_continuous_reading()` 方法，也沒有對應的 `on_cancel` 事件監聽器 | 有完整的 `cancel_continuous_reading()`：停止讀取執行緒、重算角度、觸發 `on_cancel` 事件 | ✅ **採用 `_50W`** |
| 2 | OSC handler：連續運動控制 | 沒有 `set_countinuous_motion_handler` / `ctrl_continuous_motion_handler` / `cancel_loop_handler` 這三個 handler | 有這三個 handler，對應 TouchDesigner 的連續運動控制 | ✅ **採用 `_50W`** |
| 3 | `enable_speed_ctrl()` 參數簽名 | `enable_speed_ctrl(self, speed_rpm)` —— 只吃轉速 | `enable_speed_ctrl(self, speed_rpm=100, acc_time=5000, enable=True)` —— 多了加速時間與啟用開關，行為分支也不同 | ✅ **採用 `_50W`** |
| 4 | 角度追蹤演算法（`pos_step_motion_test` / 類似路徑） | 用 `self.target_angle = angle` 之後算 `diff_angle = self.target_angle - self.current_angle` | 用 `self.current_angle = angle` 之後算 `diff_angle = self.current_angle - self.previous_angle` | ⏸ **暫緩** — 需要先確認其他變數後再統整（見 §4） |
| 5 | `servo_on()` 重置行為 | 重置時保留 `previous_encoder`，並呼叫 `save_abs_home_pos(self.current_encoder)` 把目前位置存回設定檔 | 重置時單純把 `current_encoder`/`previous_encoder` 歸零，並清空 `accumulate_pulse`，**沒有**呼叫 `save_abs_home_pos` | ✅ **兩邊優點合併**（見下方 §2.3） |
| 6 | Logging 訊息與註解風格 | 訊息較長、有步驟編號註解（`# 1) ...` `# 2) ...`） | 訊息較精簡，用 `logging.basicConfig` 統一格式 | ✅ **兩邊優點合併，採標準 logging best practice**（見下方 §2.3） |
| 7 | 180 度以上目標角度的保護 | 沒有這個檢查 | `pos_step_motion_test` 開頭多了一段：若 `diff_pulses >= base_pulse_per_degree * 180` 就直接 return 0.0，不執行 | ⏸ **暫緩** — 需要先確認其他變數後再統整（見 §4） |

> 註：第 4、7 項牽涉到實際控制馬達移動角度的計算，屬於 CLAUDE.md §3
> 「硬體安全」範疇——這兩項合併前必須先確認清楚，避免馬達移動到錯誤角度。
> 兩項都還在暫緩狀態，尚未定案，實作階段不會動到這兩塊邏輯。

### 2.3 「兩邊優點合併」的具體做法（第 5、6 項）

**第 5 項 — `servo_on()` 重置行為：**
合併後建議的行為：
* 保留原版「讀取真實編碼器目前位置」的做法（`self.current_encoder = self.read_encoder_before_gear_ratio()`），而不是像 `_50W` 直接歸零——歸零假設馬達剛好在原點，讀真實值比較不會失真。
* 保留原版 `save_abs_home_pos(self.current_encoder)`，把目前位置持久化回設定檔。
* 同時採用 `_50W` 的 `self.accumulate_pulse = 0` 重置——這個欄位是 `_50W` 新增的狀態（累積脈衝數，用於連續運動追蹤），原版沒有這個欄位，但既然合併後的程式碼會包含 `_50W` 的連續運動功能（第 1、2 項已確認採用 `_50W`），這個狀態就必須在 `servo_on()` 時一併重置，避免殘留舊值。
* `previous_encoder` 的處理維持原版邏輯（保留舊值供比較），而非像 `_50W` 直接歸零。

**第 6 項 — Logging：**
合併後建議的做法：
* 採用 `_50W` 的 `logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')`，在模組層級統一設定一次，取代兩邊各自零散的呼叫方式。
* 使用 `logging.getLogger(__name__)` 取得 logger（而非直接呼叫模組層級的 `logging.info(...)`），符合 Python logging 的慣例作法。
* 訊息內容保留原版「有步驟脈絡」的可讀性（例如註明是讀取哪個階段、哪個暫存器），但精簡成單行、不需要額外的 `# 1) ...` 編號註解——註解本身可以拿掉，脈絡直接寫進 log 訊息裡即可。
* 一律使用 lazy `%s` 格式化（`logger.info("Current Encoder Value: %s", value)`）而非 f-string，這是 logging 模組的建議寫法（避免每次呼叫都先組字串，即使該筆 log 被 log level 濾掉也一樣）。

以上兩項的合併寫法會在實作（Green 階段）時真的套用；如果你看了觉得
方向不對，請在動手寫程式前先提出。

### 2.4 新發現：Encoder Overflow 風險（兩邊都有，尚未处理）

你提出「目前都沒考慮到 encoder 會 overflow 的問題」——查證後確認這是
**兩個資料夾都存在、合併前必須處理**的真實風險，不是單一變種的問題。

**驅動器規格（`SDE_English_manual_UL_v107.pdf` §8 Servo absolute
system，p.186-188）：**
* 這是**絕對值編碼器系統**（SME 系列，帶絕對型編碼器，需要後備電池），
  不是增量式編碼器。
* 官方定義的總脈衝數公式：`Total pulse counts = R × 4,194,304 + p`，
  其中 `R`（圈數）範圍 **-32768 ~ 32767**（16-bit 有號數），`p`（圈內
  脈衝）範圍 0 ~ 4,194,303（22-bit）。超出 ±32767 圈會觸發 `AL.29`
  警報。
* 驅動器本身有專門的狀態暫存器 `PA31 (APST)`，其中 bit0＝絕對位置遺失、
  bit1＝電池電壓過低、bit2＝**overflow**、bit4＝絕對座標系統尚未設定。
* 驅動器也提供 `PA32 (APR)` / `PA33 (APP)` 這組「圈數＋圈內脈衝」分開
  儲存的暫存器，範圍就是上面的 ±32767 圈全範圍。

**目前程式碼的實際讀法（兩個資料夾皆同）：**
* `read_encoder_before_gear_ratio()` 讀的是 `0x0000`（"Motor feedback
  pulses"），`read_encoder_after_gear_ratio()` 讀的是 `0x0024`
  （"Translated motor feedback pulses"）——manual 明載這兩個都是
  **2-word（32-bit）** 暫存器，且 `ModbusResponse.get_value()`
  是用 `int.from_bytes(..., byteorder='big')`（**無號**、未處理
  wraparound）直接組出整數。
* 32-bit 只能表示到 2^32；而編碼器解析度是 2^22 pulses/rev，所以
  `0x0000` 這個原始計數器只要馬達軸轉滿 **2^32 / 2^22 = 1024 圈**就會
  無聲無息地歸零重來——完全不會觸發驅動器的 `AL.29` 或 `PA31` overflow
  旗標，因為那些是驅動器內部對「絕對座標系統」的監控，跟這個
  Modbus 讀出來的原始 32-bit 累加值是兩回事。
* 換算到輸出軸（減速後）：400W（減速比 30）約 **34.1 圈**、50W（減速比
  10）約 **102.4 圈**輸出軸旋轉後，`0x0000` 讀出來的數字就會 wrap
  一次，且完全沒有任何警示。
* 目前程式碼**完全沒有**讀取 `PA31` 狀態、沒有使用 `PA32`/`PA33`，也沀
  有任何 wraparound / modulo 修正邏輯——`diff_angle`、`pos_step_motion_by`
  等所有角度計算都假設 `current_encoder` 是單調、不會繞回的數字。

**額外的風險（manual p.187「Operation restriction」）：**
manual 明確列出「不適合搭配絕對值系統使用」的操作條件，其中包含：
**(1) Speed control mode and torque control mode**、**(3) Single way
rotation（單方向持續旋轉）**。而兩個資料夾都有的連續轉動功能
（`motionStart_CW`/`motionStart_CCW`、`speed_ctrl_action` 連續模式）
正好就是這種用法——也就是說，合併後想保留的「連續旋轉」功能，原廠
本來就不建議跟這顆絕對值編碼器搭配使用，這跟上面的 32-bit wraparound
問題是同一組風險的兩個面向（用越久/轉越多圈，位置資料越不可靠）。

**✅ 已決定：採用方案 B** — 改讀 `PA32`(APR)+`PA33`(APP)，涵蓋全範圍
（±32767 圈），取代現在的 `0x0000`/`0x0024` 單一 32-bit 累加值。

**方案 B 的實作範圍（待 §2.2 第 4、7 項一併確認後才動工，見下方）：**

* **新增暫存器定義** — `PA31`(APST)、`PA32`(APR)、`PA33`(APP) 目前
  `servo_p_register.py` 完全沒有定義（連 `PA28`(ABS)/`PA29`(CAP)/
  `PA30`(UAP) 也沒有），需要仿照現有 `calculate_address()` 機制新增。
  對照 manual 位址對照表，實際 Modbus 位址：
  * `PA31` (APST，狀態) → `0x033C`
  * `PA32` (APR，圈數) → `0x033E`
  * `PA33` (APP，圈內脈衝) → `0x0340`
* **新的讀取/角度計算邏輯** — 改成 `current_pulse_total = APR × 4,194,304 + APP`，
  取代現有 `read_encoder_before_gear_ratio()` 的單一 32-bit 讀值。
  * `APR`（圈數）必須用**有號**方式解析（manual 定義範圍含負值），
    現有 `ModbusResponse.get_value()` 的 `int.from_bytes(...)`
    **沒有** `signed=True`，這個 bug 本身也需要一併修掉（不能只加
    PA32/PA33 讀取、卻沿用同一個有問題的無號解析函式）。
  * `APP`（圈內脈衝）維持無號解析即可（範圍 0~4,194,303，本來就不會
    是負值）。
* **前置條件確認** — manual 註明 PA32/PA33 「valid only when PA28 is
  set as 1」（絕對模式）。目前程式碼沒有任何地方寫入或檢查 `PA28`，
  代表這個設定可能是驅動器出廠/安裝時就設好的硬體端設定，而非軟體
  控制——**這點需要你確認實際硬體上 `PA28` 是否已經是 1**，否則
  PA32/PA33 讀出來的值可能無效。
* **`PA31` 狀態旗標** — 雖然方案 B 主要是換讀取位址解決 wraparound，
  但既然要新增 `PA31` 定義，建議一併在每次移動前順便檢查 bit0/bit1/
  bit2/bit4，等於把方案 A 的異常偵測也免費包進來（新增的暫存器都
  在同一批修改內，不算額外的合併範圍）。

**這個決定會回頭影響 §2.2 第 4、7 項** ——兩項原本暫緩的角度追蹤演算法
與 180 度保護，現在有了明確、涵蓋全範圍的位置來源（PA32/PA33），可以
在這個基礎上重新設計，不需要再各自沿用原本兩邊都基於「單一 32-bit
累加值」的舊寫法。等你確認 `PA28` 硬體設定後，我會回頭跟你確認第 4、7
項要不要一併用新的位置讀取方式重寫。

## 3. 提議的 JSON 設定檔格式（草案，尚未實作）

```json
{
  "active_profile": "shihlin_400W",
  "encoder_pulses_per_rev": 4194304,
  "profiles": {
    "shihlin_400W": {
      "baud_rate": 115200,
      "gear_ratio": 30,
      "abs_home_pos": 62369153
    },
    "shihlin_50w": {
      "baud_rate": 9600,
      "gear_ratio": 10,
      "abs_home_pos": 1184347
    }
  }
}
```

`encoder_pulses_per_rev` 是驅動器/編碼器的固定規格，放在 profile 外層、
兩個 profile 共用。`base_pulse_per_degree` 不再直接存在 JSON 裡，而是
程式讀取設定後，用 `encoder_pulses_per_rev * gear_ratio / 360` 算出來。

程式啟動時讀 `active_profile` 決定要套用哪一組數值，之後如果要切換馬達，
只需要改這個 JSON，不需要改程式碼或維護兩份資料夾。

這只是初稿，欄位名稱、要不要拆成獨立檔案（例如 `motor_profiles.json`
獨立於 `servo_config.json`）都還可以再討論。

## 4. 尚未決定的事項

1. **第 4、7 項（角度追蹤演算法、180 度保護）** — 你標記為「需要確認其他
   變數後再統整」，暫緩定案。這兩項都牽涉 `base_pulse_per_degree` 相關的
   角度/脈衝換算，屬於 CLAUDE.md §3 硬體安全範疇，實作階段會先跳過、
   不合併這部分邏輯，等你確認完再回來處理。
   若「其他變數」是指齒輪比、編碼器解析度（pulse/rev）等硬體規格數字，
   `docs/` 資料夾裡的 SDE 驅動器說明書應該有相關規格表——需要的話我可以
   幫忙對照文件查證，你再確認是否吻合實際量測值。
2. ~~合併後程式碼放哪裡~~ **✅ 已決定：新資料夾 `servo_comm_shihlin_unified/`**
   （已建立。目前已從 `servo_comm_shihlin_50W/` 複製通訊底層並新增 PA28
   防呆檢查，見 §2.4/§4 第 4 項；`servo_control.py` 主控邏輯本體仍待第
   4、7 項定案後才搬入）。`servo_comm_shihlin/` 與 `servo_comm_shihlin_50W/`
   會保留到新版本通過兩個 motor profile 的真實硬體驗證後才考慮處理，
   不會現在就刪除或標記棄用。
3. **驗證方式** — 合併後怎麼確認兩個 profile 都還能正確控制對應的馬達？
   （例如：兩個 profile 各自的角度計算、原點回歸都要用真實硬體驗證過
   一輪，才算完成——依 CLAUDE.md §3，這需要你在場確認。）
4. ~~Encoder overflow 處理方式~~ **✅ 已決定：方案 B**（改讀
   `PA32`+`PA33`，見 §2.4）。剩下待確認：實際硬體上 `PA28` 是否已設為
   1（絕對模式），否則 PA32/PA33 讀出來的值無效。**已建立防呆檢查**
   `servo_comm_shihlin_unified/check_pa28.py`（純讀取、可直接在樹莓派
   上對真實硬體執行，測試見 `test_absolute_mode_check.py`，7 項全過）
   ——請在 Pi 上執行後回報結果。
5. **§2.2 第 4、7 項最終寫法** — 現在有了方案 B 的全範圍位置來源，這兩項
   可以基於新的 PA32/PA33 讀取方式重新設計，不用再沿用兩邊各自的舊
   「單一 32-bit 累加值」寫法。等 `PA28` 確認後會回來一起定案。

## 5. 目前進度小結

* ✅ 第 1、2、3 項：確定採用 `_50W` 的行為（連續讀取取消機制、OSC 連續
  運動 handler、`enable_speed_ctrl` 新簽名）。
* ✅ 第 5、6 項：確定採「兩邊優點合併」，做法已寫在 §2.3。
* ⏸ 第 4、7 項：暫緩，等你確認相關變數後再統整——這兩項也是唯一還會
  卡住「合併後程式碼放哪裡」與進入 TDD Red 階段的項目。
* ✅ **Encoder overflow（§2.4）** — 已決定採用方案 B（改讀
  `PA32`+`PA33`，涵蓋全範圍，順便修掉 `get_value()` 缺少 `signed=True`
  的 bug）。待確認硬體 `PA28` 是否已設為 1。
* ⏸ 第 4、7 項：現在可以基於方案 B 的新位置讀取方式重新設計，等
  `PA28` 確認後一起定案。

## 6. 下一步
等第 4、7 項確認後，我會把本文件更新成正式的實作規格（含最終的
`motor_profiles.json` schema 與程式改動範圍），才進入 TDD 的 Red 階段
（先寫測試）。第 1、2、3、5、6 項已經可以先寫進規格，但為了讓整份
`servo_control.py` 一次到位、避免分兩次改動同一批檔案，建議等 4、7 項
一起確認後再開始動工。

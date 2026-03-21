# nrf52-voice 電源管理・診断機能 変更履歴（2026-03）

このドキュメントは 2026年3月に追加した以下の機能変更をまとめます。

- マイク電源の動的制御（録音時のみ ON）
- モーション連動録音の無効化（BLE クライアント手動制御のみ）
- ウォッチドッグタイマー（WDT）
- リセット原因ログ（電池切れ vs ファームウェアクラッシュ判別）
- タップ検出 BLE 通知
- Mac クライアント 自動再接続

---

## 1. マイク電源の動的制御

### 変更内容

| 変更前 | 変更後 |
|--------|--------|
| `regulator-boot-on` により起動時から常時 ON | 起動時は OFF、録音開始時に ON、停止時に OFF |

### 対象ファイル

- `boards/xiao_ble_nrf52840_sense.overlay` — `regulator-boot-on;` を削除
- `src/main.c` — `configure_mic_power()` / `mic_power_on()` / `mic_power_off()` を追加

### 制御フロー

```
BLE write 0x01 (録音開始)
  → mic_power_on()      GPIO P1.10 → HIGH、50ms 安定待ち
  → audio_capture_start()   DMIC_TRIGGER_START、DC 収束待ち
  → BLE Audio TX 通知 開始

BLE write 0x00 (録音停止)
  → audio_capture_stop()    DMIC_TRIGGER_STOP
  → mic_power_off()     GPIO P1.10 → LOW

BLE 切断時
  → audio_capture_stop()
  → mic_power_off()     接続がなければ必ずマイクを OFF
```

### 節約効果

| 状態 | 変更前 | 変更後 |
|------|--------|--------|
| モーション監視中（録音なし） | ~7.0 mA | ~6.3 mA |
| 録音中 | ~9–10 mA | 同じ |

マイク（MSM261D3526H1CPM）の待機電流 0.65 mA を録音していない間カット。アイドル時間が長いユースケースほど効果大。

### 注意点

- マイク電源 OFF の状態で `DMIC_TRIGGER_START` を呼ぶと、全サンプルが `-8`（定数）になる。**必ず `mic_power_on()` → 50ms 待ち → `audio_capture_start()` の順で呼ぶこと。**
- 古いドキュメント（`nrf52_voice_guide.md`）には `regulator-boot-on` が必須と書いてあるが、現在の実装ではコードで制御しているため不要。

---

## 2. モーション連動録音の無効化

### 変更内容

`process_motion_sample()` 内の以下の呼び出しを削除：

```c
on_motion_started();   // 削除
on_motion_settled();   // 削除
```

モーション検出自体（BLE Motion TX 通知）は引き続き動作する。録音の開始・停止は **BLE クライアントからの write のみ** で制御する。

### 現在の録音制御方法

| 方法 | 動作 |
|------|------|
| BLE write `0x01` | 録音開始（マイク ON → PDM 開始） |
| BLE write `0x00` | 録音停止（PDM 停止 → マイク OFF） |
| モーション検出 | 録音には影響しない（通知のみ） |

Mac クライアントでは `r` キーで録音開始、`s` キーで録音停止。

### 再有効化する場合

`process_motion_sample()` の該当箇所に以下を戻す：

```c
// on_motion_started() の再有効化（モーション検出時 → 録音開始）
last_motion_time_ms = now;
on_motion_started();   // ← ここに追加

// on_motion_settled() の再有効化（モーション停止時 → 録音停止）
notify_motion_event(0x00, ...);
on_motion_settled();   // ← ここに追加
```

`on_motion_started()` / `on_motion_settled()` 関数本体は `main.c` に残存しているため、呼び出しを戻すだけでよい。

---

## 3. ウォッチドッグタイマー（WDT）

### 目的

ファームウェアのハング（無限ループ、デッドロック）を自動検出してリセット。電池切れとクラッシュの区別に使用。

### 設定

```ini
# prj.conf
CONFIG_WATCHDOG=y
```

```c
// src/main.c
#define WDT_NODE        DT_NODELABEL(wdt0)
#define WDT_TIMEOUT_MS  5000   // 5秒

// main() 内: イメージ確定後に WDT を有効化
configure_watchdog();

// メインループ: 100ms ごとに WDT をキック
wdt_feed(wdt, wdt_channel_id);
```

### 動作

- イメージ確定（起動 3 秒後）の後に WDT をアーム
- メインループが 5 秒以上止まると WDT がリセットを発生
- `WDT_OPT_PAUSE_HALTED_BY_DBG` により、デバッガで一時停止中は WDT が停止（誤トリガー防止）

---

## 4. リセット原因ログ（電池切れ vs クラッシュ判別）

### 設定

```ini
# prj.conf
CONFIG_HWINFO=y
```

### 起動時の出力

```
Reset cause: 0x00000000 [POR — power-on / battery removed]
Reset cause: 0x00020000 [WATCHDOG — firmware hang]
Reset cause: 0x00040000 [SOFTWARE]
Reset cause: 0x00010000 [PIN — external reset]
Reset cause: 0x00080000 [CPU_LOCKUP]
```

### 判別フロー

```
デバイスが突然切断された
  ├─ Mac クライアントが再スキャンして数秒以内に再接続できた
  │    → シリアルで [WATCHDOG] を確認 → ファームウェアクラッシュ
  │
  └─ 何度スキャンしても見つからない
       → 電池切れ
       → 充電後に再接続し、シリアルで [POR] を確認
```

### nRF52840 の RESETREAS レジスタ仕様

| 値 | 意味 |
|----|------|
| `0x00000000` | POR（完全電源断→再投入）。電池が完全に空になった後の起動はこれ |
| `0x00010000` | リセットピン |
| `0x00020000` | WDT（ファームウェアハング） |
| `0x00040000` | ソフトウェアリセット（`sys_reboot()` など） |
| `0x00080000` | CPU ロックアップ |

---

## 5. Mac クライアント 自動再接続

### 変更内容（`mac_client/nrf52_voice_client.py`）

```python
# 接続時: 接続時刻を記録し、切断コールバックを登録
self.client = BleakClient(
    address,
    disconnected_callback=self._on_disconnected,
    ...
)
self._connect_time = time.time()

# 切断コールバック: 接続継続時間をログ出力
def _on_disconnected(self, client):
    elapsed = time.time() - self._connect_time
    print(f"  [DISCONNECT] After {elapsed:.0f}s connected")
    if self.recorder.is_recording:
        self.recorder.stop_recording()

# main(): 切断検出後に自動再スキャン・再接続
while not control_task.done():
    if not client.is_connected:
        # 3秒間隔でスキャン
        # デバイス発見後に reconnect
```

### 動作フロー

```
予期しない切断
  → [DISCONNECT] After Xs connected をログ出力
  → WAV 録音中なら停止・保存
  → 3秒ごとに VoiceBridge52 をスキャン
  → 発見次第、オフライン時間を表示して再接続
  → インタラクティブコントロール再開
```

ファームウェアクラッシュ（WDT リセット後の自動再起動）なら数秒で再接続される。電池切れなら再接続されない。

---

## 電力消費の概要（40mAh 運用）

| コンポーネント | 録音なし | 録音中 |
|---------------|---------|--------|
| nRF52840 CPU (64MHz) | 4.6 mA | 4.6 mA |
| BLE ラジオ（アイドル） | 0.4 mA | — |
| BLE ラジオ（音声送信中） | — | 2–3 mA |
| PDM マイク | **0 mA** ← 電源 OFF | 0.65 mA |
| LSM6DS3TR-C (416Hz HP) | 0.65 mA | 0.65 mA |
| 基板クワイエセント | 0.5 mA | 0.5 mA |
| **合計** | **~6.2 mA** | **~9.4 mA** |

**40mAh 理論稼働時間:** モーション監視のみ ~6.5 時間 / 連続録音 ~4.3 時間

> **注:** 電池残量ゼロから接続した場合、数十秒で切断されることがある（残量わずかな状態での接続）。シリアルの `Reset cause` で切断原因を確認すること。

# Atom Echo S3R ファームウェア 動作確認記録・問題点

作成日: 2026-03-09

---

## 作業概要

ESP32S3 (XIAO) 向けファームウェアを M5Stack Atom Echo S3R 向けに移植し、
ビルド・フラッシュ・動作確認を実施した。

---

## 完了した作業

| 手順 | 結果 |
|------|------|
| `atom_echo_s3r/` ディレクトリ作成 | ✅ |
| `idf_component.yml` に `espressif/es8311` 追加 | ✅ |
| `main.c` を I2S PDM → I2S Standard + ES8311 に移植 | ✅ |
| ビルド (`idf.py build`) | ✅ エラーなし |
| フラッシュ (`idf.py flash`) | ✅ 成功、572KB 書込み確認 |
| デバイス USB 認識 | ✅ macOS で `/dev/cu.usbmodem1101` として列挙 |

---

## 発見した問題点

### 問題 1: `es8311_create()` API 不一致

**症状:** 初回ビルドで以下のコンパイルエラー

```
error: incompatible type for argument 1 of 'es8311_create'
  es8311_dev = es8311_create(i2c_bus, ES8311_I2C_ADDR);
```

**原因:**
`espressif/es8311` コンポーネントは**旧来の Legacy I2C API** を使用している。
計画では新 API (`i2c_master_bus_handle_t`) を想定していたが、実際のコンポーネントシグネチャは:

```c
// 実際のシグネチャ (espressif/es8311 v0.0.3)
es8311_handle_t es8311_create(const i2c_port_t port, const uint16_t dev_addr);
```

**対処:**
- `driver/i2c_master.h` → `driver/i2c.h` (Legacy) に変更
- `i2c_new_master_bus()` → `i2c_param_config()` + `i2c_driver_install()` に変更
- `CMakeLists.txt` の `esp_driver_i2c` → `driver` に変更
- `es8311_create(i2c_bus, ...)` → `es8311_create(I2C_NUM_0, ...)` に変更

---

### 問題 2: `ESP_ERROR_CHECK` によるクラッシュループ

**症状:** BLE がアドバタイズしない、USB ポートは認識されるが動作不安定

**原因:**
`audio_stream_task` 内で `ESP_ERROR_CHECK(init_audio())` を呼んでいたため、
I2C/ES8311 の初期化が失敗した場合に `abort()` → パニック → リブートのループに入る。
BLE スタックが起動する前にクラッシュするため、アドバタイズが開始されない。

**対処:**
`init_audio()` の各ステップを `ESP_ERROR_CHECK` から `if (ret != ESP_OK)` + ログ出力に変更。
失敗時はタスクを `vTaskDelete(NULL)` で終了させ、BLE は継続動作させる。

```c
// 修正後
esp_err_t audio_ret = init_audio();
if (audio_ret != ESP_OK) {
    ESP_LOGE(TAG, "Audio init failed (%s) — audio task exiting",
             esp_err_to_name(audio_ret));
    vTaskDelete(NULL);
    return;
}
```

---

### 問題 3: GPIO 0 ストラッピングピン問題 (未解決)

**症状:**
- `idf.py monitor` 接続時にデバイスが **ダウンロードモード** に遷移する
- シリアルログ: `rst:0x15 (USB_UART_CHIP_RESET), boot:0x33 (DOWNLOAD(USB/UART0))`
- BLE アドバタイズが確認できない

**原因:**
現在の `atom_echo_s3r/main.c` の設定では I2C SCL ピンが **GPIO 0** で、これは ESP32-S3 の **ブートモード ストラッピングピン**。

| GPIO 0 レベル | 起動モード |
|--------------|-----------|
| HIGH | 通常起動 (SPI Flash Boot) |
| LOW  | ダウンロードモード |

`idf_monitor` がモニター接続に伴ってデバイスをリセットする際、
そのリセット瞬間に GPIO 0 が LOW にサンプリングされる可能性がある。
これは I2C SCL ラインに接続された ES8311 コーデック、または基板の配線が
リセット時に GPIO 0 を LOW に引っ張っていることを示唆する。

**確認事項 (未実施):**
1. M5Stack Atom Echo S3R の回路図で GPIO 0 (SCL) に外部プルアップ抵抗が実装されているか確認
2. `idf.py monitor --no-reset` で再起動なしにログ取得を試みる
3. ES8311 の SCL ピンが起動時に LOW を出力していないか確認

**切り分け案:**

**案 A: I2C SCL を別のピンに変更**
GPIO 0 を SCL として使わず、別の安全なピンに変更する。
ただし Atom Echo S3R の物理的な配線が GPIO 0 固定の場合は不可。

**案 B: `--no-reset` フラグでモニター接続**
```bash
idf.py -p <serial-port> monitor --no-reset
```
デバイスをリセットせずにログを閲覧し、実際のエラーを特定する。

---

### 問題 4: `idf.py monitor` の TTY 要件 (非 TTY 環境での制限)

**症状:**
非 TTY 環境から `idf.py monitor` を実行すると:
```
Error: Monitor requires standard input to be attached to TTY.
```

**原因:**
`idf_monitor.py` がインタラクティブ TTY を必要とするため、
パイプや非 TTY 環境では動作しない。

**対処:**
- インタラクティブなターミナルからモニターを実行する
- 例:
```bash
cd /path/to/voice-bridge-ble
source ~/esp/esp-idf/export.sh
idf.py -p <serial-port> monitor --no-reset
```

---

## 次のアクション

1. **インタラクティブなターミナルで `idf.py -p <serial-port> monitor --no-reset` を実行** してシリアルログを確認する
2. ログに `I2C master initialized` / `ES8311 codec initialized` が出るか確認
3. 出ない場合はエラーメッセージを確認し、I2C ピン設定・ES8311 I2C アドレス・配線/プルアップを確認する
4. BLE アドバタイズを `AtomEchoS3R` という名前で確認する
5. GPIO 0 ストラッピング問題が再現する場合は I2C SCL ピンの変更を検討

---

## 環境情報

| 項目 | 値 |
|------|-----|
| ESP-IDF バージョン | v5.4 (`~/esp/esp-idf`) |
| ターゲットチップ | ESP32-S3-PICO-1 (LGA56) rev 0.2 |
| Flash サイズ | 8MB (GD Embedded) |
| PSRAM | 8MB (AP_3v3 Embedded) |
| USB ポート | 例: `/dev/cu.usbmodem1101` |
| `es8311` コンポーネント | `espressif/es8311` (managed, v0.0.3+) |
| BLE デバイス名 | `AtomEchoS3R` |

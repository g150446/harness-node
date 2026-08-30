# BLE OTA update notes (HarnessNode)

この手順は、XIAO nRF52840 Sense の MCUboot + MCUmgr/SMP over BLE OTA を実行するエージェント向けの必須注意事項です。

## 事前確認

1. 対象は `HarnessNode` であることを確認する。別のBLEデバイスへ書き込まない。
2. `nordic-main/prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` を、実機のslot 0より新しい値へ更新する。同じバージョンや同じハッシュを再送しない。
3. `xiao_ble/nrf52840/sense` と `--sysbuild` を使用する。生成物は `nordic-main/ota_update.bin` とする。
4. OTA中はデバイスの電源を切らず、BLEクライアント（Androidアプリ等）を同時接続させない。

## ビルド時の既知問題

- NCSのZephyrキャッシュへ書き込めない場合、`Operation not permitted` や `ToolchainCapabilityDatabase` エラーになる。SDKを変更せず、書込み可能なユーザーキャッシュを使うか、必要な権限を得てから再実行する。ビルド成果物だけを信用してアップロードを続行しない。
- `build_and_package_ota.sh` はパッケージ生成のみで、デバイスを書き込まない。signed imageの存在とサイズ（OTA slot上限以内）を確認する。

## macOSでのOTA実行

- `mac_client/ota_updater.py` のBleak/CoreBluetoothスキャンは、非対話のバックグラウンド実行でmacOSプロセスが `SIGABRT` になることがある。OTAはユーザーセッションのTerminalで実行する。
- Terminalをエージェントが開いた場合は、ウィンドウまたはタブIDを記録し、アップロードと検証が終わったら必ず閉じる。既存のユーザーTerminalは閉じない。
- 失敗時に同じイメージを無条件で再送しない。まず対象名、接続状態、slot 0/slot 1のversion・hashを読み直す。

## 完了条件

`ota_updater.py` が次の状態を報告するまで完了とみなさない。

- `OTA verified: uploaded image is active and confirmed in slot 0`
- versionが今回の署名バージョンと一致する
- slot 0 の `active=true`、`confirmed=true` を確認する

検証前にUSB抜去、電源断、別ファームウェアの上書きを行わない。

## 更新履歴

- 2026-08-30: 0.0.77（single候補遅延確定によるdouble回帰修正）をビルド・OTA。
  slot 0で`active=true`、`confirmed=true`、version=`0.0.77`を確認済み。

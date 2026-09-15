[開発メモ](NOTE.md)

# BLE-APRS

> **Under development / 開発中**

BLE Extended Advertising（LE Coded PHY）を利用し、APRS / AX.25フレームを非接続型ブロードキャストで送受信する実験プロジェクトです。USB Serial/JTAGによるKISS TNC入出力、GPS/NMEAトラッカー、BME280気象情報、受信中継機能を実装しています。

現在は次の2ボードを対象にしています。

- Seeed Studio XIAO ESP32-C3
- Seeed Studio XIAO ESP32-C6

## ディレクトリ

```text
32c3/  XIAO ESP32-C3用ESP-IDFプロジェクトとsdkconfig
32c6/  XIAO ESP32-C6用ESP-IDFプロジェクトとsdkconfig
img/   説明用画像
```

C3/C6のアプリケーションソースは同一内容です。`sdkconfig`とビルドキャッシュを別々に持つため、ターゲットを切り替えずにビルドできます。

## 主な機能

- BLE 5 Extended Advertising / Extended Scan
- LE Coded PHY
- BLE-APRS送受信
- USB Serial/JTAG KISS TNC
- GPS/NMEAによるAPRSトラッカー
- BME280温度・湿度・気圧
- Adaptive Relay Interval、nonce重複抑制、token bucket
- GPSを測位できない場合、一定時間後から`NOFIX`を生存信号として送信
- GPS測位後に欠測が続いた場合、最終測位からの経過時間を生存信号として送信
- GPS・Radio・USB状態表示用PL9823 LED

BLE payload形式：

```text
$APRS,1,<relay_interval>,<nonce>>AX.25_FRAME
```

## 配線

| 機能 | XIAO ESP32-C3 | XIAO ESP32-C6 |
|---|---:|---:|
| GPS RX (D7) | GPIO20 | GPIO17 |
| BME280 SDA (D4) | GPIO6 | GPIO22 |
| BME280 SCL (D5) | GPIO7 | GPIO23 |
| PL9823 | GPIO10 | GPIO18 (D10) |

ESP32-C6では基板上のRFアンテナスイッチも初期化します。外部U.FL／内蔵アンテナはソースの `XIAO_C6_USE_EXTERNAL_ANTENNA` で選択します。

## ビルド環境

- ESP-IDF v5.4系
- `espressif/led_strip` 3.0.3

リポジトリを取得し、ESP-IDF環境を有効化してから対象ディレクトリでビルドします。

```bash
git clone https://github.com/CQAKIBA/ble-aprs.git
cd ble-aprs
```

### XIAO ESP32-C3

```bash
cd 32c3
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

### XIAO ESP32-C6

```bash
cd 32c6
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

ポート名は環境に合わせて変更してください。初回または設定変更時は、それぞれのディレクトリで `idf.py menuconfig` を実行します。

## 注意

実験中の実装です。仕様、パケット形式、動作、配線、ビルド設定は今後変更される可能性があります。無線運用時は使用地域の法令・規則を確認してください。

// BLE-APRS experimental bridge for ESP-IDF / ESP32-C3, ESP32-C6 class devices
//
// Function:
//   UART KISS input  -> BLE Extended Advertising, LE Coded PHY
//   BLE-APRS receive -> UART KISS output
//
// BLE-APRS v0 payload:
//   "$APRS,1,<relay_interval_sec>,<nonce>>" + raw AX.25 frame without FCS
//
// Adaptive Relay Interval:
//   BLE-APRS continuously advertises the latest packet and dynamically changes
//   only the requested I-Gate relay interval according to speed and heading.
//   The I-Gate remains authoritative for the actual relay timing.
//
//   Historical note: The initial idea was inspired by APRS SmartBeaconing.
//   This BLE-APRS implementation adapts that idea to a continuous-broadcast
//   transport, where relay timing and RF advertising timing are independent.
//
// Notes:
//   - This is an experimental first cut.
//   - Use ESP-IDF with Bluedroid BLE 5.0 extended advertising enabled.
//   - Some ESP-IDF versions/chips expose slightly different extended scan report field names.
//     See the comment in gap_cb() if your build fails there.
//
// Suggested sdkconfig items:
//   CONFIG_BT_ENABLED=y
//   CONFIG_BT_BLE_50_FEATURES_SUPPORTED=y
//   CONFIG_BT_BLE_42_FEATURES_SUPPORTED=n  // BLE 5.0と同時には有効化しない
//   CONFIG_BT_BLUEDROID_ENABLED=y
//   CONFIG_BT_BLE_DYNAMIC_ENV_MEMORY=y
//   CONFIG_BT_BLE_50_EXTEND_ADV_EN=y   // Bluedroid Extended Advertising
//   CONFIG_BT_BLE_50_EXTEND_SCAN_EN=y  // Bluedroid Extended Scanning
// Controller-side BLE 5 / Extended Advertising / Coded PHY options are normally
// selected by the target in current ESP-IDF. Kconfig names vary between releases.
//   CONFIG_BT_BLE_FEAT_ADV_CODING_SELECTION=y  // optional, for S=8/S=2 preference if available
//
// Three PL9823 status LEDs (daisy-chained from the board-specific STATUS_LED_GPIO):
//   LED 0 USB   : TX red / RX green / KISS blue; overlapping events mix colors.
//   LED 1 GPS   : GSV SNR-detected satellite count by color, fix type by blink duty cycle.
//                 0=red, 1-2=yellow, 3-4=green, 5-6=cyan, 7+=blue.
//   LED 2 RADIO : advertising red blink + BLE-APRS RX strong blue / weak green flash;
//                 overlapping TX/RX activity mixes colors.
//   XIAO ESP32-C6 onboard LED (GPIO15, active-low):
//                 mirrors valid BLE-APRS RX flashes for simple I-Gate diagnostics.
// LED effects are non-blocking and rendered by a dedicated task.
//
// Nonce / relay behavior:
//   KISS input gets a new nonce for every valid frame. The built-in GPS tracker
//   also rebuilds its latest payload/nonce every second while active.
//   Adaptive Relay Interval changes relay_interval, not the BLE advertising cadence.
//   The I-Gate suppresses only same nonce and relay_interval violations; when a
//   newly received packet requests a smaller interval, it is compared against the
//   I-Gate's own elapsed time since the station was last relayed.

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/usb_serial_jtag.h"
#include "led_strip.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#define TAG "BLE_APRS"  // ログ出力タグ

// ---------------- User settings ----------------

// ---------------- XIAO board definition / minimal ESP-IDF HAL ----------------
// Keep board wiring and board-only initialization here so that the APRS/GPS/BLE
// core below remains common.  This is intentionally small rather than a broad
// refactor, and can later be replaced by a platform HAL for non-ESP targets.
#if CONFIG_IDF_TARGET_ESP32C3

#define BLE_APRS_BOARD_NAME       "Seeed XIAO ESP32-C3"
#define BLE_APRS_PLATFORM_COMMENT "2.4GHz LE Coded PHY Advertise APRS by ESP-C3"
#define BOARD_GPS_RX_GPIO         GPIO_NUM_20  // XIAO ESP32-C3 D7
#define BOARD_BME280_SDA_GPIO     GPIO_NUM_6   // XIAO ESP32-C3 D4
#define BOARD_BME280_SCL_GPIO     GPIO_NUM_7   // XIAO ESP32-C3 D5
#define BOARD_STATUS_LED_GPIO     GPIO_NUM_10  // 既存配線: PL9823 DIN

#elif CONFIG_IDF_TARGET_ESP32C6

#define BLE_APRS_BOARD_NAME       "Seeed XIAO ESP32-C6"
#define BLE_APRS_PLATFORM_COMMENT "2.4GHz LE Coded PHY Advertise APRS by ESP-C6"
#define BOARD_GPS_RX_GPIO         GPIO_NUM_17  // XIAO ESP32-C6 D7
#define BOARD_BME280_SDA_GPIO     GPIO_NUM_22  // XIAO ESP32-C6 D4
#define BOARD_BME280_SCL_GPIO     GPIO_NUM_23  // XIAO ESP32-C6 D5
// D10/GPIO18 is exposed on the XIAO header and is otherwise unused here.
// Connect the first PL9823 DIN to XIAO D10 when building for ESP32-C6.
#define BOARD_STATUS_LED_GPIO     GPIO_NUM_18  // XIAO ESP32-C6 D10: PL9823 DIN
#define BOARD_RF_SWITCH_POWER_GPIO GPIO_NUM_3  // Low enables antenna-switch control
#define BOARD_RF_ANT_SELECT_GPIO   GPIO_NUM_14 // Low=ceramic, High=external U.FL
#define BOARD_ONBOARD_LED_GPIO     GPIO_NUM_15 // XIAO ESP32-C6内蔵LEDカソード
#define BOARD_ONBOARD_LED_ON       0           // 内蔵LEDはLowアクティブ
#define BOARD_ONBOARD_LED_OFF      1           // 内蔵LED消灯レベル

// XIAO ESP32-C6で使用するアンテナを選択します。
// 1: 外部U.FLアンテナ（既定） / 0: 基板上の内蔵セラミックアンテナ
#define XIAO_C6_USE_EXTERNAL_ANTENNA 0
#if (XIAO_C6_USE_EXTERNAL_ANTENNA != 0) && (XIAO_C6_USE_EXTERNAL_ANTENNA != 1)
#error "XIAO_C6_USE_EXTERNAL_ANTENNA must be 0 or 1"
#endif

#else
#error "This source supports only ESP32-C3 and ESP32-C6 targets"
#endif

// XIAO ESP32-C3 usually exposes the ESP32-C3 native USB Serial/JTAG interface on USB-C.
// If you connect an external USB-UART adapter to hardware UART pins instead, set this to 0.
#define USE_USB_SERIAL_JTAG    1  // 1: USB Serial/JTAGをKISS入出力に使用

#define UART_PORT              UART_NUM_0  // KISS用UARTポート
#define UART_BAUD              9600  // KISS用UART通信速度
#define UART_RX_BUF            2048  // KISS用UART受信バッファ
#define UART_TX_BUF            2048  // KISS用UART送信バッファ

// GPS NMEA input. Physical pins are selected in the XIAO board section above.
#define ENABLE_GPS_NMEA        1  // 1: GPS NMEA入力を有効化
#define GPS_UART_PORT          UART_NUM_0  // GPS用UARTポート
#define GPS_UART_BAUD          9600  // GPS NMEA通信速度
#define GPS_UART_RX_PIN        BOARD_GPS_RX_GPIO  // GPS RXピン（両ボードともXIAO D7）
#define GPS_UART_TX_PIN        UART_PIN_NO_CHANGE  // GPS TXは未使用
#define GPS_UART_RX_BUF        2048  // GPS UART受信バッファ
#define GPS_DEBUG_LOG_EVERY    0  // GPS詳細ログ間隔、0で無効
#define GPS_STATUS_LOG_MS       10000  // GPS状態ログ間隔ms、0で無効

// Built-in GPS beacon AX.25/APRS identity. Replace N0CALL before on-air use.
#define GPS_APRS_SRC_CALL      "JA1UMW"  // GPSビーコン送信元コール
#define GPS_APRS_SRC_SSID      9  // GPSビーコン送信元SSID
#define GPS_APRS_DST_CALL      "BT32C3"  // GPSビーコン宛先コール
#define GPS_APRS_DST_SSID      0  // GPSビーコン宛先SSID
#define GPS_APRS_SYMBOL_TABLE  '/'  // APRSシンボルテーブル
#define GPS_APRS_SYMBOL_CODE   '['  // APRSシンボル（徒歩）

// Environmental data representation used in the environment block after positioning data.
// WX   : APRS weather tokens such as t087h51b10070.  With a non-'_' symbol,
//        services such as aprs.fi may display these as ordinary comment text.
// TEXT : Human-readable metric form such as 30.6C 51% 1007.0hPa.
#define GPS_APRS_ENV_FORMAT_WX    0  // 環境値をAPRS WX形式にする値
#define GPS_APRS_ENV_FORMAT_TEXT  1  // 環境値をテキスト形式にする値
#define GPS_APRS_ENV_FORMAT       GPS_APRS_ENV_FORMAT_TEXT  // 実際に使う環境値形式

// Put the GPS UTC time into the standard APRS position timestamp field.
// 1: Position packet starts with "/HHMMSSh" (UTC hours/minutes/seconds).
// 0: Position packet starts with "!" and carries no APRS timestamp.
// This preserves the data-generation time even when a BLE advertisement is
// received and gated some time after it was originally generated.
#define GPS_APRS_USE_TIMESTAMP    1  // 1: APRSにGPS UTC時刻を付加

// Beacon/state update cadence and stale-position status.
// A valid position is retained after GPS loss so the station can keep reporting
// live environmental data while clearly indicating the age of the position.
#define GPS_BEACON_UPDATE_INTERVAL_MS 1000  // GPS/APRS内容の更新周期ms
#define GPS_FIX_CURRENT_MAX_AGE_SEC   3  // 現在FIX扱いする最大経過秒
#define GPS_FIX_STATUS_AFTER_SEC      60  // FIX古さ警告を出す開始秒
#define GPS_FIX_STATUS_MAX_MINUTES    780  // FIX警告を分表示する上限
#define GPS_FIX_STATUS_MAX_HOURS      999  // FIX警告の最大時間表示
#define GPS_NOFIX_INITIAL_DELAY_SEC   60  // 起動直後のNOFIX送出待ち時間秒

// Cold-start NOFIX packet format.
// APRS User-Defined Data uses '{' as the data type identifier.  A second '{'
// is the experimental User ID reserved for unregistered experiments, and 'N'
// is the BLE-APRS local subtype used here for a NOFIX/health beacon.  Keep one
// ASCII space after "{{N" so raw-packet displays remain easy for humans to read.
// This packet intentionally contains no position.  A compatible APRS client may
// apply Vicinity Plot behavior; other clients can safely leave it unpositioned.
#define GPS_APRS_NOFIX_USERDEF_PREFIX "{{N "  // cold-start NOFIX用User-Defined Data接頭辞

// Adaptive Relay Interval: dynamic I-Gate relay request.
//
// This does NOT throttle BLE advertising or freeze the APRS payload. The tracker
// continuously refreshes the latest AX.25 payload/nonce, while only relay_interval
// changes according to speed and heading. The I-Gate compares that requested
// interval with its own elapsed time since the station was last relayed, so a
// newly smaller interval can pass immediately.
//
// The speed-rate curve is linear between SLOW and FAST endpoints. A sufficiently
// large heading change can request MIN_TURN_TIME_SEC. GPS FIX loss/reacquisition
// is intentionally NOT a relay-priority event: FIX freshness is represented in
// the APRS payload itself, while relay timing remains driven only by movement.
// Any newly shorter request is kept on-air for 2/3 of that interval to improve
// the chance that a connectionless BLE advertisement reaches an I-Gate.
// Set GPS_ADAPTIVE_RELAY_INTERVAL to 0 to use GPS_APRS_RELAY_INTERVAL_SEC as a fixed
// relay request while still refreshing the payload/nonce every second.
#define GPS_ADAPTIVE_RELAY_INTERVAL          1  // 1: Adaptive Relay Intervalを有効化
#define GPS_ARI_FAST_SPEED_KMH        100.0  // 高速側の基準速度km/h
#define GPS_ARI_FAST_RATE_SEC         40  // 高速時の中継希望間隔秒
#define GPS_ARI_SLOW_SPEED_KMH        5.0  // 低速側の基準速度km/h
#define GPS_ARI_SLOW_RATE_SEC         300  // 低速時の中継希望間隔秒
#define GPS_ARI_MIN_TURN_TIME_SEC     15  // 旋回時の短縮中継希望間隔秒
#define GPS_ARI_MIN_TURN_ANGLE_DEG    10.0  // 旋回判定の最低角度
#define GPS_ARI_TURN_SLOPE            240.0  // 速度に応じる旋回判定係数
#define GPS_ARI_HOLD_NUMERATOR        2  // 短い間隔を保持する時間の分子
#define GPS_ARI_HOLD_DENOMINATOR      3  // 短い間隔を保持する時間の分母（既定2/3）
#if GPS_ARI_HOLD_DENOMINATOR == 0
#error "GPS_ARI_HOLD_DENOMINATOR must not be zero"
#endif

// Free-text comment appended after APRS machine-readable fields.
// IMPORTANT when GPS_APRS_ENV_FORMAT_WX is selected:
// Some APRS weather parsers scan trailing text for additional WX tokens. Avoid
// accidental token-like substrings such as cNNN, sNNN, gNNN, tNNN, rNNN,
// pNNN, PNNN, hNN, bNNNNN, LNNN/lNNN and #NNN. For example, "ESP32C3"
// contains "P32" and may be misread as P032 = 0.32 inch midnight rain.
// Keep WX-mode free text deliberately simple, or separate letters from digits.
#define GPS_APRS_COMMENT       BLE_APRS_PLATFORM_COMMENT  // APRS末尾のフリーテキスト（C3の既存文言を保持）
#define GPS_APRS_RELAY_INTERVAL_SEC DEFAULT_RELAY_INTERVAL_SEC  // ARI無効時の固定中継希望間隔

// BME280 environmental sensor on each XIAO board's D4/D5 I2C pins.
#define ENABLE_BME280           1  // 1: BME280を有効化
#define BME280_I2C_PORT         I2C_NUM_0  // BME280用I2Cポート
#define BME280_I2C_ADDR         0x76  // BME280 I2Cアドレス
#define BME280_SDA_PIN          BOARD_BME280_SDA_GPIO  // BME280 SDA（XIAO D4）
#define BME280_SCL_PIN          BOARD_BME280_SCL_GPIO  // BME280 SCL（XIAO D5）
#define BME280_I2C_FREQ_HZ      100000  // BME280 I2CクロックHz
#define BME280_READ_INTERVAL_MS 2000  // BME280読取周期ms
#define BME280_STATUS_LOG_MS    10000  // BME280状態ログ間隔ms


// 250 bytes total BLE-APRS payload budget.
#define BLE_APRS_MAX_TOTAL     250  // BLE-APRS全体の最大byte
#define AX25_MAX               230  // AX.25フレーム最大byte

#define DEFAULT_RELAY_INTERVAL_SEC 300  // 標準の中継希望間隔秒
#define APRS_MESSAGE_RELAY_INTERVAL_SEC 0  // APRSメッセージの中継希望間隔、0=即時
#define KISS_ADVERTISE_LIFETIME_SEC 600  // KISS由来広告の保持時間秒
#define KISS_PRIORITY_SEC 60  // KISS入力をGPSより優先する秒数

// BLE advertising interval. 300 ms is gentle for a first range test.
#define ADV_INTERVAL_MS        300  // BLE Advertising周期ms
#define EXT_ADV_INSTANCE       0  // Extended Advertisingインスタンス番号

// BLE advertising/scan interval unit is 0.625 ms.
// Some ESP-IDF versions do not provide ESP_BLE_GAP_ADV_ITVL_MS() /
// ESP_BLE_GAP_SCAN_ITVL_MS(), so keep the conversion local and constant-safe.
#define BLE_MS_TO_0_625MS_UNITS(ms) ((uint16_t)(((uint32_t)(ms) * 1000U) / 625U))  // msをBLE 0.625ms単位へ変換

// Enable both for two boards that can act as bridge/repeater-ish test nodes.
#define ENABLE_KISS_TO_BLE     1  // 1: KISS→BLEを有効化
#define ENABLE_BLE_TO_KISS     1  // 1: BLE→KISSを有効化

// Debug helper: echo ordinary text typed outside KISS frames.
// This makes it easy to verify that Tera Term input reaches the ESP app.
#define ECHO_NON_KISS_TEXT     1  // 1: 通常文字入力をエコーバック

// Debug helper: after outputting a KISS binary frame, also print human-readable text.
// This text is outside KISS FEND frames, so real KISS clients should treat it as garbage/ignore it.
// Set to 0 when connecting to strict software if needed.
#define KISS_DEBUG_MONITOR_TEXT 0  // 1: KISS後に可読デバッグ文字も出力

// Gateway-side minimum interval for relaying BLE packets to UART KISS.
// This is receiver-side protection only.
#define GW_MIN_RELAY_INTERVAL_SEC 1  // I-Gateが許す最小中継間隔秒
#define GW_MAX_RELAY_INTERVAL_SEC 3600  // I-Gateが許す最大中継間隔秒
#define STATION_CACHE_TTL_SEC     3600  // I-Gate局キャッシュ保持秒

// Append the BLE receive level to APRS information text when there is room.
// Example:  [RX:-80dB] (leading space is part of the appended tag)
// The tag is added only on the gateway -> KISS path; the original BLE packet
// and its station/nonce identity are left untouched.
#define GW_APPEND_RX_LEVEL        1  // 1: I-Gateで受信RSSIを付加

// Fail-safe token bucket per station.
// Normal safe intervals should never hit this. It only catches bursts/misconfiguration.
#define GW_TOKEN_MAX              10  // 局ごとの最大トークン数
#define GW_TOKEN_REFILL_MS        6000UL  // トークン1個の回復時間ms
#define GW_BAN_TIME_MS            300000UL  // 異常バースト時の遮断時間ms

// BLE GAP callbackからUSBへ直接書き込まず、専用TX taskへ渡すためのQueue。
// KISSはRAM上で1フレームへエンコードしてからUSBへ一括writeする。
#define GW_KISS_TX_QUEUE_LEN      8  // BLE→KISS送信Queueに保持する最大フレーム数
#define GW_KISS_TX_TIMEOUT_MS     1000  // USB/UART KISS送信完了待ち時間ms

// Initialize only hardware that exists on a specific XIAO board.  Seeed's C6
// design requires GPIO3 low before GPIO14 can select the RF path.
static void board_init(void) {
#if CONFIG_IDF_TARGET_ESP32C6
    gpio_config_t rf_gpio_config = {
        .pin_bit_mask = (1ULL << BOARD_RF_SWITCH_POWER_GPIO) |
                        (1ULL << BOARD_RF_ANT_SELECT_GPIO) |
                        (1ULL << BOARD_ONBOARD_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rf_gpio_config));
    // GPIO15 is connected to the cathode of the XIAO ESP32-C6 onboard LED.
    // Keep it off until a valid BLE-APRS packet is received.
    ESP_ERROR_CHECK(gpio_set_level(BOARD_ONBOARD_LED_GPIO, BOARD_ONBOARD_LED_OFF));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_RF_SWITCH_POWER_GPIO, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_RF_ANT_SELECT_GPIO,
                                  XIAO_C6_USE_EXTERNAL_ANTENNA ? 1 : 0));
    ESP_LOGI(TAG, "%s RF antenna: %s", BLE_APRS_BOARD_NAME,
             XIAO_C6_USE_EXTERNAL_ANTENNA ? "external U.FL" : "on-board ceramic");
#endif
}

// ---------------- PL9823 status LEDs ----------------

#define ENABLE_STATUS_LEDS          1  // 1: 3連PL9823状態LEDを有効化
#define STATUS_LED_GPIO             BOARD_STATUS_LED_GPIO  // PL9823 DIN接続GPIO（ボード定義参照）
#define STATUS_LED_COUNT            3  // 状態LED個数
#define STATUS_LED_USB              0  // USB状態LED番号
#define STATUS_LED_GPS              1  // GPS状態LED番号
#define STATUS_LED_RADIO            2  // BLE無線状態LED番号
#define STATUS_LED_BRIGHTNESS       8  // LED明るさ0～255
#define STATUS_LED_TASK_INTERVAL_MS 20  // LED表示更新周期ms
#define STATUS_USB_FLASH_MS         500  // USBイベント点灯時間ms
#define STATUS_USB_FLASH_TICKS      ((STATUS_USB_FLASH_MS + STATUS_LED_TASK_INTERVAL_MS - 1) / STATUS_LED_TASK_INTERVAL_MS)  // USB点灯保持用tick数
#define STATUS_RADIO_RX_FLASH_MS    500  // BLE受信表示時間ms
#define STATUS_RADIO_RSSI_STRONG_DBM (-80)  // 強受信判定RSSI dBm

// ---------------- KISS constants ----------------

#define KISS_FEND   0xC0  // KISSフレーム区切り
#define KISS_FESC   0xDB  // KISSエスケープ開始
#define KISS_TFEND  0xDC  // KISS内のFEND置換値
#define KISS_TFESC  0xDD  // KISS内のFESC置換値
#define KISS_CMD_DATA 0x00  // KISSデータフレームコマンド

// ---------------- BLE-APRS state ----------------

static uint8_t g_current_adv[BLE_APRS_MAX_TOTAL];
static size_t  g_current_adv_len = 0;
static bool    g_adv_started = false;
// Extended advertising operations are asynchronous in Bluedroid.  Keep a tiny
// state machine so GPS/KISS updates cannot configure/start/stop the same set
// while the previous GAP operation is still in flight.
static bool    g_adv_params_ready = false;
static bool    g_adv_data_busy = false;
static bool    g_adv_start_busy = false;
static bool    g_adv_stop_busy = false;
static bool    g_adv_dirty = false;
static uint32_t g_last_kiss_input_ms = 0;
static uint32_t g_nonce_counter = 0;

#if ENABLE_STATUS_LEDS
static led_strip_handle_t g_led_strip = NULL;

// USB LED uses independent color-component hold counters.  Every actual host
// I/O event reloads its component to 0.5 s, so even a single typed character is
// guaranteed to be visible to the LED task.  Continued traffic simply keeps the
// component lit.  Direction + KISS state mix naturally:
// TX+KISS = magenta, RX+KISS = cyan.
static volatile uint16_t g_led_usb_tx_hold_ticks = 0;
static volatile uint16_t g_led_usb_rx_hold_ticks = 0;
static volatile uint16_t g_led_usb_kiss_hold_ticks = 0;

// GPS LED color uses satellites that are actually being received according to
// a complete GPGSV cycle (non-empty SNR field with SNR > 0), not the number
// selected by the navigation solution in GSA/GGA.  This makes the LED useful
// as a rough RF-visibility indicator even before a position fix is available.
static volatile uint8_t g_led_gps_satellites = 0;
static volatile uint8_t g_led_gps_fix_type = 1; // 1=no fix, 2=2D, 3=3D

// RADIO LED keeps TX and RX as independent color components.  Advertising
// supplies red; a valid BLE-APRS reception supplies blue (strong) or green
// (weak), so simultaneous TX/RX mixes naturally (magenta or yellow).
static volatile bool g_led_radio_tx_active = false;
static volatile uint32_t g_led_radio_rx_until_ms = 0;
static volatile bool g_led_radio_rx_strong = false;
#endif

// Simple receiver-side cache for one source at first.
// v0 simplification: station id is hash of AX.25 source address bytes.
typedef struct {
    bool used;
    uint32_t station_hash;
    uint32_t last_seen_ms;
    uint32_t last_relay_ms;
    char last_nonce[12];
    uint32_t last_payload_hash;
    bool tx_pending;  // BLE GAP callbackからKISS TX taskへ送信待ち中

    uint8_t tokens;
    uint32_t last_token_ms;
    uint32_t ban_until_ms;
} station_cache_t;

#define STATION_CACHE_SIZE 32  // I-Gateで保持する局数
static station_cache_t g_cache[STATION_CACHE_SIZE];

typedef struct {
    uint32_t station_hash;
    uint32_t payload_hash;
    uint16_t interval;
    int rssi;
    bool rx_tag_added;
    size_t ax25_len;
    char nonce[12];
    uint8_t ax25[AX25_MAX];
} kiss_tx_item_t;

static QueueHandle_t g_kiss_tx_queue = NULL;

// ---------------- Utility ----------------

static int io_read_bytes(uint8_t *buf, size_t len, TickType_t ticks_to_wait);
static void io_write_bytes(const uint8_t *buf, size_t len);
static void advertise_ax25_frame(const uint8_t *ax25, size_t len, uint16_t relay_interval_sec, const char *origin);

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static uint32_t fnv1a32(const uint8_t *data, size_t len) {
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= data[i];
        h *= 16777619u;
    }
    return h;
}

static bool status_led_deadline_active(uint32_t deadline, uint32_t now) {
    return deadline != 0 && (int32_t)(deadline - now) > 0;
}

static void status_led_usb_tx_event(void) {
#if ENABLE_STATUS_LEDS
    g_led_usb_tx_hold_ticks = STATUS_USB_FLASH_TICKS;
#endif
}

static void status_led_usb_rx_event(void) {
#if ENABLE_STATUS_LEDS
    g_led_usb_rx_hold_ticks = STATUS_USB_FLASH_TICKS;
#endif
}

static void status_led_usb_kiss_event(void) {
#if ENABLE_STATUS_LEDS
    g_led_usb_kiss_hold_ticks = STATUS_USB_FLASH_TICKS;
#endif
}

static void status_led_gps_update(int satellites, int gsa_fix_type, bool valid_fix) {
#if ENABLE_STATUS_LEDS
    if (satellites < 0) satellites = 0;
    if (satellites > 255) satellites = 255;
    g_led_gps_satellites = (uint8_t)satellites;

    // If GSA has not supplied a fix dimension yet, a valid GGA/RMC fix is
    // conservatively displayed as 2D until GSA says it is 3D.
    if (!valid_fix || gsa_fix_type == 1) g_led_gps_fix_type = 1;
    else if (gsa_fix_type >= 3) g_led_gps_fix_type = 3;
    else g_led_gps_fix_type = 2;
#else
    (void)satellites; (void)gsa_fix_type; (void)valid_fix;
#endif
}

static void status_led_radio_tx_active(bool active) {
#if ENABLE_STATUS_LEDS
    g_led_radio_tx_active = active;
#else
    (void)active;
#endif
}

static void status_led_radio_rx_event(int rssi) {
#if ENABLE_STATUS_LEDS
    uint32_t now = now_ms();
    // Do not extend an already-active flash on every repeated advertisement.
    // With a 300 ms advertising interval, extending the deadline would make a
    // continuously received station look like a permanently lit LED.
    if (!status_led_deadline_active(g_led_radio_rx_until_ms, now)) {
        g_led_radio_rx_strong = (rssi >= STATUS_RADIO_RSSI_STRONG_DBM);
        g_led_radio_rx_until_ms = now + STATUS_RADIO_RX_FLASH_MS;
    }
#else
    (void)rssi;
#endif
}

static uint8_t status_led_scale(uint8_t v) {
    return (uint8_t)(((uint32_t)v * STATUS_LED_BRIGHTNESS) / 255U);
}

static void status_led_satellite_color(uint8_t satellites, uint8_t *r, uint8_t *g, uint8_t *b) {
    // RF-style visibility scale using GSV satellites with a usable SNR field:
    //   0     red
    //   1..2  yellow
    //   3..4  green
    //   5..6  cyan
    //   7+    blue
    if (satellites == 0) {
        *r = 255; *g =   0; *b =   0;
    } else if (satellites <= 2) {
        *r = 255; *g = 255; *b =   0;
    } else if (satellites <= 4) {
        *r =   0; *g = 255; *b =   0;
    } else if (satellites <= 6) {
        *r =   0; *g = 255; *b = 255;
    } else {
        *r =   0; *g =   0; *b = 255;
    }
}

static void status_led_task(void *arg) {
    (void)arg;
#if ENABLE_STATUS_LEDS
    uint8_t prev[STATUS_LED_COUNT][3] = {{0}};
    bool have_prev = false;

    while (1) {
        uint32_t now = now_ms();
        uint8_t rgb[STATUS_LED_COUNT][3] = {{0}};

        // USB: independent 0.5 s hold counters make very short host I/O visible.
        // Repeated traffic reloads the corresponding counter, so the color stays
        // on until 0.5 s after the final byte/block.  Components mix naturally.
        if (g_led_usb_tx_hold_ticks > 0) {
            rgb[STATUS_LED_USB][0] = 255;
        }
        if (g_led_usb_rx_hold_ticks > 0) {
            rgb[STATUS_LED_USB][1] = 255;
        }
        if (g_led_usb_kiss_hold_ticks > 0) {
            rgb[STATUS_LED_USB][2] = 255;
        }

        // GPS: color = satellites actually detected with SNR in the last complete
        // GSV cycle.  Blink duty = fix dimension.
        uint8_t gr = 0, gg = 0, gb = 0;
        status_led_satellite_color(g_led_gps_satellites, &gr, &gg, &gb);
        uint32_t phase = now % 1000U;
        bool gps_on = true;
        if (g_led_gps_fix_type == 2) gps_on = phase < 800U;      // 0.8s on / 0.2s off
        else if (g_led_gps_fix_type >= 3) gps_on = phase < 200U; // 0.2s on / 0.8s off
        if (gps_on) {
            rgb[STATUS_LED_GPS][0] = gr;
            rgb[STATUS_LED_GPS][1] = gg;
            rgb[STATUS_LED_GPS][2] = gb;
        }

        // RADIO: TX and RX are independent color components, just like the
        // USB LED.  Advertising contributes red for 0.2 s each second.
        // A valid BLE-APRS reception contributes blue at >= -80 dBm or green
        // below -80 dBm for 0.5 s.  When they overlap the LED mixes naturally:
        // TX + strong RX = magenta, TX + weak RX = yellow.
        if (g_led_radio_tx_active && phase < 200U) {
            rgb[STATUS_LED_RADIO][0] = 255;
        }
        if (status_led_deadline_active(g_led_radio_rx_until_ms, now)) {
            if (g_led_radio_rx_strong) rgb[STATUS_LED_RADIO][2] = 255;
            else                       rgb[STATUS_LED_RADIO][1] = 255;
        }

#if CONFIG_IDF_TARGET_ESP32C6
        // Mirror only valid BLE-APRS reception, not arbitrary BLE traffic.
        // This event occurs after BLE-APRS parsing and AX.25 validation, but
        // before I-Gate relay-interval suppression.
        gpio_set_level(BOARD_ONBOARD_LED_GPIO,
                       status_led_deadline_active(g_led_radio_rx_until_ms, now)
                           ? BOARD_ONBOARD_LED_ON
                           : BOARD_ONBOARD_LED_OFF);
#endif

        bool changed = !have_prev || memcmp(prev, rgb, sizeof(rgb)) != 0;
        if (changed && g_led_strip) {
            for (int i = 0; i < STATUS_LED_COUNT; i++) {
                led_strip_set_pixel(g_led_strip, i,
                                    status_led_scale(rgb[i][0]),
                                    status_led_scale(rgb[i][1]),
                                    status_led_scale(rgb[i][2]));
            }
            led_strip_refresh(g_led_strip);
            memcpy(prev, rgb, sizeof(prev));
            have_prev = true;
        }

        // Count down USB activity after rendering.  An I/O event arriving at any
        // time reloads the relevant counter to the full 0.5 s hold time.
        if (g_led_usb_tx_hold_ticks > 0) g_led_usb_tx_hold_ticks--;
        if (g_led_usb_rx_hold_ticks > 0) g_led_usb_rx_hold_ticks--;
        if (g_led_usb_kiss_hold_ticks > 0) g_led_usb_kiss_hold_ticks--;

        vTaskDelay(pdMS_TO_TICKS(STATUS_LED_TASK_INTERVAL_MS));
    }
#else
    vTaskDelete(NULL);
#endif
}

static void status_led_init(void) {
#if ENABLE_STATUS_LEDS
    led_strip_config_t strip_config = {
        .strip_gpio_num = STATUS_LED_GPIO,
        .max_leds = STATUS_LED_COUNT,
        // ESP-IDF has no PL9823 timing model.  PL9823 is close enough to the
        // WS2812 RMT timing for this hardware, but unlike WS2812 it uses RGB
        // component order.  GRB caused intended red TX indication to appear green.
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PL9823 status LED init failed: %s", esp_err_to_name(err));
        g_led_strip = NULL;
        return;
    }
    led_strip_clear(g_led_strip);
#endif
}

static void nonce_to_base26(uint32_t n, char *out, size_t out_len) {
    // A, B, ... Z, AA, AB ...
    if (out_len < 2) return;
    char tmp[12];
    size_t p = 0;
    do {
        tmp[p++] = 'A' + (n % 26);
        n /= 26;
    } while (n && p < sizeof(tmp));

    size_t w = 0;
    while (p && w + 1 < out_len) {
        out[w++] = tmp[--p];
    }
    out[w] = 0;
}

static uint16_t clamp_interval(uint16_t sec) {
    if (sec == 0) return 0;        // immediate request
    if (sec == 65535) return 65535; // local only
    if (sec < GW_MIN_RELAY_INTERVAL_SEC) return GW_MIN_RELAY_INTERVAL_SEC;
    if (sec > GW_MAX_RELAY_INTERVAL_SEC) return GW_MAX_RELAY_INTERVAL_SEC;
    return sec;
}

// AX.25 source callsign address occupies bytes 7..13 in a normal UI frame:
//   destination 7 bytes, source 7 bytes, optional digipeaters..., control, pid, info.
// For v0 cache identity, hashing source address bytes is enough.
static bool ax25_station_hash(const uint8_t *ax25, size_t len, uint32_t *out_hash) {
    if (len < 16) return false;
    *out_hash = fnv1a32(ax25 + 7, 7);
    return true;
}

static bool ax25_minimal_validate(const uint8_t *ax25, size_t len) {
    if (len < 16) return false;

    // Walk address fields in 7-byte blocks until SSID low bit is set.
    size_t pos = 0;
    int addr_count = 0;
    while (pos + 7 <= len && addr_count < 10) {
        bool last = ax25[pos + 6] & 0x01;
        pos += 7;
        addr_count++;
        if (last) break;
    }
    if (addr_count < 2) return false;
    if (pos + 2 > len) return false;

    uint8_t control = ax25[pos];
    uint8_t pid = ax25[pos + 1];
    if (control != 0x03) return false; // UI frame
    if (pid != 0xF0) return false;     // no layer 3
    return true;
}

static bool ax25_find_info_offset(const uint8_t *ax25, size_t len, size_t *info_off, int *addr_count_out) {
    if (len < 16) return false;

    size_t pos = 0;
    int addr_count = 0;
    while (pos + 7 <= len && addr_count < 10) {
        bool last = ax25[pos + 6] & 0x01;
        pos += 7;
        addr_count++;
        if (last) break;
    }

    if (addr_count < 2) return false;
    if (pos + 2 > len) return false;
    if (ax25[pos] != 0x03 || ax25[pos + 1] != 0xF0) return false;

    if (info_off) *info_off = pos + 2;
    if (addr_count_out) *addr_count_out = addr_count;
    return true;
}

static bool ax25_get_info(const uint8_t *ax25, size_t len, const uint8_t **info, size_t *info_len) {
    size_t off = 0;
    if (!ax25_find_info_offset(ax25, len, &off, NULL)) return false;
    if (off > len) return false;
    if (info) *info = ax25 + off;
    if (info_len) *info_len = len - off;
    return true;
}

static bool aprs_should_request_immediate(const uint8_t *ax25, size_t len) {
    const uint8_t *info = NULL;
    size_t info_len = 0;
    if (!ax25_get_info(ax25, len, &info, &info_len)) return false;
    if (info_len == 0) return false;

    // Conservative v0 policy:
    // APRS messages, ACKs and REJs are carried as ':' message-format packets.
    // Unknown packet types remain rate-limited.
    return info[0] == ':';
}


static void ax25_encode_addr(uint8_t *out, const char *call, uint8_t ssid, bool last) {
    for (int i = 0; i < 6; i++) {
        char c = ' ';
        if (call && call[i] != 0) c = call[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        out[i] = ((uint8_t)c) << 1;
    }
    out[6] = 0x60 | ((ssid & 0x0F) << 1) | (last ? 0x01 : 0x00);
}

static size_t ax25_build_ui_frame(uint8_t *out, size_t out_max,
                                  const char *src_call, uint8_t src_ssid,
                                  const char *dst_call, uint8_t dst_ssid,
                                  const uint8_t *info, size_t info_len) {
    const size_t need = 7 + 7 + 2 + info_len;
    if (!out || !info || need > out_max) return 0;

    ax25_encode_addr(out + 0, dst_call, dst_ssid, false);
    ax25_encode_addr(out + 7, src_call, src_ssid, true);
    out[14] = 0x03; // UI frame
    out[15] = 0xF0; // no layer 3
    memcpy(out + 16, info, info_len);
    return need;
}

static void ax25_addr_to_call(const uint8_t *addr, char *out, size_t out_len, bool mark_repeated) {
    if (out_len == 0) return;

    char call[7];
    int p = 0;
    for (int i = 0; i < 6; i++) {
        char c = (char)(addr[i] >> 1);
        if (c != ' ') {
            call[p++] = c;
        }
    }
    call[p] = 0;

    uint8_t ssid = (addr[6] >> 1) & 0x0F;
    bool repeated = (addr[6] & 0x80) != 0;

    if (ssid) {
        snprintf(out, out_len, "%s-%u%s", call, ssid, (mark_repeated && repeated) ? "*" : "");
    } else {
        snprintf(out, out_len, "%s%s", call, (mark_repeated && repeated) ? "*" : "");
    }
}

static bool ax25_to_tnc2_monitor(const uint8_t *ax25, size_t len, char *out, size_t out_len) {
    if (out_len == 0) return false;
    out[0] = 0;

    size_t info_off = 0;
    int addr_count = 0;
    if (!ax25_find_info_offset(ax25, len, &info_off, &addr_count)) return false;

    char dst[16], src[16], digi[16];
    ax25_addr_to_call(ax25 + 0, dst, sizeof(dst), false);
    ax25_addr_to_call(ax25 + 7, src, sizeof(src), false);

    int n = snprintf(out, out_len, "%s>%s", src, dst);
    if (n < 0 || (size_t)n >= out_len) return false;
    size_t used = (size_t)n;

    for (int i = 2; i < addr_count; i++) {
        ax25_addr_to_call(ax25 + (i * 7), digi, sizeof(digi), true);
        n = snprintf(out + used, out_len - used, ",%s", digi);
        if (n < 0 || (size_t)n >= out_len - used) return false;
        used += (size_t)n;
    }

    n = snprintf(out + used, out_len - used, ":");
    if (n < 0 || (size_t)n >= out_len - used) return false;
    used += (size_t)n;

    for (size_t i = info_off; i < len && used + 1 < out_len; i++) {
        uint8_t c = ax25[i];
        out[used++] = (c >= 0x20 && c <= 0x7E) ? (char)c : '.';
    }
    out[used] = 0;
    return true;
}

static void kiss_write_debug_monitor(const uint8_t *ax25, size_t len,
                                     const char *nonce, uint16_t interval, int rssi) {
#if KISS_DEBUG_MONITOR_TEXT
    char line[320];
    char mon[280];
    size_t info_off = 0;
    int addr_count = 0;

    // CRLF after the raw KISS frame. Use numeric constants to avoid escaped-newline breakage.
    const uint8_t crlf[2] = {0x0D, 0x0A};
    io_write_bytes(crlf, sizeof(crlf));

    if (ax25_find_info_offset(ax25, len, &info_off, &addr_count)) {
        uint8_t control = ax25[info_off - 2];
        uint8_t pid = ax25[info_off - 1];
        int n = snprintf(line, sizeof(line),
                         "HDR len=%u addrs=%d info=%u ctrl=0x%02X pid=0x%02X nonce=%s interval=%u rssi=%d",
                         (unsigned)len, addr_count, (unsigned)(len - info_off),
                         control, pid, nonce ? nonce : "-", interval, rssi);
        if (n > 0) {
            io_write_bytes((const uint8_t *)line, (size_t)n);
            io_write_bytes(crlf, sizeof(crlf));
        }
    } else {
        int n = snprintf(line, sizeof(line),
                         "HDR invalid len=%u nonce=%s interval=%u rssi=%d",
                         (unsigned)len, nonce ? nonce : "-", interval, rssi);
        if (n > 0) {
            io_write_bytes((const uint8_t *)line, (size_t)n);
            io_write_bytes(crlf, sizeof(crlf));
        }
    }

    if (ax25_to_tnc2_monitor(ax25, len, mon, sizeof(mon))) {
        int n = snprintf(line, sizeof(line), "MON %s", mon);
        if (n > 0) {
            io_write_bytes((const uint8_t *)line, (size_t)n);
            io_write_bytes(crlf, sizeof(crlf));
        }
    }
#else
    (void)ax25; (void)len; (void)nonce; (void)interval; (void)rssi;
#endif
}

// ---------------- KISS encode/decode ----------------

static int io_read_bytes(uint8_t *buf, size_t len, TickType_t ticks_to_wait) {
    int n;
#if USE_USB_SERIAL_JTAG
    n = usb_serial_jtag_read_bytes(buf, len, ticks_to_wait);
#else
    n = uart_read_bytes(UART_PORT, buf, len, ticks_to_wait);
#endif
    if (n > 0) status_led_usb_rx_event();
    return n;
}

static void io_write_bytes(const uint8_t *buf, size_t len) {
    if (len > 0) status_led_usb_tx_event();
#if USE_USB_SERIAL_JTAG
    usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(100));
#else
    uart_write_bytes(UART_PORT, (const char *)buf, len);
#endif
}

// KISS forwarding uses this checked path instead of the ordinary debug/echo
// writer above.  The whole encoded KISS frame is queued to the driver in one
// call, then we wait until the peripheral has actually drained it to the host.
static bool io_write_bytes_checked(const uint8_t *buf, size_t len, TickType_t timeout_ticks) {
    if (!buf || len == 0) return false;

    status_led_usb_tx_event();

#if USE_USB_SERIAL_JTAG
    int written = usb_serial_jtag_write_bytes(buf, len, timeout_ticks);
    if (written != (int)len) {
        ESP_LOGW(TAG, "USB Serial/JTAG short write: %d/%u", written, (unsigned)len);
        return false;
    }

    esp_err_t err = usb_serial_jtag_wait_tx_done(timeout_ticks);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB Serial/JTAG TX wait failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
#else
    int written = uart_write_bytes(UART_PORT, (const char *)buf, len);
    if (written != (int)len) {
        ESP_LOGW(TAG, "UART short write: %d/%u", written, (unsigned)len);
        return false;
    }

    esp_err_t err = uart_wait_tx_done(UART_PORT, timeout_ticks);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "UART TX wait failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
#endif
}

#define KISS_ENCODED_MAX (AX25_MAX * 2 + 3)

static bool kiss_write_frame(const uint8_t *ax25, size_t len) {
    if (!ax25 || len == 0 || len > AX25_MAX) return false;

    // Worst case every AX.25 byte needs KISS escaping:
    // FEND + command + (2 * AX25_MAX) + FEND.
    uint8_t encoded[KISS_ENCODED_MAX];
    size_t used = 0;

    encoded[used++] = KISS_FEND;
    encoded[used++] = KISS_CMD_DATA;

    for (size_t i = 0; i < len; i++) {
        if (ax25[i] == KISS_FEND) {
            if (used + 2 > sizeof(encoded)) return false;
            encoded[used++] = KISS_FESC;
            encoded[used++] = KISS_TFEND;
        } else if (ax25[i] == KISS_FESC) {
            if (used + 2 > sizeof(encoded)) return false;
            encoded[used++] = KISS_FESC;
            encoded[used++] = KISS_TFESC;
        } else {
            if (used + 1 > sizeof(encoded)) return false;
            encoded[used++] = ax25[i];
        }
    }

    if (used + 1 > sizeof(encoded)) return false;
    encoded[used++] = KISS_FEND;

    bool ok = io_write_bytes_checked(encoded, used,
                                     pdMS_TO_TICKS(GW_KISS_TX_TIMEOUT_MS));
    if (ok) status_led_usb_kiss_event();
    return ok;
}

static bool build_ble_aprs_payload(const uint8_t *ax25, size_t ax25_len,
                                   uint16_t relay_interval_sec,
                                   uint8_t *out, size_t *out_len) {
    char nonce[12];
    nonce_to_base26(g_nonce_counter++, nonce, sizeof(nonce));

    char header[48];
    int hlen = snprintf(header, sizeof(header), "$APRS,1,%u,%s>", relay_interval_sec, nonce);
    if (hlen <= 0 || hlen >= (int)sizeof(header)) return false;
    if ((size_t)hlen + ax25_len > BLE_APRS_MAX_TOTAL) {
        ESP_LOGW(TAG, "AX.25 too large for BLE-APRS: header=%d ax25=%u total=%u",
                 hlen, (unsigned)ax25_len, (unsigned)(hlen + ax25_len));
        return false;
    }

    memcpy(out, header, hlen);
    memcpy(out + hlen, ax25, ax25_len);
    *out_len = (size_t)hlen + ax25_len;
    return true;
}

// ---------------- BLE advertising ----------------

static esp_ble_gap_ext_adv_params_t ext_adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
    .interval_min = BLE_MS_TO_0_625MS_UNITS(ADV_INTERVAL_MS),
    .interval_max = BLE_MS_TO_0_625MS_UNITS(ADV_INTERVAL_MS),
    .channel_map = ADV_CHNL_ALL,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
    .primary_phy = ESP_BLE_GAP_PHY_CODED,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_CODED,
    .sid = 0,
    .scan_req_notif = false,
#if CONFIG_BT_BLE_FEAT_ADV_CODING_SELECTION
    .primary_adv_phy_options = ESP_BLE_ADV_PHY_OPTIONS_PREF_S8_CODING,
    .secondary_adv_phy_options = ESP_BLE_ADV_PHY_OPTIONS_PREF_S8_CODING,
#endif
};

static esp_ble_gap_ext_adv_t ext_adv_enable = {
    .instance = EXT_ADV_INSTANCE,
    .duration = 0,     // 0 = no advertising duration limit
    .max_events = 0,   // 0 = no max event limit
};

static void ble_adv_kick(void) {
    // GAP extended-advertising calls complete asynchronously.  Never issue the
    // next operation while config/start/stop is still pending.
    if (!g_adv_params_ready || g_adv_data_busy || g_adv_start_busy || g_adv_stop_busy) {
        return;
    }

    // A cleared payload means advertising is intentionally disabled.
    if (g_current_adv_len == 0) {
        if (g_adv_started) {
            uint8_t inst = EXT_ADV_INSTANCE;
            g_adv_stop_busy = true;
            esp_err_t err = esp_ble_gap_ext_adv_stop(1, &inst);
            if (err != ESP_OK) {
                g_adv_stop_busy = false;
                ESP_LOGE(TAG, "Ext adv stop request failed: %s", esp_err_to_name(err));
            }
        }
        return;
    }

    // If advertising is already running and there is no newer payload, there is
    // nothing to do.  Only stop the set when data actually needs replacing.
    // Without this guard START_COMPLETE -> kick() -> stop -> STOP_COMPLETE ->
    // kick() -> start forms an endless stop/start loop.
    if (g_adv_started) {
        if (!g_adv_dirty) {
            return;
        }

        uint8_t inst = EXT_ADV_INSTANCE;
        g_adv_stop_busy = true;
        esp_err_t err = esp_ble_gap_ext_adv_stop(1, &inst);
        if (err != ESP_OK) {
            g_adv_stop_busy = false;
            ESP_LOGE(TAG, "Ext adv stop-for-update failed: %s", esp_err_to_name(err));
        }
        return;
    }

    if (g_adv_dirty) {
        // Clear dirty before issuing the call.  If GPS/KISS writes a newer
        // payload while this operation is in flight, ble_adv_update_payload()
        // sets it again and DATA_SET_COMPLETE will configure the newest copy.
        g_adv_dirty = false;
        g_adv_data_busy = true;
        esp_err_t err = esp_ble_gap_config_ext_adv_data_raw(
            EXT_ADV_INSTANCE, (uint16_t)g_current_adv_len, g_current_adv);
        if (err != ESP_OK) {
            g_adv_data_busy = false;
            g_adv_dirty = true;
            ESP_LOGE(TAG, "Ext adv data config request failed: %s", esp_err_to_name(err));
        }
        return;
    }

    // Data is configured and no newer payload is waiting: start the set.
    g_adv_start_busy = true;
    esp_err_t err = esp_ble_gap_ext_adv_start(1, &ext_adv_enable);
    if (err != ESP_OK) {
        g_adv_start_busy = false;
        ESP_LOGE(TAG, "Ext adv start request failed: %s", esp_err_to_name(err));
    }
}

static void ble_adv_update_payload(const uint8_t *data, size_t len) {
    if (len == 0 || len > BLE_APRS_MAX_TOTAL) {
        ESP_LOGW(TAG, "Invalid ext adv payload length: %u", (unsigned)len);
        return;
    }

    memcpy(g_current_adv, data, len);
    g_current_adv_len = len;
    g_adv_dirty = true;

    if (!g_adv_params_ready) {
        ESP_LOGI(TAG, "Ext adv payload queued until params are ready: %u bytes",
                 (unsigned)g_current_adv_len);
    }
    ble_adv_kick();
}

static void ble_adv_stop(void) {
    g_current_adv_len = 0;
    g_adv_dirty = false;
    ble_adv_kick();
}

// ---------------- BLE-APRS parser / gateway rules ----------------

typedef struct {
    uint16_t relay_interval_sec;
    char nonce[12];
    const uint8_t *ax25;
    size_t ax25_len;
} ble_aprs_packet_t;

static bool parse_ble_aprs(const uint8_t *data, size_t len, ble_aprs_packet_t *out) {
    const char prefix[] = "$APRS,";
    if (len < sizeof(prefix)) return false;
    if (memcmp(data, prefix, strlen(prefix)) != 0) return false;

    const uint8_t *gt = memchr(data, '>', len);
    if (!gt) return false;
    size_t header_len = (size_t)(gt - data);
    if (header_len >= 48) return false;

    char header[48];
    memcpy(header, data, header_len);
    header[header_len] = 0;

    // Header format: $APRS,1,60,A
    char *save = NULL;
    char *tok0 = strtok_r(header, ",", &save);
    char *tok_ver = strtok_r(NULL, ",", &save);
    char *tok_interval = strtok_r(NULL, ",", &save);
    char *tok_nonce = strtok_r(NULL, ",", &save);
    if (!tok0 || !tok_ver || !tok_interval || !tok_nonce) return false;
    if (strcmp(tok0, "$APRS") != 0) return false;
    if (strcmp(tok_ver, "1") != 0) return false;

    long interval = strtol(tok_interval, NULL, 10);
    if (interval < 0 || interval > 65535) return false;
    if (strlen(tok_nonce) == 0 || strlen(tok_nonce) >= sizeof(out->nonce)) return false;

    out->relay_interval_sec = (uint16_t)interval;
    strncpy(out->nonce, tok_nonce, sizeof(out->nonce));
    out->nonce[sizeof(out->nonce) - 1] = 0;
    out->ax25 = gt + 1;
    out->ax25_len = len - ((size_t)(gt - data) + 1);
    return out->ax25_len > 0;
}

static station_cache_t *station_get(uint32_t station_hash) {
    uint32_t t = now_ms();

    for (int i = 0; i < STATION_CACHE_SIZE; i++) {
        if (g_cache[i].used && (t - g_cache[i].last_seen_ms) > STATION_CACHE_TTL_SEC * 1000UL) {
            memset(&g_cache[i], 0, sizeof(g_cache[i]));
        }
    }

    for (int i = 0; i < STATION_CACHE_SIZE; i++) {
        if (g_cache[i].used && g_cache[i].station_hash == station_hash) {
            return &g_cache[i];
        }
    }

    for (int i = 0; i < STATION_CACHE_SIZE; i++) {
        if (!g_cache[i].used) {
            memset(&g_cache[i], 0, sizeof(g_cache[i]));
            g_cache[i].used = true;
            g_cache[i].station_hash = station_hash;
            g_cache[i].tokens = GW_TOKEN_MAX;
            g_cache[i].last_token_ms = t;
            return &g_cache[i];
        }
    }

    // Very small v0 policy: overwrite slot 0 if full.
    memset(&g_cache[0], 0, sizeof(g_cache[0]));
    g_cache[0].used = true;
    g_cache[0].station_hash = station_hash;
    g_cache[0].tokens = GW_TOKEN_MAX;
    g_cache[0].last_token_ms = t;
    return &g_cache[0];
}

static void station_refill_tokens(station_cache_t *s) {
    uint32_t t = now_ms();

    if (s->last_token_ms == 0) {
        s->last_token_ms = t;
        s->tokens = GW_TOKEN_MAX;
        return;
    }

    uint32_t elapsed = t - s->last_token_ms;
    uint32_t add = elapsed / GW_TOKEN_REFILL_MS;
    if (add == 0) return;

    uint32_t nt = (uint32_t)s->tokens + add;
    s->tokens = (nt > GW_TOKEN_MAX) ? GW_TOKEN_MAX : (uint8_t)nt;
    s->last_token_ms += add * GW_TOKEN_REFILL_MS;
}

static bool station_token_available_or_ban(station_cache_t *s) {
    uint32_t t = now_ms();

    if (s->ban_until_ms != 0 && t < s->ban_until_ms) {
        return false;
    }
    if (s->ban_until_ms != 0 && t >= s->ban_until_ms) {
        s->ban_until_ms = 0;
        s->tokens = GW_TOKEN_MAX;
        s->last_token_ms = t;
    }

    station_refill_tokens(s);

    if (s->tokens == 0) {
        s->ban_until_ms = t + GW_BAN_TIME_MS;
        ESP_LOGW(TAG, "Station banned for burst: station_hash=0x%08X", (unsigned)s->station_hash);
        return false;
    }

    return true;
}

static void station_consume_token(station_cache_t *s) {
    if (s->tokens > 0) s->tokens--;
}

static station_cache_t *station_find(uint32_t station_hash) {
    for (int i = 0; i < STATION_CACHE_SIZE; i++) {
        if (g_cache[i].used && g_cache[i].station_hash == station_hash) {
            return &g_cache[i];
        }
    }
    return NULL;
}

static void kiss_tx_task(void *arg) {
    (void)arg;
    kiss_tx_item_t item;

    while (1) {
        if (xQueueReceive(g_kiss_tx_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        station_cache_t *s = station_find(item.station_hash);
        if (!s) {
            ESP_LOGW(TAG, "KISS TX station cache entry disappeared: 0x%08X",
                     (unsigned)item.station_hash);
            continue;
        }

        // Token accounting happens in the dedicated TX task.  A USB failure must
        // not burn a token or advance last_relay_ms, otherwise a failed host write
        // would falsely suppress the next valid BLE packet for the relay interval.
        if (!station_token_available_or_ban(s)) {
            s->tx_pending = false;
            continue;
        }

        bool sent = kiss_write_frame(item.ax25, item.ax25_len);
        if (sent) {
            // Optional monitor text remains after the binary KISS frame and is
            // normally disabled in production (KISS_DEBUG_MONITOR_TEXT == 0).
            kiss_write_debug_monitor(item.ax25, item.ax25_len,
                                     item.nonce, item.interval, item.rssi);

            s->last_relay_ms = now_ms();
            s->last_payload_hash = item.payload_hash;
            strncpy(s->last_nonce, item.nonce, sizeof(s->last_nonce));
            s->last_nonce[sizeof(s->last_nonce) - 1] = 0;
            station_consume_token(s);

            ESP_LOGI(TAG,
                     "Relayed BLE-APRS to KISS: len=%u%s nonce=%s interval=%u rssi=%d tokens=%u",
                     (unsigned)item.ax25_len,
                     item.rx_tag_added ? " +RX" : "",
                     item.nonce, item.interval, item.rssi, s->tokens);
        } else {
            // Leave last_relay_ms and last_nonce unchanged so a later BLE
            // advertisement can retry instead of being suppressed as "already
            // relayed".  This is especially important when the phone/USB host is
            // still enumerating or temporarily stops draining the CDC endpoint.
            ESP_LOGW(TAG,
                     "KISS TX failed; relay state not advanced: nonce=%s interval=%u rssi=%d",
                     item.nonce, item.interval, item.rssi);
        }

        s->tx_pending = false;
    }
}

static void process_ble_aprs_report(const uint8_t *data, size_t len, int rssi) {
#if !ENABLE_BLE_TO_KISS
    (void)data; (void)len; (void)rssi;
    return;
#endif

    ble_aprs_packet_t p;
    if (!parse_ble_aprs(data, len, &p)) return;
    if (!ax25_minimal_validate(p.ax25, p.ax25_len)) {
        ESP_LOGW(TAG, "Drop invalid AX.25 len=%u", (unsigned)p.ax25_len);
        return;
    }

    // Show actual RF reception even when gateway rate limiting later suppresses
    // forwarding.  Random non-BLE-APRS advertisements do not trigger this LED.
    status_led_radio_rx_event(rssi);

    uint32_t station_hash;
    if (!ax25_station_hash(p.ax25, p.ax25_len, &station_hash)) return;

    station_cache_t *s = station_get(station_hash);
    uint32_t t = now_ms();
    s->last_seen_ms = t;

    if (p.relay_interval_sec == 65535) {
        ESP_LOGI(TAG, "Local-only packet nonce=%s rssi=%d", p.nonce, rssi);
        return;
    }

    uint16_t interval = clamp_interval(p.relay_interval_sec);
    uint32_t payload_hash = fnv1a32(p.ax25, p.ax25_len);

    if (s->ban_until_ms != 0 && t < s->ban_until_ms) {
        ESP_LOGW(TAG, "Drop banned station: station_hash=0x%08X", (unsigned)s->station_hash);
        return;
    }

    if (strcmp(s->last_nonce, p.nonce) == 0) return;

    // Do NOT suppress identical AX.25 payloads when nonce changed.
    // A stationary APRS station should still be able to send periodic beacons.
    // payload_hash is kept only for diagnostics/state, not as a drop condition.
    //
    // relay_interval is evaluated from EACH newly received packet. This is
    // intentional: an Adaptive Relay Interval tracker may lower its requested interval as it
    // accelerates or turns. If the I-Gate has already been waiting longer than
    // that newly smaller interval, the newest nonce/payload is relayed at once.
    if (interval != 0 && s->last_relay_ms != 0 && (t - s->last_relay_ms) < interval * 1000UL) return;

    // Only one host-write job per station may be pending at a time. This keeps
    // repeated ~300 ms advertisements of the same BLE packet from filling the
    // queue while the dedicated USB task is still transmitting.
    if (s->tx_pending) return;

    kiss_tx_item_t item = {
        .station_hash = station_hash,
        .payload_hash = payload_hash,
        .interval = interval,
        .rssi = rssi,
        .rx_tag_added = false,
        .ax25_len = p.ax25_len,
    };
    strncpy(item.nonce, p.nonce, sizeof(item.nonce));
    item.nonce[sizeof(item.nonce) - 1] = 0;

    if (item.ax25_len > sizeof(item.ax25)) return;
    memcpy(item.ax25, p.ax25, item.ax25_len);

    // The gateway can see RF RSSI directly, unlike an ordinary AFSK TNC path.
    // Add that receiver-local metadata to the queued AX.25 copy when there is room.
#if GW_APPEND_RX_LEVEL
    char rx_tag[20];
    int rx_tag_len = snprintf(rx_tag, sizeof(rx_tag), " [RX:%ddB]", rssi);

    if (rx_tag_len > 0 && (size_t)rx_tag_len < sizeof(rx_tag) &&
        item.ax25_len + (size_t)rx_tag_len <= sizeof(item.ax25) &&
        len + (size_t)rx_tag_len <= BLE_APRS_MAX_TOTAL) {
        memcpy(item.ax25 + item.ax25_len, rx_tag, (size_t)rx_tag_len);
        item.ax25_len += (size_t)rx_tag_len;
        item.rx_tag_added = true;
    } else {
        ESP_LOGD(TAG, "RX level tag omitted: ax25=%u ble=%u rssi=%d",
                 (unsigned)p.ax25_len, (unsigned)len, rssi);
    }
#endif

    // GAP/Bluedroid callback must return quickly. Never wait for USB here.
    // xQueueSend() copies the item, so the BLE report buffer may be released as
    // soon as this function returns.
    // Set pending before xQueueSend(): the higher-priority TX task may wake and
    // preempt this callback immediately when the item is queued.  Setting the
    // flag first avoids a race where the task clears it and this callback then
    // accidentally sets it back to true forever.
    s->tx_pending = true;
    if (!g_kiss_tx_queue || xQueueSend(g_kiss_tx_queue, &item, 0) != pdTRUE) {
        s->tx_pending = false;
        ESP_LOGW(TAG, "KISS TX queue full; will retry from a later advertisement");
        return;
    }
}

// ---------------- GAP callback ----------------

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        ESP_LOGI(TAG, "Ext adv params set");
        g_adv_params_ready = true;
        ble_adv_kick();
        break;

    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        g_adv_data_busy = false;
        ESP_LOGI(TAG, "Ext adv data set: %u bytes%s",
                 (unsigned)g_current_adv_len, g_adv_dirty ? " (newer payload pending)" : "");
        ble_adv_kick();
        break;

    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        g_adv_start_busy = false;
        g_adv_started = true;
        status_led_radio_tx_active(true);
        ESP_LOGI(TAG, "Ext adv started");
        // If a newer GPS/KISS payload arrived while start was pending, kick()
        // will stop this set and replace the data in the proper order.
        ble_adv_kick();
        break;

    case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
        g_adv_stop_busy = false;
        g_adv_started = false;
        status_led_radio_tx_active(false);
        ESP_LOGI(TAG, "Ext adv stopped");
        ble_adv_kick();
        break;

    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
        ESP_LOGI(TAG, "Ext scan params set; starting continuous scan");
        esp_ble_gap_start_ext_scan(0, 0);
        break;

    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "Ext scan started");
        break;

    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
        // ESP-IDF Bluedroid extended report field names have changed across versions.
        // Common recent layout:
        //   param->ext_adv_report.params.adv_data_len
        //   param->ext_adv_report.params.adv_data
        //   param->ext_adv_report.params.rssi
        // If your IDF version differs, inspect esp_gap_ble_api.h union esp_ble_gap_cb_param_t
        // and adjust only the following three lines.
        uint8_t *adv_data = param->ext_adv_report.params.adv_data;
        uint8_t adv_len = param->ext_adv_report.params.adv_data_len;
        int rssi = param->ext_adv_report.params.rssi;
        process_ble_aprs_report(adv_data, adv_len, rssi);
        break;
    }

    default:
        break;
    }
}

static void ble_init(void) {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));

    ESP_ERROR_CHECK(esp_ble_gap_ext_adv_set_params(EXT_ADV_INSTANCE, &ext_adv_params));

#if ENABLE_BLE_TO_KISS
    esp_ble_ext_scan_params_t scan_params = {
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
        .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
        .uncoded_cfg = {0},
        .coded_cfg = {
            .scan_type = BLE_SCAN_TYPE_PASSIVE,
            .scan_interval = BLE_MS_TO_0_625MS_UNITS(100),
            .scan_window = BLE_MS_TO_0_625MS_UNITS(100),
        },
    };
    ESP_ERROR_CHECK(esp_ble_gap_set_ext_scan_params(&scan_params));
#endif
}

// ---------------- AX.25 -> BLE advertising ----------------

static bool kiss_priority_active(void) {
    return g_last_kiss_input_ms != 0 &&
           (now_ms() - g_last_kiss_input_ms) < KISS_PRIORITY_SEC * 1000UL;
}

static void advertise_ax25_frame(const uint8_t *ax25, size_t len, uint16_t relay_interval_sec, const char *origin) {
#if ENABLE_KISS_TO_BLE
    uint8_t payload[BLE_APRS_MAX_TOTAL];
    size_t payload_len = 0;

    if (!ax25_minimal_validate(ax25, len)) {
        ESP_LOGW(TAG, "%s frame is not AX.25 UI/PID F0, len=%u", origin ? origin : "input", (unsigned)len);
        return;
    }

    if (!build_ble_aprs_payload(ax25, len, relay_interval_sec, payload, &payload_len)) {
        return;
    }

    ble_adv_update_payload(payload, payload_len);

    // GPS Adaptive Relay Interval mode refreshes payload/nonce every second; keep those
    // routine refresh logs at DEBUG so the console is not flooded. KISS input
    // remains INFO-level because it is an explicit external frame event.
    if (origin && strcmp(origin, "KISS") == 0) {
        ESP_LOGI(TAG, "%s -> BLE-APRS adv update: ax25=%u total=%u interval=%u%s",
                 origin, (unsigned)len, (unsigned)payload_len, relay_interval_sec,
                 (relay_interval_sec == 0) ? " immediate" : "");
    } else {
        ESP_LOGD(TAG, "%s -> BLE-APRS adv update: ax25=%u total=%u interval=%u%s",
                 origin ? origin : "AX.25", (unsigned)len, (unsigned)payload_len, relay_interval_sec,
                 (relay_interval_sec == 0) ? " immediate" : "");
    }
#else
    (void)ax25; (void)len; (void)relay_interval_sec; (void)origin;
#endif
}


// ---------------- BME280 temperature / humidity / pressure ----------------

#if ENABLE_BME280

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_t;

typedef struct {
    bool valid;
    float temperature_c;
    float humidity_pct;
    float pressure_hpa;
    uint32_t last_read_ms;
    uint32_t last_status_log_ms;
} bme280_state_t;

static bme280_calib_t g_bme280_cal;
static bme280_state_t g_bme280;
static int32_t g_bme280_t_fine;

static uint16_t bme280_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t bme280_s16_le(const uint8_t *p) {
    return (int16_t)bme280_u16_le(p);
}

static int16_t bme280_sign_extend_12(uint16_t v) {
    v &= 0x0FFF;
    if (v & 0x0800) v |= 0xF000;
    return (int16_t)v;
}

static esp_err_t bme280_write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg, value};
    return i2c_master_write_to_device(BME280_I2C_PORT, BME280_I2C_ADDR,
                                      tx, sizeof(tx), pdMS_TO_TICKS(100));
}

static esp_err_t bme280_read_regs(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(BME280_I2C_PORT, BME280_I2C_ADDR,
                                        &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t bme280_wait_ready(uint32_t timeout_ms) {
    uint32_t start = now_ms();
    while ((now_ms() - start) < timeout_ms) {
        uint8_t status = 0;
        esp_err_t err = bme280_read_regs(0xF3, &status, 1);
        if (err != ESP_OK) return err;
        // bit3 = measuring, bit0 = NVM image update
        if ((status & 0x09) == 0) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_ERR_TIMEOUT;
}

static bool bme280_read_calibration(void) {
    uint8_t c1[26]; // 0x88..0xA1
    uint8_t c2[7];  // 0xE1..0xE7

    if (bme280_read_regs(0x88, c1, sizeof(c1)) != ESP_OK) return false;
    if (bme280_read_regs(0xE1, c2, sizeof(c2)) != ESP_OK) return false;

    g_bme280_cal.dig_T1 = bme280_u16_le(c1 + 0);
    g_bme280_cal.dig_T2 = bme280_s16_le(c1 + 2);
    g_bme280_cal.dig_T3 = bme280_s16_le(c1 + 4);
    g_bme280_cal.dig_P1 = bme280_u16_le(c1 + 6);
    g_bme280_cal.dig_P2 = bme280_s16_le(c1 + 8);
    g_bme280_cal.dig_P3 = bme280_s16_le(c1 + 10);
    g_bme280_cal.dig_P4 = bme280_s16_le(c1 + 12);
    g_bme280_cal.dig_P5 = bme280_s16_le(c1 + 14);
    g_bme280_cal.dig_P6 = bme280_s16_le(c1 + 16);
    g_bme280_cal.dig_P7 = bme280_s16_le(c1 + 18);
    g_bme280_cal.dig_P8 = bme280_s16_le(c1 + 20);
    g_bme280_cal.dig_P9 = bme280_s16_le(c1 + 22);
    g_bme280_cal.dig_H1 = c1[25];
    g_bme280_cal.dig_H2 = bme280_s16_le(c2 + 0);
    g_bme280_cal.dig_H3 = c2[2];
    g_bme280_cal.dig_H4 = bme280_sign_extend_12(((uint16_t)c2[3] << 4) | (c2[4] & 0x0F));
    g_bme280_cal.dig_H5 = bme280_sign_extend_12(((uint16_t)c2[5] << 4) | (c2[4] >> 4));
    g_bme280_cal.dig_H6 = (int8_t)c2[6];

    return g_bme280_cal.dig_T1 != 0 && g_bme280_cal.dig_P1 != 0;
}

static float bme280_compensate_temperature(int32_t adc_t) {
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)g_bme280_cal.dig_T1 << 1))) *
                    ((int32_t)g_bme280_cal.dig_T2)) >> 11;
    int32_t var2 = (((((adc_t >> 4) - ((int32_t)g_bme280_cal.dig_T1)) *
                       ((adc_t >> 4) - ((int32_t)g_bme280_cal.dig_T1))) >> 12) *
                    ((int32_t)g_bme280_cal.dig_T3)) >> 14;

    g_bme280_t_fine = var1 + var2;
    int32_t t100 = (g_bme280_t_fine * 5 + 128) >> 8;
    return (float)t100 / 100.0f;
}

static float bme280_compensate_pressure(int32_t adc_p) {
    int64_t var1 = (int64_t)g_bme280_t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)g_bme280_cal.dig_P6;
    var2 += (var1 * (int64_t)g_bme280_cal.dig_P5) << 17;
    var2 += ((int64_t)g_bme280_cal.dig_P4) << 35;
    var1 = ((var1 * var1 * (int64_t)g_bme280_cal.dig_P3) >> 8) +
           ((var1 * (int64_t)g_bme280_cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)g_bme280_cal.dig_P1) >> 33;

    if (var1 == 0) return 0.0f;

    int64_t p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)g_bme280_cal.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)g_bme280_cal.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)g_bme280_cal.dig_P7) << 4);

    // Bosch integer algorithm returns pressure in Pa * 256.
    float pa = (float)p / 256.0f;
    return pa / 100.0f;
}

static float bme280_compensate_humidity(int32_t adc_h) {
    int32_t v = g_bme280_t_fine - 76800;
    v = (((((adc_h << 14) - (((int32_t)g_bme280_cal.dig_H4) << 20) -
             (((int32_t)g_bme280_cal.dig_H5) * v)) + 16384) >> 15) *
          (((((((v * (int32_t)g_bme280_cal.dig_H6) >> 10) *
                (((v * (int32_t)g_bme280_cal.dig_H3) >> 11) + 32768)) >> 10) +
             2097152) * (int32_t)g_bme280_cal.dig_H2 + 8192) >> 14));
    v -= (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)g_bme280_cal.dig_H1) >> 4);
    if (v < 0) v = 0;
    if (v > 419430400) v = 419430400;

    // Q22.10 %RH.
    return (float)(v >> 12) / 1024.0f;
}

static esp_err_t bme280_read_sample(float *temperature_c, float *humidity_pct, float *pressure_hpa) {
    // Forced mode saves power between the 2-second samples.
    // osrs_t=x2 (010), osrs_p=x4 (011), mode=forced (01).
    esp_err_t err = bme280_write_reg(0xF4, 0x4D);
    if (err != ESP_OK) return err;

    // Give forced mode time to enter the measuring state before polling status.
    vTaskDelay(pdMS_TO_TICKS(10));
    err = bme280_wait_ready(100);
    if (err != ESP_OK) return err;

    uint8_t d[8];
    err = bme280_read_regs(0xF7, d, sizeof(d));
    if (err != ESP_OK) return err;

    int32_t adc_p = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adc_t = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    int32_t adc_h = ((int32_t)d[6] << 8) | d[7];

    if (adc_t == 0x80000 || adc_p == 0x80000) return ESP_ERR_INVALID_RESPONSE;

    float t = bme280_compensate_temperature(adc_t);
    float p = bme280_compensate_pressure(adc_p);
    float h = bme280_compensate_humidity(adc_h);

    if (temperature_c) *temperature_c = t;
    if (pressure_hpa) *pressure_hpa = p;
    if (humidity_pct) *humidity_pct = h;
    return ESP_OK;
}

static bool bme280_init(void) {
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BME280_SDA_PIN,
        .scl_io_num = BME280_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BME280_I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(BME280_I2C_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 I2C config failed: %s", esp_err_to_name(err));
        return false;
    }
    err = i2c_driver_install(BME280_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 I2C driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t id = 0;
    err = bme280_read_regs(0xD0, &id, 1);
    if (err != ESP_OK || id != 0x60) {
        ESP_LOGE(TAG, "BME280 not found at 0x%02X: err=%s chip_id=0x%02X",
                 BME280_I2C_ADDR, esp_err_to_name(err), id);
        return false;
    }

    // Soft reset, then wait until calibration/NVM copy is complete.
    if (bme280_write_reg(0xE0, 0xB6) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(5));
    if (bme280_wait_ready(100) != ESP_OK) return false;

    if (!bme280_read_calibration()) {
        ESP_LOGE(TAG, "BME280 calibration read failed");
        return false;
    }

    // Humidity oversampling x1. This register must be written before ctrl_meas.
    if (bme280_write_reg(0xF2, 0x01) != ESP_OK) return false;
    // IIR filter x4. Device remains in sleep until each forced measurement.
    if (bme280_write_reg(0xF5, 0x08) != ESP_OK) return false;
    if (bme280_write_reg(0xF4, 0x00) != ESP_OK) return false;

    ESP_LOGI(TAG, "BME280 ready: I2C 0x%02X SDA=GPIO%d SCL=GPIO%d",
             BME280_I2C_ADDR, BME280_SDA_PIN, BME280_SCL_PIN);
    return true;
}

static void bme280_task(void *arg) {
    (void)arg;
    while (1) {
        float t = 0.0f, h = 0.0f, p = 0.0f;
        esp_err_t err = bme280_read_sample(&t, &h, &p);
        if (err == ESP_OK) {
            // Store the values first and set valid last so the GPS task sees a complete sample.
            g_bme280.temperature_c = t;
            g_bme280.humidity_pct = h;
            g_bme280.pressure_hpa = p;
            g_bme280.last_read_ms = now_ms();
            g_bme280.valid = true;

#if BME280_STATUS_LOG_MS
            uint32_t now = now_ms();
            if (g_bme280.last_status_log_ms == 0 ||
                (now - g_bme280.last_status_log_ms) >= BME280_STATUS_LOG_MS) {
                g_bme280.last_status_log_ms = now;
                ESP_LOGI(TAG, "BME280 T=%.2fC RH=%.1f%% P=%.1fhPa",
                         g_bme280.temperature_c,
                         g_bme280.humidity_pct,
                         g_bme280.pressure_hpa);
            }
#endif
        } else {
            g_bme280.valid = false;
            ESP_LOGW(TAG, "BME280 sample failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(BME280_READ_INTERVAL_MS));
    }
}

#endif // ENABLE_BME280

// ---------------- GPS NMEA parser / APRS position/weather beacon ----------------

#if ENABLE_GPS_NMEA

typedef struct {
    // Last valid position is intentionally retained across GPS loss.
    bool have_position;
    bool valid_fix;
    char utc_time[16];
    char utc_date[12];
    char lat_aprs[9];   // ddmm.mmH, last valid position
    char lon_aprs[10];  // dddmm.mmH, last valid position
    double altitude_m;  // last valid GGA altitude
    double speed_knots;
    double course_deg;
    bool have_course_speed;
    uint64_t last_valid_fix_us;

    // Software UTC clock, continuously re-seeded from NMEA.  Once seeded it
    // keeps advancing from esp_timer even if GPS/NMEA later disappears.
    bool clock_time_valid;
    bool clock_date_valid;
    int clock_hour;
    int clock_minute;
    int clock_second;
    int clock_day;
    int clock_month;
    int clock_year;
    uint64_t clock_seed_us;

    int fix_quality;
    int gsa_fix_type;
    double pdop;
    double hdop;
    double vdop;
    int active_satellites;     // satellites used by the navigation solution (GSA/GGA)
    int detected_satellites;   // satellites with SNR > 0 in the last complete GSV cycle
    int gsv_expected_messages;
    int gsv_last_message;
    int gsv_cycle_detected;
    uint32_t sentence_updates;
    uint32_t total_lines;
    uint32_t parsed_sentences;
    uint32_t checksum_errors;
    uint32_t unsupported_sentences;
    uint32_t last_rx_ms;
    uint32_t last_status_log_ms;
    bool tx_ready_logged;
} gps_state_t;

static gps_state_t g_gps;

static void gps_debug_printf(const char *fmt, ...) {
#if GPS_DEBUG_LOG_EVERY
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    size_t len = (n < (int)sizeof(line)) ? (size_t)n : sizeof(line) - 1;
    io_write_bytes((const uint8_t *)line, len);
    const uint8_t crlf[2] = {0x0D, 0x0A};
    io_write_bytes(crlf, sizeof(crlf));
#else
    (void)fmt;
#endif
}

static bool nmea_checksum_ok(const char *s) {
    if (!s || s[0] != '$') return false;
    const char *star = strchr(s, '*');
    if (!star || !star[1] || !star[2]) return false;

    uint8_t sum = 0;
    for (const char *p = s + 1; p < star; p++) sum ^= (uint8_t)*p;

    char hex[3] = {star[1], star[2], 0};
    char *end = NULL;
    long want = strtol(hex, &end, 16);
    return end == hex + 2 && want >= 0 && want <= 255 && sum == (uint8_t)want;
}

static int split_csv(char *s, char **fields, int max_fields) {
    int n = 0;
    char *p = s;
    while (n < max_fields) {
        fields[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = 0;
        p = comma + 1;
    }
    return n;
}

static bool nmea_to_aprs_lat(const char *v, const char *hemi, char *out, size_t out_len) {
    if (!v || !hemi || strlen(v) < 4 || out_len < 9) return false;
    double raw = atof(v);
    int deg = (int)(raw / 100.0);
    double min = raw - (double)deg * 100.0;
    char h = (hemi[0] == 'S') ? 'S' : 'N';
    snprintf(out, out_len, "%02d%05.2f%c", deg, min, h);
    return true;
}

static bool nmea_to_aprs_lon(const char *v, const char *hemi, char *out, size_t out_len) {
    if (!v || !hemi || strlen(v) < 5 || out_len < 10) return false;
    double raw = atof(v);
    int deg = (int)(raw / 100.0);
    double min = raw - (double)deg * 100.0;
    char h = (hemi[0] == 'W') ? 'W' : 'E';
    snprintf(out, out_len, "%03d%05.2f%c", deg, min, h);
    return true;
}

static int round_to_int(double v) {
    return (int)(v >= 0.0 ? v + 0.5 : v - 0.5);
}

static bool gps_parse_nmea_hms(const char *s, int *hh, int *mm, int *ss) {
    if (!s || strlen(s) < 6) return false;
    for (int i = 0; i < 6; i++) {
        if (s[i] < '0' || s[i] > '9') return false;
    }

    int h = (s[0] - '0') * 10 + (s[1] - '0');
    int m = (s[2] - '0') * 10 + (s[3] - '0');
    int sec = (s[4] - '0') * 10 + (s[5] - '0');
    if (h > 23 || m > 59 || sec > 59) return false;

    if (hh) *hh = h;
    if (mm) *mm = m;
    if (ss) *ss = sec;
    return true;
}

static bool gps_parse_nmea_date(const char *s, int *day, int *month, int *year) {
    // RMC date is DDMMYY.
    if (!s || strlen(s) < 6) return false;
    for (int i = 0; i < 6; i++) {
        if (s[i] < '0' || s[i] > '9') return false;
    }

    int d = (s[0] - '0') * 10 + (s[1] - '0');
    int m = (s[2] - '0') * 10 + (s[3] - '0');
    int yy = (s[4] - '0') * 10 + (s[5] - '0');
    if (d < 1 || d > 31 || m < 1 || m > 12) return false;

    int y = (yy >= 80) ? (1900 + yy) : (2000 + yy);
    if (day) *day = d;
    if (month) *month = m;
    if (year) *year = y;
    return true;
}

static bool gps_is_leap_year(int year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

static int gps_days_in_month(int year, int month) {
    static const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (month < 1 || month > 12) return 30;
    if (month == 2 && gps_is_leap_year(year)) return 29;
    return days[month - 1];
}

static void gps_add_days(int *year, int *month, int *day, uint32_t add_days) {
    int y = *year;
    int m = *month;
    int d = *day;

    while (add_days > 0) {
        int dim = gps_days_in_month(y, m);
        int remain = dim - d;
        if (add_days <= (uint32_t)remain) {
            d += (int)add_days;
            add_days = 0;
        } else {
            add_days -= (uint32_t)remain + 1U;
            d = 1;
            m++;
            if (m > 12) {
                m = 1;
                y++;
            }
        }
    }

    *year = y;
    *month = m;
    *day = d;
}

static void gps_clock_update(const char *nmea_time, const char *nmea_date) {
    int hh = 0, mm = 0, ss = 0;
    if (!gps_parse_nmea_hms(nmea_time, &hh, &mm, &ss)) return;

    // If a valid RMC date is supplied, update the date half of the clock too.
    if (nmea_date && nmea_date[0]) {
        int day = 0, month = 0, year = 0;
        if (gps_parse_nmea_date(nmea_date, &day, &month, &year)) {
            g_gps.clock_day = day;
            g_gps.clock_month = month;
            g_gps.clock_year = year;
            g_gps.clock_date_valid = true;
        }
    }

    g_gps.clock_hour = hh;
    g_gps.clock_minute = mm;
    g_gps.clock_second = ss;
    g_gps.clock_seed_us = (uint64_t)esp_timer_get_time();
    g_gps.clock_time_valid = true;
}

static bool gps_get_current_utc(int *year, int *month, int *day,
                                int *hh, int *mm, int *ss) {
    if (!g_gps.clock_time_valid) return false;

    uint64_t elapsed_sec = ((uint64_t)esp_timer_get_time() - g_gps.clock_seed_us) / 1000000ULL;
    uint64_t seed_sod = (uint64_t)g_gps.clock_hour * 3600ULL +
                        (uint64_t)g_gps.clock_minute * 60ULL +
                        (uint64_t)g_gps.clock_second;
    uint64_t total = seed_sod + elapsed_sec;
    uint32_t add_days = (uint32_t)(total / 86400ULL);
    uint32_t sod = (uint32_t)(total % 86400ULL);

    int out_h = (int)(sod / 3600U);
    int out_m = (int)((sod % 3600U) / 60U);
    int out_s = (int)(sod % 60U);

    if (hh) *hh = out_h;
    if (mm) *mm = out_m;
    if (ss) *ss = out_s;

    if (year || month || day) {
        if (!g_gps.clock_date_valid) return false;
        int y = g_gps.clock_year;
        int mo = g_gps.clock_month;
        int d = g_gps.clock_day;
        gps_add_days(&y, &mo, &d, add_days);
        if (year) *year = y;
        if (month) *month = mo;
        if (day) *day = d;
    }

    return true;
}

static bool gps_format_current_aprs_hms_timestamp(char *out, size_t out_len) {
    if (!out || out_len < 8) return false;
    int hh = 0, mm = 0, ss = 0;
    if (!gps_get_current_utc(NULL, NULL, NULL, &hh, &mm, &ss)) return false;
    snprintf(out, out_len, "%02d%02d%02dh", hh, mm, ss);
    return true;
}

static bool gps_format_current_aprs_mdhm_timestamp(char *out, size_t out_len) {
    // APRS positionless weather stations use MMDDHHMM (MDHM), with no suffix.
    if (!out || out_len < 9) return false;
    int year = 0, month = 0, day = 0, hh = 0, mm = 0, ss = 0;
    if (!gps_get_current_utc(&year, &month, &day, &hh, &mm, &ss)) return false;
    (void)year;
    (void)ss;
    snprintf(out, out_len, "%02d%02d%02d%02d", month, day, hh, mm);
    return true;
}

static bool gps_format_wx_sensor_tokens(char *out, size_t out_len) {
#if ENABLE_BME280
    if (!out || out_len == 0 || !g_bme280.valid) return false;

    int temp_f = round_to_int((double)g_bme280.temperature_c * 9.0 / 5.0 + 32.0);
    int humidity = round_to_int(g_bme280.humidity_pct);
    int pressure_01hpa = round_to_int((double)g_bme280.pressure_hpa * 10.0);

    if (temp_f < -99) temp_f = -99;
    if (temp_f > 999) temp_f = 999;
    // APRS reserves h00 to mean 100%, so clamp a theoretical 0% reading to h01.
    if (humidity < 1) humidity = 1;
    if (humidity > 100) humidity = 100;
    if (pressure_01hpa < 0) pressure_01hpa = 0;
    if (pressure_01hpa > 99999) pressure_01hpa = 99999;

    int aprs_humidity = (humidity == 100) ? 0 : humidity;
    int n = snprintf(out, out_len, "t%03dh%02db%05d",
                     temp_f, aprs_humidity, pressure_01hpa);
    return n > 0 && n < (int)out_len;
#else
    (void)out;
    (void)out_len;
    return false;
#endif
}

static bool gps_format_text_sensor_tokens(char *out, size_t out_len) {
#if ENABLE_BME280
    if (!out || out_len == 0 || !g_bme280.valid) return false;

    int humidity = round_to_int(g_bme280.humidity_pct);
    if (humidity < 0) humidity = 0;
    if (humidity > 100) humidity = 100;
    int n = snprintf(out, out_len, "%.1fC %d%% %.1fhPa",
                     (double)g_bme280.temperature_c,
                     humidity,
                     (double)g_bme280.pressure_hpa);
    return n > 0 && n < (int)out_len;
#else
    (void)out;
    (void)out_len;
    return false;
#endif
}

static bool gps_format_environment_block(char *out, size_t out_len) {
    if (!out || out_len == 0) return false;
    out[0] = 0;

#if ENABLE_BME280
    if (!g_bme280.valid) return false;

#if GPS_APRS_ENV_FORMAT == GPS_APRS_ENV_FORMAT_WX
    return gps_format_wx_sensor_tokens(out, out_len);
#elif GPS_APRS_ENV_FORMAT == GPS_APRS_ENV_FORMAT_TEXT
    return gps_format_text_sensor_tokens(out, out_len);
#else
#error "GPS_APRS_ENV_FORMAT must be GPS_APRS_ENV_FORMAT_WX or GPS_APRS_ENV_FORMAT_TEXT"
#endif
#else
    return false;
#endif
}

typedef enum {
    GPS_BEACON_REASON_TRACKING = 0,
    GPS_BEACON_REASON_INITIAL,
    GPS_BEACON_REASON_TURN,
    GPS_BEACON_REASON_KISS_RESUME,
} gps_beacon_reason_t;

typedef struct {
    bool have_payload;
    bool was_kiss_priority;

    // Local Adaptive Relay Interval reference used only for corner detection. The I-Gate
    // remains authoritative for actual relay timing; this reference approximates
    // the bearing of the most recent regular/turn beacon for corner detection.
    bool turn_reference_valid;
    uint32_t turn_reference_ms;
    double turn_reference_course_deg;

    // A newly shorter relay request is held for interval * 2/3 seconds.
    // This gives connectionless BLE advertising repeated chances to reach an
    // I-Gate, while the hold remains shorter than the requested relay interval
    // so the same short request cannot normally cause a second relay by itself.
    bool interval_hold_active;
    uint16_t held_interval_sec;
    uint32_t interval_hold_until_ms;
    uint16_t last_effective_interval_sec;
} gps_beacon_scheduler_t;

static gps_beacon_scheduler_t g_gps_beacon;

static const char *gps_beacon_reason_name(gps_beacon_reason_t reason) {
    switch (reason) {
        case GPS_BEACON_REASON_INITIAL:     return "initial";
        case GPS_BEACON_REASON_TURN:        return "turn";
        case GPS_BEACON_REASON_KISS_RESUME: return "kiss-resume";
        default:                            return "tracking";
    }
}

static uint64_t gps_fix_age_sec_now(void) {
    if (g_gps.last_valid_fix_us == 0) return UINT64_MAX;
    uint64_t now_us = (uint64_t)esp_timer_get_time();
    if (now_us < g_gps.last_valid_fix_us) return 0;
    return (now_us - g_gps.last_valid_fix_us) / 1000000ULL;
}

static bool gps_fix_is_current(void) {
    return g_gps.have_position &&
           g_gps.valid_fix &&
           gps_fix_age_sec_now() <= GPS_FIX_CURRENT_MAX_AGE_SEC;
}

static double gps_current_speed_kmh(void) {
    if (!gps_fix_is_current() || !g_gps.have_course_speed) return 0.0;
    double kmh = g_gps.speed_knots * 1.852;
    return kmh > 0.0 ? kmh : 0.0;
}

static uint16_t gps_adaptive_relay_rate_sec(double speed_kmh) {
#if GPS_ADAPTIVE_RELAY_INTERVAL
    double rate;
    if (speed_kmh <= GPS_ARI_SLOW_SPEED_KMH) {
        rate = GPS_ARI_SLOW_RATE_SEC;
    } else if (speed_kmh >= GPS_ARI_FAST_SPEED_KMH) {
        rate = GPS_ARI_FAST_RATE_SEC;
    } else {
        // Linear interpolation between the configured slow/fast endpoints.
        rate = GPS_ARI_FAST_RATE_SEC +
               (GPS_ARI_SLOW_RATE_SEC - GPS_ARI_FAST_RATE_SEC) *
               (GPS_ARI_FAST_SPEED_KMH - speed_kmh) /
               (GPS_ARI_FAST_SPEED_KMH - GPS_ARI_SLOW_SPEED_KMH);
    }
    if (rate < 1.0) rate = 1.0;
    if (rate > 65534.0) rate = 65534.0;
    return (uint16_t)rate;
#else
    (void)speed_kmh;
    return GPS_APRS_RELAY_INTERVAL_SEC;
#endif
}


static uint32_t gps_adaptive_relay_hold_ms(uint16_t interval_sec) {
#if GPS_ADAPTIVE_RELAY_INTERVAL
    // Integer arithmetic intentionally rounds down. With the default 2/3 ratio,
    // a 15 s request is therefore advertised for 10 s. Keep at least one second
    // for very small experimental interval values.
    uint32_t hold_sec = ((uint32_t)interval_sec * GPS_ARI_HOLD_NUMERATOR) /
                        GPS_ARI_HOLD_DENOMINATOR;
    if (hold_sec == 0) hold_sec = 1;
    return hold_sec * 1000UL;
#else
    (void)interval_sec;
    return 0;
#endif
}

static uint16_t gps_adaptive_relay_apply_hold(uint16_t requested_interval_sec,
                                               uint32_t now_ms_value) {
#if GPS_ADAPTIVE_RELAY_INTERVAL
    gps_beacon_scheduler_t *s = &g_gps_beacon;

    if (s->interval_hold_active &&
        (int32_t)(now_ms_value - s->interval_hold_until_ms) >= 0) {
        s->interval_hold_active = false;
    }

    if (s->interval_hold_active) {
        // Shorter is always allowed immediately; equal/longer requests do not
        // extend the existing hold. This prevents a stable short interval from
        // refreshing its own timer forever.
        if (requested_interval_sec < s->held_interval_sec) {
            s->held_interval_sec = requested_interval_sec;
            s->interval_hold_until_ms = now_ms_value +
                gps_adaptive_relay_hold_ms(requested_interval_sec);
        }
        s->last_effective_interval_sec = s->held_interval_sec;
        return s->held_interval_sec;
    }

    // A downward change starts a hold. After a hold expires, a longer current
    // Adaptive value is allowed immediately and becomes the new baseline.
    if (s->last_effective_interval_sec != 0 &&
        requested_interval_sec < s->last_effective_interval_sec) {
        s->interval_hold_active = true;
        s->held_interval_sec = requested_interval_sec;
        s->interval_hold_until_ms = now_ms_value +
            gps_adaptive_relay_hold_ms(requested_interval_sec);
    }

    s->last_effective_interval_sec = requested_interval_sec;
    return requested_interval_sec;
#else
    (void)now_ms_value;
    return requested_interval_sec;
#endif
}

static double gps_bearing_delta_deg(double a, double b) {
    double d = a - b;
    if (d < 0.0) d = -d;
    while (d >= 360.0) d -= 360.0;
    return d <= 180.0 ? d : 360.0 - d;
}

static bool gps_adaptive_relay_corner_due(double speed_kmh, uint32_t elapsed_sec,
                                       double *turn_out, double *threshold_out) {
#if GPS_ADAPTIVE_RELAY_INTERVAL
    if (!gps_fix_is_current() || !g_gps.have_course_speed || speed_kmh <= 0.0) {
        return false;
    }
    if (!g_gps_beacon.turn_reference_valid) return false;
    if (elapsed_sec < GPS_ARI_MIN_TURN_TIME_SEC) return false;

    // Turn slope uses mph internally, matching the inherited threshold equation.
    // Speed thresholds remain configured in km/h for convenient Japanese operation.
    double speed_mph = speed_kmh * 0.621371192237334;
    if (speed_mph <= 0.0) return false;

    double turn = gps_bearing_delta_deg(g_gps.course_deg,
                                        g_gps_beacon.turn_reference_course_deg);
    double threshold = GPS_ARI_MIN_TURN_ANGLE_DEG + GPS_ARI_TURN_SLOPE / speed_mph;
    if (turn_out) *turn_out = turn;
    if (threshold_out) *threshold_out = threshold;
    return turn > threshold;
#else
    (void)speed_kmh; (void)elapsed_sec; (void)turn_out; (void)threshold_out;
    return false;
#endif
}

static void gps_format_fix_status(uint64_t fix_age_sec, char *out, size_t out_len) {
    if (!out || out_len == 0) return;
    out[0] = 0;

    // 0..60 seconds: no warning text, but keep an empty [] block.
    // This deliberately provides a visible delimiter between machine/data
    // fields and the following free-text comment.
    if (fix_age_sec <= GPS_FIX_STATUS_AFTER_SEC) {
        snprintf(out, out_len, "[]");
        return;
    }

    if (fix_age_sec <= (uint64_t)GPS_FIX_STATUS_MAX_MINUTES * 60ULL) {
        uint64_t minutes = fix_age_sec / 60ULL;
        snprintf(out, out_len, "[FIX:%llum]", (unsigned long long)minutes);
        return;
    }

    uint64_t hours = fix_age_sec / 3600ULL;
    if (hours > GPS_FIX_STATUS_MAX_HOURS) hours = GPS_FIX_STATUS_MAX_HOURS;
    snprintf(out, out_len, "[FIX:%lluh]", (unsigned long long)hours);
}

static bool aprs_append_block(char *dst, size_t dst_len, const char *block) {
    if (!dst || dst_len == 0 || !block || block[0] == 0) return true;

    size_t used = strlen(dst);
    size_t add = strlen(block);
    size_t separator = used ? 1U : 0U;
    if (used + separator + add + 1U > dst_len) return false;

    if (separator) dst[used++] = ' ';
    memcpy(dst + used, block, add + 1U);
    return true;
}

static bool gps_update_aprs_beacon(uint16_t relay_interval_sec, gps_beacon_reason_t reason) {
    if (kiss_priority_active()) {
        ESP_LOGD(TAG, "GPS/environment beacon skipped during KISS priority window");
        return false;
    }

    char info[192] = "";
    const char *origin = "GPS+ENV";

    if (!g_gps.have_position) {
        // Give a hot/warm-start receiver time to acquire its first position before
        // emitting a cold-start NOFIX packet. This suppresses short-lived startup
        // diagnostics during ordinary outdoor restarts and battery replacement.
        uint64_t uptime_sec = (uint64_t)esp_timer_get_time() / 1000000ULL;
        if (uptime_sec < (uint64_t)GPS_NOFIX_INITIAL_DELAY_SEC) return false;

        // Cold start: there is no map position yet. Use APRS User-Defined Data
        // instead of Status or positionless WX. Status/WX can be retained by
        // services independently from later position packets, which makes stale
        // [NOFIX] or stale indoor weather appear to follow the station.
        //
        // "{{N " means:
        //   '{' : APRS User-Defined Data type identifier
        //   '{' : experimental User ID
        //   'N' : BLE-APRS local NOFIX/health subtype
        // followed by one human-readable ASCII space. No position is fabricated.
        int n = snprintf(info, sizeof(info), "%s[NOFIX]",
                         GPS_APRS_NOFIX_USERDEF_PREFIX);
        if (n <= 0 || n >= (int)sizeof(info)) return false;

        // Keep cold-start diagnostic data human-readable even when normal fixed-
        // position operation is configured for APRS WX tokens. BME280 is optional
        // here: the NOFIX beacon itself remains a useful liveness indication.
        char environment[48] = "";
        if (gps_format_text_sensor_tokens(environment, sizeof(environment))) {
            if (!aprs_append_block(info, sizeof(info), environment)) return false;
        }
        if (!aprs_append_block(info, sizeof(info), GPS_APRS_COMMENT)) return false;
        origin = "USERDEF-NOFIX";
    } else {
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        uint64_t fix_age_sec = (g_gps.last_valid_fix_us != 0 && now_us >= g_gps.last_valid_fix_us)
                               ? (now_us - g_gps.last_valid_fix_us) / 1000000ULL
                               : 0ULL;

        // APRS CSE/SPD extension is degrees clockwise from north and speed in knots.
        // Once the last fix is no longer current, advertise 000/000 instead of a stale vector.
        int course = 0;
        int speed = 0;
        bool position_is_current = gps_fix_is_current();
        if (position_is_current && g_gps.have_course_speed) {
            speed = round_to_int(g_gps.speed_knots);
            if (speed < 0) speed = 0;
            if (speed > 999) speed = 999;

            if (speed > 0) {
                double c = g_gps.course_deg;
                while (c < 0.0) c += 360.0;
                while (c >= 360.0) c -= 360.0;
                course = round_to_int(c);
                if (course <= 0 || course > 360) course = 360;
            }
        }

        char aprs_prefix[10] = "!";
#if GPS_APRS_USE_TIMESTAMP
        char timestamp[8];
        if (gps_format_current_aprs_hms_timestamp(timestamp, sizeof(timestamp))) {
            // '/' is APRS "position with timestamp, no messaging".
            snprintf(aprs_prefix, sizeof(aprs_prefix), "/%s", timestamp);
        }
#endif

        // Block order: positioning -> environment -> warning/status -> free text.
        // Altitude belongs to the positioning block and therefore precedes environment data.
        char positioning[96];
        int alt_ft = round_to_int(g_gps.altitude_m * 3.28084);
        int n = snprintf(positioning, sizeof(positioning),
                         "%s%s%c%s%c%03d/%03d/A=%06d",
                         aprs_prefix,
                         g_gps.lat_aprs, GPS_APRS_SYMBOL_TABLE,
                         g_gps.lon_aprs, GPS_APRS_SYMBOL_CODE,
                         course, speed, alt_ft);
        if (n <= 0 || n >= (int)sizeof(positioning)) return false;

        snprintf(info, sizeof(info), "%s", positioning);

        char environment[48] = "";
        if (gps_format_environment_block(environment, sizeof(environment))) {
            if (!aprs_append_block(info, sizeof(info), environment)) return false;
        }

        char status[20] = "";
        gps_format_fix_status(fix_age_sec, status, sizeof(status));
        if (!aprs_append_block(info, sizeof(info), status)) return false;
        if (!aprs_append_block(info, sizeof(info), GPS_APRS_COMMENT)) return false;

        if (!g_gps.tx_ready_logged) {
            g_gps.tx_ready_logged = true;
            ESP_LOGI(TAG, "GPS valid fix acquired; position beacon enabled: %s %s",
                     g_gps.lat_aprs, g_gps.lon_aprs);
        }
    }

    uint8_t ax25[AX25_MAX];
    size_t info_len = strlen(info);
    size_t ax25_len = ax25_build_ui_frame(ax25, sizeof(ax25),
                                          GPS_APRS_SRC_CALL, GPS_APRS_SRC_SSID,
                                          GPS_APRS_DST_CALL, GPS_APRS_DST_SSID,
                                          (const uint8_t *)info, info_len);
    if (ax25_len == 0) return false;

    advertise_ax25_frame(ax25, ax25_len, relay_interval_sec, origin);
    if (reason == GPS_BEACON_REASON_TRACKING) {
        ESP_LOGD(TAG, "GPS payload: reason=%s speed=%.1fkm/h relay_interval=%us info=%s",
                 gps_beacon_reason_name(reason), gps_current_speed_kmh(),
                 (unsigned)relay_interval_sec, info);
    } else {
        ESP_LOGI(TAG, "GPS payload: reason=%s speed=%.1fkm/h relay_interval=%us info=%s",
                 gps_beacon_reason_name(reason), gps_current_speed_kmh(),
                 (unsigned)relay_interval_sec, info);
    }
    return true;
}

static void gps_maybe_debug_log(void) {
#if GPS_DEBUG_LOG_EVERY
    g_gps.sentence_updates++;
    if ((g_gps.sentence_updates % GPS_DEBUG_LOG_EVERY) != 0) return;

    uint64_t fix_age_sec = 0;
    if (g_gps.last_valid_fix_us != 0) {
        fix_age_sec = ((uint64_t)esp_timer_get_time() - g_gps.last_valid_fix_us) / 1000000ULL;
    }

    gps_debug_printf("GPS time=%s date=%s pos=%s/%s alt=%.1fm speed=%.1fkt course=%.1fdeg fix=%s age=%llus q=%d gsa=%d dop=%.1f/%.1f/%.1f used=%d detected=%d",
                     g_gps.utc_time[0] ? g_gps.utc_time : "-",
                     g_gps.utc_date[0] ? g_gps.utc_date : "-",
                     g_gps.have_position ? g_gps.lat_aprs : "-",
                     g_gps.have_position ? g_gps.lon_aprs : "-",
                     g_gps.altitude_m,
                     g_gps.speed_knots,
                     g_gps.course_deg,
                     g_gps.valid_fix ? "valid" : "invalid",
                     (unsigned long long)fix_age_sec,
                     g_gps.fix_quality,
                     g_gps.gsa_fix_type,
                     g_gps.pdop, g_gps.hdop, g_gps.vdop,
                     g_gps.active_satellites,
                     g_gps.detected_satellites);
#endif
}

static void gps_parse_gga(char **f, int nf) {
    if (nf < 10) return;
    if (f[1] && f[1][0]) {
        strncpy(g_gps.utc_time, f[1], sizeof(g_gps.utc_time) - 1);
        g_gps.utc_time[sizeof(g_gps.utc_time) - 1] = 0;
        gps_clock_update(f[1], NULL);
    }

    char lat[sizeof(g_gps.lat_aprs)];
    char lon[sizeof(g_gps.lon_aprs)];
    bool pos_ok = nmea_to_aprs_lat(f[2], f[3], lat, sizeof(lat)) &&
                  nmea_to_aprs_lon(f[4], f[5], lon, sizeof(lon));

    g_gps.fix_quality = atoi(f[6] ? f[6] : "0");
    int sats = atoi(f[7] ? f[7] : "0");
    if (g_gps.active_satellites == 0) g_gps.active_satellites = sats;
    g_gps.hdop = atof(f[8] ? f[8] : "0");
    g_gps.valid_fix = g_gps.fix_quality > 0;

    status_led_gps_update(g_gps.detected_satellites, g_gps.gsa_fix_type, g_gps.valid_fix);

    if (g_gps.valid_fix && pos_ok) {
        strncpy(g_gps.lat_aprs, lat, sizeof(g_gps.lat_aprs));
        g_gps.lat_aprs[sizeof(g_gps.lat_aprs) - 1] = 0;
        strncpy(g_gps.lon_aprs, lon, sizeof(g_gps.lon_aprs));
        g_gps.lon_aprs[sizeof(g_gps.lon_aprs) - 1] = 0;
        g_gps.altitude_m = atof(f[9] ? f[9] : "0");
        g_gps.have_position = true;
        g_gps.last_valid_fix_us = (uint64_t)esp_timer_get_time();
    }

    gps_maybe_debug_log();
}

static void gps_parse_rmc(char **f, int nf) {
    if (nf < 10) return;
    if (f[1] && f[1][0]) {
        strncpy(g_gps.utc_time, f[1], sizeof(g_gps.utc_time) - 1);
        g_gps.utc_time[sizeof(g_gps.utc_time) - 1] = 0;
    }
    if (f[9] && f[9][0]) {
        strncpy(g_gps.utc_date, f[9], sizeof(g_gps.utc_date) - 1);
        g_gps.utc_date[sizeof(g_gps.utc_date) - 1] = 0;
    }
    if (f[1] && f[1][0]) {
        gps_clock_update(f[1], (f[9] && f[9][0]) ? f[9] : NULL);
    }

    char lat[sizeof(g_gps.lat_aprs)];
    char lon[sizeof(g_gps.lon_aprs)];
    bool pos_ok = nmea_to_aprs_lat(f[3], f[4], lat, sizeof(lat)) &&
                  nmea_to_aprs_lon(f[5], f[6], lon, sizeof(lon));

    g_gps.valid_fix = (f[2] && f[2][0] == 'A');
    status_led_gps_update(g_gps.detected_satellites, g_gps.gsa_fix_type, g_gps.valid_fix);

    if (g_gps.valid_fix && pos_ok) {
        strncpy(g_gps.lat_aprs, lat, sizeof(g_gps.lat_aprs));
        g_gps.lat_aprs[sizeof(g_gps.lat_aprs) - 1] = 0;
        strncpy(g_gps.lon_aprs, lon, sizeof(g_gps.lon_aprs));
        g_gps.lon_aprs[sizeof(g_gps.lon_aprs) - 1] = 0;
        g_gps.have_position = true;
        g_gps.last_valid_fix_us = (uint64_t)esp_timer_get_time();

        g_gps.speed_knots = atof(f[7] ? f[7] : "0");
        g_gps.course_deg = atof(f[8] ? f[8] : "0");
        g_gps.have_course_speed = f[7] && f[7][0] && f[8] && f[8][0];
    } else {
        g_gps.have_course_speed = false;
    }

    gps_maybe_debug_log();
}

static void gps_parse_gsa(char **f, int nf) {
    if (nf < 18) return;
    g_gps.gsa_fix_type = atoi(f[2] ? f[2] : "1");

    int active = 0;
    for (int i = 3; i <= 14 && i < nf; i++) {
        if (f[i] && f[i][0]) active++;
    }
    g_gps.active_satellites = active;

    g_gps.pdop = atof(f[15] ? f[15] : "0");
    g_gps.hdop = atof(f[16] ? f[16] : "0");
    g_gps.vdop = atof(f[17] ? f[17] : "0");
    status_led_gps_update(g_gps.detected_satellites, g_gps.gsa_fix_type, g_gps.valid_fix);
}

static void gps_parse_gsv(char **f, int nf) {
    // GSV is commonly split across multiple sentences.  Count satellites only
    // after a complete sequential cycle so the LED does not step through the
    // partial counts (for example 2 -> 5 -> 8 -> 2 -> ...).
    if (nf < 4) return;

    int total_messages = atoi(f[1] ? f[1] : "0");
    int message_number = atoi(f[2] ? f[2] : "0");
    if (total_messages <= 0 || message_number <= 0 || message_number > total_messages) {
        return;
    }

    if (message_number == 1) {
        g_gps.gsv_expected_messages = total_messages;
        g_gps.gsv_last_message = 0;
        g_gps.gsv_cycle_detected = 0;
    } else if (g_gps.gsv_expected_messages != total_messages ||
               message_number != g_gps.gsv_last_message + 1) {
        // A GSV fragment was missed or a new cycle appeared out of sequence.
        // Discard this partial cycle and wait for the next message 1.
        g_gps.gsv_expected_messages = 0;
        g_gps.gsv_last_message = 0;
        g_gps.gsv_cycle_detected = 0;
        return;
    }

    int detected_this_sentence = 0;
    // Each satellite block is: PRN, elevation, azimuth, SNR.
    // Some NMEA variants append an extra signal-ID field after the blocks; the
    // i+3 guard naturally ignores it.  Empty SNR or 0 means there is no useful
    // RF detection for this simple indicator.
    for (int i = 4; i + 3 < nf; i += 4) {
        if (!f[i] || !f[i][0] || !f[i + 3] || !f[i + 3][0]) continue;

        char *end = NULL;
        long snr = strtol(f[i + 3], &end, 10);
        if (end != f[i + 3] && snr > 0) {
            detected_this_sentence++;
        }
    }

    g_gps.gsv_cycle_detected += detected_this_sentence;
    g_gps.gsv_last_message = message_number;

    if (message_number == total_messages) {
        g_gps.detected_satellites = g_gps.gsv_cycle_detected;
        status_led_gps_update(g_gps.detected_satellites,
                              g_gps.gsa_fix_type,
                              g_gps.valid_fix);

        // Mark the cycle complete.  Keep detected_satellites as the last stable
        // result until another complete GSV cycle arrives.
        g_gps.gsv_expected_messages = 0;
        g_gps.gsv_last_message = 0;
        g_gps.gsv_cycle_detected = 0;
    }
}

static void gps_parse_sentence(const char *line) {
    if (!line || line[0] == 0) return;
    g_gps.total_lines++;
    g_gps.last_rx_ms = now_ms();

    if (!nmea_checksum_ok(line)) {
        g_gps.checksum_errors++;
        return;
    }

    char buf[128];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    char *star = strchr(buf, '*');
    if (star) *star = 0;

    char *fields[32] = {0};
    int nf = split_csv(buf, fields, 32);
    if (nf <= 0) return;

    if (strcmp(fields[0], "$GPGGA") == 0) {
        g_gps.parsed_sentences++;
        gps_parse_gga(fields, nf);
    } else if (strcmp(fields[0], "$GPRMC") == 0) {
        g_gps.parsed_sentences++;
        gps_parse_rmc(fields, nf);
    } else if (strcmp(fields[0], "$GPGSA") == 0) {
        g_gps.parsed_sentences++;
        gps_parse_gsa(fields, nf);
    } else if (strcmp(fields[0], "$GPGSV") == 0) {
        g_gps.parsed_sentences++;
        gps_parse_gsv(fields, nf);
    } else {
        g_gps.unsupported_sentences++;
    }
}

static void gps_maybe_status_log(void) {
#if GPS_STATUS_LOG_MS
    uint32_t now = now_ms();
    if (g_gps.last_status_log_ms != 0 &&
        (now - g_gps.last_status_log_ms) < GPS_STATUS_LOG_MS) {
        return;
    }
    g_gps.last_status_log_ms = now;

    uint32_t age = g_gps.last_rx_ms ? (now - g_gps.last_rx_ms) : 0;
    uint64_t fix_age_sec = 0;
    if (g_gps.last_valid_fix_us != 0) {
        fix_age_sec = ((uint64_t)esp_timer_get_time() - g_gps.last_valid_fix_us) / 1000000ULL;
    }

    gps_debug_printf("GPS status: lines=%u parsed=%u cksum_err=%u unsup=%u last_rx=%ums fix=%s fix_age=%llus q=%d gsa=%d hdop=%.1f used=%d detected=%d",
                     g_gps.total_lines,
                     g_gps.parsed_sentences,
                     g_gps.checksum_errors,
                     g_gps.unsupported_sentences,
                     age,
                     g_gps.valid_fix ? "valid" : "invalid",
                     (unsigned long long)fix_age_sec,
                     g_gps.fix_quality,
                     g_gps.gsa_fix_type,
                     g_gps.hdop,
                     g_gps.active_satellites,
                     g_gps.detected_satellites);
#endif
}

static void gps_rx_task(void *arg) {
    (void)arg;
    uint8_t rx[96];
    char line[128];
    size_t line_len = 0;

    while (1) {
        int n = uart_read_bytes(GPS_UART_PORT, rx, sizeof(rx), pdMS_TO_TICKS(200));
        for (int i = 0; i < n; i++) {
            char c = (char)rx[i];
            if (c == '\r' || c == '\n') {
                if (line_len > 0) {
                    line[line_len] = 0;
                    gps_parse_sentence(line);
                    line_len = 0;
                }
                continue;
            }
            if (line_len + 1 < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0;
            }
        }
        gps_maybe_status_log();
    }
}

static void gps_beacon_task(void *arg) {
    (void)arg;

    while (1) {
        uint32_t now = now_ms();
        bool current_fix = gps_fix_is_current();
        bool kiss_now = kiss_priority_active();

        if (kiss_now) {
            // KISS input owns the advertiser for the priority window. Remember
            // this so the latest GPS payload can retake advertising immediately
            // when the priority window closes.
            g_gps_beacon.was_kiss_priority = true;
            vTaskDelay(pdMS_TO_TICKS(GPS_BEACON_UPDATE_INTERVAL_MS));
            continue;
        }

        bool resume_after_kiss = g_gps_beacon.was_kiss_priority;
        g_gps_beacon.was_kiss_priority = false;

        double speed_kmh = gps_current_speed_kmh();
        uint16_t rate_sec = gps_adaptive_relay_rate_sec(speed_kmh);
        uint16_t requested_interval_sec = rate_sec;
        uint16_t relay_interval_sec = rate_sec;
        gps_beacon_reason_t reason = GPS_BEACON_REASON_TRACKING;
        double turn = 0.0;
        double turn_threshold = 0.0;

        // GPS FIX state is deliberately not a relay-priority trigger. A stale or
        // missing fix is already represented in the APRS payload ([FIX:Xm], [NOFIX]),
        // so brief indoor/tunnel FIX flapping must not turn into repeated short
        // relay requests. Relay priority is driven only by movement (speed/turn).
        if (!g_gps_beacon.have_payload) {
            reason = GPS_BEACON_REASON_INITIAL;
            // A station with no previous I-Gate timestamp is already admitted, so
            // the initial payload can use the normal Adaptive interval.
        } else if (resume_after_kiss) {
            reason = GPS_BEACON_REASON_KISS_RESUME;
        }

#if GPS_ADAPTIVE_RELAY_INTERVAL
        // Initialize/reinitialize the local corner reference from a fresh moving
        // solution. It is deliberately separate from I-Gate timing: the gateway's
        // last_relay_ms remains the authority that decides whether a packet passes.
        // After any FIX loss the reference is invalidated below, so reacquisition
        // simply creates a new turn baseline without requesting an early relay.
        if (current_fix && g_gps.have_course_speed && speed_kmh > 0.0) {
            if (!g_gps_beacon.turn_reference_valid) {
                g_gps_beacon.turn_reference_valid = true;
                g_gps_beacon.turn_reference_ms = now;
                g_gps_beacon.turn_reference_course_deg = g_gps.course_deg;
            } else {
                uint32_t ref_elapsed_sec =
                    (now - g_gps_beacon.turn_reference_ms) / 1000UL;

                if (gps_adaptive_relay_corner_due(speed_kmh, ref_elapsed_sec,
                                               &turn, &turn_threshold)) {
                    // Corner Pegging requests the configured minimum turn time,
                    // rather than stopping/restarting BLE or suppressing nonce
                    // updates. If the I-Gate has already waited this long, the
                    // latest packet can pass immediately.
                    reason = GPS_BEACON_REASON_TURN;
                    requested_interval_sec = GPS_ARI_MIN_TURN_TIME_SEC;
                    g_gps_beacon.turn_reference_ms = now;
                    g_gps_beacon.turn_reference_course_deg = g_gps.course_deg;
                } else if (ref_elapsed_sec >= rate_sec) {
                    // Virtual regular-beacon reference. This keeps turn comparison
                    // close to a conventional "bearing since last beacon" behavior
                    // even though actual relay timing belongs to the I-Gate.
                    g_gps_beacon.turn_reference_ms = now;
                    g_gps_beacon.turn_reference_course_deg = g_gps.course_deg;
                }
            }
        } else if (!current_fix) {
            // A stale/no-fix course must not be used as a future turn baseline.
            g_gps_beacon.turn_reference_valid = false;
        }
#endif

        // BLE-APRS connectionless reliability aid: whenever the requested relay
        // interval becomes shorter, keep that shorter value on-air for 2/3 of the
        // interval. During the hold, only an even shorter request can replace it.
        // Example: a 15 s corner request remains advertised for 10 s.
        relay_interval_sec = gps_adaptive_relay_apply_hold(requested_interval_sec, now);

        // Rebuild and advertise the latest APRS payload every update. This is the
        // key BLE-APRS behavior: nonce/data stay fresh while relay_interval alone
        // tells the I-Gate how aggressively this station currently wants relaying.
        bool sent = gps_update_aprs_beacon(relay_interval_sec, reason);
        if (sent) {
            g_gps_beacon.have_payload = true;

            // Once GPS has taken the advertiser back from KISS, the old KISS
            // lifetime timer must not later stop the GPS advertisement.
            if (g_last_kiss_input_ms != 0) g_last_kiss_input_ms = 0;

#if GPS_ADAPTIVE_RELAY_INTERVAL
            if (reason == GPS_BEACON_REASON_TURN) {
                ESP_LOGI(TAG,
                         "Adaptive Relay Interval: corner turn=%.1fdeg threshold=%.1fdeg interval=%us",
                         turn, turn_threshold, (unsigned)relay_interval_sec);
            } else if (reason != GPS_BEACON_REASON_TRACKING) {
                ESP_LOGI(TAG,
                         "Adaptive Relay Interval: reason=%s speed=%.1fkm/h interval=%us",
                         gps_beacon_reason_name(reason), speed_kmh,
                         (unsigned)relay_interval_sec);
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(GPS_BEACON_UPDATE_INTERVAL_MS));
    }
}

static void gps_uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, GPS_UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, GPS_UART_TX_PIN, GPS_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "GPS NMEA input: UART%d RX GPIO%d baud=%d", GPS_UART_PORT, GPS_UART_RX_PIN, GPS_UART_BAUD);
    gps_debug_printf("GPS NMEA input: UART%d RX GPIO%d baud=%d", GPS_UART_PORT, GPS_UART_RX_PIN, GPS_UART_BAUD);
}

#endif // ENABLE_GPS_NMEA

// ---------------- UART/KISS task ----------------

static void on_kiss_ax25_frame(const uint8_t *ax25, size_t len) {
#if ENABLE_KISS_TO_BLE
    status_led_usb_kiss_event();

    uint16_t interval = aprs_should_request_immediate(ax25, len)
                        ? APRS_MESSAGE_RELAY_INTERVAL_SEC
                        : DEFAULT_RELAY_INTERVAL_SEC;

    g_last_kiss_input_ms = now_ms();
    advertise_ax25_frame(ax25, len, interval, "KISS");
#else
    (void)ax25; (void)len;
#endif
}

static void kiss_rx_task(void *arg) {
    (void)arg;
    uint8_t rx[128];
    uint8_t frame[AX25_MAX + 8];
    size_t frame_len = 0;
    bool in_frame = false;
    bool esc = false;

    while (1) {
        int n = io_read_bytes(rx, sizeof(rx), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            uint8_t c = rx[i];

            if (c == KISS_FEND) {
                if (in_frame && frame_len > 0) {
                    uint8_t cmd = frame[0] & 0x0F;
                    if (cmd == KISS_CMD_DATA && frame_len > 1) {
                        on_kiss_ax25_frame(frame + 1, frame_len - 1);
                    }
                    // A non-empty frame was closed. Return to text/debug idle state so
                    // ordinary keyboard input is echoed until the next FEND starts a frame.
                    in_frame = false;
                } else {
                    // Empty FEND delimiters are common as KISS frame flush bytes. Treat one
                    // or more empty delimiters as preparing for the next frame, not as data.
                    in_frame = true;
                }
                esc = false;
                frame_len = 0;
                continue;
            }

            if (!in_frame) {
#if ECHO_NON_KISS_TEXT
                // Echo normal hand-typed text when we are not inside a KISS frame.
                // This is only for debug; APRSdroid/KISS operation is unaffected.
                io_write_bytes(&c, 1);

#endif
                continue;
            }

            if (esc) {
                if (c == KISS_TFEND) c = KISS_FEND;
                else if (c == KISS_TFESC) c = KISS_FESC;
                esc = false;
            } else if (c == KISS_FESC) {
                esc = true;
                continue;
            }

            if (frame_len < sizeof(frame)) {
                frame[frame_len++] = c;
            } else {
                ESP_LOGW(TAG, "KISS frame too large; dropping");
                in_frame = false;
                frame_len = 0;
                esc = false;
            }
        }

        // For KISS-originated frames, stop advertising after lifetime expires.
        if (g_adv_started && g_last_kiss_input_ms != 0 &&
            (now_ms() - g_last_kiss_input_ms) > KISS_ADVERTISE_LIFETIME_SEC * 1000UL) {
            ESP_LOGI(TAG, "KISS payload lifetime expired; stopping advertising");
            ble_adv_stop();
            g_last_kiss_input_ms = 0;
        }

    }
}

static void io_init_for_kiss(void) {
#if USE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = UART_TX_BUF,
        .rx_buffer_size = UART_RX_BUF,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI(TAG, "KISS I/O: USB Serial/JTAG");
#else
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_LOGI(TAG, "KISS I/O: UART%d baud=%d", UART_PORT, UART_BAUD);
#endif
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    board_init();
    status_led_init();
#if ENABLE_STATUS_LEDS
#if CONFIG_IDF_TARGET_ESP32C6
    // GPIO15 RX indication is independent of the external PL9823 chain.
    xTaskCreate(status_led_task, "status_led", 3072, NULL, 4, NULL);
#else
    if (g_led_strip) {
        xTaskCreate(status_led_task, "status_led", 3072, NULL, 4, NULL);
    }
#endif
#endif
    io_init_for_kiss();

#if ENABLE_BLE_TO_KISS
    // Create the BLE->KISS queue/task before ble_init() starts extended scanning.
    // This guarantees GAP reports can never arrive before the host-TX path exists.
    g_kiss_tx_queue = xQueueCreate(GW_KISS_TX_QUEUE_LEN, sizeof(kiss_tx_item_t));
    ESP_ERROR_CHECK(g_kiss_tx_queue ? ESP_OK : ESP_ERR_NO_MEM);
    BaseType_t kiss_tx_task_ok =
        xTaskCreate(kiss_tx_task, "kiss_tx", 4096, NULL, 9, NULL);
    ESP_ERROR_CHECK(kiss_tx_task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
#endif

#if ENABLE_GPS_NMEA
    gps_uart_init();
#endif
#if ENABLE_BME280
    bool bme280_ready = bme280_init();
#endif
    ble_init();

    xTaskCreate(kiss_rx_task, "kiss_rx", 4096, NULL, 8, NULL);
#if ENABLE_GPS_NMEA
    xTaskCreate(gps_rx_task, "gps_rx", 4096, NULL, 7, NULL);
    xTaskCreate(gps_beacon_task, "gps_beacon", 4096, NULL, 6, NULL);
#endif
#if ENABLE_BME280
    if (bme280_ready) {
        xTaskCreate(bme280_task, "bme280", 4096, NULL, 6, NULL);
    }
#endif

#if ENABLE_GPS_NMEA
#if GPS_APRS_ENV_FORMAT == GPS_APRS_ENV_FORMAT_WX
    ESP_LOGI(TAG, "APRS environment format: WX%s",
             GPS_APRS_USE_TIMESTAMP ? ", APRS HMS timestamp enabled" : "");
#elif GPS_APRS_ENV_FORMAT == GPS_APRS_ENV_FORMAT_TEXT
    ESP_LOGI(TAG, "APRS environment format: TEXT metric%s",
             GPS_APRS_USE_TIMESTAMP ? ", APRS HMS timestamp enabled" : "");
#endif
#endif
    ESP_LOGI(TAG, "BLE-APRS bridge started");
}

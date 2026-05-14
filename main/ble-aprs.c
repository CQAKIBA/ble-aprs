// BLE-APRS experimental bridge for ESP-IDF / ESP32-C3, ESP32-C6 class devices
//
// Function:
//   UART KISS input  -> BLE Extended Advertising, LE Coded PHY
//   BLE-APRS receive -> UART KISS output
//
// BLE-APRS v0 payload:
//   "$APRS,1,<relay_interval_sec>,<nonce>>" + raw AX.25 frame without FCS
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
//   CONFIG_BT_BLE_42_FEATURES_SUPPORTED=y
//   CONFIG_BT_BLUEDROID_ENABLED=y
//   CONFIG_BT_BLE_DYNAMIC_ENV_MEMORY=y
//   CONFIG_BT_BLE_FEAT_EXT_ADV=y       // name varies by IDF version
//   CONFIG_BT_BLE_FEAT_ADV_CODING_SELECTION=y  // optional, for S=8/S=2 preference if available
//
// NeoPixel debug LED:
//   XIAO ESP32-C3 D10 is assumed to be GPIO10. Adjust NEOPIXEL_GPIO if your board variant differs.
//   TX advertising update / heartbeat: red flash
//   KISS frame received from host:       green flash
//   BLE-APRS frame relayed to KISS:      blue flash
//
// Nonce behavior:
//   The transmitter updates nonce for every valid KISS input frame, even if the AX.25 payload is identical.
//   The receiver suppresses only same nonce and relay_interval violations.
//   Identical AX.25 payload with a new nonce is allowed after relay_interval,
//   matching normal APRS fixed-position beacons.

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "led_strip.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#define TAG "BLE_APRS"

// ---------------- User settings ----------------

// XIAO ESP32-C3 usually exposes the ESP32-C3 native USB Serial/JTAG interface on USB-C.
// If you connect an external USB-UART adapter to hardware UART pins instead, set this to 0.
#define USE_USB_SERIAL_JTAG    1

#define UART_PORT              UART_NUM_0
#define UART_BAUD              9600
#define UART_RX_BUF            2048
#define UART_TX_BUF            2048

// 250 bytes total BLE-APRS payload budget.
#define BLE_APRS_MAX_TOTAL     250
#define AX25_MAX               230

#define DEFAULT_RELAY_INTERVAL_SEC 60
#define APRS_MESSAGE_RELAY_INTERVAL_SEC 0  // immediate request
#define KISS_ADVERTISE_LIFETIME_SEC 600

// BLE advertising interval. 300 ms is gentle for a first range test.
#define ADV_INTERVAL_MS        300
#define EXT_ADV_INSTANCE       0

// BLE advertising/scan interval unit is 0.625 ms.
// Some ESP-IDF versions do not provide ESP_BLE_GAP_ADV_ITVL_MS() /
// ESP_BLE_GAP_SCAN_ITVL_MS(), so keep the conversion local and constant-safe.
#define BLE_MS_TO_0_625MS_UNITS(ms) ((uint16_t)(((uint32_t)(ms) * 1000U) / 625U))

// Enable both for two boards that can act as bridge/repeater-ish test nodes.
#define ENABLE_KISS_TO_BLE     1
#define ENABLE_BLE_TO_KISS     1

// Debug helper: echo ordinary text typed outside KISS frames.
// This makes it easy to verify that Tera Term input reaches the ESP app.
#define ECHO_NON_KISS_TEXT     1

// Debug helper: after outputting a KISS binary frame, also print human-readable text.
// This text is outside KISS FEND frames, so real KISS clients should treat it as garbage/ignore it.
// Set to 0 when connecting to strict software if needed.
#define KISS_DEBUG_MONITOR_TEXT 1

// Gateway-side minimum interval for relaying BLE packets to UART KISS.
// This is receiver-side protection only.
#define GW_MIN_RELAY_INTERVAL_SEC 1
#define GW_MAX_RELAY_INTERVAL_SEC 3600
#define STATION_CACHE_TTL_SEC     3600

// Fail-safe token bucket per station.
// Normal safe intervals should never hit this. It only catches bursts/misconfiguration.
#define GW_TOKEN_MAX              10
#define GW_TOKEN_REFILL_MS        6000UL   // +1 token per 6 seconds = 10/minute
#define GW_BAN_TIME_MS            300000UL // 5 minutes

// ---------------- NeoPixel debug LED ----------------

#define ENABLE_NEOPIXEL_DEBUG 1
#define NEOPIXEL_GPIO         10     // XIAO ESP32-C3 D10 is usually GPIO10
#define NEOPIXEL_LED_COUNT    1
#define NEOPIXEL_BRIGHTNESS   8     // 0..255, keep low for a single status LED
#define NEOPIXEL_FLASH_MS     80
#define NEOPIXEL_TX_HEARTBEAT_MS 1000

// ---------------- KISS constants ----------------

#define KISS_FEND   0xC0
#define KISS_FESC   0xDB
#define KISS_TFEND  0xDC
#define KISS_TFESC  0xDD
#define KISS_CMD_DATA 0x00

// ---------------- BLE-APRS state ----------------

static uint8_t g_current_adv[BLE_APRS_MAX_TOTAL];
static size_t  g_current_adv_len = 0;
static bool    g_adv_started = false;
static uint32_t g_last_kiss_input_ms = 0;
static uint32_t g_nonce_counter = 0;
static uint32_t g_last_tx_flash_ms = 0;

#if ENABLE_NEOPIXEL_DEBUG
static led_strip_handle_t g_led_strip = NULL;
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

    uint8_t tokens;
    uint32_t last_token_ms;
    uint32_t ban_until_ms;
} station_cache_t;

#define STATION_CACHE_SIZE 32
static station_cache_t g_cache[STATION_CACHE_SIZE];

// ---------------- Utility ----------------

static int io_read_bytes(uint8_t *buf, size_t len, TickType_t ticks_to_wait);
static void io_write_bytes(const uint8_t *buf, size_t len);

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

static void neopixel_set(uint8_t r, uint8_t g, uint8_t b) {
#if ENABLE_NEOPIXEL_DEBUG
    if (!g_led_strip) return;
    uint32_t rr = ((uint32_t)r * NEOPIXEL_BRIGHTNESS) / 255;
    uint32_t gg = ((uint32_t)g * NEOPIXEL_BRIGHTNESS) / 255;
    uint32_t bb = ((uint32_t)b * NEOPIXEL_BRIGHTNESS) / 255;
    led_strip_set_pixel(g_led_strip, 0, rr, gg, bb);
    led_strip_refresh(g_led_strip);
#else
    (void)r; (void)g; (void)b;
#endif
}

static void neopixel_off(void) {
#if ENABLE_NEOPIXEL_DEBUG
    if (!g_led_strip) return;
    led_strip_clear(g_led_strip);
#endif
}

static void neopixel_flash(uint8_t r, uint8_t g, uint8_t b, uint32_t ms) {
#if ENABLE_NEOPIXEL_DEBUG
    neopixel_set(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(ms));
    neopixel_off();
#else
    (void)r; (void)g; (void)b; (void)ms;
#endif
}

static void neopixel_init(void) {
#if ENABLE_NEOPIXEL_DEBUG
    led_strip_config_t strip_config = {
        .strip_gpio_num = NEOPIXEL_GPIO,
        .max_leds = NEOPIXEL_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
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
        ESP_LOGW(TAG, "NeoPixel init failed: %s", esp_err_to_name(err));
        g_led_strip = NULL;
        return;
    }
    neopixel_off();
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
#if USE_USB_SERIAL_JTAG
    return usb_serial_jtag_read_bytes(buf, len, ticks_to_wait);
#else
    return uart_read_bytes(UART_PORT, buf, len, ticks_to_wait);
#endif
}

static void io_write_bytes(const uint8_t *buf, size_t len) {
#if USE_USB_SERIAL_JTAG
    usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(100));
#else
    uart_write_bytes(UART_PORT, (const char *)buf, len);
#endif
}

static void kiss_write_frame(const uint8_t *ax25, size_t len) {
    uint8_t b;

    b = KISS_FEND;
    io_write_bytes(&b, 1);

    b = KISS_CMD_DATA;
    io_write_bytes(&b, 1);

    for (size_t i = 0; i < len; i++) {
        if (ax25[i] == KISS_FEND) {
            uint8_t esc[2] = {KISS_FESC, KISS_TFEND};
            io_write_bytes(esc, 2);
        } else if (ax25[i] == KISS_FESC) {
            uint8_t esc[2] = {KISS_FESC, KISS_TFESC};
            io_write_bytes(esc, 2);
        } else {
            io_write_bytes(&ax25[i], 1);
        }
    }

    b = KISS_FEND;
    io_write_bytes(&b, 1);
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

static void ble_adv_update_payload(const uint8_t *data, size_t len) {
    if (len > BLE_APRS_MAX_TOTAL) return;
    memcpy(g_current_adv, data, len);
    g_current_adv_len = len;

    if (g_adv_started) {
        uint8_t inst = EXT_ADV_INSTANCE;
        esp_ble_gap_ext_adv_stop(1, &inst);
    }
    esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_INSTANCE, (uint16_t)g_current_adv_len, g_current_adv);
}

static void ble_adv_start_if_data_ready(void) {
    if (g_current_adv_len == 0) return;
    if (!g_adv_started) {
        esp_ble_gap_ext_adv_start(1, &ext_adv_enable);
        g_adv_started = true;
        g_last_tx_flash_ms = now_ms();
        neopixel_flash(255, 0, 0, NEOPIXEL_FLASH_MS);
    }
}

static void ble_adv_stop(void) {
    if (g_adv_started) {
        uint8_t inst = EXT_ADV_INSTANCE;
        esp_ble_gap_ext_adv_stop(1, &inst);
        g_adv_started = false;
    }
    g_current_adv_len = 0;
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

static bool station_consume_token_or_ban(station_cache_t *s) {
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

    s->tokens--;
    return true;
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
    if (interval != 0 && s->last_relay_ms != 0 && (t - s->last_relay_ms) < interval * 1000UL) return;

    // Final fail-safe: token bucket catches bursts/misconfiguration.
    // Interval/nonce drops above do not consume tokens; only actual relay attempts do.
    if (!station_consume_token_or_ban(s)) return;

    kiss_write_frame(p.ax25, p.ax25_len);
    kiss_write_debug_monitor(p.ax25, p.ax25_len, p.nonce, interval, rssi);
    neopixel_flash(0, 0, 255, NEOPIXEL_FLASH_MS);

    s->last_relay_ms = t;
    s->last_payload_hash = payload_hash;
    strncpy(s->last_nonce, p.nonce, sizeof(s->last_nonce));
    s->last_nonce[sizeof(s->last_nonce) - 1] = 0;

    ESP_LOGI(TAG, "Relayed BLE-APRS to KISS: len=%u nonce=%s interval=%u rssi=%d tokens=%u",
             (unsigned)p.ax25_len, p.nonce, interval, rssi, s->tokens);
}

// ---------------- GAP callback ----------------

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        ESP_LOGI(TAG, "Ext adv params set");
        break;

    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Ext adv data set: %u bytes", (unsigned)g_current_adv_len);
        ble_adv_start_if_data_ready();
        break;

    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "Ext adv started");
        g_adv_started = true;
        break;

    case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Ext adv stopped");
        g_adv_started = false;
        if (g_current_adv_len > 0) {
            esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_INSTANCE, (uint16_t)g_current_adv_len, g_current_adv);
        }
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

// ---------------- UART/KISS task ----------------

static void on_kiss_ax25_frame(const uint8_t *ax25, size_t len) {
#if ENABLE_KISS_TO_BLE
    neopixel_flash(0, 255, 0, NEOPIXEL_FLASH_MS);

    if (!ax25_minimal_validate(ax25, len)) {
        ESP_LOGW(TAG, "KISS frame is not AX.25 UI/PID F0, len=%u", (unsigned)len);
        return;
    }

    uint8_t payload[BLE_APRS_MAX_TOTAL];
    size_t payload_len = 0;
    uint16_t interval = aprs_should_request_immediate(ax25, len)
                        ? APRS_MESSAGE_RELAY_INTERVAL_SEC
                        : DEFAULT_RELAY_INTERVAL_SEC;

    if (!build_ble_aprs_payload(ax25, len, interval, payload, &payload_len)) {
        return;
    }

    g_last_kiss_input_ms = now_ms();
    ble_adv_update_payload(payload, payload_len);

    ESP_LOGI(TAG, "KISS -> BLE-APRS adv update: ax25=%u total=%u interval=%u%s",
             (unsigned)len, (unsigned)payload_len, interval,
             (interval == 0) ? " immediate" : "");
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
                if (in_frame && frame_len >= 1) {
                    uint8_t cmd = frame[0] & 0x0F;
                    if (cmd == KISS_CMD_DATA && frame_len > 1) {
                        on_kiss_ax25_frame(frame + 1, frame_len - 1);
                    }
                }
                in_frame = true;
                esc = false;
                frame_len = 0;
                continue;
            }

            if (!in_frame) {
#if ECHO_NON_KISS_TEXT
                // Echo normal hand-typed text when we are not inside a KISS frame.
                // This is only for debug; APRSdroid/KISS operation is unaffected.
                io_write_bytes(&c, 1);

                // Optional tiny visual cue that ordinary serial input reached the app.
                // Use numeric CR/LF constants to avoid character-literal escaping issues.
                if (c != 0x0D && c != 0x0A) {
                    neopixel_flash(16, 16, 16, 20);
                }
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

        if (g_adv_started && g_current_adv_len > 0 &&
            (now_ms() - g_last_tx_flash_ms) >= NEOPIXEL_TX_HEARTBEAT_MS) {
            g_last_tx_flash_ms = now_ms();
            neopixel_flash(255, 0, 0, NEOPIXEL_FLASH_MS);
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

    neopixel_init();
    io_init_for_kiss();
    ble_init();

    xTaskCreate(kiss_rx_task, "kiss_rx", 4096, NULL, 8, NULL);

    ESP_LOGI(TAG, "BLE-APRS bridge started");
}

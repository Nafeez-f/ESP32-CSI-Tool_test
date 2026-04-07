/*
 * passive_csi — clean passive CSI sniffer for ISAC research
 * ==========================================================
 *
 * What this does
 * --------------
 * Sits on one Wi-Fi channel in promiscuous / null mode (does NOT join any
 * network).  Every frame received — data frames AND management frames
 * (including beacons) — triggers the Wi-Fi chip's channel estimator, which
 * produces CSI.  Both the CSI and the raw frame metadata are written to the
 * serial port as a CSV row.
 *
 * Why management frames matter for ISAC
 * ---------------------------------------
 * Your AP/hotspot sends a beacon frame every ~100 ms regardless of whether
 * any data is flowing.  Each beacon generates one CSI sample.  This gives
 * you a guaranteed ~10 samples/sec baseline even during complete idle.
 *
 * When an application (YouTube, browsing) becomes active, DATA frames appear
 * between the beacons at rates of 20-40/sec.  The CSI changes.  The
 * difference between "beacon-only" CSI and "beacon + data" CSI is the ISAC
 * sensing signal.
 *
 * CSV columns (one row per frame)
 * --------------------------------
 *  type         Always "CSI"
 *  mac          Transmitter MAC address (who sent this frame)
 *  frame_type   beacon | probe_resp | data | qos_data | null_data |
 *               qos_null | ack | rts | cts | other_mgmt | other_ctrl | other
 *  to_ds        1 = frame is going toward the AP (uplink)
 *  from_ds      1 = frame is coming from the AP (downlink)
 *  sig_mode     0 = legacy 802.11b/g   1 = HT 802.11n   3 = VHT 802.11ac
 *  mcs          Modulation and Coding Scheme (0-7 for HT)
 *  bandwidth    0 = 20 MHz   1 = 40 MHz
 *  sig_len      Frame length in bytes including FCS
 *               ~1460 during YouTube/download, ~112 for beacons, ~28 for null-data
 *  rssi         Received signal strength (dBm)
 *  noise_floor  RF noise floor (units of 0.25 dBm)
 *  ampdu_cnt    Number of MPDUs in this A-MPDU burst (0 when not aggregated)
 *  channel      Primary channel the frame arrived on
 *  timestamp    ESP32 steady clock (microseconds since boot)
 *  retry        1 = this is a retransmission (bad channel / congestion)
 *  tid          QoS Traffic ID (4/5=Video, 0/3=BestEffort, 6/7=Voice, -1=no QoS)
 *  seq_num      802.11 sequence number (0-4095); gap = lost frame
 *  CSI_DATA     Raw complex coefficients [im0 re0 im1 re1 ...]
 *
 * How to build and flash
 * ----------------------
 *  cd passive_csi
 *  idf.py menuconfig          # set channel + bandwidth under "Passive CSI Config"
 *  idf.py flash monitor
 *
 * How to collect data
 *  idf.py monitor | grep "^CSI," > experiment.csv
 *
 * How to analyse
 *  python3 ../python_utils/csi_analyzer.py experiment.csv
 *  python3 ../python_utils/csi_analyzer.py experiment.csv --plot
 *  python3 ../python_utils/csi_analyzer.py experiment.csv --timeline
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sstream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

// ---------------------------------------------------------------------------
// Config from menuconfig
// ---------------------------------------------------------------------------

#ifdef CONFIG_CSI_CHANNEL
#define LISTEN_CHANNEL CONFIG_CSI_CHANNEL
#else
#define LISTEN_CHANNEL 6
#endif

#ifdef CONFIG_CSI_CHANNEL_BW
#define CHANNEL_BW CONFIG_CSI_CHANNEL_BW
#else
#define CHANNEL_BW 1   // HT40-above by default
#endif

#ifdef CONFIG_CSI_LLTF_ONLY
#define LLTF_ONLY CONFIG_CSI_LLTF_ONLY
#else
#define LLTF_ONLY 1
#endif

// ---------------------------------------------------------------------------
// 802.11 frame header parsing
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t frame_ctrl;
    uint16_t duration;
    uint8_t  addr1[6];
    uint8_t  addr2[6];
    uint8_t  addr3[6];
    uint16_t seq_ctrl;
} __attribute__((packed)) dot11_hdr_t;

#define FC_TYPE(fc)      (((fc) >> 2)  & 0x3)
#define FC_SUBTYPE(fc)   (((fc) >> 4)  & 0xF)
#define FC_TO_DS(fc)     (((fc) >> 8)  & 0x1)
#define FC_FROM_DS(fc)   (((fc) >> 9)  & 0x1)
#define FC_RETRY(fc)     (((fc) >> 11) & 0x1)

#define TYPE_MGMT  0
#define TYPE_CTRL  1
#define TYPE_DATA  2

#define IS_QOS_DATA(subtype)  ((subtype) & 0x8)  // bit 3 set = QoS data subtype
#define QOS_TID(qc)           ((qc) & 0xF)

// Returns a short string label for the frame type/subtype.
static const char* frame_label(uint8_t type, uint8_t subtype) {
    if (type == TYPE_MGMT) {
        switch (subtype) {
            case 8:  return "beacon";
            case 5:  return "probe_resp";
            case 0:  return "assoc_req";
            case 1:  return "assoc_resp";
            case 4:  return "probe_req";
            case 11: return "auth";
            case 12: return "deauth";
            case 10: return "disassoc";
            default: return "other_mgmt";
        }
    }
    if (type == TYPE_CTRL) {
        switch (subtype) {
            case 13: return "ack";
            case 11: return "rts";
            case 12: return "cts";
            case 8:  return "block_ack";
            case 9:  return "block_ack_req";
            default: return "other_ctrl";
        }
    }
    if (type == TYPE_DATA) {
        if (subtype == 0)  return "data";
        if (subtype == 4)  return "null_data";
        if (subtype == 8)  return "qos_data";
        if (subtype == 12) return "qos_null";
        return "other_data";
    }
    return "other";
}

// ---------------------------------------------------------------------------
// Per-MAC frame header cache
// (promiscuous callback runs first and caches header fields;
//  CSI callback reads from the cache)
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  mac[6];
    bool     valid;
    uint8_t  frame_type;
    uint8_t  frame_subtype;
    uint8_t  to_ds;
    uint8_t  from_ds;
    uint8_t  retry;
    uint16_t seq_num;
    int8_t   tid;        // -1 if not a QoS frame
} frame_cache_t;

#define CACHE_SIZE 16
static frame_cache_t s_cache[CACHE_SIZE];
static int           s_cache_count = 0;
static SemaphoreHandle_t s_cache_mutex;

static frame_cache_t* cache_get_or_create(const uint8_t mac[6]) {
    for (int i = 0; i < s_cache_count; i++) {
        if (memcmp(s_cache[i].mac, mac, 6) == 0) return &s_cache[i];
    }
    if (s_cache_count < CACHE_SIZE) {
        frame_cache_t *e = &s_cache[s_cache_count++];
        memcpy(e->mac, mac, 6);
        e->valid = false;
        return e;
    }
    // Evict oldest entry (slot 0)
    memmove(&s_cache[0], &s_cache[1], sizeof(frame_cache_t) * (CACHE_SIZE - 1));
    frame_cache_t *e = &s_cache[CACHE_SIZE - 1];
    memcpy(e->mac, mac, 6);
    e->valid = false;
    return e;
}

static const frame_cache_t* cache_find(const uint8_t mac[6]) {
    for (int i = 0; i < s_cache_count; i++) {
        if (s_cache[i].valid && memcmp(s_cache[i].mac, mac, 6) == 0)
            return &s_cache[i];
    }
    return NULL;
}

// ---------------------------------------------------------------------------
// Promiscuous RX callback — runs for every received frame, parses MAC header
// ---------------------------------------------------------------------------

static void promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *) buf;
    const uint8_t *payload = pkt->payload;
    uint16_t len = pkt->rx_ctrl.sig_len;

    if (len < 10) return;

    const dot11_hdr_t *hdr = (const dot11_hdr_t *) payload;
    uint16_t fc      = hdr->frame_ctrl;
    uint8_t  ftype   = FC_TYPE(fc);
    uint8_t  fsubtype = FC_SUBTYPE(fc);

    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);

    frame_cache_t *e = cache_get_or_create(hdr->addr2);
    e->valid        = true;
    e->frame_type   = ftype;
    e->frame_subtype = fsubtype;
    e->to_ds        = FC_TO_DS(fc);
    e->from_ds      = FC_FROM_DS(fc);
    e->retry        = FC_RETRY(fc);
    e->tid          = -1;

    if (len >= sizeof(dot11_hdr_t)) {
        e->seq_num = (hdr->seq_ctrl >> 4) & 0xFFF;
    } else {
        e->seq_num = 0;
    }

    // QoS Control field immediately follows the 24-byte base header
    if (ftype == TYPE_DATA && IS_QOS_DATA(fsubtype) && len >= 26) {
        uint16_t qos = *((const uint16_t *)(payload + 24));
        e->tid = QOS_TID(qos);
    }

    xSemaphoreGive(s_cache_mutex);
}

// ---------------------------------------------------------------------------
// CSI callback — fires for every frame that produces a CSI estimate
// ---------------------------------------------------------------------------

static SemaphoreHandle_t s_csi_mutex;

static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    wifi_csi_info_t d = info[0];

    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    const frame_cache_t *fc = cache_find(d.mac);

    // Copy what we need before releasing the cache mutex
    const char *flabel  = "unknown";
    int  to_ds   = -1, from_ds = -1, retry = -1, tid = -1, seq = -1;
    if (fc) {
        flabel  = frame_label(fc->frame_type, fc->frame_subtype);
        to_ds   = fc->to_ds;
        from_ds = fc->from_ds;
        retry   = fc->retry;
        tid     = fc->tid;
        seq     = fc->seq_num;
    }
    xSemaphoreGive(s_cache_mutex);

    xSemaphoreTake(s_csi_mutex, portMAX_DELAY);

    char mac[20];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             d.mac[0], d.mac[1], d.mac[2], d.mac[3], d.mac[4], d.mac[5]);

    // Steady-clock timestamp in microseconds
    int64_t ts_us = (int64_t) d.rx_ctrl.timestamp;

#if LLTF_ONLY
    int csi_len = (d.len >= 128) ? 128 : d.len;
#else
    int csi_len = d.len;
#endif

    // Print CSV row
    // Columns: type,mac,frame_type,to_ds,from_ds,sig_mode,mcs,bandwidth,
    //          sig_len,rssi,noise_floor,ampdu_cnt,channel,timestamp_us,
    //          retry,tid,seq_num,CSI_DATA
    printf("CSI,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lld,%d,%d,%d,[",
           mac,
           flabel,
           to_ds,
           from_ds,
           (int) d.rx_ctrl.sig_mode,
           (int) d.rx_ctrl.mcs,
           (int) d.rx_ctrl.cwb,
           (int) d.rx_ctrl.sig_len,
           (int) d.rx_ctrl.rssi,
           (int) d.rx_ctrl.noise_floor,
           (int) d.rx_ctrl.ampdu_cnt,
           (int) d.rx_ctrl.channel,
           (long long) ts_us,
           retry,
           tid,
           seq);

    int8_t *buf = d.buf;
    for (int i = 0; i < csi_len; i++) {
        printf("%d ", (int) buf[i]);
    }
    printf("]\n");
    fflush(stdout);

    vTaskDelay(0);   // yield so other tasks (serial TX) can run
    xSemaphoreGive(s_csi_mutex);
}

// ---------------------------------------------------------------------------
// Print CSV header
// ---------------------------------------------------------------------------

static void print_header(void) {
    printf("type,mac,frame_type,to_ds,from_ds,sig_mode,mcs,bandwidth,"
           "sig_len,rssi,noise_floor,ampdu_cnt,channel,timestamp_us,"
           "retry,tid,seq_num,CSI_DATA\n");
    fflush(stdout);
}

// ---------------------------------------------------------------------------
// Wi-Fi init
// ---------------------------------------------------------------------------

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Accept data frames (all sub-types) AND management frames (beacons etc.)
    // Beacons are the key addition: they generate CSI at ~10 Hz even when
    // no application data is flowing, giving a stable idle baseline.
    const wifi_promiscuous_filter_t filt = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA   |
                       WIFI_PROMIS_FILTER_MASK_DATA_MPDU  |
                       WIFI_PROMIS_FILTER_MASK_DATA_AMPDU |
                       WIFI_PROMIS_FILTER_MASK_MGMT
    };
    esp_wifi_set_promiscuous_rx_cb(promiscuous_cb);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_filter(&filt);

    // Set channel and bandwidth
    wifi_second_chan_t second;
#if CHANNEL_BW == 2
    second = WIFI_SECOND_CHAN_BELOW;
#elif CHANNEL_BW == 1
    second = WIFI_SECOND_CHAN_ABOVE;
#else
    second = WIFI_SECOND_CHAN_NONE;
#endif
    ESP_ERROR_CHECK(esp_wifi_set_channel(LISTEN_CHANNEL, second));

    printf("passive_csi: channel=%d  bw=%s  lltf_only=%d\n",
           LISTEN_CHANNEL,
           second == WIFI_SECOND_CHAN_NONE  ? "HT20"       :
           second == WIFI_SECOND_CHAN_ABOVE ? "HT40-above" : "HT40-below",
           LLTF_ONLY);
}

// ---------------------------------------------------------------------------
// CSI init
// ---------------------------------------------------------------------------

static void csi_init(void) {
    ESP_ERROR_CHECK(esp_wifi_set_csi(1));

    wifi_csi_config_t csi_cfg = {};
    csi_cfg.lltf_en          = 1;
    csi_cfg.htltf_en         = 1;
    csi_cfg.stbc_htltf2_en   = 1;
    csi_cfg.ltf_merge_en     = 1;
    csi_cfg.channel_filter_en = 0;  // keep raw per-subcarrier data
    csi_cfg.manu_scale       = 0;

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

extern "C" void app_main(void) {
    // NVS required by Wi-Fi driver
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_cache_mutex = xSemaphoreCreateMutex();
    s_csi_mutex   = xSemaphoreCreateMutex();

    wifi_init();
    csi_init();
    print_header();

    // The Wi-Fi driver callbacks run in the Wi-Fi task.
    // Main task just keeps the firmware alive.
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

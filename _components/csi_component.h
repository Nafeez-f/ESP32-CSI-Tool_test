#ifndef ESP32_CSI_CSI_COMPONENT_H
#define ESP32_CSI_CSI_COMPONENT_H

#include "time_component.h"
#include "frame_header_component.h"
#include "math.h"
#include <sstream>
#include <iostream>
#include <string.h>

char *project_type;

#define CSI_RAW 1
#define CSI_AMPLITUDE 0
#define CSI_PHASE 0

#define CSI_TYPE CSI_RAW

SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

// ---- ISAC: physical environment label ------------------------------------
// Optionally set via serial command "TAG: <label>" (max 31 chars).
// This labels the PHYSICAL ENVIRONMENT, not the traffic type.
// Examples: "person_present", "empty_room", "walking", "sitting"
// Traffic type (YouTube vs idle vs browsing) is derived automatically from
// frame metadata — no human intervention needed. See comm_class below.
static char isac_env_label[32] = {0};

void isac_set_activity_label(const char *label) {
    strncpy(isac_env_label, label, sizeof(isac_env_label) - 1);
    isac_env_label[sizeof(isac_env_label) - 1] = '\0';
    printf("Environment label set to: '%s'\n", isac_env_label);
}

// ---- ISAC: management-frame output control --------------------------------
// Beacons, probe req/resp fire the CSI callback every ~100ms per AP on the
// channel.  They are labelled comm_class="mgmt" so you can always tell them
// apart from real data traffic.
// Default: INCLUDED — gives you continuous CSI even during idle periods.
// If output bandwidth is a problem, send "HIDEMGMT" to suppress them.
static bool isac_show_mgmt = true;

// ---- ISAC: automatic communication class ---------------------------------
// pkt_type: from frame_header_consume_pkt_type() (-1 = unknown)
// tid:      QoS TID from frame-header cache (-1 = cache miss)
// sig_len:  from rx_ctrl
//
// Rules (applied in priority order):
//   "mgmt"       — management frame (beacon, probe, etc.)
//   "video"      — QoS TID 4 or 5  (Video access category)
//                  OR sig_len > 800 bytes with TID unknown
//   "voice"      — QoS TID 6 or 7  (Voice access category)
//   "background" — QoS TID 1 or 2  (Background: cloud sync, OS updates)
//   "browsing"   — QoS TID 0 or 3 with sig_len > 200 bytes
//   "idle"       — sig_len <= 100 bytes (only keepalives, null data)
//   "data"       — everything else (unclassified Best Effort)
static const char* _comm_class(int pkt_type, int tid, int sig_len) {
    if (pkt_type == WIFI_PKT_MGMT)                    return "mgmt";
    if (tid == 6 || tid == 7)                          return "voice";
    if (tid == 4 || tid == 5)                          return "video";
    if (tid == 1 || tid == 2)                          return "background";
    if (sig_len > 800)                                 return "video";
    if ((tid == 0 || tid == 3) && sig_len > 200)       return "browsing";
    if (sig_len <= 100)                                return "idle";
    return "data";
}

// ---- ISAC: MAC address filter helpers ------------------------------------
// Up to 4 MAC addresses can be watched at once (covers PC + router pair, etc.)
#define ISAC_MAX_WATCH_MACS 4
static uint8_t isac_watch_macs[ISAC_MAX_WATCH_MACS][6];
static int     isac_watch_mac_count = 0;
static bool    isac_mac_filter_enabled = false;

static bool _parse_mac(const char *str, uint8_t out[6]) {
    unsigned int b[6];
    if (sscanf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        if (sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
                   &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
            return false;
        }
    }
    for (int i = 0; i < 6; i++) out[i] = (uint8_t) b[i];
    return true;
}

// Call with a colon-separated MAC string, e.g. "AA:BB:CC:DD:EE:FF".
// Returns false if the MAC could not be parsed or the watch list is full.
bool isac_add_watch_mac(const char *mac_str) {
    if (isac_watch_mac_count >= ISAC_MAX_WATCH_MACS) return false;
    uint8_t parsed[6];
    if (!_parse_mac(mac_str, parsed)) return false;
    memcpy(isac_watch_macs[isac_watch_mac_count], parsed, 6);
    isac_watch_mac_count++;
    isac_mac_filter_enabled = (isac_watch_mac_count > 0);
    printf("ISAC watching MAC #%d: %s\n", isac_watch_mac_count, mac_str);
    return true;
}

void isac_clear_watch_macs() {
    isac_watch_mac_count = 0;
    isac_mac_filter_enabled = false;
    printf("ISAC MAC watch list cleared\n");
}

static bool _mac_is_watched(const uint8_t mac[6]) {
    if (!isac_mac_filter_enabled) return true;
    for (int i = 0; i < isac_watch_mac_count; i++) {
        if (memcmp(isac_watch_macs[i], mac, 6) == 0) return true;
    }
    return false;
}

// ---- CSI callback --------------------------------------------------------

void _wifi_csi_cb(void *ctx, wifi_csi_info_t *data) {
    wifi_csi_info_t d = data[0];

    // Drop frames from MACs we are not interested in (ISAC filter)
    if (!_mac_is_watched(d.mac)) return;

    // Correlate with the promiscuous callback to get frame type.
    int pkt_type = frame_header_consume_pkt_type(d.mac);

    // Suppress management frames by default (beacons, probes, etc.).
    // They generate CSI but carry no application data — they overwhelm the
    // output and mask the communication CSI you actually want for ISAC.
    if (!isac_show_mgmt && pkt_type == WIFI_PKT_MGMT) return;

    xSemaphoreTake(mutex, portMAX_DELAY);
    std::stringstream ss;

    char mac[20] = {0};
    sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X", d.mac[0], d.mac[1], d.mac[2], d.mac[3], d.mac[4], d.mac[5]);

    const mac_frame_cache_t *fh = frame_header_get(d.mac);
    int  fh_retry    = fh ? fh->retry    : -1;
    int  fh_seq_num  = fh ? fh->seq_num  : -1;
    int  fh_to_ds    = fh ? fh->to_ds    : -1;
    int  fh_from_ds  = fh ? fh->from_ds  : -1;
    int  fh_tid      = fh ? fh->tid      : -1;
    int  fh_is_qos   = fh ? fh->is_qos   : -1;
    int  fh_duration = fh ? fh->duration : -1;

    const char *comm_class = _comm_class(pkt_type, fh_tid, d.rx_ctrl.sig_len);

    ss << "CSI_DATA,"
       << project_type << ","
       << mac << ","
       // https://github.com/espressif/esp-idf/blob/9d0ca60398481a44861542638cfdc1949bb6f312/components/esp_wifi/include/esp_wifi_types.h#L314
       << d.rx_ctrl.rssi << ","
       << d.rx_ctrl.rate << ","
       << d.rx_ctrl.sig_mode << ","
       << d.rx_ctrl.mcs << ","
       << d.rx_ctrl.cwb << ","
       << d.rx_ctrl.smoothing << ","
       << d.rx_ctrl.not_sounding << ","
       << d.rx_ctrl.aggregation << ","
       << d.rx_ctrl.stbc << ","
       << d.rx_ctrl.fec_coding << ","
       << d.rx_ctrl.sgi << ","
       << d.rx_ctrl.noise_floor << ","
       << d.rx_ctrl.ampdu_cnt << ","
       << d.rx_ctrl.channel << ","
       << d.rx_ctrl.secondary_channel << ","
       << d.rx_ctrl.timestamp << ","
       << d.rx_ctrl.ant << ","
       << d.rx_ctrl.sig_len << ","
       << d.rx_ctrl.rx_state << ","
       << real_time_set << ","
       << get_steady_clock_timestamp() << ","
       << data->len << ","
       // 802.11 MAC header fields (-1 = cache miss on very first frames)
       << fh_seq_num  << ","   // sequence number (gap = lost frame)
       << fh_retry    << ","   // 1=retransmission
       << fh_to_ds    << ","   // 1=uplink (device→router)
       << fh_from_ds  << ","   // 1=downlink (router→device)
       << fh_tid      << ","   // QoS TID: 4/5=Video, 0/3=BestEffort, 6/7=Voice
       << fh_is_qos   << ","   // 1=QoS data frame
       << fh_duration << ","   // NAV channel reservation (µs)
       // AUTO: communication class derived from metadata — no commands needed
       << comm_class << ","
       // OPTIONAL: physical environment label set by TAG: command
       // Use this for: "person_present", "empty_room", "walking", etc.
       // Leave it empty if you only care about comm_class
       << isac_env_label << ",[";

#if CONFIG_SHOULD_COLLECT_ONLY_LLTF
    int data_len = 128;
#else
    int data_len = data->len;
#endif

int8_t *my_ptr;
#if CSI_RAW
    my_ptr = data->buf;
    for (int i = 0; i < data_len; i++) {
        ss << (int) my_ptr[i] << " ";
    }
#endif
#if CSI_AMPLITUDE
    my_ptr = data->buf;
    for (int i = 0; i < data_len / 2; i++) {
        ss << (int) sqrt(pow(my_ptr[i * 2], 2) + pow(my_ptr[(i * 2) + 1], 2)) << " ";
    }
#endif
#if CSI_PHASE
    my_ptr = data->buf;
    for (int i = 0; i < data_len / 2; i++) {
        ss << (int) atan2(my_ptr[i*2], my_ptr[(i*2)+1]) << " ";
    }
#endif
    ss << "]\n";

    printf(ss.str().c_str());
    fflush(stdout);
    vTaskDelay(0);
    xSemaphoreGive(mutex);
}

void _print_csi_csv_header() {
    // comm_class: automatic traffic classification from frame metadata (no commands needed)
    //   video / voice / browsing / background / idle / data
    // env_label: optional physical environment label set by TAG: command
    //   e.g. person_present / empty_room / walking — empty string if unused
    char *header_str = (char *) "type,role,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,real_time_set,real_timestamp,len,seq_num,retry,to_ds,from_ds,tid,is_qos,duration,comm_class,env_label,CSI_DATA\n";
    outprintf(header_str);
}

void csi_init(char *type) {
    project_type = type;

#ifdef CONFIG_SHOULD_COLLECT_CSI
    // Register the MAC frame header cache callback BEFORE enabling CSI so
    // the first CSI callback already has header data available.
    frame_header_init();

    ESP_ERROR_CHECK(esp_wifi_set_csi(1));

    // @See: https://github.com/espressif/esp-idf/blob/master/components/esp_wifi/include/esp_wifi_types.h#L401
    wifi_csi_config_t configuration_csi;
    configuration_csi.lltf_en = 1;
    configuration_csi.htltf_en = 1;
    configuration_csi.stbc_htltf2_en = 1;
    configuration_csi.ltf_merge_en = 1;
    configuration_csi.channel_filter_en = 0;
    configuration_csi.manu_scale = 0;

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&configuration_csi));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&_wifi_csi_cb, NULL));

    _print_csi_csv_header();

    // Load compile-time MAC filter list from Kconfig (passive mode ISAC)
#ifdef CONFIG_ISAC_WATCH_MAC_1
    if (strlen(CONFIG_ISAC_WATCH_MAC_1) > 0) isac_add_watch_mac(CONFIG_ISAC_WATCH_MAC_1);
#endif
#ifdef CONFIG_ISAC_WATCH_MAC_2
    if (strlen(CONFIG_ISAC_WATCH_MAC_2) > 0) isac_add_watch_mac(CONFIG_ISAC_WATCH_MAC_2);
#endif
#ifdef CONFIG_ISAC_WATCH_MAC_3
    if (strlen(CONFIG_ISAC_WATCH_MAC_3) > 0) isac_add_watch_mac(CONFIG_ISAC_WATCH_MAC_3);
#endif
#ifdef CONFIG_ISAC_WATCH_MAC_4
    if (strlen(CONFIG_ISAC_WATCH_MAC_4) > 0) isac_add_watch_mac(CONFIG_ISAC_WATCH_MAC_4);
#endif
#endif
}

#endif //ESP32_CSI_CSI_COMPONENT_H

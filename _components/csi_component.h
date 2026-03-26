#ifndef ESP32_CSI_CSI_COMPONENT_H
#define ESP32_CSI_CSI_COMPONENT_H

#include "time_component.h"
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

// ---- ISAC: runtime activity label ----------------------------------------
// Set via serial command "TAG: <label>" (max 31 chars). Empty = unlabelled.
static char isac_activity_label[32] = {0};

void isac_set_activity_label(const char *label) {
    strncpy(isac_activity_label, label, sizeof(isac_activity_label) - 1);
    isac_activity_label[sizeof(isac_activity_label) - 1] = '\0';
    printf("ISAC activity label set to: '%s'\n", isac_activity_label);
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

    xSemaphoreTake(mutex, portMAX_DELAY);
    std::stringstream ss;

    char mac[20] = {0};
    sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X", d.mac[0], d.mac[1], d.mac[2], d.mac[3], d.mac[4], d.mac[5]);

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
       // ISAC: activity label for this sample (empty string if unlabelled)
       << isac_activity_label << ",[";

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
    char *header_str = (char *) "type,role,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,real_time_set,real_timestamp,len,activity,CSI_DATA\n";
    outprintf(header_str);
}

void csi_init(char *type) {
    project_type = type;

#ifdef CONFIG_SHOULD_COLLECT_CSI
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

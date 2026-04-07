#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "../../_components/nvs_component.h"
#include "../../_components/sd_component.h"
#include "../../_components/frame_header_component.h"
#include "../../_components/csi_component.h"
#include "../../_components/time_component.h"
#include "../../_components/input_component.h"

#ifdef CONFIG_WIFI_CHANNEL
#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL
#else
#define WIFI_CHANNEL 6
#endif

#ifdef CONFIG_SHOULD_COLLECT_CSI
#define SHOULD_COLLECT_CSI 1
#else
#define SHOULD_COLLECT_CSI 0
#endif

#ifdef CONFIG_SHOULD_COLLECT_ONLY_LLTF
#define SHOULD_COLLECT_ONLY_LLTF 1
#else
#define SHOULD_COLLECT_ONLY_LLTF 0
#endif

#ifdef CONFIG_SEND_CSI_TO_SERIAL
#define SEND_CSI_TO_SERIAL 1
#else
#define SEND_CSI_TO_SERIAL 0
#endif

#ifdef CONFIG_SEND_CSI_TO_SD
#define SEND_CSI_TO_SD 1
#else
#define SEND_CSI_TO_SD 0
#endif

void config_print() {
    printf("\n\n\n\n\n\n\n\n");
    printf("-----------------------\n");
    printf("ESP32 CSI Tool Settings\n");
    printf("-----------------------\n");
    printf("PROJECT_NAME: %s\n", "PASSIVE");
    printf("CONFIG_ESPTOOLPY_MONITOR_BAUD: %d\n", CONFIG_ESPTOOLPY_MONITOR_BAUD);
    printf("CONFIG_ESP_CONSOLE_UART_BAUDRATE: %d\n", CONFIG_ESP_CONSOLE_UART_BAUDRATE);
    printf("IDF_VER: %s\n", IDF_VER);
    printf("-----------------------\n");
    printf("WIFI_CHANNEL: %d\n", WIFI_CHANNEL);
    printf("SHOULD_COLLECT_CSI: %d\n", SHOULD_COLLECT_CSI);
    printf("SHOULD_COLLECT_ONLY_LLTF: %d\n", SHOULD_COLLECT_ONLY_LLTF);
    printf("SEND_CSI_TO_SERIAL: %d\n", SEND_CSI_TO_SERIAL);
    printf("SEND_CSI_TO_SD: %d\n", SEND_CSI_TO_SD);
    printf("-----------------------\n");
    printf("ISAC MODE\n");
    printf("  Compile-time MAC filter:\n");
#ifdef CONFIG_ISAC_WATCH_MAC_1
    if (strlen(CONFIG_ISAC_WATCH_MAC_1) > 0) printf("    MAC_1: %s\n", CONFIG_ISAC_WATCH_MAC_1);
#endif
#ifdef CONFIG_ISAC_WATCH_MAC_2
    if (strlen(CONFIG_ISAC_WATCH_MAC_2) > 0) printf("    MAC_2: %s\n", CONFIG_ISAC_WATCH_MAC_2);
#endif
#ifdef CONFIG_ISAC_WATCH_MAC_3
    if (strlen(CONFIG_ISAC_WATCH_MAC_3) > 0) printf("    MAC_3: %s\n", CONFIG_ISAC_WATCH_MAC_3);
#endif
#ifdef CONFIG_ISAC_WATCH_MAC_4
    if (strlen(CONFIG_ISAC_WATCH_MAC_4) > 0) printf("    MAC_4: %s\n", CONFIG_ISAC_WATCH_MAC_4);
#endif
    printf("  Management frames (beacons, probes) are INCLUDED and labelled mgmt.\n");
    printf("  This gives continuous CSI even when no data is flowing.\n");
    printf("  Use HIDEMGMT to suppress them if output is too fast.\n");
    printf("\n");
    printf("  Runtime commands:\n");
    printf("    SCAN              - find your router's channel automatically\n");
    printf("    CHANNEL: <n>      - switch to channel n (1-13) without reflashing\n");
    printf("    BANDWIDTH: <mode> - 20 | 40above | 40below (default: 40above)\n");
    printf("                        use 40above/40below if hotspot MAC is missing\n");
    printf("    WATCHMAC: <mac>   - filter CSI to this MAC (e.g. your router BSSID)\n");
    printf("    CLEARMAC          - remove all MAC filters\n");
    printf("    TAG: <label>      - label subsequent CSI rows\n");
    printf("    SETTIME: <unix>   - set real-time clock\n");
    printf("    SHOWMGMT          - include management frame CSI (default)\n");
    printf("    HIDEMGMT          - suppress management frame CSI\n");
    printf("    LISTMACS          - show all MACs seen so far with frame counts\n");
    printf("-----------------------\n");
    printf("\n\n\n\n\n\n\n\n");
}

void passive_init() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Include MGMT frames in the promiscuous filter so the frame-header
    // callback can identify them.  The CSI callback then classifies them as
    // "mgmt" and suppresses them by default (SHOWMGMT to re-enable).
    // DATA_MPDU and DATA_AMPDU are needed for 802.11n/ac aggregated traffic
    // (YouTube, video calls, etc.).
    const wifi_promiscuous_filter_t filt = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA |
                           WIFI_PROMIS_FILTER_MASK_DATA_MPDU |
                           WIFI_PROMIS_FILTER_MASK_DATA_AMPDU |
                           WIFI_PROMIS_FILTER_MASK_MGMT
    };

    int curChannel = WIFI_CHANNEL;

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_filter(&filt);
    // WIFI_SECOND_CHAN_NONE (HT20) misses 802.11n HT40 frames from modern APs/hotspots.
    // iPhone hotspot and most 802.11n APs negotiate HT40 with capable clients.
    // HT40 data frames span the primary + secondary channel simultaneously.
    // Use WIFI_SECOND_CHAN_ABOVE to capture 40MHz frames where secondary is above
    // the primary (e.g. primary=ch6, secondary=ch10 — the most common arrangement).
    // If the hotspot MAC still does not appear, try WIFI_SECOND_CHAN_BELOW instead,
    // or use the runtime CHANNEL: command to switch and observe.
    esp_wifi_set_channel(curChannel, WIFI_SECOND_CHAN_ABOVE);
}

extern "C" void app_main(void) {
    config_print();
    nvs_init();
    sd_init();
    passive_init();
    csi_init((char *) "PASSIVE");
    input_loop();
}

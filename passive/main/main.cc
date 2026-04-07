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

#ifdef CONFIG_ISAC_WIFI_SSID
#define ISAC_WIFI_SSID CONFIG_ISAC_WIFI_SSID
#else
#define ISAC_WIFI_SSID ""
#endif

#ifdef CONFIG_ISAC_WIFI_PASSWORD
#define ISAC_WIFI_PASSWORD CONFIG_ISAC_WIFI_PASSWORD
#else
#define ISAC_WIFI_PASSWORD ""
#endif

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static void _wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("ISAC: WiFi disconnected, reconnecting...\n");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        printf("ISAC: Connected! IP=" IPSTR "\n", IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

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
    if (strlen(ISAC_WIFI_SSID) > 0) {
        printf("  STA+Promiscuous: joining '%s' for HT capability\n", ISAC_WIFI_SSID);
    } else {
        printf("  Pure promiscuous (WIFI_MODE_NULL)\n");
        printf("  WARNING: HT/VHT frames will NOT produce CSI!\n");
        printf("  Set SSID/password in menuconfig for full ISAC capture.\n");
    }
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
    printf("-----------------------\n");
    printf("\n\n\n\n\n\n\n\n");
}

void passive_init() {
    const wifi_promiscuous_filter_t filt = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA |
                           WIFI_PROMIS_FILTER_MASK_DATA_MPDU |
                           WIFI_PROMIS_FILTER_MASK_DATA_AMPDU |
                           WIFI_PROMIS_FILTER_MASK_MGMT
    };

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (strlen(ISAC_WIFI_SSID) > 0) {
        // STA + promiscuous: connect to the network so the radio negotiates
        // HT capabilities. Promiscuous mode on top captures CSI from ALL
        // frames on the channel — including those between other devices
        // (MacBook <-> router). This is the only way to get CSI from HT/VHT
        // frames in passive/ISAC mode.
        s_wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        esp_event_handler_instance_t inst_any, inst_ip;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &_wifi_event_handler, NULL, &inst_any));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &_wifi_event_handler, NULL, &inst_ip));

        wifi_config_t wifi_config = {};
        strlcpy((char *)wifi_config.sta.ssid, ISAC_WIFI_SSID, sizeof(wifi_config.sta.ssid));
        strlcpy((char *)wifi_config.sta.password, ISAC_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
        wifi_config.sta.channel = WIFI_CHANNEL;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        esp_wifi_set_ps(WIFI_PS_NONE);

        printf("ISAC: Connecting to '%s' on channel %d...\n", ISAC_WIFI_SSID, WIFI_CHANNEL);
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                            pdFALSE, pdFALSE, 10000 / portTICK_PERIOD_MS);

        esp_wifi_set_promiscuous(true);
        esp_wifi_set_promiscuous_filter(&filt);

        printf("ISAC: STA+Promiscuous active. CSI from ALL frames on channel.\n");
    } else {
        // Pure promiscuous (original behaviour). Only legacy frames produce CSI.
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
        ESP_ERROR_CHECK(esp_wifi_start());

        esp_wifi_set_promiscuous(true);
        esp_wifi_set_promiscuous_filter(&filt);
        esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_ABOVE);

        printf("ISAC: Pure promiscuous on channel %d (no STA — HT frames won't produce CSI)\n",
               WIFI_CHANNEL);
    }
}

extern "C" void app_main(void) {
    config_print();
    nvs_init();
    sd_init();
    passive_init();
    csi_init((char *) "PASSIVE");
    input_loop();
}

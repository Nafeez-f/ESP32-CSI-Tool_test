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
    printf("ISAC passive mode on channel %d\n", WIFI_CHANNEL);
    printf("  comm_class labels each CSI row automatically:\n");
    printf("    video/voice/browsing/data/idle/mgmt\n");
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
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    const wifi_promiscuous_filter_t filt = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA |
                           WIFI_PROMIS_FILTER_MASK_DATA_MPDU |
                           WIFI_PROMIS_FILTER_MASK_DATA_AMPDU |
                           WIFI_PROMIS_FILTER_MASK_MGMT
    };

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_ABOVE);
}

extern "C" void app_main(void) {
    config_print();
    nvs_init();
    sd_init();
    passive_init();
    csi_init((char *) "PASSIVE");
    input_loop();
}

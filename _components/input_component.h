#ifndef ESP32_CSI_INPUT_COMPONENT_H
#define ESP32_CSI_INPUT_COMPONENT_H

#include "csi_component.h"
#include "esp_wifi.h"

char input_buffer[256];
int input_buffer_pointer = 0;

// Returns true and fills `out` if `buf` starts with `prefix`.
static bool _starts_with(const char *buf, const char *prefix, const char **out) {
    size_t plen = strlen(prefix);
    if (strncmp(buf, prefix, plen) == 0) {
        if (out) *out = buf + plen;
        return true;
    }
    return false;
}

// ---- Runtime channel switching -------------------------------------------

static volatile int _scan_frame_count = 0;

// Temporary CSI callback used during SCAN that just counts frames
static void _scan_csi_cb(void *ctx, wifi_csi_info_t *data) {
    _scan_frame_count++;
}

void _set_channel(int ch) {
    if (ch < 1 || ch > 13) {
        printf("CHANNEL: invalid value %d (must be 1-13 for 2.4 GHz)\n", ch);
        return;
    }
    esp_err_t err = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    if (err == ESP_OK) {
        printf("CHANNEL: now listening on channel %d\n", ch);
    } else {
        printf("CHANNEL: failed to set channel %d (err 0x%x)\n", ch, err);
    }
}

// Scan all 2.4 GHz channels for 2 seconds each and print frame counts.
// This lets you find your router's channel without a separate tool.
void _do_channel_scan() {
    printf("\nSCAN: cycling channels 1-13, 2 seconds each...\n");
    printf("SCAN: the channel with the most frames is your router's channel.\n");
    printf("SCAN: (CSI rows are suppressed during scan)\n\n");

    // Swap in the counting callback
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&_scan_csi_cb, NULL));

    int best_ch = 1, best_count = 0;
    for (int ch = 1; ch <= 13; ch++) {
        _scan_frame_count = 0;
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        int cnt = _scan_frame_count;
        printf("SCAN: ch %2d  →  %3d frames/2s %s\n",
               ch, cnt, cnt > best_count ? "  ← best so far" : "");
        if (cnt > best_count) { best_count = cnt; best_ch = ch; }
    }

    printf("\nSCAN: done. Best channel: %d (%d frames)\n", best_ch, best_count);
    printf("SCAN: run  CHANNEL: %d  to lock onto it.\n\n", best_ch);

    // Restore the real CSI callback and stay on the best channel
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&_wifi_csi_cb, NULL));
    _set_channel(best_ch);
}

// ---- Command dispatcher --------------------------------------------------

void _handle_input() {
    const char *rest = nullptr;

    if (match_set_timestamp_template(input_buffer)) {
        // SETTIME: <unix_seconds>.<usec>
        printf("Setting local time to %s\n", input_buffer);
        time_set(input_buffer);

    } else if (_starts_with(input_buffer, "TAG: ", &rest)) {
        // TAG: <label>   — annotate subsequent CSI rows with this activity label.
        // Send "TAG: " (with nothing after the space) to clear the label.
        isac_set_activity_label(rest);

    } else if (_starts_with(input_buffer, "WATCHMAC: ", &rest)) {
        // WATCHMAC: AA:BB:CC:DD:EE:FF  — add a MAC to the filter list.
        if (!isac_add_watch_mac(rest)) {
            printf("ISAC: failed to add MAC '%s' (bad format or list full)\n", rest);
        }

    } else if (strcmp(input_buffer, "CLEARMAC") == 0) {
        // CLEARMAC  — remove all MAC filters; revert to logging every sender.
        isac_clear_watch_macs();

    } else if (_starts_with(input_buffer, "CHANNEL: ", &rest)) {
        // CHANNEL: <n>  — switch to 2.4 GHz channel n (1-13) without reflashing.
        int ch = atoi(rest);
        _set_channel(ch);

    } else if (strcmp(input_buffer, "SCAN") == 0) {
        // SCAN  — cycle channels 1-13, 2s each, print frame counts.
        // Use this to discover your router's channel automatically.
        _do_channel_scan();

    } else {
        printf("Unknown command: '%s'\n", input_buffer);
        printf("Commands: SETTIME | TAG | WATCHMAC | CLEARMAC | CHANNEL | SCAN\n");
    }
}

void input_check() {
    uint8_t ch = fgetc(stdin);

    while (ch != 0xFF) {
        if (ch == '\n') {
            _handle_input();
            input_buffer[0] = '\0';
            input_buffer_pointer = 0;
        } else {
            input_buffer[input_buffer_pointer] = ch;
            input_buffer[input_buffer_pointer + 1] = '\0';
            input_buffer_pointer++;
        }

        ch = fgetc(stdin);
    }
}

void input_loop() {
    while (true) {
        input_check();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

#endif //ESP32_CSI_INPUT_COMPONENT_H

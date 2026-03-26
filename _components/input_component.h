#ifndef ESP32_CSI_INPUT_COMPONENT_H
#define ESP32_CSI_INPUT_COMPONENT_H

#include "csi_component.h"

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
        // Only CSI from watched MACs will be logged.  First WATCHMAC enables filtering.
        if (!isac_add_watch_mac(rest)) {
            printf("ISAC: failed to add MAC '%s' (bad format or list full)\n", rest);
        }

    } else if (strcmp(input_buffer, "CLEARMAC") == 0) {
        // CLEARMAC  — remove all MAC filters; revert to logging every sender.
        isac_clear_watch_macs();

    } else {
        printf("Unable to handle input '%s'\n", input_buffer);
        printf("Known commands: SETTIME, TAG, WATCHMAC, CLEARMAC\n");
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

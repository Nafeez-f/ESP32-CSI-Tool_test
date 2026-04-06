#ifndef ESP32_CSI_FRAME_HEADER_COMPONENT_H
#define ESP32_CSI_FRAME_HEADER_COMPONENT_H

/*
 * frame_header_component.h
 *
 * Registers a promiscuous RX callback that parses the 802.11 MAC frame header
 * for every DATA frame received.  The parsed fields are stored in a small
 * per-sender-MAC cache.  The CSI callback reads from this cache so that each
 * CSI_DATA row includes the 802.11 MAC-layer fields that are NOT available in
 * wifi_pkt_rx_ctrl_t / wifi_csi_info_t alone.
 *
 * Fields extracted here (all sourced from the raw frame header):
 *
 *   retry     — Retransmission flag.  1 = the sender had to retransmit this
 *               frame because the previous attempt was not ACK'd.
 *               High retry rate indicates a bad channel, congestion, or
 *               heavy interference — the channel is struggling to carry data.
 *               For ISAC: sudden spikes in retry often correlate with a
 *               physical obstruction (body blocking line-of-sight).
 *
 *   seq_num   — 802.11 sequence number (0–4095, wraps).
 *               Consecutive CSI rows should have consecutive seq_nums.
 *               A gap of N means N frames were lost and you have N missing CSI
 *               samples — important for time-series integrity in sensing.
 *
 *   to_ds     — "To Distribution System" bit.  1 = uplink (MacBook → router).
 *   from_ds   — "From Distribution System" bit. 1 = downlink (router → MacBook).
 *               Together these identify frame direction without needing to
 *               compare MAC addresses manually.
 *
 *   tid       — QoS Traffic Identifier (0–7).  Maps to access categories:
 *                 0, 3 = Best Effort (web browsing, general TCP)
 *                 1, 2 = Background (file sync, OS updates)
 *                 4, 5 = Video (YouTube, video calls)  ← most useful for ISAC
 *                 6, 7 = Voice (VoIP, FaceTime)
 *               Only valid when is_qos=1 (802.11n/ac data traffic almost always
 *               uses QoS frames; legacy 802.11b/g uses TID=0).
 *               For ISAC: filter to TID 4/5 to isolate video traffic frames.
 *
 *   duration  — Network Allocation Vector in microseconds.  The sender
 *               announces how long it intends to hold the channel.
 *               Proportional to frame size + round-trip ACK overhead.
 *               Can be used as a per-frame throughput proxy independent of sig_len.
 *
 * Implementation note:
 *   Both the promiscuous RX callback and the CSI callback run in the Wi-Fi
 *   driver task context (same FreeRTOS task, non-preemptively).  They cannot
 *   truly interleave, so the cache does not need a mutex.  If you port this to
 *   a multi-core build with separate task pinning, add a spinlock.
 */

#include <stdint.h>
#include <string.h>
#include "esp_wifi.h"

// ---- 802.11 MAC frame header layout (IEEE 802.11-2020, Section 9.3) -------

typedef struct {
    uint16_t frame_ctrl;
    uint16_t duration;
    uint8_t  addr1[6];   // Receiver / Destination
    uint8_t  addr2[6];   // Transmitter / Source  (= mac in CSI row)
    uint8_t  addr3[6];   // BSSID (infrastructure) or SA/DA
    uint16_t seq_ctrl;
    // QoS Control (2 bytes) follows immediately for QoS data subtypes
} __attribute__((packed)) ieee80211_mac_hdr_t;

// Frame Control bit extraction (ESP32 = little-endian → direct bit indexing)
#define FC_TYPE(fc)       (((fc) >>  2) & 0x3)
#define FC_SUBTYPE(fc)    (((fc) >>  4) & 0xF)
#define FC_TO_DS(fc)      (((fc) >>  8) & 0x1)
#define FC_FROM_DS(fc)    (((fc) >>  9) & 0x1)
#define FC_RETRY(fc)      (((fc) >> 11) & 0x1)
#define FC_PROTECTED(fc)  (((fc) >> 14) & 0x1)

#define FC_TYPE_DATA   2
// Any subtype with bit 3 set is a QoS data subtype
#define IS_QOS_SUBTYPE(st)  ((st) & 0x8)

// Sequence Control field
#define SEQCTL_SEQ_NUM(sc)  (((sc) >> 4) & 0xFFF)
#define SEQCTL_FRAG_NUM(sc) ((sc) & 0xF)

// QoS Control field — TID is the lowest 4 bits
#define QOS_TID(qc)  ((qc) & 0xF)

// ---- Per-MAC frame header cache -------------------------------------------

typedef struct {
    uint8_t  mac[6];
    bool     valid;
    uint16_t duration;      // NAV in microseconds
    uint8_t  retry;         // retransmission flag
    uint8_t  to_ds;
    uint8_t  from_ds;
    uint16_t seq_num;       // 0–4095
    uint8_t  tid;           // QoS TID (0 if not a QoS frame)
    uint8_t  is_qos;        // 1 if QoS data subtype
} mac_frame_cache_t;

#define MAC_CACHE_SIZE 8
static mac_frame_cache_t _mac_frame_cache[MAC_CACHE_SIZE];
static int _mac_frame_cache_count = 0;

static mac_frame_cache_t* _cache_find_or_create(const uint8_t mac[6]) {
    for (int i = 0; i < _mac_frame_cache_count; i++) {
        if (memcmp(_mac_frame_cache[i].mac, mac, 6) == 0)
            return &_mac_frame_cache[i];
    }
    if (_mac_frame_cache_count < MAC_CACHE_SIZE) {
        mac_frame_cache_t* e = &_mac_frame_cache[_mac_frame_cache_count++];
        memcpy(e->mac, mac, 6);
        e->valid = false;
        return e;
    }
    // Cache full: evict least-recently-updated (slot 0), shift left
    memmove(&_mac_frame_cache[0], &_mac_frame_cache[1],
            sizeof(mac_frame_cache_t) * (MAC_CACHE_SIZE - 1));
    mac_frame_cache_t* e = &_mac_frame_cache[MAC_CACHE_SIZE - 1];
    memcpy(e->mac, mac, 6);
    e->valid = false;
    return e;
}

// Returns the cached entry for this MAC, or NULL if not seen yet.
const mac_frame_cache_t* frame_header_get(const uint8_t mac[6]) {
    for (int i = 0; i < _mac_frame_cache_count; i++) {
        if (_mac_frame_cache[i].valid &&
            memcmp(_mac_frame_cache[i].mac, mac, 6) == 0)
            return &_mac_frame_cache[i];
    }
    return NULL;
}

// ---- Promiscuous RX callback ----------------------------------------------

static void _frame_header_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_DATA) return;

    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *) buf;
    const uint8_t *payload = pkt->payload;
    uint16_t pkt_len = pkt->rx_ctrl.sig_len;

    // Need at least 24 bytes for the basic 802.11 MAC header
    if (pkt_len < 24) return;

    const ieee80211_mac_hdr_t *hdr = (const ieee80211_mac_hdr_t *) payload;
    uint16_t fc = hdr->frame_ctrl;

    if (FC_TYPE(fc) != FC_TYPE_DATA) return;

    uint8_t subtype = FC_SUBTYPE(fc);
    mac_frame_cache_t *cache = _cache_find_or_create(hdr->addr2);

    cache->valid     = true;
    cache->duration  = hdr->duration;
    cache->retry     = FC_RETRY(fc);
    cache->to_ds     = FC_TO_DS(fc);
    cache->from_ds   = FC_FROM_DS(fc);
    cache->seq_num   = SEQCTL_SEQ_NUM(hdr->seq_ctrl);
    cache->is_qos    = IS_QOS_SUBTYPE(subtype) ? 1 : 0;
    cache->tid       = 0;

    if (cache->is_qos && pkt_len >= 26) {
        uint16_t qos_ctrl = *((const uint16_t *)(payload + 24));
        cache->tid = QOS_TID(qos_ctrl);
    }
}

// Call once after esp_wifi_start() and esp_wifi_set_promiscuous(true).
void frame_header_init() {
    memset(_mac_frame_cache, 0, sizeof(_mac_frame_cache));
    _mac_frame_cache_count = 0;
    esp_wifi_set_promiscuous_rx_cb(_frame_header_promiscuous_cb);
}

#endif // ESP32_CSI_FRAME_HEADER_COMPONENT_H

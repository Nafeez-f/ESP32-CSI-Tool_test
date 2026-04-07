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

// ---- Per-MAC discovery stats (for LISTMACS command) -----------------------
// Tracks every unique sender MAC seen on the channel with counts and RSSI,
// so the user can identify their hotspot BSSID without external tools.

#define MAC_STATS_SIZE 32

typedef struct {
    uint8_t  mac[6];
    bool     used;
    uint32_t data_frames;
    uint32_t mgmt_frames;
    uint32_t ht_frames;       // sig_mode > 0 (HT/VHT — real traffic)
    int32_t  rssi_sum;
    uint32_t rssi_count;
    uint8_t  last_to_ds;
    uint8_t  last_from_ds;
    uint32_t max_sig_len;
} mac_stats_entry_t;

static mac_stats_entry_t _mac_stats[MAC_STATS_SIZE];
static int _mac_stats_count = 0;

static mac_stats_entry_t* _mac_stats_find_or_create(const uint8_t mac[6]) {
    for (int i = 0; i < _mac_stats_count; i++) {
        if (_mac_stats[i].used && memcmp(_mac_stats[i].mac, mac, 6) == 0)
            return &_mac_stats[i];
    }
    if (_mac_stats_count < MAC_STATS_SIZE) {
        mac_stats_entry_t *e = &_mac_stats[_mac_stats_count++];
        memset(e, 0, sizeof(*e));
        memcpy(e->mac, mac, 6);
        e->used = true;
        return e;
    }
    return NULL;
}

void frame_header_print_mac_stats() {
    printf("\n=== MACs seen on this channel ===\n");
    printf("%-19s %6s %6s %5s %5s %6s %s\n",
           "MAC", "Data", "Mgmt", "HT", "RSSI", "MaxLen", "Direction");
    printf("-------------------------------------------------------------------\n");

    uint32_t total_ht = 0;
    for (int i = 0; i < _mac_stats_count; i++) {
        mac_stats_entry_t *e = &_mac_stats[i];
        if (!e->used) continue;
        total_ht += e->ht_frames;
        int avg_rssi = e->rssi_count > 0 ? (int)(e->rssi_sum / (int32_t)e->rssi_count) : 0;
        const char *dir = "?";
        if (e->last_to_ds && !e->last_from_ds) dir = "uplink";
        else if (!e->last_to_ds && e->last_from_ds) dir = "downlink";
        else if (!e->last_to_ds && !e->last_from_ds) dir = "AP/mgmt";
        printf("%02X:%02X:%02X:%02X:%02X:%02X %6lu %6lu %5lu %5d %6lu  %s\n",
               e->mac[0], e->mac[1], e->mac[2],
               e->mac[3], e->mac[4], e->mac[5],
               (unsigned long)e->data_frames,
               (unsigned long)e->mgmt_frames,
               (unsigned long)e->ht_frames,
               avg_rssi,
               (unsigned long)e->max_sig_len,
               dir);
    }
    printf("-------------------------------------------------------------------\n");

    if (total_ht == 0) {
        printf("WARNING: No HT/VHT frames seen (HT column all zeros)!\n");
        printf("  This means you are only capturing legacy-rate frames (beacons,\n");
        printf("  keepalives). Your hotspot's data traffic is likely using HT40.\n");
        printf("  Try:  BANDWIDTH: 40above  or  BANDWIDTH: 40below\n");
        printf("  Or run SCAN which now tests all bandwidth modes automatically.\n\n");
    } else {
        printf("HT column = 802.11n/ac frames (the real data traffic).\n");
        printf("Your hotspot: strong RSSI + high HT count + data+mgmt frames.\n");
    }
    printf("Use  WATCHMAC: <mac>  to filter to your hotspot + laptop.\n\n");
}

// ---- Single-frame context for CSI callback correlation ---------------------
// The promiscuous RX callback runs BEFORE the CSI callback for every received
// frame (both execute in the Wi-Fi driver task, sequentially).  This struct
// passes the 802.11 frame type so the CSI callback can distinguish data from
// management frames — critical for ISAC because beacons generate valid CSI but
// carry no application data and would otherwise flood the output.

typedef struct {
    uint8_t  mac[6];
    wifi_promiscuous_pkt_type_t pkt_type;  // WIFI_PKT_MGMT, WIFI_PKT_DATA, …
    bool     fresh;   // set by promisc CB, cleared when consumed by CSI CB
} _frame_ctx_t;

static _frame_ctx_t _last_frame_ctx = {{0}, (wifi_promiscuous_pkt_type_t)0, false};

// Called by CSI callback.  Returns the wifi_promiscuous_pkt_type_t if the
// promiscuous callback just processed a frame from this MAC.
// Returns -1 if no correlation is available (frame type not in filter).
static int frame_header_consume_pkt_type(const uint8_t mac[6]) {
    if (_last_frame_ctx.fresh && memcmp(_last_frame_ctx.mac, mac, 6) == 0) {
        _last_frame_ctx.fresh = false;
        return (int)_last_frame_ctx.pkt_type;
    }
    return -1;
}

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
    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *) buf;
    const uint8_t *payload = pkt->payload;
    uint16_t pkt_len = pkt->rx_ctrl.sig_len;

    if (pkt_len < 24) return;

    const ieee80211_mac_hdr_t *hdr = (const ieee80211_mac_hdr_t *) payload;
    uint16_t fc = hdr->frame_ctrl;

    // Record frame type + sender MAC so the CSI callback can correlate.
    memcpy(_last_frame_ctx.mac, hdr->addr2, 6);
    _last_frame_ctx.pkt_type = type;
    _last_frame_ctx.fresh    = true;

    // Update per-MAC discovery stats (for LISTMACS command)
    mac_stats_entry_t *st = _mac_stats_find_or_create(hdr->addr2);
    if (st) {
        if (type == WIFI_PKT_MGMT) st->mgmt_frames++;
        else                       st->data_frames++;
        if (pkt->rx_ctrl.sig_mode > 0) st->ht_frames++;
        st->rssi_sum += pkt->rx_ctrl.rssi;
        st->rssi_count++;
        if (pkt_len > st->max_sig_len) st->max_sig_len = pkt_len;
        st->last_to_ds   = FC_TO_DS(fc);
        st->last_from_ds = FC_FROM_DS(fc);
    }

    // Only fill the per-MAC detail cache for DATA frames.
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

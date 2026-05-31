#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "packet.h"
#include "protocol.h"
#include "store.h"
#include "radio_driver.h"


#define SLOT_PROCESSING_US      (88U)
#define PRE_P2_SLOT_PROCESSING_US (40U)
#define FAST_RAMPUP             (1U)

#if FAST_RAMPUP
#define RADIO_RAMPUP_US         (40U)
#else
#define RADIO_RAMPUP_US         (140U)
#endif

#define TX_CHAIN_DELAY           (10U)  //delay between tx event address and rx event address

#define SLOT_PHY_OVERHEAD_BYTES (8U)
#ifndef USE_PRE_P2
#define USE_PRE_P2             (0U)
#endif
#ifndef APP_DATA_LEN
#define APP_DATA_LEN           (0U)
#endif
#define LOCAL_PAYLOAD_META_LEN (6U)
#if (APP_DATA_LEN > (TIMECAST_STORE_MAX_DATA_LEN - LOCAL_PAYLOAD_META_LEN))
#error "APP_DATA_LEN exceeds TIMECAST_STORE_MAX_DATA_LEN budget"
#endif
#define LOCAL_PAYLOAD_LEN ((uint8_t)(LOCAL_PAYLOAD_META_LEN + APP_DATA_LEN))
#define LOCAL_P2_PAYLOAD_LEN ((uint8_t)(PACKET_P2_DATA_HDR_LEN + LOCAL_PAYLOAD_LEN))
#define PACKET_AIR_TIME_US(payload_len) \
    (8U * ((uint32_t)(payload_len) + SLOT_PHY_OVERHEAD_BYTES))
#define P2_PAYLOAD_TO_SUBSLOT_US(payload_len) \
    (SLOT_PROCESSING_US + RADIO_RAMPUP_US + \
     PACKET_AIR_TIME_US(payload_len))
#ifndef P1_SLOT_US
#define P1_SLOT_US              \
    (SLOT_PROCESSING_US + RADIO_RAMPUP_US + \
     PACKET_AIR_TIME_US(PACKET_P1_SYNC_PAYLOAD_LEN))
#endif
#ifndef P2_SUBSLOT_US
#define P2_SUBSLOT_US           \
    P2_PAYLOAD_TO_SUBSLOT_US(LOCAL_P2_PAYLOAD_LEN)
#endif
#ifndef PRE_P2_SUBSLOT_US
#define PRE_P2_SUBSLOT_US       \
    (PRE_P2_SLOT_PROCESSING_US + RADIO_RAMPUP_US + \
     PACKET_AIR_TIME_US(PACKET_PRE_P2_CTRL_LEN))
#endif

#ifndef LOCAL_NODE_ID
#define LOCAL_NODE_ID (NODE_ID)
#endif



#ifndef MASTER_START_DELAY_US
#define MASTER_START_DELAY_US   (2U * P1_SLOT_US)
#endif
#ifndef ROUND_GAP_US
#define ROUND_GAP_US            (200U * P1_SLOT_US)
#endif
#ifndef NTX
#define NTX                     (16U)
#endif
#if (NTX > 63U)
#error "NTX exceeds 7-bit packed relay_cnt budget"
#endif
#ifndef P2_NODE_COUNT
#define P2_NODE_COUNT           (4U)
#endif
#ifndef P2_START_GUARD_US
#define P2_START_GUARD_US       (P1_SLOT_US)
#endif
#ifndef PRE_P2_SUBSLOT_GUARD_US
#define PRE_P2_SUBSLOT_GUARD_US (40U)
#endif
#ifndef P2_SUBSLOT_GUARD_US
#define P2_SUBSLOT_GUARD_US     (88U)
#endif
#ifndef P2_RX_WINDOW_US
#define P2_RX_WINDOW_US         (RADIO_RAMPUP_US + 50U)
#endif
#ifndef P2_RX_WINDOW_MARGIN_US
#define P2_RX_WINDOW_MARGIN_US  (24U)
#endif
#ifndef P1_RX_TS_TO_SLOT_START_US
#define P1_RX_TS_TO_SLOT_START_US (40U)

#endif
#ifndef P1_RX_LEAD_US
#define P1_RX_LEAD_US           (80U)
#endif
#ifndef P2_RX_LEAD_US
#define P2_RX_LEAD_US           (40U)
#endif
#ifndef TX_MIN_ARM_LEAD_US
#define TX_MIN_ARM_LEAD_US      (8U)
#endif
#ifndef P1_SCAN_LOG_INTERVAL_US
#define P1_SCAN_LOG_INTERVAL_US (1000000U)
#endif
#ifndef MASTER_P2_INCOMPLETE_PRE_THRESHOLD
#define MASTER_P2_INCOMPLETE_PRE_THRESHOLD (3U)
#endif

static uint8_t rx_buffer[255] = {0};

#define P1_SLOT_TICKS         US_TO_TIMER_TICKS(P1_SLOT_US)
#define PRE_P2_SUBSLOT_TICKS  US_TO_TIMER_TICKS(PRE_P2_SUBSLOT_US)
#define P2_SUBSLOT_TICKS      US_TO_TIMER_TICKS(P2_SUBSLOT_US)
#define P1_SLOT_ACTIVE_US     (P1_SLOT_US - SLOT_PROCESSING_US)
#define PRE_P2_SUBSLOT_PERIOD_US (PRE_P2_SUBSLOT_US + PRE_P2_SUBSLOT_GUARD_US)
#define P2_SUBSLOT_PERIOD_US  (P2_SUBSLOT_US + P2_SUBSLOT_GUARD_US)
#define P1_SYNC_DURATION_TICKS ((uint32_t)(2U * NTX) * P1_SLOT_TICKS)
#define P1_SLOT_ACTIVE_TICKS  US_TO_TIMER_TICKS(P1_SLOT_ACTIVE_US)
#define PRE_P2_SLOT_PROCESSING_TICKS US_TO_TIMER_TICKS(PRE_P2_SLOT_PROCESSING_US)
#define SLOT_PROCESSING_TICKS US_TO_TIMER_TICKS(SLOT_PROCESSING_US)
#define PRE_P2_SUBSLOT_PERIOD_TICKS US_TO_TIMER_TICKS(PRE_P2_SUBSLOT_PERIOD_US)
#define P2_SUBSLOT_PERIOD_TICKS US_TO_TIMER_TICKS(P2_SUBSLOT_PERIOD_US)
#define CLASS_COUNT      (16U)
#define CLASS_MAX_ID     (CLASS_COUNT - 1U)
#define CLASS_INVALID_ID (CLASS_COUNT)
#define PACKED_CLASS_LEN(node_count) (((uint32_t)(node_count) + 1U) / 2U)
#ifndef TEST_NODE2_CLASS_PERIOD
#define TEST_NODE2_CLASS_PERIOD (0U)
#endif
#ifndef TEST_NODE2_CLASS_MIN
#define TEST_NODE2_CLASS_MIN    (1U)
#endif
#ifndef TEST_NODE2_CLASS_MAX
#define TEST_NODE2_CLASS_MAX    (CLASS_MAX_ID)
#endif
#if (TEST_NODE2_CLASS_MIN == 0U)
#error "TEST_NODE2_CLASS_MIN must fit the local payload metadata"
#endif
#if (TEST_NODE2_CLASS_MAX > CLASS_MAX_ID)
#error "TEST_NODE2_CLASS_MAX exceeds class range"
#endif
#if (TEST_NODE2_CLASS_MIN > TEST_NODE2_CLASS_MAX)
#error "TEST_NODE2_CLASS_MIN exceeds TEST_NODE2_CLASS_MAX"
#endif
static timecast_store_t g_store;
static timecast_protocol_state_t g_proto;
static uint32_t g_round_count;
static uint32_t g_round_p2_start_ticks;
static uint8_t g_committed_class[TIMECAST_STORE_MAX_NODES];
static uint8_t g_local_desired_class;
static uint8_t g_local_committed_payload_len;
static uint8_t g_local_committed_payload[TIMECAST_STORE_MAX_DATA_LEN];
/* Update control state:
 * - g_update_pending_latched survives across rounds until a committed schedule
 *   catches up with the local desired class.
 * - g_round_tx_update_req is frozen after P1, so P2 packet flags stay stable
 *   for the whole round.
 */
static bool g_update_pending_latched;
static bool g_round_tx_update_req;
static bool g_round_run_pre;
static bool g_master_run_pre_next_round;
static bool g_master_force_initial_pre = true;
static uint32_t g_master_p2_incomplete_rounds;
static uint32_t g_round_pre_commit_slot_ticks;

typedef struct {
    uint32_t addr_timeout;
    uint32_t addr_timeout_by_subslot[TIMECAST_STORE_MAX_NODES];
    uint32_t end_timeout;
    uint32_t crc_fail;
    uint32_t accepted;
    uint32_t rejected;
    uint8_t last_slot;
    uint8_t last_subslot;
    uint8_t last_owner;
    uint8_t last_status;
    int32_t last_addr_offset_ticks;
} p2_rx_diag_t;

static p2_rx_diag_t g_p2_rx_diag;

typedef struct {
    bool flag_tx;
    bool have_schedule;
    uint8_t slot_idx;
    uint8_t ntx_done;
    uint32_t start_local_ticks;
    uint32_t slot_ticks;
    uint32_t rx_valid;
    uint8_t packed_len;
    uint8_t packed_schedule[PACKET_PRE_COMMIT_MAX_SCHEDULE_LEN];
} pre_commit_state_t;

static pre_commit_state_t g_pre_commit;

static inline bool _is_master(void);

static timecast_protocol_cfg_t g_proto_cfg = {
    .local_node_id = (uint8_t)LOCAL_NODE_ID,
    .ntx = NTX,
    .p2_node_count = P2_NODE_COUNT,
    .glossy_slot_ticks = P1_SLOT_TICKS,
    .p1_rx_ts_to_slot_start_ticks = US_TO_TIMER_TICKS(P1_RX_TS_TO_SLOT_START_US),
    .p1_guard_ticks = US_TO_TIMER_TICKS(P2_START_GUARD_US),
    .pre_p2_subslot_ticks = PRE_P2_SUBSLOT_TICKS,
    .pre_p2_guard_ticks = US_TO_TIMER_TICKS(PRE_P2_SUBSLOT_GUARD_US),
    .pre_p2_rx_window_ticks = US_TO_TIMER_TICKS(P2_RX_WINDOW_US),
    .p2_subslot_ticks = P2_SUBSLOT_TICKS,
    .p2_guard_ticks = US_TO_TIMER_TICKS(P2_SUBSLOT_GUARD_US),
    .p2_rx_window_ticks = US_TO_TIMER_TICKS(P2_RX_WINDOW_US),
    .p2_payload_base_ticks =
        US_TO_TIMER_TICKS(SLOT_PROCESSING_US + RADIO_RAMPUP_US +
                                (8U * SLOT_PHY_OVERHEAD_BYTES)),
    .p2_payload_byte_ticks = US_TO_TIMER_TICKS(8U),
};



static inline uint32_t _p2_duration_ticks(uint8_t node_count)
{
    return ((uint32_t)(2U * NTX) * (uint32_t)node_count * P2_SUBSLOT_PERIOD_TICKS);
}

static inline uint32_t _pre_p2_duration_ticks(uint8_t node_count)
{
    return ((uint32_t)(2U * NTX) * (uint32_t)node_count * PRE_P2_SUBSLOT_PERIOD_TICKS);
}

static inline uint32_t _p2_max_duration_ticks(uint8_t node_count)
{
    return ((uint32_t)(2U * NTX) * (uint32_t)node_count *
            (g_proto_cfg.p2_payload_base_ticks +
             ((uint32_t)PACKET_P2_DATA_MAX_PAYLOAD_LEN *
              g_proto_cfg.p2_payload_byte_ticks) +
             g_proto_cfg.p2_guard_ticks));
}

static inline uint8_t _class_to_payload_len(uint8_t class_id);

static inline uint32_t _p2_duration_from_classes_ticks(const uint8_t *classes, uint8_t node_count)
{
    uint8_t source_id;
    uint32_t slot_ticks = 0U;

    for (source_id = 0U; source_id < node_count; source_id++) {
        uint8_t p2_payload_len = _class_to_payload_len(classes[source_id]);

        slot_ticks += g_proto_cfg.p2_payload_base_ticks +
                      ((uint32_t)p2_payload_len * g_proto_cfg.p2_payload_byte_ticks) +
                      g_proto_cfg.p2_guard_ticks;
    }

    return (uint32_t)(2U * NTX) * slot_ticks;
}

static inline uint32_t _fixed_p2_slot_ticks(void)
{
    return (uint32_t)g_proto_cfg.p2_node_count *
           (g_proto_cfg.p2_subslot_ticks + g_proto_cfg.p2_guard_ticks);
}

static uint32_t _original_p2_duration_ticks(void)
{
    return _p2_duration_ticks(g_proto_cfg.p2_node_count);
}

static uint32_t _pre_commit_slot_ticks(uint8_t node_count)
{
    uint32_t payload_len = PACKET_PRE_COMMIT_BASE_LEN +
                           PACKED_CLASS_LEN(node_count);

    return US_TO_TIMER_TICKS(SLOT_PROCESSING_US +
                                   RADIO_RAMPUP_US +
                                   PACKET_AIR_TIME_US(payload_len));
}

static uint32_t _pre_commit_duration_ticks(uint32_t slot_ticks)
{
    return (uint32_t)(2U * NTX) * slot_ticks;
}

static uint32_t _original_round_period_ticks(void)
{
    uint32_t period_ticks = P1_SYNC_DURATION_TICKS + g_proto_cfg.p1_guard_ticks;

    period_ticks += _original_p2_duration_ticks();

    return period_ticks + US_TO_TIMER_TICKS(ROUND_GAP_US);
}

static uint32_t _improved_round_period_ticks(bool run_pre)
{
    uint32_t period_ticks = P1_SYNC_DURATION_TICKS + g_proto_cfg.p1_guard_ticks;

    if (run_pre) {
        period_ticks += _pre_p2_duration_ticks(g_proto_cfg.p2_node_count);
        period_ticks += g_proto_cfg.p1_guard_ticks;
        period_ticks += _pre_commit_duration_ticks(g_round_pre_commit_slot_ticks);
        period_ticks += g_proto_cfg.p1_guard_ticks;
    }

    period_ticks += (uint32_t)(2U * NTX) * p2_get_slot_ticks(&g_proto, &g_proto_cfg);
    return period_ticks + US_TO_TIMER_TICKS(ROUND_GAP_US);
}

static inline bool _is_master(void)
{
    return (LOCAL_NODE_ID == 0U);
}

static inline uint8_t _class_span_bytes(void)
{
    return (uint8_t)(((PACKET_P2_DATA_MAX_PAYLOAD_LEN -
                       PACKET_P2_DATA_HDR_LEN + 1U) +
                      CLASS_COUNT - 1U) / CLASS_COUNT);
}

static inline uint8_t _class_to_payload_len(uint8_t class_id)
{
    uint32_t payload_len;

    if (class_id > CLASS_MAX_ID) {
        return 0U;
    }
    if (class_id == CLASS_MAX_ID) {
        return PACKET_P2_DATA_MAX_PAYLOAD_LEN;
    }

    payload_len = (uint32_t)PACKET_P2_DATA_HDR_LEN +
                  ((uint32_t)(class_id + 1U) * (uint32_t)_class_span_bytes()) - 1U;

    return (uint8_t)payload_len;
}

static inline uint8_t _payload_len_to_class(uint8_t payload_len)
{
    uint32_t offset;
    uint32_t class_id;

    if (payload_len <= PACKET_P2_DATA_HDR_LEN) {
        return 0U;
    }
    if (payload_len == PACKET_P2_DATA_MAX_PAYLOAD_LEN) {
        return CLASS_MAX_ID;
    }
    if (payload_len > PACKET_P2_DATA_MAX_PAYLOAD_LEN) {
        return CLASS_INVALID_ID;
    }

    offset = (uint32_t)payload_len - (uint32_t)PACKET_P2_DATA_HDR_LEN;
    class_id = offset / (uint32_t)_class_span_bytes();
    if (class_id > CLASS_MAX_ID) {
        return CLASS_INVALID_ID;
    }

    return (uint8_t)class_id;
}


static void _format_class_map(char *dst, size_t dst_len,
                              const uint8_t *classes, bool desired_local_map)
{
    static const char lut[] = "0123456789ABCDEF";
    uint8_t source_id;

    if (!dst || (dst_len == 0U)) {
        return;
    }

    if ((dst_len <= (size_t)g_proto_cfg.p2_node_count) ||
        (!desired_local_map && !classes)) {
        dst[0] = '\0';
        return;
    }

    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        uint8_t class_id;

        if (desired_local_map) {
            if (source_id != (uint8_t)LOCAL_NODE_ID) {
                dst[source_id] = '.';
                continue;
            }
            class_id = g_local_desired_class;
        }
        else {
            class_id = classes[source_id];
        }
        dst[source_id] = (class_id <= CLASS_MAX_ID) ? lut[class_id] : '?';
    }
    dst[g_proto_cfg.p2_node_count] = '\0';
}

static inline const char *_role_name(void)
{
    return _is_master() ? "master" : "follower";
}

static uint8_t _local_payload_len_for_source(uint8_t source_id)
{
    (void)source_id;
#if (TEST_NODE2_CLASS_PERIOD > 0U)
    if (source_id == 2U) {
        uint32_t round_idx = (g_round_count > 0U) ? (g_round_count - 1U) : 0U;
        uint32_t class_span = (uint32_t)TEST_NODE2_CLASS_MAX -
                              (uint32_t)TEST_NODE2_CLASS_MIN + 1U;
        uint8_t class_id = (uint8_t)((uint32_t)TEST_NODE2_CLASS_MIN +
                                     ((round_idx / (uint32_t)TEST_NODE2_CLASS_PERIOD) %
                                      class_span));
        uint8_t p2_payload_len = _class_to_payload_len(class_id);

        return (uint8_t)(p2_payload_len - PACKET_P2_DATA_HDR_LEN);
    }
#endif

    return LOCAL_PAYLOAD_LEN;
}

static uint8_t _build_local_payload_with_len(uint8_t source_id, uint8_t *dst, uint8_t payload_len)
{
    if (!dst) {
        return 0U;
    }

    if ((payload_len < LOCAL_PAYLOAD_META_LEN) ||
        (payload_len > TIMECAST_STORE_MAX_DATA_LEN)) {
        return 0U;
    }

    memset(dst, 0, payload_len);
    dst[0] = source_id;
    dst[1] = _is_master() ? 1U : 0U;
    dst[2] = (uint8_t)(g_round_count & 0xFFU);
    dst[3] = (uint8_t)((g_round_count >> 8) & 0xFFU);
    dst[4] = (uint8_t)((g_round_count >> 16) & 0xFFU);
    dst[5] = (uint8_t)((g_round_count >> 24) & 0xFFU);
    if (payload_len > LOCAL_PAYLOAD_META_LEN) {
        memset(&dst[LOCAL_PAYLOAD_META_LEN], 'c',
               (size_t)(payload_len - LOCAL_PAYLOAD_META_LEN));
    }

    return payload_len;
}

static void _pack_class_schedule(const uint8_t *classes, uint8_t node_count,
                                 uint8_t *packed_out, uint8_t *packed_len_out)
{
    uint8_t source_id;
    uint8_t packed_len = (uint8_t)PACKED_CLASS_LEN(node_count);

    memset(packed_out, 0, packed_len);
    for (source_id = 0U; source_id < node_count; source_id++) {
        uint8_t class_id = classes[source_id];
        uint8_t byte_idx = (uint8_t)(source_id / 2U);

        if ((source_id & 0x1U) == 0U) {
            packed_out[byte_idx] |= class_id;
        }
        else {
            packed_out[byte_idx] |= (uint8_t)(class_id << 4);
        }
    }

    *packed_len_out = packed_len;
}

static void _unpack_class_schedule(const uint8_t *packed, uint8_t node_count, uint8_t *classes_out)
{
    uint8_t source_id;

    if (!packed || !classes_out) {
        return;
    }

    for (source_id = 0U; source_id < node_count; source_id++) {
        uint8_t packed_byte = packed[source_id / 2U];

        if ((source_id & 0x1U) == 0U) {
            classes_out[source_id] = (uint8_t)(packed_byte & 0x0FU);
        }
        else {
            classes_out[source_id] = (uint8_t)((packed_byte >> 4) & 0x0FU);
        }
    }
}

static void _seed_round_schedule_from_committed(void)
{
    uint8_t source_id;

    g_proto.pre_p2.complete = true;
    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        uint8_t p2_payload_len = _class_to_payload_len(g_committed_class[source_id]);

        g_proto.pre_p2.present[source_id] = 1U;
        g_proto.pre_p2.p2_payload_len[source_id] = p2_payload_len;
    }
}

static void _build_schedule_from_pre_collect(uint8_t *classes_out)
{
    uint8_t source_id;

    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        if (pre_p2_has_p2_payload_len(&g_proto, source_id)) {
            uint8_t class_id = _payload_len_to_class(g_proto.pre_p2.p2_payload_len[source_id]);

            classes_out[source_id] = class_id;
        }
        else {
            classes_out[source_id] = CLASS_MAX_ID;
        }
    }
}

static void _apply_round_schedule_to_proto(const uint8_t *classes)
{
    uint8_t source_id;

    g_proto.pre_p2.complete = true;
    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        uint8_t p2_payload_len = _class_to_payload_len(classes[source_id]);

        g_proto.pre_p2.present[source_id] = 1U;
        g_proto.pre_p2.p2_payload_len[source_id] = p2_payload_len;
    }
}

static void _master_request_pre_next_round(void)
{
    if (_is_master()) {
        g_master_run_pre_next_round = true;
    }
}

static void _freeze_round_update_advertisement(void)
{
    g_round_tx_update_req = g_update_pending_latched && !g_round_run_pre;
    if (g_round_tx_update_req) {
        _master_request_pre_next_round();
    }
}

static bool _local_packet_should_request_update(uint8_t owner_id)
{
    return (owner_id == (uint8_t)LOCAL_NODE_ID) && g_round_tx_update_req;
}

static void _commit_schedule(const uint8_t *classes)
{
    uint8_t source_id;
    uint8_t local_source_id = (uint8_t)LOCAL_NODE_ID;
    uint8_t payload[TIMECAST_STORE_MAX_DATA_LEN];
    uint8_t committed_p2_payload_len;
    uint8_t committed_data_len;
    uint8_t desired_payload_len;
    uint8_t payload_len;

    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        g_committed_class[source_id] = classes[source_id];
    }


    committed_p2_payload_len = _class_to_payload_len(classes[local_source_id]);
    committed_data_len = (uint8_t)(committed_p2_payload_len -
                                    PACKET_P2_DATA_HDR_LEN);
    desired_payload_len = _local_payload_len_for_source(local_source_id);
    if ((desired_payload_len <= committed_data_len) ||
        (g_local_committed_payload_len == 0U)) {
        payload_len = (desired_payload_len <= committed_data_len) ?
                        desired_payload_len : LOCAL_PAYLOAD_META_LEN;
        payload_len = _build_local_payload_with_len(local_source_id, payload, payload_len);
        memcpy(g_local_committed_payload, payload, payload_len);
        g_local_committed_payload_len = payload_len;
    }

    g_update_pending_latched = (g_local_desired_class != g_committed_class[LOCAL_NODE_ID]) ? 1U : 0U;
}

static void _commit_round_schedule(const uint8_t *classes)
{
    _commit_schedule(classes);
    _apply_round_schedule_to_proto(classes);
}

static void _refresh_local_payload(void)
{
    uint8_t source_id = (uint8_t)LOCAL_NODE_ID;
    uint8_t payload_len;
    uint8_t payload_class;

    if (source_id >= g_proto_cfg.p2_node_count) {
        return;
    }

    payload_len = _local_payload_len_for_source(source_id);
    payload_class = _payload_len_to_class((uint8_t)(PACKET_P2_DATA_HDR_LEN + payload_len));

    g_local_desired_class = payload_class;
    if (g_local_desired_class != g_committed_class[source_id]) {
        g_update_pending_latched = true;
    }
}

static void _load_local_desired_payload_for_p2(void)
{
    uint8_t source_id = (uint8_t)LOCAL_NODE_ID;
    uint8_t payload[TIMECAST_STORE_MAX_DATA_LEN];
    uint8_t payload_len;

    payload_len = _build_local_payload_with_len(source_id, payload,
                                                _local_payload_len_for_source(source_id));
    store_write(&g_store, source_id, payload, payload_len);
}

static void _load_local_committed_payload_for_p2(void)
{
    uint8_t source_id = (uint8_t)LOCAL_NODE_ID;

    if ((source_id < g_proto_cfg.p2_node_count) && (g_local_committed_payload_len > 0U)) {
        store_write(&g_store, source_id, g_local_committed_payload, g_local_committed_payload_len);
    }
}

static void _wait_out_p2_schedule(uint32_t p2_start_ticks)
{
    uint32_t p2_duration_ticks =
        _p2_duration_from_classes_ticks(g_committed_class, g_proto_cfg.p2_node_count);
    uint32_t p2_end_ticks;

    if (p2_duration_ticks == 0U) {
        p2_duration_ticks = _p2_max_duration_ticks(g_proto_cfg.p2_node_count);
    }

    p2_end_ticks = p2_start_ticks + p2_duration_ticks;

    while ((int32_t)(p2_end_ticks - now_ticks()) > 0) {}
}

static uint32_t _elapsed_since_ticks(uint32_t start_ticks, uint32_t now_tick)
{
    if (start_ticks == 0U) {
        return 0U;
    }

    return now_tick - start_ticks;
}

static void _pre_commit_start(uint32_t start_local_ticks, const uint8_t *classes)
{
    memset(&g_pre_commit, 0, sizeof(g_pre_commit));
    g_pre_commit.start_local_ticks = start_local_ticks;
    g_pre_commit.slot_ticks = _pre_commit_slot_ticks(g_proto_cfg.p2_node_count);
    g_proto.phase = TIMECAST_PHASE_PRE_COMMIT;
    if (!_is_master()) {
        return;
    }

    _pack_class_schedule(classes, g_proto_cfg.p2_node_count,
                         g_pre_commit.packed_schedule, &g_pre_commit.packed_len);
    g_pre_commit.have_schedule = true;
    g_pre_commit.flag_tx = true;
}

static uint32_t _pre_commit_slot_start_ticks(void)
{
    return g_pre_commit.start_local_ticks +
           ((uint32_t)g_pre_commit.slot_idx * g_pre_commit.slot_ticks);
}

static uint32_t _pre_commit_p2_start_ticks(void)
{
    return g_pre_commit.start_local_ticks +
           _pre_commit_duration_ticks(g_pre_commit.slot_ticks) +
           g_proto_cfg.p1_guard_ticks;
}

static void _handle_pre_commit_rx(const uint8_t *buf)
{
    pre_commit_frame_t frame;
    uint8_t packed_schedule[PACKET_PRE_COMMIT_MAX_SCHEDULE_LEN];
    size_t packed_len = 0U;

    decode_pre_commit(buf, g_proto_cfg.p2_node_count, &frame, packed_schedule, &packed_len);


    if (!g_pre_commit.have_schedule) {
        memcpy(g_pre_commit.packed_schedule, packed_schedule, packed_len);
        g_pre_commit.packed_len = (uint8_t)packed_len;
        g_pre_commit.have_schedule = true;
    }
    g_pre_commit.flag_tx = true;
    g_pre_commit.rx_valid++;
}

static void _pre_commit_finish_slot(bool did_tx)
{
    if (did_tx) {
        g_pre_commit.flag_tx = false;
    }
    else if (g_pre_commit.have_schedule) {
        g_pre_commit.flag_tx = true;
    }

    g_pre_commit.slot_idx++;
}

static void _run_pre_commit_slot(void)
{
    uint32_t slot_start_ticks = _pre_commit_slot_start_ticks();
    uint32_t slot_end_ticks = slot_start_ticks + g_pre_commit.slot_ticks;
    uint32_t slot_active_end_ticks = slot_end_ticks - SLOT_PROCESSING_TICKS;
    uint32_t rx_window_end_ticks = slot_start_ticks + g_proto_cfg.p2_rx_window_ticks;
    uint32_t now_tick = now_ticks();
    bool do_tx = g_pre_commit.have_schedule && g_pre_commit.flag_tx &&
                 (g_pre_commit.ntx_done < NTX);

    if ((int32_t)(rx_window_end_ticks - slot_active_end_ticks) > 0) {
        rx_window_end_ticks = slot_active_end_ticks;
    }

    if ((int32_t)(now_tick - slot_start_ticks) >= 0) {
        _pre_commit_finish_slot(do_tx);
        return;
    }

    if (do_tx) {
        pre_commit_frame_t frame;
        uint8_t payload[PACKET_PRE_COMMIT_MAX_PAYLOAD_LEN] = {0};

        
        frame.relay_cnt = g_pre_commit.slot_idx;
        encode_pre_commit(payload, &frame, g_pre_commit.packed_schedule, g_pre_commit.packed_len);

        radio_tx_arm(payload, slot_start_ticks);

        WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, slot_end_ticks);
        
        g_pre_commit.ntx_done++;
        _pre_commit_finish_slot(true);
        return;
    }

    radio_rx_arm(rx_buffer, slot_start_ticks - US_TO_TIMER_TICKS(P2_RX_LEAD_US));
    WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS != 0U, rx_window_end_ticks);
    if (NRF_RADIO->EVENTS_ADDRESS != 0U) {

        WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, slot_active_end_ticks + US_TO_TIMER_TICKS(TX_CHAIN_DELAY));
        if ((NRF_RADIO->EVENTS_END != 0U) && (NRF_RADIO->CRCSTATUS == 1)) {
            _handle_pre_commit_rx(rx_buffer);
        }
    }

    _pre_commit_finish_slot(false);
}

static void _log_rf_stats_if_needed(uint16_t present_count, uint16_t participant_count)
{
    (void)present_count;
    (void)participant_count;
}

static uint32_t _p2_rx_diag_total(void)
{
    return g_p2_rx_diag.addr_timeout +
           g_p2_rx_diag.end_timeout +
           g_p2_rx_diag.crc_fail +
           g_p2_rx_diag.accepted +
           g_p2_rx_diag.rejected;
}

static void _log_p2_rx_diag(void)
{
    if (_p2_rx_diag_total() == 0U) {
        return;
    }

    printf(",p2d={addr=%" PRIu32 ",addrsub=[",
           g_p2_rx_diag.addr_timeout);
    for (uint8_t subslot = 0U; subslot < g_proto_cfg.p2_node_count; subslot++) {
        printf("%s%" PRIu32,
               (subslot == 0U) ? "" : ",",
               g_p2_rx_diag.addr_timeout_by_subslot[subslot]);
    }
    printf("],end=%" PRIu32 ",crc=%" PRIu32
           ",ok=%" PRIu32 ",rej=%" PRIu32 ",last=%u/%u/%u/st%u/ad=%" PRId32 "}",
           g_p2_rx_diag.end_timeout,
           g_p2_rx_diag.crc_fail,
           g_p2_rx_diag.accepted,
           g_p2_rx_diag.rejected,
           (unsigned)g_p2_rx_diag.last_slot,
           (unsigned)g_p2_rx_diag.last_subslot,
           (unsigned)g_p2_rx_diag.last_owner,
           (unsigned)g_p2_rx_diag.last_status,
           g_p2_rx_diag.last_addr_offset_ticks);
}

static void _log_round_summary_original(void)
{
    uint8_t p2_node_count = p2_get_node_list_len(&g_proto);
    uint32_t now_tick = now_ticks();
    uint32_t p2_duration_ticks = _elapsed_since_ticks(g_round_p2_start_ticks, now_tick);
    uint16_t present_count = store_present_count(&g_store);
    uint16_t participant_count = store_participant_count(&g_store);
    uint32_t p2_slot_ticks = _fixed_p2_slot_ticks();
    uint32_t p2_reject_total = g_proto.p2.reject_decode +
                               g_proto.p2.reject_type +
                               g_proto.p2.reject_epoch +
                               g_proto.p2.reject_slot +
                               g_proto.p2.reject_subslot +
                               g_proto.p2.reject_present;
    uint32_t tx_sched_fail_total = g_proto.p1_tx_sched_fails + g_proto.p2_tx_sched_fails;
    bool has_errors = (g_proto.slot_misses > 0U) ||
                      (tx_sched_fail_total > 0U) ||
                      (g_proto.p2.slot_misses > 0U) ||
                      (g_proto.p2.rx_ignored > 0U) ||
                      (p2_reject_total > 0U);

    printf("[timecast] round{id=%u,role=%s,round=%" PRIu32
           ",epoch=%" PRIu32 ",joined=%u,hop=%u,syncslot=%u"
           ",rx=%" PRIu32
           ",p2rx=%" PRIu32 ",p2store=%" PRIu32
           ",present=%u/%u,p2n=%u"
           ",p1slot=%u,p2slot=%" PRIu32 ",p2rdu=%" PRIu32
           ",tref=%" PRIu32 ",p2start=%" PRIu32
           ",tcofs=%" PRIu32,
           (unsigned)LOCAL_NODE_ID,
           _role_name(),
           g_round_count,
           g_proto.current_epoch,
           (unsigned)g_proto.joined,
           (unsigned)g_proto.p1.local_hop,
           (unsigned)g_proto.p1.slot_idx,
           g_proto.rx_valid,
           g_proto.p2.rx_valid,
           g_proto.p2.store_updates,
           (unsigned)present_count,
           (unsigned)participant_count,
           (unsigned)p2_node_count,
           (unsigned)P1_SLOT_US,
           TIMER_TICKS_TO_US(p2_slot_ticks),
           TIMER_TICKS_TO_US(p2_duration_ticks),
           TIMER_TICKS_TO_US(p1_get_tref_local_ticks(&g_proto)),
           TIMER_TICKS_TO_US(g_round_p2_start_ticks),
           TIMER_TICKS_TO_US(g_proto_cfg.p1_rx_ts_to_slot_start_ticks));
    if (has_errors) {
        printf(",err={late=%" PRIu32 ",txf={p1=%" PRIu32 ",p2=%" PRIu32 "}"
               ",p2miss=%" PRIu32 ",p2ign=%" PRIu32 ",p2rej=%" PRIu32 "}",
               g_proto.slot_misses,
               g_proto.p1_tx_sched_fails,
               g_proto.p2_tx_sched_fails,
               g_proto.p2.slot_misses,
               g_proto.p2.rx_ignored,
               p2_reject_total);
        if (p2_reject_total > 0U) {
            printf(",p2r={dec=%" PRIu32 ",type=%" PRIu32
                   ",ep=%" PRIu32 ",slot=%" PRIu32
                   ",sub=%" PRIu32 ",dup=%" PRIu32 "}",
                   g_proto.p2.reject_decode,
                   g_proto.p2.reject_type,
                   g_proto.p2.reject_epoch,
                   g_proto.p2.reject_slot,
                   g_proto.p2.reject_subslot,
                   g_proto.p2.reject_present);
        }
    }
    _log_p2_rx_diag();
    _log_rf_stats_if_needed(present_count, participant_count);
    printf("}\n");
}

static void _log_round_summary_pre_p2(void)
{
    uint8_t p2_node_count = p2_get_node_list_len(&g_proto);
    uint8_t local_pending_updates = (g_local_desired_class != g_committed_class[LOCAL_NODE_ID]) ? 1U : 0U;
    uint32_t now_tick = now_ticks();
    uint32_t p2_duration_ticks = _elapsed_since_ticks(g_round_p2_start_ticks, now_tick);
    uint16_t present_count = store_present_count(&g_store);
    uint16_t participant_count = store_participant_count(&g_store);
    uint32_t p2_slot_ticks = p2_get_slot_ticks(&g_proto, &g_proto_cfg);
    uint32_t pre_p2_reject_total = g_proto.pre_p2.reject_decode;
    uint32_t p2_reject_total = g_proto.p2.reject_decode +
                               g_proto.p2.reject_type +
                               g_proto.p2.reject_epoch +
                               g_proto.p2.reject_slot +
                               g_proto.p2.reject_subslot +
                               g_proto.p2.reject_present;
    uint32_t tx_sched_fail_total = g_proto.p1_tx_sched_fails +
                                   g_proto.pre_p2_tx_sched_fails +
                                   g_proto.p2_tx_sched_fails;
    bool has_errors = (g_proto.slot_misses > 0U) ||
                      (tx_sched_fail_total > 0U) ||
                      (g_proto.pre_p2.slot_misses > 0U) ||
                      (pre_p2_reject_total > 0U) ||
                      (g_proto.p2.slot_misses > 0U) ||
                      (g_proto.p2.rx_ignored > 0U) ||
                      (p2_reject_total > 0U);
    bool log_update_state = g_round_run_pre ||
                            g_update_pending_latched ||
                            (local_pending_updates > 0U) ||
                            g_master_run_pre_next_round ||
                            (g_master_p2_incomplete_rounds > 0U);
    char committed_class_map[TIMECAST_STORE_MAX_NODES + 1];
    char desired_class_map[TIMECAST_STORE_MAX_NODES + 1];

    printf("[timecast] round{id=%u,role=%s,round=%" PRIu32
           ",epoch=%" PRIu32 ",joined=%u,hop=%u,syncslot=%u"
           ",rx=%" PRIu32
           ",pre=%u,upd=%u"
           ",pp2rx=%" PRIu32
           ",p2rx=%" PRIu32 ",p2store=%" PRIu32
           ",present=%u/%u,p2n=%u"
           ",p1slot=%u,pp2sub=%u,p2slot=%" PRIu32 ",p2rdu=%" PRIu32
           ",tref=%" PRIu32 ",p2start=%" PRIu32
           ",tcofs=%" PRIu32,
           (unsigned)LOCAL_NODE_ID,
           _role_name(),
           g_round_count,
           g_proto.current_epoch,
           (unsigned)g_proto.joined,
           (unsigned)g_proto.p1.local_hop,
           (unsigned)g_proto.p1.slot_idx,
           g_proto.rx_valid,
           (unsigned)g_round_run_pre,
           (unsigned)g_update_pending_latched,
           g_proto.pre_p2.rx_valid,
           g_proto.p2.rx_valid,
           g_proto.p2.store_updates,
           (unsigned)present_count,
           (unsigned)participant_count,
           (unsigned)p2_node_count,
           (unsigned)P1_SLOT_US,
           (unsigned)PRE_P2_SUBSLOT_PERIOD_US,
           TIMER_TICKS_TO_US(p2_slot_ticks),
           TIMER_TICKS_TO_US(p2_duration_ticks),
           TIMER_TICKS_TO_US(p1_get_tref_local_ticks(&g_proto)),
           TIMER_TICKS_TO_US(g_round_p2_start_ticks),
           TIMER_TICKS_TO_US(g_proto_cfg.p1_rx_ts_to_slot_start_ticks));
    if (has_errors) {
        printf(",err={late=%" PRIu32 ",txf={p1=%" PRIu32 ",pp2=%" PRIu32 ",p2=%" PRIu32 "}"
               ",pp2miss=%" PRIu32 ",pp2rej=%" PRIu32
               ",p2miss=%" PRIu32 ",p2ign=%" PRIu32 ",p2rej=%" PRIu32 "}",
               g_proto.slot_misses,
               g_proto.p1_tx_sched_fails,
               g_proto.pre_p2_tx_sched_fails,
               g_proto.p2_tx_sched_fails,
               g_proto.pre_p2.slot_misses,
               pre_p2_reject_total,
               g_proto.p2.slot_misses,
               g_proto.p2.rx_ignored,
               p2_reject_total);
        if (pre_p2_reject_total > 0U) {
            printf(",pp2r={dec=%" PRIu32 "}",
                   g_proto.pre_p2.reject_decode);
        }
        if (p2_reject_total > 0U) {
            printf(",p2r={dec=%" PRIu32 ",type=%" PRIu32
                   ",ep=%" PRIu32 ",slot=%" PRIu32
                   ",sub=%" PRIu32 ",dup=%" PRIu32 "}",
                   g_proto.p2.reject_decode,
                   g_proto.p2.reject_type,
                   g_proto.p2.reject_epoch,
                   g_proto.p2.reject_slot,
                   g_proto.p2.reject_subslot,
                   g_proto.p2.reject_present);
        }
    }
    if (log_update_state) {
        _format_class_map(committed_class_map, sizeof(committed_class_map),
                          g_committed_class, false);
        _format_class_map(desired_class_map, sizeof(desired_class_map),
                          NULL, true);
        printf(",u={loc=%u,next=%u,inc=%" PRIu32
               ",pcrx=%" PRIu32 ",pchave=%u,cc=%s,dc=%s}",
               (unsigned)local_pending_updates,
               (unsigned)g_master_run_pre_next_round,
               g_master_p2_incomplete_rounds,
               g_pre_commit.rx_valid,
               (unsigned)g_pre_commit.have_schedule,
               committed_class_map,
               desired_class_map);
    }
    _log_p2_rx_diag();
    _log_rf_stats_if_needed(present_count, participant_count);
    printf("}\n");
}

static void _handle_p1_rx(const uint8_t *buf, uint32_t rx_time)
{
    p1_sync_frame_t frame;

    decode_p1_sync(buf, &frame);
    p1_handle_rx(&g_proto, &frame, rx_time, &g_proto_cfg);
    g_round_run_pre = ((frame.flags & PACKET_P1_FLAG_RUN_PRE) != 0U);
    g_proto.rx_valid++;

}

static void _handle_pre_p2_rx(const uint8_t *buf)
{
    uint8_t class_id;
    uint8_t p2_payload_len;

    decode_pre_p2_ctrl(buf, &class_id);

    p2_payload_len = _class_to_payload_len(class_id);

    pre_p2_handle_rx(&g_proto, p2_payload_len, &g_proto_cfg);
}

static void _handle_p2_rx(const uint8_t *buf)
{
    p2_data_frame_t frame;
    uint8_t data[PACKET_P2_DATA_MAX_DATA_LEN];


    decode_p2_data(buf, &frame, data);


    if (!p2_handle_rx(&g_proto, &g_store, &frame, data)) {
        g_proto.p2.rx_ignored++;
        return;
    }

    if ((frame.flags & PACKET_P2_FLAG_UPDATE_REQ) != 0U) {
        g_update_pending_latched = true;
        _master_request_pre_next_round();
    }
}

static void _scan_until_reference(void)
{
    uint32_t rx_ticks;
    
    
    while (p1_is_active(&g_proto) &&
           !p1_has_tref(&g_proto)) {

            _try_rx_enable(rx_buffer);

           WAIT_UNTIL(NRF_RADIO->EVENTS_ADDRESS!=0, US_TO_TIMER_TICKS(P1_SCAN_LOG_INTERVAL_US));
        if (NRF_RADIO->EVENTS_ADDRESS == 0) {
            printf("[timecast] waiting{id=%u,round=%" PRIu32 "}\n",
                   (unsigned)LOCAL_NODE_ID, g_round_count);
        }else{
            rx_ticks = get_last_address_time_ticks();
            WAIT_UNTIL(NRF_RADIO->EVENTS_END != 0, P1_SLOT_TICKS);
            if(NRF_RADIO->EVENTS_END != 0 && NRF_RADIO->CRCSTATUS == 1){
                _handle_p1_rx(rx_buffer, rx_ticks - US_TO_TIMER_TICKS(TX_CHAIN_DELAY));
            }
        }
    }
}

static void _run_p1_slot(void)
{
    bool do_tx = p1_should_tx(&g_proto);
    uint32_t slot_start_ticks = p1_get_slot_start_local_ticks(&g_proto, &g_proto_cfg);
    uint32_t slot_active_end_ticks = slot_start_ticks + P1_SLOT_ACTIVE_TICKS - RADIO_RAMPUP_TIME_TICKS;
    uint32_t now_tick = now_ticks();

    if ((int32_t)(now_tick - slot_start_ticks + RADIO_RAMPUP_TIME_TICKS) >= 0) {
        g_proto.slot_misses++;
        printf("[timecast] slot miss: slot=%u now=%" PRIu32 " start=%" PRIu32 "\n",
               (unsigned)p1_get_slot_idx(&g_proto),
               now_tick, slot_start_ticks);
        (void)p1_finish_slot(&g_proto, &g_proto_cfg, do_tx);
        return;
    }

    if (do_tx) {
        p1_sync_frame_t frame;
        uint8_t payload[PACKET_P1_SYNC_PAYLOAD_LEN] = {0};

        p1_prepare_tx(&g_proto, &g_proto_cfg, &frame);

        frame.flags = g_round_run_pre ? PACKET_P1_FLAG_RUN_PRE : 0U;
        encode_p1_sync(payload, sizeof(payload), &frame);

        radio_tx_arm(payload, slot_start_ticks - RADIO_RAMPUP_TIME_TICKS);
        WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS!=0, slot_start_ticks + US_TO_TIMER_TICKS(40));
        if (NRF_RADIO->EVENTS_ADDRESS==0) {
            g_proto.p1_tx_sched_fails++;
            uint32_t now_tick = now_ticks();
            int32_t slack_ticks = (int32_t)(slot_start_ticks - now_tick);
            printf("[timecast] TX schedule failed: now=%" PRIu32
                   " deadline=%" PRIu32 " slack=%" PRId32 " ticks fails=%" PRIu32   "\n",
                   now_tick, slot_start_ticks + US_TO_TIMER_TICKS(40), slack_ticks, g_proto.p1_tx_sched_fails);
            p1_finish_slot(&g_proto, &g_proto_cfg, false);
        }else{
            WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END!=0, slot_active_end_ticks);
            p1_finish_slot(&g_proto, &g_proto_cfg, true);
        }


        return;
    }

    radio_rx_arm(rx_buffer, slot_start_ticks - RADIO_RAMPUP_TIME_TICKS );
    WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS!=0, slot_start_ticks + US_TO_TIMER_TICKS(60));
    //printf("start to addr=%" PRId32 "\n", TIMER_TICKS_TO_US(get_last_address_time_ticks() - get_last_ready_time_ticks()));

    WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END!=0, slot_active_end_ticks);
    p1_finish_slot(&g_proto, &g_proto_cfg, false);
}

static void _run_pre_p2_subslot(void)
{
    bool tx_slot = pre_p2_is_tx_slot(&g_proto);
    uint32_t subslot_start_ticks =
        pre_p2_get_subslot_start_local_ticks(&g_proto, &g_proto_cfg);
    uint32_t subslot_end_ticks = subslot_start_ticks + g_proto_cfg.pre_p2_subslot_ticks;
    uint32_t subslot_active_end_ticks = subslot_end_ticks - PRE_P2_SLOT_PROCESSING_TICKS;
    uint32_t rx_window_end_ticks = subslot_start_ticks + g_proto_cfg.pre_p2_rx_window_ticks;
    uint32_t now_tick = now_ticks();
    uint8_t owner_id = g_proto.pre_p2.subslot_idx;

    if ((int32_t)(rx_window_end_ticks - subslot_active_end_ticks) > 0) {
        rx_window_end_ticks = subslot_active_end_ticks;
    }

    if ((int32_t)(now_tick - subslot_start_ticks) >= 0) {
        g_proto.pre_p2.slot_misses++;
        printf("[timecast] pre-p2 miss: slot=%u sub=%u now=%" PRIu32 " deadline=%" PRIu32 "\n",
               (unsigned)pre_p2_get_slot_idx(&g_proto),
               (unsigned)pre_p2_get_subslot_idx(&g_proto),
               now_tick, subslot_start_ticks);
        pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (tx_slot) {
        uint8_t p2_payload_len;
        uint8_t class_id;
        uint8_t payload[PACKET_PRE_P2_CTRL_LEN] = {0};

        if(!g_proto.pre_p2.present[owner_id]){
            WAIT_UNTIL_ABS(0, subslot_active_end_ticks);
            pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }
        p2_payload_len = g_proto.pre_p2.p2_payload_len[owner_id];

        class_id = _payload_len_to_class(p2_payload_len);

        encode_pre_p2_ctrl(payload, class_id);

        radio_tx_arm(payload, subslot_start_ticks);

        WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS != 0U, 
            subslot_start_ticks + RADIO_RAMPUP_TIME_TICKS + US_TO_TIMER_TICKS(40U));
        if (NRF_RADIO->EVENTS_ADDRESS == 0U) {
            g_proto.pre_p2_tx_sched_fails++;
            printf("[timecast] pre-p2 TX schedule failed: slot=%u sub=%u\n",
                   (unsigned)pre_p2_get_slot_idx(&g_proto),
                   (unsigned)pre_p2_get_subslot_idx(&g_proto));
            pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }

        WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, subslot_active_end_ticks);
        pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (pre_p2_is_complete(&g_proto) ||
        pre_p2_has_p2_payload_len(&g_proto, owner_id)) {
        WAIT_UNTIL_ABS(0, subslot_active_end_ticks);
        pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    radio_rx_arm(rx_buffer, subslot_start_ticks - US_TO_TIMER_TICKS(P2_RX_LEAD_US));
    WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS != 0U, rx_window_end_ticks);
    if (NRF_RADIO->EVENTS_ADDRESS != 0U) {

        WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, 
            subslot_active_end_ticks + US_TO_TIMER_TICKS(TX_CHAIN_DELAY));

        if ((NRF_RADIO->EVENTS_END != 0U) && (NRF_RADIO->CRCSTATUS == 1)) {
            _handle_pre_p2_rx(rx_buffer);
        }
    }

    pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
}

static void _run_p2_subslot(void)
{
    bool tx_slot = p2_is_tx_slot(&g_proto);
    uint32_t subslot_ticks = p2_get_subslot_ticks(&g_proto);
    uint32_t subslot_start_ticks = p2_get_subslot_start_local_ticks(&g_proto);
    uint32_t subslot_end_ticks = subslot_start_ticks + subslot_ticks;
    uint32_t subslot_active_end_ticks = subslot_end_ticks - SLOT_PROCESSING_TICKS;
    uint32_t rx_window_end_ticks = subslot_start_ticks + g_proto_cfg.p2_rx_window_ticks;
    uint32_t now_tick = now_ticks();
    uint8_t owner_id = p2_get_owner_node_id(&g_proto);

    if ((int32_t)(rx_window_end_ticks - subslot_active_end_ticks) > 0) {
        rx_window_end_ticks = subslot_active_end_ticks;
    }

    if ((int32_t)(now_tick - subslot_start_ticks) >= 0) {
        g_proto.p2.slot_misses++;
        printf("[timecast] p2 miss: slot=%u sub=%u now=%" PRIu32 " deadline=%" PRIu32 "\n",
               (unsigned)p2_get_slot_idx(&g_proto),
               (unsigned)p2_get_subslot_idx(&g_proto),
               now_tick, subslot_start_ticks);
        (void)p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (tx_slot) {
        p2_data_frame_t frame;
        const uint8_t *data_ptr = NULL;
        uint8_t payload[PACKET_P2_DATA_MAX_PAYLOAD_LEN] = {0};


        if (!p2_prepare_tx(&g_proto, &g_store, &g_proto_cfg, &frame, &data_ptr)) {
            now_tick = now_ticks();
            WAIT_UNTIL(0, subslot_active_end_ticks - now_tick);
            p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }
        frame.flags = _local_packet_should_request_update(owner_id) ?
                      PACKET_P2_FLAG_UPDATE_REQ : 0U;

        encode_p2_data(payload, &frame, data_ptr);

        radio_tx_arm(payload, subslot_start_ticks);

        WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS!=0, subslot_start_ticks + RADIO_RAMPUP_TIME_TICKS + US_TO_TIMER_TICKS(40));
        //printf("addr time=%" PRId32 "\n", TIMER_TICKS_TO_US(get_last_address_time_ticks() - subslot_start_ticks));
        if (NRF_RADIO->EVENTS_ADDRESS == 0) {
            g_proto.p2_tx_sched_fails++;
            printf("[timecast] P2 TX schedule failed: slot=%u sub=%u\n",
                   (unsigned)p2_get_slot_idx(&g_proto),
                   (unsigned)p2_get_subslot_idx(&g_proto));
            p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }else{
            WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END!=0, subslot_active_end_ticks);
        }

        p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (store_has_data(&g_store, owner_id)) {

        WAIT_UNTIL_ABS(false, subslot_active_end_ticks);
        p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    radio_rx_arm(rx_buffer, subslot_start_ticks - US_TO_TIMER_TICKS(P2_RX_LEAD_US));

    WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS!=0, rx_window_end_ticks);
    //printf("addr time=%" PRId32 "\n", TIMER_TICKS_TO_US(get_last_address_time_ticks() - get_last_ready_time_ticks()));
    if(NRF_RADIO->EVENTS_ADDRESS!=0){
        uint32_t address_ticks = get_last_address_time_ticks();

        g_p2_rx_diag.last_slot = p2_get_slot_idx(&g_proto);
        g_p2_rx_diag.last_subslot = p2_get_subslot_idx(&g_proto);
        g_p2_rx_diag.last_owner = owner_id;
        g_p2_rx_diag.last_addr_offset_ticks =
            (int32_t)(address_ticks - subslot_start_ticks);

        WAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END!=0, subslot_active_end_ticks + US_TO_TIMER_TICKS(TX_CHAIN_DELAY));
        //printf("end time=%" PRId32 "\n", TIMER_TICKS_TO_US(get_last_end_time_ticks() - get_last_address_time_ticks()));
        if(NRF_RADIO->EVENTS_END == 0){
            g_p2_rx_diag.end_timeout++;
            g_p2_rx_diag.last_status = 2U;
        }
        else if (NRF_RADIO->CRCSTATUS != 1) {
            g_p2_rx_diag.crc_fail++;
            g_p2_rx_diag.last_status = 3U;
        }
        else {
            uint32_t rx_valid_before = g_proto.p2.rx_valid;

            _handle_p2_rx(rx_buffer);

            if (g_proto.p2.rx_valid != rx_valid_before) {
                g_p2_rx_diag.accepted++;
                g_p2_rx_diag.last_status = 4U;
            }
            else {
                g_p2_rx_diag.rejected++;
                g_p2_rx_diag.last_status = 5U;
            }
        }
    }
    else {
        uint8_t subslot_idx = p2_get_subslot_idx(&g_proto);

        g_p2_rx_diag.addr_timeout++;
        if (subslot_idx < TIMECAST_STORE_MAX_NODES) {
            g_p2_rx_diag.addr_timeout_by_subslot[subslot_idx]++;
        }
        g_p2_rx_diag.last_slot = p2_get_slot_idx(&g_proto);
        g_p2_rx_diag.last_subslot = subslot_idx;
        g_p2_rx_diag.last_owner = owner_id;
        g_p2_rx_diag.last_status = 1U;
        g_p2_rx_diag.last_addr_offset_ticks = 0;
    }
    

    p2_finish_subslot(&g_proto, &g_proto_cfg);
}

static void _mark_configured_participants(void)
{
    uint8_t node_id;

    store_clear(&g_store);
    for (node_id = 0U; node_id < g_proto_cfg.p2_node_count; node_id++) {
        (void)store_mark_participant(&g_store, node_id);
    }
}

static void _reset_round_runtime_state(void)
{
    memset(&g_pre_commit, 0, sizeof(g_pre_commit));
    memset(&g_p2_rx_diag, 0, sizeof(g_p2_rx_diag));
    g_round_tx_update_req = false;
    _mark_configured_participants();

    g_round_pre_commit_slot_ticks = _pre_commit_slot_ticks(g_proto_cfg.p2_node_count);
}

static void _select_pre_p2_for_round(void)
{
    if (USE_PRE_P2) {
        if (_is_master()) {
            g_round_run_pre = g_master_force_initial_pre || g_master_run_pre_next_round;
            g_master_run_pre_next_round = false;
            g_master_force_initial_pre = false;
        }
        else {
            g_round_run_pre = false;
        }
    }
    else {
        g_round_run_pre = false;
    }
}

static bool _p2_chain_is_complete(void)
{
    return store_present_count(&g_store) >= store_participant_count(&g_store);
}

static void _master_track_p2_completeness(void)
{
    if (!USE_PRE_P2 || !_is_master() ||
        (MASTER_P2_INCOMPLETE_PRE_THRESHOLD == 0U)) {
        return;
    }

    if (_p2_chain_is_complete()) {
        /* Only reset the reconnect counter here.  Do not clear
         * g_master_run_pre_next_round: an update request may already have
         * queued pre for the next round. */
        g_master_p2_incomplete_rounds = 0U;
        return;
    }

    g_master_p2_incomplete_rounds++;
    if (g_master_p2_incomplete_rounds >= MASTER_P2_INCOMPLETE_PRE_THRESHOLD) {
        _master_request_pre_next_round();
        g_master_p2_incomplete_rounds = 0U;
    }
}

static void _prepare_round(void)
{
    g_round_count++;
    _reset_round_runtime_state();
    _select_pre_p2_for_round();
    _refresh_local_payload();
}

static void _run_p1_phase(uint32_t next_master_round_start_ticks)
{
    if (_is_master()) {
        /* Enter P1 before slot 0 so the first TX is already scheduled
         * when the round start timestamp arrives. */
        p1_start(&g_proto, next_master_round_start_ticks,
                                   &g_proto_cfg, true, g_round_count);
    }
    else {
        p1_start(&g_proto, 0U, &g_proto_cfg, false, 0U);
        _scan_until_reference();
    }

    while (p1_is_active(&g_proto) &&
           p1_has_tref(&g_proto)) {
        _run_p1_slot();
    }
}

static void _run_round_original(uint32_t *next_master_round_start_ticks)
{
    uint32_t p2_start_ticks;

    _prepare_round();
    _run_p1_phase(*next_master_round_start_ticks);

    p2_start_ticks = p1_get_next_phase_start_local_ticks(&g_proto);
    g_round_p2_start_ticks = p2_start_ticks;
    _load_local_desired_payload_for_p2();

    /* Enter fixed P2 before subslot 0 so the first TX/RX is armed ahead of
     * the common P2 start timestamp. */
    p2_start_original(&g_proto, p2_start_ticks, &g_proto_cfg);
    while (p2_is_active(&g_proto)) {
        _run_p2_subslot();
    }

    _log_round_summary_original();

    if (_is_master()) {
        *next_master_round_start_ticks += _original_round_period_ticks();
    }
}


static uint32_t _run_pre_p2_collect(uint32_t start_ticks)
{
    pre_p2_start(&g_proto, &g_store, start_ticks, &g_proto_cfg);

     g_proto.pre_p2.p2_payload_len[LOCAL_NODE_ID] = _class_to_payload_len(g_local_desired_class);
    while (pre_p2_is_active(&g_proto)) {
        _run_pre_p2_subslot();
    }

    return pre_p2_get_p2_start_local_ticks(&g_proto);
}

static void _run_round_pre_p2(uint32_t *next_master_round_start_ticks)
{
    uint32_t next_p1_phase_start_ticks;
    uint32_t next_collect_phase_start_ticks;
    uint32_t p2_start_ticks;
    uint8_t round_schedule_class[TIMECAST_STORE_MAX_NODES];
    bool skip_p2 = false;

    _prepare_round();
    _run_p1_phase(*next_master_round_start_ticks);

    _freeze_round_update_advertisement();

    next_p1_phase_start_ticks = p1_get_next_phase_start_local_ticks(&g_proto);
    next_collect_phase_start_ticks = next_p1_phase_start_ticks;
    p2_start_ticks = next_p1_phase_start_ticks;
    if (g_round_run_pre) {
        next_collect_phase_start_ticks = _run_pre_p2_collect(next_p1_phase_start_ticks);
        if (_is_master()) {
            _build_schedule_from_pre_collect(round_schedule_class);
        }
        _pre_commit_start(next_collect_phase_start_ticks, round_schedule_class);
        while (g_pre_commit.slot_idx < (uint8_t)(2U * NTX)) {
            _run_pre_commit_slot();
        }
        p2_start_ticks = _pre_commit_p2_start_ticks();
        if (g_pre_commit.have_schedule) {
            _unpack_class_schedule(g_pre_commit.packed_schedule,
                                   g_proto_cfg.p2_node_count,
                                   round_schedule_class);
            _commit_round_schedule(round_schedule_class);
        }
        else {
            skip_p2 = !_is_master();
            g_update_pending_latched = true;
        }
    }
    else {
        _seed_round_schedule_from_committed();
    }

    g_round_p2_start_ticks = p2_start_ticks;
    if (!skip_p2) {
        _load_local_committed_payload_for_p2();
    }
    if (skip_p2) {
        _wait_out_p2_schedule(p2_start_ticks);
    }
    else {
        /* Enter adaptive P2 before subslot 0 so the first TX/RX is armed ahead of
         * the common P2 start timestamp. */
        p2_start_pre_p2(&g_proto, p2_start_ticks, &g_proto_cfg);
        while (p2_is_active(&g_proto)) {
            _run_p2_subslot();
        }
    }

    _master_track_p2_completeness();
    _log_round_summary_pre_p2();

    if (_is_master()) {
        *next_master_round_start_ticks += _improved_round_period_ticks(g_round_run_pre);
    }
}

int main(void)
{
    uint32_t next_master_round_start_ticks;
    uint8_t node_id;

    printf("TimeCast start. node_id=%u hop=p1 role=%s ntx=%u pre_p2=%u app_data=%u payload=%u\n",
           (unsigned)LOCAL_NODE_ID,
           _role_name(),
           (unsigned)NTX,
           (unsigned)USE_PRE_P2,
           (unsigned)APP_DATA_LEN,
           (unsigned)LOCAL_PAYLOAD_LEN);
    printf("[timecast] timing: p1_slot=%u us pre_p2_subslot=%u us p2_subslot=%u us "
           "(proc={p1/p2:%u,pp2:%u} ramp=%u p1_air=%u p2_air=%u fast_ru=%u tc=%" PRIu32 ") "
           "tx_min_arm=%u p1{rx_lead=%u} pp2{guard=%u} p2{rx_lead=%u nodes=%u guard=%u} gap=%u\n",
           (unsigned)P1_SLOT_US,
           (unsigned)PRE_P2_SUBSLOT_US,
           (unsigned)P2_SUBSLOT_US,
           (unsigned)SLOT_PROCESSING_US,
           (unsigned)PRE_P2_SLOT_PROCESSING_US,
           (unsigned)RADIO_RAMPUP_US,
           (unsigned)PACKET_AIR_TIME_US(PACKET_P1_SYNC_PAYLOAD_LEN),
           (unsigned)PACKET_AIR_TIME_US(PACKET_P2_DATA_HDR_LEN +
                                              LOCAL_PAYLOAD_LEN),
           (unsigned)FAST_RAMPUP,
           TIMER_TICKS_TO_US(g_proto_cfg.p1_rx_ts_to_slot_start_ticks),
           (unsigned)TX_MIN_ARM_LEAD_US,
           (unsigned)P1_RX_LEAD_US,
           (unsigned)PRE_P2_SUBSLOT_GUARD_US,
           (unsigned)P2_RX_LEAD_US,
           (unsigned)P2_NODE_COUNT,
           (unsigned)P2_SUBSLOT_GUARD_US,
           (unsigned)ROUND_GAP_US);


    radio_start();

    store_init(&g_store, (uint8_t)LOCAL_NODE_ID);
    (void)store_mark_participant(&g_store, (uint8_t)LOCAL_NODE_ID);
    protocol_init(&g_proto, _is_master());
    for (node_id = 0U; node_id < g_proto_cfg.p2_node_count; node_id++) {
        g_committed_class[node_id] = _is_master() ? CLASS_MAX_ID : _payload_len_to_class((uint8_t)(PACKET_P2_DATA_HDR_LEN + LOCAL_PAYLOAD_META_LEN));
    }
    _refresh_local_payload();
    _commit_schedule(g_committed_class);

    g_round_count = 0U;
    next_master_round_start_ticks = now_ticks() +
                                    US_TO_TIMER_TICKS(MASTER_START_DELAY_US);

    while (1) {
        if (USE_PRE_P2) {
            _run_round_pre_p2(&next_master_round_start_ticks);
        }
        else {
            _run_round_original(&next_master_round_start_ticks);
        }
    }

    return 0;
}

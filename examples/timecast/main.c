#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "packet.h"
#include "nrf_sf_radio/link_radio.h"
#include "protocol.h"
#include "store.h"
#include "nrf_sf_radio/radio_driver.h"


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
#if (NTX > 63U)
#error "NTX exceeds 7-bit packed relay_cnt budget"
#endif

static uint8_t rx_buffer[255] = {0};

#define P1_SLOT_TICKS         NRF_SF_RADIO_US_TO_TIMER_TICKS(P1_SLOT_US)
#define PRE_P2_SUBSLOT_TICKS  NRF_SF_RADIO_US_TO_TIMER_TICKS(PRE_P2_SUBSLOT_US)
#define P2_SUBSLOT_TICKS      NRF_SF_RADIO_US_TO_TIMER_TICKS(P2_SUBSLOT_US)
#define P1_SLOT_ACTIVE_US     (P1_SLOT_US - SLOT_PROCESSING_US)
#define PRE_P2_SUBSLOT_PERIOD_US (PRE_P2_SUBSLOT_US + PRE_P2_SUBSLOT_GUARD_US)
#define P2_SUBSLOT_PERIOD_US  (P2_SUBSLOT_US + P2_SUBSLOT_GUARD_US)
#define P1_SYNC_DURATION_TICKS ((uint32_t)(2U * NTX) * P1_SLOT_TICKS)
#define P1_SLOT_ACTIVE_TICKS  NRF_SF_RADIO_US_TO_TIMER_TICKS(P1_SLOT_ACTIVE_US)
#define PRE_P2_SLOT_PROCESSING_TICKS NRF_SF_RADIO_US_TO_TIMER_TICKS(PRE_P2_SLOT_PROCESSING_US)
#define SLOT_PROCESSING_TICKS NRF_SF_RADIO_US_TO_TIMER_TICKS(SLOT_PROCESSING_US)
#define PRE_P2_SUBSLOT_PERIOD_TICKS NRF_SF_RADIO_US_TO_TIMER_TICKS(PRE_P2_SUBSLOT_PERIOD_US)
#define P2_SUBSLOT_PERIOD_TICKS NRF_SF_RADIO_US_TO_TIMER_TICKS(P2_SUBSLOT_PERIOD_US)
#define CLASS_COUNT      (16U)
#define CLASS_MAX_ID     (CLASS_COUNT - 1U)
#define CLASS_INVALID_ID (CLASS_COUNT)
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
static uint8_t g_scheduled_class[TIMECAST_STORE_MAX_NODES];
static uint8_t g_local_desired_class;
static uint8_t g_local_scheduled_payload_len;
static uint8_t g_local_scheduled_payload[TIMECAST_STORE_MAX_DATA_LEN];

static bool g_update_pending_latched;
static bool g_round_tx_update_req;
static bool g_round_run_pre;
static bool g_master_run_pre_next_round;
static bool g_master_force_initial_pre = true;
static uint32_t g_master_p2_incomplete_rounds;
static bool g_syn_least_once = false;

static bool _is_master(void);

static timecast_protocol_cfg_t g_proto_cfg = {
    .local_node_id = (uint8_t)LOCAL_NODE_ID,
    .ntx = NTX,
    .p2_node_count = P2_NODE_COUNT,
    .glossy_slot_ticks = P1_SLOT_TICKS,
    .p1_rx_ts_to_slot_start_ticks = NRF_SF_RADIO_US_TO_TIMER_TICKS(P1_RX_TS_TO_SLOT_START_US),
    .p1_guard_ticks = NRF_SF_RADIO_US_TO_TIMER_TICKS(P2_START_GUARD_US),
    .pre_p2_subslot_ticks = PRE_P2_SUBSLOT_TICKS,
    .pre_p2_guard_ticks = NRF_SF_RADIO_US_TO_TIMER_TICKS(PRE_P2_SUBSLOT_GUARD_US),
    .pre_p2_rx_window_ticks = NRF_SF_RADIO_US_TO_TIMER_TICKS(P2_RX_WINDOW_US),
    .p2_subslot_ticks = P2_SUBSLOT_TICKS,
    .p2_guard_ticks = NRF_SF_RADIO_US_TO_TIMER_TICKS(P2_SUBSLOT_GUARD_US),
    .p2_rx_window_ticks = NRF_SF_RADIO_US_TO_TIMER_TICKS(P2_RX_WINDOW_US),
    .p2_payload_base_ticks =
        NRF_SF_RADIO_US_TO_TIMER_TICKS(SLOT_PROCESSING_US + RADIO_RAMPUP_US +
                                (8U * SLOT_PHY_OVERHEAD_BYTES)),
    .p2_payload_byte_ticks = NRF_SF_RADIO_US_TO_TIMER_TICKS(8U),
};



static uint32_t _p2_duration_ticks(uint8_t node_count)
{
    return ((uint32_t)(2U * NTX) * (uint32_t)node_count * P2_SUBSLOT_PERIOD_TICKS);
}

static uint32_t _pre_p2_duration_ticks(uint8_t node_count)
{
    return ((uint32_t)(2U * NTX) * (uint32_t)node_count * PRE_P2_SUBSLOT_PERIOD_TICKS);
}

static uint8_t _class_span_bytes(void)
{
    return (uint8_t)(((PACKET_P2_DATA_MAX_DATA_LEN + 1U) +
                      CLASS_COUNT - 1U) / CLASS_COUNT);
}

static uint8_t _class_to_payload_len(uint8_t class_id)
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

static uint32_t _fixed_p2_slot_ticks(void)
{
    return (uint32_t)g_proto_cfg.p2_node_count *
           (g_proto_cfg.p2_subslot_ticks + g_proto_cfg.p2_guard_ticks);
}

static uint32_t _original_p2_duration_ticks(void)
{
    return _p2_duration_ticks(g_proto_cfg.p2_node_count);
}



static uint32_t _pre_commit_duration_ticks(uint32_t slot_ticks)
{
    return (uint32_t)(2U * NTX) * slot_ticks;
}

static uint32_t _original_round_period_ticks(void)
{
    uint32_t period_ticks = P1_SYNC_DURATION_TICKS + g_proto_cfg.p1_guard_ticks;

    period_ticks += _original_p2_duration_ticks();

    return period_ticks + NRF_SF_RADIO_US_TO_TIMER_TICKS(ROUND_GAP_US);
}

static uint32_t _improved_round_period_ticks(bool run_pre)
{
    uint32_t period_ticks = P1_SYNC_DURATION_TICKS + g_proto_cfg.p1_guard_ticks;

    if (run_pre) {
        period_ticks += _pre_p2_duration_ticks(g_proto_cfg.p2_node_count);
        period_ticks += g_proto_cfg.p1_guard_ticks;
        period_ticks += _pre_commit_duration_ticks(g_proto.pre_commit.slot_ticks);
        period_ticks += g_proto_cfg.p1_guard_ticks;
    }

    period_ticks += (uint32_t)(2U * NTX) * p2_get_slot_ticks(&g_proto, &g_proto_cfg);
    return period_ticks + NRF_SF_RADIO_US_TO_TIMER_TICKS(ROUND_GAP_US);
}

static bool _is_master(void)
{
    return (LOCAL_NODE_ID == 0U);
}


static uint8_t _payload_len_to_class(uint8_t payload_len)
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

static uint8_t _local_data_len_for_source(uint8_t source_id)
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


static void _build_schedule_from_pre_collect(uint8_t *classes_out)
{
    uint8_t source_id;
    uint8_t class_id;
    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        if (g_proto.pre_p2.present[source_id]) {
            class_id = _payload_len_to_class(g_proto.pre_p2.p2_payload_len[source_id]);

            classes_out[source_id] = class_id;
        }
        else {
            classes_out[source_id] = CLASS_MAX_ID;
        }
    }
}

static void _apply_round_schedule_to_proto(void)
{
    uint8_t source_id;

    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        g_proto.pre_p2.present[source_id] = 1U;
        g_proto.pre_p2.p2_payload_len[source_id] = _class_to_payload_len(g_scheduled_class[source_id]);
    }
}

static void _master_request_pre_next_round(void)
{
    if (_is_master()) {
        g_master_run_pre_next_round = true;
    }
}


static bool _local_packet_should_request_update(uint8_t owner_id)
{
    return (owner_id == (uint8_t)LOCAL_NODE_ID) && g_round_tx_update_req;
}


static void _commit_round_schedule(const uint8_t *classes)
{
    uint8_t source_id;
    uint8_t local_source_id = (uint8_t)LOCAL_NODE_ID;
    uint8_t payload[TIMECAST_STORE_MAX_DATA_LEN];
    uint8_t scheduled_p2_payload_len;
    uint8_t scheduled_data_len;
    uint8_t desired_payload_len;
    uint8_t payload_len;
    

    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        g_scheduled_class[source_id] = classes[source_id];
    }


    scheduled_p2_payload_len = _class_to_payload_len(classes[local_source_id]);
    scheduled_data_len = (uint8_t)(scheduled_p2_payload_len -
                                    PACKET_P2_DATA_HDR_LEN);
    desired_payload_len = _local_data_len_for_source(local_source_id);
    if ((desired_payload_len <= scheduled_data_len) ||
        (g_local_scheduled_payload_len == 0U)) {
        payload_len = (desired_payload_len <= scheduled_data_len) ?
                        desired_payload_len : LOCAL_PAYLOAD_META_LEN;
        _build_local_payload_with_len(local_source_id, payload, payload_len);
        memcpy(g_local_scheduled_payload, payload, payload_len);
        g_local_scheduled_payload_len = payload_len;
    }

    g_update_pending_latched = (g_local_desired_class != g_scheduled_class[LOCAL_NODE_ID]) ? 1U : 0U;

}

static void _local_payload_init(void)
{
    uint8_t node_id;
    uint8_t source_id = (uint8_t)LOCAL_NODE_ID;
    uint8_t payload_len;
    uint8_t payload_class;
    uint8_t payload[TIMECAST_STORE_MAX_DATA_LEN];

    if (source_id >= g_proto_cfg.p2_node_count) {
        return;
    }

    for (node_id = 0U; node_id < g_proto_cfg.p2_node_count; node_id++) {
        g_scheduled_class[node_id] = _is_master() ? CLASS_MAX_ID : _payload_len_to_class((uint8_t)(PACKET_P2_DATA_HDR_LEN + LOCAL_PAYLOAD_META_LEN));
    }

    payload_len = _local_data_len_for_source(source_id);
    payload_class = _payload_len_to_class((uint8_t)(PACKET_P2_DATA_HDR_LEN + payload_len));

    payload_len = _build_local_payload_with_len(source_id, payload, LOCAL_PAYLOAD_META_LEN);
    memcpy(g_local_scheduled_payload, payload, payload_len);
    g_local_scheduled_payload_len = payload_len;
    
    g_local_desired_class = payload_class;
    if (g_local_desired_class != g_scheduled_class[source_id]) {
        g_update_pending_latched = true;
    }
}

static void _load_local_desired_payload_for_p2(void)
{
    uint8_t source_id = (uint8_t)LOCAL_NODE_ID;
    uint8_t payload[TIMECAST_STORE_MAX_DATA_LEN];
    uint8_t payload_len;

    payload_len = _build_local_payload_with_len(source_id, payload,
                                                _local_data_len_for_source(source_id));
    (void)store_import(&g_store, source_id, payload, payload_len);
}

static void _load_local_scheduled_payload_for_p2(void)
{
    uint8_t source_id = (uint8_t)LOCAL_NODE_ID;

    if ((source_id < g_proto_cfg.p2_node_count) && (g_local_scheduled_payload_len > 0U)) {
        (void)store_import(&g_store, source_id, g_local_scheduled_payload, g_local_scheduled_payload_len);
    }
}


static uint32_t _elapsed_since_ticks(uint32_t start_ticks, uint32_t now_tick)
{
    if (start_ticks == 0U) {
        return 0U;
    }

    return now_tick - start_ticks;
}

static uint32_t _pre_commit_slot_start_ticks(void)
{
    return g_proto.pre_commit.start_local_ticks +
           ((uint32_t)g_proto.pre_commit.slot_idx * g_proto.pre_commit.slot_ticks);
}

static uint32_t _pre_commit_p2_start_ticks(void)
{
    return g_proto.pre_commit.start_local_ticks +
           _pre_commit_duration_ticks(g_proto.pre_commit.slot_ticks) +
           g_proto_cfg.p1_guard_ticks;
}

static void _handle_pre_commit_rx(const uint8_t *buf)
{
    uint8_t packed_schedule[PACKET_PRE_COMMIT_MAX_SCHEDULE_LEN];
    size_t packed_len = 0U;

    decode_pre_commit(buf, g_proto_cfg.p2_node_count, packed_schedule, &packed_len);


    if (!g_proto.pre_commit.have_schedule) {
        memcpy(g_proto.pre_commit.packed_schedule, packed_schedule, packed_len);
        g_proto.pre_commit.packed_len = (uint8_t)packed_len;
        g_proto.pre_commit.have_schedule = true;
    }
    g_proto.pre_commit.flag_tx = true;
    g_proto.pre_commit.rx_valid++;
}

static void _pre_commit_finish_slot(bool did_tx)
{
    if (did_tx) {
        g_proto.pre_commit.flag_tx = false;
    }
    else if (g_proto.pre_commit.have_schedule) {
        g_proto.pre_commit.flag_tx = true;
    }

    g_proto.pre_commit.slot_idx++;
    if(g_proto.pre_commit.slot_idx >= (uint8_t)(2U * NTX)){
        g_proto.pre_commit.active = false;
    }
}

static void _run_pre_commit_slot(void)
{
    uint32_t slot_start_ticks = _pre_commit_slot_start_ticks();
    uint32_t slot_end_ticks = slot_start_ticks + g_proto.pre_commit.slot_ticks;
    uint32_t slot_active_end_ticks = slot_end_ticks - SLOT_PROCESSING_TICKS;
    uint32_t rx_window_end_ticks = slot_start_ticks + g_proto_cfg.p2_rx_window_ticks;
    uint32_t now_tick = nrf_sf_radio_now_ticks();
    bool do_tx = g_proto.pre_commit.have_schedule && g_proto.pre_commit.flag_tx &&
                 (g_proto.pre_commit.ntx_done < NTX);


    if ((int32_t)(now_tick - slot_start_ticks) >= 0) {
        _pre_commit_finish_slot(do_tx);
        return;
    }

    if (do_tx) {
        uint8_t payload[PACKET_PRE_COMMIT_MAX_PAYLOAD_LEN] = {0};

        encode_pre_commit(payload, g_proto.pre_commit.packed_schedule,
                          g_proto.pre_commit.packed_len);

        if (!nrf_sf_radio_tx_start(payload, slot_start_ticks,
                                   slot_active_end_ticks,
                                   g_proto.pre_commit.packed_len)) {
            printf("[timecast-at] pre-commit tx error\n");
        }

        g_proto.pre_commit.ntx_done++;
        _pre_commit_finish_slot(true);
        return;
    }

    if (!nrf_sf_radio_rx_start(rx_buffer,
                               slot_start_ticks -
                               NRF_SF_RADIO_US_TO_TIMER_TICKS(P2_RX_LEAD_US),
                               rx_window_end_ticks, slot_active_end_ticks)) {
        printf("[timecast-at] pre-commit rx error\n");
    }
    else {
        _handle_pre_commit_rx(rx_buffer);
    }

    _pre_commit_finish_slot(false);
}


static void _log_round_summary_original(void)
{
    uint32_t now_tick = nrf_sf_radio_now_ticks();
    uint32_t p2_duration_ticks = _elapsed_since_ticks(g_round_p2_start_ticks, now_tick);
    uint16_t present_count = store_present_count(&g_store);
    uint32_t p2_slot_ticks = _fixed_p2_slot_ticks();
    printf("[timecast] round{id=%u,role=%s,round=%" PRIu32
           ",epoch=%" PRIu32 ",joined=%u,hop=%u,syncslot=%u"
           ",rx=%" PRIu32
           ",p2rx=%" PRIu32 ",p2store=%" PRIu32
           ",present=%u/%u"
           ",p1slot=%u,p2slot=%" PRIu32 ",p2rdu=%" PRIu32
           ",p2start=%" PRIu32,
           (unsigned)LOCAL_NODE_ID,
           _is_master() ? "master" : "follower",
           g_round_count,
           g_proto.current_epoch,
           (unsigned)g_proto.joined,
           (unsigned)g_proto.p1.local_hop,
           (unsigned)g_proto.p1.slot_idx,
           g_proto.rx_valid,
           g_proto.p2.rx_valid,
           g_proto.p2.store_updates,
           (unsigned)present_count,
           (unsigned)P2_NODE_COUNT,
           (unsigned)P1_SLOT_US,
           NRF_SF_RADIO_TIMER_TICKS_TO_US(p2_slot_ticks),
           NRF_SF_RADIO_TIMER_TICKS_TO_US(p2_duration_ticks),
           NRF_SF_RADIO_TIMER_TICKS_TO_US(g_round_p2_start_ticks));

    printf("}\n");
}

static void _log_round_summary_pre_p2(void)
{
    uint8_t local_pending_updates = (g_local_desired_class != g_scheduled_class[LOCAL_NODE_ID]) ? 1U : 0U;
    uint32_t now_tick = nrf_sf_radio_now_ticks();
    uint32_t p2_duration_ticks = _elapsed_since_ticks(g_round_p2_start_ticks, now_tick);
    uint16_t present_count = store_present_count(&g_store);
    uint32_t p2_slot_ticks = p2_get_slot_ticks(&g_proto, &g_proto_cfg);
    bool log_update_state = g_round_run_pre ||
                            g_update_pending_latched ||
                            (local_pending_updates > 0U) ||
                            g_master_run_pre_next_round ||
                            (g_master_p2_incomplete_rounds > 0U);
    char scheduled_class_map[TIMECAST_STORE_MAX_NODES + 1];
    char desired_class_map[TIMECAST_STORE_MAX_NODES + 1];

    printf("[timecast] round{id=%u,role=%s,round=%" PRIu32
           ",epoch=%" PRIu32 ",joined=%u,hop=%u,syncslot=%u"
           ",p1rx=%" PRIu32
           ",pre=%u,upd=%u"
           ",pp2rx=%" PRIu32
           ",p2rx=%" PRIu32 ",p2store=%" PRIu32
           ",present=%u/%u"
           ",p1slot=%u,pp2sub=%u,p2slot=%" PRIu32 ",p2rdu=%" PRIu32
            ",p2start=%" PRIu32,
           (unsigned)LOCAL_NODE_ID,
           _is_master() ? "master" : "follower",
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
           (unsigned)P2_NODE_COUNT,
           (unsigned)P1_SLOT_US,
           (unsigned)PRE_P2_SUBSLOT_PERIOD_US,
           NRF_SF_RADIO_TIMER_TICKS_TO_US(p2_slot_ticks),
           NRF_SF_RADIO_TIMER_TICKS_TO_US(p2_duration_ticks),
           NRF_SF_RADIO_TIMER_TICKS_TO_US(g_round_p2_start_ticks));

    if (log_update_state) {
        _format_class_map(scheduled_class_map, sizeof(scheduled_class_map),
                          g_scheduled_class, false);
        _format_class_map(desired_class_map, sizeof(desired_class_map),
                          NULL, true);
        printf(",u={loc=%u,next=%u,inc=%" PRIu32
               ",pcrx=%" PRIu32 ",pchave=%u,cc=%s,dc=%s}",
               (unsigned)local_pending_updates,
               (unsigned)g_master_run_pre_next_round,
               g_master_p2_incomplete_rounds,
               g_proto.pre_commit.rx_valid,
               (unsigned)g_proto.pre_commit.have_schedule,
               scheduled_class_map,
               desired_class_map);
    }
    printf("}\n");
}

static void _handle_p1_rx(const uint8_t *buf, uint32_t rx_time)
{
    p1_sync_frame_t frame;

    if(!decode_p1_sync(buf, &frame)){
        return;
    }
    p1_handle_rx(&g_proto, &frame, rx_time, &g_proto_cfg);
    g_round_run_pre = ((frame.flags & 1U) != 0U);
    g_proto.rx_valid++;

}

static void _handle_pre_p2_rx(const uint8_t *buf)
{
    uint8_t class_id;
    uint8_t p2_payload_len;

    decode_pre_p2(buf, &class_id);

    p2_payload_len = _class_to_payload_len(class_id);

    pre_p2_handle_rx(&g_proto, p2_payload_len, &g_proto_cfg);
}

static void _handle_p2_rx(const uint8_t *buf)
{
    p2_data_frame_t frame;
    uint8_t data[PACKET_P2_DATA_MAX_DATA_LEN];


    decode_p2_data(buf, &frame, data);


    if (!p2_handle_rx(&g_proto, &g_store, &frame, data)) {
        return;
    }

    if ((frame.flags & 1U) != 0U) {
        g_update_pending_latched = true;
        _master_request_pre_next_round();
    }
}

static void _scan_until_reference(void)
{
    uint32_t rx_ticks = 0;
    while (g_proto.p1.active &&
           !g_proto.p1.has_tref) {
        rx_ticks = nrf_sf_radio_rx_listen_until_packet(rx_buffer, P1_SLOT_TICKS,
                                     P1_SCAN_LOG_INTERVAL_US);
        if (rx_ticks >2U) {
            _handle_p1_rx(rx_buffer, rx_ticks);
        }
        else if(rx_ticks == 0){
            printf("[timecast] waiting{id=%u,round=%" PRIu32 "}\n",
                   (unsigned)LOCAL_NODE_ID, g_round_count);
        }
    }
}

static void _run_p1_slot(void)
{
    bool do_tx = g_proto.p1.flag_tx;
    uint32_t slot_start_ticks = p1_get_slot_start_local_ticks(&g_proto, &g_proto_cfg);
    uint32_t slot_active_end_ticks = slot_start_ticks + P1_SLOT_ACTIVE_TICKS -
                                     NRF_SF_RADIO_RAMPUP_TIME_TICKS;
    uint32_t now_tick = nrf_sf_radio_now_ticks();

    if ((int32_t)(now_tick - slot_start_ticks + NRF_SF_RADIO_RAMPUP_TIME_TICKS) >= 0) {
        printf("[timecast] slot miss: slot=%u now=%" PRIu32 " start=%" PRIu32 "\n",
               (unsigned)g_proto.p1.slot_idx,
               now_tick, slot_start_ticks);
        (void)p1_finish_slot(&g_proto, &g_proto_cfg, do_tx);
        return;
    }

    if (do_tx) {
        p1_sync_frame_t frame;
        uint8_t payload[PACKET_P1_SYNC_APP_LEN] = {0};
        p1_prepare_tx(&g_proto, &frame);
        frame.flags = g_round_run_pre ? 1U : 0U;
        encode_p1_sync(payload, sizeof(payload), &frame);
        if (nrf_sf_radio_tx_start(payload,
                                  slot_start_ticks -
                                  NRF_SF_RADIO_RAMPUP_TIME_TICKS,
                                  slot_active_end_ticks,
                                  PACKET_P1_SYNC_APP_LEN)) {
            p1_finish_slot(&g_proto, &g_proto_cfg, true);
        }
        else {
            g_proto.p1_tx_sched_fails++;
            uint32_t now_tick = nrf_sf_radio_now_ticks();
            int32_t slack_ticks = (int32_t)(slot_start_ticks - now_tick);
            printf("[timecast] TX schedule failed: now=%" PRIu32
                   " deadline=%" PRIu32 " slack=%" PRId32 " ticks fails=%" PRIu32   "\n",
                   now_tick, slot_start_ticks + NRF_SF_RADIO_US_TO_TIMER_TICKS(40),
                   slack_ticks, g_proto.p1_tx_sched_fails);

            p1_finish_slot(&g_proto, &g_proto_cfg, false);
        }

        return;
    }

    nrf_sf_radio_wait_until_abs(NULL, slot_active_end_ticks);

    p1_finish_slot(&g_proto, &g_proto_cfg, false);
}

static void _run_pre_p2_subslot(void)
{
    bool tx_slot = g_proto.pre_p2.tx_slot;
    uint32_t subslot_start_ticks =
        pre_p2_get_subslot_start_local_ticks(&g_proto, &g_proto_cfg);
    uint32_t subslot_end_ticks = subslot_start_ticks + g_proto_cfg.pre_p2_subslot_ticks;
    uint32_t subslot_active_end_ticks = subslot_end_ticks - PRE_P2_SLOT_PROCESSING_TICKS;
    uint32_t rx_window_end_ticks = subslot_start_ticks + g_proto_cfg.pre_p2_rx_window_ticks;
    uint32_t now_tick = nrf_sf_radio_now_ticks();
    uint8_t owner_id = g_proto.pre_p2.subslot_idx;

    if ((int32_t)(now_tick - subslot_start_ticks) >= 0) {
        printf("[timecast] pre-p2 miss: slot=%u sub=%u now=%" PRIu32 " deadline=%" PRIu32 "\n",
               (unsigned)g_proto.p2.slot_idx,
               (unsigned)g_proto.pre_p2.subslot_idx,
               now_tick, subslot_start_ticks);
        pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (tx_slot) {
        uint8_t p2_payload_len;
        uint8_t class_id;
        uint8_t payload[PACKET_PRE_P2_CTRL_LEN] = {0};

        if (!g_proto.pre_p2.present[owner_id]) {
            nrf_sf_radio_wait_until_abs(NULL, subslot_active_end_ticks);
            pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }
        p2_payload_len = g_proto.pre_p2.p2_payload_len[owner_id];

        class_id = _payload_len_to_class(p2_payload_len);

        encode_pre_p2(payload, class_id);
        if (!nrf_sf_radio_tx_start(payload, subslot_start_ticks,
                                   subslot_active_end_ticks,
                                   PACKET_PRE_P2_CTRL_APP_LEN)) {
            printf("[timecast-at] pre-collect tx error\n");
        }
        pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (g_proto.pre_p2.complete ||
        g_proto.pre_p2.present[owner_id]) {
        nrf_sf_radio_wait_until_abs(NULL, subslot_active_end_ticks);
        pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (nrf_sf_radio_rx_start(rx_buffer,
                              subslot_start_ticks -
                              NRF_SF_RADIO_US_TO_TIMER_TICKS(P2_RX_LEAD_US),
                              rx_window_end_ticks, subslot_active_end_ticks)) {
        _handle_pre_p2_rx(rx_buffer);
    }
    else {
        printf("[timecast-at] pre-collect rx error\n");
    }

    pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
}

static void _run_p2_subslot(void)
{
    bool tx_slot = g_proto.p2.tx_slot;
    uint32_t subslot_ticks = g_proto.p2.subslot_ticks[g_proto.p2.subslot_idx];
    uint32_t subslot_start_ticks = p2_get_subslot_start_local_ticks(&g_proto);
    uint32_t subslot_end_ticks = subslot_start_ticks + subslot_ticks;
    uint32_t subslot_active_end_ticks = subslot_end_ticks - SLOT_PROCESSING_TICKS;
    uint32_t rx_window_end_ticks = subslot_start_ticks + g_proto_cfg.p2_rx_window_ticks;
    uint32_t now_tick = nrf_sf_radio_now_ticks();
    uint8_t owner_id = g_proto.p2.subslot_idx;

    if ((int32_t)(now_tick - subslot_start_ticks) >= 0) {
        printf("[timecast] p2 miss: slot=%u sub=%u now=%" PRIu32 " deadline=%" PRIu32 "\n",
               (unsigned)g_proto.p2.slot_idx,
               (unsigned)g_proto.p2.subslot_idx,
               now_tick, subslot_start_ticks);
        (void)p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (tx_slot) {
        p2_data_frame_t frame;
        const uint8_t *data_ptr = NULL;
        uint8_t payload[PACKET_P2_DATA_MAX_PAYLOAD_LEN] = {0};

        if (!p2_prepare_tx(&g_proto, &g_store, &g_proto_cfg, &frame, &data_ptr)) {
            nrf_sf_radio_wait_until_abs(NULL, subslot_active_end_ticks);
            p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }
        frame.flags = _local_packet_should_request_update(owner_id) ? 1U : 0U;
        encode_p2_data(payload, &frame, data_ptr);
        if (!nrf_sf_radio_tx_start(payload, subslot_start_ticks,
                                   subslot_active_end_ticks,
                                   PACKET_P2_DATA_APP_HDR_LEN +
                                   frame.data_len)) {
            printf("[timecast] P2 TX schedule failed: slot=%u sub=%u\n",
                   (unsigned)g_proto.p2.slot_idx,
                   (unsigned)g_proto.p2.subslot_idx);
            p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }

        p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (store_has_data(&g_store, owner_id)) {

        nrf_sf_radio_wait_until_abs(NULL, subslot_active_end_ticks);
        p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (nrf_sf_radio_rx_start(rx_buffer,
                              subslot_start_ticks -
                              NRF_SF_RADIO_US_TO_TIMER_TICKS(P2_RX_LEAD_US),
                              rx_window_end_ticks, subslot_active_end_ticks)) {
        _handle_p2_rx(rx_buffer);
    }

    p2_finish_subslot(&g_proto, &g_proto_cfg);
}

static bool _p2_chain_is_complete(void)
{
    return g_store.present_count >= P2_NODE_COUNT;
}

static void _master_track_p2_completeness(void)
{
    if (!USE_PRE_P2 || !_is_master() ||
        (MASTER_P2_INCOMPLETE_PRE_THRESHOLD == 0U)) {
        return;
    }

    if (_p2_chain_is_complete()) {
        
        g_master_p2_incomplete_rounds = 0U;
        return;
    }

    g_master_p2_incomplete_rounds++;
    if (g_master_p2_incomplete_rounds >= MASTER_P2_INCOMPLETE_PRE_THRESHOLD) {
        _master_request_pre_next_round();
        g_master_p2_incomplete_rounds = 0U;
    }
}

static void _wait_until_round_end(void)
{
    uint32_t p2_ticks;

    p2_ticks = 2*NTX*g_proto.p2.slot_ticks;
    nrf_sf_radio_wait_until_abs(NULL, g_round_p2_start_ticks + p2_ticks);
}

static void _prepare_round(void)
{
    uint8_t source_id = (uint8_t)LOCAL_NODE_ID;
    uint8_t data_len;
    uint8_t payload_class;

    g_round_count++;
    g_round_tx_update_req = false;
    
    memset(g_store.entries, 0, sizeof(g_store.entries));
    g_store.present_count = 0;
    

    data_len = _local_data_len_for_source(source_id);
    payload_class = _payload_len_to_class((uint8_t)(PACKET_P2_DATA_HDR_LEN + data_len));

    g_local_desired_class = payload_class;
    if (g_local_desired_class != g_scheduled_class[source_id]) {
        g_update_pending_latched = true;
    }

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

static void _run_p1_phase(uint32_t next_master_round_start_ticks)
{
    if (_is_master()) {
        
        p1_start(&g_proto, next_master_round_start_ticks,
                                   &g_proto_cfg, true, g_round_count);
    }
    else {
        p1_start(&g_proto, 0U, &g_proto_cfg, false, 0U);
        _scan_until_reference();
    }

    while (g_proto.p1.active &&
           g_proto.p1.has_tref) {
        _run_p1_slot();
    }
}

static void _run_round_original(uint32_t *next_master_round_start_ticks)
{
    uint32_t p2_start_ticks;

    _prepare_round();
    _run_p1_phase(*next_master_round_start_ticks);

    p2_start_ticks = g_proto.p1.next_phase_start_local_ticks;
    g_round_p2_start_ticks = p2_start_ticks;
    _load_local_desired_payload_for_p2();

    
    p2_start_original(&g_proto, p2_start_ticks, &g_proto_cfg);
    while (g_proto.p2.active) {
        _run_p2_subslot();
    }

    _log_round_summary_original();

    if (_is_master()) {
        *next_master_round_start_ticks += _original_round_period_ticks();
    }
}


static uint32_t _run_pre_p2_collect(uint32_t start_ticks)
{
    pre_p2_start(&g_proto, LOCAL_NODE_ID, start_ticks, &g_proto_cfg, _class_to_payload_len(g_local_desired_class));

     
    while (g_proto.pre_p2.active) {
        _run_pre_p2_subslot();
    }

    return g_proto.pre_p2.commit_start_local_tick;
}

static uint32_t _run_pre_p2_commit(uint32_t start_ticks, uint8_t *classes)
{
    if (_is_master()) {
        _build_schedule_from_pre_collect(classes);
    }

    pre_commit_start(&g_proto, start_ticks, &g_proto_cfg, classes, _is_master());

    while (g_proto.pre_commit.active) {
        _run_pre_commit_slot();
    }
    
    return _pre_commit_p2_start_ticks();
}

static void _run_round_pre_p2(uint32_t *next_master_round_start_ticks)
{
    uint32_t next_p1_phase_start_ticks;
    uint32_t next_collect_phase_start_ticks;
    uint32_t p2_start_ticks;
    uint8_t round_schedule_class[TIMECAST_STORE_MAX_NODES];

    _prepare_round();
    _run_p1_phase(*next_master_round_start_ticks);

    
    g_round_tx_update_req = g_update_pending_latched && !g_round_run_pre;
    if (g_round_tx_update_req) {
        _master_request_pre_next_round();
    }

    next_p1_phase_start_ticks = g_proto.p1.next_phase_start_local_ticks;
    p2_start_ticks = next_p1_phase_start_ticks;
    if (g_round_run_pre) {
        next_collect_phase_start_ticks = _run_pre_p2_collect(next_p1_phase_start_ticks);

        p2_start_ticks = _run_pre_p2_commit(next_collect_phase_start_ticks, round_schedule_class);

        if (g_proto.pre_commit.have_schedule) {
            _unpack_class_schedule(g_proto.pre_commit.packed_schedule,
                                   g_proto_cfg.p2_node_count,
                                   round_schedule_class);
            _commit_round_schedule(round_schedule_class);
            _apply_round_schedule_to_proto();
            g_proto.pre_p2.complete = true;
            g_syn_least_once = true;
        }
    }
    else {
        if(g_syn_least_once)
        g_proto.pre_p2.complete = true;
        _apply_round_schedule_to_proto();
    }

    g_round_p2_start_ticks = p2_start_ticks;
    _load_local_scheduled_payload_for_p2();

    p2_start_pre_p2(&g_proto, p2_start_ticks, &g_proto_cfg);
    if(!g_proto.p2.active){
        _wait_until_round_end();
    }
    while (g_proto.p2.active) {
        _run_p2_subslot();
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

    printf("TimeCast start. node_id=%u hop=p1 role=%s ntx=%u pre_p2=%u app_data=%u payload=%u\n",
           (unsigned)LOCAL_NODE_ID,
           _is_master() ? "master" : "follower",
           (unsigned)NTX,
           (unsigned)USE_PRE_P2,
           (unsigned)APP_DATA_LEN,
           (unsigned)LOCAL_PAYLOAD_LEN);


    nrf_sf_radio_start();

    store_init(&g_store, (uint8_t)LOCAL_NODE_ID);
    protocol_init(&g_proto, _is_master());
    
    _local_payload_init();

    g_round_count = 0U;
    next_master_round_start_ticks = nrf_sf_radio_now_ticks() +
                                    NRF_SF_RADIO_US_TO_TIMER_TICKS(MASTER_START_DELAY_US);

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

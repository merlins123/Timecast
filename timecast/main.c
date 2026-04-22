#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "packet.h"
#include "protocol.h"
#include "store.h"
#include "tta_driver.h"
#include "radio_util.h"

#ifndef TCAST_SLOT_PROCESSING_US
#define TCAST_SLOT_PROCESSING_US      (88U)
#endif
#ifndef TCAST_PRE_P2_SLOT_PROCESSING_US
#define TCAST_PRE_P2_SLOT_PROCESSING_US (40U)
#endif
#ifndef TCAST_FAST_RAMPUP
#define TCAST_FAST_RAMPUP             (1U)
#endif
#ifndef TCAST_RADIO_RAMPUP_US
#if TCAST_FAST_RAMPUP
#define TCAST_RADIO_RAMPUP_US         (40U)
#else
#define TCAST_RADIO_RAMPUP_US         (140U)
#endif
#endif
#ifndef TCAST_SLOT_PHY_OVERHEAD_BYTES
#define TCAST_SLOT_PHY_OVERHEAD_BYTES (16U)
#endif
#ifndef TCAST_USE_PRE_P2
#define TCAST_USE_PRE_P2             (0U)
#endif
#ifndef TCAST_APP_DATA_LEN
#define TCAST_APP_DATA_LEN           (0U)
#endif
#define TCAST_LOCAL_PAYLOAD_META_LEN (8U)
#if (TCAST_APP_DATA_LEN > (TIMECAST_STORE_MAX_DATA_LEN - TCAST_LOCAL_PAYLOAD_META_LEN))
#error "TCAST_APP_DATA_LEN exceeds TIMECAST_STORE_MAX_DATA_LEN budget"
#endif
#if (TCAST_APP_DATA_LEN > 0U)
#define TCAST_LOCAL_PAYLOAD_DATA_CAPACITY (TCAST_APP_DATA_LEN)
#else
#define TCAST_LOCAL_PAYLOAD_DATA_CAPACITY (1U)
#endif
#define TCAST_LOCAL_PAYLOAD_LEN ((uint8_t)(TCAST_LOCAL_PAYLOAD_META_LEN + TCAST_APP_DATA_LEN))
#define TCAST_LOCAL_P2_PAYLOAD_LEN ((uint8_t)(TIMECAST_PACKET_P2_DATA_HDR_LEN + TCAST_LOCAL_PAYLOAD_LEN))
#define TCAST_PACKET_AIR_TIME_US(payload_len) \
    (8U * ((uint32_t)(payload_len) + TCAST_SLOT_PHY_OVERHEAD_BYTES))
#define TCAST_P2_PAYLOAD_TO_SUBSLOT_US(payload_len) \
    (TCAST_SLOT_PROCESSING_US + TCAST_RADIO_RAMPUP_US + \
     TCAST_PACKET_AIR_TIME_US(payload_len))
#ifndef TCAST_P1_SLOT_US
#define TCAST_P1_SLOT_US              \
    (TCAST_SLOT_PROCESSING_US + TCAST_RADIO_RAMPUP_US + \
     TCAST_PACKET_AIR_TIME_US(TIMECAST_PACKET_P1_SYNC_PAYLOAD_LEN))
#endif
#ifndef TCAST_P2_SUBSLOT_US
#define TCAST_P2_SUBSLOT_US           \
    TCAST_P2_PAYLOAD_TO_SUBSLOT_US(TCAST_LOCAL_P2_PAYLOAD_LEN)
#endif
#ifndef TCAST_PRE_P2_SUBSLOT_US
#define TCAST_PRE_P2_SUBSLOT_US       \
    (TCAST_PRE_P2_SLOT_PROCESSING_US + TCAST_RADIO_RAMPUP_US + \
     TCAST_PACKET_AIR_TIME_US(TIMECAST_PACKET_PRE_P2_CTRL_LEN))
#endif

#ifndef TC_NODE_ID
#define TC_NODE_ID (NODE_ID)
#endif
#ifndef TCAST_LOCAL_HOP
#define TCAST_LOCAL_HOP (TC_NODE_ID)
#endif

#ifndef TCAST_MASTER_START_DELAY_US
#define TCAST_MASTER_START_DELAY_US   (2U * TCAST_P1_SLOT_US)
#endif
#ifndef TCAST_ROUND_GAP_US
#define TCAST_ROUND_GAP_US            (200U * TCAST_P1_SLOT_US)
#endif
#ifndef TCAST_NTX
#define TCAST_NTX                     (16U)
#endif
#ifndef TCAST_P2_NODE_COUNT
#define TCAST_P2_NODE_COUNT           (4U)
#endif
#ifndef TCAST_VIRTUAL_SOURCE_MOD
#define TCAST_VIRTUAL_SOURCE_MOD      (4U)
#endif
#if TCAST_VIRTUAL_SOURCE_MOD < 2
#error "TCAST_VIRTUAL_SOURCE_MOD must include master and at least one follower"
#endif
#ifndef TCAST_P2_START_GUARD_US
#define TCAST_P2_START_GUARD_US       (TCAST_P1_SLOT_US)
#endif
#ifndef TCAST_PRE_P2_SUBSLOT_GUARD_US
#define TCAST_PRE_P2_SUBSLOT_GUARD_US (40U)
#endif
#ifndef TCAST_P2_SUBSLOT_GUARD_US
#define TCAST_P2_SUBSLOT_GUARD_US     (88U)
#endif
#ifndef TCAST_P2_RX_WINDOW_US
#define TCAST_P2_RX_WINDOW_US         (TCAST_RADIO_RAMPUP_US + 40U)
#endif
#ifndef TCAST_P2_RX_WINDOW_MARGIN_US
#define TCAST_P2_RX_WINDOW_MARGIN_US  (24U)
#endif
#ifndef TCAST_P1_RX_TS_TO_SLOT_START_US
#define TCAST_P1_RX_TS_TO_SLOT_START_US (TCAST_RADIO_RAMPUP_US + 40U)
#endif
#ifndef TCAST_LOG_RX_EACH
#define TCAST_LOG_RX_EACH             (0U)
#endif
#ifndef TCAST_ERR_LOG_EVERY
#define TCAST_ERR_LOG_EVERY           (256U)
#endif
#ifndef TCAST_P1_TX_ARM_US
#define TCAST_P1_TX_ARM_US            (64U)
#endif
#ifndef TCAST_P1_RX_LEAD_US
#define TCAST_P1_RX_LEAD_US           (80U)
#endif
#ifndef TCAST_P2_TX_ARM_US
#define TCAST_P2_TX_ARM_US            (64U)
#endif
#ifndef TCAST_P2_RX_LEAD_US
#define TCAST_P2_RX_LEAD_US           (80U)
#endif
#ifndef TCAST_TX_MIN_ARM_LEAD_US
#define TCAST_TX_MIN_ARM_LEAD_US      (8U)
#endif
#ifndef TCAST_P1_SCAN_LOG_INTERVAL_US
#define TCAST_P1_SCAN_LOG_INTERVAL_US (1000000U)
#endif

#define P1_SLOT_US            (TCAST_P1_SLOT_US)
#define PRE_P2_SUBSLOT_US     (TCAST_PRE_P2_SUBSLOT_US)
#define P2_SUBSLOT_US         (TCAST_P2_SUBSLOT_US)
#define NTX                   (TCAST_NTX)
#define P2_NODE_COUNT_MAX     (TCAST_P2_NODE_COUNT)
#define P1_SLOT_TICKS         TCAST_US_TO_TIMER_TICKS(P1_SLOT_US)
#define PRE_P2_SUBSLOT_TICKS  TCAST_US_TO_TIMER_TICKS(PRE_P2_SUBSLOT_US)
#define P2_SUBSLOT_TICKS      TCAST_US_TO_TIMER_TICKS(P2_SUBSLOT_US)
#define P1_SLOT_ACTIVE_US     (P1_SLOT_US - TCAST_SLOT_PROCESSING_US)
#define PRE_P2_SUBSLOT_PERIOD_US (PRE_P2_SUBSLOT_US + TCAST_PRE_P2_SUBSLOT_GUARD_US)
#define P2_SUBSLOT_PERIOD_US  (P2_SUBSLOT_US + TCAST_P2_SUBSLOT_GUARD_US)
#define P1_SYNC_DURATION_TICKS ((uint32_t)(2U * NTX) * P1_SLOT_TICKS)
#define P1_SLOT_ACTIVE_TICKS  TCAST_US_TO_TIMER_TICKS(P1_SLOT_ACTIVE_US)
#define PRE_P2_SLOT_PROCESSING_TICKS TCAST_US_TO_TIMER_TICKS(TCAST_PRE_P2_SLOT_PROCESSING_US)
#define SLOT_PROCESSING_TICKS TCAST_US_TO_TIMER_TICKS(TCAST_SLOT_PROCESSING_US)
#define PRE_P2_SUBSLOT_PERIOD_TICKS TCAST_US_TO_TIMER_TICKS(PRE_P2_SUBSLOT_PERIOD_US)
#define P2_SUBSLOT_PERIOD_TICKS TCAST_US_TO_TIMER_TICKS(P2_SUBSLOT_PERIOD_US)
#define ROUND_TX_TS_MAX       ((uint32_t)NTX * \
                               (1U + (uint32_t)P2_NODE_COUNT_MAX + \
                                (uint32_t)(TCAST_USE_PRE_P2 ? P2_NODE_COUNT_MAX : 0U)))
static timecast_store_t g_store;
static timecast_protocol_state_t g_proto;
static uint32_t g_round_count;
static uint32_t g_round_tx_tick[ROUND_TX_TS_MAX];
static uint16_t g_round_tx_tick_count;
static uint32_t g_round_tx_tick_overflow;
static uint32_t g_p1_offset_sample_count;
static uint32_t g_p1_tx_slot_start_ticks;
static bool g_p1_tx_cal_pending;
static tta_driver_stats_t g_round_start_stats;

typedef struct __attribute__((packed)) {
    uint8_t node_id;
    uint8_t role;
    uint16_t reserved;
    uint32_t round_tag;
    char data[TCAST_LOCAL_PAYLOAD_DATA_CAPACITY];
} timecast_local_payload_t;

static inline bool _is_master(void);

static timecast_protocol_cfg_t g_proto_cfg = {
    .use_pre_p2 = (bool)(TCAST_USE_PRE_P2 != 0U),
    .local_node_id = (uint8_t)TC_NODE_ID,
    .local_hop = (uint8_t)TCAST_LOCAL_HOP,
    .ntx = NTX,
    .p2_node_count = P2_NODE_COUNT_MAX,
    .glossy_slot_ticks = P1_SLOT_TICKS,
    .p1_rx_ts_to_slot_start_ticks = TCAST_US_TO_TIMER_TICKS(TCAST_P1_RX_TS_TO_SLOT_START_US),
    .p1_guard_ticks = TCAST_US_TO_TIMER_TICKS(TCAST_P2_START_GUARD_US),
    .pre_p2_subslot_ticks = PRE_P2_SUBSLOT_TICKS,
    .pre_p2_guard_ticks = TCAST_US_TO_TIMER_TICKS(TCAST_PRE_P2_SUBSLOT_GUARD_US),
    .pre_p2_rx_window_ticks = TCAST_US_TO_TIMER_TICKS(TCAST_P2_RX_WINDOW_US),
    .p2_subslot_ticks = P2_SUBSLOT_TICKS,
    .p2_guard_ticks = TCAST_US_TO_TIMER_TICKS(TCAST_P2_SUBSLOT_GUARD_US),
    .p2_rx_window_ticks = TCAST_US_TO_TIMER_TICKS(TCAST_P2_RX_WINDOW_US),
    .p2_payload_base_ticks =
        TCAST_US_TO_TIMER_TICKS(TCAST_SLOT_PROCESSING_US + TCAST_RADIO_RAMPUP_US +
                                (8U * TCAST_SLOT_PHY_OVERHEAD_BYTES)),
    .p2_payload_byte_ticks = TCAST_US_TO_TIMER_TICKS(8U),
};

static inline bool _log_this_error(uint32_t counter)
{
    if (TCAST_ERR_LOG_EVERY == 0U) {
        return false;
    }

    return ((counter % TCAST_ERR_LOG_EVERY) == 1U);
}

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
             ((uint32_t)TIMECAST_PACKET_P2_DATA_MAX_PAYLOAD_LEN *
              g_proto_cfg.p2_payload_byte_ticks) +
             g_proto_cfg.p2_guard_ticks));
}

static inline uint32_t _p2_subslot_ticks_from_payload_len(uint8_t p2_payload_len)
{
    if ((p2_payload_len < TIMECAST_PACKET_P2_DATA_HDR_LEN) ||
        (p2_payload_len > TIMECAST_PACKET_P2_DATA_MAX_PAYLOAD_LEN)) {
        return 0U;
    }

    return g_proto_cfg.p2_payload_base_ticks +
           ((uint32_t)p2_payload_len * g_proto_cfg.p2_payload_byte_ticks);
}

static uint32_t _estimated_p2_slot_ticks(bool assume_max_missing)
{
    uint8_t source_id;
    uint8_t default_p2_payload_len = assume_max_missing ?
                                     (uint8_t)TIMECAST_PACKET_P2_DATA_MAX_PAYLOAD_LEN :
                                     TCAST_LOCAL_P2_PAYLOAD_LEN;
    uint32_t slot_ticks = 0U;

    if (!g_proto_cfg.use_pre_p2) {
        return (uint32_t)g_proto_cfg.p2_node_count *
               (g_proto_cfg.p2_subslot_ticks + g_proto_cfg.p2_guard_ticks);
    }

    if (g_proto.p2.slot_ticks > 0U) {
        return g_proto.p2.slot_ticks;
    }

    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        uint8_t p2_payload_len = default_p2_payload_len;
        uint32_t subslot_ticks;

        if (timecast_protocol_pre_p2_has_p2_payload_len(&g_proto, source_id)) {
            p2_payload_len = g_proto.pre_p2.p2_payload_len[source_id];
        }
        subslot_ticks = _p2_subslot_ticks_from_payload_len(p2_payload_len);
        if (subslot_ticks == 0U) {
            subslot_ticks = _p2_subslot_ticks_from_payload_len(default_p2_payload_len);
        }

        slot_ticks += subslot_ticks + g_proto_cfg.p2_guard_ticks;
    }

    return slot_ticks;
}

static uint32_t _estimated_p2_duration_ticks(void)
{
    return (uint32_t)(2U * NTX) * _estimated_p2_slot_ticks(_is_master());
}

static uint32_t _round_period_ticks(void)
{
    uint32_t period_ticks = P1_SYNC_DURATION_TICKS + g_proto_cfg.p1_guard_ticks;

    if (g_proto_cfg.use_pre_p2) {
        period_ticks += _pre_p2_duration_ticks(g_proto_cfg.p2_node_count);
        period_ticks += g_proto_cfg.p1_guard_ticks;
        period_ticks += _estimated_p2_duration_ticks();
    }
    else {
        period_ticks += _p2_duration_ticks(g_proto_cfg.p2_node_count);
    }

    return period_ticks + TCAST_US_TO_TIMER_TICKS(TCAST_ROUND_GAP_US);
}

static void _record_tx_tick(uint32_t ts_tick)
{
    if (g_round_tx_tick_count < ROUND_TX_TS_MAX) {
        g_round_tx_tick[g_round_tx_tick_count++] = ts_tick;
    }
    else {
        g_round_tx_tick_overflow++;
    }
}

static void _start_p1_offset_sample(uint32_t slot_start_ticks)
{
    g_p1_tx_slot_start_ticks = slot_start_ticks;
    g_p1_tx_cal_pending = true;
}

static void _finish_p1_offset_sample(uint32_t tx_tick)
{
    uint32_t sample_ticks;
    uint32_t p2_rx_window_ticks;

    if (!g_p1_tx_cal_pending) {
        return;
    }

    g_p1_tx_cal_pending = false;
    sample_ticks = tx_tick - g_p1_tx_slot_start_ticks;
    if (sample_ticks >= P1_SLOT_TICKS) {
        return;
    }

    if (g_p1_offset_sample_count == 0U) {
        g_proto_cfg.p1_rx_ts_to_slot_start_ticks = sample_ticks;
    }
    else {
        g_proto_cfg.p1_rx_ts_to_slot_start_ticks =
            ((3U * g_proto_cfg.p1_rx_ts_to_slot_start_ticks) + sample_ticks + 2U) / 4U;
    }

    p2_rx_window_ticks = g_proto_cfg.p1_rx_ts_to_slot_start_ticks +
                         TCAST_US_TO_TIMER_TICKS(TCAST_P2_RX_WINDOW_MARGIN_US);
    if (p2_rx_window_ticks < TCAST_US_TO_TIMER_TICKS(TCAST_P2_RX_WINDOW_US)) {
        p2_rx_window_ticks = TCAST_US_TO_TIMER_TICKS(TCAST_P2_RX_WINDOW_US);
    }
    g_proto_cfg.pre_p2_rx_window_ticks = p2_rx_window_ticks;
    g_proto_cfg.p2_rx_window_ticks = p2_rx_window_ticks;
    g_p1_offset_sample_count++;
}

static void _wait_until_ticks(uint32_t deadline_ticks)
{
    while ((int32_t)(deadline_ticks - radio_util_now_ticks()) > 0) {
        tta_driver_process();
    }
    tta_driver_process();
}

static inline bool _is_master(void)
{
    return (TC_NODE_ID == 0U);
}

static inline const char *_role_name(void)
{
    return _is_master() ? "master" : "follower";
}

static inline uint32_t _arm_time_ticks(uint32_t start_ticks, uint32_t lead_ticks)
{
    uint32_t now_ticks = radio_util_now_ticks();

    if ((int32_t)(start_ticks - now_ticks) > (int32_t)lead_ticks) {
        return start_ticks - lead_ticks;
    }

    return now_ticks;
}

static void _wait_for_arm(uint32_t start_ticks, uint32_t lead_ticks)
{
    _wait_until_ticks(_arm_time_ticks(start_ticks, lead_ticks));
}

static int _drain_rx_until_idle(uint32_t deadline_ticks)
{
    int res;

    do {
        res = tta_driver_rx_disable();
        if ((res != -EBUSY) || ((int32_t)(radio_util_now_ticks() - deadline_ticks) >= 0)) {
            tta_driver_process();
            return res;
        }
        tta_driver_process();
    } while (1);
}

static void _drain_tx_until_done(uint32_t tx_done_target, uint32_t deadline_ticks)
{
    tta_driver_stats_t stats;

    do {
        tta_driver_get_stats(&stats);
        if ((int32_t)(stats.tx_done - tx_done_target) >= 0) {
            break;
        }
        if ((int32_t)(radio_util_now_ticks() - deadline_ticks) >= 0) {
            break;
        }
        tta_driver_process();
    } while (1);
    tta_driver_process();
}

static int _try_rx_enable(void)
{
    int res = tta_driver_rx_enable();

    if (res == -EBUSY) {
        return 0;
    }
    if ((res < 0) && (res != -EBUSY)) {
        g_proto.rx_enable_fails++;
    }

    return res;
}

static void _reset_round_trace(void)
{
    g_round_tx_tick_count = 0U;
    g_round_tx_tick_overflow = 0U;
}

static void _refresh_local_payload(void)
{
    timecast_local_payload_t payload;
    uint8_t source_id;

    memset(&payload, 0, sizeof(payload));
    payload.role = _is_master() ? 1U : 0U;
    payload.reserved = 0U;
    payload.round_tag = g_round_count;
#if (TCAST_APP_DATA_LEN > 0U)
    memset(payload.data, 'c', TCAST_APP_DATA_LEN);
#endif

    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        uint8_t owner_id;

        if (source_id == 0U) {
            owner_id = 0U;
        }
        else {
            owner_id = (uint8_t)(((uint32_t)(source_id - 1U) %
                                  ((uint32_t)TCAST_VIRTUAL_SOURCE_MOD - 1U)) + 1U);
        }

        if (owner_id != (uint8_t)TC_NODE_ID) {
            continue;
        }

        payload.node_id = source_id;
        (void)timecast_store_write(&g_store, source_id, &payload, TCAST_LOCAL_PAYLOAD_LEN);
    }
}

static uint32_t _current_p2_start_ticks(void)
{
    if (g_proto_cfg.use_pre_p2) {
        return timecast_protocol_pre_p2_get_p2_start_local_ticks(&g_proto);
    }

    return timecast_protocol_p1_get_next_phase_start_local_ticks(&g_proto);
}

static void _wait_out_estimated_p2(uint32_t p2_start_ticks)
{
    uint32_t p2_end_ticks = p2_start_ticks + _estimated_p2_duration_ticks();

    if ((int32_t)(p2_end_ticks - radio_util_now_ticks()) > 0) {
        _wait_until_ticks(p2_end_ticks);
    }
}

static uint32_t _actual_round_duration_ticks(void)
{
    uint32_t start_ticks;

    if (g_proto_cfg.use_pre_p2) {
        start_ticks = timecast_protocol_p1_get_next_phase_start_local_ticks(&g_proto);
    }
    else {
        start_ticks = _current_p2_start_ticks();
    }

    if (start_ticks == 0U) {
        return 0U;
    }

    return radio_util_now_ticks() - start_ticks;
}

static void _log_round_summary(void)
{
    tta_driver_stats_t stats;
    uint8_t p2_node_count = timecast_protocol_p2_get_node_list_len(&g_proto);
    uint32_t round_duration_ticks = _actual_round_duration_ticks();
    uint16_t present_count = timecast_store_present_count(&g_store);
    uint16_t participant_count = timecast_store_participant_count(&g_store);
    uint32_t p2_slot_ticks = _estimated_p2_slot_ticks(_is_master());
    uint32_t pre_p2_reject_total = g_proto.pre_p2.reject_decode +
                                   g_proto.pre_p2.reject_mode +
                                   g_proto.pre_p2.reject_len +
                                   g_proto.pre_p2.reject_window +
                                   g_proto.pre_p2.reject_present;
    uint32_t p2_reject_total = g_proto.p2.reject_decode +
                               g_proto.p2.reject_mode +
                               g_proto.p2.reject_type +
                               g_proto.p2.reject_self +
                               g_proto.p2.reject_epoch +
                               g_proto.p2.reject_slot +
                               g_proto.p2.reject_subslot +
                               g_proto.p2.reject_window +
                               g_proto.p2.reject_present;
    uint32_t tx_sched_fail_total = g_proto.p1_tx_sched_fails +
                                   g_proto.pre_p2_tx_sched_fails +
                                   g_proto.p2_tx_sched_fails;
    bool has_errors = (g_proto.slot_misses > 0U) ||
                      (tx_sched_fail_total > 0U) ||
                      (g_proto.rx_enable_fails > 0U) ||
                      (g_proto.pre_p2.slot_misses > 0U) ||
                      (g_proto.pre_p2.rx_ignored > 0U) ||
                      (pre_p2_reject_total > 0U) ||
                      (g_proto.p2.slot_misses > 0U) ||
                      (g_proto.p2.rx_ignored > 0U) ||
                      (p2_reject_total > 0U);

    printf("[timecast] round{id=%u,role=%s,round=%" PRIu32
           ",epoch=%" PRIu32 ",joined=%u,hop=%u,relay=%" PRId16
           ",rx=%" PRIu32 ",ign=%" PRIu32
           ",pp2=%u/%u,pp2rx=%" PRIu32
           ",p2rx=%" PRIu32 ",p2store=%" PRIu32
           ",present=%u/%u,p2n=%u"
           ",p1slot=%u,pp2sub=%u,p2slot=%" PRIu32 ",rdu=%" PRIu32
           ",tref=%" PRIu32 ",p2start=%" PRIu32
           ",tcofs=%" PRIu32 ",tcs=%" PRIu32,
           (unsigned)TC_NODE_ID,
           _role_name(),
           g_round_count,
           g_proto.current_epoch,
           (unsigned)g_proto.joined,
           (unsigned)g_proto.p1.local_hop,
           g_proto.p1.relay_cnt,
           g_proto.rx_valid,
           g_proto.rx_ignored,
           (unsigned)g_proto.pre_p2.known_count,
           (unsigned)p2_node_count,
           g_proto.pre_p2.rx_valid,
           g_proto.p2.rx_valid,
           g_proto.p2.store_updates,
           (unsigned)present_count,
           (unsigned)participant_count,
           (unsigned)p2_node_count,
           (unsigned)P1_SLOT_US,
           (unsigned)PRE_P2_SUBSLOT_PERIOD_US,
           TCAST_TIMER_TICKS_TO_US(p2_slot_ticks),
           TCAST_TIMER_TICKS_TO_US(round_duration_ticks),
           TCAST_TIMER_TICKS_TO_US(timecast_protocol_p1_get_tref_local_ticks(&g_proto)),
           TCAST_TIMER_TICKS_TO_US(_current_p2_start_ticks()),
           TCAST_TIMER_TICKS_TO_US(g_proto_cfg.p1_rx_ts_to_slot_start_ticks),
           g_p1_offset_sample_count);
    if (has_errors) {
        printf(",err={late=%" PRIu32 ",txf={p1=%" PRIu32 ",pp2=%" PRIu32 ",p2=%" PRIu32 "},rxf=%" PRIu32
               ",pp2miss=%" PRIu32 ",pp2ign=%" PRIu32 ",pp2rej=%" PRIu32
               ",p2miss=%" PRIu32 ",p2ign=%" PRIu32 ",p2rej=%" PRIu32 "}",
               g_proto.slot_misses,
               g_proto.p1_tx_sched_fails,
               g_proto.pre_p2_tx_sched_fails,
               g_proto.p2_tx_sched_fails,
               g_proto.rx_enable_fails,
               g_proto.pre_p2.slot_misses,
               g_proto.pre_p2.rx_ignored,
               pre_p2_reject_total,
               g_proto.p2.slot_misses,
               g_proto.p2.rx_ignored,
               p2_reject_total);
        if (pre_p2_reject_total > 0U) {
            printf(",pp2r={dec=%" PRIu32 ",mode=%" PRIu32
                   ",len=%" PRIu32 ",win=%" PRIu32 ",dup=%" PRIu32 "}",
                   g_proto.pre_p2.reject_decode,
                   g_proto.pre_p2.reject_mode,
                   g_proto.pre_p2.reject_len,
                   g_proto.pre_p2.reject_window,
                   g_proto.pre_p2.reject_present);
        }
        if (p2_reject_total > 0U) {
            printf(",p2r={dec=%" PRIu32 ",mode=%" PRIu32 ",type=%" PRIu32
                   ",self=%" PRIu32 ",ep=%" PRIu32 ",slot=%" PRIu32
                   ",sub=%" PRIu32 ",win=%" PRIu32 ",dup=%" PRIu32 "}",
                   g_proto.p2.reject_decode,
                   g_proto.p2.reject_mode,
                   g_proto.p2.reject_type,
                   g_proto.p2.reject_self,
                   g_proto.p2.reject_epoch,
                   g_proto.p2.reject_slot,
                   g_proto.p2.reject_subslot,
                   g_proto.p2.reject_window,
                   g_proto.p2.reject_present);
        }
    }
    if (present_count < participant_count) {
        tta_driver_get_stats(&stats);
        printf(",rf={addr=%" PRIu32 ",ok=%" PRIu32 ",crc=%" PRIu32 ",evt=%" PRIu32 "}",
               stats.rx_address - g_round_start_stats.rx_address,
               stats.rx_crc_ok - g_round_start_stats.rx_crc_ok,
               stats.rx_crc_fail - g_round_start_stats.rx_crc_fail,
               stats.evt_drop - g_round_start_stats.evt_drop);
    }
    printf("}\n");
}

static void _handle_p1_rx(const tta_event_t *event)
{
    timecast_p1_sync_frame_t frame;


    if (timecast_packet_decode_p1_sync(event->payload, event->payload_len, &frame) &&
        timecast_protocol_p1_handle_rx(&g_proto, &frame, event->timestamp_tick, &g_proto_cfg)) {
        g_proto.rx_valid++;
        return;
    }

    g_proto.rx_ignored++;
}

static void _handle_pre_p2_rx(const tta_event_t *event)
{
    uint8_t p2_payload_len;

    if (!timecast_packet_decode_pre_p2_ctrl(event->payload, event->payload_len, &p2_payload_len)) {
        g_proto.pre_p2.reject_decode++;
        g_proto.pre_p2.rx_ignored++;
        return;
    }

    if (!timecast_protocol_pre_p2_handle_rx(&g_proto, p2_payload_len,
                                            event->timestamp_tick, &g_proto_cfg)) {
        g_proto.pre_p2.rx_ignored++;
    }
}

static void _handle_p2_rx(const tta_event_t *event)
{
    timecast_p2_data_frame_t frame;
    uint8_t data[TIMECAST_PACKET_P2_DATA_MAX_DATA_LEN];

    if (!timecast_packet_decode_p2_data(event->payload, event->payload_len, &frame, data)) {
        g_proto.p2.reject_decode++;
        g_proto.p2.rx_ignored++;
        return;
    }

    if (!timecast_protocol_p2_handle_rx(&g_proto, &g_store, &frame, data,
                                        event->timestamp_tick, &g_proto_cfg)) {
        g_proto.p2.rx_ignored++;
    }
}

static void _on_tta_event(const tta_event_t *event, void *arg)
{
    (void)arg;

    if (event->type == TTA_EVENT_TX_DONE) {
        _finish_p1_offset_sample(event->timestamp_tick);
        _record_tx_tick(event->timestamp_tick);
        return;
    }
    if (event->type == TTA_EVENT_ERROR) {
        puts("[timecast] driver error");
        return;
    }
    if (event->type != TTA_EVENT_RX_DONE) {
        return;
    }

    if (g_proto.phase == TIMECAST_PHASE_P1_SYNC) {
        _handle_p1_rx(event);
        return;
    }

    if (g_proto.phase == TIMECAST_PHASE_PRE_P2) {
        _handle_pre_p2_rx(event);
        return;
    }

    _handle_p2_rx(event);
}

static void _scan_until_reference(void)
{
    uint32_t last_log_ticks = radio_util_now_ticks();

    while (timecast_protocol_p1_is_active(&g_proto) &&
           !timecast_protocol_p1_has_tref(&g_proto)) {
        int res = _try_rx_enable();

        if (res < 0) {
            if (_log_this_error(g_proto.rx_enable_fails)) {
                printf("[timecast] RX scan enable failed: rc=%d fails=%" PRIu32 "\n",
                       res, g_proto.rx_enable_fails);
            }
        }

        tta_driver_process();

        if ((int32_t)(radio_util_now_ticks() - last_log_ticks) >=
            (int32_t)TCAST_US_TO_TIMER_TICKS(TCAST_P1_SCAN_LOG_INTERVAL_US)) {
            printf("[timecast] waiting{id=%u,round=%" PRIu32 "}\n",
                   (unsigned)TC_NODE_ID, g_round_count);
            last_log_ticks = radio_util_now_ticks();
        }
    }
}

static void _run_p1_slot(void)
{
    bool do_tx = timecast_protocol_p1_should_tx(&g_proto);
    uint32_t slot_start_ticks = timecast_protocol_p1_get_slot_start_local_ticks(&g_proto,
                                                                                 &g_proto_cfg);
    uint32_t slot_end_ticks = slot_start_ticks + P1_SLOT_TICKS;
    uint32_t slot_active_end_ticks = slot_start_ticks + P1_SLOT_ACTIVE_TICKS;
    uint32_t now_ticks = radio_util_now_ticks();

    if ((int32_t)(now_ticks - slot_start_ticks) >= 0) {
        g_proto.slot_misses++;
        if (_log_this_error(g_proto.slot_misses)) {
            printf("[timecast] slot miss: slot=%u now=%" PRIu32 " start=%" PRIu32 "\n",
                   (unsigned)timecast_protocol_p1_get_slot_idx(&g_proto),
                   now_ticks, slot_start_ticks);
        }
        (void)timecast_protocol_p1_finish_slot(&g_proto, &g_proto_cfg, do_tx);
        return;
    }

    if (do_tx) {
        timecast_p1_sync_frame_t frame;
        tta_driver_stats_t stats;
        uint8_t payload[TIMECAST_PACKET_P1_SYNC_PAYLOAD_LEN] = {0};
        uint32_t tx_done_target;

        if (!timecast_protocol_p1_prepare_tx(&g_proto, &g_proto_cfg, &frame) ||
            !timecast_packet_encode_p1_sync(payload, sizeof(payload), &frame)) {
            g_proto.p1_tx_sched_fails++;
            if (_log_this_error(g_proto.p1_tx_sched_fails)) {
                puts("[timecast] TX payload build failed");
            }
            (void)timecast_protocol_p1_finish_slot(&g_proto, &g_proto_cfg, true);
            return;
        }

        tta_driver_get_stats(&stats);
        tx_done_target = stats.tx_done + 1U;
        _start_p1_offset_sample(slot_start_ticks);
        if (tta_driver_tx(payload, sizeof(payload), slot_start_ticks) < 0) {
            g_p1_tx_cal_pending = false;
            g_proto.p1_tx_sched_fails++;
            if (_log_this_error(g_proto.p1_tx_sched_fails)) {
                uint32_t now_ticks = radio_util_now_ticks();
                int32_t slack_ticks = (int32_t)(slot_start_ticks - now_ticks);
                printf("[timecast] TX schedule failed: now=%" PRIu32
                       " deadline=%" PRIu32 " slack=%" PRId32 " ticks fails=%" PRIu32 "\n",
                       now_ticks, slot_start_ticks, slack_ticks, g_proto.p1_tx_sched_fails);
            }
        }

        _wait_until_ticks(slot_active_end_ticks);
        _drain_tx_until_done(tx_done_target, slot_end_ticks);
        (void)timecast_protocol_p1_finish_slot(&g_proto, &g_proto_cfg, true);
        return;
    }

    /* P1 only establishes the common schedule. Once a node is synchronized and
     * enters the round-driven slot loop, RX slots stay radio-off. */
    _wait_until_ticks(slot_active_end_ticks);
    (void)timecast_protocol_p1_finish_slot(&g_proto, &g_proto_cfg, false);
}

static void _run_pre_p2_subslot(void)
{
    bool tx_slot = timecast_protocol_pre_p2_is_tx_slot(&g_proto);
    uint32_t subslot_start_ticks =
        timecast_protocol_pre_p2_get_subslot_start_local_ticks(&g_proto, &g_proto_cfg);
    uint32_t subslot_end_ticks = subslot_start_ticks + g_proto_cfg.pre_p2_subslot_ticks;
    uint32_t subslot_active_end_ticks = subslot_end_ticks - PRE_P2_SLOT_PROCESSING_TICKS;
    uint32_t rx_window_end_ticks = subslot_start_ticks + g_proto_cfg.pre_p2_rx_window_ticks;
    uint32_t now_ticks = radio_util_now_ticks();
    uint8_t owner_id = timecast_protocol_pre_p2_get_owner_node_id(&g_proto);

    if ((int32_t)(rx_window_end_ticks - subslot_active_end_ticks) > 0) {
        rx_window_end_ticks = subslot_active_end_ticks;
    }

    if ((int32_t)(now_ticks - subslot_start_ticks) >= 0) {
        g_proto.pre_p2.slot_misses++;
        if (_log_this_error(g_proto.pre_p2.slot_misses)) {
            printf("[timecast] pre-p2 miss: slot=%u sub=%u now=%" PRIu32 " deadline=%" PRIu32 "\n",
                   (unsigned)timecast_protocol_pre_p2_get_slot_idx(&g_proto),
                   (unsigned)timecast_protocol_pre_p2_get_subslot_idx(&g_proto),
                   now_ticks, subslot_start_ticks);
        }
        (void)timecast_protocol_pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (tx_slot) {
        uint8_t p2_payload_len;
        uint8_t payload[TIMECAST_PACKET_PRE_P2_CTRL_LEN] = {0};
        uint32_t tx_active_end_ticks;

        if (!timecast_protocol_pre_p2_prepare_tx(&g_proto, &p2_payload_len) ||
            !timecast_packet_encode_pre_p2_ctrl(payload, sizeof(payload), p2_payload_len)) {
            _wait_until_ticks(subslot_active_end_ticks);
            (void)timecast_protocol_pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }

        if (tta_driver_tx(payload, sizeof(payload), subslot_start_ticks) < 0) {
            g_proto.pre_p2_tx_sched_fails++;
            if (_log_this_error(g_proto.pre_p2_tx_sched_fails)) {
                printf("[timecast] pre-p2 TX schedule failed: slot=%u sub=%u\n",
                       (unsigned)timecast_protocol_pre_p2_get_slot_idx(&g_proto),
                       (unsigned)timecast_protocol_pre_p2_get_subslot_idx(&g_proto));
            }
            (void)timecast_protocol_pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }

        tx_active_end_ticks = subslot_start_ticks +
                              TCAST_US_TO_TIMER_TICKS(TCAST_RADIO_RAMPUP_US +
                                                      TCAST_PACKET_AIR_TIME_US(
                                                          TIMECAST_PACKET_PRE_P2_CTRL_LEN));
        if ((int32_t)(tx_active_end_ticks - subslot_active_end_ticks) > 0) {
            tx_active_end_ticks = subslot_active_end_ticks;
        }
        _wait_until_ticks(tx_active_end_ticks);
        (void)timecast_protocol_pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (timecast_protocol_pre_p2_is_complete(&g_proto) ||
        (owner_id == UINT8_MAX) ||
        timecast_protocol_pre_p2_has_p2_payload_len(&g_proto, owner_id)) {
        _wait_until_ticks(subslot_active_end_ticks);
        (void)timecast_protocol_pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    _wait_for_arm(subslot_start_ticks, TCAST_US_TO_TIMER_TICKS(TCAST_P2_RX_LEAD_US));
    {
        int res = _try_rx_enable();
        if ((res < 0) && _log_this_error(g_proto.rx_enable_fails)) {
            printf("[timecast] pre-p2 RX enable failed: slot=%u sub=%u fails=%" PRIu32 "\n",
                   (unsigned)timecast_protocol_pre_p2_get_slot_idx(&g_proto),
                   (unsigned)timecast_protocol_pre_p2_get_subslot_idx(&g_proto),
                   g_proto.rx_enable_fails);
        }
        if (res < 0) {
            (void)timecast_protocol_pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }
    }

    _wait_until_ticks(rx_window_end_ticks);
    if (tta_driver_rx_disable() == -EBUSY) {
        (void)_drain_rx_until_idle(subslot_end_ticks);
    }

    (void)timecast_protocol_pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
}

static void _run_p2_subslot(void)
{
    bool tx_slot = timecast_protocol_p2_is_tx_slot(&g_proto);
    uint32_t subslot_ticks = timecast_protocol_p2_get_subslot_ticks(&g_proto, &g_proto_cfg);
    uint32_t subslot_start_ticks = timecast_protocol_p2_get_subslot_start_local_ticks(&g_proto,
                                                                                       &g_proto_cfg);
    uint32_t subslot_end_ticks = subslot_start_ticks + subslot_ticks;
    uint32_t subslot_active_end_ticks = subslot_end_ticks - SLOT_PROCESSING_TICKS;
    uint32_t rx_window_end_ticks = subslot_start_ticks + g_proto_cfg.p2_rx_window_ticks;
    uint32_t now_ticks = radio_util_now_ticks();
    uint8_t owner_id = timecast_protocol_p2_get_owner_node_id(&g_proto);

    if ((int32_t)(rx_window_end_ticks - subslot_active_end_ticks) > 0) {
        rx_window_end_ticks = subslot_active_end_ticks;
    }

    if ((int32_t)(now_ticks - subslot_start_ticks) >= 0) {
        g_proto.p2.slot_misses++;
        if (_log_this_error(g_proto.p2.slot_misses)) {
            printf("[timecast] p2 miss: slot=%u sub=%u now=%" PRIu32 " deadline=%" PRIu32 "\n",
                   (unsigned)timecast_protocol_p2_get_slot_idx(&g_proto),
                   (unsigned)timecast_protocol_p2_get_subslot_idx(&g_proto),
                   now_ticks, subslot_start_ticks);
        }
        (void)timecast_protocol_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if (tx_slot) {
        timecast_p2_data_frame_t frame;
        const uint8_t *data_ptr = NULL;
        tta_driver_stats_t stats;
        uint8_t payload[TIMECAST_PACKET_P2_DATA_MAX_PAYLOAD_LEN] = {0};
        uint32_t tx_active_end_ticks;
        uint32_t tx_done_target;

        if (!timecast_protocol_p2_prepare_tx(&g_proto, &g_store, &g_proto_cfg, &frame, &data_ptr)) {
            _wait_until_ticks(subslot_active_end_ticks);
            (void)timecast_protocol_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }

        if (!timecast_packet_encode_p2_data(payload, sizeof(payload), &frame, data_ptr)) {
            g_proto.p2_tx_sched_fails++;
            if (_log_this_error(g_proto.p2_tx_sched_fails)) {
                puts("[timecast] P2 payload build failed");
            }
            (void)timecast_protocol_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }
        tta_driver_get_stats(&stats);
        tx_done_target = stats.tx_done + 1U;
        if (tta_driver_tx(payload,
                          (size_t)(TIMECAST_PACKET_P2_DATA_HDR_LEN + frame.data_len),
                          subslot_start_ticks) < 0) {
            g_proto.p2_tx_sched_fails++;
            if (_log_this_error(g_proto.p2_tx_sched_fails)) {
                printf("[timecast] P2 TX schedule failed: slot=%u sub=%u\n",
                       (unsigned)timecast_protocol_p2_get_slot_idx(&g_proto),
                       (unsigned)timecast_protocol_p2_get_subslot_idx(&g_proto));
            }
            (void)timecast_protocol_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }

        tx_active_end_ticks = subslot_start_ticks +
                              TCAST_US_TO_TIMER_TICKS(TCAST_RADIO_RAMPUP_US +
                                                      TCAST_PACKET_AIR_TIME_US(
                                                          TIMECAST_PACKET_P2_DATA_HDR_LEN +
                                                          frame.data_len));
        if ((int32_t)(tx_active_end_ticks - subslot_active_end_ticks) > 0) {
            tx_active_end_ticks = subslot_active_end_ticks;
        }
        _wait_until_ticks(tx_active_end_ticks);
        _drain_tx_until_done(tx_done_target, subslot_end_ticks);
        (void)timecast_protocol_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    if ((owner_id == UINT8_MAX) || timecast_store_has_data(&g_store, owner_id)) {
        _wait_until_ticks(subslot_active_end_ticks);
        (void)timecast_protocol_p2_finish_subslot(&g_proto, &g_proto_cfg);
        return;
    }

    _wait_for_arm(subslot_start_ticks, TCAST_US_TO_TIMER_TICKS(TCAST_P2_RX_LEAD_US));
    {
        int res = _try_rx_enable();
        if ((res < 0) && _log_this_error(g_proto.rx_enable_fails)) {
            printf("[timecast] P2 RX enable failed: slot=%u sub=%u fails=%" PRIu32 "\n",
                   (unsigned)timecast_protocol_p2_get_slot_idx(&g_proto),
                   (unsigned)timecast_protocol_p2_get_subslot_idx(&g_proto),
                   g_proto.rx_enable_fails);
        }
        if (res < 0) {
            (void)timecast_protocol_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }
    }

    _wait_until_ticks(rx_window_end_ticks);
    if (tta_driver_rx_disable() == -EBUSY) {
        (void)_drain_rx_until_idle(subslot_end_ticks);
    }

    (void)timecast_protocol_p2_finish_subslot(&g_proto, &g_proto_cfg);
}

int main(void)
{
    uint32_t next_master_round_start_ticks;
    uint32_t next_phase_start_ticks;
    uint32_t p2_start_ticks;
    printf("TimeCast start. node_id=%u hop=%u role=%s ntx=%u vmod=%u pre_p2=%u app_data=%u payload=%u\n",
           (unsigned)TC_NODE_ID,
           (unsigned)g_proto_cfg.local_hop,
           _role_name(),
           (unsigned)NTX,
           (unsigned)TCAST_VIRTUAL_SOURCE_MOD,
           (unsigned)g_proto_cfg.use_pre_p2,
           (unsigned)TCAST_APP_DATA_LEN,
           (unsigned)TCAST_LOCAL_PAYLOAD_LEN);
    printf("[timecast] timing: p1_slot=%u us pre_p2_subslot=%u us p2_subslot=%u us "
           "(proc={p1/p2:%u,pp2:%u} ramp=%u p1_air=%u p2_air=%u fast_ru=%u tc=%" PRIu32 ") "
           "tx_min_arm=%u p1{rx_lead=%u} pp2{guard=%u} p2{rx_lead=%u nodes=%u guard=%u} gap=%u\n",
           (unsigned)P1_SLOT_US,
           (unsigned)PRE_P2_SUBSLOT_US,
           (unsigned)P2_SUBSLOT_US,
           (unsigned)TCAST_SLOT_PROCESSING_US,
           (unsigned)TCAST_PRE_P2_SLOT_PROCESSING_US,
           (unsigned)TCAST_RADIO_RAMPUP_US,
           (unsigned)TCAST_PACKET_AIR_TIME_US(TIMECAST_PACKET_P1_SYNC_PAYLOAD_LEN),
           (unsigned)TCAST_PACKET_AIR_TIME_US(TIMECAST_PACKET_P2_DATA_HDR_LEN +
                                              TCAST_LOCAL_PAYLOAD_LEN),
           (unsigned)TCAST_FAST_RAMPUP,
           TCAST_TIMER_TICKS_TO_US(g_proto_cfg.p1_rx_ts_to_slot_start_ticks),
           (unsigned)TCAST_TX_MIN_ARM_LEAD_US,
           (unsigned)TCAST_P1_RX_LEAD_US,
           (unsigned)TCAST_PRE_P2_SUBSLOT_GUARD_US,
           (unsigned)TCAST_P2_RX_LEAD_US,
           (unsigned)P2_NODE_COUNT_MAX,
           (unsigned)TCAST_P2_SUBSLOT_GUARD_US,
           (unsigned)TCAST_ROUND_GAP_US);

    radio_util_init();
    tta_driver_init();
    tta_driver_set_event_cb(_on_tta_event, NULL);
    tta_driver_start();

    timecast_store_init(&g_store, (uint8_t)TC_NODE_ID);
    (void)timecast_store_mark_participant(&g_store, (uint8_t)TC_NODE_ID);
    timecast_protocol_init(&g_proto, _is_master());
    _refresh_local_payload();

    g_round_count = 0U;
    g_p1_offset_sample_count = 0U;
    g_p1_tx_slot_start_ticks = 0U;
    g_p1_tx_cal_pending = false;
    _reset_round_trace();
    next_master_round_start_ticks = radio_util_now_ticks() +
                                    TCAST_US_TO_TIMER_TICKS(TCAST_MASTER_START_DELAY_US);

    while (1) {
        uint8_t node_id;

        g_round_count++;
        _reset_round_trace();
        tta_driver_get_stats(&g_round_start_stats);
        timecast_store_clear(&g_store);
        for (node_id = 0U; node_id < g_proto_cfg.p2_node_count; node_id++) {
            (void)timecast_store_mark_participant(&g_store, node_id);
        }
        _refresh_local_payload();

        if (_is_master()) {
            /* Enter P1 before slot 0 so the first TX is already scheduled
             * when the round start timestamp arrives. */
            timecast_protocol_p1_start(&g_proto, next_master_round_start_ticks,
                                       &g_proto_cfg, true, g_round_count);
        }
        else {
            timecast_protocol_p1_start(&g_proto, 0U, &g_proto_cfg, false, 0U);
            _scan_until_reference();
        }

        while (timecast_protocol_p1_is_active(&g_proto) &&
               timecast_protocol_p1_has_tref(&g_proto)) {
            _run_p1_slot();
        }

        next_phase_start_ticks = timecast_protocol_p1_get_next_phase_start_local_ticks(&g_proto);
        if (g_proto_cfg.use_pre_p2) {
            timecast_protocol_pre_p2_start(&g_proto, &g_store, next_phase_start_ticks,
                                           &g_proto_cfg);
            while (timecast_protocol_pre_p2_is_active(&g_proto)) {
                _run_pre_p2_subslot();
            }
            p2_start_ticks = timecast_protocol_pre_p2_get_p2_start_local_ticks(&g_proto);
        }
        else {
            p2_start_ticks = next_phase_start_ticks;
        }

        tta_driver_process();

        /* Enter P2 before subslot 0 so the first TX/RX is armed ahead of the
         * common P2 start timestamp. */
        timecast_protocol_p2_start(&g_proto, p2_start_ticks, &g_proto_cfg);
        if (g_proto_cfg.use_pre_p2 &&
            !timecast_protocol_pre_p2_is_complete(&g_proto)) {
            _wait_out_estimated_p2(p2_start_ticks);
        }
        while (timecast_protocol_p2_is_active(&g_proto)) {
            _run_p2_subslot();
        }

        _log_round_summary();

        if (_is_master()) {
            next_master_round_start_ticks += _round_period_ticks();
        }
    }

    return 0;
}

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
#ifndef TCAST_USE_PRE_COMMIT
#define TCAST_USE_PRE_COMMIT         (1U)
#endif
#ifndef TCAST_APP_DATA_LEN
#define TCAST_APP_DATA_LEN           (0U)
#endif
#define TCAST_LOCAL_PAYLOAD_META_LEN (8U)
#if (TCAST_APP_DATA_LEN > (TIMECAST_STORE_MAX_DATA_LEN - TCAST_LOCAL_PAYLOAD_META_LEN))
#error "TCAST_APP_DATA_LEN exceeds TIMECAST_STORE_MAX_DATA_LEN budget"
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
#if (TCAST_NTX > 63U)
#error "TCAST_NTX exceeds 7-bit packed relay_cnt budget"
#endif
#ifndef TCAST_P2_NODE_COUNT
#define TCAST_P2_NODE_COUNT           (4U)
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
#ifndef TCAST_ERR_LOG_EVERY
#define TCAST_ERR_LOG_EVERY           (256U)
#endif
#ifndef TCAST_P1_RX_LEAD_US
#define TCAST_P1_RX_LEAD_US           (80U)
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
#ifndef TCAST_MASTER_P2_INCOMPLETE_PRE_THRESHOLD
#define TCAST_MASTER_P2_INCOMPLETE_PRE_THRESHOLD (3U)
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
#define TCAST_CLASS_COUNT      (16U)
#define TCAST_CLASS_MAX_ID     (TCAST_CLASS_COUNT - 1U)
#define TCAST_CLASS_INVALID_ID (TCAST_CLASS_COUNT)
#define TCAST_PACKED_CLASS_LEN(node_count) (((uint32_t)(node_count) + 1U) / 2U)
#ifndef TCAST_TEST_NODE2_CLASS_PERIOD
#define TCAST_TEST_NODE2_CLASS_PERIOD (0U)
#endif
#ifndef TCAST_TEST_NODE2_CLASS_MIN
#define TCAST_TEST_NODE2_CLASS_MIN    (1U)
#endif
#ifndef TCAST_TEST_NODE2_CLASS_MAX
#define TCAST_TEST_NODE2_CLASS_MAX    (TCAST_CLASS_MAX_ID)
#endif
#if (TCAST_TEST_NODE2_CLASS_MIN == 0U)
#error "TCAST_TEST_NODE2_CLASS_MIN must fit the local payload metadata"
#endif
#if (TCAST_TEST_NODE2_CLASS_MAX > TCAST_CLASS_MAX_ID)
#error "TCAST_TEST_NODE2_CLASS_MAX exceeds class range"
#endif
#if (TCAST_TEST_NODE2_CLASS_MIN > TCAST_TEST_NODE2_CLASS_MAX)
#error "TCAST_TEST_NODE2_CLASS_MIN exceeds TCAST_TEST_NODE2_CLASS_MAX"
#endif
static timecast_store_t g_store;
static timecast_protocol_state_t g_proto;
static uint32_t g_round_count;
static uint32_t g_p1_offset_sample_count;
static uint32_t g_p1_tx_slot_start_ticks;
static bool g_p1_tx_cal_pending;
static tta_driver_stats_t g_round_start_stats;
static uint32_t g_round_p2_start_ticks;
static uint8_t g_committed_class[TIMECAST_STORE_MAX_NODES];
static uint8_t g_local_desired_class;
static uint8_t g_local_committed_payload_len;
static uint8_t g_local_committed_payload[TIMECAST_STORE_MAX_DATA_LEN];
static uint8_t g_round_schedule_class[TIMECAST_STORE_MAX_NODES];
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
static bool g_round_pre_commit_applied;
static bool g_round_pre_commit_enabled;
static uint32_t g_round_pre_commit_slot_ticks;

typedef struct {
    bool active;
    bool flag_tx;
    bool have_schedule;
    uint8_t slot_idx;
    uint8_t ntx_done;
    int16_t relay_cnt;
    uint32_t start_local_ticks;
    uint32_t slot_ticks;
    uint32_t rx_valid;
    uint8_t packed_len;
    uint8_t packed_schedule[TIMECAST_PACKET_PRE_COMMIT_MAX_SCHEDULE_LEN];
} tcast_pre_commit_state_t;

static tcast_pre_commit_state_t g_pre_commit;

static inline bool _is_master(void);

static timecast_protocol_cfg_t g_proto_cfg = {
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

typedef enum {
    TCAST_RUN_MODE_ORIGINAL = 0,
    TCAST_RUN_MODE_PRE_P2,
} tcast_run_mode_t;

static const tcast_run_mode_t g_run_mode =
    (TCAST_USE_PRE_P2 != 0U) ? TCAST_RUN_MODE_PRE_P2 : TCAST_RUN_MODE_ORIGINAL;

static inline bool _use_pre_p2_mode(void)
{
    return (g_run_mode == TCAST_RUN_MODE_PRE_P2);
}

static inline bool _use_pre_commit_mode(void)
{
    return _use_pre_p2_mode() && (TCAST_USE_PRE_COMMIT != 0U);
}

static inline const char *_run_mode_name(void)
{
    return _use_pre_p2_mode() ? "pre_p2" : "original";
}

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

static inline uint8_t _class_to_payload_len(uint8_t class_id);

static inline uint32_t _p2_duration_from_classes_ticks(const uint8_t *classes, uint8_t node_count)
{
    uint8_t source_id;
    uint32_t slot_ticks = 0U;

    if (!classes) {
        return 0U;
    }

    for (source_id = 0U; source_id < node_count; source_id++) {
        uint8_t p2_payload_len = _class_to_payload_len(classes[source_id]);

        if (p2_payload_len == 0U) {
            return 0U;
        }
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
    uint32_t payload_len = TIMECAST_PACKET_PRE_COMMIT_BASE_LEN +
                           TCAST_PACKED_CLASS_LEN(node_count);

    return TCAST_US_TO_TIMER_TICKS(TCAST_SLOT_PROCESSING_US +
                                   TCAST_RADIO_RAMPUP_US +
                                   TCAST_PACKET_AIR_TIME_US(payload_len));
}

static uint32_t _pre_commit_duration_ticks(uint32_t slot_ticks)
{
    return (uint32_t)(2U * NTX) * slot_ticks;
}

static uint32_t _original_round_period_ticks(void)
{
    uint32_t period_ticks = P1_SYNC_DURATION_TICKS + g_proto_cfg.p1_guard_ticks;

    period_ticks += _original_p2_duration_ticks();

    return period_ticks + TCAST_US_TO_TIMER_TICKS(TCAST_ROUND_GAP_US);
}

static uint32_t _improved_round_period_ticks(bool run_pre)
{
    uint32_t period_ticks = P1_SYNC_DURATION_TICKS + g_proto_cfg.p1_guard_ticks;

    if (run_pre) {
        period_ticks += _pre_p2_duration_ticks(g_proto_cfg.p2_node_count);
        period_ticks += g_proto_cfg.p1_guard_ticks;
        if (g_round_pre_commit_enabled) {
            period_ticks += _pre_commit_duration_ticks(g_round_pre_commit_slot_ticks);
            period_ticks += g_proto_cfg.p1_guard_ticks;
        }
    }

    period_ticks += (uint32_t)(2U * NTX) * timecast_protocol_p2_get_slot_ticks(&g_proto, &g_proto_cfg);
    return period_ticks + TCAST_US_TO_TIMER_TICKS(TCAST_ROUND_GAP_US);
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

static inline uint8_t _class_span_bytes(void)
{
    return (uint8_t)(((TIMECAST_PACKET_P2_DATA_MAX_PAYLOAD_LEN -
                       TIMECAST_PACKET_P2_DATA_HDR_LEN + 1U) +
                      TCAST_CLASS_COUNT - 1U) / TCAST_CLASS_COUNT);
}

static inline uint8_t _class_to_payload_len(uint8_t class_id)
{
    uint32_t payload_len;

    if (class_id > TCAST_CLASS_MAX_ID) {
        return 0U;
    }
    if (class_id == TCAST_CLASS_MAX_ID) {
        return TIMECAST_PACKET_P2_DATA_MAX_PAYLOAD_LEN;
    }

    payload_len = (uint32_t)TIMECAST_PACKET_P2_DATA_HDR_LEN +
                  ((uint32_t)(class_id + 1U) * (uint32_t)_class_span_bytes()) - 1U;

    return (uint8_t)payload_len;
}

static inline uint8_t _payload_len_to_class(uint8_t payload_len)
{
    uint32_t offset;
    uint32_t class_id;

    if (payload_len <= TIMECAST_PACKET_P2_DATA_HDR_LEN) {
        return 0U;
    }
    if (payload_len == TIMECAST_PACKET_P2_DATA_MAX_PAYLOAD_LEN) {
        return TCAST_CLASS_MAX_ID;
    }
    if (payload_len > TIMECAST_PACKET_P2_DATA_MAX_PAYLOAD_LEN) {
        return TCAST_CLASS_INVALID_ID;
    }

    offset = (uint32_t)payload_len - (uint32_t)TIMECAST_PACKET_P2_DATA_HDR_LEN;
    class_id = offset / (uint32_t)_class_span_bytes();
    if (class_id > TCAST_CLASS_MAX_ID) {
        return TCAST_CLASS_INVALID_ID;
    }

    return (uint8_t)class_id;
}

static inline uint8_t _payload_data_len_to_class(uint8_t data_len)
{
    if (data_len > TIMECAST_PACKET_P2_DATA_MAX_DATA_LEN) {
        return TCAST_CLASS_INVALID_ID;
    }

    return _payload_len_to_class((uint8_t)(TIMECAST_PACKET_P2_DATA_HDR_LEN + data_len));
}

static inline uint8_t _minimum_payload_class(void)
{
    return _payload_data_len_to_class(TCAST_LOCAL_PAYLOAD_META_LEN);
}

static inline uint8_t _initial_committed_class(void)
{
    return _is_master() ? TCAST_CLASS_MAX_ID : _minimum_payload_class();
}

static inline uint8_t _local_source_id(void)
{
    return (uint8_t)TC_NODE_ID;
}

static inline bool _has_local_source(void)
{
    return (_local_source_id() < g_proto_cfg.p2_node_count);
}

static uint8_t _local_pending_update_count(void)
{
    uint8_t source_id = _local_source_id();

    if (!_has_local_source()) {
        return 0U;
    }

    return (g_local_desired_class != g_committed_class[source_id]) ? 1U : 0U;
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
            if (source_id != _local_source_id()) {
                dst[source_id] = '.';
                continue;
            }
            class_id = g_local_desired_class;
        }
        else {
            class_id = classes[source_id];
        }
        dst[source_id] = (class_id <= TCAST_CLASS_MAX_ID) ? lut[class_id] : '?';
    }
    dst[g_proto_cfg.p2_node_count] = '\0';
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

static uint8_t _local_payload_len_for_source(uint8_t source_id)
{
    (void)source_id;
#if (TCAST_TEST_NODE2_CLASS_PERIOD > 0U)
    if (source_id == 2U) {
        uint32_t round_idx = (g_round_count > 0U) ? (g_round_count - 1U) : 0U;
        uint32_t class_span = (uint32_t)TCAST_TEST_NODE2_CLASS_MAX -
                              (uint32_t)TCAST_TEST_NODE2_CLASS_MIN + 1U;
        uint8_t class_id = (uint8_t)((uint32_t)TCAST_TEST_NODE2_CLASS_MIN +
                                     ((round_idx / (uint32_t)TCAST_TEST_NODE2_CLASS_PERIOD) %
                                      class_span));
        uint8_t p2_payload_len = _class_to_payload_len(class_id);

        return (uint8_t)(p2_payload_len - TIMECAST_PACKET_P2_DATA_HDR_LEN);
    }
#endif

    return TCAST_LOCAL_PAYLOAD_LEN;
}

static uint8_t _build_local_payload_with_len(uint8_t source_id, uint8_t *dst, uint8_t payload_len)
{
    if (!dst) {
        return 0U;
    }

    if ((payload_len < TCAST_LOCAL_PAYLOAD_META_LEN) ||
        (payload_len > TIMECAST_STORE_MAX_DATA_LEN)) {
        return 0U;
    }

    memset(dst, 0, payload_len);
    dst[0] = source_id;
    dst[1] = _is_master() ? 1U : 0U;
    dst[4] = (uint8_t)(g_round_count & 0xFFU);
    dst[5] = (uint8_t)((g_round_count >> 8) & 0xFFU);
    dst[6] = (uint8_t)((g_round_count >> 16) & 0xFFU);
    dst[7] = (uint8_t)((g_round_count >> 24) & 0xFFU);
    if (payload_len > TCAST_LOCAL_PAYLOAD_META_LEN) {
        memset(&dst[TCAST_LOCAL_PAYLOAD_META_LEN], 'c',
               (size_t)(payload_len - TCAST_LOCAL_PAYLOAD_META_LEN));
    }

    return payload_len;
}

static uint8_t _build_local_payload(uint8_t source_id, uint8_t *dst)
{
    return _build_local_payload_with_len(source_id, dst, _local_payload_len_for_source(source_id));
}

static void _pack_class_schedule(const uint8_t *classes, uint8_t node_count,
                                 uint8_t *packed_out, uint8_t *packed_len_out)
{
    uint8_t source_id;
    uint8_t packed_len = (uint8_t)TCAST_PACKED_CLASS_LEN(node_count);

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

    g_proto.pre_p2.node_count = g_proto_cfg.p2_node_count;
    g_proto.pre_p2.complete = true;
    g_proto.pre_p2.known_count = g_proto_cfg.p2_node_count;
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
        if (timecast_protocol_pre_p2_has_p2_payload_len(&g_proto, source_id)) {
            uint8_t class_id = _payload_len_to_class(g_proto.pre_p2.p2_payload_len[source_id]);

            classes_out[source_id] = class_id;
        }
        else {
            classes_out[source_id] = TCAST_CLASS_MAX_ID;
        }
    }
}

static void _apply_round_schedule_to_proto(const uint8_t *classes)
{
    uint8_t source_id;

    g_proto.pre_p2.node_count = g_proto_cfg.p2_node_count;
    g_proto.pre_p2.complete = true;
    g_proto.pre_p2.known_count = g_proto_cfg.p2_node_count;
    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        uint8_t p2_payload_len = _class_to_payload_len(classes[source_id]);

        g_proto.pre_p2.present[source_id] = 1U;
        g_proto.pre_p2.p2_payload_len[source_id] = p2_payload_len;
    }
}

static bool _local_update_pending(void)
{
    return (_local_pending_update_count() > 0U);
}

static void _latch_update_request(void)
{
    g_update_pending_latched = true;
}

static void _refresh_update_latch_from_committed_schedule(void)
{
    g_update_pending_latched = _local_update_pending();
}

static void _freeze_round_update_advertisement(void)
{
    g_round_tx_update_req = g_update_pending_latched && !g_round_run_pre;
}

static bool _local_packet_should_request_update(uint8_t owner_id)
{
    return (owner_id == _local_source_id()) && g_round_tx_update_req;
}

static void _master_request_pre_next_round(void)
{
    if (_is_master()) {
        g_master_run_pre_next_round = true;
    }
}

static void _commit_schedule(const uint8_t *classes)
{
    uint8_t source_id;
    uint8_t local_source_id = _local_source_id();
    uint8_t payload[TIMECAST_STORE_MAX_DATA_LEN];
    uint8_t committed_p2_payload_len;
    uint8_t committed_data_len;
    uint8_t payload_len;

    for (source_id = 0U; source_id < g_proto_cfg.p2_node_count; source_id++) {
        g_committed_class[source_id] = classes[source_id];
        g_round_schedule_class[source_id] = classes[source_id];
    }

    if (local_source_id < g_proto_cfg.p2_node_count) {
        committed_p2_payload_len = _class_to_payload_len(classes[local_source_id]);
        committed_data_len = (uint8_t)(committed_p2_payload_len -
                                       TIMECAST_PACKET_P2_DATA_HDR_LEN);
        payload_len = _local_payload_len_for_source(local_source_id);
        if (payload_len > committed_data_len) {
            payload_len = committed_data_len;
        }
        payload_len = _build_local_payload_with_len(local_source_id, payload, payload_len);
        memcpy(g_local_committed_payload, payload, payload_len);
        g_local_committed_payload_len = payload_len;
    }

    _refresh_update_latch_from_committed_schedule();
}

static void _commit_round_schedule(const uint8_t *classes)
{
    _commit_schedule(classes);
    _apply_round_schedule_to_proto(classes);
}

static void _refresh_local_payload(void)
{
    uint8_t source_id = _local_source_id();
    uint8_t payload[TIMECAST_STORE_MAX_DATA_LEN];
    uint8_t payload_len;
    uint8_t payload_class;
    uint8_t committed_payload_len;

    if (source_id >= g_proto_cfg.p2_node_count) {
        return;
    }

    payload_len = _build_local_payload(source_id, payload);
    payload_class = _payload_data_len_to_class(payload_len);
    if (payload_class > TCAST_CLASS_MAX_ID) {
        _latch_update_request();
        return;
    }
    g_local_desired_class = payload_class;
    if (payload_class != g_committed_class[source_id]) {
        _latch_update_request();
    }

    committed_payload_len = _class_to_payload_len(g_committed_class[source_id]);
    if (committed_payload_len == 0U) {
        _latch_update_request();
        return;
    }
    if ((uint8_t)(TIMECAST_PACKET_P2_DATA_HDR_LEN + payload_len) <= committed_payload_len) {
        (void)timecast_store_write(&g_store, source_id, payload, payload_len);
    }
    else if (g_local_committed_payload_len > 0U) {
        (void)timecast_store_write(&g_store, source_id,
                                   g_local_committed_payload,
                                   g_local_committed_payload_len);
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

    if ((int32_t)(p2_end_ticks - radio_util_now_ticks()) > 0) {
        _wait_until_ticks(p2_end_ticks);
    }
}

static uint32_t _elapsed_since_ticks(uint32_t start_ticks, uint32_t now_ticks)
{
    if (start_ticks == 0U) {
        return 0U;
    }

    return now_ticks - start_ticks;
}

static void _pre_commit_start(uint32_t start_local_ticks, const uint8_t *classes)
{
    memset(&g_pre_commit, 0, sizeof(g_pre_commit));
    g_pre_commit.active = g_round_run_pre && g_round_pre_commit_enabled;
    g_pre_commit.start_local_ticks = start_local_ticks;
    g_pre_commit.slot_ticks = _pre_commit_slot_ticks(g_proto_cfg.p2_node_count);
    if (g_pre_commit.active) {
        g_proto.phase = TIMECAST_PHASE_PRE_COMMIT;
    }
    if (!_is_master() || !g_pre_commit.active || !classes) {
        return;
    }

    _pack_class_schedule(classes, g_proto_cfg.p2_node_count,
                         g_pre_commit.packed_schedule, &g_pre_commit.packed_len);
    g_pre_commit.have_schedule = true;
    g_pre_commit.flag_tx = true;
    g_pre_commit.relay_cnt = -1;
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

static void _handle_pre_commit_rx(const tta_event_t *event)
{
    timecast_pre_commit_frame_t frame;
    uint8_t packed_schedule[TIMECAST_PACKET_PRE_COMMIT_MAX_SCHEDULE_LEN];
    size_t packed_len = 0U;
    uint32_t slot_start_ticks;
    int32_t rx_offset_ticks;

    if (!timecast_packet_decode_pre_commit(event->payload, event->payload_len,
                                           g_proto_cfg.p2_node_count,
                                           &frame, packed_schedule, &packed_len)) {
        return;
    }
    if (!g_pre_commit.active) {
        return;
    }

    slot_start_ticks = _pre_commit_slot_start_ticks();
    rx_offset_ticks = (int32_t)(event->timestamp_tick - slot_start_ticks);
    if ((rx_offset_ticks < 0) ||
        (rx_offset_ticks > (int32_t)g_proto_cfg.p2_rx_window_ticks)) {
        return;
    }

    if (!g_pre_commit.have_schedule) {
        memcpy(g_pre_commit.packed_schedule, packed_schedule, packed_len);
        g_pre_commit.packed_len = (uint8_t)packed_len;
        g_pre_commit.have_schedule = true;
    }
    if (((int16_t)frame.relay_cnt > g_pre_commit.relay_cnt) ||
        (g_pre_commit.relay_cnt < 0)) {
        g_pre_commit.relay_cnt = (int16_t)frame.relay_cnt;
    }
    g_pre_commit.flag_tx = true;
    g_pre_commit.rx_valid++;
}

static void _pre_commit_finish_slot(bool did_tx)
{
    if (!g_pre_commit.active) {
        return;
    }

    if (did_tx) {
        g_pre_commit.flag_tx = false;
    }
    else if (g_pre_commit.have_schedule) {
        g_pre_commit.flag_tx = true;
    }

    g_pre_commit.slot_idx++;
    if (g_pre_commit.slot_idx >= (uint8_t)(2U * NTX)) {
        g_pre_commit.active = false;
    }
}

static void _run_pre_commit_slot(void)
{
    uint32_t slot_start_ticks = _pre_commit_slot_start_ticks();
    uint32_t slot_end_ticks = slot_start_ticks + g_pre_commit.slot_ticks;
    uint32_t slot_active_end_ticks = slot_end_ticks - SLOT_PROCESSING_TICKS;
    uint32_t rx_window_end_ticks = slot_start_ticks + g_proto_cfg.p2_rx_window_ticks;
    uint32_t now_ticks = radio_util_now_ticks();
    bool do_tx = g_pre_commit.have_schedule && g_pre_commit.flag_tx &&
                 (g_pre_commit.ntx_done < NTX);

    if ((int32_t)(rx_window_end_ticks - slot_active_end_ticks) > 0) {
        rx_window_end_ticks = slot_active_end_ticks;
    }

    if ((int32_t)(now_ticks - slot_start_ticks) >= 0) {
        _pre_commit_finish_slot(do_tx);
        return;
    }

    if (do_tx) {
        timecast_pre_commit_frame_t frame;
        uint8_t payload[TIMECAST_PACKET_PRE_COMMIT_MAX_PAYLOAD_LEN] = {0};
        uint32_t tx_active_end_ticks;

        g_pre_commit.relay_cnt++;
        frame.relay_cnt = (uint8_t)g_pre_commit.relay_cnt;
        if (!timecast_packet_encode_pre_commit(payload, sizeof(payload), &frame,
                                               g_pre_commit.packed_schedule,
                                               g_pre_commit.packed_len)) {
            _pre_commit_finish_slot(true);
            return;
        }
        if (tta_driver_tx(payload,
                          TIMECAST_PACKET_PRE_COMMIT_BASE_LEN + g_pre_commit.packed_len,
                          slot_start_ticks) < 0) {
            _pre_commit_finish_slot(true);
            return;
        }

        tx_active_end_ticks = slot_start_ticks +
                              TCAST_US_TO_TIMER_TICKS(
                                  TCAST_RADIO_RAMPUP_US +
                                  TCAST_PACKET_AIR_TIME_US(
                                      TIMECAST_PACKET_PRE_COMMIT_BASE_LEN +
                                      g_pre_commit.packed_len));
        if ((int32_t)(tx_active_end_ticks - slot_active_end_ticks) > 0) {
            tx_active_end_ticks = slot_active_end_ticks;
        }
        _wait_until_ticks(tx_active_end_ticks);
        g_pre_commit.ntx_done++;
        _pre_commit_finish_slot(true);
        return;
    }

    _wait_for_arm(slot_start_ticks, TCAST_US_TO_TIMER_TICKS(TCAST_P1_RX_LEAD_US));
    if (_try_rx_enable() < 0) {
        _pre_commit_finish_slot(false);
        return;
    }

    _wait_until_ticks(rx_window_end_ticks);
    if (tta_driver_rx_disable() == -EBUSY) {
        (void)_drain_rx_until_idle(slot_end_ticks);
    }

    _pre_commit_finish_slot(false);
}

static void _log_rf_stats_if_needed(uint16_t present_count, uint16_t participant_count)
{
    tta_driver_stats_t stats;

    if (present_count >= participant_count) {
        return;
    }

    tta_driver_get_stats(&stats);
    printf(",rf={addr=%" PRIu32 ",ok=%" PRIu32 ",crc=%" PRIu32 ",evt=%" PRIu32 "}",
           stats.rx_address - g_round_start_stats.rx_address,
           stats.rx_crc_ok - g_round_start_stats.rx_crc_ok,
           stats.rx_crc_fail - g_round_start_stats.rx_crc_fail,
           stats.evt_drop - g_round_start_stats.evt_drop);
}

static void _log_round_summary_original(void)
{
    uint8_t p2_node_count = timecast_protocol_p2_get_node_list_len(&g_proto);
    uint32_t now_ticks = radio_util_now_ticks();
    uint32_t round_duration_ticks =
        _elapsed_since_ticks(timecast_protocol_p1_get_next_phase_start_local_ticks(&g_proto),
                             now_ticks);
    uint32_t p2_duration_ticks = _elapsed_since_ticks(g_round_p2_start_ticks, now_ticks);
    uint16_t present_count = timecast_store_present_count(&g_store);
    uint16_t participant_count = timecast_store_participant_count(&g_store);
    uint32_t p2_slot_ticks = _fixed_p2_slot_ticks();
    uint32_t p2_reject_total = g_proto.p2.reject_decode +
                               g_proto.p2.reject_mode +
                               g_proto.p2.reject_type +
                               g_proto.p2.reject_self +
                               g_proto.p2.reject_epoch +
                               g_proto.p2.reject_slot +
                               g_proto.p2.reject_subslot +
                               g_proto.p2.reject_window +
                               g_proto.p2.reject_present;
    uint32_t tx_sched_fail_total = g_proto.p1_tx_sched_fails + g_proto.p2_tx_sched_fails;
    bool has_errors = (g_proto.slot_misses > 0U) ||
                      (tx_sched_fail_total > 0U) ||
                      (g_proto.rx_enable_fails > 0U) ||
                      (g_proto.p2.slot_misses > 0U) ||
                      (g_proto.p2.rx_ignored > 0U) ||
                      (p2_reject_total > 0U);

    printf("[timecast] round{id=%u,role=%s,round=%" PRIu32
           ",epoch=%" PRIu32 ",joined=%u,hop=%u,relay=%" PRId16
           ",rx=%" PRIu32 ",ign=%" PRIu32
           ",p2rx=%" PRIu32 ",p2store=%" PRIu32
           ",present=%u/%u,p2n=%u"
           ",p1slot=%u,p2slot=%" PRIu32 ",rdu=%" PRIu32 ",p2rdu=%" PRIu32
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
           g_proto.p2.rx_valid,
           g_proto.p2.store_updates,
           (unsigned)present_count,
           (unsigned)participant_count,
           (unsigned)p2_node_count,
           (unsigned)P1_SLOT_US,
           TCAST_TIMER_TICKS_TO_US(p2_slot_ticks),
           TCAST_TIMER_TICKS_TO_US(round_duration_ticks),
           TCAST_TIMER_TICKS_TO_US(p2_duration_ticks),
           TCAST_TIMER_TICKS_TO_US(timecast_protocol_p1_get_tref_local_ticks(&g_proto)),
           TCAST_TIMER_TICKS_TO_US(g_round_p2_start_ticks),
           TCAST_TIMER_TICKS_TO_US(g_proto_cfg.p1_rx_ts_to_slot_start_ticks),
           g_p1_offset_sample_count);
    if (has_errors) {
        printf(",err={late=%" PRIu32 ",txf={p1=%" PRIu32 ",p2=%" PRIu32 "},rxf=%" PRIu32
               ",p2miss=%" PRIu32 ",p2ign=%" PRIu32 ",p2rej=%" PRIu32 "}",
               g_proto.slot_misses,
               g_proto.p1_tx_sched_fails,
               g_proto.p2_tx_sched_fails,
               g_proto.rx_enable_fails,
               g_proto.p2.slot_misses,
               g_proto.p2.rx_ignored,
               p2_reject_total);
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
    _log_rf_stats_if_needed(present_count, participant_count);
    printf("}\n");
}

static void _log_round_summary_pre_p2(void)
{
    uint8_t p2_node_count = timecast_protocol_p2_get_node_list_len(&g_proto);
    uint8_t local_pending_updates = _local_pending_update_count();
    uint32_t now_ticks = radio_util_now_ticks();
    uint32_t round_duration_ticks =
        _elapsed_since_ticks(timecast_protocol_p1_get_next_phase_start_local_ticks(&g_proto),
                             now_ticks);
    uint32_t p2_duration_ticks = _elapsed_since_ticks(g_round_p2_start_ticks, now_ticks);
    uint16_t present_count = timecast_store_present_count(&g_store);
    uint16_t participant_count = timecast_store_participant_count(&g_store);
    uint32_t p2_slot_ticks = timecast_protocol_p2_get_slot_ticks(&g_proto, &g_proto_cfg);
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
    bool log_update_state = g_round_run_pre ||
                            g_round_pre_commit_enabled ||
                            g_round_pre_commit_applied ||
                            g_update_pending_latched ||
                            (local_pending_updates > 0U) ||
                            g_master_run_pre_next_round ||
                            (g_master_p2_incomplete_rounds > 0U);
    char committed_class_map[TIMECAST_STORE_MAX_NODES + 1];
    char desired_class_map[TIMECAST_STORE_MAX_NODES + 1];

    printf("[timecast] round{id=%u,role=%s,round=%" PRIu32
           ",epoch=%" PRIu32 ",joined=%u,hop=%u,relay=%" PRId16
           ",rx=%" PRIu32 ",ign=%" PRIu32
           ",pre=%u,pc=%u,upd=%u"
           ",pp2=%u/%u,pp2rx=%" PRIu32
           ",p2rx=%" PRIu32 ",p2store=%" PRIu32
           ",present=%u/%u,p2n=%u"
           ",p1slot=%u,pp2sub=%u,p2slot=%" PRIu32 ",rdu=%" PRIu32 ",p2rdu=%" PRIu32
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
           (unsigned)g_round_run_pre,
           (unsigned)g_round_pre_commit_applied,
           (unsigned)g_update_pending_latched,
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
           TCAST_TIMER_TICKS_TO_US(p2_duration_ticks),
           TCAST_TIMER_TICKS_TO_US(timecast_protocol_p1_get_tref_local_ticks(&g_proto)),
           TCAST_TIMER_TICKS_TO_US(g_round_p2_start_ticks),
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
    _log_rf_stats_if_needed(present_count, participant_count);
    printf("}\n");
}

static void _handle_p1_rx(const tta_event_t *event)
{
    timecast_p1_sync_frame_t frame;

    if (timecast_packet_decode_p1_sync(event->payload, event->payload_len, &frame) &&
        timecast_protocol_p1_handle_rx(&g_proto, &frame, event->timestamp_tick, &g_proto_cfg)) {
        g_round_run_pre = ((frame.flags & TIMECAST_PACKET_P1_FLAG_RUN_PRE) != 0U);
        g_proto.rx_valid++;
        return;
    }

    g_proto.rx_ignored++;
}

static void _handle_pre_p2_rx(const tta_event_t *event)
{
    uint8_t class_id;
    uint8_t p2_payload_len;

    if (!timecast_packet_decode_pre_p2_ctrl(event->payload, event->payload_len, &class_id)) {
        g_proto.pre_p2.reject_decode++;
        g_proto.pre_p2.rx_ignored++;
        return;
    }
    if (class_id > TCAST_CLASS_MAX_ID) {
        g_proto.pre_p2.reject_len++;
        g_proto.pre_p2.rx_ignored++;
        return;
    }
    p2_payload_len = _class_to_payload_len(class_id);
    if (p2_payload_len == 0U) {
        g_proto.pre_p2.reject_len++;
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
        return;
    }

    if ((frame.flags & TIMECAST_PACKET_P2_FLAG_UPDATE_REQ) != 0U) {
        _latch_update_request();
        _master_request_pre_next_round();
    }
}

static void _on_tta_event(const tta_event_t *event, void *arg)
{
    (void)arg;

    if (event->type == TTA_EVENT_TX_DONE) {
        _finish_p1_offset_sample(event->timestamp_tick);
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

    if (g_proto.phase == TIMECAST_PHASE_PRE_COMMIT) {
        _handle_pre_commit_rx(event);
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

        if (!timecast_protocol_p1_prepare_tx(&g_proto, &g_proto_cfg, &frame)) {
            g_proto.p1_tx_sched_fails++;
            if (_log_this_error(g_proto.p1_tx_sched_fails)) {
                puts("[timecast] TX payload build failed");
            }
            (void)timecast_protocol_p1_finish_slot(&g_proto, &g_proto_cfg, true);
            return;
        }
        frame.flags = g_round_run_pre ? TIMECAST_PACKET_P1_FLAG_RUN_PRE : 0U;
        if (!timecast_packet_encode_p1_sync(payload, sizeof(payload), &frame)) {
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
        uint8_t class_id;
        uint8_t payload[TIMECAST_PACKET_PRE_P2_CTRL_LEN] = {0};
        uint32_t tx_active_end_ticks;

        if (!timecast_protocol_pre_p2_prepare_tx(&g_proto, &p2_payload_len)) {
            _wait_until_ticks(subslot_active_end_ticks);
            (void)timecast_protocol_pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }
        class_id = _payload_len_to_class(p2_payload_len);
        if (class_id > TCAST_CLASS_MAX_ID) {
            g_proto.pre_p2_tx_sched_fails++;
            (void)timecast_protocol_pre_p2_finish_subslot(&g_proto, &g_proto_cfg);
            return;
        }
        if (!timecast_packet_encode_pre_p2_ctrl(payload, sizeof(payload), class_id)) {
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
        frame.flags = _local_packet_should_request_update(owner_id) ?
                      TIMECAST_PACKET_P2_FLAG_UPDATE_REQ : 0U;

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

static void _mark_configured_participants(void)
{
    uint8_t node_id;

    timecast_store_clear(&g_store);
    for (node_id = 0U; node_id < g_proto_cfg.p2_node_count; node_id++) {
        (void)timecast_store_mark_participant(&g_store, node_id);
    }
}

static void _reset_round_runtime_state(void)
{
    tta_driver_get_stats(&g_round_start_stats);
    memset(&g_pre_commit, 0, sizeof(g_pre_commit));
    g_round_tx_update_req = false;
    _mark_configured_participants();

    g_round_pre_commit_applied = false;
    g_round_pre_commit_enabled = false;
    g_round_pre_commit_slot_ticks = _pre_commit_slot_ticks(g_proto_cfg.p2_node_count);
}

static void _select_pre_p2_for_round(void)
{
    if (_use_pre_p2_mode()) {
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

static void _master_queue_pre_if_update_is_pending(void)
{
    if (_use_pre_p2_mode() && _is_master() &&
        g_update_pending_latched && !g_round_run_pre) {
        _master_request_pre_next_round();
    }
}

static bool _p2_chain_is_complete(void)
{
    return timecast_store_present_count(&g_store) >= timecast_store_participant_count(&g_store);
}

static void _master_track_p2_completeness(void)
{
    if (!_use_pre_p2_mode() || !_is_master() ||
        (TCAST_MASTER_P2_INCOMPLETE_PRE_THRESHOLD == 0U)) {
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
    if (g_master_p2_incomplete_rounds >= TCAST_MASTER_P2_INCOMPLETE_PRE_THRESHOLD) {
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
    _master_queue_pre_if_update_is_pending();
}

static void _run_p1_phase(uint32_t next_master_round_start_ticks)
{
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
}

static void _run_round_original(uint32_t *next_master_round_start_ticks)
{
    uint32_t p2_start_ticks;

    _prepare_round();
    _run_p1_phase(*next_master_round_start_ticks);

    p2_start_ticks = timecast_protocol_p1_get_next_phase_start_local_ticks(&g_proto);
    g_round_p2_start_ticks = p2_start_ticks;
    tta_driver_process();

    /* Enter fixed P2 before subslot 0 so the first TX/RX is armed ahead of
     * the common P2 start timestamp. */
    timecast_protocol_p2_start_original(&g_proto, p2_start_ticks, &g_proto_cfg);
    while (timecast_protocol_p2_is_active(&g_proto)) {
        _run_p2_subslot();
    }

    _log_round_summary_original();

    if (_is_master()) {
        *next_master_round_start_ticks += _original_round_period_ticks();
    }
}

static void _override_local_pre_p2_payload_len(void)
{
    uint8_t local_source_id = _local_source_id();
    uint8_t p2_payload_len;

    if ((local_source_id >= g_proto_cfg.p2_node_count) ||
        !g_proto.pre_p2.present[local_source_id]) {
        return;
    }

    p2_payload_len = _class_to_payload_len(g_local_desired_class);
    if (p2_payload_len == 0U) {
        g_proto.pre_p2.present[local_source_id] = 0U;
        if (g_proto.pre_p2.known_count > 0U) {
            g_proto.pre_p2.known_count--;
        }
        g_proto.pre_p2.complete = false;
        return;
    }

    g_proto.pre_p2.p2_payload_len[local_source_id] = p2_payload_len;
}

static uint32_t _run_pre_p2_collect(uint32_t start_ticks)
{
    timecast_protocol_pre_p2_start(&g_proto, &g_store, start_ticks, &g_proto_cfg);
    _override_local_pre_p2_payload_len();

    while (timecast_protocol_pre_p2_is_active(&g_proto)) {
        _run_pre_p2_subslot();
    }

    return timecast_protocol_pre_p2_get_p2_start_local_ticks(&g_proto);
}

static void _run_round_pre_p2(uint32_t *next_master_round_start_ticks)
{
    uint32_t next_phase_start_ticks;
    uint32_t p2_start_ticks;
    bool skip_p2 = false;

    _prepare_round();
    _run_p1_phase(*next_master_round_start_ticks);

    _freeze_round_update_advertisement();

    next_phase_start_ticks = timecast_protocol_p1_get_next_phase_start_local_ticks(&g_proto);
    if (g_round_run_pre) {
        p2_start_ticks = _run_pre_p2_collect(next_phase_start_ticks);
        if (_use_pre_commit_mode()) {
            g_round_pre_commit_enabled = true;
            if (_is_master()) {
                _build_schedule_from_pre_collect(g_round_schedule_class);
            }
            _pre_commit_start(p2_start_ticks,
                              _is_master() ? g_round_schedule_class : NULL);
            while (g_pre_commit.active) {
                _run_pre_commit_slot();
            }
            p2_start_ticks = _pre_commit_p2_start_ticks();
            if (g_pre_commit.have_schedule) {
                _unpack_class_schedule(g_pre_commit.packed_schedule,
                                       g_proto_cfg.p2_node_count,
                                       g_round_schedule_class);
                _commit_round_schedule(g_round_schedule_class);
                g_round_pre_commit_applied = true;
            }
            else {
                skip_p2 = !_is_master();
                _latch_update_request();
            }
        }
        else if (timecast_protocol_pre_p2_is_complete(&g_proto)) {
            _build_schedule_from_pre_collect(g_round_schedule_class);
            _commit_round_schedule(g_round_schedule_class);
        }
        else {
            _seed_round_schedule_from_committed();
            _latch_update_request();
        }
    }
    else {
        p2_start_ticks = next_phase_start_ticks;
        _seed_round_schedule_from_committed();
    }

    g_round_p2_start_ticks = p2_start_ticks;
    tta_driver_process();
    if (skip_p2) {
        _wait_out_p2_schedule(p2_start_ticks);
    }
    else {
        /* Enter adaptive P2 before subslot 0 so the first TX/RX is armed ahead of
         * the common P2 start timestamp. */
        timecast_protocol_p2_start_pre_p2(&g_proto, p2_start_ticks, &g_proto_cfg);
        while (timecast_protocol_p2_is_active(&g_proto)) {
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

    printf("TimeCast start. node_id=%u hop=%u role=%s mode=%s ntx=%u pre_p2=%u pre_commit=%u app_data=%u payload=%u\n",
           (unsigned)TC_NODE_ID,
           (unsigned)g_proto_cfg.local_hop,
           _role_name(),
           _run_mode_name(),
           (unsigned)NTX,
           (unsigned)_use_pre_p2_mode(),
           (unsigned)_use_pre_commit_mode(),
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
    for (node_id = 0U; node_id < g_proto_cfg.p2_node_count; node_id++) {
        g_committed_class[node_id] = _initial_committed_class();
        g_round_schedule_class[node_id] = g_committed_class[node_id];
    }
    _refresh_local_payload();
    _commit_schedule(g_committed_class);

    g_round_count = 0U;
    g_p1_offset_sample_count = 0U;
    g_p1_tx_slot_start_ticks = 0U;
    g_p1_tx_cal_pending = false;
    next_master_round_start_ticks = radio_util_now_ticks() +
                                    TCAST_US_TO_TIMER_TICKS(TCAST_MASTER_START_DELAY_US);

    while (1) {
        if (_use_pre_p2_mode()) {
            _run_round_pre_p2(&next_master_round_start_ticks);
        }
        else {
            _run_round_original(&next_master_round_start_ticks);
        }
    }

    return 0;
}

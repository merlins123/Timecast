#ifndef TIMECAST_PROTOCOL_H
#define TIMECAST_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "packet.h"
#include "store.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TIMECAST_PHASE_P1_SYNC = 0,
    TIMECAST_PHASE_PRE_P2,
    TIMECAST_PHASE_PRE_COMMIT,
    TIMECAST_PHASE_P2_DATA,
} timecast_protocol_phase_t;

typedef struct {
    bool active;
    bool flag_tx;
    bool has_tref;
    uint8_t slot_idx;
    uint8_t ntx_done;
    uint8_t local_hop;
    uint32_t tref_local_ticks;
    uint32_t next_phase_start_local_ticks;
} timecast_protocol_p1_t;

typedef struct {
    bool active;
    bool tx_slot;
    bool complete;
    uint8_t slot_idx;
    uint8_t subslot_idx;
    uint16_t total_subslot;
    uint32_t start_local_ticks;
    uint32_t commit_start_local_tick;
    uint32_t rx_valid;
    uint8_t p2_payload_len[TIMECAST_STORE_MAX_NODES];
    uint8_t present[TIMECAST_STORE_MAX_NODES];
} timecast_protocol_pre_p2_t;

typedef struct {
    bool active;
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

typedef struct {
    bool active;
    bool tx_slot;
    uint8_t node_count;
    uint8_t slot_idx;
    uint8_t subslot_idx;
    uint16_t total_subslot;
    uint32_t start_local_ticks;
    uint32_t slot_ticks;
    uint32_t rx_valid;
    uint32_t store_updates;
    uint32_t subslot_ticks[TIMECAST_STORE_MAX_NODES];
    uint32_t subslot_offset_ticks[TIMECAST_STORE_MAX_NODES];
} timecast_protocol_p2_t;

typedef struct {
    timecast_protocol_phase_t phase;
    bool joined;
    uint32_t current_epoch;
    uint32_t rx_valid;
    uint32_t p1_tx_sched_fails;
    timecast_protocol_p1_t p1;
    timecast_protocol_pre_p2_t pre_p2;
    pre_commit_state_t pre_commit;
    timecast_protocol_p2_t p2;
} timecast_protocol_state_t;

typedef struct {
    uint8_t local_node_id;
    uint8_t ntx;
    uint8_t p2_node_count;
    uint32_t glossy_slot_ticks;
    uint32_t p1_rx_ts_to_slot_start_ticks;
    uint32_t p1_guard_ticks;
    uint32_t pre_p2_subslot_ticks;
    uint32_t pre_p2_guard_ticks;
    uint32_t pre_p2_rx_window_ticks;
    uint32_t p2_subslot_ticks;
    uint32_t p2_guard_ticks;
    uint32_t p2_rx_window_ticks;
    uint32_t p2_payload_base_ticks;
    uint32_t p2_payload_byte_ticks;
} timecast_protocol_cfg_t;

void protocol_init(timecast_protocol_state_t *state, bool initiator);
void p1_start(timecast_protocol_state_t *state,
                                uint32_t start_local_ticks,
                                const timecast_protocol_cfg_t *cfg,
                                bool initiator,
                                uint32_t epoch);
uint32_t p1_get_slot_start_local_ticks(const timecast_protocol_state_t *state,
                                                         const timecast_protocol_cfg_t *cfg);
void p1_prepare_tx(timecast_protocol_state_t *state,
                                     p1_sync_frame_t *frame);
void p1_handle_rx(timecast_protocol_state_t *state,
                                    const p1_sync_frame_t *frame,
                                    uint32_t rx_local_ticks,
                                    const timecast_protocol_cfg_t *cfg);
bool p1_finish_slot(timecast_protocol_state_t *state,
                                      const timecast_protocol_cfg_t *cfg,
                                      bool did_tx);
void pre_p2_start(timecast_protocol_state_t *state,
                                    uint8_t source_id,
                                    uint32_t start_local_ticks,
                                    const timecast_protocol_cfg_t *cfg, uint8_t desired_len);
uint32_t pre_p2_get_subslot_start_local_ticks(
    const timecast_protocol_state_t *state, const timecast_protocol_cfg_t *cfg);
bool pre_p2_prepare_tx(const timecast_protocol_state_t *state,
                                         uint8_t *p2_payload_len);
void pre_p2_handle_rx(timecast_protocol_state_t *state,
                                        uint8_t p2_payload_len,
                                        const timecast_protocol_cfg_t *cfg);
void pre_p2_finish_subslot(timecast_protocol_state_t *state,
                                             const timecast_protocol_cfg_t *cfg);
void pre_commit_start(timecast_protocol_state_t *state,
                                    uint32_t start_local_ticks,
                                    const timecast_protocol_cfg_t *cfg, const uint8_t *classes, bool master);
uint32_t pre_commit_slot_ticks(const timecast_protocol_cfg_t *cfg);
void p2_start_original(timecast_protocol_state_t *state,
                                         uint32_t start_local_ticks,
                                         const timecast_protocol_cfg_t *cfg);
void p2_start_pre_p2(timecast_protocol_state_t *state,
                                       uint32_t start_local_ticks,
                                       const timecast_protocol_cfg_t *cfg);
uint32_t p2_get_subslot_start_local_ticks(const timecast_protocol_state_t *state);
uint32_t p2_get_slot_ticks(const timecast_protocol_state_t *state,
                                             const timecast_protocol_cfg_t *cfg);
bool p2_prepare_tx(const timecast_protocol_state_t *state,
                                     const timecast_store_t *store,
                                     const timecast_protocol_cfg_t *cfg,
                                     p2_data_frame_t *frame,
                                     const uint8_t **data_ptr);
bool p2_handle_rx(timecast_protocol_state_t *state,
                                    timecast_store_t *store,
                                    const p2_data_frame_t *frame,
                                    const uint8_t *data);
void p2_finish_subslot(timecast_protocol_state_t *state,
                                         const timecast_protocol_cfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif 

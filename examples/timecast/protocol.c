#include "protocol.h"

#include <string.h>

static uint8_t _slot_budget(const timecast_protocol_cfg_t *cfg)
{
    return (uint8_t)((uint32_t)cfg->ntx * 2U);
}

static uint32_t _p1_sync_duration_ticks(const timecast_protocol_cfg_t *cfg)
{
    return (uint32_t)_slot_budget(cfg) * cfg->glossy_slot_ticks;
}

static uint32_t _pre_p2_subslot_period_ticks(const timecast_protocol_cfg_t *cfg)
{
    return cfg->pre_p2_subslot_ticks + cfg->pre_p2_guard_ticks;
}

static uint32_t _pre_p2_duration_ticks(const timecast_protocol_cfg_t *cfg)
{
    return (uint32_t)_slot_budget(cfg) * (uint32_t)cfg->p2_node_count *
           _pre_p2_subslot_period_ticks(cfg);
}

static bool _tx_first(const timecast_protocol_state_t *state)
{
    return state && ((state->p1.local_hop & 0x1U) == 0U);
}


static uint32_t _p2_subslot_ticks_from_payload_len(const timecast_protocol_cfg_t *cfg,
                                                          uint8_t p2_payload_len)
{
    
    return cfg->p2_payload_base_ticks +
           ((uint32_t)p2_payload_len * cfg->p2_payload_byte_ticks);
}

static void _set_next_phase_start(timecast_protocol_state_t *state,
                                         const timecast_protocol_cfg_t *cfg,
                                         uint32_t tref_local_ticks)
{
    state->p1.next_phase_start_local_ticks = tref_local_ticks +
                                             _p1_sync_duration_ticks(cfg) +
                                             cfg->p1_guard_ticks;
}

static void _set_pre_p2_commit_start(timecast_protocol_state_t *state,
                                        const timecast_protocol_cfg_t *cfg)
{
    state->pre_p2.commit_start_local_tick = state->pre_p2.start_local_ticks +
                                            _pre_p2_duration_ticks(cfg) +
                                            cfg->p1_guard_ticks;
}

static bool _pre_p2_all_present(const timecast_protocol_state_t *state,
                                const timecast_protocol_cfg_t *cfg)
{
    uint8_t source_id;

    for (source_id = 0U; source_id < cfg->p2_node_count; source_id++) {
        if (state->pre_p2.present[source_id] == 0U) {
            return false;
        }
    }

    return true;
}

static void _pre_p2_store_p2_payload_len(timecast_protocol_state_t *state,
                                         const timecast_protocol_cfg_t *cfg,
                                         uint8_t source_id,
                                         uint8_t p2_payload_len)
{
    if (!state || !cfg || (source_id >= cfg->p2_node_count)) {
        return;
    }

    state->pre_p2.present[source_id] = 1U;
    state->pre_p2.p2_payload_len[source_id] = p2_payload_len;
    state->pre_p2.complete = _pre_p2_all_present(state, cfg);
}


static void _p2_build_fixed_schedule(timecast_protocol_state_t *state,
                                     const timecast_protocol_cfg_t *cfg)
{
    uint8_t source_id;
    uint32_t slot_ticks = 0U;

    for (source_id = 0U; source_id < state->p2.node_count; source_id++) {
        state->p2.subslot_offset_ticks[source_id] = slot_ticks;
        state->p2.subslot_ticks[source_id] = cfg->p2_subslot_ticks;
        slot_ticks += cfg->p2_subslot_ticks + cfg->p2_guard_ticks;
    }

    state->p2.slot_ticks = slot_ticks;
}

static bool _p2_build_adaptive_schedule(timecast_protocol_state_t *state,
                                        const timecast_protocol_cfg_t *cfg)
{
    uint8_t source_id;
    uint32_t slot_ticks = 0U;

    for (source_id = 0U; source_id < state->p2.node_count; source_id++) {
        uint8_t p2_payload_len = state->pre_p2.p2_payload_len[source_id];
        uint32_t subslot_ticks;

        if (!state->pre_p2.present[source_id]) {
            return false;
        }
        subslot_ticks = _p2_subslot_ticks_from_payload_len(cfg, p2_payload_len);
        if (subslot_ticks == 0U) {
            return false;
        }

        state->p2.subslot_offset_ticks[source_id] = slot_ticks;
        state->p2.subslot_ticks[source_id] = subslot_ticks;
        slot_ticks += subslot_ticks + cfg->p2_guard_ticks;
    }

    state->p2.slot_ticks = slot_ticks;
    return true;
}

void protocol_init(timecast_protocol_state_t *state, bool initiator)
{
    if (!state) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->phase = TIMECAST_PHASE_P1_SYNC;
    state->p1.active = true;
    state->p1.flag_tx = initiator;
    state->p1.local_hop = initiator ? 0U : UINT8_MAX;
    state->joined = initiator;
}

void p1_start(timecast_protocol_state_t *state,
                                uint32_t start_local_ticks,
                                const timecast_protocol_cfg_t *cfg,
                                bool initiator,
                                uint32_t epoch)
{
    if (!state) {
        return;
    }

    memset(&state->p1, 0, sizeof(state->p1));
    memset(&state->pre_p2, 0, sizeof(state->pre_p2));
    memset(&state->p2, 0, sizeof(state->p2));

    state->phase = TIMECAST_PHASE_P1_SYNC;
    state->joined = initiator;
    state->current_epoch = epoch;
    state->rx_valid = 0U;
    state->p1_tx_sched_fails = 0U;
    state->p1.active = true;
    state->p1.flag_tx = initiator;
    state->p1.local_hop = initiator ? 0U : UINT8_MAX;
    if (initiator) {
        state->p1.has_tref = true;
        state->p1.tref_local_ticks = start_local_ticks;
        _set_next_phase_start(state, cfg, start_local_ticks);
    }
}

uint32_t p1_get_slot_start_local_ticks(const timecast_protocol_state_t *state,
                                                         const timecast_protocol_cfg_t *cfg)
{
    if (!state || !cfg || !state->p1.has_tref) {
        return 0U;
    }

    return state->p1.tref_local_ticks +
           ((uint32_t)state->p1.slot_idx * cfg->glossy_slot_ticks);
}

void p1_prepare_tx(timecast_protocol_state_t *state,
                                     const timecast_protocol_cfg_t *cfg,
                                     p1_sync_frame_t *frame)
{
    
    frame->packet_type = PACKET_TYPE_P1_SYNC;
    frame->sender_node_id = cfg->local_node_id;
    frame->relay_cnt = (uint8_t)state->p1.slot_idx;
    frame->flags = 0U;
    frame->epoch = state->current_epoch;

    state->p1.ntx_done++;

}

void p1_handle_rx(timecast_protocol_state_t *state,
                                    const p1_sync_frame_t *frame,
                                    uint32_t rx_local_ticks,
                                    const timecast_protocol_cfg_t *cfg)
{
    uint32_t tref_local_ticks;
    bool first_reference;

    first_reference = !state->p1.has_tref;
    state->current_epoch = frame->epoch;

    if (!state->p1.has_tref || (frame->relay_cnt > state->p1.slot_idx)) {
        state->p1.slot_idx = frame->relay_cnt;
    }

    if (!state->p1.has_tref) {
        tref_local_ticks = rx_local_ticks - cfg->p1_rx_ts_to_slot_start_ticks -
                           ((uint32_t)frame->relay_cnt * cfg->glossy_slot_ticks);
        state->p1.has_tref = true;
        state->p1.local_hop = (uint8_t)(frame->relay_cnt + 1U);
        state->p1.tref_local_ticks = tref_local_ticks;
        _set_next_phase_start(state, cfg, tref_local_ticks);
        state->joined = true;
    }

    if (first_reference) {
        state->p1.slot_idx = (uint8_t)(frame->relay_cnt + 1U);
        state->p1.flag_tx = (state->p1.ntx_done < cfg->ntx);
    }

}

bool p1_finish_slot(timecast_protocol_state_t *state,
                                      const timecast_protocol_cfg_t *cfg,
                                      bool did_tx)
{
    uint8_t slot_budget;

    if (!state || !cfg || !state->p1.active) {
        return false;
    }

    slot_budget = _slot_budget(cfg);

    if (!did_tx && state->p1.has_tref && (state->p1.ntx_done < cfg->ntx)) {
        state->p1.flag_tx = true;
    }
    else {
        state->p1.flag_tx = false;
    }

    state->p1.slot_idx++;
    if (state->p1.slot_idx >= slot_budget) {
        state->p1.active = false;
    }

    return state->p1.active;
}

void pre_p2_start(timecast_protocol_state_t *state,
                                    uint8_t source_id,
                                    uint32_t start_local_ticks,
                                    const timecast_protocol_cfg_t *cfg, uint8_t desired_len)
{

    memset(&state->pre_p2, 0, sizeof(state->pre_p2));
    state->phase = TIMECAST_PHASE_PRE_P2;
    state->pre_p2.tx_slot = _tx_first(state);
    state->pre_p2.start_local_ticks = start_local_ticks;
    _set_pre_p2_commit_start(state, cfg);
    state->pre_p2.active = state->joined;
    

    state->pre_p2.present[source_id] = 1U;
    state->pre_p2.p2_payload_len[source_id] = desired_len;
}

uint32_t pre_p2_get_subslot_start_local_ticks(
    const timecast_protocol_state_t *state, const timecast_protocol_cfg_t *cfg)
{

    return state->pre_p2.start_local_ticks +
           ((uint32_t)state->pre_p2.total_subslot * _pre_p2_subslot_period_ticks(cfg));
}

bool pre_p2_prepare_tx(const timecast_protocol_state_t *state,
                                         uint8_t *p2_payload_len)
{
    uint8_t owner_id;

    owner_id = state->pre_p2.subslot_idx;
    if (!state->pre_p2.present[owner_id]) {
        return false;
    }

    *p2_payload_len = state->pre_p2.p2_payload_len[owner_id];
    return true;
}

void pre_p2_handle_rx(timecast_protocol_state_t *state,
                                        uint8_t p2_payload_len,
                                        const timecast_protocol_cfg_t *cfg)
{
    uint8_t owner_id;



    owner_id = state->pre_p2.subslot_idx;

    _pre_p2_store_p2_payload_len(state, cfg, owner_id, p2_payload_len);

    state->pre_p2.rx_valid++;

}

void pre_p2_finish_subslot(timecast_protocol_state_t *state,
                                             const timecast_protocol_cfg_t *cfg)
{
    state->pre_p2.total_subslot++;
    state->pre_p2.subslot_idx++;
    if (state->pre_p2.subslot_idx < cfg->p2_node_count) {
        return;
    }

    state->pre_p2.subslot_idx = 0U;
    state->pre_p2.slot_idx++;
    state->pre_p2.tx_slot = !state->pre_p2.tx_slot;
    if (state->pre_p2.slot_idx >= _slot_budget(cfg)) {
        state->pre_p2.active = false;
    }

}

static uint8_t _packed_class_len(uint8_t node_count)
{
    return (uint8_t)(((uint32_t)node_count + 1U) / 2U);
}

uint32_t pre_commit_slot_ticks(const timecast_protocol_cfg_t *cfg)
{
    uint8_t payload_len = (uint8_t)(PACKET_PRE_COMMIT_BASE_LEN +
                                    _packed_class_len(cfg->p2_node_count));

    return cfg->p2_payload_base_ticks +
           ((uint32_t)payload_len * cfg->p2_payload_byte_ticks);
}

static void _pack_class_schedule(const uint8_t *classes, uint8_t node_count,
                                 uint8_t *packed_out, uint8_t *packed_len_out)
{
    uint8_t source_id;
    uint8_t packed_len = _packed_class_len(node_count);

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

void pre_commit_start(timecast_protocol_state_t *state,
                                    uint32_t start_local_ticks,
                                    const timecast_protocol_cfg_t *cfg, const uint8_t *classes, bool master)
{
    memset(&state->pre_commit, 0, sizeof(state->pre_commit));
    state->pre_commit.start_local_ticks = start_local_ticks;
    state->pre_commit.slot_ticks = pre_commit_slot_ticks(cfg);
    state->phase = TIMECAST_PHASE_PRE_COMMIT;
    state->pre_commit.active = state->joined;
    if (!master) {
        return;
    }

    _pack_class_schedule(classes, cfg->p2_node_count,
                         state->pre_commit.packed_schedule, &state->pre_commit.packed_len);
    state->pre_commit.have_schedule = true;
    state->pre_commit.flag_tx = true;
}

static bool _p2_start_common(timecast_protocol_state_t *state,
                             uint32_t start_local_ticks,
                             const timecast_protocol_cfg_t *cfg)
{
    if (!state || !cfg) {
        return false;
    }

    memset(&state->p2, 0, sizeof(state->p2));
    state->phase = TIMECAST_PHASE_P2_DATA;
    state->p2.tx_slot = _tx_first(state);
    state->p2.node_count = cfg->p2_node_count;
    state->p2.start_local_ticks = start_local_ticks;
    state->p2.active = state->joined;

    return state->p2.active;
}

void p2_start_original(timecast_protocol_state_t *state,
                                         uint32_t start_local_ticks,
                                         const timecast_protocol_cfg_t *cfg)
{
    if (!_p2_start_common(state, start_local_ticks, cfg)) {
        return;
    }

    _p2_build_fixed_schedule(state, cfg);
}

void p2_start_pre_p2(timecast_protocol_state_t *state,
                                       uint32_t start_local_ticks,
                                       const timecast_protocol_cfg_t *cfg)
{
    if (!_p2_start_common(state, start_local_ticks, cfg)) {
        return;
    }

    state->p2.active = state->pre_p2.complete && _p2_build_adaptive_schedule(state, cfg);
}

uint32_t p2_get_subslot_start_local_ticks(const timecast_protocol_state_t *state)
{

    return state->p2.start_local_ticks +
           ((uint32_t)state->p2.slot_idx * state->p2.slot_ticks) +
           state->p2.subslot_offset_ticks[state->p2.subslot_idx];
}

uint32_t p2_get_slot_ticks(const timecast_protocol_state_t *state,
                                             const timecast_protocol_cfg_t *cfg)
{
    if (!state || !cfg) {
        return 0U;
    }

    if (state->p2.slot_ticks > 0U) {
        return state->p2.slot_ticks;
    }

    return (uint32_t)cfg->p2_node_count * (cfg->p2_subslot_ticks + cfg->p2_guard_ticks);
}

bool p2_prepare_tx(const timecast_protocol_state_t *state,
                                     const timecast_store_t *store,
                                     const timecast_protocol_cfg_t *cfg,
                                     p2_data_frame_t *frame,
                                     const uint8_t **data_ptr)
{
    const timecast_store_entry_t *entry;
    uint8_t owner_id;

    (void)cfg;

    owner_id = state->p2.subslot_idx;


    entry = &store->entries[owner_id];
    if (!entry->present || (entry->len == 0U)) {
        return false;
    }

    frame->packet_type = PACKET_TYPE_P2_DATA;
    frame->source_node_id = owner_id;
    frame->slot_idx = state->p2.slot_idx;
    frame->subslot_idx = state->p2.subslot_idx;
    frame->flags = 0U;
    frame->data_len = entry->len;
    frame->epoch = state->current_epoch;
    *data_ptr = entry->data;
    return true;
}

bool p2_handle_rx(timecast_protocol_state_t *state,
                                    timecast_store_t *store,
                                    const p2_data_frame_t *frame,
                                    const uint8_t *data)
{

    if (frame->packet_type != PACKET_TYPE_P2_DATA) {
        return false;
    }
    if (frame->data_len > PACKET_P2_DATA_MAX_DATA_LEN) {
        return false;
    }
    if (frame->epoch != state->current_epoch) {
        return false;
    }
    if (frame->slot_idx != state->p2.slot_idx) {
        return false;
    }
    if (frame->subslot_idx != state->p2.subslot_idx) {
        return false;
    }

    state->p2.rx_valid++;
    if (store_import(store, frame->source_node_id, data, frame->data_len)) {
        state->p2.store_updates++;
    }

    return true;
}

void p2_finish_subslot(timecast_protocol_state_t *state,
                                         const timecast_protocol_cfg_t *cfg)
{
    uint8_t node_count;

    node_count = state->p2.node_count;

    state->p2.total_subslot++;
    state->p2.subslot_idx++;
    if (state->p2.subslot_idx >= node_count) {
        state->p2.subslot_idx = 0U;
        state->p2.slot_idx++;
        state->p2.tx_slot = !state->p2.tx_slot;
        if (state->p2.slot_idx >= _slot_budget(cfg)) {
            state->p2.active = false;
        }
    }


}

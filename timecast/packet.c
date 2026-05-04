#include "packet.h"

#include <string.h>

static inline void _u32_to_le(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
    dst[2] = (uint8_t)((v >> 16) & 0xFFu);
    dst[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline uint32_t _u32_from_le(const uint8_t *src)
{
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

bool timecast_packet_encode_p1_sync(uint8_t *dst, size_t dst_len,
                                    const timecast_p1_sync_frame_t *frame)
{
    if (!dst || !frame || (dst_len < TIMECAST_PACKET_P1_SYNC_PAYLOAD_LEN)) {
        return false;
    }

    dst[0] = TIMECAST_PACKET_MAGIC_0;
    dst[1] = TIMECAST_PACKET_MAGIC_1;
    dst[2] = TIMECAST_PACKET_MAGIC_2;
    dst[3] = TIMECAST_PACKET_MAGIC_3;
    dst[4] = TIMECAST_PACKET_TYPE_P1_SYNC;
    dst[5] = frame->sender_node_id;
    dst[6] = (uint8_t)(frame->relay_cnt & TIMECAST_PACKET_P1_RELAY_CNT_MASK);
    if ((frame->flags & TIMECAST_PACKET_P1_FLAG_RUN_PRE) != 0U) {
        dst[6] |= TIMECAST_PACKET_P1_RUN_PRE_WIRE_BIT;
    }
    _u32_to_le(&dst[7], frame->epoch);

    return true;
}

bool timecast_packet_decode_p1_sync(const uint8_t *radio_payload, size_t payload_len,
                                    timecast_p1_sync_frame_t *frame)
{
    const uint8_t *app;

    if (!radio_payload || !frame || (payload_len < TIMECAST_PACKET_P1_SYNC_RX_LEN)) {
        return false;
    }

    app = &radio_payload[TIMECAST_PACKET_ADV_ADDR_LEN];
    if ((app[0] != TIMECAST_PACKET_MAGIC_0) ||
        (app[1] != TIMECAST_PACKET_MAGIC_1) ||
        (app[2] != TIMECAST_PACKET_MAGIC_2) ||
        (app[3] != TIMECAST_PACKET_MAGIC_3) ||
        (app[4] != TIMECAST_PACKET_TYPE_P1_SYNC)) {
        return false;
    }

    frame->magic[0] = app[0];
    frame->magic[1] = app[1];
    frame->magic[2] = app[2];
    frame->magic[3] = app[3];
    frame->packet_type = app[4];
    frame->sender_node_id = app[5];
    frame->relay_cnt = (uint8_t)(app[6] & TIMECAST_PACKET_P1_RELAY_CNT_MASK);
    frame->flags = ((app[6] & TIMECAST_PACKET_P1_RUN_PRE_WIRE_BIT) != 0U) ?
                   TIMECAST_PACKET_P1_FLAG_RUN_PRE : 0U;
    frame->epoch = _u32_from_le(&app[7]);

    return true;
}

bool timecast_packet_encode_p2_data(uint8_t *dst, size_t dst_len,
                                    const timecast_p2_data_frame_t *frame,
                                    const uint8_t *data)
{
    if (!dst || !frame || (frame->data_len > TIMECAST_PACKET_P2_DATA_MAX_DATA_LEN) ||
        (dst_len < (size_t)(TIMECAST_PACKET_P2_DATA_HDR_LEN + frame->data_len)) ||
        ((frame->data_len > 0U) && !data)) {
        return false;
    }

    dst[0] = TIMECAST_PACKET_TYPE_P2_DATA;
    dst[1] = (uint8_t)(frame->source_node_id & TIMECAST_PACKET_P2_SOURCE_ID_MASK);
    if ((frame->flags & TIMECAST_PACKET_P2_FLAG_UPDATE_REQ) != 0U) {
        dst[1] |= TIMECAST_PACKET_P2_UPDATE_WIRE_BIT;
    }
    dst[2] = frame->slot_idx;
    dst[3] = frame->subslot_idx;
    dst[4] = frame->data_len;
    _u32_to_le(&dst[5], frame->epoch);
    if (frame->data_len > 0U) {
        memcpy(&dst[TIMECAST_PACKET_P2_DATA_HDR_LEN], data, frame->data_len);
    }

    return true;
}

bool timecast_packet_decode_p2_data(const uint8_t *radio_payload, size_t payload_len,
                                    timecast_p2_data_frame_t *frame,
                                    uint8_t *data_out)
{
    const uint8_t *app;
    size_t app_len;

    if (!radio_payload || !frame || (payload_len < TIMECAST_PACKET_P2_DATA_RX_MIN_LEN)) {
        return false;
    }

    app = &radio_payload[TIMECAST_PACKET_ADV_ADDR_LEN];
    app_len = payload_len - TIMECAST_PACKET_ADV_ADDR_LEN;

    if (app[0] != TIMECAST_PACKET_TYPE_P2_DATA) {
        return false;
    }

    frame->packet_type = app[0];
    frame->source_node_id = (uint8_t)(app[1] & TIMECAST_PACKET_P2_SOURCE_ID_MASK);
    frame->slot_idx = app[2];
    frame->subslot_idx = app[3];
    frame->flags = ((app[1] & TIMECAST_PACKET_P2_UPDATE_WIRE_BIT) != 0U) ?
                   TIMECAST_PACKET_P2_FLAG_UPDATE_REQ : 0U;
    frame->data_len = app[4];
    frame->epoch = _u32_from_le(&app[5]);

    if ((frame->data_len > TIMECAST_PACKET_P2_DATA_MAX_DATA_LEN) ||
        (app_len < (size_t)(TIMECAST_PACKET_P2_DATA_HDR_LEN + frame->data_len))) {
        return false;
    }

    if (data_out && (frame->data_len > 0U)) {
        memcpy(data_out, &app[TIMECAST_PACKET_P2_DATA_HDR_LEN], frame->data_len);
    }

    return true;
}

bool timecast_packet_encode_pre_p2_ctrl(uint8_t *dst, size_t dst_len, uint8_t class_id)
{
    if (!dst || (dst_len < TIMECAST_PACKET_PRE_P2_CTRL_LEN)) {
        return false;
    }

    dst[0] = class_id;
    return true;
}

bool timecast_packet_decode_pre_p2_ctrl(const uint8_t *radio_payload, size_t payload_len,
                                        uint8_t *class_id)
{
    const uint8_t *app;

    if (!radio_payload || !class_id || (payload_len != TIMECAST_PACKET_PRE_P2_RX_LEN)) {
        return false;
    }

    app = &radio_payload[TIMECAST_PACKET_ADV_ADDR_LEN];
    *class_id = app[0];
    return true;
}

bool timecast_packet_encode_pre_commit(uint8_t *dst, size_t dst_len,
                                       const timecast_pre_commit_frame_t *frame,
                                       const uint8_t *packed_schedule,
                                       size_t packed_len)
{
    if (!dst || !frame || !packed_schedule ||
        (packed_len == 0U) ||
        (packed_len > TIMECAST_PACKET_PRE_COMMIT_MAX_SCHEDULE_LEN) ||
        (dst_len < (TIMECAST_PACKET_PRE_COMMIT_BASE_LEN + packed_len))) {
        return false;
    }

    dst[0] = frame->relay_cnt;
    memcpy(&dst[TIMECAST_PACKET_PRE_COMMIT_BASE_LEN], packed_schedule, packed_len);

    return true;
}

bool timecast_packet_decode_pre_commit(const uint8_t *radio_payload, size_t payload_len,
                                       uint8_t node_count,
                                       timecast_pre_commit_frame_t *frame,
                                       uint8_t *packed_schedule_out,
                                       size_t *packed_len_out)
{
    const uint8_t *app;
    size_t app_len;
    size_t packed_len;

    if (!radio_payload || !frame || !packed_schedule_out || !packed_len_out ||
        (node_count == 0U) ||
        (node_count > TIMECAST_STORE_MAX_NODES) ||
        (payload_len < TIMECAST_PACKET_PRE_COMMIT_RX_MIN_LEN)) {
        return false;
    }

    app = &radio_payload[TIMECAST_PACKET_ADV_ADDR_LEN];
    app_len = payload_len - TIMECAST_PACKET_ADV_ADDR_LEN;

    packed_len = (size_t)((node_count + 1U) / 2U);
    if ((packed_len > TIMECAST_PACKET_PRE_COMMIT_MAX_SCHEDULE_LEN) ||
        (app_len != (TIMECAST_PACKET_PRE_COMMIT_BASE_LEN + packed_len))) {
        return false;
    }

    frame->relay_cnt = app[0];
    memcpy(packed_schedule_out, &app[TIMECAST_PACKET_PRE_COMMIT_BASE_LEN], packed_len);
    *packed_len_out = packed_len;
    return true;
}

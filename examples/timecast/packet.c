#include "packet.h"
#include "nrf_sf_radio/link_radio.h"
#include <string.h>


static void _u32_to_le(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
    dst[2] = (uint8_t)((v >> 16) & 0xFFu);
    dst[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t _u32_from_le(const uint8_t *src)
{
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}


void encode_p1_sync(uint8_t *dst, size_t dst_len,
                                    const p1_sync_frame_t *frame)
{
    (void)dst_len;

    dst[0] = frame->packet_type;
    dst[1] = (uint8_t)(frame->relay_cnt & PACKET_P1_RELAY_CNT_MASK);
    if ((frame->flags & 1U) != 0U) {
        dst[1] |= PACKET_P1_RUN_PRE_WIRE_BIT;
    }
    _u32_to_le(&dst[2], frame->epoch);

}

bool decode_p1_sync(const uint8_t *payload,
                                    p1_sync_frame_t *frame)
{
    payload = nrf_sf_radio_payload(payload);

    frame->packet_type = payload[0];
    if(frame->packet_type != PACKET_TYPE_P1_SYNC){
        return false;
    }

    frame->relay_cnt = (uint8_t)(payload[1] & PACKET_P1_RELAY_CNT_MASK);
    frame->flags = ((payload[1] & PACKET_P1_RUN_PRE_WIRE_BIT) != 0U) ?
                   1U : 0U;
    frame->epoch = _u32_from_le(&payload[2]);
    

    return true;
}

void encode_p2_data(uint8_t *dst,
                                    const p2_data_frame_t *frame,
                                    const uint8_t *data)
{

    dst[0] = frame->packet_type;
    dst[1] = (uint8_t)(frame->source_node_id & PACKET_P2_SOURCE_ID_MASK);
    if ((frame->flags & 1U) != 0U) {
        dst[1] |= PACKET_P2_UPDATE_WIRE_BIT;
    }
    dst[2] = frame->slot_idx;
    dst[3] = frame->subslot_idx;
    dst[4] = frame->data_len;
    _u32_to_le(&dst[5], frame->epoch);
    if (frame->data_len > 0U) {
        memcpy(&dst[PACKET_P2_DATA_APP_HDR_LEN], data, frame->data_len);
    }

}

void decode_p2_data(const uint8_t *payload,
                                    p2_data_frame_t *frame,
                                    uint8_t *data_out)
{
    payload = nrf_sf_radio_payload(payload);

    frame->packet_type = payload[0];
    frame->source_node_id = (uint8_t)(payload[1] & PACKET_P2_SOURCE_ID_MASK);
    frame->slot_idx = payload[2];
    frame->subslot_idx = payload[3];
    frame->flags = ((payload[1] & PACKET_P2_UPDATE_WIRE_BIT) != 0U) ?
                   1U : 0U;
    frame->data_len = payload[4];
    frame->epoch = _u32_from_le(&payload[5]);


    if (data_out && (frame->data_len > 0U)) {
        memcpy(data_out, &payload[PACKET_P2_DATA_APP_HDR_LEN], frame->data_len);
    }


}

void encode_pre_p2(uint8_t *dst, uint8_t class_id)
{

    dst[0] = class_id;


}

void decode_pre_p2(const uint8_t *payload, uint8_t *class_id)
{
    payload = nrf_sf_radio_payload(payload);

    *class_id = payload[0];

}

void encode_pre_commit(uint8_t *dst,
                                const uint8_t *packed_schedule,
                                size_t packed_len)
{

    memcpy(dst, packed_schedule, packed_len);


}

void decode_pre_commit(const uint8_t *payload,
                                       uint8_t node_count,
                                       uint8_t *packed_schedule_out,
                                       size_t *packed_len_out)
{
    size_t packed_len;

    payload = nrf_sf_radio_payload(payload);

    packed_len = (size_t)((node_count + 1U) / 2U);

    memcpy(packed_schedule_out, payload, packed_len);
    *packed_len_out = packed_len;

}


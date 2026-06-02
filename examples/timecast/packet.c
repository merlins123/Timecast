#include "packet.h"

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

static void _put_radio_hdr(uint8_t *dst, uint8_t length)
{
    dst[0] = 0x42;
    dst[1] = length;
    dst[2] = (uint8_t)(Adv_Ad_Lo & 0xFFu);
    dst[3] = (uint8_t)((Adv_Ad_Lo >> 8) & 0xFFu);
    dst[4] = (uint8_t)((Adv_Ad_Lo >> 16) & 0xFFu);
    dst[5] = (uint8_t)(Adv_Ad_Hi & 0xFFu);
    dst[6] = (uint8_t)((Adv_Ad_Hi >> 8) & 0xFFu);
    dst[7] = (uint8_t)((Adv_Ad_Hi >> 16) & 0xFFu);
}

static const uint8_t *_app_payload(const uint8_t *radio_payload)
{
    return &radio_payload[PACKET_RADIO_HDR_LEN];
}


void encode_p1_sync(uint8_t *dst, size_t dst_len,
                                    const p1_sync_frame_t *frame)
{
    (void)dst_len;

    _put_radio_hdr(dst, PACKET_ADV_ADDR_LEN + PACKET_P1_SYNC_APP_LEN);
    dst[8] = frame->packet_type;
    dst[9] = frame->sender_node_id;
    dst[10] = (uint8_t)(frame->relay_cnt & PACKET_P1_RELAY_CNT_MASK);
    if ((frame->flags & 1U) != 0U) {
        dst[10] |= PACKET_P1_RUN_PRE_WIRE_BIT;
    }
    _u32_to_le(&dst[11], frame->epoch);

}

bool decode_p1_sync(const uint8_t *radio_payload,
                                    p1_sync_frame_t *frame)
{
    const uint8_t *app;

    if(radio_payload[1] != PACKET_ADV_ADDR_LEN + PACKET_P1_SYNC_APP_LEN){
        return false;
    }
    app = _app_payload(radio_payload);

    frame->packet_type = app[0];
    if(frame->packet_type != PACKET_TYPE_P1_SYNC){
        return false;
    }
    
    frame->sender_node_id = app[1];
    frame->relay_cnt = (uint8_t)(app[2] & PACKET_P1_RELAY_CNT_MASK);
    frame->flags = ((app[2] & PACKET_P1_RUN_PRE_WIRE_BIT) != 0U) ?
                   1U : 0U;
    frame->epoch = _u32_from_le(&app[3]);
    

    return true;
}

void encode_p2_data(uint8_t *dst,
                                    const p2_data_frame_t *frame,
                                    const uint8_t *data)
{

    _put_radio_hdr(dst, PACKET_ADV_ADDR_LEN + PACKET_P2_DATA_APP_HDR_LEN + frame->data_len);
    dst[8] = frame->packet_type;
    dst[9] = (uint8_t)(frame->source_node_id & PACKET_P2_SOURCE_ID_MASK);
    if ((frame->flags & 1U) != 0U) {
        dst[9] |= PACKET_P2_UPDATE_WIRE_BIT;
    }
    dst[10] = frame->slot_idx;
    dst[11] = frame->subslot_idx;
    dst[12] = frame->data_len;
    _u32_to_le(&dst[13], frame->epoch);
    if (frame->data_len > 0U) {
        memcpy(&dst[PACKET_P2_DATA_HDR_LEN], data, frame->data_len);
    }

}

void decode_p2_data(const uint8_t *radio_payload,
                                    p2_data_frame_t *frame,
                                    uint8_t *data_out)
{
    const uint8_t *app;


    app = _app_payload(radio_payload);


    frame->packet_type = app[0];
    frame->source_node_id = (uint8_t)(app[1] & PACKET_P2_SOURCE_ID_MASK);
    frame->slot_idx = app[2];
    frame->subslot_idx = app[3];
    frame->flags = ((app[1] & PACKET_P2_UPDATE_WIRE_BIT) != 0U) ?
                   1U : 0U;
    frame->data_len = app[4];
    frame->epoch = _u32_from_le(&app[5]);


    if (data_out && (frame->data_len > 0U)) {
        memcpy(data_out, &app[PACKET_P2_DATA_APP_HDR_LEN], frame->data_len);
    }


}

void encode_pre_p2(uint8_t *dst, uint8_t class_id)
{


    _put_radio_hdr(dst, PACKET_ADV_ADDR_LEN + PACKET_PRE_P2_CTRL_APP_LEN);
    dst[8] = class_id;


}

void decode_pre_p2(const uint8_t *radio_payload, uint8_t *class_id)
{
    const uint8_t *app;

    app = _app_payload(radio_payload);
    *class_id = app[0];

}

void encode_pre_commit(uint8_t *dst,
                                const uint8_t *packed_schedule,
                                size_t packed_len)
{

    _put_radio_hdr(dst, PACKET_ADV_ADDR_LEN + packed_len);
    memcpy(&dst[PACKET_PRE_COMMIT_BASE_LEN], packed_schedule, packed_len);


}

void decode_pre_commit(const uint8_t *radio_payload,
                                       uint8_t node_count,
                                       uint8_t *packed_schedule_out,
                                       size_t *packed_len_out)
{
    const uint8_t *app;
    size_t packed_len;

    app = _app_payload(radio_payload);

    packed_len = (size_t)((node_count + 1U) / 2U);

    memcpy(packed_schedule_out, app, packed_len);
    *packed_len_out = packed_len;

}

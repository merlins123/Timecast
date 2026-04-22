#ifndef TIMECAST_PACKET_H
#define TIMECAST_PACKET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "store.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TIMECAST_PACKET_ADV_ADDR_LEN (6U)
#define TIMECAST_PACKET_MAGIC_0      ('T')
#define TIMECAST_PACKET_MAGIC_1      ('C')
#define TIMECAST_PACKET_MAGIC_2      ('S')
#define TIMECAST_PACKET_MAGIC_3      ('1')
#define TIMECAST_PACKET_TYPE_P1_SYNC (0x01U)
#define TIMECAST_PACKET_TYPE_P2_DATA (0x02U)

typedef struct __attribute__((packed)) {
    uint8_t magic[4];
    uint8_t packet_type;
    uint8_t sender_node_id;
    uint8_t relay_cnt;
    uint32_t epoch;
} timecast_p1_sync_frame_t;

enum {
    TIMECAST_PACKET_P1_SYNC_PAYLOAD_LEN = sizeof(timecast_p1_sync_frame_t),
    TIMECAST_PACKET_P1_SYNC_RX_LEN = TIMECAST_PACKET_ADV_ADDR_LEN + sizeof(timecast_p1_sync_frame_t),
};

typedef struct __attribute__((packed)) {
    uint8_t packet_type;
    uint8_t source_node_id;
    uint8_t slot_idx;
    uint8_t subslot_idx;
    uint8_t data_len;
    uint32_t epoch;
} timecast_p2_data_frame_t;

enum {
    TIMECAST_PACKET_P2_DATA_HDR_LEN = sizeof(timecast_p2_data_frame_t),
    TIMECAST_PACKET_P2_DATA_MAX_DATA_LEN = TIMECAST_STORE_MAX_DATA_LEN,
    TIMECAST_PACKET_P2_DATA_MAX_PAYLOAD_LEN =
        TIMECAST_PACKET_P2_DATA_HDR_LEN + TIMECAST_PACKET_P2_DATA_MAX_DATA_LEN,
    TIMECAST_PACKET_P2_DATA_RX_MIN_LEN =
        TIMECAST_PACKET_ADV_ADDR_LEN + TIMECAST_PACKET_P2_DATA_HDR_LEN,
    TIMECAST_PACKET_PRE_P2_CTRL_LEN = 1U,
    TIMECAST_PACKET_PRE_P2_RX_LEN = TIMECAST_PACKET_ADV_ADDR_LEN + TIMECAST_PACKET_PRE_P2_CTRL_LEN,
};

bool timecast_packet_encode_p1_sync(uint8_t *dst, size_t dst_len,
                                    const timecast_p1_sync_frame_t *frame);
bool timecast_packet_decode_p1_sync(const uint8_t *radio_payload, size_t payload_len,
                                    timecast_p1_sync_frame_t *frame);
bool timecast_packet_encode_p2_data(uint8_t *dst, size_t dst_len,
                                    const timecast_p2_data_frame_t *frame,
                                    const uint8_t *data);
bool timecast_packet_decode_p2_data(const uint8_t *radio_payload, size_t payload_len,
                                    timecast_p2_data_frame_t *frame,
                                    uint8_t *data_out);
bool timecast_packet_encode_pre_p2_ctrl(uint8_t *dst, size_t dst_len, uint8_t p2_payload_len);
bool timecast_packet_decode_pre_p2_ctrl(const uint8_t *radio_payload, size_t payload_len,
                                        uint8_t *p2_payload_len);

#ifdef __cplusplus
}
#endif

#endif /* TIMECAST_PACKET_H */

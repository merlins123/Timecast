#ifndef PACKET_H
#define PACKET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "store.h"

#ifdef __cplusplus
extern "C" {
#endif

#define Adv_Ad_Lo      (0xABABABUL)
#define Adv_Ad_Hi      (0xABABC0UL)

#define PACKET_S0_LEN       (1U)
#define PACKET_LENGTH_LEN   (1U)
#define PACKET_ADV_ADDR_LEN (6U)
#define PACKET_RADIO_HDR_LEN \
    (PACKET_S0_LEN + PACKET_LENGTH_LEN + PACKET_ADV_ADDR_LEN)
#define PACKET_MAGIC_0      ('T')
#define PACKET_MAGIC_1      ('C')
#define PACKET_MAGIC_2      ('S')
#define PACKET_MAGIC_3      ('1')
#define PACKET_TYPE_P1_SYNC (0x01U)
#define PACKET_TYPE_P2_DATA (0x02U)

#define PACKET_P1_FLAG_RUN_PRE      (0x01U)
#define PACKET_P2_FLAG_UPDATE_REQ   (0x01U)

#define PACKET_P1_RELAY_CNT_MASK    (0x7FU)
#define PACKET_P1_RUN_PRE_WIRE_BIT  (0x80U)
#define PACKET_P2_SOURCE_ID_MASK    (0x7FU)
#define PACKET_P2_UPDATE_WIRE_BIT   (0x80U)

#if (TIMECAST_STORE_MAX_NODES > PACKET_P2_SOURCE_ID_MASK)
#error "TIMECAST_STORE_MAX_NODES exceeds 7-bit packed source_node_id budget"
#endif

typedef struct __attribute__((packed)) {
    uint8_t magic[4];
    uint8_t packet_type;
    uint8_t sender_node_id;
    uint8_t relay_cnt;
    uint8_t flags;
    uint32_t epoch;
} p1_sync_frame_t;

enum {
    PACKET_P1_SYNC_APP_LEN = sizeof(p1_sync_frame_t)-1, /* flags are packed into relay_cnt[7] on the wire */
    PACKET_P1_SYNC_PAYLOAD_LEN = PACKET_RADIO_HDR_LEN + PACKET_P1_SYNC_APP_LEN,
    PACKET_P1_SYNC_RX_LEN = PACKET_P1_SYNC_PAYLOAD_LEN,
};

typedef struct __attribute__((packed)) {
    uint8_t packet_type;
    uint8_t source_node_id;
    uint8_t slot_idx;
    uint8_t subslot_idx;
    uint8_t flags;
    uint8_t data_len;
    uint32_t epoch;
} p2_data_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t relay_cnt;
} pre_commit_frame_t;

enum {
    PACKET_P2_DATA_APP_HDR_LEN = sizeof(p2_data_frame_t)-1, /* flags are packed into relay_cnt[7] on the wire */
    PACKET_P2_DATA_HDR_LEN = PACKET_RADIO_HDR_LEN + PACKET_P2_DATA_APP_HDR_LEN,
    PACKET_P2_DATA_MAX_DATA_LEN = TIMECAST_STORE_MAX_DATA_LEN,
    PACKET_P2_DATA_MAX_PAYLOAD_LEN =
        PACKET_P2_DATA_HDR_LEN + PACKET_P2_DATA_MAX_DATA_LEN,
    PACKET_P2_DATA_RX_MIN_LEN = PACKET_P2_DATA_HDR_LEN,
    PACKET_PRE_P2_CTRL_APP_LEN = 1U,
    PACKET_PRE_P2_CTRL_LEN = PACKET_RADIO_HDR_LEN + PACKET_PRE_P2_CTRL_APP_LEN,
    PACKET_PRE_COMMIT_APP_BASE_LEN = sizeof(pre_commit_frame_t),
    PACKET_PRE_COMMIT_BASE_LEN = PACKET_RADIO_HDR_LEN + PACKET_PRE_COMMIT_APP_BASE_LEN,
    PACKET_PRE_COMMIT_MAX_SCHEDULE_LEN = (TIMECAST_STORE_MAX_NODES + 1U) / 2U,
    PACKET_PRE_COMMIT_MAX_PAYLOAD_LEN =
        PACKET_PRE_COMMIT_BASE_LEN + PACKET_PRE_COMMIT_MAX_SCHEDULE_LEN,
    PACKET_PRE_COMMIT_RX_MIN_LEN = PACKET_PRE_COMMIT_BASE_LEN,
};

void encode_p1_sync(uint8_t *dst, size_t dst_len,
                                    const p1_sync_frame_t *frame);
void decode_p1_sync(const uint8_t *radio_payload,
                                    p1_sync_frame_t *frame);
void encode_p2_data(uint8_t *dst, 
                                    const p2_data_frame_t *frame,
                                    const uint8_t *data);
void decode_p2_data(const uint8_t *radio_payload, 
                                    p2_data_frame_t *frame,
                                    uint8_t *data_out);
void encode_pre_p2_ctrl(uint8_t *dst, uint8_t class_id);
void decode_pre_p2_ctrl(const uint8_t *radio_payload,
                                        uint8_t *class_id);
void encode_pre_commit(uint8_t *dst, const pre_commit_frame_t *frame,
                                       const uint8_t *packed_schedule,
                                       size_t packed_len);
void decode_pre_commit(const uint8_t *radio_payload,
                                       uint8_t node_count,
                                       pre_commit_frame_t *frame,
                                       uint8_t *packed_schedule_out,
                                       size_t *packed_len_out);

#ifdef __cplusplus
}
#endif

#endif /* PACKET_H */

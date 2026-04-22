#ifndef TIMECAST_TTA_DRIVER_H
#define TIMECAST_TTA_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TTA_EVENT_NONE = 0,
    TTA_EVENT_RX_DONE,
    TTA_EVENT_TX_DONE,
    TTA_EVENT_RX_TIMEOUT,
    TTA_EVENT_ERROR,
} tta_event_type_t;

typedef struct {
    tta_event_type_t type;
    uint32_t timestamp_tick;
    uint8_t payload[127];
    size_t payload_len;
} tta_event_t;

typedef void (*tta_event_cb_t)(const tta_event_t *event, void *arg);

typedef struct {
    uint32_t rx_address;
    uint32_t rx_crc_ok;
    uint32_t rx_crc_fail;
    uint32_t tx_done;
    uint32_t evt_drop;
} tta_driver_stats_t;

int tta_driver_init(void);
int tta_driver_set_event_cb(tta_event_cb_t cb, void *arg);
int tta_driver_start(void);
int tta_driver_rx_enable(void);
int tta_driver_rx_disable(void);
int tta_driver_tx(const uint8_t *data, size_t len, uint32_t tx_deadline_ticks);
void tta_driver_process(void);
void tta_driver_get_stats(tta_driver_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* TIMECAST_TTA_DRIVER_H */

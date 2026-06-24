#ifndef LINK_RADIO_H
#define LINK_RADIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TX_CHAIN_DELAY           (10U)  

#define Adv_Ad_Lo      (0xABABABUL)
#define Adv_Ad_Hi      (0xABABC0UL)

#define S0_LEN       (1U)
#define LENGTH_LEN   (1U)
#define ADV_ADDR_LEN (6U)
#define HDR_LEN \
    (S0_LEN + LENGTH_LEN + ADV_ADDR_LEN)

const uint8_t *link_radio_payload(const uint8_t *packet);
bool tx_start(uint8_t *payload, uint32_t txen_ticks, uint32_t event_end_ticks, uint8_t packet_length);
bool rx_start(uint8_t *rx_buffer, uint32_t rxen_ticks, uint32_t rx_window_end_ticks, uint32_t rx_end_ticks);
uint32_t rx_listen_until_packet(uint8_t *rx_buffer, uint32_t rx_end_ticks, uint32_t runtimeout_us);
void wait_until_ticks(uint32_t slot_active_end_ticks);

#ifdef __cplusplus
}
#endif

#endif

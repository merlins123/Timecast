#ifndef TIMECAST_RADIO_DRIVER_H
#define TIMECAST_RADIO_DRIVER_H

#include <stdint.h>
#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

int radio_start(void);
uint32_t get_last_ready_time_ticks(void);
uint32_t get_last_end_time_ticks(void);
uint32_t get_last_address_time_ticks(void);
uint32_t now_ticks(void);
void timer2_reset_tx(void);
void timer2_reset_rx(void);
void radio_tx_arm(uint8_t *buf, uint32_t deadline_ticks);
void radio_rx_arm(uint8_t *buf, uint32_t deadline_ticks);
void _try_rx_enable(uint8_t *buf);

#define WAIT_UNTIL_ABS(cond, deadline) \
    do{ \
        while(!(cond) && ((int32_t)(now_ticks() - (deadline)) < 0)) {} \
    } while(0)

#define WAIT_UNTIL(cond, timeout_ticks) \
    do { \
        volatile uint32_t now; \
        now = now_ticks();  \
        while (!(cond) && ((int32_t)(now_ticks() - (now + (timeout_ticks))) < 0)) {} \
    } while (0)

#define TIMER_TICKS_PER_US   (16U)
#define US_TO_TIMER_TICKS(us) ((uint32_t)(us) << 4U)
#define TIMER_TICKS_TO_US(tk) ((uint32_t)(tk) >> 4U)
#define RADIO_RAMPUP_TIME_TICKS US_TO_TIMER_TICKS(40U)

#ifdef __cplusplus
}
#endif

#endif /* TIMECAST_RADIO_DRIVER_H */

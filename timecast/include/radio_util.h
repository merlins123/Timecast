#ifndef TIMECAST_RADIO_UTIL_H
#define TIMECAST_RADIO_UTIL_H

#include <stdint.h>

#ifdef CPU_FAM_NRF52
#include "cpu.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

uint32_t radio_util_get_last_address_time_ticks(void);
int radio_util_init(void);
int radio_util_ppi_anchor_init(void);
int radio_util_ppi_anchor_arm(uint32_t deadline_ticks);

#ifdef CPU_FAM_NRF52
#define TCAST_TIMER_TICKS_PER_US   (16U)
#define TCAST_US_TO_TIMER_TICKS(us) ((uint32_t)(us) << 4U)
#define TCAST_TIMER_TICKS_TO_US(tk) ((uint32_t)(tk) >> 4U)
#else
#define TCAST_TIMER_TICKS_PER_US   (1U)
#define TCAST_US_TO_TIMER_TICKS(us) ((uint32_t)(us))
#define TCAST_TIMER_TICKS_TO_US(tk) ((uint32_t)(tk))
#endif

static inline uint32_t radio_util_now_ticks(void)
{
#ifdef CPU_FAM_NRF52
    NRF_TIMER3->TASKS_CAPTURE[2] = 1;
    return NRF_TIMER3->CC[2];
#else
    return 0;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* TIMECAST_RADIO_UTIL_H */

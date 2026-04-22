//nRF52 Radio 底层操作
#include "radio_util.h"

#ifdef CPU_FAM_NRF52
#include "cpu.h"
#endif

#ifdef CPU_FAM_NRF52
#define BLUEFLOOD_PPI_TIMER      NRF_TIMER3
#define BLUEFLOOD_PPI_CH         (15U)
#define BLUEFLOOD_PPI_CH_TS      (14U)
#ifndef TCAST_TX_MIN_ARM_LEAD_US
#define TCAST_TX_MIN_ARM_LEAD_US (8U)
#endif
/* Keep compare slightly in the future to avoid programming an already-passed
 * compare edge. A large floor here directly adds TX jitter to relay nodes. */
#define BLUEFLOOD_MIN_ARM_US     (TCAST_TX_MIN_ARM_LEAD_US)

static int _ppi_anchor_inited;
#endif

int radio_util_init(void)
{
#ifdef CPU_FAM_NRF52
    return radio_util_ppi_anchor_init();
#else
    return 0;
#endif
}

int radio_util_ppi_anchor_init(void)
{
#ifdef CPU_FAM_NRF52
    if (_ppi_anchor_inited) {
        return 0;
    }

    BLUEFLOOD_PPI_TIMER->MODE = TIMER_MODE_MODE_Timer;
    BLUEFLOOD_PPI_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    BLUEFLOOD_PPI_TIMER->PRESCALER = 0; /* 16 MHz / 2^0 = 16 MHz (1 tick = 62.5 ns) */
    BLUEFLOOD_PPI_TIMER->TASKS_CLEAR = 1;
    BLUEFLOOD_PPI_TIMER->TASKS_START = 1;

    NRF_PPI->CH[BLUEFLOOD_PPI_CH].EEP = (uint32_t)&BLUEFLOOD_PPI_TIMER->EVENTS_COMPARE[0];
    /* Scheme 2: Link Timer Compare directly to Radio TX Enable for zero-jitter trigger */
    NRF_PPI->CH[BLUEFLOOD_PPI_CH].TEP = (uint32_t)&NRF_RADIO->TASKS_TXEN;
    NRF_PPI->CHENSET = (1u << BLUEFLOOD_PPI_CH);
    
    /* Configure PPI for RX Timestamping: ADDRESS -> CAPTURE[1] */
    NRF_PPI->CH[BLUEFLOOD_PPI_CH_TS].EEP = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[BLUEFLOOD_PPI_CH_TS].TEP = (uint32_t)&BLUEFLOOD_PPI_TIMER->TASKS_CAPTURE[1];
    NRF_PPI->CHENSET |= (1u << BLUEFLOOD_PPI_CH_TS);

    _ppi_anchor_inited = 1;
#endif
    return 0;
}

uint32_t radio_util_get_last_address_time_ticks(void)
{
#ifdef CPU_FAM_NRF52
    return BLUEFLOOD_PPI_TIMER->CC[1];
#else
    return 0;
#endif
}

int radio_util_ppi_anchor_arm(uint32_t deadline_ticks)
{
#ifdef CPU_FAM_NRF52
    if (!_ppi_anchor_inited) {
        radio_util_ppi_anchor_init();
    }

    uint32_t now_ticks = radio_util_now_ticks();
    int32_t delta_ticks = (int32_t)(deadline_ticks - now_ticks);
    int32_t min_arm_ticks = (int32_t)TCAST_US_TO_TIMER_TICKS(BLUEFLOOD_MIN_ARM_US);
    if (delta_ticks <= 0) {
        return -1;
    }
    if (delta_ticks < min_arm_ticks) {
        delta_ticks = min_arm_ticks;
    }

    BLUEFLOOD_PPI_TIMER->EVENTS_COMPARE[0] = 0;
    BLUEFLOOD_PPI_TIMER->CC[0] = now_ticks + (uint32_t)delta_ticks;
#else
    (void)deadline_ticks;
#endif
    return 0;
}

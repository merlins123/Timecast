#include "radio_driver.h"



static const uint8_t ble_hw_frequency_channels[40] = {
     4,  6,  8, 10, 12, 14, 16, 18, 20, 22, /* 0-9 */
    24, 28, 30, 32, 34, 36, 38, 40, 42, 44, /* 10-19 */
    46, 48, 50, 52, 54, 56, 58, 60, 62, 64, /* 20-29 */
    66, 68, 70, 72, 74, 76, 78,  2, 26, 80, /* 30-39 */
};

#ifndef ACCESS_ADDRESS
#define ACCESS_ADDRESS (0x8E89BED6u)
#endif
#ifndef FAST_RAMPUP
#define FAST_RAMPUP (1U)
#endif
#define AA_PREFIX ((uint32_t)((ACCESS_ADDRESS >> 24) & 0xFFu))
#define AA_BASE   ((uint32_t)((ACCESS_ADDRESS << 8) & 0xFFFFFF00u))

#define READY_TIME_PPI_CH   (9U)
#define END_TIME_PPI_CH     (10u)
#define TX_START_PPI_CH     (11U)
#define TIMER2_START_PPI_CH (12U)
#define RX_EN_PPI_CH        (13U)
#define TX_EN_PPI_CH        (15U)
#define ADDRESS_TIME_PPI_CH (14U)

static int _ppi_anchor_inited;

static void timer2_init(void)
{
    NRF_TIMER2->TASKS_SHUTDOWN = 1;

    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER2->PRESCALER = 0;
    NRF_TIMER2->TASKS_CLEAR = 1;

    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    NRF_TIMER2->INTENCLR = 0xffffffffUL;

    NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_STOP_Msk | TIMER_SHORTS_COMPARE0_CLEAR_Msk;

    NRF_PPI->CH[TIMER2_START_PPI_CH].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[0];
    NRF_PPI->CH[TIMER2_START_PPI_CH].TEP = (uint32_t)&NRF_TIMER2->TASKS_START;

    NRF_PPI->CH[TX_START_PPI_CH].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
    NRF_PPI->CH[TX_START_PPI_CH].TEP = (uint32_t)&NRF_RADIO->TASKS_START;

    NRF_PPI->CHENSET = (1UL << TIMER2_START_PPI_CH) | (1UL << TX_START_PPI_CH);
}

static int _timer_ppi_init(void)
{
    if (_ppi_anchor_inited) {
        return 0;
    }

    NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER3->PRESCALER = 0;
    NRF_TIMER3->TASKS_CLEAR = 1;
    NRF_TIMER3->TASKS_START = 1;

    NRF_PPI->CH[TX_EN_PPI_CH].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[0];
    NRF_PPI->CH[TX_EN_PPI_CH].TEP = (uint32_t)&NRF_RADIO->TASKS_TXEN;

    NRF_PPI->CH[RX_EN_PPI_CH].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[0];
    NRF_PPI->CH[RX_EN_PPI_CH].TEP = (uint32_t)&NRF_RADIO->TASKS_RXEN;

    NRF_PPI->CH[ADDRESS_TIME_PPI_CH].EEP = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[ADDRESS_TIME_PPI_CH].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[1];
    NRF_PPI->CHENSET = (1UL << ADDRESS_TIME_PPI_CH);

    NRF_PPI->CH[END_TIME_PPI_CH].EEP = (uint32_t)&NRF_RADIO->EVENTS_END;
    NRF_PPI->CH[END_TIME_PPI_CH].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[3];
    NRF_PPI->CHENSET = (1UL << END_TIME_PPI_CH);

    NRF_PPI->CH[READY_TIME_PPI_CH].EEP = (uint32_t)&NRF_RADIO->EVENTS_READY;
    NRF_PPI->CH[READY_TIME_PPI_CH].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[4];
    NRF_PPI->CHENSET = (1UL << READY_TIME_PPI_CH);

    timer2_init();
    _ppi_anchor_inited = 1;

    return 0;
}

uint32_t get_last_ready_time_ticks(void)
{
    return NRF_TIMER3->CC[4];
}


uint32_t get_last_end_time_ticks(void)
{
    return NRF_TIMER3->CC[3];
}

uint32_t get_last_address_time_ticks(void)
{
    return NRF_TIMER3->CC[1];
}

uint32_t now_ticks(void)
{
    NRF_TIMER3->TASKS_CAPTURE[2] = 1;
    return NRF_TIMER3->CC[2];
}

void timer2_reset_tx(void)
{
    NRF_PPI->CHENSET = (1UL << TX_EN_PPI_CH);
    NRF_PPI->CHENCLR = (1UL << RX_EN_PPI_CH);

  
    NRF_TIMER2->TASKS_SHUTDOWN = 1;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    NRF_TIMER2->CC[0] = RADIO_RAMPUP_TIME_TICKS;

    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_RADIO->EVENTS_DISABLED = 0U;
    NRF_RADIO->TASKS_DISABLE = 1U;

    NRF_RADIO->EVENTS_END = 0U;
    NRF_RADIO->EVENTS_ADDRESS = 0U;
    NRF_RADIO->EVENTS_READY = 0U;

    WAIT_UNTIL(NRF_RADIO->EVENTS_DISABLED != 0U, US_TO_TIMER_TICKS(2U));
    NRF_RADIO->EVENTS_DISABLED = 0U;
}

void radio_tx_arm(uint8_t *buf, uint32_t deadline_ticks)
{
    timer2_reset_tx();
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->CC[0] = deadline_ticks;
    NRF_RADIO->PACKETPTR = (uint32_t)buf;
}

void timer2_reset_rx(void)
{
    NRF_PPI->CHENSET = (1UL << RX_EN_PPI_CH);
    NRF_PPI->CHENCLR = (1UL << TX_EN_PPI_CH);

    NRF_TIMER2->TASKS_SHUTDOWN = 1;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    NRF_TIMER2->CC[0] = RADIO_RAMPUP_TIME_TICKS;

    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_RADIO->EVENTS_DISABLED = 0U;
    NRF_RADIO->TASKS_DISABLE = 1U;

    NRF_RADIO->EVENTS_END = 0U;
    NRF_RADIO->EVENTS_ADDRESS = 0U;
    NRF_RADIO->EVENTS_READY = 0U;

    WAIT_UNTIL(NRF_RADIO->EVENTS_DISABLED != 0U, US_TO_TIMER_TICKS(2U));
    NRF_RADIO->EVENTS_DISABLED = 0U;
}

void radio_rx_arm(uint8_t* buf, uint32_t deadline_ticks)
{
    timer2_reset_rx();
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->CC[0] = deadline_ticks;
    NRF_RADIO->PACKETPTR = (uint32_t)buf;

}

void _try_rx_enable(uint8_t *buf)
{   
    NRF_RADIO->EVENTS_DISABLED = 0U;
    NRF_RADIO->TASKS_DISABLE = 1;
    WAIT_UNTIL(NRF_RADIO->EVENTS_DISABLED != 0U, US_TO_TIMER_TICKS(2U));
    
    NRF_TIMER2->TASKS_SHUTDOWN  = 1;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    NRF_TIMER2->CC[0]       = RADIO_RAMPUP_TIME_TICKS; 

    NRF_RADIO->EVENTS_READY = 0U;
    NRF_RADIO->EVENTS_DISABLED = 0U;
    NRF_RADIO->EVENTS_END = 0U;
    NRF_RADIO->EVENTS_ADDRESS = 0U;
    NRF_RADIO->EVENTS_PAYLOAD = 0U;

    NRF_RADIO->PACKETPTR = (uint32_t)buf;

    NRF_RADIO->TASKS_RXEN = 1;
    NRF_TIMER2->TASKS_START = 1;
}

static void _init_radio(void)
{
    /* Ensure high frequency crystal is running for reliable RADIO operation. */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

    /* Enable Radio Power */
    NRF_RADIO->POWER = 1;
    NRF_RADIO->TASKS_DISABLE = 1;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_ADDRESS = 0;

    /* Configure Radio for BLE 1Mbit */
    NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit;
    uint32_t modecnf0 = (RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos);
#if FAST_RAMPUP
    modecnf0 |= (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
#else
    modecnf0 |= (RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos);
#endif
    NRF_RADIO->MODECNF0 = modecnf0;

    NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos;
    NRF_RADIO->FREQUENCY = ble_hw_frequency_channels[37];      /* BLE Advertising channel*/
    NRF_RADIO->DATAWHITEIV = 5;   /* Whitening IV must match BLE channel index */

    /* Configure Packet Address */
    NRF_RADIO->PREFIX0 = AA_PREFIX;
    NRF_RADIO->BASE0 = AA_BASE;
    NRF_RADIO->TXADDRESS = 0;
    NRF_RADIO->RXADDRESSES = 1;      /*bitmask*/ 

    /* Configure Packet Format (Simple proprietary or BLE-like) */
    /* LFLEN=8bit, S0LEN=1 (PDU Header), S1LEN=0 */
    NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                       (8 << RADIO_PCNF0_LFLEN_Pos) |
                       (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos) |
                       (RADIO_PCNF0_S1INCL_Automatic << RADIO_PCNF0_S1INCL_Pos)|
                       (0 << RADIO_PCNF0_S1LEN_Pos);
    /* MAXLEN=251, STATLEN=0, BALEN=3(+1) */
    NRF_RADIO->PCNF1 = (251 << RADIO_PCNF1_MAXLEN_Pos) | 
                       (3 << RADIO_PCNF1_BALEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                       (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

    /* CRC Config */
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                        (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT = 0x555555;
    NRF_RADIO->CRCPOLY = 0x00065B;

    /* Shortcut: automatically disable after packet end. */
    NRF_RADIO->SHORTS = (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

}




int radio_start(void)
{
    _init_radio();
    _timer_ppi_init();


    return 0;
}

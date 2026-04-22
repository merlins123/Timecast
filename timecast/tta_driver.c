#include "tta_driver.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include "cpu.h"

#include "radio_util.h"

static tta_event_cb_t _event_cb;
static void *_event_arg;
static int _started;

/* Radio packet buffer (must be aligned for DMA if needed, though nRF52 handles unaligned mostly fine) */
static uint8_t _radio_packet[260] __attribute__((aligned(4)));
static uint8_t _rx_packet[260] __attribute__((aligned(4)));

static volatile int _tx_scheduled;
static volatile int _tx_active;
static volatile int _rx_active;
static volatile int _rx_address_seen;
static volatile uint32_t _tx_armed_at_ticks;
static volatile uint32_t _stat_rx_address;
static volatile uint32_t _stat_rx_crc_ok;
static volatile uint32_t _stat_rx_crc_fail;
static volatile uint32_t _stat_tx_done;
static volatile uint32_t _stat_evt_drop;

static const uint8_t ble_hw_frequency_channels[40] = {
     4,  6,  8, 10, 12, 14, 16, 18, 20, 22, /* 0-9 */
    24, 28, 30, 32, 34, 36, 38, 40, 42, 44, /* 10-19 */
    46, 48, 50, 52, 54, 56, 58, 60, 62, 64, /* 20-29 */
    66, 68, 70, 72, 74, 76, 78,  2, 26, 80, /* 30-39 */
};

#define TX_STUCK_TIMEOUT_US (2000U)
#define TX_DISABLE_WAIT_TIMEOUT_US (120U)
#ifndef TCAST_TX_MIN_ARM_LEAD_US
#define TCAST_TX_MIN_ARM_LEAD_US (8U)
#endif
#ifndef TCAST_ACCESS_ADDRESS
#define TCAST_ACCESS_ADDRESS (0xA7B3C5D9u)
#endif
#ifndef TCAST_FAST_RAMPUP
#define TCAST_FAST_RAMPUP (1U)
#endif
#define TCAST_AA_PREFIX ((uint32_t)((TCAST_ACCESS_ADDRESS >> 24) & 0xFFu))
#define TCAST_AA_BASE   ((uint32_t)((TCAST_ACCESS_ADDRESS << 8) & 0xFFFFFF00u))

/* Increase queue size to buffer burst events during main loop sleep */
#define EVT_QUEUE_SIZE 32
static tta_event_t _evt_queue[EVT_QUEUE_SIZE];
static volatile unsigned _evt_head;
static volatile unsigned _evt_tail;


static void _emit_event(tta_event_type_t type, const uint8_t *payload,
                        size_t len, uint32_t timestamp_tick)
{
    if (!_event_cb) {
        return;
    }

    tta_event_t ev;
    ev.type = type;
    ev.timestamp_tick = timestamp_tick;
    ev.payload_len = (len <= sizeof(ev.payload)) ? len : sizeof(ev.payload);

    if (payload && ev.payload_len) {
        memcpy(ev.payload, payload, ev.payload_len);
    }

    _event_cb(&ev, _event_arg);
}

static void _defer_event_from_isr(tta_event_type_t type, const uint8_t *payload,
                                  size_t len, uint32_t timestamp_tick)
{
    unsigned next = (_evt_head + 1) % EVT_QUEUE_SIZE;
    if (next == _evt_tail) {
        _stat_evt_drop++;
        return; /* Queue full, drop event */
    }
    
    _evt_queue[_evt_head].type = type;
    _evt_queue[_evt_head].timestamp_tick = timestamp_tick;
    _evt_queue[_evt_head].payload_len = (len <= sizeof(_evt_queue[_evt_head].payload)) ? len : sizeof(_evt_queue[_evt_head].payload);
    if (payload && len) memcpy(_evt_queue[_evt_head].payload, payload, _evt_queue[_evt_head].payload_len);
    _evt_head = next;
}

static int _wait_radio_disabled(uint32_t timeout_ticks)
{
    uint32_t start_ticks = radio_util_now_ticks();
    while (NRF_RADIO->EVENTS_DISABLED == 0) {
        if ((int32_t)(radio_util_now_ticks() - start_ticks) > (int32_t)timeout_ticks) {
            return -ETIMEDOUT;
        }
    }
    return 0;
}

/* Interrupt handler for Radio events */
void isr_radio(void)
{
    if (NRF_RADIO->EVENTS_ADDRESS) {
        NRF_RADIO->EVENTS_ADDRESS = 0;
        _rx_address_seen = 1;
    }

    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;
        /* Transmission finished */
        if (_tx_active) {
            _tx_active = 0;
            _tx_scheduled = 0;
            _stat_tx_done++;
            /* Use hardware-captured ADDRESS edge as TX timestamp to avoid ISR-entry jitter. */
            uint32_t tx_tick = radio_util_get_last_address_time_ticks();
            _defer_event_from_isr(TTA_EVENT_TX_DONE, NULL, 0, tx_tick);
        } else if (_rx_active) {
            /* Check CRC status */
            _stat_rx_address++;
            if (NRF_RADIO->CRCSTATUS) {
                /* Packet format: [S0 (1B)] [LEN (1B)] [PAYLOAD...].
                 * len is 8-bit in this PHY config, so it is always <= 255. */
                uint8_t len = _rx_packet[1];
                uint32_t rx_tick = radio_util_get_last_address_time_ticks();
                _stat_rx_crc_ok++;
                _defer_event_from_isr(TTA_EVENT_RX_DONE, &_rx_packet[2], len, rx_tick);
            }
            else {
                _stat_rx_crc_fail++;
            }
            
            _rx_active = 0;
            _rx_address_seen = 0;
        }
    }
    cortexm_isr_end();
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
#if defined(RADIO_MODECNF0_RU_Msk) && defined(RADIO_MODECNF0_DTX_Center)
    /* Match Blueflood: use fast radio ramp-up on nRF52-class devices. */
    uint32_t modecnf0 = (RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos);
#if TCAST_FAST_RAMPUP
    modecnf0 |= (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
#else
    modecnf0 |= (RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos);
#endif
    NRF_RADIO->MODECNF0 = modecnf0;
#endif
    NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;
    NRF_RADIO->FREQUENCY = ble_hw_frequency_channels[37];      /* BLE Advertising channel*/
    NRF_RADIO->DATAWHITEIV = 5;   /* Whitening IV must match BLE channel index */

    /* Configure Packet Address */
    NRF_RADIO->PREFIX0 = TCAST_AA_PREFIX;
    NRF_RADIO->BASE0 = TCAST_AA_BASE;
    NRF_RADIO->TXADDRESS = 0;
    NRF_RADIO->RXADDRESSES = 1;

    /* Configure Packet Format (Simple proprietary or BLE-like) */
    /* LFLEN=8bit, S0LEN=1 (PDU Header), S1LEN=0 */
    NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                       (8 << RADIO_PCNF0_LFLEN_Pos) |
                       (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos) |
                       (RADIO_PCNF0_S1INCL_Automatic << RADIO_PCNF0_S1INCL_Pos);
    /* MAXLEN=255, STATLEN=0, BALEN=3(+1) */
    NRF_RADIO->PCNF1 = (255 << RADIO_PCNF1_MAXLEN_Pos) | 
                       (3 << RADIO_PCNF1_BALEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                       (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

    /* CRC Config */
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                        (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT = 0x555555;
    NRF_RADIO->CRCPOLY = 0x00065B;

    /* Shortcuts: Ready->Start, End->Disable */
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

    /* Enable Interrupts */
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk | RADIO_INTENSET_ADDRESS_Msk;
    NVIC_EnableIRQ(RADIO_IRQn);
}

int tta_driver_init(void)
{
    _event_cb = NULL;
    _event_arg = NULL;
    _started = 0;

    _tx_scheduled = 0;
    _tx_active = 0;
    _rx_active = 0;
    _rx_address_seen = 0;
    _tx_armed_at_ticks = 0;
    _stat_rx_address = 0;
    _stat_rx_crc_ok = 0;
    _stat_rx_crc_fail = 0;
    _stat_tx_done = 0;
    _stat_evt_drop = 0;
    _evt_head = 0;
    _evt_tail = 0;

    return 0;
}

int tta_driver_set_event_cb(tta_event_cb_t cb, void *arg)
{
    _event_cb = cb;
    _event_arg = arg;
    return 0;
}

int tta_driver_start(void)
{
    _init_radio();
    radio_util_ppi_anchor_init();

    _started = 1;
    return 0;
}

int tta_driver_rx_enable(void)
{
    uint32_t radio_state;

    if (!_started) {
        return -EAGAIN;
    }
    if (_tx_scheduled) {
        return -EBUSY;
    }
    
    /* If already active, do not re-trigger RX to avoid resetting the radio */
    if (_rx_active) {
        return 0;
    }

    radio_state = NRF_RADIO->STATE;
    if (radio_state != RADIO_STATE_STATE_Disabled) {
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        if (_wait_radio_disabled(TCAST_US_TO_TIMER_TICKS(TX_DISABLE_WAIT_TIMEOUT_US)) < 0) {
            return -EBUSY;
        }
    }

    NRF_RADIO->PACKETPTR = (uint32_t)_rx_packet;
    _rx_address_seen = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    _rx_active = 1;

    return 0;
}

int tta_driver_rx_disable(void)
{
    if (!_started) {
        return -EAGAIN;
    }
    if (!_rx_active) {
        return 0;
    }
    if (_rx_address_seen) {
        return -EBUSY;
    }

    _rx_active = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;

    return 0;
}

int tta_driver_tx(const uint8_t *data, size_t len, uint32_t tx_deadline_ticks)
{
    uint32_t radio_state;

    if (!_started) {
        return -EAGAIN;
    }
    if (!data && len) {
        return -EINVAL;
    }
    /* Layout: [2B header][6B AdvA][len bytes payload] */
    if (len > (sizeof(_radio_packet) - 8)) {
        return -EMSGSIZE;
    }
    while (_tx_scheduled) {
        uint32_t now_ticks = radio_util_now_ticks();
        uint32_t cur_radio_state = NRF_RADIO->STATE;

        if ((_tx_active == 0) || (cur_radio_state == RADIO_STATE_STATE_Disabled)) {
            _tx_scheduled = 0;
            _tx_active = 0;
            break;
        }

        if ((int32_t)(now_ticks - _tx_armed_at_ticks) >
            (int32_t)TCAST_US_TO_TIMER_TICKS(TX_STUCK_TIMEOUT_US)) {
            _tx_scheduled = 0;
            _tx_active = 0;
            _rx_active = 0;
            NRF_RADIO->TASKS_DISABLE = 1;
            break;
        }

        if ((int32_t)(tx_deadline_ticks - now_ticks) <=
            (int32_t)TCAST_US_TO_TIMER_TICKS(TCAST_TX_MIN_ARM_LEAD_US)) {
            return -EBUSY;
        }
    }

    if (_rx_active) {
        _rx_active = 0;
        _rx_address_seen = 0;
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        if (_wait_radio_disabled(TCAST_US_TO_TIMER_TICKS(TX_DISABLE_WAIT_TIMEOUT_US)) < 0) {
            return -EBUSY;
        }
    }

    radio_state = NRF_RADIO->STATE;
    if (radio_state != RADIO_STATE_STATE_Disabled) {
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        if (_wait_radio_disabled(TCAST_US_TO_TIMER_TICKS(TX_DISABLE_WAIT_TIMEOUT_US)) < 0) {
            return -EBUSY;
        }
    }

    /* Prepare Packet in Buffer (BLE Advertising Channel PDU) */
    /* Header (2B) + Payload (AdvA 6B + Data) */
    _radio_packet[0] = 0x02; /* S0: PDU Type 2 (ADV_NONCONN_IND), TxAdd=0 (Public), RxAdd=0 */
    _radio_packet[1] = len + 6;  /* Length field: Data len + 6 bytes AdvA */
    /* Dummy Advertiser Address (Public) - required for valid BLE packet */
    const uint8_t dummy_addr[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    memcpy(&_radio_packet[2], dummy_addr, 6);
    memcpy(&_radio_packet[8], data, len);

    NRF_RADIO->PACKETPTR = (uint32_t)_radio_packet;
    _tx_scheduled = 1;
    _tx_active = 1;
    _tx_armed_at_ticks = radio_util_now_ticks();
    if (radio_util_ppi_anchor_arm(tx_deadline_ticks) < 0) {
        _tx_scheduled = 0;
        _tx_active = 0;
        return -EBUSY;
    }
    return 0;
}


void tta_driver_process(void)
{
    while ((_evt_tail != _evt_head) && _event_cb) {
        tta_event_t ev = _evt_queue[_evt_tail];
        _evt_tail = (_evt_tail + 1) % EVT_QUEUE_SIZE;
        _emit_event(ev.type, ev.payload, ev.payload_len, ev.timestamp_tick);
    }
}

void tta_driver_get_stats(tta_driver_stats_t *stats)
{
    if (!stats) {
        return;
    }
    stats->rx_address = _stat_rx_address;
    stats->rx_crc_ok = _stat_rx_crc_ok;
    stats->rx_crc_fail = _stat_rx_crc_fail;
    stats->tx_done = _stat_tx_done;
    stats->evt_drop = _stat_evt_drop;
}

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nimble_autoadv.h"
#include "nimble_autoadv_params.h"
#include "nimble_riot.h"

#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"

#include "net/bluetil/ad.h"

#include "ztimer.h"

#define BROADCAST_STATUS_INTERVAL_MS      (5000U)

/* Public Broadcast Announcement UUID */
#define PBA_SERVICE_UUID                  0x1852

/* Sample Auracast broadcast configuration */
static const uint8_t _auracast_broadcast_id[3] = { 0x46, 0x52, 0x01 }; /* "FR\1" */
static const char _auracast_program_info[] = "RIOT";

/* Manufacturer data with a fake company identifier (0xFFFF) */
static const uint8_t _vendor_metadata[] = {
    0xff, 0xff,               /* reserved company identifier */
    0x01,                     /* demo revision */
    0x01,                     /* 1 audio stream, hard coded */
    0x02,                     /* language = English */
    0x00,                     /* free-form flags */
};

static uint32_t _next_status_tick;

static int _gap_event(struct ble_gap_event *event, void *arg);
static void _configure_auracast_adv(void);
static void _log_status(void);

static void _add_field(uint8_t type, const void *data, size_t len)
{
    int res = nimble_autoadv_add_field(type, data, len);
    if (res != BLUETIL_AD_OK) {
        printf("auracast: failed to add adv field 0x%02x (%d)\n", type, res);
    }
}

static void _setup_pba_service_data(void)
{
    /* Service data layout: UUID + broadcast ID + features + metadata length + metadata */
    uint8_t service_data[2 + sizeof(_auracast_broadcast_id) + 2
                         + sizeof(_auracast_program_info) - 1];
    uint8_t *pos = service_data;

    *pos++ = (uint8_t)(PBA_SERVICE_UUID & 0xff);
    *pos++ = (uint8_t)(PBA_SERVICE_UUID >> 8);

    memcpy(pos, _auracast_broadcast_id, sizeof(_auracast_broadcast_id));
    pos += sizeof(_auracast_broadcast_id);

    *pos++ = 0x01;                      /* features: general broadcast */
    *pos++ = sizeof(_auracast_program_info) - 1;

    memcpy(pos, _auracast_program_info, sizeof(_auracast_program_info) - 1);

    _add_field(BLE_HS_ADV_TYPE_SVC_DATA_UUID16, service_data, sizeof(service_data));
}

static void _configure_auracast_adv(void)
{
    nimble_autoadv_cfg_t cfg;
    nimble_autoadv_get_cfg(&cfg);

    cfg.adv_itvl_ms = 80;
    cfg.flags &= ~(NIMBLE_AUTOADV_FLAG_CONNECTABLE | NIMBLE_AUTOADV_FLAG_SCANNABLE);
    cfg.flags &= ~NIMBLE_AUTOADV_FLAG_LEGACY;
    cfg.phy = NIMBLE_PHY_1M;
    cfg.tx_power = 3;

    nimble_autoadv_cfg_update(&cfg);

    _setup_pba_service_data();
    _add_field(BLE_HS_ADV_TYPE_MFG_DATA, _vendor_metadata,
               sizeof(_vendor_metadata));
}

static int _gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            puts("auracast: unexpected connection, disconnecting");
            ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        else {
            printf("auracast: connection attempt failed (%d), restarting adv\n",
                   event->connect.status);
            nimble_autoadv_start(NULL);
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        puts("auracast: disconnected, restarting adv");
        nimble_autoadv_start(NULL);
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        puts("auracast: adv completed, restarting");
        nimble_autoadv_start(NULL);
        break;
    default:
        break;
    }

    return 0;
}

static void _log_status(void)
{
    uint32_t now = ztimer_now(ZTIMER_MSEC);
    if (now < _next_status_tick) {
        return;
    }
    _next_status_tick = now + BROADCAST_STATUS_INTERVAL_MS;

    printf("auracast: broadcasting ID %02X%02X%02X \"%s\"\n",
           _auracast_broadcast_id[0],
           _auracast_broadcast_id[1],
           _auracast_broadcast_id[2],
           _auracast_program_info);
}

int main(void)
{
    puts("RIOT Auracast demo starting...");

    nimble_riot_init();

    _configure_auracast_adv();
    nimble_autoadv_set_gap_cb(_gap_event, NULL);
    nimble_autoadv_start(NULL);

    _next_status_tick = ztimer_now(ZTIMER_MSEC);

    while (1) {
        ztimer_sleep(ZTIMER_MSEC, 250);
        _log_status();
    }

    return 0;
}

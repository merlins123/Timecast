#include "store.h"

#include <string.h>

static bool _valid_node_id(uint8_t node_id)
{
    return (node_id < TIMECAST_STORE_MAX_NODES);
}

static bool _valid_store_payload(const timecast_store_t *store, uint8_t node_id,
                                 const void *data, uint8_t len)
{
    return store && data && (len > 0U) &&
           (len <= TIMECAST_STORE_MAX_DATA_LEN) &&
           _valid_node_id(node_id);
}

void store_init(timecast_store_t *store, uint8_t local_node_id)
{
    if (!store) {
        return;
    }

    store->local_node_id = local_node_id;
    memset(store->entries, 0, sizeof(store->entries));
    store->present_count = 0;

}



bool store_import(timecast_store_t *store, uint8_t node_id,
                           const void *data, uint8_t len)
{
    timecast_store_entry_t *entry;

    if (!_valid_store_payload(store, node_id, data, len)) {
        return false;
    }

    entry = &store->entries[node_id];
    if (entry->present) {
        return false;
    }

    if (!entry->present) {
        store->present_count++;
    }

    memcpy(entry->data, data, len);
    entry->len = len;
    entry->present = true;
    return true;
}

bool store_has_data(const timecast_store_t *store, uint8_t node_id)
{
    return store && _valid_node_id(node_id) && store->entries[node_id].present;
}

uint16_t store_present_count(const timecast_store_t *store)
{
    return store ? store->present_count : 0U;
}



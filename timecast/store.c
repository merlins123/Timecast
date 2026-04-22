#include "store.h"

#include <string.h>

static bool _valid_node_id(uint8_t node_id)
{
    return (node_id < TIMECAST_STORE_MAX_NODES);
}

void timecast_store_clear(timecast_store_t *store)
{
    if (!store) {
        return;
    }

    memset(store->entries, 0, sizeof(store->entries));
    store->participant_count = 0;
    store->present_count = 0;
}

void timecast_store_init(timecast_store_t *store, uint8_t local_node_id)
{
    if (!store) {
        return;
    }

    store->local_node_id = local_node_id;
    timecast_store_clear(store);
}

bool timecast_store_mark_participant(timecast_store_t *store, uint8_t node_id)
{
    timecast_store_entry_t *entry;

    if (!store || !_valid_node_id(node_id)) {
        return false;
    }

    entry = &store->entries[node_id];
    if (!entry->participating) {
        entry->participating = true;
        store->participant_count++;
    }

    return true;
}

bool timecast_store_write(timecast_store_t *store, uint8_t node_id,
                          const void *data, uint8_t len)
{
    timecast_store_entry_t *entry;

    if (!store || !data || (len == 0U) || (len > TIMECAST_STORE_MAX_DATA_LEN) ||
        !_valid_node_id(node_id)) {
        return false;
    }

    entry = &store->entries[node_id];
    if (!entry->participating) {
        entry->participating = true;
        store->participant_count++;
    }
    if (!entry->present) {
        store->present_count++;
    }

    memcpy(entry->data, data, len);
    entry->len = len;
    entry->present = true;

    return true;
}

bool timecast_store_write_local(timecast_store_t *store, const void *data, uint8_t len)
{
    if (!store) {
        return false;
    }

    return timecast_store_write(store, store->local_node_id, data, len);
}

bool timecast_store_import(timecast_store_t *store, uint8_t node_id,
                           const void *data, uint8_t len)
{
    timecast_store_entry_t *entry;

    if (!store || !data || (len == 0U) || (len > TIMECAST_STORE_MAX_DATA_LEN) ||
        !_valid_node_id(node_id)) {
        return false;
    }

    entry = &store->entries[node_id];
    if (entry->present) {
        return false;
    }

    if (!entry->participating) {
        entry->participating = true;
        store->participant_count++;
    }
    if (!entry->present) {
        store->present_count++;
    }

    memcpy(entry->data, data, len);
    entry->len = len;
    entry->present = true;

    return true;
}

const timecast_store_entry_t *timecast_store_get(const timecast_store_t *store, uint8_t node_id)
{
    if (!store || !_valid_node_id(node_id)) {
        return NULL;
    }

    return &store->entries[node_id];
}

bool timecast_store_has_data(const timecast_store_t *store, uint8_t node_id)
{
    const timecast_store_entry_t *entry = timecast_store_get(store, node_id);

    return entry ? entry->present : false;
}

uint16_t timecast_store_present_count(const timecast_store_t *store)
{
    return store ? store->present_count : 0U;
}

uint16_t timecast_store_participant_count(const timecast_store_t *store)
{
    return store ? store->participant_count : 0U;
}

bool timecast_store_is_complete(const timecast_store_t *store)
{
    return store && (store->participant_count > 0U) &&
           (store->present_count >= store->participant_count);
}

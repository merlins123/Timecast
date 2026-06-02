#ifndef TIMECAST_STORE_H
#define TIMECAST_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TIMECAST_STORE_MAX_NODES
#define TIMECAST_STORE_MAX_NODES (64U)
#endif

#ifndef TIMECAST_STORE_MAX_DATA_LEN
#define TIMECAST_STORE_MAX_DATA_LEN (110U)
#endif

typedef struct {
    bool present;          
    uint8_t len;
    uint8_t data[TIMECAST_STORE_MAX_DATA_LEN];
} timecast_store_entry_t;

typedef struct {
    uint8_t local_node_id;
    uint16_t present_count;
    timecast_store_entry_t entries[TIMECAST_STORE_MAX_NODES];
} timecast_store_t;

void store_init(timecast_store_t *store, uint8_t local_node_id);
void store_mark_participant(timecast_store_t *store, uint8_t node_id);
bool store_import(timecast_store_t *store, uint8_t node_id,
                           const void *data, uint8_t len);
bool store_has_data(const timecast_store_t *store, uint8_t node_id);
uint16_t store_present_count(const timecast_store_t *store);


#ifdef __cplusplus
}
#endif

#endif 

#ifndef GPS_DRIVER_H_
#define GPS_DRIVER_H_

#include "inttypes.h"

typedef enum
{ // TODO: add more error codes
    GPS_OK = 0,
    GPS_ERROR = -1,
    GPS_ERROR_WRONG_PARAM = -2,
    GPS_ERROR_TIMEOUT = -3,
    GPS_ERROR_NO_MEM = -4,

    GPS_ERROR_UNKNOWN = -255,
} gps_error_t;

typedef enum
{
    GPS_MEMORY_LAYER_RAM = 0b00000001,
    GPS_MEMORY_LAYER_BBR = 0b00000010,
    GPS_MEMORY_LAYER_FLASH = 0b00000100,
    GPS_MEMOTY_LAYER_MAX = 0b00000111
} gps_memory_layer_t;

typedef struct
{
    uint32_t key;
    union
    {
        uint8_t u1;
        uint16_t u2;
        uint32_t u4;
        uint64_t u8;
        int8_t i1;
        int16_t i2;
        int32_t i4;
        int64_t i8;
        float r4;
        double r8;
        uint8_t e1;
        uint16_t e2;
        uint32_t e4;
        uint8_t x1;
        uint16_t x2;
        uint32_t x4;
        uint64_t x8;
        uint8_t l;
    } value; // TODO muszą być little-endian. Jak to sprawdzić? Może #define?

} gps_config_item_t;

#define MAX_CONFIG_ITEMS     (64U)
#define MAX_CONFIG_ITEM_SIZE (12U)

//============== CONFIGURATION API ==============

// GENERAL CONFIGURATION

/**
 * @brief Set the configuration manually based on the GPS documentation.
 * Useful for options not supported by this library.
 * List of configs: section 5.8 of
 * https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf
 *
 * @param layer
 * @param config
 * @param config_len number of config items, 0 < config_len <= MAX_CONFIG_ITEMS
 * @return gps_error_t
 */
gps_error_t gps_set_config(gps_memory_layer_t layer, gps_config_item_t config[], uint8_t config_len);

#endif // GPS_DRIVER_H_

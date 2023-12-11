#ifndef GPS_DRIVER_H_
#define GPS_DRIVER_H_

#include "inttypes.h"
#include "ubx-cfg-defines.h"
#include <errno.h>

typedef enum
{ // TODO: add more error codes
    GPS_OK = 0,
    GPS_ERROR = EPROTO,
    GPS_ERROR_WRONG_PARAM = ENOTSUP,
    GPS_ERROR_TIMEOUT = ETIME,
    GPS_ERROR_NO_MEM = ENOSR,
} gps_error_t;

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
 * @param layer Layer to write the configuration to, can be more than one layer.
 * @param config
 * @param config_len number of config items, 0 < config_len <= MAX_CONFIG_ITEMS
 * @return gps_error_t
 */
gps_error_t gps_set_config(ubx_cfg_memory_set_del_layer_t layer, gps_config_item_t config[], uint8_t config_len);

/**
 * @brief Get configuration manually based on the GPS documentation.
 * List of configs: section 5.8 of
 * https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf
 *
 * @param layer Layer to read the configuration from. Only one layer can be read at a time.
 * @param keys
 * @param keys_len
 * @return gps_error_t
 */
gps_error_t gps_get_config(ubx_cfg_memory_get_layer_t layer, uint32_t keys[], uint8_t keys_len);

/**
 * @brief Delete configuration manually based on the GPS documentation.
 * List of configs: section 5.8 of
 * https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf
 *
 * @param layer Layer to delete the configuration from, can be more than one layer.
 * @param keys
 * @param keys_len
 * @return gps_error_t
 */
gps_error_t gps_del_config(ubx_cfg_memory_set_del_layer_t layer, uint32_t keys[], uint8_t keys_len);

// SPECIFIC CONFIGURATION

/**
 * @brief Set GPS rate configuration.
 *
 * @param nominal_time_ms Nominal time between GNSS measurements. The minimum value is 25. 0 to left unchanged.
 * @param ratio Ratio of number of measurements to number of navigation solutions. E.g. 5 means five measurements for
 * every navigation solution. The minimum value is 1. The maximum value is 127. 0 to left unchanged.
 * @param time_system Time system to which measurements are aligned. UBX_CFG_TYPES_TIME_SYSTEM_NONE to left unchanged.
 * @param layer Layer to write the configuration to, can be more than one layer.
 * @return gps_error_t
 */
gps_error_t gps_set_cfg_rate(uint16_t nominal_time_ms, uint16_t ratio, ubxCfgTimeSystem_t time_system,
                             ubx_cfg_memory_set_del_layer_t layer);

#endif // GPS_DRIVER_H_

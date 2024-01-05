#ifndef GPS_DRIVER_H_
#define GPS_DRIVER_H_

#include "gps-driver-context.h"
#include "inttypes.h"
#include "ubx-cfg-defines.h"
#include "ubx-data-defines.h"
#include <errno.h>
#include <zephyr/device.h>

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

//============== GENERAL API ==============

/**
 * @brief Send msg to GPS.
 *
 * @param dev
 * @param payload Complete msg to send to GPS (including preamble, msg class, msg id, payload and checksum).
 * @param payload_len
 * @param wait_for_resp If true, wait for response from GPS.
 * @param handler Handler for response from GPS. If NULL, no handler will be called.
 * @param user_data User data passed to handler. Can be NULL.
 * @return gps_error_t
 */
gps_error_t gps_send_generic(const struct device *dev, uint8_t *payload, uint16_t payload_len, uint8_t wait_for_resp,
                             gps_handler_t handler, void *user_data);

//============== CONFIGURATION API ==============

// GENERAL CONFIGURATION

/**
 * @brief Set the configuration manually based on the GPS documentation.
 * Useful for options not supported by this library.
 * List of configs: section 5.8 of
 * https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf
 *
 * @param dev
 * @param layer Layer to write the configuration to, can be more than one layer.
 * @param config
 * @param config_len number of config items, 0 < config_len <= MAX_CONFIG_ITEMS
 * @param wait_for_resp If true, wait for response from GPS.
 * @param handler Handler for response from GPS. If NULL, no handler will be called.
 * @param user_data User data passed to handler. Can be NULL.
 * @return gps_error_t
 */
gps_error_t gps_set_config(const struct device *dev, ubx_cfg_memory_set_del_layer_t layer, gps_config_item_t config[],
                           uint8_t config_len, uint8_t wait_for_resp, gps_handler_t handler, void *user_data);

/**
 * @brief Get configuration manually based on the GPS documentation.
 * List of configs: section 5.8 of
 * https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf
 *
 * @param dev
 * @param layer Layer to read the configuration from. Only one layer can be read at a time.
 * @param keys
 * @param keys_len
 * @param wait_for_resp If true, wait for response from GPS.
 * @param handler Handler for response from GPS. If NULL, no handler will be called.
 * @param user_data User data passed to handler. Can be NULL.
 * @return gps_error_t
 */
gps_error_t gps_get_config(const struct device *dev, ubx_cfg_memory_get_layer_t layer, uint32_t keys[],
                           uint8_t keys_len, uint8_t wait_for_resp, gps_handler_t handler, void *user_data);

/**
 * @brief Delete configuration manually based on the GPS documentation.
 * List of configs: section 5.8 of
 * https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf
 *
 * @param dev
 * @param layer Layer to delete the configuration from, can be more than one layer.
 * @param keys
 * @param keys_len
 * @param wait_for_resp If true, wait for response from GPS.
 * @param handler Handler for response from GPS. If NULL, no handler will be called.
 * @param user_data User data passed to handler. Can be NULL.
 * @return gps_error_t
 */
gps_error_t gps_del_config(const struct device *dev, ubx_cfg_memory_set_del_layer_t layer, uint32_t keys[],
                           uint8_t keys_len, uint8_t wait_for_resp, gps_handler_t handler, void *user_data);

// SPECIFIC CONFIGURATION

/**
 * @brief Set GPS rate configuration.
 *
 * @param dev
 * @param nominal_time_ms Nominal time between GNSS measurements. The minimum value is 25. 0 to left unchanged.
 * @param ratio Ratio of number of measurements to number of navigation solutions. E.g. 5 means five measurements for
 * every navigation solution. The minimum value is 1. The maximum value is 127. 0 to left unchanged.
 * @param time_system Time system to which measurements are aligned. UBX_CFG_TYPES_TIME_SYSTEM_NONE to left unchanged.
 * @param layer Layer to write the configuration to, can be more than one layer.
 * @param wait_for_resp If true, wait for response from GPS.
 * @param handler Handler for response from GPS. If NULL, no handler will be called.
 * @param user_data User data passed to handler. Can be NULL.
 * @return gps_error_t
 */
gps_error_t gps_set_cfg_rate(const struct device *dev, uint16_t nominal_time_ms, uint16_t ratio,
                             ubxCfgTimeSystem_t time_system, ubx_cfg_memory_set_del_layer_t layer,
                             uint8_t wait_for_resp, gps_handler_t handler, void *user_data);

/**
 * @brief Disable all NMEA output messages (GGA, GLL, GSA, GSV, RMC, VTG).
 *
 * @param dev
 * @param layer Layer to write the configuration to, can be more than one layer.
 * @param wait_for_resp If true, wait for response from GPS.
 * @param handler Handler for response from GPS. If NULL, no handler will be called.
 * @param user_data User data passed to handler. Can be NULL.
 * @return gps_error_t
 */
gps_error_t gps_set_cfg_disable_all_nmea(const struct device *dev, ubx_cfg_memory_set_del_layer_t layer,
                                         uint8_t wait_for_resp, gps_handler_t handler, void *user_data);

/**
 * @brief Enable UBX-NAV-PVT messages
 *
 * @param dev
 * @param layer Layer to write the configuration to, can be more than one layer.
 * @param wait_for_resp If true, wait for response from GPS.
 * @param handler Handler for response from GPS. If NULL, no handler will be called.
 * @param user_data User data passed to handler. Can be NULL.
 * @return gps_error_t
 */
gps_error_t gps_set_enable_ubx_nav_pvt(const struct device *dev, ubx_cfg_memory_set_del_layer_t layer,
                                       uint8_t wait_for_resp, gps_handler_t handler, void *user_data);

/**
 * @brief Disable UBX-NAV-PVT messages
 *
 * @param dev
 * @param layer Layer to write the configuration to, can be more than one layer.
 * @param wait_for_resp If true, wait for response from GPS.
 * @param handler Handler for response from GPS. If NULL, no handler will be called.
 * @param user_data User data passed to handler. Can be NULL.
 * @return gps_error_t
 */
gps_error_t gps_set_disable_ubx_nav_pvt(const struct device *dev, ubx_cfg_memory_set_del_layer_t layer,
                                        uint8_t wait_for_resp, gps_handler_t handler, void *user_data);

//============== HANDLERS ==============

/**
 * @brief Register callback for specific UBX message.
 *
 * @param dev
 * @param msg_class Message class (e.g. UBX_NAV_CLASS) for which callback will be registered.
 * @param msg_id Message id (e.g. UBX_NAV_PVT_ID) for which callback will be registered.
 * @param handler Function to be called when message is received.
 * @param user_data (optional) Data passed to handler. Can be NULL. User is responsible for allocating and freeing this
 * @param one_time_flag True if callback should be automatically unregistered after first call.
 * @return gps_error_t GPS_OK if callback was registered, GPS_ERROR_NO_MEM if callback list is full,
 * GPS_ERROR_WRONG_PARAM, GPS_ERROR
 */
gps_error_t gps_register_callback(const struct device *dev, uint8_t msg_class, uint8_t msg_id, gps_handler_t handler,
                                  void *user_data, uint8_t one_time_flag);

/**
 * @brief Unregister previously registered callback. If callback was not registered, nothing happens. If more than one
 * callback is registered for given class and id pair, only one (first occurence) will be unregistered.
 *
 * @param dev
 * @param msg_class Message class (e.g. UBX_NAV_CLASS) for which callback will be unregistered.
 * @param msg_id Message id (e.g. UBX_NAV_PVT_ID) for which callback will be unregistered.
 * @return gps_error_t
 */
gps_error_t gps_unregister_callback(const struct device *dev, uint8_t msg_class, uint8_t msg_id);

/**
 * @brief List all registered callbacks for given device. Class and id will be LOG_INFed for each registered callback.
 *
 * @param dev
 */
void gps_list_callbacks(const struct device *dev);

#endif // GPS_DRIVER_H_

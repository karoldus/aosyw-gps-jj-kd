#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "../drivers/gps-driver/gps-driver.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(main);

const struct device *neo_m9n_gps = DEVICE_DT_GET(DT_ALIAS(neo_m9n));

void disable_navpvt_handler_test(const struct device *dev, uint8_t gps_code, uint8_t class, uint8_t id, ubx_msg_t *msg,
                                 void *user_data)
{
    LOG_INF("disable_navpvt_handler_test: %d, %x, %x", gps_code, class, id);

    if (msg != NULL)
    {
        if (msg->payload != NULL)
        {
            free(msg->payload);
            msg->payload = NULL;
        }
    }
}

void enable_navpvt_handler_test(const struct device *dev, uint8_t gps_code, uint8_t class, uint8_t id, ubx_msg_t *msg,
                                void *user_data)
{
    LOG_INF("enable_navpvt_handler_test: %d, %x, %x", gps_code, class, id);

    if (msg != NULL)
    {
        if (msg->payload != NULL)
        {
            free(msg->payload);
            msg->payload = NULL;
        }
    }
}

void nav_pvt_handler(const struct device *dev, uint8_t gps_code, uint8_t class, uint8_t id, ubx_msg_t *msg,
                     void *user_data)
{
    // LOG_INF("nav_pvt_handler: %d, %x, %x", gps_code, class, id);

    ubx_nav_pvt_t *pvt_msg = (ubx_nav_pvt_t *)msg->payload;

    LOG_INF("GPS time: %02d:%02d:%02d.%03d UTC", pvt_msg->hour, pvt_msg->min, pvt_msg->sec, pvt_msg->nano / 1000000);
    LOG_INF("GPS position: %d.%d, %d.%d", pvt_msg->lat / 10000000, pvt_msg->lat % 10000000, pvt_msg->lon / 10000000,
            pvt_msg->lon % 10000000);

    if (msg != NULL)
    {
        if (msg->payload != NULL)
        {
            free(msg->payload);
            msg->payload = NULL;
        }
    }
}

int main(void)
{
    k_sleep(K_MSEC(1000));

    gps_set_disable_ubx_nav_pvt(neo_m9n_gps, GPS_MEMORY_GET_LAYER_BBR, 1, disable_navpvt_handler_test, NULL);

    k_sleep(K_MSEC(5000));

    gps_register_callback(neo_m9n_gps, UBX_NAV_CLASS, UBX_NAV_PVT_ID, nav_pvt_handler, NULL, 0);
    gps_set_enable_ubx_nav_pvt(neo_m9n_gps, GPS_MEMORY_GET_LAYER_BBR, 1, enable_navpvt_handler_test, NULL);

    gps_set_cfg_rate(neo_m9n_gps, 40, 1, UBX_CFG_TYPES_TIME_SYSTEM_NONE, GPS_MEMORY_GET_LAYER_BBR, 1, NULL, NULL);

    k_sleep(K_MSEC(2000));

    gps_set_cfg_rate(neo_m9n_gps, 1000, 1, UBX_CFG_TYPES_TIME_SYSTEM_NONE, GPS_MEMORY_GET_LAYER_BBR, 1, NULL, NULL);

    k_sleep(K_FOREVER);

    return 0;
}

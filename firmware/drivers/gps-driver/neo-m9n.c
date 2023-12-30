#include "neo-m9n.h"
#include "testowy.h"

LOG_MODULE_REGISTER(M9N);

static struct neo_m9n_api api;

static inline int neo_m9n_bus_write(const struct device *dev, uint8_t *payload, uint16_t payload_len)
{
    const struct neo_m9n_config *cfg = dev->config;

    return cfg->bus_io->write(&cfg->bus, payload, payload_len);
}

static inline int neo_m9n_bus_read(const struct device *dev, uint8_t *payload, k_timeout_t timeout)
{
    const struct neo_m9n_config *cfg = dev->config;

    return cfg->bus_io->read(dev, payload, timeout);
}

int neo_m9n_get_data(const struct device *dev, uint8_t *payload)
{
    return neo_m9n_bus_read(dev, payload, K_FOREVER);
}

static inline int neo_m9n_set_odr(const struct device *dev, neo_m9n_odr_t odr)
{
    int ret;

    switch (odr)
    {
    case NEO_M9N_ODR_1HZ:
        ret = neo_m9n_bus_write(dev, gps_config_1Hz, sizeof(gps_config_1Hz));
        break;

    case NEO_M9N_ODR_10HZ:
        ret = -ENOTSUP;
        break;

    case NEO_M9N_ODR_25HZ:
        ret = neo_m9n_bus_write(dev, gps_config_25Hz, sizeof(gps_config_25Hz));
        break;

    default:
        ret = -ENOTSUP;
        break;
    }

    return ret;
}

static inline int neo_m9n_set_pvt(const struct device *dev)
{
    int ret;

    ret = neo_m9n_bus_write(dev, gps_config_default, sizeof(gps_config_default));
    if (ret)
    {
        goto error;
    }

    k_sleep(K_MSEC(100));

    ret = neo_m9n_bus_write(dev, gps_config_GLL_off, sizeof(gps_config_GLL_off));
    if (ret)
    {
        goto error;
    }

    k_sleep(K_MSEC(100));

    ret = neo_m9n_bus_write(dev, gps_config_GSA_off, sizeof(gps_config_GSA_off));
    if (ret)
    {
        goto error;
    }

    k_sleep(K_MSEC(100));

    ret = neo_m9n_bus_write(dev, gps_config_GSV_off, sizeof(gps_config_GSV_off));
    if (ret)
    {
        goto error;
    }

    k_sleep(K_MSEC(100));

    ret = neo_m9n_bus_write(dev, gps_config_GGA_off, sizeof(gps_config_GGA_off));
    if (ret)
    {
        goto error;
    }

    k_sleep(K_MSEC(100));

    ret = neo_m9n_bus_write(dev, gps_config_VTG_off, sizeof(gps_config_VTG_off));
    if (ret)
    {
        goto error;
    }

    k_sleep(K_MSEC(100));

    ret = neo_m9n_bus_write(dev, gps_config_RMC_off, sizeof(gps_config_RMC_off));
    if (ret)
    {
        goto error;
    }

    k_sleep(K_MSEC(100));

    ret = neo_m9n_bus_write(dev, gps_config_UART1_NAV_PVT_on, sizeof(gps_config_UART1_NAV_PVT_on));
    if (ret)
    {
        goto error;
    }

error:
    return ret;
}

#ifdef CONFIG_PM_DEVICE
static int pm_action(const struct device *dev, enum pm_device_action action)
{
    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:
        /* suspend the device */
        break;
    case PM_DEVICE_ACTION_RESUME:
        /* resume the device */
        break;
    case PM_DEVICE_ACTION_TURN_ON:
        /*
         * powered on the device, used when the power
         * domain this device belongs is resumed.
         */
        break;
    case PM_DEVICE_ACTION_TURN_OFF:
        /*
         * power off the device, used when the power
         * domain this device belongs is suspended.
         */
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

static int neo_m9n_init(const struct device *dev)
{
    int ret;

    struct neo_m9n_data *dev_data = dev->data;

    const struct neo_m9n_config *dev_config = dev->config;

    LOG_INF("Initializing u-blox NEO M9N");

    k_sem_init(&dev_data->tx_sem, 0, 1);

    ret = dev_config->bus_io->init(dev);
    if (ret)
    {
        LOG_ERR("Error while initializing bus");
        goto error;
    }

    ret = neo_m9n_set_pvt(dev);
    if (ret)
    {
        LOG_ERR("Error while setting PVT %d", ret);
        goto error;
    }

    k_sleep(K_MSEC(100));

    ret = neo_m9n_set_odr(dev, dev_data->odr);
    if (ret)
    {
        LOG_ERR("Error while setting odr %d", ret);
        goto error;
    }

    LOG_INF("Initializing u-blox NEO M9N completed");
error:
    return ret;
}

#define NEO_M9N_CONFIG_UART(inst) .bus.uart = DEVICE_DT_GET(DT_INST_BUS(inst)), .bus_io = &neo_m9n_bus_io_uart,

#define NEO_M9N_CONFIG_I2C(inst) .bus.i2c = I2C_DT_SPEC_INST_GET(inst),

#define NEO_M9N_BUS_CFG(inst)                                                                                          \
    COND_CODE_1(DT_INST_ON_BUS(inst, i2c), (NEO_M9N_CONFIG_I2C(inst)), (NEO_M9N_CONFIG_UART(inst)))

#define NEO_M9N_DEFINE(inst)                                                                                           \
    static struct neo_m9n_data neo_m9n_data_##inst = {.odr = DT_INST_PROP(inst, output_rate)};                         \
    static const struct neo_m9n_config neo_m9n_config_##inst = {NEO_M9N_BUS_CFG(inst)};                                \
                                                                                                                       \
    PM_DEVICE_DT_INST_DEFINE(inst, pm_action);                                                                         \
                                                                                                                       \
    DEVICE_DT_INST_DEFINE(inst, neo_m9n_init, NULL, &neo_m9n_data_##inst, &neo_m9n_config_##inst, POST_KERNEL,         \
                          CONFIG_NEO_M9N_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(NEO_M9N_DEFINE)

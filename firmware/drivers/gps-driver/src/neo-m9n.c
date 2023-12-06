#include "../include/neo-m9n.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(M9N);

static struct neo_m9n_api api;

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

    LOG_INF("Initializing u-blox NEO M9N");

    return 0;
}

#define NEO_M9N_CONFIG_UART(inst) .uart = DEVICE_DT_GET(DT_INST_BUS(inst)),

#define NEO_M9N_CONFIG_I2C(inst) .i2c = I2C_DT_SPEC_INST_GET(inst),

#define NEO_M9N_BUS_CFG(inst)                                                                                          \
    COND_CODE_1(DT_INST_ON_BUS(inst, i2c), (NEO_M9N_CONFIG_I2C(inst)), (NEO_M9N_CONFIG_UART(inst)))

void print_dbg(void)
{
    LOG_INF("%d, %d", NEO_M9N_BUS_UART, NEO_M9N_BUS_I2C);
}

#define NEO_M9N_DEFINE(inst)                                                                                           \
    static struct neo_m9n_data neo_m9n_data_##inst;                                                                    \
    static const struct neo_m9n_config neo_m9n_config_##inst = {NEO_M9N_BUS_CFG(inst)};                                \
                                                                                                                       \
    PM_DEVICE_DT_INST_DEFINE(inst, pm_action)                                                                          \
                                                                                                                       \
    DEVICE_DT_INST_DEFINE(inst, neo_m9n_init, NULL, &neo_m9n_data_##inst, &neo_m9n_config_##inst, POST_KERNEL,         \
                          CONFIG_NEO_M9N_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(NEO_M9N_DEFINE)
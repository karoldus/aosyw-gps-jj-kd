#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>

#define DT_DRV_COMPAT u_blox_m9n

#define CONFIG_NEO_M9N_INIT_PRIORITY 90

#define NEO_M9N_BUS_UART DT_ANY_INST_ON_BUS_STATUS_OKAY(uart)
#define NEO_M9N_BUS_I2C  DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

struct neo_m9n_bus
{
#if NEO_M9N_BUS_UART
    const struct device *uart;
#endif
#if NEO_M9N_BUS_I2C
    struct i2c_dt_spec i2c;
#endif
};

struct neo_m9n_data
{
    uint8_t test_data;
};

struct neo_m9n_config
{
    const struct device *uart;
};

typedef struct neo_m9n_api
{
};

void print_dbg(void);
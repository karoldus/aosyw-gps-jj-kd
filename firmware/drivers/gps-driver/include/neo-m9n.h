#ifndef __NEO_M9N_H__
#define __NEO_M9N_H__

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#define DT_DRV_COMPAT u_blox_m9n

#define CONFIG_NEO_M9N_INIT_PRIORITY 90

#define NEO_M9N_BUS_UART DT_ANY_INST_ON_BUS_STATUS_OKAY(uart)
#define NEO_M9N_BUS_I2C  DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

// #undef DT_DRV_COMPAT

union neo_m9n_bus
{
#if NEO_M9N_BUS_UART
    struct device *uart;
#endif
#if NEO_M9N_BUS_I2C
    struct i2c_dt_spec i2c;
#endif
};

typedef int (*neo_m9n_bus_init_fn)(const union neo_m9n_bus *bus);
typedef int (*neo_m9n_write_fn)(const union neo_m9n_bus *bus, uint8_t *payload, uint16_t payload_len);

struct neo_m9n_bus_io
{
    neo_m9n_bus_init_fn init;
    neo_m9n_write_fn write;
};

#if NEO_M9N_BUS_UART
extern const struct neo_m9n_bus_io neo_m9n_bus_io_uart;
#endif

#if NEO_M9N_BUS_I2C
extern const struct neo_m9n_bus_io neo_m9n_bus_io_i2c;
#endif

struct neo_m9n_data
{
    uint8_t odr;
};

struct neo_m9n_config
{
    union neo_m9n_bus bus;
    const struct neo_m9n_bus_io *bus_io;
};

typedef enum
{
    NEO_M9N_ODR_1HZ = 1,
    NEO_M9N_ODR_10HZ = 10,
    NEO_M9N_ODR_25HZ = 25
} neo_m9n_odr_t;

void test_api_func(void);

typedef struct neo_m9n_api
{
};

void print_dbg(void);

#endif /* __NEO_M9N_H__ */
#ifndef __NEO_M9N_H__
#define __NEO_M9N_H__

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/sem.h>

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

typedef int (*neo_m9n_bus_init_fn)(const struct device *dev);
typedef int (*neo_m9n_write_fn)(const union neo_m9n_bus *bus, uint8_t *payload, uint16_t payload_len);
typedef int (*neo_m9n_read_fn)(const struct device *dev, uint8_t *payload, k_timeout_t timeout);

struct neo_m9n_bus_io
{
    neo_m9n_bus_init_fn init;
    neo_m9n_write_fn write;
    neo_m9n_read_fn read;
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
    struct k_msgq msgq;
    char msgq_buffer[5 * 1000];
    uint8_t rx_buff[1000];

    struct k_sem tx_sem;
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

typedef struct neo_m9n_api
{
};

int neo_m9n_get_data(const struct device *dev, uint8_t *payload);

#endif /* __NEO_M9N_H__ */
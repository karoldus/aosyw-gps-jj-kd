#include "../include/neo-m9n.h"

#if NEO_M9N_BUS_UART

LOG_MODULE_REGISTER(M9N_UART);

#define GPS_UART_BAUDRATE   38400
#define GPS_UART_BUF_SIZE   100
#define GPS_UART_RX_TIMEOUT 5000

const struct uart_config uart_cfg2 = {.baudrate = GPS_UART_BAUDRATE,
                                      .parity = UART_CFG_PARITY_NONE,
                                      .stop_bits = UART_CFG_STOP_BITS_1,
                                      .data_bits = UART_CFG_DATA_BITS_8,
                                      .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

static uint8_t rx_buff[GPS_UART_BUF_SIZE] = {0};

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev);

    switch (evt->type)
    {
    case UART_TX_DONE:
        // do something
        break;

    case UART_TX_ABORTED:
        // do something
        break;

    case UART_RX_RDY:
        LOG_INF("GPS");
        break;

    case UART_RX_BUF_REQUEST:
        // do something
        break;

    case UART_RX_BUF_RELEASED:
        // do something
        break;

    case UART_RX_DISABLED:
        uart_rx_enable(dev, rx_buff, sizeof(rx_buff), GPS_UART_RX_TIMEOUT);
        break;

    case UART_RX_STOPPED:
        // do something
        break;

    default:
        break;
    }
}

static int neo_m9n_uart_init(const union neo_m9n_bus *bus)
{
    int ret;

    if (!device_is_ready(bus->uart))
    {
        ret = -ENODEV;
        LOG_ERR("UART bus not ready");
        goto error;
    }

    ret = uart_configure(bus->uart, &uart_cfg2);
    if (ret)
    {
        LOG_ERR("Error while configuring UART bus");
        goto error;
    }

    ret = uart_callback_set(bus->uart, uart_cb, rx_buff);
    if (ret)
    {
        LOG_ERR("Error while setting UART callback");
        goto error;
    }

error:
    return ret;
}

static int neo_m9n_write_uart(const union neo_m9n_bus *bus, uint8_t *payload, uint16_t payload_len)
{
    return uart_tx(bus->uart, payload, payload_len, SYS_FOREVER_US);
}

const struct neo_m9n_bus_io neo_m9n_bus_io_uart = {
    .init = neo_m9n_uart_init,
    .write = neo_m9n_write_uart,
};

#endif /* NEO_M9N_BUS_UART */
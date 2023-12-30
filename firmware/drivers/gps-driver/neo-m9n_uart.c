#include "neo-m9n.h"

#if NEO_M9N_BUS_UART

LOG_MODULE_REGISTER(M9N_UART);

#define GPS_UART_BUF_SIZE   1000
#define GPS_UART_RX_TIMEOUT 5000

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev);

    const struct device *device = user_data;

    struct neo_m9n_data *dev_data = device->data;

    int ret;

    switch (evt->type)
    {
    case UART_TX_DONE:
        k_sem_give(&dev_data->tx_sem);
        break;

    case UART_RX_RDY:
        LOG_INF("off: %d, len: %d ", evt->data.rx.offset, evt->data.rx.len);
        k_msgq_put(&dev_data->msgq, evt->data.rx.buf + evt->data.rx.offset, K_NO_WAIT);
        break;

    case UART_RX_BUF_REQUEST:
        uart_rx_buf_rsp(dev, dev_data->rx_buff, GPS_UART_BUF_SIZE);
        break;

    case UART_RX_DISABLED:
        ret = uart_rx_enable(dev, dev_data->rx_buff, GPS_UART_BUF_SIZE, GPS_UART_RX_TIMEOUT);
        if (ret)
        {
            LOG_ERR("Failed to re-enable UART");
        }

        break;

    case UART_TX_ABORTED:
        // currently not used
        break;

    case UART_RX_BUF_RELEASED:
        // currently not used
        break;

    case UART_RX_STOPPED:
        // currently not used
        break;

    default:
        break;
    }
}

static int neo_m9n_uart_init(const struct device *dev)
{
    int ret;

    struct neo_m9n_data *dev_data = dev->data;

    const struct neo_m9n_config *dev_config = dev->config;

    union neo_m9n_bus *bus = &dev_config->bus;

    if (!device_is_ready(bus->uart))
    {
        ret = -ENODEV;
        LOG_ERR("UART bus not ready");
        goto error;
    }

    k_msgq_init(&dev_data->msgq, dev_data->msgq_buffer, GPS_UART_BUF_SIZE, 5);

    ret = uart_callback_set(bus->uart, uart_cb, dev);
    if (ret)
    {
        LOG_ERR("Error while setting UART callback");
        goto error;
    }
    k_sleep(K_MSEC(100));

    ret = uart_rx_enable(bus->uart, dev_data->rx_buff, GPS_UART_BUF_SIZE, GPS_UART_RX_TIMEOUT);
    if (ret < 0)
    {
        LOG_ERR("Failed to enable UART RX");
    }

error:
    return ret;
}

static int neo_m9n_read_uart(const struct device *dev, uint8_t *payload, k_timeout_t timeout)
{
    struct neo_m9n_data *dev_data = dev->data;

    int ret = k_msgq_get(&dev_data->msgq, payload, timeout);

    return 0;
}

static int neo_m9n_write_uart(const union neo_m9n_bus *bus, uint8_t *payload, uint16_t payload_len)
{
    return uart_tx(bus->uart, payload, payload_len, SYS_FOREVER_US);
}

const struct neo_m9n_bus_io neo_m9n_bus_io_uart = {
    .init = neo_m9n_uart_init,
    .write = neo_m9n_write_uart,
    .read = neo_m9n_read_uart,
};

#endif /* NEO_M9N_BUS_UART */
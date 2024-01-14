#include "neo-m9n.h"

#if NEO_M9N_BUS_UART

LOG_MODULE_REGISTER(M9N_UART);

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev);

    const struct device *device = user_data;

    struct neo_m9n_data *dev_data = device->data;

    int ret;
    uint32_t written;

    switch (evt->type)
    {
    case UART_TX_DONE:
        k_sem_give(&dev_data->tx_sem);
        break;

    case UART_RX_RDY:
        // LOG_INF("off: %d, len: %d ", evt->data.rx.offset, evt->data.rx.len);

        written = ring_buf_put(&dev_data->rx_rb, evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
        if (written != evt->data.rx.len)
        {
            LOG_WRN("Received bytes dropped from ring buf");
        }

        k_sem_give(&dev_data->rx_sem);
        break;

    case UART_RX_BUF_REQUEST:
        uart_rx_buf_rsp(dev, dev_data->rx_buff, NEO_M9N_RX_BUF_SIZE);
        break;

    case UART_RX_DISABLED:
        ret = uart_rx_enable(dev, dev_data->rx_buff, NEO_M9N_RX_BUF_SIZE, NEO_M9N_UART_ASYNC_RX_TIMEOUT_US);
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

    ret = uart_callback_set(bus->uart, uart_cb, dev);
    if (ret)
    {
        LOG_ERR("Error while setting UART callback");
        goto error;
    }

    ret = uart_rx_enable(bus->uart, dev_data->rx_buff, NEO_M9N_RX_BUF_SIZE, NEO_M9N_UART_ASYNC_RX_TIMEOUT_US);
    if (ret < 0)
    {
        LOG_ERR("Failed to enable UART RX");
    }

error:
    return ret;
}

static int neo_m9n_read_uart(const struct device *dev, uint8_t *buf, size_t size, size_t *bytes_read)
{
    struct neo_m9n_data *dev_data = dev->data;

    struct modem_iface_uart_data *data;

    if (!dev || !dev_data)
    {
        return -EINVAL;
    }

    if (size == 0)
    {
        *bytes_read = 0;
        return 0;
    }

    *bytes_read = ring_buf_get(&dev_data->rx_rb, buf, size);

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
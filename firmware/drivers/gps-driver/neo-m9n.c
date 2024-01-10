#include "neo-m9n.h"
#include "testowy.h"
#include <zephyr/kernel.h>

#include "gps-rx.h"

LOG_MODULE_REGISTER(M9N);

/**
 * @brief Run wait_for_resp handler and clean it. Used only for timeout and error sending.
 *
 * @param dev
 * @param gps_handler_code
 * @param msg
 */
static void run_and_clean_wait_for_resp_handler(struct device *dev, uint8_t gps_handler_code, ubx_msg_t *msg)
{
    struct neo_m9n_data *dev_data = dev->data;

    k_sem_take(&(dev_data->callbacks_sem), K_FOREVER);

    uint8_t class = dev_data->wait_for_resp_callback.class;
    uint8_t id = dev_data->wait_for_resp_callback.id;

    dev_data->wait_for_resp_callback.class = 0;
    dev_data->wait_for_resp_callback.id = 0;

    if (dev_data->wait_for_resp_callback.handler != NULL)
    {
        dev_data->wait_for_resp_callback.handler(dev, gps_handler_code, class, id, msg,
                                                 dev_data->wait_for_resp_callback.user_data);
    }
    else
    {
        LOG_INF("No handler for response"); // TODO: remove
    }

    dev_data->wait_for_resp_callback.handler = NULL;
    dev_data->wait_for_resp_callback.user_data = NULL;

    k_sem_give(&(dev_data->tx_wait_for_resp_sem));
    k_sem_give(&(dev_data->callbacks_sem));

    return;
}

void tx_thread(struct device *dev, void *p2, void *p3)
{
    LOG_INF("TX thread started");

    struct neo_m9n_data *dev_data = dev->data;
    gps_driver_tx_queue_element_t tx_queue_element;

    while (1)
    {
        k_msgq_get(&(dev_data->tx_queue), &tx_queue_element, K_FOREVER);

        uint8_t class = 0;
        uint8_t id = 0;

        if (tx_queue_element.gps_payload_len >= 4)
        {
            class = tx_queue_element.gps_payload[2];
            id = tx_queue_element.gps_payload[3];
        }

        LOG_ERR("TEST - class 0x%X, id 0x%X", class, id);

        if (k_sem_take(&(dev_data->tx_wait_for_resp_sem), K_MSEC(GPS_SEND_MSG_MUTEX_TIMEOUT_MS)) != 0)
        {
            LOG_ERR("Previous msg not responded - timeout");
            run_and_clean_wait_for_resp_handler(dev, GPS_HANDLER_CODES_TIMEOUT, NULL);
        }

        k_sem_take(&(dev_data->callbacks_sem), K_FOREVER);

        if (tx_queue_element.wait_for_response)
        {
            dev_data->wait_for_resp_callback.handler = tx_queue_element.response_handler;
            dev_data->wait_for_resp_callback.user_data = tx_queue_element.handler_user_data;
            dev_data->wait_for_resp_callback.class = class;
            dev_data->wait_for_resp_callback.id = id;
        }
        else
        {
            dev_data->wait_for_resp_callback.handler = NULL;
            dev_data->wait_for_resp_callback.user_data = NULL;
            dev_data->wait_for_resp_callback.class = 0;
            dev_data->wait_for_resp_callback.id = 0;
        }

        LOG_ERR("TEST2 - class 0x%X, id 0x%X", dev_data->wait_for_resp_callback.class,
                dev_data->wait_for_resp_callback.id);

        k_sem_give(&(dev_data->callbacks_sem));

        int ret = neo_m9n_bus_write(dev, tx_queue_element.gps_payload, tx_queue_element.gps_payload_len);

        if (ret != 0)
        {
            LOG_ERR("Error while sending msg to GPS");
            run_and_clean_wait_for_resp_handler(dev, GPS_HANDLER_CODES_ERROR_SENDING, NULL);
        }

        if (tx_queue_element.wait_for_response == 1)
        {
            // RX thread will give this semaphore when response is received
            if (k_sem_take(&(dev_data->tx_wait_for_resp_sem), K_MSEC(GPS_SEND_MSG_MUTEX_TIMEOUT_MS)) != 0)
            {
                // RX thread didn't give semaphore in time
                LOG_ERR("Current msg not responded - timeout");

                run_and_clean_wait_for_resp_handler(dev, GPS_HANDLER_CODES_TIMEOUT, NULL);
            }
        }

        k_sem_give(&(dev_data->tx_wait_for_resp_sem));
    }
}

void rx_thread(struct device *dev, void *p2, void *p3)
{
    while (1)
    {
        LOG_INF("RX");
        k_sleep(K_MSEC(1000));

        if (0)
        {
            gps_handle_raw_rx(dev, NULL, 0); // TODO logic and test this function
        }
    }
}

static struct neo_m9n_api api;

int neo_m9n_bus_write(const struct device *dev, uint8_t *payload, uint16_t payload_len)
{
    const struct neo_m9n_config *cfg = dev->config;

    return cfg->bus_io->write(&cfg->bus, payload, payload_len);
}

static inline int neo_m9n_bus_read(const struct device *dev, uint8_t *buf, size_t size, size_t *bytes_read)
{
    const struct neo_m9n_config *cfg = dev->config;

    return cfg->bus_io->read(dev, buf, size, bytes_read);
}

int neo_m9n_get_data(const struct device *dev, uint8_t *buf, size_t size, size_t *bytes_read, k_timeout_t timeout)
{
    int ret;
    struct neo_m9n_data *dev_data = dev->data;

    ret = k_sem_take(&dev_data->rx_sem, timeout);

    if (ret)
    {
        return ret;
    }

    return neo_m9n_bus_read(dev, buf, size, bytes_read);
}

int neo_m9n_get_packet(const struct device *dev, uint8_t *buf, size_t size, size_t *bytes_read, k_timeout_t timeout)
{
    int ret;
    uint8_t temp_buf[size];
    size_t *temp_bytes_read;
    ret = neo_m9n_get_data(dev, temp_buf, size, temp_bytes_read, timeout);

    if (ret)
    {
        goto error;
    }

    memcpy(buf, temp_buf, *temp_bytes_read);
    *bytes_read = *temp_bytes_read;

    if (*temp_bytes_read < 6)
    {
        ret = neo_m9n_get_data(dev, temp_buf, size, temp_bytes_read, timeout);
        if (ret)
        {
            goto error;
        }
        if(size - *bytes_read < *temp_bytes_read)
        {
            ret = -1;
            goto error;
        }

        memcpy(buf[*bytes_read], temp_buf, *temp_bytes_read);
        *bytes_read += *temp_bytes_read;
    }

    if(buf[0] != 0xb5 || buf[1] != 0x62)
    {
        ret = -1;
        goto error;
    }

    uint16_t temp_len;
    temp_len = buf[4] + (buf[5] << 8);

    if((temp_len + 8) > size)
    {
        ret = -1;
        goto error;
    }

    while((temp_len + 8) < *bytes_read)
    {
        ret = neo_m9n_get_data(dev, temp_buf, size, temp_bytes_read, timeout);
        if (ret)
        {
            goto error;
        }
        memcpy(buf[*bytes_read], temp_buf, *temp_bytes_read);
        *bytes_read += *temp_bytes_read;
    }

error:
    return ret;
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
    k_sem_init(&dev_data->rx_sem, 0, 1);

    ring_buf_init(&dev_data->rx_rb, sizeof(dev_data->iface_rb_buf), dev_data->iface_rb_buf);

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

    ret = k_sem_init(&(dev_data->tx_wait_for_resp_sem), 1, 1);
    if (ret)
    {
        LOG_ERR("Error while initializing semaphore %d", ret);
        goto error;
    }

    k_thread_stack_t *tx_thread_stack_area = k_malloc(GPS_DRIVER_TX_THREAD_STACK_SIZE);
    if (tx_thread_stack_area == NULL)
    {
        LOG_ERR("Error while allocating tx thread stack");
        goto error;
    }

    k_thread_stack_t *rx_thread_stack_area = k_malloc(GPS_DRIVER_RX_THREAD_STACK_SIZE);
    if (rx_thread_stack_area == NULL)
    {
        LOG_ERR("Error while allocating rx thread stack");
        goto error;
    }

    k_tid_t tx_thread_pid =
        k_thread_create(&(dev_data->tx_thread_data), tx_thread_stack_area, GPS_DRIVER_TX_THREAD_STACK_SIZE, tx_thread,
                        dev, (void *)34, NULL, GPS_DRIVER_TX_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);

    k_tid_t rx_thread_pid =
        k_thread_create(&(dev_data->rx_thread_data), rx_thread_stack_area, GPS_DRIVER_RX_THREAD_STACK_SIZE, rx_thread,
                        dev, NULL, NULL, GPS_DRIVER_RX_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);

    if (tx_thread_pid == NULL || rx_thread_pid == NULL)
    {
        LOG_ERR("Error while creating threads");
        goto error;
    }

    dev_data->rx_thread_stack = rx_thread_stack_area;
    dev_data->tx_thread_stack = tx_thread_stack_area;

    k_msgq_init(&(dev_data->tx_queue), dev_data->tx_queue_data, sizeof(gps_driver_tx_queue_element_t),
                GPS_DRIVER_TX_QUEUE_SIZE);

    ret = k_sem_init(&(dev_data->callbacks_sem), 1, 1);
    if (ret)
    {
        LOG_ERR("Error while initializing semaphore %d", ret);
        goto error;
    }

    LOG_INF("Initializing u-blox NEO M9N completed");

error:
    if (tx_thread_stack_area != NULL)
    {
        k_free(tx_thread_stack_area);
    }
    if (rx_thread_stack_area != NULL)
    {
        k_free(rx_thread_stack_area);
    }
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
                          NEO_M9N_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(NEO_M9N_DEFINE)

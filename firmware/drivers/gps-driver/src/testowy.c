#include "../include/testowy.h"
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define GPS_UART_BAUDRATE   38400
#define GPS_UART_RX_TIMEOUT 5000

LOG_MODULE_REGISTER(GPS_test);

static int gps_data_rdy = 0;

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

static uint8_t rx_buff[GPS_UART_BUF_SIZE] = {0};

GPSData_t gpsData;

K_MSGQ_DEFINE(msgq, GPS_UART_BUF_SIZE, 5, 4);

const struct uart_config uart_cfg = {.baudrate = GPS_UART_BAUDRATE,
                                     .parity = UART_CFG_PARITY_NONE,
                                     .stop_bits = UART_CFG_STOP_BITS_1,
                                     .data_bits = UART_CFG_DATA_BITS_8,
                                     .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

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
        // memcpy(rx_buff_callback, evt->data.rx.buf, evt->data.rx.len);
        k_msgq_put(&msgq, rx_buff + evt->data.rx.offset, K_NO_WAIT);
        // memset(rx_buff, 0, GPS_UART_BUF_SIZE);
        gps_data_rdy = 1;
        break;

    case UART_RX_BUF_REQUEST:
        // do something
        break;

    case UART_RX_BUF_RELEASED:
        // do something
        break;

    case UART_RX_DISABLED:
        uart_rx_enable(uart, rx_buff, sizeof(rx_buff), GPS_UART_RX_TIMEOUT);
        break;

    case UART_RX_STOPPED:
        // do something
        break;

    default:
        break;
    }
}

void gps_init_test()
{
    int ret = 0;
    memset(gpsData.rx_buff_gpsGet, 0, sizeof(gpsData.rx_buff_gpsGet));
    if (!device_is_ready(uart))
    {
        LOG_ERR("UART %s is not ready\n", uart->name);
        return;
    }

    ret = uart_configure(uart, &uart_cfg);
    if (ret)
    {
        LOG_ERR("Error while configuring");
        return;
    }

    ret = uart_callback_set(uart, uart_cb, rx_buff);
    if (ret)
    {
        LOG_ERR("Error while setting callback: %d", ret);
        return;
    }

    ret = uart_tx(uart, gps_config_default, sizeof(gps_config_default), SYS_FOREVER_US);
    if (ret)
    {
        LOG_ERR("Error while sending config");
        return;
    }

    k_sleep(K_MSEC(100));

    ret = uart_tx(uart, gps_config_GLL_off, sizeof(gps_config_GLL_off), SYS_FOREVER_US);
    if (ret)
    {
        LOG_ERR("Error while sending config");
        return;
    }

    k_sleep(K_MSEC(100));

    ret = uart_tx(uart, gps_config_GSA_off, sizeof(gps_config_GSA_off), SYS_FOREVER_US);
    if (ret)
    {
        LOG_ERR("Error while sending config");
        return;
    }

    k_sleep(K_MSEC(100));

    ret = uart_tx(uart, gps_config_GSV_off, sizeof(gps_config_GSV_off), SYS_FOREVER_US);
    if (ret)
    {
        LOG_ERR("Error while sending config");
        return;
    }

    k_sleep(K_MSEC(100));

    ret = uart_tx(uart, gps_config_GGA_off, sizeof(gps_config_GGA_off), SYS_FOREVER_US);
    if (ret)
    {
        LOG_ERR("Error while sending config");
        return;
    }

    k_sleep(K_MSEC(100));

    ret = uart_tx(uart, gps_config_VTG_off, sizeof(gps_config_VTG_off), SYS_FOREVER_US);
    if (ret)
    {
        LOG_ERR("Error while sending config");
        return;
    }

    k_sleep(K_MSEC(100));

    ret = uart_tx(uart, gps_config_RMC_off, sizeof(gps_config_RMC_off), SYS_FOREVER_US);
    if (ret)
    {
        LOG_ERR("Error while sending config");
        return;
    }

    k_sleep(K_MSEC(100));

    ret = uart_tx(uart, gps_config_UART1_NAV_PVT_on, sizeof(gps_config_UART1_NAV_PVT_on), SYS_FOREVER_US);
    if (ret)
    {
        LOG_ERR("Error while sending config");
        return;
    }

    k_sleep(K_MSEC(100));

    // added
    ret = uart_rx_enable(uart, rx_buff, sizeof(rx_buff), GPS_UART_RX_TIMEOUT);
    if (ret)
    {
        LOG_ERR("Error while enabling RX");
        return;
    }
    k_sleep(K_MSEC(100));

    // ret = uart_tx(uart, gps_config_1Hz, sizeof(gps_config_1Hz), SYS_FOREVER_US);
    // if (ret)
    // {
    //     LOG_ERR("Error while sending config");
    //     return;
    // }

    // k_sleep(K_MSEC(500));

    LOG_ERR("GPS send new");

    ret = uart_tx(uart, gps_new_config_25Hz, sizeof(gps_new_config_25Hz), SYS_FOREVER_US);
    if (ret)
    {
        LOG_ERR("Error while sending config");
        return;
    }

    k_sleep(K_MSEC(100));
}

GPSData_t gps_get_test()
{
    k_msgq_get(&msgq, gpsData.rx_buff_gpsGet, K_FOREVER);
    printf("GPS\n");
    gps_data_rdy = 0;

    return gpsData;
}
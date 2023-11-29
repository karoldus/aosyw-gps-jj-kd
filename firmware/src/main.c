#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "../drivers/gps-driver/include/testowy.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(main);

#define TEST_THREAD_STACKSIZE 2048
#define TEST_THREAD_PRIORITY  3

k_tid_t testThread_pid;
struct k_thread testThread_data;

struct k_thread_runtime_stats thread_stats;

GPSData_t gps_data;
char str[GPS_UART_BUF_SIZE * 4];

K_THREAD_STACK_DEFINE(test_thread_stack_area, TEST_THREAD_STACKSIZE);

void test_thread(void *p1, void *p2, void *p3)
{

    while (1)
    {
        gps_data = gps_get_test();
        for (int i = 0; i < GPS_UART_BUF_SIZE; i++)
        {
            sprintf(&str[i * 2], "%02x", gps_data.rx_buff_gpsGet[i]);
        }
        str[200] = '\0';
        LOG_INF("GPS data: %s", str);
        memset(str, '\0', GPS_UART_BUF_SIZE * 4);
    }
}

int main(void)
{
    gps_init_test();

    testThread_pid =
        k_thread_create(&testThread_data, test_thread_stack_area, K_THREAD_STACK_SIZEOF(test_thread_stack_area),
                        test_thread, NULL, NULL, NULL, TEST_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);

    k_sleep(K_FOREVER);

    return 0;
}

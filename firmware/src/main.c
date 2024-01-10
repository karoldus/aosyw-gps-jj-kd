#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "../drivers/gps-driver/gps-driver.h"

#include "../drivers/gps-driver/neo-m9n.h"

#include "../drivers/gps-driver/gps-rx.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(main);

#define TEST_THREAD_STACKSIZE 2048
#define TEST_THREAD_PRIORITY  3

k_tid_t testThread_pid;
struct k_thread testThread_data;

struct k_thread_runtime_stats thread_stats;

char str[NEO_M9N_RX_BUF_SIZE * 4];

K_THREAD_STACK_DEFINE(test_thread_stack_area, TEST_THREAD_STACKSIZE);

const struct device *neo_m9n_gps = DEVICE_DT_GET(DT_ALIAS(neo_m9n));

void test_thread(void *p1, void *p2, void *p3)
{
    const struct neo_m9n_data *dev_data = neo_m9n_gps->data;
    static uint8_t rx_buff_2[NEO_M9N_RX_BUF_SIZE] = {0};
    size_t bytes_read;

    while (1)
    {
        neo_m9n_get_packet(neo_m9n_gps, rx_buff_2, NEO_M9N_RX_BUF_SIZE, &bytes_read, K_FOREVER);
        for (int i = 0; i < bytes_read; i++)
        {
            sprintf(&str[i * 2], "%02x", rx_buff_2[i]);
        }
        str[200] = '\0';
        LOG_INF("GPS data: %s", str);
        memset(str, '\0', NEO_M9N_RX_BUF_SIZE * 4);
    }
}

int main(void)
{
    testThread_pid =
        k_thread_create(&testThread_data, test_thread_stack_area, K_THREAD_STACK_SIZEOF(test_thread_stack_area),
                        test_thread, NULL, NULL, NULL, TEST_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);

    k_sleep(K_FOREVER);

    return 0;
}

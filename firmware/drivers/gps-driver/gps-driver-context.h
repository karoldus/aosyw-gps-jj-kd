#ifndef GPS_DRIVER_CONTEXT_H_
#define GPS_DRIVER_CONTEXT_H_

#include "inttypes.h"

#define GPS_DRIVER_TX_THREAD_STACK_SIZE (1024U)
#define GPS_DRIVER_TX_THREAD_PRIORITY   3

#define GPS_DRIVER_RX_THREAD_STACK_SIZE (1024U)
#define GPS_DRIVER_RX_THREAD_PRIORITY   3

#define GPS_DRIVER_TX_QUEUE_SIZE (10U)

#define GPS_DRIVER_CALLBACKS_MAX_COUNT (10U)

#define GPS_SEND_MSG_MUTEX_TIMEOUT_MS (3000U)

/**
 * @brief Structure to communicate between high level (user API) and low level (hardware driver).
 * Represents a UBX frame without preamble and checksum.
 *
 */
typedef struct
{
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t payload_len;
    uint8_t *payload;
} ubx_msg_t;

#define GPS_HANDLER_CODES_RESPONSE      (0U)
#define GPS_HANDLER_CODES_ACK           (1U)
#define GPS_HANDLER_CODES_NACK          (2U)
#define GPS_HANDLER_CODES_TIMEOUT       (3U)
#define GPS_HANDLER_CODES_ERROR_SENDING (4U)

typedef void (*gps_handler_t)(const struct device *dev, uint8_t gps_handler_code, uint8_t class, uint8_t id,
                              ubx_msg_t *resp_msg, void *user_data);

typedef struct
{
    uint8_t *gps_payload; // ready to send to GPS (including preamble, msg class, msg id, payload and checksum)
    uint16_t gps_payload_len;
    uint8_t wait_for_response;      // 1 or 0
    gps_handler_t response_handler; // NULL if wait_for_response == 0
    void *handler_user_data;
} gps_driver_tx_queue_element_t;

#endif // GPS_DRIVER_CONTEXT_H_

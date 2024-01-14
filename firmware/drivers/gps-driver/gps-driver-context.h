#ifndef GPS_DRIVER_CONTEXT_H_
#define GPS_DRIVER_CONTEXT_H_

#include "inttypes.h"
#include <errno.h>

#define GPS_DRIVER_TX_QUEUE_SIZE       (10U)
#define GPS_DRIVER_CALLBACKS_MAX_COUNT (10U)
#define GPS_SEND_MSG_MUTEX_TIMEOUT_MS  (3000U)

typedef enum
{ // TODO: add more error codes
    GPS_OK = 0,
    GPS_ERROR = EPROTO,
    GPS_ERROR_WRONG_PARAM = ENOTSUP,
    GPS_ERROR_TIMEOUT = ETIME,
    GPS_ERROR_NO_MEM = ENOSR,
} gps_error_t;

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

/**
 * @brief Codes that will be passed to user handler.
 */
#define GPS_HANDLER_CODES_RESPONSE      (0U) // response from GPS
#define GPS_HANDLER_CODES_ACK           (1U) // ACK from GPS
#define GPS_HANDLER_CODES_NACK          (2U) // NACK from GPS
#define GPS_HANDLER_CODES_TIMEOUT       (3U) // timeout while waiting for response from GPS
#define GPS_HANDLER_CODES_ERROR_SENDING (4U) // error while sending msg to GPS

/**
 * @brief Format of user handler function that will be called when response from GPS is received.
 *
 * @param dev
 * @param gps_handler_code One of GPS_HANDLER_CODES_*
 * @param class Class of received msg (can be 0 if no response received). If ACK or NACK received, class will be class
 * for which ACK/NACK was received (not ACK class).
 * @param id Id of received msg (can be 0 if no response received). If ACK or NACK received, id will be id for which
 * ACK/NACK was received (not ACK id).
 * @param resp_msg Can be NULL if no response received (timeout or error sending msg). If not NULL, it will be
 * automaticly freed after handler execution, but user must free payload.
 * @param user_data Data passed by user during handler registration. Can be NULL. User is responsible for freeing this
 * memory.
 *
 */
typedef void (*gps_handler_t)(const struct device *dev, uint8_t gps_handler_code, uint8_t class, uint8_t id,
                              ubx_msg_t *resp_msg, void *user_data);

/**
 * @brief Element passed to tx queue to send msg to GPS by tx thread.
 */
typedef struct
{
    uint8_t *gps_payload; // payload ready to send to GPS (including preamble, msg class, msg id, payload and checksum)
    uint16_t gps_payload_len;
    uint8_t wait_for_response;      // If 1, wait for response from GPS. If 0, don't wait for response.
    gps_handler_t response_handler; // Handler for response from GPS. Use only if wait_for_response is 1.
    void *handler_user_data;        // User data passed to handler. Can be NULL. Use only if wait_for_response is 1.
} gps_driver_tx_queue_element_t;

#endif // GPS_DRIVER_CONTEXT_H_

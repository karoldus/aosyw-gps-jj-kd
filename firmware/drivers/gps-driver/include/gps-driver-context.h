#ifndef GPS_DRIVER_CONTEXT_H_
#define GPS_DRIVER_CONTEXT_H_

#include "inttypes.h"

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

#endif // GPS_DRIVER_CONTEXT_H_

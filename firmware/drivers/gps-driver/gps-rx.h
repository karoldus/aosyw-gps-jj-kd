#ifndef GPS_RX_H_
#define GPS_RX_H_

#include "gps-driver-context.h"
#include <zephyr/device.h>

/**
 * @brief Handle raw data received from GPS.
 *
 * @param dev
 * @param raw_payload Raw data received from GPS (including preamble, msg class, msg id, payload and checksum).
 * @param raw_payload_len Length of allocated memory for raw_payload (can be greater than actual payload length).
 */
void gps_handle_raw_rx(const struct device *dev, uint8_t *raw_payload, uint16_t raw_payload_len);

#endif // GPS_RX_H_

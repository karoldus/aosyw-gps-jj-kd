#include "gps-rx.h"

#include "neo-m9n.h"
#include "ubx-cfg-defines.h"
#include "ubx-data-defines.h"
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gps_rx);

// static gps_error_t gps_handle_received_ack(const struct device *dev, ubx_msg_t *msg);
// static gps_error_t gps_handle_received_other_class(const struct device *dev, ubx_msg_t *msg);
// static gps_error_t gps_handle_received_msg(const struct device *dev, ubx_msg_t *msg);

/**
 * @brief Verify checksum of UBX message
 *
 * @param payload Payload of UBX message including preamble, class, id, length, payload and checksum
 * @param payload_len Length of payload including preamble, class, id, length, payload and checksum
 * @return gps_error_t GPS_OK if checksum is correct, GPS_ERROR otherwise
 */
static gps_error_t _verify_checksum(uint8_t *payload, uint16_t payload_len)
{
    if (payload == NULL || payload_len == 0)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    uint8_t checksum_a = 0;
    uint8_t checksum_b = 0;

    for (int i = 2; i < payload_len - 2; i++) // first 2 bytes are preamble, last 2 bytes are checksum
    {
        checksum_a += *(payload + i);
        checksum_b += checksum_a;
    }

    if (checksum_a != *(payload + payload_len - 2) || checksum_b != *(payload + payload_len - 1))
    {
        return GPS_ERROR;
    }

    return GPS_OK;
}

static gps_error_t gps_find_and_execute_handler(const struct device *dev, ubx_msg_t *msg, uint8_t class, uint8_t id,
                                                uint8_t gps_code)
{
    if (dev == NULL || msg == NULL)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    struct neo_m9n_data *dev_data = dev->data;

    // Check for wait_for_resp handler

    k_sem_take(&(dev_data->callbacks_sem), K_FOREVER);

    if (dev_data->wait_for_resp_callback.class == class && dev_data->wait_for_resp_callback.id == id)
    {
        if (dev_data->wait_for_resp_callback.handler != NULL)
        {
            dev_data->wait_for_resp_callback.handler(dev, gps_code, class, id, msg,
                                                     dev_data->wait_for_resp_callback.user_data);
        }
        else
        {
            LOG_INF("No handler for response");
        }

        dev_data->wait_for_resp_callback.handler = NULL;
        dev_data->wait_for_resp_callback.user_data = NULL;
        dev_data->wait_for_resp_callback.class = 0;
        dev_data->wait_for_resp_callback.id = 0;

        k_sem_give(&(dev_data->tx_wait_for_resp_sem));
        k_sem_give(&(dev_data->callbacks_sem));

        return GPS_OK;
    }

    // Check for other handlers

    for (int i = 0; i < GPS_DRIVER_CALLBACKS_MAX_COUNT; i++)
    {
        if (dev_data->callback_list[i].class == class && dev_data->callback_list[i].id == id)
        {
            if (dev_data->callback_list[i].handler != NULL)
            {
                dev_data->callback_list[i].handler(dev, gps_code, class, id, msg, dev_data->callback_list[i].user_data);
            }
            else
            {
                LOG_INF("No handler for response");
            }

            if (dev_data->callback_list[i].one_time)
            {
                dev_data->callback_list[i].class = 0;
                dev_data->callback_list[i].id = 0;
                dev_data->callback_list[i].handler = NULL;
                dev_data->callback_list[i].user_data = NULL;
                dev_data->callback_list_count--;
            }

            k_sem_give(&(dev_data->callbacks_sem));
            return GPS_OK; // TODO: remove if we want to call all handlers and make signal flag
        }
    }

    k_sem_give(&(dev_data->callbacks_sem));
    return GPS_ERROR;
}

static gps_error_t gps_handle_received_ack(const struct device *dev, ubx_msg_t *msg)
{
    if (msg == NULL || dev == NULL)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    if (msg->msg_class != UBX_ACK_CLASS || msg->payload_len != UBX_ACK_ACK_PAYLOAD_LEN)
    {
        LOG_ERR("gps_handle_received_ack: wrong ACK payload length");
        return GPS_ERROR_WRONG_PARAM;
    }

    if (msg->msg_id != UBX_ACK_ACK_ID && msg->msg_id != UBX_ACK_NAK_ID)
    {
        LOG_ERR("gps_handle_received_ack: wrong ACK/NAK ID");
        return GPS_ERROR_WRONG_PARAM;
    }

    uint8_t ack_msg_class = (uint8_t) * (msg->payload);
    uint8_t ack_msg_id = (uint8_t) * (msg->payload + 1);
    uint8_t gps_code = 0;

    if (msg->msg_id == UBX_ACK_ACK_ID)
    {
        gps_code = GPS_HANDLER_CODES_ACK;
        LOG_INF("ACK received for msg class: 0x%02X, msg id: 0x%02X", ack_msg_class, ack_msg_id);
    }
    else
    {
        gps_code = GPS_HANDLER_CODES_NACK;
        LOG_INF("NAK received for msg class: 0x%02X, msg id: 0x%02X", ack_msg_class, ack_msg_id);
    }

    gps_error_t err = gps_find_and_execute_handler(dev, msg, ack_msg_class, ack_msg_id, gps_code);

    if (err != GPS_OK)
    {
        LOG_ERR("No handler for class 0x%02X, id 0x%02X", ack_msg_class, ack_msg_id);

        // free msg payload
        if (msg->payload != NULL)
        {
            free(msg->payload);
        }
    }

    return GPS_OK;
}

static gps_error_t gps_handle_received_other_class(const struct device *dev, ubx_msg_t *msg)
{
    if (msg == NULL || dev == NULL)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    gps_error_t err = gps_find_and_execute_handler(dev, msg, msg->msg_class, msg->msg_id, GPS_HANDLER_CODES_RESPONSE);

    if (err != GPS_OK)
    {
        LOG_ERR("No handler for class 0x%02X, id 0x%02X", msg->msg_class, msg->msg_id);

        // free msg payload
        if (msg->payload != NULL)
        {
            free(msg->payload);
        }
    }

    return GPS_OK;
}

/**
 * @brief Generic function for handling received UBX message
 *
 * @param dev
 * @param msg
 * @return gps_error_t
 */
static gps_error_t gps_handle_received_msg(const struct device *dev, ubx_msg_t *msg)
{
    if (msg == NULL || dev == NULL)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    switch (msg->msg_class)
    {
    case UBX_ACK_CLASS:
        return gps_handle_received_ack(dev, msg);
        break;

    default:
        return gps_handle_received_other_class(dev, msg);
        break;
    }

    return GPS_ERROR;
}

void gps_handle_raw_rx(const struct device *dev, uint8_t *raw_payload, uint16_t raw_payload_len)
{
    if (dev == NULL || raw_payload == NULL || raw_payload_len < UBX_MIN_MSG_LEN)
    {
        return;
    }

    // check preamble
    if (raw_payload[0] != UBX_PREAMBLE1 || raw_payload[1] != UBX_PREAMBLE2)
    {
        LOG_ERR("gps_handle_raw_rx: wrong preamble");
        return;
    }

    uint8_t msg_class = raw_payload[2];
    uint8_t msg_id = raw_payload[3];

    uint16_t payload_len_from_msg = (uint16_t)raw_payload[4] + ((uint16_t)raw_payload[5] << 8);
    uint16_t all_payload_len = payload_len_from_msg + UBX_MIN_MSG_LEN;

    if (_verify_checksum(raw_payload, all_payload_len) != GPS_OK)
    {
        LOG_ERR("gps_handle_raw_rx: wrong checksum");
        return;
    }

    ubx_msg_t msg;
    msg.msg_class = msg_class;
    msg.msg_id = msg_id;
    msg.payload_len = payload_len_from_msg;

    uint8_t *payload = malloc(payload_len_from_msg * sizeof(uint8_t));
    if (payload == NULL)
    {
        LOG_ERR("gps_handle_raw_rx: malloc failed");
        return;
    }

    memcpy(payload, raw_payload + 6, payload_len_from_msg);
    msg.payload = payload;

    gps_handle_received_msg(dev, &msg);

    if (raw_payload_len >= (all_payload_len + UBX_MIN_MSG_LEN))
    {
        LOG_ERR("gps_handle_raw_rx: more than one msg in buffer. Raw len: %d", raw_payload_len);
        gps_handle_raw_rx(dev, (uint8_t *)((void *)raw_payload + all_payload_len), (raw_payload_len - all_payload_len));
    }
}
#include "../include/gps-driver.h"
#include "../include/gps-driver-context.h"
#include "../include/ubx-cfg-defines.h"
#include "../include/ubx-data-defines.h"

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/logging/log.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(gps_driver);

#define _GET_NUMBER_OF_BYTES(key)                                                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
        number_of_bytes = (((key) & (0x7 << 28)) >> 28);                                                               \
        number_of_bytes = (number_of_bytes == 0x01 || number_of_bytes == 0x02) ? 1                                     \
                          : (number_of_bytes == 0x03)                          ? 2                                     \
                          : (number_of_bytes == 0x04)                          ? 4                                     \
                          : (number_of_bytes == 0x05)                          ? 8                                     \
                                                                               : 0;                                                             \
    } while (0)

//================================= STATIC FUNCTIONS ==================================

static uint8_t _is_little_endian()
{
    int32_t x = 1;

    return (*((char *)(&x)) == 1);
}

static gps_error_t _generate_payload_from_config_item(gps_config_item_t *config_item, uint8_t *out_payload,
                                                      uint16_t *out_payload_len)
{
    if (config_item == NULL || out_payload == NULL || out_payload_len == NULL)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    uint32_t key = config_item->key;
    uint8_t number_of_bytes = 0;
    _GET_NUMBER_OF_BYTES(key);
    if (number_of_bytes == 0)
    {
        LOG_ERR("Wrong key in config item - cannot get number of bytes");
        return GPS_ERROR_WRONG_PARAM;
    }

    // TODO: big/little endian ?
    memcpy(out_payload, &config_item->key, sizeof(config_item->key));
    memcpy(out_payload + sizeof(config_item->key), &config_item->value, number_of_bytes);

    *out_payload_len = sizeof(config_item->key) + number_of_bytes;

    return GPS_OK;
}

//=============================== GLOBAL FUNCTIONS =========================

gps_error_t gps_set_config(ubx_cfg_memory_set_del_layer_t layers, gps_config_item_t config[], uint8_t config_len)
{
    if (config_len == 0 || config_len > MAX_CONFIG_ITEMS || config == NULL || layers == 0)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    ubx_msg_t msg;
    msg.msg_class = UBX_CFG_CLASS;
    msg.msg_id = UBX_CFG_VALSET_ID;

    uint8_t *payload = malloc(config_len * MAX_CONFIG_ITEM_SIZE + 4); // 4 = 1 for version + 1 for layer + 2 reserved

    if (payload == NULL)
    {
        return GPS_ERROR_NO_MEM;
    }

    *payload = 0x00;                                      // version
    *(payload + 1) = (layers & GPS_MEMOTY_SET_LAYER_MAX); // layer
    *(payload + 2) = 0x00;                                // reserved
    *(payload + 3) = 0x00;                                // reserved

    uint16_t payload_len = 4;

    for (int i = 0; i < config_len; i++)
    {
        uint16_t payload_item_len = 0;
        gps_error_t err =
            _generate_payload_from_config_item(&config[i], (uint8_t *)(payload + payload_len), &payload_item_len);

        if (err != GPS_OK)
        {
            free(payload);
            return err;
        }

        payload_len += payload_item_len;
    }

    msg.payload_len = payload_len;
    msg.payload = payload;

    // TODO send msg and delete test below
    printf("GPS SET CONFIG MSG: ");
    for (int i = 0; i < payload_len; i++)
    {
        printf("%02X ", payload[i]);
    }
    printf(" LEN: 0x%2X\n", payload_len);

    return GPS_OK;
}

gps_error_t gps_get_config(ubx_cfg_memory_get_layer_t layer, uint32_t keys[], uint8_t keys_len)
{
    if (keys_len == 0 || keys_len > MAX_CONFIG_ITEMS || keys == NULL)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    if (layer != GPS_MEMORY_GET_LAYER_RAM && layer != GPS_MEMORY_GET_LAYER_BBR && layer != GPS_MEMORY_GET_LAYER_FLASH &&
        layer != GPS_MEMOTY_GET_LAYER_DEFAULT)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    ubx_msg_t msg;
    msg.msg_class = UBX_CFG_CLASS;
    msg.msg_id = UBX_CFG_VALGET_ID;

    uint8_t *payload = malloc(keys_len * sizeof(uint32_t) + 4); // 4 = 1 for version + 1 for layer + 2 for position

    if (payload == NULL)
    {
        return GPS_ERROR_NO_MEM;
    }

    *(payload) = 0x00; // version
    *(payload + 1) = layer;
    *(payload + 2) = 0x00; // position
    *(payload + 3) = 0x00; // position

    for (int i = 0; i < keys_len; i++)
    {
        memcpy(payload + 4 + i * sizeof(uint32_t), &keys[i], sizeof(uint32_t));
    }

    // memcpy(payload, keys, keys_len * sizeof(uint32_t));

    msg.payload_len = keys_len * sizeof(uint32_t) + 4;
    msg.payload = payload;

    // TODO send msg and delete test below, wait for response (how?)
    printf("GPS GET CONFIG MSG: ");
    for (int i = 0; i < msg.payload_len; i++)
    {
        printf("%02X ", payload[i]);
    }
    printf(" LEN: 0x%2X\n", msg.payload_len);

    return GPS_OK;
}

gps_error_t gps_del_config(ubx_cfg_memory_set_del_layer_t layer, uint32_t keys[], uint8_t keys_len)
{
    if (keys_len == 0 || keys_len > MAX_CONFIG_ITEMS || keys == NULL)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    layer = layer & (GPS_MEMORY_SET_LAYER_BBR | GPS_MEMORY_SET_LAYER_FLASH);

    if (layer == 0)
    {
        LOG_ERR("Wrong layer. Must be BBR or/and FLASH");
        return GPS_ERROR_WRONG_PARAM;
    }

    ubx_msg_t msg;
    msg.msg_class = UBX_CFG_CLASS;
    msg.msg_id = UBX_CFG_VALDEL_ID;

    uint8_t *payload = malloc(keys_len * sizeof(uint32_t) + 4); // 4 = 1 for version + 1 for layer + 2 reserved

    if (payload == NULL)
    {
        return GPS_ERROR_NO_MEM;
    }

    *(payload) = 0x00; // version
    *(payload + 1) = layer;
    *(payload + 2) = 0x00; // reserved
    *(payload + 3) = 0x00; // reserved

    for (int i = 0; i < keys_len; i++)
    {
        memcpy(payload + 4 + i * sizeof(uint32_t), &keys[i], sizeof(uint32_t));
    }

    // memcpy(payload, keys, keys_len * sizeof(uint32_t));

    msg.payload_len = keys_len * sizeof(uint32_t) + 4;
    msg.payload = payload;

    // TODO send msg and delete test below, wait for response (how?)
    printf("GPS DEL CONFIG MSG: ");
    for (int i = 0; i < msg.payload_len; i++)
    {
        printf("%02X ", payload[i]);
    }
    printf(" LEN: 0x%2X\n", msg.payload_len);

    return GPS_OK;
}

//============================== SPECIFIC CONFIGURATION ==============================

gps_error_t gps_set_cfg_rate(uint16_t nominal_time_ms, uint16_t ratio, ubxCfgTimeSystem_t time_system,
                             ubx_cfg_memory_set_del_layer_t layer)
{
    gps_config_item_t items[3];
    uint8_t items_len = 0;

    if (nominal_time_ms != 0)
    {
        items[items_len].key = UBX_CFG_KEY_ID_RATE_MEAS_U2;
        items[items_len].value.u2 = nominal_time_ms;
        items_len++;
    }

    if (ratio != 0)
    {
        items[items_len].key = UBX_CFG_KEY_ID_RATE_NAV_U2;
        items[items_len].value.u2 = ratio;
        items_len++;
    }

    if (time_system != UBX_CFG_TYPES_TIME_SYSTEM_NONE)
    {
        items[items_len].key = UBX_CFG_KEY_ID_RATE_TIMEREF_E1;
        items[items_len].value.e1 = time_system;
        items_len++;
    }

    if (items_len == 0)
    {
        LOG_ERR("No items to set");
        return GPS_ERROR_WRONG_PARAM;
    }

    return gps_set_config(layer, items, items_len);
}

/* TODO: delete comment

Musimy wysłać:
{
0xb5, 0x62, 0x06, 0x8a, 0x9A, 0x00, 0x00, 0x07, 0x00, 0x00, 0xBA, 0x00, 0x91, 0x20, 0x00, 0xBE, 0x00, 0x91,
0x20, 0x00, 0xBB, 0x00, 0x91, 0x20, 0x00, 0xBC, 0x00, 0x91, 0x20, 0x00, 0xBD, 0x00, 0x91, 0x20, 0x00, 0xC9,
0x00, 0x91, 0x20, 0x00, 0xCD, 0x00, 0x91, 0x20, 0x00, 0xCA, 0x00, 0x91, 0x20, 0x00, 0xCB, 0x00, 0x91, 0x20,
0x00, 0xCC, 0x00, 0x91, 0x20, 0x00, 0xBF, 0x00, 0x91, 0x20, 0x00, 0xC3, 0x00, 0x91, 0x20, 0x00, 0xC0, 0x00,
0x91, 0x20, 0x00, 0xC1, 0x00, 0x91, 0x20, 0x00, 0xC2, 0x00, 0x91, 0x20, 0x00, 0xC4, 0x00, 0x91, 0x20, 0x00,
0xC8, 0x00, 0x91, 0x20, 0x00, 0xC5, 0x00, 0x91, 0x20, 0x00, 0xC6, 0x00, 0x91, 0x20, 0x00, 0xC7, 0x00, 0x91,
0x20, 0x00, 0xAB, 0x00, 0x91, 0x20, 0x00, 0xAF, 0x00, 0x91, 0x20, 0x00, 0xAC, 0x00, 0x91, 0x20, 0x00, 0xAD,
0x00, 0x91, 0x20, 0x00, 0xAE, 0x00, 0x91, 0x20, 0x00, 0xB0, 0x00, 0x91, 0x20, 0x00, 0xB4, 0x00, 0x91, 0x20,
0x00, 0xB1, 0x00, 0x91, 0x20, 0x00, 0xB2, 0x00, 0x91, 0x20, 0x00, 0xB3, 0x00, 0x91, 0x20, 0x00, 0x10, 0x21
}

lub bez flash:
{
0xb5, 0x62, 0x06, 0x8a, 0x9A, 0x00, 0x00, 0x03, 0x00, 0x00, 0xBA, 0x00, 0x91, 0x20, 0x00, 0xBE, 0x00, 0x91,
0x20, 0x00, 0xBB, 0x00, 0x91, 0x20, 0x00, 0xBC, 0x00, 0x91, 0x20, 0x00, 0xBD, 0x00, 0x91, 0x20, 0x00, 0xC9,
0x00, 0x91, 0x20, 0x00, 0xCD, 0x00, 0x91, 0x20, 0x00, 0xCA, 0x00, 0x91, 0x20, 0x00, 0xCB, 0x00, 0x91, 0x20,
0x00, 0xCC, 0x00, 0x91, 0x20, 0x00, 0xBF, 0x00, 0x91, 0x20, 0x00, 0xC3, 0x00, 0x91, 0x20, 0x00, 0xC0, 0x00,
0x91, 0x20, 0x00, 0xC1, 0x00, 0x91, 0x20, 0x00, 0xC2, 0x00, 0x91, 0x20, 0x00, 0xC4, 0x00, 0x91, 0x20, 0x00,
0xC8, 0x00, 0x91, 0x20, 0x00, 0xC5, 0x00, 0x91, 0x20, 0x00, 0xC6, 0x00, 0x91, 0x20, 0x00, 0xC7, 0x00, 0x91,
0x20, 0x00, 0xAB, 0x00, 0x91, 0x20, 0x00, 0xAF, 0x00, 0x91, 0x20, 0x00, 0xAC, 0x00, 0x91, 0x20, 0x00, 0xAD,
0x00, 0x91, 0x20, 0x00, 0xAE, 0x00, 0x91, 0x20, 0x00, 0xB0, 0x00, 0x91, 0x20, 0x00, 0xB4, 0x00, 0x91, 0x20,
0x00, 0xB1, 0x00, 0x91, 0x20, 0x00, 0xB2, 0x00, 0x91, 0x20, 0x00, 0xB3, 0x00, 0x91, 0x20, 0x00, 0x0C, 0xBD
}

*/
gps_error_t gps_set_cfg_disable_all_nmea(ubx_cfg_memory_set_del_layer_t layer)
{
    const gps_config_item_t item_list[] = {
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GGA_I2C_U1, {0}},   {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GGA_SPI_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GGA_UART1_U1, {0}}, {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GGA_UART2_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GGA_USB_U1, {0}},   {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GLL_I2C_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GLL_SPI_U1, {0}},   {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GLL_UART1_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GLL_UART2_U1, {0}}, {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GLL_USB_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSA_I2C_U1, {0}},   {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSA_SPI_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSA_UART1_U1, {0}}, {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSA_UART2_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSA_USB_U1, {0}},   {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSV_I2C_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSV_SPI_U1, {0}},   {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSV_UART1_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSV_UART2_U1, {0}}, {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_GSV_USB_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_RMC_I2C_U1, {0}},   {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_RMC_SPI_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_RMC_UART1_U1, {0}}, {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_RMC_UART2_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_RMC_USB_U1, {0}},   {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_VTG_I2C_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_VTG_SPI_U1, {0}},   {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_VTG_UART1_U1, {0}},
        {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_VTG_UART2_U1, {0}}, {UBX_CFG_KEY_ID_MSGOUT_NMEA_ID_VTG_USB_U1, {0}},
    };

    return gps_set_config(layer, (gps_config_item_t *)item_list, sizeof(item_list) / sizeof(gps_config_item_t));
}

// TODO: test if it works
gps_error_t gps_set_enable_ubx_nav_pvt(ubx_cfg_memory_set_del_layer_t layer)
{
    const gps_config_item_t item_list[] = {{UBX_CFG_KEY_ID_MSGOUT_UBX_NAV_PVT_I2C_U1, {1}},
                                           {UBX_CFG_KEY_ID_MSGOUT_UBX_NAV_PVT_SPI_U1, {1}},
                                           {UBX_CFG_KEY_ID_MSGOUT_UBX_NAV_PVT_UART1_U1, {1}},
                                           {UBX_CFG_KEY_ID_MSGOUT_UBX_NAV_PVT_UART2_U1, {1}},
                                           {UBX_CFG_KEY_ID_MSGOUT_UBX_NAV_PVT_USB_U1, {1}}};

    return gps_set_config(layer, (gps_config_item_t *)item_list, sizeof(item_list) / sizeof(gps_config_item_t));
}

/////////////////////////////////////////////////////////////////////////
//======================== DATA AND ACK HANDLING ========================
/////////////////////////////////////////////////////////////////////////

static gps_error_t gps_handle_received_ack(ubx_msg_t *msg)
{
    if (msg == NULL)
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

    uint8_t ack_msg_class = *(msg->payload);
    uint8_t ack_msg_id = *(msg->payload + 1);

    // TODO: logic

    if (msg->msg_id == UBX_ACK_ACK_ID)
    {
        LOG_INF("ACK received for msg class: 0x%02X, msg id: 0x%02X", ack_msg_class, ack_msg_id);
    }
    else
    {
        LOG_INF("NAK received for msg class: 0x%02X, msg id: 0x%02X", ack_msg_class, ack_msg_id);
    }

    return GPS_OK;
}

static gps_error_t gps_handle_received_cfg(ubx_msg_t *msg)
{
    // TODO: implement
    return GPS_OK;
}

static gps_error_t gps_handle_received_nav(ubx_msg_t *msg)
{
    // TODO: implement
    return GPS_OK;
}

static gps_error_t gps_handle_received_other_class(ubx_msg_t *msg)
{
    // TODO: implement
    return GPS_OK;
}

static gps_error_t gps_handle_received_msg(ubx_msg_t *msg)
{
    if (msg == NULL)
    {
        return GPS_ERROR_WRONG_PARAM;
    }

    switch (msg->msg_class)
    {
    case UBX_ACK_CLASS:
        return gps_handle_received_ack(msg);
        break;

    case UBX_CFG_CLASS:
        return gps_handle_received_cfg(msg);
        break;

    case UBX_NAV_CLASS:
        return gps_handle_received_nav(msg);
        break;

    default:
        return gps_handle_received_other_class(msg);
        break;
    }

    return GPS_ERROR;
}
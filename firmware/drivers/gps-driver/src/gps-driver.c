#include "../include/gps-driver.h"
#include "../include/gps-driver-context.h"
#include "../include/ubx-defines.h"

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

gps_error_t gps_set_config(gps_memory_set_layer_t layers, gps_config_item_t config[], uint8_t config_len)
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

gps_error_t gps_get_config(gps_memory_get_layer_t layer, uint32_t keys[], uint8_t keys_len)
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

gps_error_t gps_del_config(gps_memory_set_layer_t layer, uint32_t keys[], uint8_t keys_len)
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

#ifndef UBX_DATA_DEFINES_H
#define UBX_DATA_DEFINES_H

#include "inttypes.h"

#define UBX_MIN_MSG_LEN 8 // preamble + class + id + length + checksum

//============================ CLASSES =========================================

#define UBX_NAV_CLASS 0x01
#define UBX_ACK_CLASS 0x05

//============================ IDS =============================================

//======== UBX-NAV ========
#define UBX_NAV_PVT_ID 0x07

//======== UBX-ACK ========
#define UBX_ACK_ACK_ID 0x01
#define UBX_ACK_NAK_ID 0x00

//============================ PAYLOAD LENGTHS ================================
//======== UBX-NAV ========
#define UBX_NAV_PVT_PAYLOAD_LEN 92

//======== UBX-ACK ========
#define UBX_ACK_ACK_PAYLOAD_LEN 2

//============================ STRUCTURES =======================================

//======== UBX-NAV ========

typedef struct __attribute__((__packed__))
{
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint16_t flags3;
    uint8_t reserved0[4];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
} ubx_nav_pvt_t;

#endif // UBX_DATA_DEFINES_H

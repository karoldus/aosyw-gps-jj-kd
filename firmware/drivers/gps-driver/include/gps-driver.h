#ifndef GPS_DRIVER_H_
#define GPS_DRIVER_H_

typedef enum
{ // TODO: add more error codes
    GPS_OK = 0,
    GPS_ERROR = -1,
    GPS_ERROR_TIMEOUT = -2,
    GPS_ERROR_CHECKSUM = -3,
    GPS_ERROR_PAYLOAD = -4,
    GPS_ERROR_UNKNOWN = -5,
} gps_error_t;

//============== CONFIGURATION API ==============

#endif // GPS_DRIVER_H_

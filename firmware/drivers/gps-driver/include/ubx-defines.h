#ifndef UBX_DEFINES_H_
#define UBX_DEFINES_H_

//============================ CLASSES =========================================

#define UBX_CFG_CLASS 0x06

//============================ ID ==============================================

#define UBX_CFG_VALDEL_ID 0x8C
#define UBX_CFG_VALGET_ID 0x8B
#define UBX_CFG_VALSET_ID 0x8A

//============================ CONFIG KEYS =====================================

#define UBX_CFG_RATE_MEAS_KEY    0x30210001 // [U2] Nominal time between GNSS measurements, in milliseconds
#define UBX_CFG_RATE_NAV_KEY     0x30210002 // [U2] Ratio of number of measurements to number of navigation solutions
#define UBX_CFG_RATE_TIMEREF_KEY 0x20210003 // [E1] Time system to which measurements are aligned

typedef enum
{
    UBX_CFG_RATE_TIMREF_UTC = 0,  // Align measurements to UTC time
    UBX_CFG_RATE_TIMREF_GPS = 1,  // Align measurements to GPS time
    UBX_CFG_RATE_TIMREF_GLO = 2,  // Align measurements to GLONASS time
    UBX_CFG_RATE_TIMREF_BDS = 3,  // Align measurements to BeiDou time
    UBX_CFG_RATE_TIMREF_GAL = 4,  // Align measurements to Galileo time
    UBX_CFG_RATE_TIMREF_NAVIC = 5 // Align measurements to NavIC time
} ubx_cfg_rate_timeref_t;


#define UBX_CFG_ANA_USE_ANA_KEY 0x10230001 // [L] Use AssistNow Autonomous
#define UBX_CFG_ANA_ORBMAXERR_KEY 0x30230002 // [U2] Maximum acceptable (modeled) orbit error. Range is from 5 to 1000 (m)

#define UBX_CFG_BATCH_ENABLE_KEY 0x10260013 // [L] Enable data batching
#define UBX_CFG_BATCH_PIOENABLE_KEY 0x10260014 // [L] Enable PIO notification
#define UBX_CFG_BATCH_MAXENTRIES_KEY 0x30260015 // [U2] Maximum entries in buffer. Size of buffer in number of epochs to store. The firmware will reject this configuration if it exceeds the available memory.
#define UBX_CFG_BATCH_WARNTHRS_KEY 0x30260016 // [U2] Buffer fill level warning threshold. Buffer fill level that triggers PIO notification, in number of epochs stored.
#define UBX_CFG_BATCH_PIOACTIVELOW_KEY 0x10260018 // [L] PIO is active low
#define UBX_CFG_BATCH_PIOID_KEY 0x20260019 // [U1] PIO ID for buffer level notification. PIO that is used for buffer fill level notification. It must not be assigned to a different function.
#define UBX_CFG_BATCH_EXTRAPVT_KEY 0x1026001a // [L] Include extra PVT data. Include additional PVT information in UBX-LOG-BATCH messages. If not selected only basic information is included.
#define UBX_CFG_BATCH_EXTRAODO_KEY 0x1026001b // [L] Include odometer data. The fields distance, totalDistance, and distanceStd in UBX-LOG-BATCH are only valid if this flag is set.

#define UBX_CFG_GEOFENCE_CONFLVL_KEY 0x20240011 // [E1] - Required confidence level for state evaluation. This value times the position's standard deviation (sigma) defines the confidence band. See Table 8 below for a list of possible constants for this item.
#define UBX_CFG_GEOFENCE_USE_PIO_KEY 0x10240012 // [L] - Use PIO combined fence state output
#define UBX_CFG_GEOFENCE_PINPOL_KEY 0x20240013 // [E1] - PIO pin polarity. See Table 9 below for a list of possible constants for this item.
#define UBX_CFG_GEOFENCE_PIN_KEY 0x20240014 // [U1] - PIO pin number
#define UBX_CFG_GEOFENCE_USE_FENCE1_KEY 0x10240020 // [L] - Use first geofence
#define UBX_CFG_GEOFENCE_FENCE1_LAT_KEY 0x40240021 // [I4] 1e-7 deg - Latitude of the first geofence circle center
#define UBX_CFG_GEOFENCE_FENCE1_LON_KEY 0x40240022 // [I4] 1e-7 deg - Longitude of the first geofence circle center
#define UBX_CFG_GEOFENCE_FENCE1_RAD_KEY 0x40240023 // [U4] 0.01 m - Radius of the first geofence circle
#define UBX_CFG_GEOFENCE_USE_FENCE2_KEY 0x10240030 // [L] - Use second geofence
#define UBX_CFG_GEOFENCE_FENCE2_LAT_KEY 0x40240031 // [I4] 1e-7 deg - Latitude of the second geofence circle center
#define UBX_CFG_GEOFENCE_FENCE2_LON_KEY 0x40240032 // [I4] 1e-7 deg - Longitude of the second geofence circle center
#define UBX_CFG_GEOFENCE_FENCE2_RAD_KEY 0x40240033 // [U4] 0.01 m - Radius of the second geofence circle
#define UBX_CFG_GEOFENCE_USE_FENCE3_KEY 0x10240040 // [L] - Use third geofence
#define UBX_CFG_GEOFENCE_FENCE3_LAT_KEY 0x40240041 // [I4] 1e-7 deg - Latitude of the third geofence circle center
#define UBX_CFG_GEOFENCE_FENCE3_LON_KEY 0x40240042 // [I4] 1e-7 deg - Longitude of the third geofence circle center
#define UBX_CFG_GEOFENCE_FENCE3_RAD_KEY 0x40240043 // [U4] 0.01 m - Radius of the third geofence circle
#define UBX_CFG_GEOFENCE_USE_FENCE4_KEY 0x10240050 // [L] - Use fourth geofence
#define UBX_CFG_GEOFENCE_FENCE4_LAT_KEY 0x40240051 // [I4] 1e-7 deg - Latitude of the fourth geofence circle center
#define UBX_CFG_GEOFENCE_FENCE4_LON_KEY 0x40240052 // [I4] 1e-7 deg - Longitude of the fourth geofence circle center
#define UBX_CFG_GEOFENCE_FENCE4_RAD_KEY 0x40240053 // [U4] 0.01 m - Radius of the fourth geofence circle

// Enum dla CFG-GEOFENCE-CONFLVL
enum CFG_GEOFENCE_CONFLVL {
    CFG_GEOFENCE_CONFLVL_NO_CONFIDENCE = 0,     // No confidence (L000)
    CFG_GEOFENCE_CONFLVL_68_PERCENT = 1,        // 68% (L680)
    CFG_GEOFENCE_CONFLVL_95_PERCENT = 2,        // 95% (L950)
    CFG_GEOFENCE_CONFLVL_99_7_PERCENT = 3,      // 99.7% (L997)
    CFG_GEOFENCE_CONFLVL_99_99_PERCENT = 4,     // 99.99% (L9999)
    CFG_GEOFENCE_CONFLVL_99_9999_PERCENT = 5   // 99.9999% (L999999)
};

// Enum dla CFG-GEOFENCE-PINPOL
enum CFG_GEOFENCE_PINPOL {
    CFG_GEOFENCE_PINPOL_LOW_INSIDE = 0,     // PIO low means inside geofence (LOW_IN)
    CFG_GEOFENCE_PINPOL_LOW_OUTSIDE = 1     // PIO low means outside geofence (LOW_OUT)
};

// Enums dla CFG-HW-ANT_SUP_ENGINE
enum CFG_HW_ANT_SUP_ENGINE {
    CFG_HW_ANT_SUP_ENGINE_EXT = 0, // Use the EXT engine
    CFG_HW_ANT_SUP_ENGINE_MADC = 1 // Use the MADC engine
};

// Definicje dla CFG-HW-ANT
#define UBX_CFG_HW_ANT_CFG_VOLTCTRL_KEY 0x10a3002e // [L] - Active antenna voltage control flag. Enable active antenna voltage control flag. Used by EXT and MADC engines.
#define UBX_CFG_HW_ANT_CFG_SHORTDET_KEY 0x10a3002f // [L] - Short antenna detection flag. Enable short antenna detection flag. Used by EXT and MADC engines.
#define UBX_CFG_HW_ANT_CFG_SHORTDET_POL_KEY 0x10a30030 // [L] - Short antenna detection polarity. Set to true if polarity of the antenna short detection is active low. Used by EXT engine.
#define UBX_CFG_HW_ANT_CFG_OPENDET_KEY 0x10a30031 // [L] - Open antenna detection flag. Enable open antenna detection flag. Used by EXT and MADC engines.
#define UBX_CFG_HW_ANT_CFG_OPENDET_POL_KEY 0x10a30032 // [L] - Open antenna detection polarity. Set to true if polarity of the antenna open detection is active low. Used by EXT engine.
#define UBX_CFG_HW_ANT_CFG_PWRDOWN_KEY 0x10a30033 // [L] - Power down antenna flag. Enable power down antenna logic in the event of antenna short circuit. CFG-HW-ANT_CFG_SHORTDET must be enabled to use this feature. Used by EXT and MADC engines.
#define UBX_CFG_HW_ANT_CFG_PWRDOWN_POL_KEY 0x10a30034 // [L] - Power down antenna logic polarity. Set to true if polarity of the antenna power down logic is active high. Used by EXT and MADC engines.
#define UBX_CFG_HW_ANT_CFG_RECOVER_KEY 0x10a30035 // [L] - Automatic recovery from short state flag. Enable automatic recovery from short state. Used by EXT and MADC engines.
#define UBX_CFG_HW_ANT_SUP_SWITCH_PIN_KEY 0x20a30036 // [U1] - ANT1 PIO number. Antenna Switch (ANT1) PIO number. Used by EXT and MADC engines.
#define UBX_CFG_HW_ANT_SUP_SHORT_PIN_KEY 0x20a30037 // [U1] - ANT0 PIO number. Antenna Short (ANT0) PIO number. Used by EXT engine.
#define UBX_CFG_HW_ANT_SUP_OPEN_PIN_KEY 0x20a30038 // [U1] - ANT2 PIO number. Antenna Switch (ANT2) PIO number. Used by EXT engine.
#define UBX_CFG_HW_ANT_ON_SHORT_US_KEY 0x30a3003c // [U2] - ANT on->short timeout[us]. Delay in microseconds between turning the antenna power supply on and enabling the antenna short circuit detection.
#define UBX_CFG_HW_ANT_SUP_ENGINE_KEY 0x20a30054 // [E1] - Antenna supervisor engine selection. Select the engine used to evaluate antenna state. See Table 11 below for a list of possible constants for this item.
#define UBX_CFG_HW_ANT_SUP_SHORT_THR_KEY 0x20a30055 // [U1] mV - Antenna supervisor MADC engine short detection threshold. Threshold above which antenna short is detected. Used by MADC engine.
#define UBX_CFG_HW_ANT_SUP_OPEN_THR_KEY 0x20a30056 // [U1] mV - Antenna supervisor MADC engine open detection threshold. Threshold below which antenna open/disconnected is detected. Used by MADC engine.

// Definicje dla CFG-I2C
#define UBX_CFG_I2C_ADDRESS_KEY 0x20510001 // [U1] - I2C address of the receiver (7 bits)
#define UBX_CFG_I2C_EXTENDEDTIMEOUT_KEY 0x10510002 // [L] - Flag to disable timeouting the interface after 1.5 s
#define UBX_CFG_I2C_ENABLED_KEY 0x10510003 // [L] - Flag to indicate if the I2C interface should be enabled
#define UBX_CFG_I2CINPROT_UBX_KEY 0x10710001 // [L] - Flag to indicate if UBX should be an input protocol on I2C
#define UBX_CFG_I2CINPROT_NMEA_KEY 0x10710002 // [L] - Flag to indicate if NMEA should be an input protocol on I2C
#define UBX_CFG_I2CINPROT_RTCM3X_KEY 0x10710004 // [L] - Flag to indicate if RTCM3X should be an input protocol on I2C
#define UBX_CFG_I2COUTPROT_UBX_KEY 0x10720001 // [L] - Flag to indicate if UBX should be an output protocol on I2C
#define UBX_CFG_I2COUTPROT_NMEA_KEY 0x10720002 // [L] - Flag to indicate if NMEA should be an output protocol on I2C

// Enums dla CFG-INFMSG
enum CFG_INFMSG {
    CFG_INFMSG_ERROR = 0x01,  // Enable ERROR information messages
    CFG_INFMSG_WARNING = 0x02,  // Enable WARNING information messages
    CFG_INFMSG_NOTICE = 0x04,  // Enable NOTICE information messages
    CFG_INFMSG_TEST = 0x08,  // Enable TEST information messages
    CFG_INFMSG_DEBUG = 0x10  // Enable DEBUG information messages
};

// Definicje dla CFG-INFMSG
#define UBX_CFG_INFMSG_UBX_I2C_KEY 0x20920001 // [X1] - Information message enable flags for the UBX protocol on the I2C interface. See Table 16 below for a list of possible constants for this item.
#define UBX_CFG_INFMSG_UBX_UART1_KEY 0x20920002 // [X1] - Information message enable flags for the UBX protocol on the UART1 interface. See Table 16 below for a list of possible constants for this item.
#define UBX_CFG_INFMSG_UBX_UART2_KEY 0x20920003 // [X1] - Information message enable flags for the UBX protocol on the UART2 interface. See Table 16 below for a list of possible constants for this item.
#define UBX_CFG_INFMSG_UBX_USB_KEY 0x20920004 // [X1] - Information message enable flags for the UBX protocol on the USB interface. See Table 16 below for a list of possible constants for this item.
#define UBX_CFG_INFMSG_UBX_SPI_KEY 0x20920005 // [X1] - Information message enable flags for the UBX protocol on the SPI interface. See Table 16 below for a list of possible constants for this item.
#define UBX_CFG_INFMSG_NMEA_I2C_KEY 0x20920006 // [X1] - Information message enable flags for the NMEA protocol on the I2C interface. See Table 16 below for a list of possible constants for this item.
#define UBX_CFG_INFMSG_NMEA_UART1_KEY 0x20920007 // [X1] - Information message enable flags for the NMEA protocol on the UART1 interface. See Table 16 below for a list of possible constants for this item.
#define UBX_CFG_INFMSG_NMEA_UART2_KEY 0x20920008 // [X1] - Information message enable flags for the NMEA protocol on the UART2 interface. See Table 16 below for a list of possible constants for this item.
#define UBX_CFG_INFMSG_NMEA_USB_KEY 0x20920009 // [X1] - Information message enable flags for the NMEA protocol on the USB interface. See Table 16 below for a list of possible constants for this item.
#define UBX_CFG_INFMSG_NMEA_SPI_KEY 0x2092000a // [X1] - Information message enable flags for the NMEA protocol on the SPI interface. See Table 16 below for a list of possible constants for this item.

// Definicje dla CFG-ITFM
#define UBX_CFG_ITFM_BBTHRESHOLD_KEY 0x20410001 // [U1] - Broadband jamming detection threshold
#define UBX_CFG_ITFM_CWTHRESHOLD_KEY 0x20410002 // [U1] - CW jamming detection threshold
#define UBX_CFG_ITFM_ENABLE_KEY 0x1041000d // [L] - Enable interference detection
#define UBX_CFG_ITFM_ANTSETTING_KEY 0x20410010 // [E1] - Antenna setting. See Table 18 below for a list of possible constants for this item.
#define UBX_CFG_ITFM_ENABLE_AUX_KEY 0x10410013 // [L] - Scan auxiliary bands

// Enums dla CFG-ITFM_ANTSETTING
enum CFG_ITFM_ANTSETTING {
    CFG_ITFM_ANTSETTING_UNKNOWN = 0, // Unknown
    CFG_ITFM_ANTSETTING_PASSIVE = 1, // Passive
    CFG_ITFM_ANTSETTING_ACTIVE = 2 // Active
};

// Definicje dla CFG-LOGFILTER
#define UBX_CFG_LOGFILTER_RECORD_ENA_KEY 0x10de0002 // [L] - Recording enabled
#define UBX_CFG_LOGFILTER_ONCE_PER_WAKE_UP_ENA_KEY 0x10de0003 // [L] - Once per wake up
#define UBX_CFG_LOGFILTER_APPLY_ALL_FILTERS_KEY 0x10de0004 // [L] - Apply all filter settings
#define UBX_CFG_LOGFILTER_MIN_INTERVAL_KEY 0x30de0005 // [U2] s - Minimum time interval between logged positions
#define UBX_CFG_LOGFILTER_TIME_THRS_KEY 0x30de0006 // [U2] s - Time threshold
#define UBX_CFG_LOGFILTER_SPEED_THRS_KEY 0x30de0007 // [U2] m/s - Speed threshold
#define UBX_CFG_LOGFILTER_POSITION_THRS_KEY 0x40de0008 // [U4] m - Position threshold

// Definicje dla CFG-MOT
#define UBX_CFG_MOT_GNSSSPEED_THRS_KEY 0x20250038 // [U1] 0.01 m/s - GNSS speed threshold below which platform is considered as stationary (a.k.a. static hold threshold)
#define UBX_CFG_MOT_GNSSDIST_THRS_KEY 0x3025003b // [U2] - Distance above which GNSS-based stationary motion is exit (a.k.a. static hold distance threshold)





#endif // UBX_DEFINES_H

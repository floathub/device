/*

   Versioning for protocol, device, etc.
 
*/


#define  FLOATHUB_PROTOCOL_VERSION 2
#define  FLOATHUB_ENCRYPT_VERSION  2
#define  FLOATHUB_MODEL_DESCRIPTION "FHX.P9.O3.v2.5.2"


/*
   Option for different kinds of Barometric Hardware
*/

#define BARO_HWARE_BMP280 1
#define BARO_HWARE_BME280 2
#define BARO_HWARE_BME680 3
#define BARO_HWARE_BMP180 4

#define BARO_HWARE BARO_HWARE_BMP280
//#define BARO_HWARE BARO_HWARE_BME280
//#define BARO_HWARE BARO_HWARE_BME680
//#define BARO_HWARE BARO_HWARE_BMP180

/*
   Option for different kinds of GPS Hardware
*/

#define GPS_HWARE_OLD_ULTIMATE 1
#define GPS_HWARE_NEW_ULTIMATE 2
#define GPS_HWARE_HILETGO      3
#define GPS_HWARE_CASIC        4


//#define GPS_HWARE GPS_HWARE_OLD_ULTIMATE
//#define GPS_HWARE GPS_HWARE_NEW_ULTIMATE
#define GPS_HWARE GPS_HWARE_HILETGO
//#define GPS_HWARE GPS_HWARE_CASIC



/*
  Option for different voltage dividers
*/

#define VOLTAGE_DIVIDER  37.213		// Orig Shield
//#define VOLTAGE_DIVIDER  20.06      // Teensy-Compatible Shield


/*
  Compile time option/debug flags
*/

//#define CONSOLE_DEBUG_ON
//#define GPS_DEBUG_ON
//#define GPS_SOURCE_DEBUG_ON
//#define PUMP_DEBUG_ON
//#define EXECUTION_PATH_DEBUG_ON
//#define NMEA_DEBUG_ON
//#define DEBUG_MEMORY_ON
//#define STRESS_MEMORY_ON
//#define BYPASS_AES_ON
//#define BARO_DEBUG_ON	
//#define ACTIVE_DEBUG_ON
//#define SERIAL_DEBUG_ON
//#define SOFTSERIAL_DEBUG_ON



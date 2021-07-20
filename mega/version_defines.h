/*

   Versioning for protocol, device, etc.
 
*/


#define  FLOATHUB_PROTOCOL_VERSION 2
#define  FLOATHUB_ENCRYPT_VERSION  2
#define  FLOATHUB_MODEL_DESCRIPTION "FHX.P9.O3.v2.5.0"


/*
   Option for different kinds of Barometric Hardware
*/

#define BARO_HWARE_BMP280 1
#define BARO_HWARE_BME280 2
#define BARO_HWARE_BME680 3
#define BARO_HWARE_BMP180 4

//#define BARO_HWARE BARO_HWARE_BMP280
#define BARO_HWARE BARO_HWARE_BME280
//#define BARO_HWARE BARO_HWARE_BME680
//#define BARO_HWARE BARO_HWARE_BMP180



/*
  Compile time option/debug flags
*/

//#define CONSOLE_DEBUG_ON
//#define GPS_DEBUG_ON
//#define GPS_SOURCE_DEBUG_ON
//#define PUMP_DEBUG_ON
//#define EXECUTION_PATH_DEBUG_ON
//#define NMEA_DEBUG_ON
//                  #define DEBUG_MEMORY_ON
//                  #define STRESS_MEMORY_ON
//                  #define BYPASS_AES_ON
//#define BARO_DEBUG_ON	
//#define ACTIVE_DEBUG_ON
//#define SERIAL_DEBUG_ON
//#define SOFTSERIAL_DEBUG_ON



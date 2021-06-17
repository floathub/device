/*

  FloatHub Code for dealing with N2k data. 

  None of this would be possible without the remarkable 
  NMEA2000 Library from Timo Lappalainen & Kave Oy <www.kave.fi>
  
  Also owes a great deal to their NMEA0183 library as well.

*/

// #include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <NMEA0183.h>
#include <NMEA2000.h>
#include <NMEA0183AISMessages.h>

#ifndef N2K_VARIABLES
#define N2K_VARIABLES


//cycle counter for output
char n2k_output_cycle;

//Overall GPS/fix status
bool n2k_gps_valid;

// Rapid Location Variables 
double n2k_latitude;
double n2k_longitude;
unsigned long n2k_location_timestamp;

// Rapid course and speed Variables
double n2k_cog;
double n2k_sog;
unsigned long n2k_cogsog_timestamp;

// Variation variables
double n2k_variation;
unsigned long n2k_variation_timestamp;

// Deviation variables
double n2k_deviation;
unsigned long n2k_deviation_timestamp;

// Heading (way vessel is point, not COG)

double n2k_heading_true;
unsigned long n2k_heading_true_timestamp;
double n2k_heading_magnetic;
unsigned long n2k_heading_magnetic_timestamp;

// Depth variables
double n2k_depth;
double n2k_offset;
unsigned long n2k_depth_timestamp;

// SOG
double n2k_stw;
unsigned long n2k_stw_timestamp;


// GPS/GNSS Fix Variables
unsigned char n2k_siv;
double n2k_hdp;
double n2k_altitude;
double n2k_fix_geosep;
//double n2k_fix_age;
//uint16_t n2k_fix_refid;
unsigned long n2k_fix_timestamp;
uint16_t n2k_fix_days_1970;
double n2k_fix_seconds;
bool n2k_fix_valid;


// Water Temp
double n2k_water_temperature;
unsigned long n2k_water_temperature_timestamp;

// Wind
double n2k_wind_true_speed;
double n2k_wind_true_direction;
unsigned long n2k_wind_true_timestamp;

double n2k_wind_apparent_speed;
double n2k_wind_apparent_direction;
unsigned long n2k_wind_apparent_timestamp;


//
//  Batteries
//

#define MAX_N2K_BATTERIES 6
unsigned char  n2k_battery_map[MAX_N2K_BATTERIES];
double         n2k_battery_voltage[MAX_N2K_BATTERIES];
unsigned long  n2k_battery_timestamp[MAX_N2K_BATTERIES];

//
//  Need one of these objects for AIS messages
//

tNMEA0183AISMsg NMEA0183AISMsg;

//
//  Conversion flags
//

bool FLAG_GPS_N2K_TO_NMEA;
bool FLAG_GPS_NMEA_TO_N2K;

bool FLAG_ENV_INT_TO_N2K;
bool FLAG_ENV_INT_TO_NMEA;
bool FLAG_ENV_N2K_TO_NMEA;
bool FLAG_ENV_NMEA_TO_N2K;

bool FLAG_VOL_INT_TO_N2K;

bool FLAG_NAV_N2K_TO_NMEA;
bool FLAG_NAV_NMEA_TO_N2K;

bool FLAG_DEP_N2K_TO_NMEA;
bool FLAG_DEP_NMEA_TO_N2K;

bool FLAG_WIN_N2K_TO_NMEA;
bool FLAG_WIN_NMEA_TO_N2K;

bool FLAG_AIS_N2K_TO_NMEA;
bool FLAG_AIS_NMEA_TO_N2K;

#endif

void HandleNMEA2000Messages(const tN2kMsg &N2kMsg);


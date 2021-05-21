/*

  FloatHub Arduino Code
  (c) 2011-2021 Modiot Labs
  (begun June 6, 2011)

  February 2021 
  Reworked to run on a Teensy rather than the original Arduino Mega

  December 2020 
  Major rework to do all the encryption at the ESP level so we have enough
  resources on the mega to handle the new N2K processing. All AES moved to 
  the ESP8266 level. 

  March 2018
  Rewrite of cellular system to use Particle Elctron rather than Hologram
  Dash (which was suddenly discontinued)

  January-February 2018
  Celullar communications via Hologram DASH and AIS relaying to fdr server.

  September, 2017
  Better NMEA and HS-NMEA parsing, plus NMEA by default on wired-USB port

  December, 2016
  Final tweaking to a 1.0 release, lot of fine tuning fiddly bits with WiFi
  Public/Private dropping, reconnecting, etc.

  August, 2016
  Tweaked Serial/NMEA issues. Note that the only tested and recommended
  development environment is spec'd in the README.  Lot of the environment
  modifications are related to serial/nmea/soft-serial timing issues and
  things can go haywire without those recommended settings.

  May, 2016
  More responsibility over in the ESP8266 for most configuration values. 
  This mega code now just really collects sampled data and marks it up
  appropriately.  It then hands messages off to the ESP8266.  Also moved GPS
  _back_ to real serial as getting I2C daughterboards made was a nightmare. 
  As a result, moved High Speed NMEA --> in to software serial.

  January, 2016
  Fairly massive rewrite to move GPS to I2C and embed an ESP8266 as main
  comm/configuration channel, thereby also ending up with a NMEA WiFi
  multiplexor. 

  July, 2015
  Simplified active to just use GPS SOG. Had NMEA values get nuked based on timestamp.

  March 26, 2014
  Realized that watchdog_timestamps have to be set to current millis()
  values, not stale variables that may have been set some time ago in the
  WiFi code where some calls are blocking and a lot of time may have
  elapsed.

  April 4, 2014
  Change wireless to use Adafruit CC3000 shield as the "official" Arduino
  wifi shield is simply not reliable. Remove hardware reset. 

  March 6, 2014
  Make changes to use a hardware line tied to the reset pin via a 1K Ohm
  resistor.  This can be triggered after repeated failed attempts to
  communicate, in an attempt to make the Arduino wifi shield acceptably
  reliable (in tends to come back up ok after a hardware reset).

  Feb 21, 2014
  Put Wifi version on hold as Arduion Wifi Shield is simply not reliable
  with current library and firmware versions.

  Feb 20, 2014
  Moved _back_ to Arduino development environment, as only easy wat to
  compile with Beta 1.5 (necessary to interact with reliable 1.1.0 WiFi
  firmware)
  
  Feb 19, 2014
  Added NMEA _out_ so device can also be standalone GPS and/or fold GPS NMEA
  into other incoming NMEA (on NMEA in).
  
  May 2013
  Added NMEA input to the mix, so basic feature complete with NMEA,
  encryption, etc.  Added protocol/encryption version info to the protocol
  itself, plus numbering of pumps, batteries and chargers. 

  April, 2013
  Brought in encrypted/$FHS code that was working back in July and made it
  trunk. It encrypts content using aes-128-cbc encryption
 
  March, 2013
  Moved to a proper text-based build environment and versioning system on
  git-hub.

  Nov. 6  2012
  Working _extremely_ well now, including active sensor, data storage when
  GPRS unavailable, etc.  Remaining task; cleanout enough string stuff, then
  implement NMEA input on Serial2.


*/


/*
  Anything which changes what the Floathub Data Receiver (fdr) needs to do
  to parse data should bump something in the version defines.  This include
  also covers any compile time flags for version/hardware variations.
*/

#include "version_defines.h"


/*
  Based on what's in defines (i.e. what baro/temp device is attached) include the correct code
*/

#if BARO_HWARE == BARO_HWARE_BMP280
  #include "src/libs/Adafruit_BMP280/Adafruit_BMP280.h"
#elif BARO_HWARE == BARO_HWARE_BME280
  #include "src/libs/Adafruit_BME280/Adafruit_BME280.h"
#elif BARO_HWARE == BARO_HWARE_BME680
  #include "src/libs/Adafruit_BME680/Adafruit_BME680.h"
#elif BARO_HWARE == BARO_HWARE_BMP180
  #include "src/libs/Adafruit_BMP/Adafruit_BMP085.h"
#endif

/*
  Possibly n2k stuff
*/

#include "n2k.h"
#ifdef N2K_CODE_ON
// #define USE_MCP_CAN_CLOCK_SET 8
// #define N2k_SPI_CS_PIN 11
// #define N2k_CAN_INT_PIN 21 // Interrupt pin definition for Ardino Mega or other "CAN bus shield" boards.
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
//#include "N2kDataToNMEA0183.h"
//#include <N2kMsg.h>
//#include <NMEA2000.h>
//#include <N2kMessages.h>


// #include "BoardSerialNumber.h"

//#ifdef ARDUINO
//#define NMEA0183_Out_Stream_Speed 115200
//#define NMEA0183_Out_Stream Serial

const unsigned long TransmitMessages[] PROGMEM={0};
const unsigned long ReceiveMessages[] PROGMEM={/*126992L,*/127250UL,127258UL,128259UL,128267UL,129025UL,129026L,129029L,0};
#endif

/*
  Libraries and whatnot
*/
#include <EEPROM.h>
#include "src/libs/Time/Time.h"


/*
  Watchdog setup
*/
#include <Watchdog_t4.h>
WDT_T4<WDT1> watchdog;


/*
  Various communication and account settings/data
*/

String          float_hub_id;			// default: factoryX
byte            float_hub_aes_key[16]; 
bool	        currently_connected = false;
bool	        cellular_connected = false;
bool	        console_mode;
unsigned long   boot_counter;			
boolean         led_state = false;              //  For cycling on and off  
bool		user_reset_pin_state;		//  Measured state of user reset pin; 
unsigned long	user_reset_pin_timestamp = 0;	//  Last time the user rest pin was initially pushed in
int		send_message_failures = 0;	//  Number of times we can fail to send something to ESP8266 before we dual reboot
float           speed_threshold = 1.75;
byte		stationary_interval = 10;
#define         OUTLIER_VECTOR_SIZE 13
float           speed_vector[OUTLIER_VECTOR_SIZE];



/*
   Hardware pins and constant values
*/
#define		ESP8266_SERIAL_READY_PIN 30	//  ESP8266 sets this HIGH to say it is ready for more serial data
#define		ESP8266_RESET_PIN  31		//  Pin we drive to ground to reset the esp8266 programmatically from here
#define         USER_RESET_PIN    4		//  Pin that signals when user is holding down the reset button
#define         USER_RESET_REBOOT_TIME 5000	//  Hold down reset pin for 5 seconds to make device reboot
#define         USER_RESET_FACTORY_TIME 20000	//  Hold down reset pin for 20 seconds to make device factory reset
#define		MAX_SEND_MESSAGE_FAILURES 5
//#define		VOLTAGE_DIVIDER 37.213 		//  Use to get from AnalogRead() to voltage estimate
#define		VOLTAGE_DIVIDER 16.123 		//  Use to get from AnalogRead() to voltage estimate



/*
  Status LED's (pins)
*/

#define COM_LED_1   2
#define COM_LED_2   3

#define GPS_LED_1   5
#define GPS_LED_2   6


/*
   Some global Strings, character arrays
*/

#define MAX_LATEST_MESSAGE_SIZE 1024
String latest_message_to_send = "";
String a_string = "";
String b_string = "";
char   temp_string[20];  
String mac_address;


/*
  Handy variables to use at various stages (better to be global, less memory)
*/

byte 	byte_zero, byte_one, byte_two, byte_three;
int  	handy;
float	float_one;
unsigned int i;


/*
    Overall timing parameters
*/

unsigned long sensor_sample_interval = 10000;     	//  Check temperature, pressure, every 10 seconds
unsigned long gps_interval = 50;                  	//  Read GPS serial every 1/20 of a second
unsigned long voltage_interval = 5000;            	//  Check batteries/chargers every 5 second
unsigned long pump_interval = 400;               	//  Check pump state every 400 milliseconds
unsigned long active_reporting_interval = 30000;  	//  When in use, report data every 30 seconds
//unsigned long idle_reporting_interval = 10000;   	//  Stress testing during development
//unsigned long idle_reporting_interval = 600000;   	//  When idle, report data every 10 minutes
//unsigned long idle_reporting_interval = 30000;   	//  Demoboat only every 30 seconds
unsigned long console_reporting_interval = 5000;  	//  Report to USB console every 5 seconds  
unsigned long console_interval = 250;             	//  Check console for input every 250 milliseconds
unsigned long esp8266_interval = 300;			//  Check for input from esp8266 on Serial 7 3ish times per second 
unsigned long led_update_interval = 200;          	//  Update the LED's every 200 miliseconds
unsigned long nmea_update_interval = 100;         	//  Update NMEA serial-in line every 1/10 of a second
unsigned long hsnmea_update_interval = 10;		//  Update HS NMEA (SoftwareSerial based) 100 times every second
unsigned long hardware_watchdog_interval = 120; 	//  Do a hardware reset if we don't pat the dog every 2 minutes
unsigned long nmea_sample_interval = 30000;		//  Nuke nmea data older than 30 seconds
#ifdef N2K_CODE_ON
//unsigned long n2k_interval = 1;			        //  Read N2K data 500 times a second
unsigned long n2k_output_interval = 300;	        //  Output n2k data (as 0183) once every 1/3 of a second
unsigned long n2k_interval = 10;			//  Read N2K data 100 times a second
//unsigned long n2k_interval = 2;			//  Read N2K data 10 times a second
//unsigned long n2k_interval = 1000;			//  Read N2K data 10 times a second
//unsigned long n2k_interval = 5000;			//  Read N2K data 10 times a second
#endif 
  
  
  
unsigned long sensor_previous_timestamp = 0;
unsigned long gps_previous_timestamp = 0;
unsigned long voltage_previous_timestamp = 0;
unsigned long pump_previous_timestamp = 0;
unsigned long previous_active_timestamp = 0;
unsigned long previous_idle_timestamp = 0;
unsigned long previous_console_timestamp = 0; 
unsigned long console_previous_timestamp = 0;
unsigned long esp8266_previous_timestamp =0;
unsigned long led_previous_timestamp = 0;
unsigned long nmea_previous_timestamp = 0;
unsigned long hsnmea_previous_timestamp = 0;
unsigned long nmea_speed_water_timestamp = 0;
unsigned long nmea_depth_water_timestamp = 0;
unsigned long nmea_wind_speed_timestamp = 0;
unsigned long nmea_wind_direction_timestamp = 0;
unsigned long nmea_wind_speed_abs_timestamp = 0;
unsigned long nmea_wind_direction_abs_timestamp = 0;
unsigned long nmea_water_temperature_timestamp = 0;
unsigned long nmea_heading_magnetic_timestamp = 0;
unsigned long nmea_heading_true_timestamp = 0;
unsigned long nmea_gga_timestamp = 0;
unsigned long nmea_rmc_timestamp = 0;
#ifdef N2K_CODE_ON
unsigned long n2k_previous_timestamp = 0; 
unsigned long n2k_output_previous_timestamp = 0; 
#endif 



/*
  Is the device currently "active" (i.e. is the vessel in movement and sending high frequency updates)?
*/
  
bool currently_active = true;


/*
   Baro and temp stuff
*/


#if BARO_HWARE == BARO_HWARE_BMP280
  Adafruit_BMP280 bhware;
#elif BARO_HWARE == BARO_HWARE_BME280
  Adafruit_BME280 bhware;
#elif BARO_HWARE == BARO_HWARE_BME680
  Adafruit_BME680 bhware;
#elif BARO_HWARE == BARO_HWARE_BMP180
  Adafruit_BMP085 bhware;
#endif

#define TEMPERATURE_BIAS 15	//  degrees F that BMP, on average, over reports temperature by
float temperature;
float pressure;
float temperature_vector[OUTLIER_VECTOR_SIZE];



/*
  Some global variables used in parsing (e.g. from the GPS module)
*/

#define	MAX_CONSOLE_BUFFER  255
#define	MAX_ESP8266_BUFFER  2048
#define MAX_GPS_BUFFER	    100
#define	MAX_NMEA_BUFFER	    100

String console_read_buffer;
String esp8266_read_buffer;
String gps_read_buffer;
String nmea_read_buffer;
String hsnmea_read_buffer;
byte nmea_cycle;

bool           gps_valid = false;
String         gps_utc = "";          //  UTC time and date
unsigned long  gps_utc_unix = 0;
String         gps_latitude = "";
String         gps_longitude = "";
String         gps_sog = "";          //  Speed over ground
String         gps_bearing_true = ""; //  Not magnetic!
String         gps_siv = "";          //  Number of satellites in view
String         gps_hdp = "";          //  Horizontal dillution of precision (how good is our fix)
String         gps_altitude = "";     //  We should be able to build a tide table out of this.


/*
  Global variables for battery banks/chargers/pumps
*/

float  battery_one;
float  battery_two;
float  battery_three;

float  charger_one;
float  charger_two;
float  charger_three;

bool pump_one_state = false;
bool pump_two_state = false;
bool pump_three_state = false;


/*
  Global variables for NMEA (0183) data
*/

float	nmea_speed_water = -1.0;	// Speed through water in knots, < 0 means invalid/no reading (do not report)
float	nmea_depth_water = -1.0;	// Depth of water below transducer, < 0 means invalid/not available
float	nmea_wind_speed = -1.0;		// Speed of true wind in knots, < 0 invalid/not available (apparent)
float	nmea_wind_direction = -1.0;	// Angle of true wind in degrees, < 0 invalid/not available (apparent)
float	nmea_wind_speed_abs = -1.0; 	// Speed of true wind in knots, < 0 invalid/not available (absolute)
float	nmea_wind_direction_abs = -1.0;	// Angle of true wind in degrees, < 0 invalid/not available (absolute)
float 	nmea_water_temperature = -1.0;	// Temperature of water in _FARENHEIT_, < 0 invalid/not available
float 	nmea_heading_true = -1.0;	// Temperature of water in _FARENHEIT_, < 0 invalid/not available
float 	nmea_heading_magnetic = -1.0;	// Temperature of water in _FARENHEIT_, < 0 invalid/not available




/*
  Setup for barometric and temperature
*/

void bhware_setup()
{
  if(!bhware.begin())
  {
    #ifdef BARO_DEBUG_ON
    debug_info("Failed !!! to initialize BARO HARDWARE");
    #endif
  }
  #if BARO_HWARE == BARO_HWARE_BME680
    bhware.setTemperatureOversampling(BME680_OS_8X);
    bhware.setHumidityOversampling(BME680_OS_2X);
    bhware.setPressureOversampling(BME680_OS_4X);
    bhware.setIIRFilterSize(BME680_FILTER_SIZE_3);
  #endif

}



void parse_esp8266()
{

  if(esp8266_read_buffer.startsWith(F("$FHI")))
  {
    if(esp8266_read_buffer.length() == 23 && esp8266_read_buffer.substring(20).startsWith(F("c=")))
    {
      if(esp8266_read_buffer.charAt(22) == '1')
      {
        currently_connected = true;
      }
      else
      {
        currently_connected = false;
      }
    }
    if(esp8266_read_buffer.length() == 23 && esp8266_read_buffer.substring(20).startsWith(F("d=")))
    {
      if(esp8266_read_buffer.charAt(22) == '1')
      {
        cellular_connected = true;
      }
      else
      {
        cellular_connected = false;
      }
    }
    else if(esp8266_read_buffer.length() >= 30 && esp8266_read_buffer.substring(20).startsWith(F("i=")))
    {
      a_string = esp8266_read_buffer.substring(22,30);
      if(a_string != float_hub_id)
      {
        float_hub_id = a_string;
        write_eeprom_memory();
      }
    }
    else if(esp8266_read_buffer.length() >= 25 && esp8266_read_buffer.substring(20).startsWith(F("t=")))
    {
      a_string = esp8266_read_buffer.substring(22);
      float_one = a_string.toFloat();
      if(float_one >= 0.0 && float_one <= 99.99 && speed_threshold != float_one)
      {
        speed_threshold = float_one;
      }
    }
    else if(esp8266_read_buffer.length() >= 23 && esp8266_read_buffer.substring(20).startsWith(F("T=")))
    {
      a_string = esp8266_read_buffer.substring(22);
      byte_one = a_string.toInt();
      if(byte_one >= 1 && byte_one <= 240 && stationary_interval != byte_one)
      {
        stationary_interval = byte_one;
      }
    }
    else if(esp8266_read_buffer.length() >= 54 && esp8266_read_buffer.substring(20).startsWith(F("k=")))
    {
      char a_char;
      bool something_changed = false;
      a_string = esp8266_read_buffer.substring(22,54);

      for(i = 0; i < 16; i++)
      {
        int new_value = 0;
        a_char = a_string.charAt(i * 2); 
        if (a_char <='9')
        {
          new_value = (a_char - '0' ) * 16;
        }
        else
        {
          new_value = (a_char - 'a' + 10) * 16;
        }
    
        a_char = a_string.charAt(1 + (i * 2));
        if (a_char <='9')
        {
          new_value += a_char - '0';
        }
        else
        {
          new_value += a_char - 'a' + 10;
        }

        if(float_hub_aes_key[i] != new_value)
        {
          something_changed = true;
          float_hub_aes_key[i] = new_value;
        }
      }
      if(something_changed)
      {
        write_eeprom_memory();
      }
    }
    else if(esp8266_read_buffer.length() >= 34 && esp8266_read_buffer.substring(20).startsWith(F("m=")))
    { 
      mac_address = esp8266_read_buffer.substring(22,34);
    }
  }
  else
  {
    //
    // Otherwise, just show it on the console
    //

    Serial.println(esp8266_read_buffer);
  }
}  



void parse_console()
{
  //
  // We could evesdrop here if we wanted to, but currently do not other than
  // to check for "cons", "nmea", of "factory" message.  Push all other
  // input up to the esp8266 on Serial7.
  //

  if(console_read_buffer == "cons")
  {
    console_mode = true;
  }
  else if(console_read_buffer == "nmea")
  {
    console_mode = false;
  }
  else if(console_read_buffer == "test")
  {
    resetESP();
  }
  else if(console_read_buffer == "factory")
  {
    factoryReset();
  }
  else
  {
    Serial7.println(console_read_buffer);
  }
}  

void esp8266_read()
{
  while(Serial7.available() && (int) esp8266_read_buffer.length() < MAX_ESP8266_BUFFER)
  {
     int incoming_byte = Serial7.read();
     if(incoming_byte == '\r')
     {
       parse_esp8266();
       esp8266_read_buffer = "";
     }
     else if (incoming_byte == '\n')
     {
       // don't do anything
     }
     else
     {
       esp8266_read_buffer += String((char) incoming_byte);
     }
  }
  if((int) esp8266_read_buffer.length() >= MAX_ESP8266_BUFFER - 1 )
  {
    esp8266_read_buffer = "";
  }
}




void console_read()
{

  while(Serial.available() && (int) console_read_buffer.length() < MAX_CONSOLE_BUFFER)
  {
     int incoming_byte = Serial.read();
     if(incoming_byte == '\r')
     {
       parse_console();
       console_read_buffer = "";
     }
     else if (incoming_byte == '\n')
     {
       // don't do anything
     }
     else
     {
       console_read_buffer += String((char) incoming_byte);
     }
  }
  if((int) console_read_buffer.length() >= MAX_CONSOLE_BUFFER - 1 )
  {
    console_read_buffer = "";
  }
}


bool esp8266IsReady()
{
  if(digitalRead(ESP8266_SERIAL_READY_PIN) == HIGH)
  {
    return true;
  }
  return false;
}


void add_checksum_and_send_nmea_string(String nmea_string)
{
  byte_zero = 0;
  for(int j = nmea_string.indexOf('$') + 1; j < nmea_string.lastIndexOf('*'); j++)
  {
    byte_zero = byte_zero ^ nmea_string.charAt(j);
  }

  String checksum = String(byte_zero, HEX); 
  checksum.toUpperCase();
  if(checksum.length() < 2)
  {
    nmea_string += F("0");
  }
  nmea_string += checksum;

  if(console_mode == false)
  {
    Serial.println(nmea_string);
  }
  Serial8.println(nmea_string);

  if(esp8266IsReady())
  {
    Serial7.print(F("E="));
    Serial7.println(nmea_string);
  }
  #ifdef SERIAL_DEBUG_ON
  else
  {
    debug_info(F("XA NMEA"));
  }
  #endif
}


void bhware_read()
{

  temperature = (1.8 * bhware.readTemperature()) + 32 - TEMPERATURE_BIAS ;
  pressure = bhware.readPressure() * 0.000295300;  

  //
  //  On some boats this temp/pressure chip seems to get "stuck".  It is
  //  probably a low voltage issue.  This checks if readings are static and
  //  tries to "reboot" the chip if they are.
  //

  bool all_same = true;
  for(i=0; i< OUTLIER_VECTOR_SIZE - 1; i++)
  {
      temperature_vector[i] = temperature_vector[i+1];
      if (temperature != temperature_vector[i])
      {
        all_same = false;
      }
  }
  if(all_same)
  {
    bhware_setup();
  }
  
  temperature_vector[OUTLIER_VECTOR_SIZE - 1] = temperature;

  //
  //	Output Temperature and Pressure as NMEA sentences in case anyone is listening
  //

  //  NMEA tags _can_ be like this in the old version of the protocol:
  //
  // 	Barometer
  //
  //	  MMB 
  //
  // 	    $--MMB,x.x,I,y.y,B*hh<CR><LF>
  //
  //		x.x Barometric pressure, inches of mercury
  // 		y.y Barometric pressure, bars
  //
  //    Air (Abmient) Temperature	
  //
  //	  MTA 
  //
  //        $--MTA,x.x,C*hh<CR><LF>
  //
  //	      x.x Temperature, degrees C
  //
  //
  //  And there's the combined MDA, which does pressure and air temp (and a bunch of other things)	
  //
  //  But "right way" now is apparently XDR like this:
  //
  //    $IIXDR,C,24.24,C,ENV_OUTSIDE_T,P,100700,P,ENV_ATMOS_P,H,64.356,P,ENV_OUTSIDE_H*37
  //           
  //

  
  //
  // We round robin these to not flood the NMEA out channel with realtively minor info
  //

  //
  //
  //	So first the MTA Air Temperature
  //
  //
  

  if(nmea_cycle == 0)
  {
    a_string = String(F("$IIMTA,")) 
             + String(temperature,2) 
             + String(F(",F*"));

    add_checksum_and_send_nmea_string(a_string);
  }

  //
  //
  //	Now the MDA combined
  //
  //

  else if(nmea_cycle == 1)
  {
    a_string = String(F("$IIMDA,")) 
             + String(pressure,3)
             + String(F(",I,"))
             + String(pressure * 0.03386388158, 6)
    	     + String(F(",B,"))
             + String((temperature - 32.0) * (5.0 / 9.0),2)
             + String(F(",C,,,,,,,,,,,,,,*"));

    add_checksum_and_send_nmea_string(a_string);   
  }
  
  //
  // 	Then the old school MMB for Air Pressure
  //

  else if(nmea_cycle == 2)
  {
    a_string = String(F("$IIMMB,"))
             + String(pressure,3)
             + String(F(",I,"))
             + String(pressure * 0.03386388158, 6)
             + String(F(",B*"));

    add_checksum_and_send_nmea_string(a_string);  
  }

  //
  // 	Finally new school XDR (Type, Data, Units, ID)
  //

  else if(nmea_cycle == 3)
  {
    a_string = String(F("$IIXDR,C,"))
             + String((temperature - 32.0) * (5.0 / 9.0), 2)
             + String(F(",C,TempAir,P,"))
             + String(pressure * 0.03386388158, 6)
             + String(F(",B,Barometric*"));

    add_checksum_and_send_nmea_string(a_string);  
  }

  nmea_cycle += 1;
  if(nmea_cycle > 3)
  {
    nmea_cycle = 0;
  }
}



int my_compare_function (const void * arg1, const void * arg2)
{
  float * a = (float *) arg1;  // cast to pointers to integers
  float * b = (float *) arg2;

  if (*a < *b)
  {
    return -1;
  }

  else if (*a > *b)
  {
    return 1;
  }
  return 0;
} 

void parse_gps_buffer_as_rmc()
{

  int time_start = 6;
  int time_break = gps_read_buffer.indexOf('.', time_start + 1);
  int status_start = gps_read_buffer.indexOf(',', time_start + 1);
  int lat_start =  gps_read_buffer.indexOf(',', status_start + 1);
  int nors_start =  gps_read_buffer.indexOf(',', lat_start + 1);
  int lon_start =  gps_read_buffer.indexOf(',', nors_start + 1);
  int wore_start = gps_read_buffer.indexOf(',', lon_start + 1);
  int sog_start = gps_read_buffer.indexOf(',', wore_start + 1);
  int tmg_start = gps_read_buffer.indexOf(',', sog_start + 1); 
  int date_start = gps_read_buffer.indexOf(',', tmg_start + 1);
  int var_start =  gps_read_buffer.indexOf(',', date_start + 1);

  //
  // Some (external!) GPS devices don't have a decimal point in the time
  // value, so we have to check for that here
  //
  if(time_break > status_start)
  {
    time_break = status_start;
  }

  
  if( time_start < 0    ||    time_start >= (int) gps_read_buffer.length()    ||
      time_break < 0    ||    time_break >= (int) gps_read_buffer.length()    ||
      status_start < 0  ||    status_start >= (int) gps_read_buffer.length()  ||
      lat_start < 0     ||    lat_start >= (int) gps_read_buffer.length()     ||
      nors_start < 0    ||    nors_start >= (int) gps_read_buffer.length()    ||
      lon_start < 0     ||    lon_start >= (int) gps_read_buffer.length()     ||
      wore_start < 0    ||    wore_start >= (int) gps_read_buffer.length()    ||
      sog_start < 0     ||    sog_start >= (int) gps_read_buffer.length()     ||
      tmg_start < 0     ||    tmg_start >= (int) gps_read_buffer.length()     ||
      date_start < 0    ||    date_start >= (int) gps_read_buffer.length()    ||
      var_start < 0     ||    var_start >= (int) gps_read_buffer.length()     ||
      gps_read_buffer.lastIndexOf('$') > 0)
  {
      #ifdef GPS_DEBUG_ON
      debug_info(F("Bad RMC string"));
      debug_info(gps_read_buffer);
      #endif

      #ifdef ACTIVE_DEBUG_ON
      debug_info(F("active: OFF BRMC"));
      #endif
      currently_active = false;


      return;
  }
     
  if (status_start  == time_start + 1 ||
      lat_start == status_start + 1 ||
      nors_start == lat_start + 1 ||
      lon_start == nors_start + 1 ||
      wore_start == lon_start + 1 ||
      //sog_start == wore_start + 1 ||
      //tmg_start == sog_start + 1 ||
      //date_start == tmg_start + 1 ||
      var_start == date_start + 1)
  

  {
      #ifdef GPS_DEBUG_ON
      debug_info(F("Empty RMC string"));
      debug_info(gps_read_buffer);
      #endif

      #ifdef ACTIVE_DEBUG_ON
      debug_info(F("active: OFF BRMD"));
      #endif
      currently_active = false;

      return;
  }
      
     
  String is_valid = gps_read_buffer.substring(status_start + 1, lat_start);
  
  /*
    Break GPS time/date into integer parts
  */

  gps_utc  = gps_read_buffer.substring(time_start + 1, time_break);
  gps_utc += gps_read_buffer.substring(date_start + 1, var_start - 2);
  gps_utc += F("20");
  gps_utc += gps_read_buffer.substring(var_start - 2, var_start);

  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(0,2).toCharArray(temp_string, 3);
  int  gps_time_hour = atoi(temp_string);
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(2,4).toCharArray(temp_string, 3);
  int  gps_time_minute = atoi(temp_string);

  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(4,6).toCharArray(temp_string, 3);
  int  gps_time_second = atoi(temp_string);
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(6,8).toCharArray(temp_string, 3);  
  int  gps_time_day = atoi(temp_string);
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(8,10).toCharArray(temp_string, 3);
  int  gps_time_month = atoi(temp_string);

  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(10,14).toCharArray(temp_string, 5);
  int  gps_time_year = atoi(temp_string);
  
  /*
    As long as the year is "reasonable", assume we have something close to right time
  */

  if(gps_time_year > 2010 && is_valid == "A")
  {
    setTime(gps_time_hour,gps_time_minute,gps_time_second,gps_time_day,gps_time_month,gps_time_year);
    gps_utc_unix = now();
    if(!gps_valid) 
    {
      #ifdef GPS_DEBUG_ON
      debug_info(F("Gps fix"));
      #endif
    }
    gps_valid = true;
  }
  else
  {
    if(gps_valid)
    {
      #ifdef GPS_DEBUG_ON
      debug_info(F("Gps no fix"));
      #endif
    }
    gps_valid = false;
  }
  

  //
  //  Use running average of GPS positions to see if we are moving (active)
  //
 

  if(gps_valid)
  {
    if(tmg_start > sog_start + 1)
    {
      gps_sog  = gps_read_buffer.substring(sog_start + 1, tmg_start);
    }
    else
    {
      gps_sog = "";
    }

    if(date_start > tmg_start + 1)
    {
      gps_bearing_true  = gps_read_buffer.substring(tmg_start + 1, date_start);
    }
    else
    {
      gps_bearing_true = "";
    }

    //
    //   Use speed over gound to figure out if active.  Used to have some
    // really complicated math in here for running average of latitude and
    // longitude, then had point observation, which was ok.  Now doing
    // median of 5 most recent values (minimal outlier filter)
    //

    memset(temp_string, 0, 20 * sizeof(char));
    gps_sog.substring(0,gps_sog.length()).toCharArray(temp_string, 19);
    float_one = atof(temp_string);
    
    //
    // Add current observation to the speed vector and make a copy
    //
    float sorted_vector[OUTLIER_VECTOR_SIZE];
    for(i=0; i< OUTLIER_VECTOR_SIZE - 1 ; i++)
    {
      speed_vector[i] = speed_vector[i+1];
      sorted_vector[i] = speed_vector[i];
    }
    speed_vector[OUTLIER_VECTOR_SIZE - 1] = float_one;
    sorted_vector[OUTLIER_VECTOR_SIZE - 1] = float_one;

    //
    // Sort copy
    //
    
    qsort (sorted_vector, OUTLIER_VECTOR_SIZE, sizeof (float), my_compare_function);
    
    //
    //  Use *MEDIAN* speed value to determine if we are "active"
    //
  
    if(sorted_vector[(OUTLIER_VECTOR_SIZE / 2) + 1 ] > speed_threshold)  
    {
      #ifdef ACTIVE_DEBUG_ON
      debug_info(F("active: ON "), float_one);
      #endif
      currently_active = true;
    }
    else
    {
      #ifdef ACTIVE_DEBUG_ON
      debug_info(F("active: OFF "), float_one);
      #endif
      currently_active = false;
    }
  }
  else  
  {    
    #ifdef ACTIVE_DEBUG_ON
    debug_info(F("active: OFF BGPS"));
    #endif
    currently_active = false;
  }
 
}


void parse_gps_buffer_as_gga()
{

  int time_start = 6;
  int lat_start =  gps_read_buffer.indexOf(',', time_start + 1);
  int nors_start =  gps_read_buffer.indexOf(',', lat_start + 1);
  int lon_start =  gps_read_buffer.indexOf(',', nors_start + 1);
  int wore_start = gps_read_buffer.indexOf(',', lon_start + 1);
  int qual_start = gps_read_buffer.indexOf(',', wore_start + 1);
  int siv_start = gps_read_buffer.indexOf(',', qual_start + 1); 
  int hdp_start = gps_read_buffer.indexOf(',', siv_start + 1);
  int alt_start =  gps_read_buffer.indexOf(',', hdp_start + 1);
  int altu_start = gps_read_buffer.indexOf(',', alt_start + 1);
     
  if( time_start < 0 ||   time_start >= (int) gps_read_buffer.length()   ||
      lat_start < 0  ||   lat_start >= (int) gps_read_buffer.length()    ||
      nors_start < 0 ||   nors_start >= (int) gps_read_buffer.length()   ||
      lon_start < 0  ||   lon_start >= (int) gps_read_buffer.length()    ||
      wore_start < 0 ||   wore_start >= (int) gps_read_buffer.length()   ||
      qual_start < 0 ||   qual_start >= (int) gps_read_buffer.length()   ||
      siv_start < 0  ||   siv_start >= (int) gps_read_buffer.length()    ||
      hdp_start < 0  ||   hdp_start >= (int) gps_read_buffer.length()    ||
      alt_start < 0  ||   alt_start >= (int) gps_read_buffer.length()    ||
      altu_start < 0 ||   altu_start >= (int) gps_read_buffer.length()   ||
      gps_read_buffer.lastIndexOf('$') > 0)
  {
      gps_valid = false;
      #ifdef GPS_DEBUG_ON
      debug_info(F("Bad GGA string"));
      debug_info(gps_read_buffer);
      #endif
      return;
  }
  
  if (lat_start  == time_start + 1 ||
      nors_start == lat_start + 1 ||
      lon_start  == nors_start + 1 ||
      wore_start == lon_start + 1 ||
      qual_start == wore_start + 1 ||
      siv_start  == qual_start + 1 ||
      hdp_start  == siv_start + 1 ||
      alt_start  == hdp_start + 1 ||
      altu_start == alt_start + 1)
  {
      gps_valid = false;
      #ifdef GPS_DEBUG_ON
      debug_info(F("Empty GGA string"));
      debug_info(gps_read_buffer);
      #endif
      return;
  }
      

  gps_latitude  = gps_read_buffer.substring(lat_start + 1, lat_start + 3);
  gps_latitude += F(" ");
  gps_latitude += gps_read_buffer.substring(lat_start + 3, nors_start);
  while(gps_latitude.length() < 11)
  {
    gps_latitude += F("0");
  }
  gps_latitude += gps_read_buffer.substring(nors_start + 1, lon_start);

  gps_longitude  = gps_read_buffer.substring(lon_start + 1, lon_start + 4);
  gps_longitude += F(" ");
  gps_longitude += gps_read_buffer.substring(lon_start + 4, wore_start);
  while(gps_longitude.length() < 12)
  {
    gps_longitude += F("0");
  }
  gps_longitude += gps_read_buffer.substring(wore_start + 1, qual_start);

  gps_siv  = gps_read_buffer.substring(siv_start + 1, hdp_start);
  gps_hdp  = gps_read_buffer.substring(hdp_start + 1, alt_start);
  gps_altitude  = gps_read_buffer.substring(alt_start + 1, altu_start);
    
  //
  //	Extra error checking here to make sure things are valid
  //
  
  if(gps_altitude.length() < 1 ||
     gps_hdp.length() < 1)
  {
    gps_valid = false;
  }
  
}

void push_hsnmea_only_to_esp8266()
{
  if(console_mode == false)
  {
    Serial.println(hsnmea_read_buffer);
  }
  if(!esp8266IsReady())
  {
    #ifdef SERIAL_DEBUG_ON
    debug_info(F("XC NMEA"));
    #endif
    return;
  }
  Serial7.print(F("E="));
  Serial7.println(hsnmea_read_buffer);
}


void push_out_nmea_sentence(bool from_nmea_in)
{
  if(!esp8266IsReady())
  {
    #ifdef SERIAL_DEBUG_ON
    debug_info(F("XB NMEA"));
    #endif
  }
  else
  {
    if(from_nmea_in)
    {  
      Serial7.print(F("E="));
      Serial7.println(nmea_read_buffer);
    }
    else
    {
      Serial7.print(F("E="));
      Serial7.println(gps_read_buffer);
    }
  }

  if(from_nmea_in)
  {
    if(console_mode == false)
    {
      Serial.println(nmea_read_buffer);
    }
    Serial8.println(nmea_read_buffer);
  }
  else
  {
    if(console_mode == false)
    {
      Serial.println(gps_read_buffer);
    }
    Serial8.println(gps_read_buffer);
  }
}


bool validate_gps_buffer()
{
  byte_zero = 0;
  for(int j = gps_read_buffer.indexOf('$') + 1; j < gps_read_buffer.lastIndexOf('*'); j++)
  {
    byte_zero = byte_zero ^ gps_read_buffer.charAt(j);
  }
  a_string = String(byte_zero, HEX);
  a_string.toUpperCase();
  if(gps_read_buffer.endsWith(a_string))
  {
    return true;
  }

  #ifdef GPS_DEBUG_ON
  debug_info(String(F("Bad GPS buffer: ")) + gps_read_buffer);
  #endif

  return false;  
}


bool validate_and_maybe_remediate_gps_buffer()
{
  if(validate_gps_buffer())
  {
    return true;
  }
  if(gps_read_buffer.lastIndexOf('$') > 0)
  {
    gps_read_buffer = gps_read_buffer.substring(gps_read_buffer.lastIndexOf('$')); 
    return validate_gps_buffer();  
  }
  return false;
}


bool validate_nmea_buffer(bool hsnmea = false)
{
  byte_zero = 0;

  if(hsnmea)
  {
    int start_point = hsnmea_read_buffer.indexOf('!');
    if(start_point < 0)
    {
      start_point = hsnmea_read_buffer.indexOf('$');
    }
    for(int j = start_point + 1; j < hsnmea_read_buffer.lastIndexOf('*'); j++)
    {
      byte_zero = byte_zero ^ hsnmea_read_buffer.charAt(j);
    }
  }
  else
  {
    for(int j = nmea_read_buffer.indexOf('$') + 1; j < nmea_read_buffer.lastIndexOf('*'); j++)
    {
      byte_zero = byte_zero ^ nmea_read_buffer.charAt(j);
    }
  }
  a_string = String(byte_zero, HEX);
  a_string.toUpperCase();
  if(hsnmea)
  {
    if(hsnmea_read_buffer.endsWith(a_string))
    {
      return true;
    }
  }
  else
  {
    if(nmea_read_buffer.endsWith(a_string))
    {
      return true;
    }
  }
  #ifdef SOFTSERIAL_DEBUG_ON
  if(hsnmea)
  {
    debug_info(String(F("Bad HSNMEA buffer: ")) + hsnmea_read_buffer + ", Wanted: " + a_string);
  }
  #endif
  #ifdef NMEA_DEBUG_ON
  if(!hsnmea)
  {
    debug_info(String(F("Bad NMEA buffer: ")) + nmea_read_buffer + ", Wanted: " + a_string);
  }
  #endif

  return false;  
}


void gps_read()
{

  while(Serial1.available() && (int) gps_read_buffer.length() < MAX_GPS_BUFFER)
  {
    int incoming_byte = Serial1.read();
    if(incoming_byte == '\n')
    {
      if(validate_and_maybe_remediate_gps_buffer())
      {
        if(gps_read_buffer.indexOf(F("$GPRMC,")) == 0)
        {
          #ifdef GPS_DEBUG_ON
          debug_info(F("--GPS BUF RMC--"));
          debug_info(gps_read_buffer);
          #endif
          if(millis() - nmea_rmc_timestamp > nmea_sample_interval || 
            (millis() < nmea_sample_interval && nmea_rmc_timestamp == 0))
	  {
            #ifdef GPS_SOURCE_DEBUG_ON
            debug_info(F("GPS RMC INTERNAL"));
            #endif
            #ifdef N2K_CODE_ON
            if(n2k_gps_valid == false)
            {
            #endif
              push_out_nmea_sentence(false);
              parse_gps_buffer_as_rmc(); 
            #ifdef N2K_CODE_ON
            }
            #endif
	  }
        }
        else if(gps_read_buffer.indexOf(F("$GPGGA,")) == 0)
        {
          #ifdef GPS_DEBUG_ON
          debug_info(F("--GPS BUF GGA--"));
          debug_info(gps_read_buffer);
          #endif
          if(millis() - nmea_gga_timestamp > nmea_sample_interval || 
            (millis() < nmea_sample_interval && nmea_gga_timestamp == 0))
	  {
            #ifdef GPS_SOURCE_DEBUG_ON
            debug_info(F("GPS GGA INTERNAL"));
            #endif
            #ifdef N2K_CODE_ON
            if(n2k_gps_valid == false)
            {
            #endif
              push_out_nmea_sentence(false);  
              parse_gps_buffer_as_gga();  
            #ifdef N2K_CODE_ON
            }
            #endif
	  }
        }
      }
      gps_read_buffer = "";
    }
    else if (incoming_byte == '\r')
    {
      // don't do anything
    }
    else
    {
      gps_read_buffer += String((char) incoming_byte);
    }
  }
  if((int) gps_read_buffer.length() >= MAX_GPS_BUFFER - 1 )
  {
    #ifdef GPS_DEBUG_ON
    debug_info(F("nuked gps buf"));
    #endif
    gps_read_buffer = "";
  }

}


void voltage_read()
{
  battery_one 	= analogRead(A15) / VOLTAGE_DIVIDER; 
  battery_two 	= analogRead(A16) / VOLTAGE_DIVIDER;
  battery_three = analogRead(A17) / VOLTAGE_DIVIDER;

  charger_one 	= analogRead(A0) / VOLTAGE_DIVIDER;
  charger_two 	= analogRead(A1) / VOLTAGE_DIVIDER;
  charger_three	= analogRead(A2) / VOLTAGE_DIVIDER;
}


void append_formatted_value(String &the_string, int value)
{
  if(value < 10)
  {
    the_string += F("0");
    the_string += value;
  }
  else
  {
    the_string += value;
  }
}


void add_timestamp_to_string(String &the_string)
{
  the_string += F(",U:");
  append_formatted_value(the_string, hour());
  append_formatted_value(the_string, minute());
  append_formatted_value(the_string, second());
  append_formatted_value(the_string, day());
  append_formatted_value(the_string, month());
  the_string += year();          
}


void individual_pump_read(int pump_number, bool &state, int analog_input)
{
  float pump_value = analogRead(analog_input) / VOLTAGE_DIVIDER;
  #ifdef PUMP_DEBUG_ON
  debug_info(F("Pump ") + String(pump_number) + F(" on input ") + String(analog_input) + F(" reads "), pump_value);
  #endif
  if(pump_value > 2.0)
  {
     if(state == false)
     {
       #ifdef PUMP_DEBUG_ON
       debug_info(F("Pump turned on!"));
       #endif
       latest_message_to_send = F("$FHB:");
       latest_message_to_send += float_hub_id;
       latest_message_to_send += F(":");
       latest_message_to_send += FLOATHUB_PROTOCOL_VERSION;
       latest_message_to_send += F("$");
       if(gps_valid || timeStatus() != timeNotSet)
       {
         add_timestamp_to_string(latest_message_to_send);
       }
       latest_message_to_send += F(",P");
       latest_message_to_send += pump_number;
       latest_message_to_send += F(":1");

       if(float_hub_id == "factoryX" and mac_address.length() == 12)
       {
         latest_message_to_send += F(",I:");
         latest_message_to_send += mac_address;
       }       

       send_message_to_esp8266();
       echo_info(latest_message_to_send);
       state = true;       
     }
  }
  else
  {
     if(state == true)
     {
       #ifdef PUMP_DEBUG_ON
       debug_info(F("Pump turned off!"));
       #endif
       latest_message_to_send = F("$FHB:");
       latest_message_to_send += float_hub_id;
       latest_message_to_send += F(":");
       latest_message_to_send += FLOATHUB_PROTOCOL_VERSION;
       latest_message_to_send += F("$");

       if(gps_valid || timeStatus() != timeNotSet)
       {
         add_timestamp_to_string(latest_message_to_send);
       }
       latest_message_to_send += F(",P");
       latest_message_to_send += pump_number;
       latest_message_to_send += F(":0");

       if(float_hub_id == "factoryX" and mac_address.length() == 12)
       {
         latest_message_to_send += F(",I:");
         latest_message_to_send += mac_address;
       }       

       send_message_to_esp8266();

       echo_info(latest_message_to_send);
       state = false;
     }
  }
}


void pump_read()
{
  individual_pump_read(1, pump_one_state, A3);
  individual_pump_read(2, pump_two_state, A6);
  individual_pump_read(3, pump_three_state, A7);
}



void append_float_to_string(String &the_string, float x)
{
  memset(temp_string, 0, 20 * sizeof(char));
  dtostrf(x,4,2,temp_string);
  the_string += String(temp_string);
}


void possibly_append_data(float value, float test, String tag)
{
  if(value > test)
  {
    latest_message_to_send += tag;
    append_float_to_string(latest_message_to_send, value);
  }
}


void report_state(bool console_only)
{
  if(console_only)
  {
    if(console_mode)
    {
      latest_message_to_send = F("$FHC:");
    }
    else
    {
      return;
    }
  }
  else
  {
    latest_message_to_send = F("$FHA:");
  }

  latest_message_to_send += float_hub_id;
  latest_message_to_send += F(":");
  latest_message_to_send += FLOATHUB_PROTOCOL_VERSION;
  latest_message_to_send += F("$");
  if(gps_valid == true || timeStatus() != timeNotSet )
  {
     add_timestamp_to_string(latest_message_to_send);
  }

  if(mac_address.length() == 12 && ( float_hub_id == "factoryX" || random(0, 100) < 3))
  {
    latest_message_to_send += F(",I:");
    latest_message_to_send += mac_address;
  }
  
  latest_message_to_send += F(",T:");
  append_float_to_string(latest_message_to_send, temperature);

  latest_message_to_send += F(",P:");
  append_float_to_string(latest_message_to_send, pressure);

  if(gps_valid == true && gps_altitude.length() > 0)
  {
    latest_message_to_send += F(",L:");
    latest_message_to_send += gps_latitude;
    
    latest_message_to_send += F(",O:");
    latest_message_to_send += gps_longitude;

    latest_message_to_send += F(",A:");
    latest_message_to_send += gps_altitude;

    latest_message_to_send += F(",H:");
    latest_message_to_send += gps_hdp;
    
    latest_message_to_send += F(",S:");
    latest_message_to_send += gps_sog;

    latest_message_to_send += F(",B:");
    latest_message_to_send += gps_bearing_true;
  }

  if(gps_siv.length() > 0)
  {
    latest_message_to_send += String(F(",N:"));
    latest_message_to_send += gps_siv;
  }

  possibly_append_data(battery_one, 1.0, F(",V1:"));
  possibly_append_data(battery_two, 1.0, F(",V2:"));
  possibly_append_data(battery_three, 1.0, F(",V3:"));

  possibly_append_data(charger_one, 1.0, F(",C1:"));
  possibly_append_data(charger_two, 1.0, F(",C2:"));
  possibly_append_data(charger_three, 1.0, F(",C3:"));

  //
  //	Add NMEA data
  //
  
  possibly_append_data(nmea_speed_water, -0.5, F(",R:"));
  possibly_append_data(nmea_depth_water, -0.5, F(",D:"));
  possibly_append_data(nmea_wind_speed, -0.5, F(",J:"));
  possibly_append_data(nmea_wind_direction, -0.5, F(",K:"));
  possibly_append_data(nmea_wind_speed_abs, -0.5, F(",W:"));
  possibly_append_data(nmea_wind_direction_abs, -0.5, F(",X:"));
  possibly_append_data(nmea_water_temperature, -0.5, F(",Y:"));
  possibly_append_data(nmea_heading_true, -0.5, F(",G:"));
  possibly_append_data(nmea_heading_magnetic, -0.5, F(",M:"));

  if(!console_only)
  {
    send_message_to_esp8266();
  }
  echo_info(latest_message_to_send);

}


void echo_info(String some_info)
{
  if(console_mode)
  {
    Serial.println(some_info);
  }
}


void debug_info_core(String some_info)
{
  some_info.replace('\n','|');
  some_info.replace('\r','|');

  Serial.print(F("$FHD:"));
  Serial.print(float_hub_id);
  Serial.print(F(":"));
  Serial.print(FLOATHUB_PROTOCOL_VERSION);
  Serial.print(F("$    "));
  //Serial.print(String(F("[")) + String(hour()) + F(":") + String(minute()) + F(":") + String(second()) + F("] "));
  Serial.print(some_info);

  Serial7.print(F("B=$FHD:"));
  Serial7.print(float_hub_id);
  Serial7.print(F(":"));
  Serial7.print(FLOATHUB_PROTOCOL_VERSION);
  Serial7.print(F("$    "));
  //Serial7.print(String(F("[")) + String(hour()) + F(":") + String(minute()) + F(":") + String(second()) + F("] "));
  Serial7.print(some_info);
}


void debug_info(String some_info)
{
  debug_info_core(some_info);
  Serial.println();
  Serial7.println();
}


void debug_info(String some_info, float x)
{
  debug_info_core(some_info);
  Serial.println(x);
  Serial7.println(x);
}


void debug_info(String some_info, int x)
{
  debug_info_core(some_info);
  Serial.println(x);
  Serial7.println(x);
}


void update_leds()
{
  if(user_reset_pin_state == HIGH)
  {
    unsigned long current_timestamp = millis();
    
    if(current_timestamp - user_reset_pin_timestamp < USER_RESET_REBOOT_TIME)
    {
      if(led_state)
      {
        digitalWrite(GPS_LED_1, LOW);
        digitalWrite(GPS_LED_2, HIGH);
        digitalWrite(COM_LED_1, LOW);
        digitalWrite(COM_LED_2, HIGH);
      }
      else
      {
        digitalWrite(GPS_LED_1, LOW);
        digitalWrite(GPS_LED_2, LOW);
        digitalWrite(COM_LED_1, LOW);
        digitalWrite(COM_LED_2, LOW);
      }
      led_state = !led_state;
    }
    else if (current_timestamp - user_reset_pin_timestamp > USER_RESET_REBOOT_TIME && current_timestamp - user_reset_pin_timestamp < USER_RESET_FACTORY_TIME)
    {
      if(led_state)
      {
        digitalWrite(GPS_LED_1, HIGH);
        digitalWrite(GPS_LED_2, LOW);
        digitalWrite(COM_LED_1, LOW);
        digitalWrite(COM_LED_2, HIGH);
      }
      else
      {
        digitalWrite(GPS_LED_1, LOW);
        digitalWrite(GPS_LED_2, HIGH);
        digitalWrite(COM_LED_1, HIGH);
        digitalWrite(COM_LED_2, LOW);
      }
      led_state = !led_state;
    }
    else 
    {
      if(led_state)
      {
        digitalWrite(GPS_LED_1, LOW);
        digitalWrite(GPS_LED_2, LOW);
        digitalWrite(COM_LED_1, LOW);
        digitalWrite(COM_LED_2, LOW);
      }
      else
      {
        digitalWrite(GPS_LED_1, HIGH);
        digitalWrite(GPS_LED_2, LOW);
        digitalWrite(COM_LED_1, HIGH);
        digitalWrite(COM_LED_2, LOW);
      }
      led_state = !led_state;
    }
    return;
  }

  if(n2k_gps_valid || 
     (millis() - nmea_gga_timestamp < nmea_sample_interval && nmea_gga_timestamp != 0))
  {
      digitalWrite(GPS_LED_1, LOW);
      digitalWrite(GPS_LED_2, LOW);
      if(random(0,100) < 25)
      {
        digitalWrite(GPS_LED_1, LOW);
        digitalWrite(GPS_LED_2, HIGH);
      }
  }
  else
  {
    if(gps_valid)
    {
      digitalWrite(GPS_LED_1, LOW);
      digitalWrite(GPS_LED_2, HIGH);
    }
    else
    {
      digitalWrite(GPS_LED_1, HIGH);
      digitalWrite(GPS_LED_2, LOW);
    }
  }



  if(currently_connected)
  {
    digitalWrite(COM_LED_1, LOW);
    digitalWrite(COM_LED_2, HIGH);
  } 
  else
  {
    if(cellular_connected)
    {
      digitalWrite(COM_LED_1, LOW);
      digitalWrite(COM_LED_2, LOW);
      if(random(0,100) < 25)
      {
        digitalWrite(COM_LED_1, LOW);
        digitalWrite(COM_LED_2, HIGH);
      }
    }
    else
    {
      digitalWrite(COM_LED_1, HIGH);
      digitalWrite(COM_LED_2, LOW);
    }
  } 
}


void gps_setup()
{
  //
  //  Setup gps on serial device 1, and make it send only GGA and RMC NMEA sentences
  //

  Serial1.begin(9600);
  delay(500);

  //
  //	Configure in case the GPS is a MTK3339 / Ultimate GPS breakout from Adafruit
  //

  Serial1.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));		// GGA & RMC every second

  //
  //   	Or a uBlox NEO-6M
  // 

  Serial1.println("$PUBX,40,GLL,0,0,0,0,0,0*5C");
  Serial1.println("$PUBX,40,GGA,0,1,0,0,0,0*5B"); 
  Serial1.println("$PUBX,40,GSA,0,0,0,0,0,0*4E");
  Serial1.println("$PUBX,40,RMC,0,1,0,0,0,0*46");
  Serial1.println("$PUBX,40,GSV,0,0,0,0,0,0*59");
  Serial1.println("$PUBX,40,VTG,0,0,0,0,0,0*5E");
  
  //
  //	Assuming a NEO-6M, try and set it to "at-sea" mode
  //

  uint8_t configure_gps_command[] = {
    0xB5, 0x62,			// Ublox "sync" characters
    0x06, 0x24, 		// Message class and ID (class is configuration, id is navigation confiuration 
    0x24, 0x00, 		// 0x24 = 36 bytes in length of rest of message
    0xFF, 0xFF, 		// Mask (all on = apply all settings that follow)
    0x06,			// Set Dynamic platform model to 6 ("at sea") 
    0x03, 			// Set fix mode to Auto 2D/3D
    0x00, 0x00, 0x00, 0x00,	// Fixed altitude for 2D Mode 
    0x10, 0x27, 0x00, 0x00, 	// Fixed altitude variance for 2D Mode 
    0x05,			// Min elevation for a satellite to be used 
    0x00,			// Max time to perform an extrapolation if GPS signal lost 
    0xFA, 0x00, 		// Position DOP Mask
    0xFA, 0x00, 		// Time DOP Mask
    0x64, 0x00, 		// Position Accuracy Mask
    0x2C, 0x01, 		// Time Accuracy Mask
    0x1A, 			// Static Hold Threshold
    0x00,			// DGPS timeout 
    0x00, 0x00, 0x00, 0x00, 	// Always zero
    0x00, 0x00, 0x00, 0x00, 	// Always zero
    0x00, 0x00, 0x00, 0x00, 	// Always zero
    0x30, 0x48 };
  
  for(i=0; i < sizeof(configure_gps_command)/sizeof(uint8_t); i++)
  {
    Serial1.write(configure_gps_command[i]);
  }
  Serial1.println();
}


void resetFunc()
{
  watchdog.reset();
}


void init_eeprom_memory()
{

  //
  //  This function is called only during factory reset or very first intial startup
  //
    
  //
  //  Set boot counter to 0
  //

  for(i = 6; i < 10; i++)
  {
    EEPROM.write(i,0);
  }
  
  //
  //  Set default id
  //
  
  EEPROM.write(10, 'f');
  EEPROM.write(11, 'a');
  EEPROM.write(12, 'c');
  EEPROM.write(13, 't');
  EEPROM.write(14, 'o');
  EEPROM.write(15, 'r');
  EEPROM.write(16, 'y');
  EEPROM.write(17, 'X');
  
  //
  //  Set to some kind of default AES key
  //
  
  EEPROM.write(18, 0x00);
  EEPROM.write(19, 0x01);
  EEPROM.write(20, 0x02);
  EEPROM.write(21, 0x03);
  EEPROM.write(22, 0x04);
  EEPROM.write(23, 0x05);
  EEPROM.write(24, 0x06);
  EEPROM.write(25, 0x07);
  EEPROM.write(26, 0x08);
  EEPROM.write(27, 0x09);
  EEPROM.write(28, 0x0A);
  EEPROM.write(29, 0x0B);
  EEPROM.write(30, 0x0C);
  EEPROM.write(31, 0x0D);
  EEPROM.write(32, 0x0E);
  EEPROM.write(33, 0x0F);


  //
  //  Do this last to show EEPROM set
  //

  for(i = 0; i < 6; i++) {
    EEPROM.write(i, 42); }
  
}


void write_eeprom_memory()
{

  //
  //  This just pushes current variables (e.g. float hub id) to EEPROM 
  //
    
  //
  //  Store id
  //
  
  for(i = 0; i < 8; i++)
  {
    EEPROM.write(10 + i, float_hub_id[i]);
  }
  
  //
  //  Store AES key
  //
  
  for(i = 0; i < 16; i++)
  {
    EEPROM.write(18 + i, float_hub_aes_key[i]);
  }
}


void read_eeprom_memory()
{
  char next_char;
  //
  //  As part of startup, set variables from non-volatile EEPROM
  //

  //
  //  Read, augment, the write back boot counter
  //

  boot_counter  = (unsigned int) EEPROM.read(9);
  boot_counter += (unsigned int) EEPROM.read(8) * 256;
  boot_counter += (unsigned int) EEPROM.read(7) * 65536;
  boot_counter += (unsigned int) EEPROM.read(6) * 16777216;

  boot_counter += 1;
  
  byte_zero   = byte(boot_counter);
  byte_one    = byte(boot_counter >> 8);
  byte_two    = byte(boot_counter >> 16);
  byte_three  = byte(boot_counter >> 24);
  
  EEPROM.write(9, byte_zero);
  EEPROM.write(8, byte_one);
  EEPROM.write(7, byte_two);
  EEPROM.write(6, byte_three);
  
  //
  //  Read floathub id
  //
  
  float_hub_id = "";
  for(i = 0; i < 8; i++)
  {
    next_char = EEPROM.read(10 + i);
    float_hub_id += next_char;
  }
  
  //
  //  Read AES key
  //
  
  for(i = 0; i < 16; i++)
  {
    float_hub_aes_key[i] = EEPROM.read(18 + i);
  }    
}  


bool popout_nmea_value(String data_type, int comma_begin, int comma_end, float &target, bool convert_ctof = false)
{
  if(nmea_read_buffer.substring(3,6) == data_type &&
     comma_begin > 0 &&
     comma_end > 0 &&
     comma_end > comma_begin)
  {
    if(comma_end - comma_begin > 1)
    {
      memset(temp_string, 0, 20 * sizeof(char));
      nmea_read_buffer.substring(comma_begin + 1, comma_end).toCharArray(temp_string, 9);
      target = atof(temp_string);
      if(convert_ctof)
      {
        target = (target * 1.8) + 32.0;	// FARENHEIT
      }
      return true;
    }
  }
  return false;
}


void parse_nmea_sentence()
{
  #ifdef NMEA_DEBUG_ON
  debug_info(String(F("NMEA buf:")) + nmea_read_buffer);
  #endif    

  int commas[8] = {-1, -1, -1, -1, -1, -1, -1 , -1};
  float temp_float = 0.0;

  if(nmea_read_buffer.length() < 10 	||
     nmea_read_buffer.indexOf('$') != 0	||
     nmea_read_buffer.indexOf('*') < 0)
  {
    return;
  }
  
  //
  //	Get an index of all comma locations in the sentence
  //   
  
  commas[0] = nmea_read_buffer.indexOf(',');
  for(i = 1; i < 8; i++)
  {
    if(commas[i - 1] < 0)
    {
      break;
    }
    commas[i] = nmea_read_buffer.indexOf(',', commas[i - 1] + 1);
  }

  //
  // If we have location data coming in on NMEA in, we will use that instead
  // of our onboard GPS
  //

  if(nmea_read_buffer.indexOf(F("GGA")) == 3)
  {
    #ifdef GPS_SOURCE_DEBUG_ON
    debug_info(F("GPS GGA EXTERNAL"));
    #endif
    b_string = gps_read_buffer;
    gps_read_buffer = nmea_read_buffer;
    parse_gps_buffer_as_gga();
    if(gps_valid)
    {
      nmea_gga_timestamp = millis();
    }
    gps_read_buffer = b_string; 
  }

  else if(nmea_read_buffer.indexOf(F("RMC")) == 3)
  {
    #ifdef GPS_SOURCE_DEBUG_ON
    debug_info(F("GPS RMC EXTERNAL"));
    #endif
    b_string = gps_read_buffer;
    gps_read_buffer = nmea_read_buffer;
    parse_gps_buffer_as_rmc();
    if(gps_valid)
    {
      nmea_rmc_timestamp = millis();
    }
    gps_read_buffer = b_string; 
  }

  //
  //	Depth below transducer
  //

  else if( popout_nmea_value(F("DBT"), commas[2], commas[3], nmea_depth_water))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA dbt water:"), nmea_depth_water);
    #endif    
    nmea_depth_water_timestamp = millis();
  }

  //
  //	Depth of water
  //

  else if( popout_nmea_value(F("DPT"), commas[0], commas[1], nmea_depth_water))
  {
    temp_float = 0.0;
    if( popout_nmea_value(F("DPT"), commas[1], commas[2], temp_float))
    {
      nmea_depth_water += temp_float;
    }
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA dpt water:"), nmea_depth_water);
    #endif
    nmea_depth_water_timestamp = millis();
  }

  //
  //	Mean Temperature of Water
  //


  else if( popout_nmea_value(F("MTW"), commas[0], commas[1], nmea_water_temperature, true))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA w temp:"), nmea_water_temperature);
    #endif    
    nmea_water_temperature_timestamp = millis();
  }

  //
  //	Speed Through Water
  //

  else if( popout_nmea_value(F("VHW"), commas[4], commas[5], nmea_speed_water))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA s water:"), nmea_speed_water);
    #endif    
    nmea_speed_water_timestamp = millis();
  }
  
  //
  //	Wind Speed and direction if speed works
  //
  
  else if( popout_nmea_value(F("MWV"), commas[2], commas[3], temp_float))
  {
    if( nmea_read_buffer[commas[1] + 1] == 'R' && nmea_read_buffer[commas[4] + 1] == 'A')
    {
      nmea_wind_speed = temp_float;
      char speed_units = nmea_read_buffer[commas[3] + 1];
      if(speed_units == 'K')
      {
        nmea_wind_speed = nmea_wind_speed * 0.539957;
      }
      else if(speed_units == 'M')
      {
        nmea_wind_speed = nmea_wind_speed * 1.94384;
      }
      else if(speed_units == 'S')
      {
        nmea_wind_speed = nmea_wind_speed * 0.868976;
      }

      #ifdef NMEA_DEBUG_ON
      debug_info(F("NMEA w speed:"), nmea_wind_speed);
      #endif
      nmea_wind_speed_timestamp = millis();
      if( popout_nmea_value(F("MWV"), commas[0], commas[1], nmea_wind_direction))
      {
        #ifdef NMEA_DEBUG_ON
        debug_info(F("NMEA w direction:"), nmea_wind_direction);
        #endif
        nmea_wind_direction_timestamp = millis();
      }
    }
  }

  //
  //	Absolute Wind Speed and direction 
  //
  
  else if(  popout_nmea_value(F("MWD"), commas[4], commas[5], nmea_wind_speed_abs))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA abw speed:"), nmea_wind_speed_abs);
    #endif
    nmea_wind_speed_abs_timestamp = millis();
    if(	popout_nmea_value(F("MWD"), commas[0], commas[1], nmea_wind_direction_abs))
    {
      #ifdef NMEA_DEBUG_ON
      debug_info(F("NMEA abw direction:"), nmea_wind_direction_abs);
      #endif
      nmea_wind_direction_abs_timestamp = millis();
    }
  }

  //
  //	Heading Magnetic and (possibly) True via HDG
  //

  else if(  popout_nmea_value(F("HDG"), commas[0], commas[1], nmea_heading_magnetic))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA h mag1:"), nmea_heading_magnetic);
    #endif
    nmea_heading_magnetic_timestamp = millis();
    float deviation = 0.0;
    float variation = 0.0;
    if ( popout_nmea_value(F("HDG"), commas[1], commas[2], deviation) ||
         popout_nmea_value(F("HDG"), commas[3], commas[4], variation)  )
    {
      if (nmea_read_buffer[commas[2] + 1] == 'W')
      {
        deviation = deviation * -1.0;
      }
      if (nmea_read_buffer[commas[4] + 1] == 'W')
      {
        variation = variation * -1.0;
      }

      nmea_heading_true = nmea_heading_magnetic + deviation + variation ;
      nmea_heading_true_timestamp = millis();
      #ifdef NMEA_DEBUG_ON
      debug_info(F("NMEA h tru1:"), nmea_heading_true);
      #endif

    }
  }

  //
  //	Heading Magnetic via HDM
  //

  else if(  popout_nmea_value(F("HDM"), commas[0], commas[1], nmea_heading_magnetic))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA h mag2:"), nmea_heading_magnetic);
    #endif
    nmea_heading_magnetic_timestamp = millis();
  }

  //
  //	Heading True via HDT
  //

  else if(  popout_nmea_value(F("HDT"), commas[0], commas[1], nmea_heading_true))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA h tru2:"), nmea_heading_true);
    #endif
    nmea_heading_true_timestamp = millis();
  }
}


void parse_hsnmea_sentence()
{

  //
  // We don't parse actual AIS !AIVDM type sentences, _but_ if we are
  // getting regular NMEA data over the HS port (probably from a
  // multiplexer), we will try and parse that by quickly swapping out the
  // regular NMEA buffer
  //

  if(hsnmea_read_buffer.indexOf('$') == 0)
  {
    a_string = nmea_read_buffer;
    nmea_read_buffer = hsnmea_read_buffer;
    parse_nmea_sentence();
    nmea_read_buffer = a_string; 
  }
}


void update_nmea()
{
  bool keep_going = true;
  while(Serial8.available() && (int) nmea_read_buffer.length() < MAX_NMEA_BUFFER && keep_going == true)
  {
     int incoming_byte = Serial8.read();
     if(incoming_byte == '\n')
     {
       if(validate_nmea_buffer())
       {
         push_out_nmea_sentence(true);
         parse_nmea_sentence();
       }
       nmea_read_buffer = "";
       keep_going = false;
     }
     else if (incoming_byte == '\r')
     {
       // don't do anything
     }
     else
     {
       nmea_read_buffer += String((char) incoming_byte);
     }
  }
  if((int) nmea_read_buffer.length() >= MAX_NMEA_BUFFER - 1 )
  {
    nmea_read_buffer = "";
  }
}


void update_hsnmea()
{

  bool keep_going = true;

  while(Serial6.available() && (int) hsnmea_read_buffer.length() < MAX_NMEA_BUFFER && keep_going == true)
  {
    int incoming_byte = Serial6.read();
    if(incoming_byte == '\n')
    {
      if(validate_nmea_buffer(true))
      {
        #ifdef SOFTSERIAL_DEBUG_ON
        debug_info("SoftSer good: " + String(hsnmea_read_buffer));
        #endif
        push_hsnmea_only_to_esp8266();   
        parse_hsnmea_sentence();
      }
      #ifdef SOFTSERIAL_DEBUG_ON
      else
      {
        debug_info("SoftSer full: " + String(hsnmea_read_buffer));
      }
      #endif
      hsnmea_read_buffer = "";
      keep_going = false;
    }
    else if (incoming_byte == '\r')
    {
      // don't do anything
    }
    else
    {
      hsnmea_read_buffer += String((char) incoming_byte);
    }
  }
  if((int) hsnmea_read_buffer.length() >= MAX_NMEA_BUFFER - 1 )
  {
    #ifdef SOFTSERIAL_DEBUG_ON
    debug_info("SoftSer toss: " + String(hsnmea_read_buffer));
    #endif
    
    hsnmea_read_buffer = "";
  }

}


bool try_and_send_message_to_esp8266()
{

  //
  //  Somewhat perversly, we take our time here as we want to be sure
  // nothing else is filling up the serial connection to the ESP (e.g.  NMEA
  // data)
  //

  #ifdef SERIAL_DEBUG_ON
  if(!esp8266IsReady())
  {
    debug_info(F("Waiting @ FHx"));
  }
  else
  {
    debug_info(F("Sailed @ FHx"));
  }
  #endif

  //
  //  Wait up to 10 full seconds for esp8266 to become ready (we almost
  // never have to, but if it is off struggling with a DNS lookup or
  // something, we _want_ to wait
  // 

  unsigned long time_stamp = millis();
  while(!esp8266IsReady() && (millis() - time_stamp) < 10000UL)
  {
  }
  
  if(!esp8266IsReady())
  {
    return false;
  }

  byte plain[MAX_LATEST_MESSAGE_SIZE];

  plain[0] = 'S';
  plain[1] = '=';
  if(Serial7.write(plain, 2) != 2)
  {
    return false;
  }


  unsigned int k = 0;
  for(i = 0; i < latest_message_to_send.length(); i = i + 32)
  {
    for(k = 0; k + i < latest_message_to_send.length() && k < 32; k++)
    {
      plain[k] = (char) latest_message_to_send.charAt(i + k);
    } 
    if(Serial7.write(plain, k) != k)
    {
      return false;
    }
    delay(10);
  }

  byte_zero = 0;
  for(i = 0; i < latest_message_to_send.length(); i = i + 1)
  {
    byte_zero = byte_zero ^ latest_message_to_send.charAt(i);
  }

  String checksum = String(byte_zero, HEX); 
  checksum.toUpperCase();
  if(checksum.length() < 2)
  {
    checksum =  String(F("@0")) + checksum + String(F("\r\n"));
  }
  else
  {
    checksum =  String(F("@")) + checksum + String(F("\r\n"));
  }
  if(Serial7.print(checksum) != 5)
  {
    return false;
  }

  return true;
}


void factoryReset()
{

  //
  // Tell the esp to do a factory and then rewrite out own eeprom 
  //

  digitalWrite(GPS_LED_1, LOW);
  digitalWrite(GPS_LED_2, LOW);
  digitalWrite(COM_LED_1, LOW);
  digitalWrite(COM_LED_2, LOW);

  //Serial7.println("FactoryResetNow");
  resetESP();
  init_eeprom_memory();
  resetFunc();
}


void dualReboot()
{
  digitalWrite(GPS_LED_1, LOW);
  digitalWrite(GPS_LED_2, LOW);
  digitalWrite(COM_LED_1, LOW);
  digitalWrite(COM_LED_2, LOW);
  resetESP();
  delay(4000);
  resetFunc();
}


void  send_message_to_esp8266()
{
  if(!try_and_send_message_to_esp8266())
  {
    send_message_failures++;
    if(send_message_failures > MAX_SEND_MESSAGE_FAILURES)
    {
      dualReboot();
    }
  }
  else
  {
    send_message_failures = 0;
  }
}


void zero_nmea_values()
{
  //
  //	Can't have nmea values persisting long after they were read, so this
  //	is called to nuke them if they are stale more than 30 seconds
  //
   
  if(millis() - nmea_speed_water_timestamp > nmea_sample_interval)
  {
    nmea_speed_water = -1.0;
  }
  if(millis() - nmea_depth_water_timestamp > nmea_sample_interval)
  {
    nmea_depth_water = -1.0;
  }
  if(millis() - nmea_wind_speed_timestamp > nmea_sample_interval)
  {
    nmea_wind_speed = -1.0;
  }
  if(millis() - nmea_wind_direction_timestamp > nmea_sample_interval)
  {
    nmea_wind_direction = -1.0;
  }
  if(millis() - nmea_wind_speed_abs_timestamp > nmea_sample_interval)
  {
    nmea_wind_speed_abs = -1.0;
  }
  if(millis() - nmea_wind_direction_abs_timestamp > nmea_sample_interval)
  {
    nmea_wind_direction_abs = -1.0;
  }
  if(millis() - nmea_water_temperature_timestamp > nmea_sample_interval)
  {
    nmea_water_temperature = -1.0;
  }
  if(millis() - nmea_heading_magnetic_timestamp > nmea_sample_interval)
  {
    nmea_heading_magnetic = -1.0;
  }
  if(millis() - nmea_heading_true_timestamp > nmea_sample_interval)
  {
    nmea_heading_true = -1.0;
  }
}


void resetESP()
{
  digitalWrite(ESP8266_RESET_PIN, LOW);
  delay(10);
  digitalWrite(ESP8266_RESET_PIN, HIGH);
  delay(100);
  digitalWrite(ESP8266_RESET_PIN, HIGH);
}




void setup()
{

  //
  //	Setup User-Reset and ESP-Reset pins
  //

  //pinMode(USER_RESET_PIN, INPUT_PULLUP);
  pinMode(USER_RESET_PIN, INPUT_PULLDOWN);
  user_reset_pin_state = LOW;
  user_reset_pin_timestamp = 0;

  pinMode(ESP8266_SERIAL_READY_PIN, INPUT);
  pinMode(ESP8266_RESET_PIN, OUTPUT);
  digitalWrite(ESP8266_RESET_PIN, HIGH);

  //
  // Watchdog
  //

  WDT_timings_t config;
  config.timeout = hardware_watchdog_interval; /* in seconds, 0->128 */
  watchdog.begin(config);

  //
  // Battery pins
  //

  pinMode(A15, INPUT_PULLDOWN);  
  pinMode(A16, INPUT_PULLDOWN);  
  pinMode(A17, INPUT_PULLDOWN);  

  //
  // Charger pins
  //

  pinMode(A0, INPUT_PULLDOWN);  
  pinMode(A1, INPUT_PULLDOWN);  
  pinMode(A2, INPUT_PULLDOWN);  

  //
  // Pump pins
  //

  pinMode(A3, INPUT_PULLDOWN);  
  pinMode(A6, INPUT_PULLDOWN);  
  pinMode(A7, INPUT_PULLDOWN);  

  //
  // Arduino Strings
  //
  
  console_read_buffer.reserve(MAX_CONSOLE_BUFFER);
  esp8266_read_buffer.reserve(MAX_ESP8266_BUFFER);
  gps_read_buffer.reserve(MAX_GPS_BUFFER);
  nmea_read_buffer.reserve(MAX_NMEA_BUFFER);
  hsnmea_read_buffer.reserve(MAX_NMEA_BUFFER);
  latest_message_to_send.reserve(MAX_LATEST_MESSAGE_SIZE);
  a_string.reserve(MAX_LATEST_MESSAGE_SIZE);
  b_string.reserve(MAX_GPS_BUFFER);
  mac_address.reserve(32);
  mac_address = "";

  for(i=0; i < 13; i++)
  {
    speed_vector[i] = 0.0;
    temperature_vector[i] = 100.0;
  }

  //
  //  Do we put out FHA/FHB/FHC messages on the console?
  //

  console_mode = false;
  #ifdef CONSOLE_DEBUG_ON
  console_mode = true;
  #endif

  //
  // Assume initial connection state is not
  //

  currently_connected = false;
  cellular_connected = false;

  //
  //  Serial7 is the ESP8266 (WiFi chip)
  //

  //Serial7.begin(115200);
  Serial7.begin(256000);

  //
  //  Misc setup
  //

  bhware_setup();
  gps_setup();


  //
  //	Setup NMEA ports
  //
  
  Serial8.begin(4800);
  Serial6.begin(38400);
  
  //
  //  Seed the random number generator
  //
  
  randomSeed(analogRead(A14));
  
  //
  //  Setup LED pins, initial is red on both
  //
  
  pinMode(GPS_LED_1, OUTPUT);
  pinMode(GPS_LED_2, OUTPUT);
  pinMode(COM_LED_1, OUTPUT);
  pinMode(COM_LED_2, OUTPUT);

  digitalWrite(GPS_LED_1, HIGH);
  digitalWrite(GPS_LED_2, LOW);
  digitalWrite(COM_LED_1, HIGH);
  digitalWrite(COM_LED_2, LOW);

  led_state = false;

  //
  //  Handle EEPROM logic for persistant settings (if the first 6 bytes of
  //  EEPROM memory are not all set to 42, then this is a completely
  //  unitialized device)
  //
  
  for(i = 0; i < 6; i++)
  {
    handy = EEPROM.read(i);
    if(handy != 42)
    {
      init_eeprom_memory();
      break;
    }
  }
  
  read_eeprom_memory();
  



  //
  //  Do one sensor read for temperature and barometric so first console messages have this data
  //
  
  bhware_read();
  
  //
  //  If this is an N2K model, setup N2K
  //

  #ifdef N2K_CODE_ON
  n2k_setup();
  #endif


}

void loop()
{

  /*
      Obviously this is main execution loop. There are a number of values we read and actions we take base on timing
  */
   
  unsigned long current_timestamp = millis();
 



  if(current_timestamp - gps_previous_timestamp >  gps_interval)
  {
    gps_previous_timestamp = current_timestamp;
    gps_read();
  } 



  if(current_timestamp - esp8266_previous_timestamp > esp8266_interval)
  {
    esp8266_previous_timestamp = current_timestamp;
    esp8266_read();
  }

  if(current_timestamp - sensor_previous_timestamp >  sensor_sample_interval)
  {
    sensor_previous_timestamp = current_timestamp;
    bhware_read();
  } 

  if(current_timestamp - voltage_previous_timestamp >  voltage_interval)
  {
    voltage_previous_timestamp = current_timestamp;
    voltage_read();
  } 


  if(current_timestamp - pump_previous_timestamp >  pump_interval)
  {
    pump_previous_timestamp = current_timestamp;
    pump_read();
  } 


  if(current_timestamp - console_previous_timestamp > console_interval)
  {
    console_previous_timestamp = current_timestamp;
    console_read();
  }

  if(current_timestamp - nmea_previous_timestamp > nmea_update_interval)
  {
    nmea_previous_timestamp = current_timestamp;
    update_nmea();
  }
  

  if(current_timestamp - hsnmea_previous_timestamp > hsnmea_update_interval)
  {
    hsnmea_previous_timestamp = current_timestamp;
    update_hsnmea();
  }



  #ifdef N2K_CODE_ON

  if(current_timestamp - n2k_previous_timestamp >  n2k_interval)
  {
    n2k_previous_timestamp = current_timestamp;
    NMEA2000.ParseMessages();
    //n2k_read();
  } 


  if(current_timestamp - n2k_output_previous_timestamp >  n2k_output_interval)
  {
    n2k_output_previous_timestamp = current_timestamp;
    n2k_output();
    //n2k_read();
  } 

  #endif

  
  /*
      Reporting routines
  */
  


  if(currently_active == true)
  {
    if(current_timestamp - previous_active_timestamp >  active_reporting_interval)
    {
      report_state(false);
      zero_nmea_values();
      previous_active_timestamp = current_timestamp;
      previous_idle_timestamp = current_timestamp;
    }
  }
  else
  {
   if(current_timestamp - previous_idle_timestamp >  ((unsigned long) stationary_interval * 60 * 1000))
    {
      report_state(false);
      zero_nmea_values();
      previous_active_timestamp = current_timestamp;
      previous_idle_timestamp = current_timestamp;
    }
  }


  if(current_timestamp - previous_console_timestamp >  console_reporting_interval)
  {
    previous_console_timestamp = current_timestamp;
    report_state(true);
    zero_nmea_values();
    watchdog.feed();	// Bump watchdog timer up to current time;
  }


  //
  //	Check on user reset button
  //

  if(digitalRead(USER_RESET_PIN) != user_reset_pin_state)
  {
    if(user_reset_pin_state == LOW)
    {
      user_reset_pin_timestamp = millis();
    }
    else
    {
      if(millis() - user_reset_pin_timestamp > USER_RESET_FACTORY_TIME) 
      {
        factoryReset();
        Serial.println("Factory reset");
      }
      else if(millis() - user_reset_pin_timestamp > USER_RESET_REBOOT_TIME) 
      {
        dualReboot();
        Serial.println("Dual reboot");
      }
      //user_reset_pin_timestamp = 0; 
    }
    user_reset_pin_state = digitalRead(USER_RESET_PIN);
  }


  //
  // Handle LED's to show communication state
  //
  

  if(current_timestamp - led_previous_timestamp > led_update_interval)
  {
    led_previous_timestamp = current_timestamp;
    update_leds();
  }

}
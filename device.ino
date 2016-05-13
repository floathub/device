/*

 FloatHub Arduino Code
 (c) 2011-2016 Modiot Labs
 (begun June 6, 2011)

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
 
 
  Nov. 6  2012
  
  Working _extremely_ well now, including active sensor, data storage when
  GPRS unavailable, etc.  Remaining task; cleanout enough string stuff, then
  implement NMEA input on Serial2.


 
  March, 2013

  Moved to a proper text-based build environment and versioning system on
  git-hub.


  April, 2013
  
  Brought in encrypted/$FHS code that was working back in July and made it
  trunk. It encrypts content using aes-128-cbc encryption

  May 2013
  
  Added NMEA input to the mix, so basic feature complete with NMEA,
  encryption, etc.  Added protocol/encryption version info to the protocol
  itself, plus numbering of pumps, batteries and chargers. 
 
*/

#include <Wire.h>
#include "./libs/Adafruit_BMP/Adafruit_BMP085.h"
#include <EEPROM.h>
#include <stdio.h>
#include "./libs/Time/Time.h"
#include "./libs/AES/AES.h"
#include "./libs/Base64/Base64.h"
#include <avr/wdt.h>
#include <util/crc16.h>
#include <SPI.h>

#include "sometypes.h"

/*
  Compile time option/debug flags
*/

//#define GPS_DEBUG_ON
//#define PUMP_DEBUG_ON
//#define EXECUTION_PATH_DEBUG_ON
//#define NMEA_DEBUG_ON
//#define DEBUG_MEMORY_ON
//#define STRESS_MEMORY_ON
//#define BYPASS_AES_ON
//#define BARO_DEBUG_ON	
//#define ACTIVE_DEBUG_ON

/*
  Anything which changes what the Floathub Data Receiver (fdr) needs to do
  to parse data should bump one these settings (if it's an enryption change,
  it's obviously the second one)
*/

#include "./esp8266/version_defines.h"

/*
  Some AES variables
*/

#define MAX_AES_CIPHER_LENGTH 1024
AES aes;
byte iv[16];
byte volatile_iv[16];
byte plain[MAX_AES_CIPHER_LENGTH];
byte cipher[MAX_AES_CIPHER_LENGTH];


/*
  Various communication and account settings/data
*/

String        float_hub_id;			// default: outofbox
byte          float_hub_aes_key[16]; 
bool	      currently_connected = false;
unsigned long boot_counter;			

/*
  Status LED's
*/

#define COM_LED_1   6
#define COM_LED_2   7

#define GPS_LED_1   8
#define GPS_LED_2   9

/*
   Some global Strings, character arrays
*/


#define MAX_LATEST_MESSAGE_SIZE 512
#define MAX_NEW_MESSAGE_SIZE 256 
String latest_message_to_send = "";
String new_message = "";
String a_string = "";
char   temp_string[20];  

/*
  Handy variables to use at various stages (better to be global, less memory)
*/

byte 	byte_zero, byte_one, byte_two, byte_three;
int  	int_one, handy;
float	float_one;
unsigned int i;


/*
    Overall timing parameters
*/

unsigned long sensor_sample_interval = 10000;     	//  Check temperature, pressure, every 10 seconds
unsigned long gps_interval = 50;                  	//  Read GPS serial 
unsigned long voltage_interval = 5000;            	//  Check batteries/chargers every 5 second
unsigned long pump_interval = 1200;               	//  Check pump state every 1.2 seconds
unsigned long active_reporting_interval = 30000;  	//  When in use, report data every 30 seconds
unsigned long idle_reporting_interval = 600000;   	//  When idle, report data every 10 minutes
unsigned long console_reporting_interval = 5000;  	//  Report to USB console every 5 seconds  
unsigned long console_interval = 250;             	//  Check console for input every 250 milliseconds
unsigned long esp8266_interval = 100;			//  Check for input from esp8266 on Serial 1  
unsigned long gprs_or_wifi_watchdog_interval = 120000;  //  Reboot the GPRS/wifi module after 2 minutes of no connection
unsigned long led_update_interval = 200;          	//  Update the LED's every 200 miliseconds
unsigned long nmea_update_interval = 100;         	//  Update NMEA in serial line every 1/10 of a second
unsigned long hardware_watchdog_interval = 120000; 	//  Do a hardware reset if we don't pat the dog every 2 minutes
unsigned long nmea_sample_interval = 30000;		//  Nuke nmea data older than 30 seconds
boolean com_led_state = false;         	  		//  For cycling on and off  
  
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
unsigned long hardware_watchdog_timestamp = 0;
unsigned long nmea_speed_water_timestamp = 0;
unsigned long nmea_depth_water_timestamp = 0;
unsigned long nmea_wind_speed_timestamp = 0;
unsigned long nmea_wind_direction_timestamp = 0;
unsigned long nmea_water_temperature_timestamp = 0;

/*
  Is the device currently "active" (i.e. is the vessel in movement and sending high frequency updates)?
*/
  
bool currently_active = true;


/*
  Watchdog variables
*/
  
long gprs_or_wifi_watchdog_timestamp = 0;
int  gprs_or_wifi_watchdog_counter = 0;
#define WATCHDOG_COUNTER_MAX	12

/*

  We use I2C (Wire.h) for Pressure and Temp
  
*/


Adafruit_BMP085 bmp;
#define BARO_HISTORY_LENGTH 10
float temperature;
float pressure;
float pressure_history[BARO_HISTORY_LENGTH];
float temperature_history[BARO_HISTORY_LENGTH];


/*
  Some global variables used in parsing from the GPS module
*/

#define	MAX_CONSOLE_BUFFER  255
#define MAX_GPS_BUFFER	    200
#define MAX_WIFI_BUFFER	     64
#define	MAX_NMEA_BUFFER	    100

String console_read_buffer;
String esp8266_read_buffer;
String gps_read_buffer;
String nmea_read_buffer;
byte nmea_cycle;

bool           gps_valid = false;
String         gps_utc = "";          //  UTC time and date
unsigned long  gps_utc_unix = 0;
String         gps_latitude = "";
String         gps_longitude = "";
String         gps_sog = "";          //  Speed over ground
String         gps_bearing_true ="";  //  Not magnetic!
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

pump_state pump_one_state = off;
pump_state pump_two_state = off;
pump_state pump_three_state = off;


/*
  Global variables for NMEA data
*/

float	nmea_speed_water = -1.0;	// Speed through water in knots, < 0 means invalid/no reading (do not report)
float	nmea_depth_water = -1.0;	// Depth of water below transducer, < 0 means invalid/not available
float	nmea_wind_speed = -1.0;		// Speed of true wind in knots, < 0 invalid/not available
float	nmea_wind_direction = -1.0;	// Angle of true wind in degrees, < 0 invalid/not available
float 	nmea_water_temperature = -1.0;	// Temperature of water in _FARENHEIT_, < 0 invalid/not available





/*
    Handy for figuring out if something is making us run out of memory
*/

#ifdef DEBUG_MEMORY_ON
extern int __bss_end;
extern int *__brkval;
void print_free_memory()
{
  int free_memory;
  
  if((int)__brkval == 0)
  {
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  }
  else
  {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  debug_info(F("Mem Free:"), free_memory);
}
#endif



/*
    Setup for barometric and temperature
*/


void bmp_setup()
{
  bmp.begin();
  for(i = 0; i < BARO_HISTORY_LENGTH; i++)
  {
    pressure_history[i] = 0.0;
    temperature_history[i] = 0.0;
  }   
}


void gps_setup()
{
  //
  //  Setup gps on serial device 3, and make it send only GGA and RMC NMEA sentences
  //

  Serial1.begin(115200);

  Serial3.begin(9600);
  delay(1000);

  //
  //	Really Old GPS
  //

  /*
  Serial3.begin(4800);
  Serial3.println("$PSTMNMEACONFIG,0,4800,66,1");
  Serial3.println(F("$PSRF103,00,00,02,01*26"));  //  GGA ON every 2 seconds
  Serial3.println(F("$PSRF103,01,00,00,01*25"));  //  GLL OFF
  Serial3.println(F("$PSRF103,02,00,00,01*26"));  //  GSA OFF
  Serial3.println(F("$PSRF103,03,00,00,01*27"));  //  GSV OFF
  Serial3.println(F("$PSRF103,04,00,02,01*22"));  //  RMC ON every 2 seconds
  Serial3.println(F("$PSRF103,05,00,00,01*21"));  //  VTG Off
  */
  
  //
  //	Old GPS on Serial 
  //	MTK3339 / Ultimate GPS breakout from Adafruit
  //
  
 
  //Serial3.println("$PMTK314,0,2,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");	// Every 2 seconds
  //Serial3.println("$PMTK314,0,3,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2A");		// GGA every 1 second, RMC every 3
  //Serial3.println("$PMTK314,0,3,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");		// GGA every 2 second, RMC every 3
  //Serial3.println("$PMTK314,0,5,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2E");		// GGA every 3 second, RMC every 5
  Serial3.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");		// GGA & RMC every second
  //Serial3.println("$PMTK314,0,2,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");		// GGA every 3 second, RMC every 2
 
}



void watchdog_setup()
{
  cli(); 
  wdt_reset();
  MCUSR &= ~(1<<WDRF); 
  // Enter Watchdog Configuration mode:
  WDTCSR = (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings: interrupte enable, 0110 for timer
  WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
  sei();
}

void pat_the_watchdog()
{
  //
  //	As long as we've managed to communciate at some relatively recent point, pat the dog
  //

    hardware_watchdog_timestamp = millis(); 

/*
  if(!gprs_or_wifi_communications_on)
  {  
  }
  else if(gprs_or_wifi_watchdog_counter < WATCHDOG_COUNTER_MAX)
  {
    hardware_watchdog_timestamp = millis();   
  }
  else
  {
    #ifdef GPRS_DEBUG_ON
    debug_info(F("No dog pat"));
    #endif
    #ifdef WIFI_DEBUG_ON
    debug_info(F("No dog pat"));
    #endif
  }
*/
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

ISR(WDT_vect) // Watchdog timer interrupt.
{ 
  if(millis() - hardware_watchdog_timestamp > hardware_watchdog_interval)
  {
    resetFunc();     				     // This will call location zero and cause a reboot.
  }
}


void init_eeprom_memory()
{

  //
  //  This function is called only during factory reset or intial startup
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
  
  EEPROM.write(10, 'o');
  EEPROM.write(11, 'u');
  EEPROM.write(12, 't');
  EEPROM.write(13, 'o');
  EEPROM.write(14, 'f');
  EEPROM.write(15, 'b');
  EEPROM.write(16, 'o');
  EEPROM.write(17, 'x');
  
  //
  //  Set to some kind of default AES key
  //
  
  EEPROM.write(4080, 0x00);
  EEPROM.write(4081, 0x01);
  EEPROM.write(4082, 0x02);
  EEPROM.write(4083, 0x03);
  EEPROM.write(4084, 0x04);
  EEPROM.write(4085, 0x05);
  EEPROM.write(4086, 0x06);
  EEPROM.write(4087, 0x07);
  EEPROM.write(4088, 0x08);
  EEPROM.write(4089, 0x09);
  EEPROM.write(4090, 0x0A);
  EEPROM.write(4091, 0x0B);
  EEPROM.write(4092, 0x0C);
  EEPROM.write(4093, 0x0D);
  EEPROM.write(4094, 0x0E);
  EEPROM.write(4095, 0x0F);


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
    EEPROM.write(4080 + i, float_hub_aes_key[i]);
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
    float_hub_aes_key[i] = EEPROM.read(4080 + i);
  }    
}  


void setup()
{
  

  console_read_buffer.reserve(MAX_CONSOLE_BUFFER);
  esp8266_read_buffer.reserve(MAX_CONSOLE_BUFFER);
  gps_read_buffer.reserve(MAX_GPS_BUFFER);
  nmea_read_buffer.reserve(MAX_NMEA_BUFFER);
  new_message.reserve(MAX_NEW_MESSAGE_SIZE);
  latest_message_to_send.reserve(MAX_LATEST_MESSAGE_SIZE);
  a_string.reserve(MAX_LATEST_MESSAGE_SIZE);

  bmp_setup();
  gps_setup();
  latest_message_to_send = "";
  
  //
  //  Setup main serial port for local data monitoring
  //
  
  Serial.begin(115200);
  delay(200);
  
  //
  //	Setup NMEA in port
  //
  
  Serial2.begin(4800);
  
  
  //
  //  Seed the random number generator
  //
  
  randomSeed(analogRead(0));
  
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


  //
  //  Handle EEPROM logic for persistant settings (if the first 6 bytes of
  //  EEPROM memory are not all set to 42, then this is a completely
  //  unitialized device)
  //
  
  int a = 0;
  for(i = 0; i < 6; i++)
  {
    a = EEPROM.read(i);
    if(a != 42)
    {
      init_eeprom_memory();
      break;
    }
  }
  
  read_eeprom_memory();
  
  //
  //  Do one sensor read for temperature and barometric so first console messages have this data
  //
  
  bmp_read();
  
  //
  //	Setup hardware watchdog timer 
  //
  
  watchdog_setup();
  

  //
  //  Announce we are up
  //
  

  help_info("Up and running...");
  display_current_variables();  

}


void help_info(String some_info)
{
   Serial.print(F("$FHH:"));
   Serial.print(float_hub_id);
   Serial.print(F(":"));
   Serial.print(FLOATHUB_PROTOCOL_VERSION);
   Serial.print(F("$    "));
   Serial.println(some_info);
}

void display_current_variables()
{
  new_message = "code=";
  new_message += FLOATHUB_PROTOCOL_VERSION;
  new_message += ".";
  new_message += FLOATHUB_ENCRYPT_VERSION;
  
  new_message += ",m=";
  new_message += FLOATHUB_MODEL_DESCRIPTION;
  new_message += ",b=";
  new_message += boot_counter;
  help_info(new_message);
  new_message = "";

  String line = "i=";
  line += float_hub_id;
  help_info(line);
  
  line = "k=";
  for(i = 0; i < 16; i++)
  {
    if(float_hub_aes_key[i] < 16)
    {
      line += "0";
    }
    line += String(float_hub_aes_key[i], HEX);
  }
  help_info(line);
  
}


void add_checksum_and_send_nmea_string(String nmea_string)
{

  byte_zero = 0;
  for(i = nmea_string.indexOf('$') + 1; i < nmea_string.lastIndexOf('*'); i++)
  {
    byte_zero = byte_zero ^ nmea_string.charAt(i);
  }

  Serial2.print(a_string);   

  String checksum = String(byte_zero, HEX); 
  checksum.toUpperCase();
  if(checksum.length() < 2)
  {
    nmea_string += "0";
  }
  nmea_string += checksum;

  Serial2.println(nmea_string);
  Serial1.print("E=");
  Serial1.println(nmea_string);

}

void bmp_read()
{


  //
  //	On some boards we very occasionally get an odd reading, so this
  //	history, average thing is just there to get rid of outliers
  //
  
  float average = 0.0;
  handy = 0;  

  for(i =0; i < BARO_HISTORY_LENGTH - 1 ; i++)
  {
    if(temperature_history[i] > 1)
    {
      average += temperature_history[i];
      handy ++;
    }
    temperature_history[i] = temperature_history[i+1];
  }

  //temperature_history[BARO_HISTORY_LENGTH - 1] = (1.8 * bmp.readTemperature()) + 32 - 5.6;
  temperature_history[BARO_HISTORY_LENGTH - 1] = (1.8 * bmp.readTemperature()) + 32;

  if(handy > 0)
  {
    average = average / ((float) handy);
    float closest = abs(temperature_history[BARO_HISTORY_LENGTH - 1] - average);
    temperature = temperature_history[BARO_HISTORY_LENGTH - 1];
    #ifdef BARO_DEBUG_ON
    debug_info("Temperature " + String(BARO_HISTORY_LENGTH) + ": ", temperature_history[BARO_HISTORY_LENGTH - 1]);
    #endif
    for(int_one = BARO_HISTORY_LENGTH - 2; int_one >= 0; int_one--)
    {
      #ifdef BARO_DEBUG_ON
      debug_info("Temperature " + String(int_one) + ": ", temperature_history[int_one]);
      #endif
      if(abs(temperature_history[int_one] - average) < closest)
      {
        closest = abs(temperature_history[int_one] - average);
        temperature = temperature_history[int_one];
      }
    }
  }
  else
  {
    temperature = temperature_history[BARO_HISTORY_LENGTH - 1];
  }



  for(i =0; i < BARO_HISTORY_LENGTH - 1 ; i++)
  {
    if(pressure_history[i] > 1)
    {
      average += pressure_history[i];
      handy ++;
    }
    pressure_history[i] = pressure_history[i+1];
  }
  pressure_history[BARO_HISTORY_LENGTH - 1] = bmp.readPressure() * 0.000295300;

  if(handy > 0)
  {
    average = average / ((float) handy);
    float closest = abs(pressure_history[BARO_HISTORY_LENGTH - 1] - average);
    pressure = pressure_history[BARO_HISTORY_LENGTH - 1];
    #ifdef BARO_DEBUG_ON
    debug_info("Pressure " + String(BARO_HISTORY_LENGTH) + ": ", pressure_history[BARO_HISTORY_LENGTH - 1]);
    #endif
    for(int_one = BARO_HISTORY_LENGTH - 2; int_one >= 0; int_one--)
    {
      #ifdef BARO_DEBUG_ON
      debug_info("Pressure " + String(int_one) + ": ", pressure_history[int_one]);
      #endif
      if(abs(pressure_history[int_one] - average) < closest)
      {
        closest = abs(pressure_history[int_one] - average);
        pressure = pressure_history[int_one];
      }
    }
  }
  else
  {
    pressure = pressure_history[BARO_HISTORY_LENGTH - 1];
  }



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
    a_string = F("$WIMTA,");
    append_float_to_string(a_string, temperature);
    a_string += F(",F*");

    add_checksum_and_send_nmea_string(a_string);
  }

  //
  //
  //	Now the MDA combined
  //
  //


  else if(nmea_cycle == 1)
  {
    a_string = F("$WIMDA,");
    append_float_to_string(a_string, pressure);
    a_string += F(",I,");
    append_float_to_string(a_string, pressure * 0.03386388158);
    a_string += F(",B,");
    append_float_to_string(a_string, (temperature - 32.0) * (5.0 / 9.0));
    a_string += F(",C,,,,,,,,,,,,,,*");


    add_checksum_and_send_nmea_string(a_string);   
  }
  
  //
  // 	Then the old school MMB for Air Pressure
  //

  else if(nmea_cycle == 2)
  {
    a_string = F("$WIMMB,");
    append_float_to_string(a_string, pressure);
    a_string += F(",I,");
    append_float_to_string(a_string, pressure * 0.03386388158);
    a_string += F(",B*");

    add_checksum_and_send_nmea_string(a_string);  
  }

  //
  // 	Finally new school XDR (Type, Data, Units, ID)
  //

  else if(nmea_cycle == 3)
  {
    a_string = F("$WIXDR,C,");
    append_float_to_string(a_string, (temperature - 32.0) * (5.0 / 9.0));
    a_string += F(",C,FHUB_TEMP,P,");
    append_float_to_string(a_string, pressure * 0.03386388158);
    a_string += F(",B,FHUB_BARO*");

    add_checksum_and_send_nmea_string(a_string);  
  }

  nmea_cycle += 1;
  if(nmea_cycle > 3)
  {
    nmea_cycle = 0;
  }
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
      return;
  }
      
     
  
  String is_valid = gps_read_buffer.substring(status_start + 1, lat_start);
  
  /*
    Break GPS time/date into integer parts
  */

  gps_utc  = gps_read_buffer.substring(time_start + 1, time_break);
  gps_utc += gps_read_buffer.substring(date_start + 1, var_start - 2);
  gps_utc += "20";
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
    gps_sog  = gps_read_buffer.substring(sog_start + 1, tmg_start);
    gps_bearing_true  = gps_read_buffer.substring(tmg_start + 1, date_start);

    //
    //   Use speed over gound to figure out if active. Used to have some
    // really complicated math in here for running average of latitude and
    // longitude, but this is far simpler and seems more reliable
    //

    memset(temp_string, 0, 20 * sizeof(char));
    gps_sog.substring(0,gps_sog.length()).toCharArray(temp_string, 19);
    float_one = atof(temp_string);
  
    //if(float_one > 0.0)  //  Always active for debugging
    if(float_one > 0.25)  //  Moving faster than a 1/4 knot?
    {
      #ifdef ACTIVE_DEBUG_ON
      debug_info(F("active: ON"));
      #endif
      currently_active = true;
    }
    else
    {
      #ifdef ACTIVE_DEBUG_ON
      debug_info(F("active: OFF"));
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
      
    
  gps_latitude  = gps_read_buffer.substring(lat_start + 1, lat_start + 3);
  gps_latitude += " ";
  gps_latitude += gps_read_buffer.substring(lat_start + 3, nors_start);
  while(gps_latitude.length() < 11)
  {
    gps_latitude += "0";
  }
  gps_latitude += gps_read_buffer.substring(nors_start + 1, lon_start);
    
  gps_longitude  = gps_read_buffer.substring(lon_start + 1, lon_start + 4);
  gps_longitude += " ";
  gps_longitude += gps_read_buffer.substring(lon_start + 4, wore_start);
  while(gps_longitude.length() < 12)
  {
    gps_longitude += "0";
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

void push_out_nmea_sentence(bool from_nmea_in)
{
  if(from_nmea_in)
  {
    Serial2.println(nmea_read_buffer);
    Serial1.print("E=");
    Serial1.println(nmea_read_buffer);
  }
  else
  {
    Serial2.println(gps_read_buffer);
    Serial1.print("E=");
    Serial1.println(gps_read_buffer);
  }
}



bool validate_gps_buffer()
{
  byte_zero = 0;
  for(i = gps_read_buffer.indexOf('$') + 1; i < gps_read_buffer.lastIndexOf('*'); i++)
  {
    byte_zero = byte_zero ^ gps_read_buffer.charAt(i);
  }
  a_string = String(byte_zero, HEX);
  a_string.toUpperCase();
  if(gps_read_buffer.endsWith(a_string))
  {
    return true;
  }


  #ifdef GPS_DEBUG_ON
  debug_info("Bad GPS buffer: " + gps_read_buffer);
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

bool validate_nmea_buffer()
{
  byte_zero = 0;
  for(i = nmea_read_buffer.indexOf('$') + 1; i < nmea_read_buffer.lastIndexOf('*'); i++)
  {
    byte_zero = byte_zero ^ nmea_read_buffer.charAt(i);
  }
  a_string = String(byte_zero, HEX);
  a_string.toUpperCase();
  if(nmea_read_buffer.endsWith(a_string))
  {
    return true;
  }
  #ifdef NMEA_DEBUG_ON
  debug_info("Bad NMEA buffer: " + nmea_read_buffer);
  #endif
  return false;  
}


void gps_read()
{

  while(Serial3.available() && (int) gps_read_buffer.length() < MAX_GPS_BUFFER)
  {
    int incoming_byte = Serial3.read();
    if(incoming_byte == '\n')
    {
      if(validate_and_maybe_remediate_gps_buffer())
      {
        if(gps_read_buffer.indexOf("$GPRMC,") == 0)
        {
          #ifdef GPS_DEBUG_ON
          debug_info(F("--GPS BUF RMC--"));
          debug_info(gps_read_buffer);
          #endif
          push_out_nmea_sentence(false);
          parse_gps_buffer_as_rmc();
        }
        else if(gps_read_buffer.indexOf("$GPGGA,") == 0)
        {
          #ifdef GPS_DEBUG_ON
          debug_info(F("--GPS BUF GGA--"));
          debug_info(gps_read_buffer);
          #endif
          push_out_nmea_sentence(false);
          parse_gps_buffer_as_gga();
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
  battery_one 	= analogRead(3) / 37.213; 
  battery_two 	= analogRead(2) / 37.213;
  battery_three = analogRead(1) / 37.213; 

  charger_one 	= analogRead(6) / 37.213; 
  charger_two 	= analogRead(5) / 37.213; 
  charger_three	= analogRead(4) / 37.213;
}



void append_formatted_value(String &the_string, int value)
{
  if(value < 10)
  {
    the_string += "0";
    the_string += value;
  }
  else
  {
    the_string += value;
  }
}

void add_timestamp_to_string(String &the_string)
{
  the_string += ",U:";
  append_formatted_value(the_string, hour());
  append_formatted_value(the_string, minute());
  append_formatted_value(the_string, second());
  append_formatted_value(the_string, day());
  append_formatted_value(the_string, month());
  the_string += year();          
}


void individual_pump_read(int pump_number, pump_state &state, int analog_input)
{
  float pump_value = analogRead(analog_input) / 37.213;
  #ifdef PUMP_DEBUG_ON
  debug_info("Pump " + String(pump_number) + " on input " + String(analog_input) + " reads ", pump_value);
  #endif
  if(pump_value > 2.0)
  {
     if(state == unknown)
     {
       state = on;
     }
     else if(state == off)
     {
       #ifdef PUMP_DEBUG_ON
       debug_info(F("Pump turned on!"));
       #endif
       new_message = "$FHB:";
       new_message += float_hub_id;
       new_message += ":";
       new_message += FLOATHUB_PROTOCOL_VERSION;
       new_message += "$";
       if(gps_valid || timeStatus() != timeNotSet)
       {
         add_timestamp_to_string(new_message);
       }
       new_message += ",P";
       new_message += pump_number;
       new_message += ":1";
       
       //queue_pump_message(pump_number,1);
       echo_info(new_message);
       state = on;       
     }
  }
  else
  {
     if(state == unknown)
     {
       state = off;
     }
     else if(state == on)
     {
       #ifdef PUMP_DEBUG_ON
       debug_info(F("Pump turned off!"));
       #endif
       new_message = "$FHB:";
       new_message += float_hub_id;
       new_message += ":";
       new_message += FLOATHUB_PROTOCOL_VERSION;
       new_message += "$";

       if(gps_valid|| timeStatus() != timeNotSet)
       {
         add_timestamp_to_string(new_message);
       }
       new_message += ",P";
       new_message += pump_number;
       new_message += ":0";
       //queue_pump_message(pump_number,0);
       echo_info(new_message);
       state = off;
     }
  }
}


void pump_read()
{
  individual_pump_read(1, pump_one_state, 9);
  individual_pump_read(2, pump_two_state, 8);
  individual_pump_read(3, pump_three_state, 7);
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
    new_message += tag;
    append_float_to_string(new_message, value);
  }
}
void report_state(bool console_only)
{
  if(console_only)
  {
    new_message = "$FHC:";
  }
  else
  {
    new_message = "$FHA:";
  }

  new_message += float_hub_id;
  new_message += ":";
  new_message += FLOATHUB_PROTOCOL_VERSION;
  new_message += "$";
  if(gps_valid == true || timeStatus() != timeNotSet )
  {
     add_timestamp_to_string(new_message);
  }
  
  new_message += ",T:";
  append_float_to_string(new_message, temperature);

  new_message += ",P:";
  append_float_to_string(new_message, pressure);

  if(gps_valid == true && gps_altitude.length() > 0)
  {
    new_message += ",L:";
    new_message += gps_latitude;
    
    new_message += ",O:";
    new_message += gps_longitude;

    new_message += ",A:";
    new_message += gps_altitude;

    new_message += ",H:";
    new_message += gps_hdp;
    
    new_message += ",S:";
    new_message += gps_sog;

    new_message += ",B:";
    new_message += gps_bearing_true;
  }

  if(gps_siv.length() > 0)
  {
    new_message += String(",N:");
    new_message += gps_siv;
  }

  possibly_append_data(battery_one, 0.2, ",V1:");
  possibly_append_data(battery_two, 0.2, ",V2:");
  possibly_append_data(battery_three, 0.2, ",V3:");

  possibly_append_data(charger_one, 0.2, ",C1:");
  possibly_append_data(charger_two, 0.2, ",C2:");
  possibly_append_data(charger_three, 0.2, ",C3:");

  //
  //	Add NMEA data
  //
  
  possibly_append_data(nmea_speed_water, -0.5, ",R:");
  possibly_append_data(nmea_depth_water, -0.5, ",D:");
  possibly_append_data(nmea_wind_speed, -0.5, ",J:");
  possibly_append_data(nmea_wind_direction, -0.5, ",K:");
  possibly_append_data(nmea_water_temperature, -0.5, ",Y:");

  if(!console_only)
  {
    //queue_detailed_message();
  }
  echo_info(new_message);
}

void parse_esp8266()
{


  if(esp8266_read_buffer.startsWith("$FHI"))
  {
    if(esp8266_read_buffer.length() == 23 && esp8266_read_buffer.substring(20).startsWith("c="))
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
    else if(esp8266_read_buffer.length() >= 30 && esp8266_read_buffer.substring(20).startsWith("i="))
    {
      a_string = esp8266_read_buffer.substring(22,30);
      if(a_string != float_hub_id)
      {
        float_hub_id = a_string;
        write_eeprom_memory();
      }
    }
    else if(esp8266_read_buffer.length() >= 54 && esp8266_read_buffer.substring(20).startsWith("k="))
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
  }
  else
  {
    //
    // Otherwise, just show it on the console
    //

    //Serial.print("ESP sent: ");
    Serial.println(esp8266_read_buffer);
  }
}  

void parse_console()
{
  //
  // We could evesdrop here if we wanted to, but currently do not.  Just
  // push all input up the esp8266 on Serial1.
  //

  Serial.print("Sending: ");
  Serial.println(console_read_buffer);
  Serial1.println(console_read_buffer);

}  


void esp8266_read()
{

  while(Serial1.available() && (int) esp8266_read_buffer.length() < MAX_CONSOLE_BUFFER)
  {
     int incoming_byte = Serial1.read();
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
  if((int) esp8266_read_buffer.length() >= MAX_CONSOLE_BUFFER - 1 )
  {
    esp8266_read_buffer = "";
  }
}

void console_read()
{
  String display_string;

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


void echo_info(String some_info)
{
  Serial.println(some_info);
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
  Serial.print(some_info);
}

void debug_info(String some_info)
{
  debug_info_core(some_info);
  Serial.println();
}

void debug_info(String some_info, float x)
{
  debug_info_core(some_info);
  Serial.println(x);
}


void debug_info(String some_info, int x)
{
  debug_info_core(some_info);
  Serial.println(x);
}



void update_leds()
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

  if(currently_connected)
  {
    digitalWrite(COM_LED_1, LOW);
    digitalWrite(COM_LED_2, HIGH);
  } 
  else
  {
    digitalWrite(COM_LED_1, HIGH);
    digitalWrite(COM_LED_2, LOW);
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
    debug_info("NMEA buf:" + nmea_read_buffer);
    #endif    

  int commas[8] = {-1, -1, -1, -1, -1, -1, -1 , -1};

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
  //	Depth below transducer
  //

  if( popout_nmea_value("DBT", commas[2], commas[3], nmea_depth_water))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info("NMEA d water:", nmea_depth_water);
    #endif    
    nmea_depth_water_timestamp = millis();
  }

  //
  //	Mean Temperature of Water
  //


  else if( popout_nmea_value("MTW", commas[0], commas[1], nmea_water_temperature, true))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info("NMEA w temp:", nmea_water_temperature);
    #endif    
    nmea_water_temperature_timestamp = millis();
  }

  //
  //	Speed Through Water
  //

  else if( popout_nmea_value("VHW", commas[4], commas[5], nmea_speed_water))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info("NMEA s water:", nmea_speed_water);
    #endif    
    nmea_speed_water_timestamp = millis();
  }
  
  //
  //	Wind Speed and direction if speed works
  //
  
  else if(  popout_nmea_value("MWV", commas[2], commas[3], nmea_wind_speed))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info("NMEA w speed:", nmea_wind_speed);
    #endif
    nmea_wind_speed_timestamp = millis();
    if(	popout_nmea_value("MWV", commas[0], commas[1], nmea_wind_direction))
    {
      #ifdef NMEA_DEBUG_ON
      debug_info("NMEA w direction:", nmea_wind_direction);
      #endif
      nmea_wind_direction_timestamp = millis();
    }
  }
}

void update_nmea()
{
  while(Serial2.available() && (int) nmea_read_buffer.length() < MAX_NMEA_BUFFER)
  {
     int incoming_byte = Serial2.read();
     if(incoming_byte == '\n')
     {
       if(validate_nmea_buffer())
       {
         push_out_nmea_sentence(true);
         parse_nmea_sentence();
       }
       nmea_read_buffer = "";
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

void encode_latest_message_to_send()
{

  #ifdef BYPASS_AES_ON
    return;
  #endif


  #ifdef STRESS_MEMORY_ON
  for(i=0; i < 10; i++)
  {
    latest_message_to_send += F(" MEMORY STRESS TEST");
  }
  #endif

  //
  //  Take the latest message that is set up to go over GPRS and AES encode it, then Base-64 convert it
  //
    
  unsigned int cipher_length, base64_length;
 
  aes.set_key (float_hub_aes_key, 128) ;
  
  for(i = 0; i < 16; i++)
  {
    iv[i] = random(0, 256);
    volatile_iv[i] = iv[i];
  }

  //
  //  Copy current message to plain
  //
    
  for(i = 0; i < latest_message_to_send.length(); i++)
  {
    plain[i] = latest_message_to_send.charAt(i);
  }
  cipher_length = i;
  if(cipher_length % 16 == 0)
  {
    for(i = 0; i < 16; i++)
    {
      plain[cipher_length + i] = 16;
    }
    cipher_length += 16;
  }
  else
  {
    for(i = 0; i < 16 - (cipher_length % 16); i++)
    {
      plain[cipher_length + i] = 16 - (cipher_length % 16);
    }
    cipher_length += i;
  }

  //
  //  Encrypt current message with AES CBC
  // 

  aes.cbc_encrypt (plain, cipher,  cipher_length / 4, volatile_iv) ;
  
  //
  //  Now we reuse the plain text array to store cipher with the 
  //  initialization vector at the beginning
  //

  
  for(i = 0; i < 16; i++)
  {
    plain[i] = iv[i];
  }
  for(i = 0; i < cipher_length; i++)
  {
    plain[16 + i] = cipher[i];
  }
  

  //
  //  Now convert that long line of bytes in plain to base 64, recycling the cipher array to hold it
  //
  
  base64_length = base64_encode( (char *) cipher, (char *) plain, cipher_length + 16);
  cipher[base64_length] = '\0';
  latest_message_to_send = "$FHS:";
  latest_message_to_send += float_hub_id;
  latest_message_to_send += ":";
  latest_message_to_send += FLOATHUB_ENCRYPT_VERSION;
  latest_message_to_send += "$,";
  for(i = 0; i < base64_length; i++)
  {
    latest_message_to_send += (char) cipher[i];
  }

  //
  //  Clean up?
  // 

  aes.clean();  //  Not sure if that does anything useful or not.   

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
  if(millis() - nmea_water_temperature_timestamp > nmea_sample_interval)
  {
    nmea_water_temperature = -1.0;
  }
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
    bmp_read();
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
  
  /*
      Reporting routines
  */
  
  if(currently_active == true)
  {
    if(current_timestamp - previous_active_timestamp >  active_reporting_interval)
    {
      previous_active_timestamp = current_timestamp;
      report_state(false);
      zero_nmea_values();
    }
  }
  else
  {
    if(current_timestamp - previous_idle_timestamp >  idle_reporting_interval)
    {
      previous_idle_timestamp = current_timestamp;
      report_state(false);
      zero_nmea_values();
    }
  }
  if(current_timestamp - previous_console_timestamp >  console_reporting_interval)
  {
    previous_console_timestamp = current_timestamp;
    report_state(true);
    zero_nmea_values();
    #ifdef DEBUG_MEMORY_ON
    print_free_memory();
    #endif
    pat_the_watchdog();	// Bump watchdog timer up to current time;
  }
  
  /*
    Handle LED's to show communication state
  */
  
  if(current_timestamp - led_previous_timestamp > led_update_interval)
  {
    led_previous_timestamp = current_timestamp;
    update_leds();
  }
  
  #ifdef EXECUTION_PATH_DEBUG_ON
  if(random(0,100) < 50)
  {
    digitalWrite(GPS_LED_1, HIGH);
    digitalWrite(GPS_LED_2, LOW);
  }
  else
  {
    digitalWrite(GPS_LED_1, LOW);
    digitalWrite(GPS_LED_2, HIGH);
  }
  #endif
}




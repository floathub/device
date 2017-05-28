/*

 FloatHub Arduino Code
 (c) 2011-2016 Modiot Labs
 (begun June 6, 2011)


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
#include <SoftwareSerial.h>

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
//#define SERIAL_DEBUG_ON
//#define SOFTSERIAL_DEBUG_ON



/*
  Anything which changes what the Floathub Data Receiver (fdr) needs to do
  to parse data should bump one these settings (if it's an enryption change,
  it's obviously the second one)
*/

#include "./esp8266/version_defines.h"

/*
  Some AES variables
*/

#define MAX_AES_CIPHER_LENGTH 800
AES aes;
byte iv[16];
byte volatile_iv[16];
byte plain[MAX_AES_CIPHER_LENGTH];
byte cipher[MAX_AES_CIPHER_LENGTH];


/*
  Various communication and account settings/data
*/

String          float_hub_id;			// default: outofbox
byte            float_hub_aes_key[16]; 
bool	        currently_connected = false;
unsigned long   boot_counter;			
boolean         led_state = false;              //  For cycling on and off  
#define		ESP8266_SERIAL_READY_PIN 2	//  ESP8266 sets this HIGH to say it is ready for more serial data
#define		ESP8266_RESET_PIN  3		//  Pin we drive to ground to reset the esp8266 programmatically from here
#define         USER_RESET_PIN    4		//  Pin that signals when user is holding down the reset button
bool		user_reset_pin_state;		//  Measured state of user reset pin; 
unsigned long	user_reset_pin_timestamp = 0;	//  Last time the user rest pin was initially pushed in
#define         USER_RESET_REBOOT_TIME 5000	//  Hold down reset pin for 5 seconds to make device reboot
#define         USER_RESET_FACTORY_TIME 20000	//  Hold down reset pin for 20 seconds to make device factory reset
int		send_message_failures = 0;	//  Number of times we can fail to send something to ESP8266 before we dual reboot
#define		MAX_SEND_MESSAGE_FAILURES 5

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
//unsigned long idle_reporting_interval = 10000;   	//  Stress testing during dvelopment
unsigned long idle_reporting_interval = 600000;   	//  When idle, report data every 10 minutes
//unsigned long idle_reporting_interval = 30000;   	//  Demoboat only every 30 seconds
unsigned long console_reporting_interval = 5000;  	//  Report to USB console every 5 seconds  
unsigned long console_interval = 250;             	//  Check console for input every 250 milliseconds
unsigned long esp8266_interval = 100;			//  Check for input from esp8266 on Serial 1  
unsigned long led_update_interval = 200;          	//  Update the LED's every 200 miliseconds
unsigned long nmea_update_interval = 100;         	//  Update NMEA in serial line every 1/10 of a second
unsigned long hsnmea_update_interval = 100;		//  Update HS NMEA (SoftwareSerial based) every 1/10 of a second 
unsigned long hardware_watchdog_interval = 120000; 	//  Do a hardware reset if we don't pat the dog every 2 minutes
unsigned long nmea_sample_interval = 30000;		//  Nuke nmea data older than 30 seconds
  
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
unsigned long hardware_watchdog_timestamp = 0;
unsigned long nmea_speed_water_timestamp = 0;
unsigned long nmea_depth_water_timestamp = 0;
unsigned long nmea_wind_speed_timestamp = 0;
unsigned long nmea_wind_direction_timestamp = 0;
unsigned long nmea_water_temperature_timestamp = 0;


/*
  Software Serial Stuff
*/


#define SOFT_SERIAL_RX_PIN 10
#define SOFT_SERIAL_TX_PIN 11  // We don't actually currently use this for anything    
SoftwareSerial soft_serial (SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN); 


/*
  Is the device currently "active" (i.e. is the vessel in movement and sending high frequency updates)?
*/
  
bool currently_active = true;


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

  Serial3.begin(9600);
  delay(1000);

  //
  //	Configure in case the GPS is a MTK3339 / Ultimate GPS breakout from Adafruit
  //

  Serial3.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));		// GGA & RMC every second

  //
  //   	Or a uBlox NEO-6M
  // 

  Serial3.println("$PUBX,40,GLL,0,0,0,0,0,0*5C");
  Serial3.println("$PUBX,40,GGA,0,1,0,0,0,0*5B"); 
  Serial3.println("$PUBX,40,GSA,0,0,0,0,0,0*4E");
  Serial3.println("$PUBX,40,RMC,0,1,0,0,0,0*46");
  Serial3.println("$PUBX,40,GSV,0,0,0,0,0,0*59");
  Serial3.println("$PUBX,40,VTG,0,0,0,0,0,0*5E");

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


void resetESP()
{
  digitalWrite(ESP8266_RESET_PIN, LOW);
  delay(3000);
  digitalWrite(ESP8266_RESET_PIN, HIGH);
}

void setup()
{
  

  console_read_buffer.reserve(MAX_CONSOLE_BUFFER);
  esp8266_read_buffer.reserve(MAX_CONSOLE_BUFFER);
  gps_read_buffer.reserve(MAX_GPS_BUFFER);
  nmea_read_buffer.reserve(MAX_NMEA_BUFFER);
  hsnmea_read_buffer.reserve(MAX_NMEA_BUFFER);
  new_message.reserve(MAX_NEW_MESSAGE_SIZE);
  latest_message_to_send.reserve(MAX_LATEST_MESSAGE_SIZE);
  a_string.reserve(MAX_LATEST_MESSAGE_SIZE);


  //
  //  Serial1 is the ESP8266 (WiFi chip)
  //

  pinMode(ESP8266_SERIAL_READY_PIN, INPUT);
  Serial1.begin(115200);

  //
  //  SoftwareSerial at 38400
  //

  pinMode(SOFT_SERIAL_RX_PIN, INPUT);
  pinMode(SOFT_SERIAL_TX_PIN, OUTPUT);  
  soft_serial.begin(38400);
  soft_serial.listen();
  

  bmp_setup();
  gps_setup();
  latest_message_to_send = "";
  send_message_failures = 0;
  
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

  led_state = false;

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
  //	Setup User-Reset and ESP-Reset pins
  //

  pinMode(USER_RESET_PIN, INPUT);
  user_reset_pin_state = LOW;
  user_reset_pin_timestamp = 0;

  pinMode(ESP8266_RESET_PIN, OUTPUT);
  digitalWrite(ESP8266_RESET_PIN, HIGH);


  //
  //  Announce we are up
  //
  

  help_info(F("Up and running..."));
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
  new_message = F("code=");
  new_message += FLOATHUB_PROTOCOL_VERSION;
  new_message += F(".");
  new_message += FLOATHUB_ENCRYPT_VERSION;
  
  new_message += F(",m=");
  new_message += FLOATHUB_MODEL_DESCRIPTION;
  new_message += F(",b=");
  new_message += boot_counter;
  help_info(new_message);
  new_message = "";

  String line = F("i=");
  line += float_hub_id;
  help_info(line);
  
  line = F("k=");
  for(i = 0; i < 16; i++)
  {
    if(float_hub_aes_key[i] < 16)
    {
      line += F("0");
    }
    line += String(float_hub_aes_key[i], HEX);
  }
  help_info(line);
  
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
  for(i = nmea_string.indexOf('$') + 1; i < nmea_string.lastIndexOf('*'); i++)
  {
    byte_zero = byte_zero ^ nmea_string.charAt(i);
  }

  String checksum = String(byte_zero, HEX); 
  checksum.toUpperCase();
  if(checksum.length() < 2)
  {
    nmea_string += F("0");
  }
  nmea_string += checksum;



  Serial2.println(nmea_string);

  if(esp8266IsReady())
  {
    Serial1.print(F("E="));
    Serial1.println(nmea_string);
  }
  #ifdef SERIAL_DEBUG_ON
  else
  {
    debug_info(F("XA NMEA"));
  }
  #endif

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
    debug_info(String(F("Temperature ")) + String(BARO_HISTORY_LENGTH) + String(F(": ")), temperature_history[BARO_HISTORY_LENGTH - 1]);
    #endif
    for(int_one = BARO_HISTORY_LENGTH - 2; int_one >= 0; int_one--)
    {
      #ifdef BARO_DEBUG_ON
      debug_info(String(F("Temperature ")) + String(int_one) + String(F(": ")), temperature_history[int_one]);
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
    debug_info(String(F("Pressure ")) + String(BARO_HISTORY_LENGTH) + String(F(": ")), pressure_history[BARO_HISTORY_LENGTH - 1]);
    #endif
    for(int_one = BARO_HISTORY_LENGTH - 2; int_one >= 0; int_one--)
    {
      #ifdef BARO_DEBUG_ON
      debug_info(String(F("Pressure ")) + String(int_one) + String(F(": ")), pressure_history[int_one]);
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
  if(!esp8266IsReady())
  {
    #ifdef SERIAL_DEBUG_ON
    debug_info(F("XC NMEA"));
    #endif
    return;
  }
  Serial1.print(F("E="));
  Serial1.println(hsnmea_read_buffer);
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
      Serial1.print(F("E="));
      Serial1.println(nmea_read_buffer);
    }
    else
    {
      Serial1.print(F("E="));
      Serial1.println(gps_read_buffer);
    }
  }

  if(from_nmea_in)
  {
    Serial2.println(nmea_read_buffer);
  }
  else
  {
    Serial2.println(gps_read_buffer);
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
    if(start_point < 2)
    {
      start_point = hsnmea_read_buffer.indexOf('$');
    }
    for(i = start_point + 1; i < hsnmea_read_buffer.lastIndexOf('*'); i++)
    {
      byte_zero = byte_zero ^ hsnmea_read_buffer.charAt(i);
    }
  }
  else
  {
    for(i = nmea_read_buffer.indexOf('$') + 1; i < nmea_read_buffer.lastIndexOf('*'); i++)
    {
      byte_zero = byte_zero ^ nmea_read_buffer.charAt(i);
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
  #ifdef NMEA_DEBUG_ON
  if(hsnmea)
  {
    debug_info(String(F("Bad NMEA buffer: ")) + hsnmea_read_buffer + ", Wanted: " + a_string);
  }
  else
  {
    debug_info(String(F("Bad NMEA buffer: ")) + nmea_read_buffer + ", Wanted: " + a_string);
  }
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
        if(gps_read_buffer.indexOf(F("$GPRMC,")) == 0)
        {
          #ifdef GPS_DEBUG_ON
          debug_info(F("--GPS BUF RMC--"));
          debug_info(gps_read_buffer);
          #endif
          push_out_nmea_sentence(false);
          parse_gps_buffer_as_rmc();

        }
        else if(gps_read_buffer.indexOf(F("$GPGGA,")) == 0)
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
  battery_one 	= analogRead(1) / 37.213; 
  battery_two 	= analogRead(2) / 37.213;
  battery_three = analogRead(3) / 37.213; 

  charger_one 	= analogRead(4) / 37.213; 
  charger_two 	= analogRead(5) / 37.213; 
  charger_three	= analogRead(6) / 37.213;
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


void individual_pump_read(int pump_number, pump_state &state, int analog_input)
{
  float pump_value = analogRead(analog_input) / 37.213;
  #ifdef PUMP_DEBUG_ON
  debug_info(F("Pump ") + String(pump_number) + F(" on input ") + String(analog_input) + F(" reads "), pump_value);
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
       new_message = F("$FHB:");
       new_message += float_hub_id;
       new_message += F(":");
       new_message += FLOATHUB_PROTOCOL_VERSION;
       new_message += F("$");
       if(gps_valid || timeStatus() != timeNotSet)
       {
         add_timestamp_to_string(new_message);
       }
       new_message += F(",P");
       new_message += pump_number;
       new_message += F(":1");
       
       latest_message_to_send = new_message;
       encode_latest_message_to_send();
       send_encoded_message_to_esp8266();
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
       new_message = F("$FHB:");
       new_message += float_hub_id;
       new_message += F(":");
       new_message += FLOATHUB_PROTOCOL_VERSION;
       new_message += F("$");

       if(gps_valid|| timeStatus() != timeNotSet)
       {
         add_timestamp_to_string(new_message);
       }
       new_message += F(",P");
       new_message += pump_number;
       new_message += F(":0");

       latest_message_to_send = new_message;
       encode_latest_message_to_send();
       send_encoded_message_to_esp8266();

       echo_info(new_message);
       state = off;
     }
  }
}


void pump_read()
{
  individual_pump_read(1, pump_one_state, 7);
  individual_pump_read(2, pump_two_state, 8);
  individual_pump_read(3, pump_three_state, 9);
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
    new_message = F("$FHC:");
  }
  else
  {
    new_message = F("$FHA:");
  }

  new_message += float_hub_id;
  new_message += F(":");
  new_message += FLOATHUB_PROTOCOL_VERSION;
  new_message += F("$");
  if(gps_valid == true || timeStatus() != timeNotSet )
  {
     add_timestamp_to_string(new_message);
  }
  
  new_message += F(",T:");
  append_float_to_string(new_message, temperature);

  new_message += F(",P:");
  append_float_to_string(new_message, pressure);

  if(gps_valid == true && gps_altitude.length() > 0)
  {
    new_message += F(",L:");
    new_message += gps_latitude;
    
    new_message += F(",O:");
    new_message += gps_longitude;

    new_message += F(",A:");
    new_message += gps_altitude;

    new_message += F(",H:");
    new_message += gps_hdp;
    
    new_message += F(",S:");
    new_message += gps_sog;

    new_message += F(",B:");
    new_message += gps_bearing_true;
  }

  if(gps_siv.length() > 0)
  {
    new_message += String(F(",N:"));
    new_message += gps_siv;
  }

  possibly_append_data(battery_one, 0.2, F(",V1:"));
  possibly_append_data(battery_two, 0.2, F(",V2:"));
  possibly_append_data(battery_three, 0.2, F(",V3:"));

  possibly_append_data(charger_one, 0.2, F(",C1:"));
  possibly_append_data(charger_two, 0.2, F(",C2:"));
  possibly_append_data(charger_three, 0.2, F(",C3:"));

  //
  //	Add NMEA data
  //
  
  possibly_append_data(nmea_speed_water, -0.5, F(",R:"));
  possibly_append_data(nmea_depth_water, -0.5, F(",D:"));
  possibly_append_data(nmea_wind_speed, -0.5, F(",J:"));
  possibly_append_data(nmea_wind_direction, -0.5, F(",K:"));
  possibly_append_data(nmea_water_temperature, -0.5, F(",Y:"));

  if(!console_only)
  {
    latest_message_to_send = new_message;
    encode_latest_message_to_send();
    send_encoded_message_to_esp8266();
  }
  echo_info(new_message);

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
    else if(esp8266_read_buffer.length() >= 30 && esp8266_read_buffer.substring(20).startsWith(F("i=")))
    {
      a_string = esp8266_read_buffer.substring(22,30);
      if(a_string != float_hub_id)
      {
        float_hub_id = a_string;
        write_eeprom_memory();
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
  // We could evesdrop here if we wanted to, but currently do not.  Just
  // push all input up the esp8266 on Serial1.
  //

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
  Serial.print(String(F("[")) + String(hour()) + F(":") + String(minute()) + F(":") + String(second()) + F("] "));
  Serial.print(some_info);

  Serial1.print(F("B=$FHD:"));
  Serial1.print(float_hub_id);
  Serial1.print(F(":"));
  Serial1.print(FLOATHUB_PROTOCOL_VERSION);
  Serial1.print(F("$    "));
  Serial1.print(String(F("[")) + String(hour()) + F(":") + String(minute()) + F(":") + String(second()) + F("] "));
  Serial1.print(some_info);

}

void debug_info(String some_info)
{
  debug_info_core(some_info);
  Serial.println();
  Serial1.println();
}

void debug_info(String some_info, float x)
{
  debug_info_core(some_info);
  Serial.println(x);
  Serial1.println(x);
}


void debug_info(String some_info, int x)
{
  debug_info_core(some_info);
  Serial.println(x);
  Serial1.println(x);
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
        digitalWrite(GPS_LED_1, LOW);
        digitalWrite(GPS_LED_2, HIGH);
        digitalWrite(COM_LED_1, LOW);
        digitalWrite(COM_LED_2, HIGH);
    }
    else 
    {
        digitalWrite(GPS_LED_1, HIGH);
        digitalWrite(GPS_LED_2, LOW);
        digitalWrite(COM_LED_1, HIGH);
        digitalWrite(COM_LED_2, LOW);
    }
    return;
  }


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
  debug_info(String(F("NMEA buf:")) + nmea_read_buffer);
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

  if( popout_nmea_value(F("DBT"), commas[2], commas[3], nmea_depth_water))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA d water:"), nmea_depth_water);
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
  
  else if(  popout_nmea_value(F("MWV"), commas[2], commas[3], nmea_wind_speed))
  {
    #ifdef NMEA_DEBUG_ON
    debug_info(F("NMEA w speed:"), nmea_wind_speed);
    #endif
    nmea_wind_speed_timestamp = millis();
    if(	popout_nmea_value(F("MWV"), commas[0], commas[1], nmea_wind_direction))
    {
      #ifdef NMEA_DEBUG_ON
      debug_info(F("NMEA w direction:"), nmea_wind_direction);
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


void update_hsnmea()
{
  while(soft_serial.available() && (int) hsnmea_read_buffer.length() < MAX_NMEA_BUFFER)
  {
    int incoming_byte = soft_serial.read();
    if(incoming_byte == '\n')
    {
      if(validate_nmea_buffer(true))
      {
        #ifdef SOFTSERIAL_DEBUG_ON
        debug_info("SoftSer good: " + String(hsnmea_read_buffer));
        #endif
        push_hsnmea_only_to_esp8266();   //   We just push out HS NMEA (AIS), we do _not_ parse any of it
      }
      #ifdef SOFTSERIAL_DEBUG_ON
      else
      {
        debug_info("SoftSer full: " + String(hsnmea_read_buffer));
      }
      #endif
      hsnmea_read_buffer = "";
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




bool try_and_send_encoded_message_to_esp8266()
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

  
  plain[0] = 'S';
  plain[1] = '=';
  if(Serial1.write(plain, 2) != 2)
  {
    return false;
  }


  for(i = 0; i < latest_message_to_send.length(); i = i + 32)
  {
    for(handy = 0; handy + i < latest_message_to_send.length() && handy < 32; handy++)
    {
      plain[handy] = (char) latest_message_to_send.charAt(i + handy);
    } 
    if(Serial1.write(plain, handy) != handy)
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
  if(Serial1.print(checksum) != 5)
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

  Serial1.println("factory");
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
  resetFunc();
}



void  send_encoded_message_to_esp8266()
{
  if(!try_and_send_encoded_message_to_esp8266())
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
  latest_message_to_send = F("$FHS:");
  latest_message_to_send += float_hub_id;
  latest_message_to_send += F(":");
  latest_message_to_send += FLOATHUB_ENCRYPT_VERSION;
  latest_message_to_send += F("$,");
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
  

  if(current_timestamp - hsnmea_previous_timestamp > hsnmea_update_interval)
  {
    hsnmea_previous_timestamp = current_timestamp;
    update_hsnmea();
  }
  
  /*
      Reporting routines
  */
  
  if(currently_active == true && currently_connected)
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
    if(current_timestamp - previous_idle_timestamp >  idle_reporting_interval)
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
    #ifdef DEBUG_MEMORY_ON
    print_free_memory();
    #endif
    pat_the_watchdog();	// Bump watchdog timer up to current time;
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
      }
      else if(millis() - user_reset_pin_timestamp > USER_RESET_REBOOT_TIME) 
      {
        dualReboot();
      }
      user_reset_pin_timestamp = 0; 
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




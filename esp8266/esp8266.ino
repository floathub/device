/*

 FloatHub ESP8266 Code
 (c) 2015-2018 Modiot Labs
 
*/


/*

   Many parts of this were based on logic from WiFi Manager by tzapu:

   https://github.com/tzapu/WiFiManager

   which was, in turn, inspired by:

   http://www.esp8266.com/viewtopic.php?f=29&t=2520
   https://github.com/chriscook8/esp-arduino-apboot
   https://github.com/esp8266/Arduino/tree/esp8266/hardware/esp8266com/esp8266/libraries/DNSServer/examples/CaptivePortalAdvanced

*/

//
//	Debugging options
//

//#define HTTP_DEBUG_ON
//#define MDNS_DEBUG_ON
//#define STAT_DEBUG_ON
//#define INPT_DEBUG_ON
//#define WIFI_DEBUG_ON
//#define FILE_DEBUG_ON
//#define FILE_SERVE_ON	// Useful when debuggig to see SPIFF files from a browser
//#define CELL_DEBUG_ON
//#define AISR_DEBUG_ON

//
//  Have to have a separate string for cellular debugging as cellular stuff
// is called on an interrupt so we can'd directly print stuff out
// (especially over virtual serial) during the interrupt call
//

#ifdef CELL_DEBUG_ON
String cellular_debug_string;
#endif

//
//  Global defines
//

#define MAX_COOKIES 10
//#define CELLULAR_CODE_ON	


#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <FS.h>
#include "./libs/AES/AES.h"
#include "./libs/Base64/Base64.h"
#include "static.h"
#include "version_defines.h"

#ifdef CELLULAR_CODE_ON
#include "./libs/SPISlave/SPISlave.h"
#endif


//
// When we do AES encryption (to relay AIS data)
//

#define MAX_AES_CIPHER_LENGTH 800
AES aes;
byte iv[16];
byte volatile_iv[16];
byte plain_text[MAX_AES_CIPHER_LENGTH];
byte cipher_text[MAX_AES_CIPHER_LENGTH];



//
//	Need access to lower level calls to be able to turn DHCP on and off
//

extern "C" {
#include "c_types.h"
#include "ets_sys.h" 
#include "os_type.h"
#include "osapi.h"  
#include "mem.h"
#include "user_interface.h"
#include "smartconfig.h"
#include "lwip/opt.h"
#include "lwip/err.h"
#include "lwip/dns.h"
}


//
//  User settable values (almost anything in this list can be changed via the web interface
// or the cli)
//


//
//	Wireless Related
//

String public_wifi_ssid;
String public_wifi_password;
String private_wifi_ssid;
String private_wifi_password;
String mdns_name;
bool	public_ip_is_static = false;
bool	called_mdns_after_connection = false;
bool	tried_public_wifi_recently = false;
IPAddress public_static_ip;
IPAddress public_static_gate;
IPAddress public_static_mask;
IPAddress public_static_dns;


//
//	Account Related 
//

String        float_hub_id;			// default: factoryX
byte          float_hub_aes_key[16]; 

//
//	Other Settings
//

bool	nmea_mux_on;				// default: yes
bool    nmea_mux_private;			// default: no
bool    relay_ais_data;				// default: yes
bool    relay_ais_cellular;		        // default: no
bool    force_wifi_connect;			// default: no
unsigned int  nmea_mux_port;			// default: 2319
String	web_interface_username;			// default: floathub
String	web_interface_password;			// default: floathub


//
//	Advanced (hidden) settings
//

bool	      phone_home_on;			// default: yes
String        float_hub_server;			// default: fdr.floathub.net
unsigned int  float_hub_server_port;		// default: 50003
bool	      virtual_serial_on;		// default: no
unsigned int  virtual_serial_port;		// default: 1923

unsigned long boot_counter;			


//
//	Other global stuff
//

#define MAX_LATEST_MESSAGE_SIZE 512
String latest_message_to_send = "";
WiFiClient fdr_client;

#define MAX_WIFI_READ_BUFFER_SIZE 32
String wifi_read_buffer;

enum communication_state
{
    idle,
    waiting_for_response
};
        
communication_state current_communication_state = idle; 
int  serial_ready_pin = 5;

bool filesystem_is_working;
bool latest_message_from_file;
long unsigned int low_file_pointer;
long unsigned int high_file_pointer;
#define MIN_SPIFF_SPACE	4096

//
//  Cellular status
//
#ifdef CELLULAR_CODE_ON
bool cellular_link_up = 0;
unsigned long cellular_link_timestamp = 0;
bool cellular_link_ready = 0;
String cellular_provider;
String sim_card_number;
String latest_cellular_message_to_send = "";
#endif

//
//	Struct to hold our authentication cookies
//

struct COOKIES
{
  unsigned long time;
  IPAddress address;
  long random_number;
  bool valid;
};

//
//	Now we create a bunch of 'em
//

COOKIES cookies[MAX_COOKIES];


//
//	Timing for all the various activitues 
//


unsigned long house_keeping_interval     	   = 3000;  // Do house keeping every 3 seconds
unsigned long wifi_housekeeping_interval     	   = 20000; // Do wifi house keeping three times a minute 
unsigned long nmea_housekeeping_interval 	   = 1000;  // Check on nmea connections every second	
unsigned long virtual_serial_housekeeping_interval = 500;   // Check on virtual serial connections every 1/2 second
unsigned long console_read_interval		   = 10;    // Check on "console" (mostly stuff from main board) every 1/100th of a second
unsigned long fdr_communications_interval	   = 100;   // Check on status of pushing out FDR messages every 1/10 of a second
unsigned long heartbeat_interval		   = 1000;  // Once a second, send heartbeat update 
unsigned long network_interval			   = 200;   // Every 1/5th of a second, look at web requests/etc
int           heartbeat_cycle			   = 0;     // Which heartbeat value to send
int	      wifi_housekeeping_cycle_counter	   = 1;     // Cycle counter for WiFi house keeping

unsigned long house_keeping_previous_timestamp = 0;
unsigned long wifi_housekeeping_previous_timestamp = 0;
unsigned long nmea_housekeeping_previous_timestamp = 0;
unsigned long virtual_serial_housekeeping_previous_timestamp = 0;
unsigned long console_previous_timestamp = 0; 
unsigned long fdr_communications_previous_timestamp = 0;
unsigned long heartbeat_previous_timestamp = 0;
unsigned long network_previous_timestamp = 0; 


//
// Servers of many types and size for such a small little microprocessor :-)
//

#define NUMB_NMEA_CLIENTS 4

ESP8266WebServer  web_server ( 80 );
WiFiServer       *nmea_server = 0;
WiFiClient        nmea_client[NUMB_NMEA_CLIENTS];
WiFiServer	 *virtual_serial_server = 0;
WiFiClient	  virtual_serial_client;
unsigned long     expected_fdr_response_time;
#define		  FDR_RESPONSE_MAX	15000UL  // If we don't hear back within 15 seconds, assume message did not get through


//
//	Some buffers for reading the above
//


#define MAX_CONSOLE_BUFFER 1024

String  console_read_buffer;
String	virtual_serial_read_buffer;

//
//	Watchdog stuff
//

unsigned watchdog_timestamp;
#define  WATCHDOG_TIME_LIMIT 21600000UL		    // If we do not manage to get any data through to FDR after 6 hours, reboot the whole shebang 
#define  WATCHDOG_USE_CELLULAR_TIME_LIMIT 30000UL   // After boot up, give it 30 seconds


//
//  Flag to signal that we need to jump out of any SIPSlave callbacks
// _super_ quick if we are doing SPIFF stuff
//

bool busy_doing_spiffs_stuff = false;

void startNoInterrupts()
{
  noInterrupts();
  busy_doing_spiffs_stuff = true; 
}

void endNoInterrupts()
{
  interrupts();
  busy_doing_spiffs_stuff = false; 
}

void help_info(String some_info)
{
  Serial.print(F("$FHH:"));
  Serial.print(float_hub_id);
  Serial.print(F(":"));
  Serial.print(FLOATHUB_PROTOCOL_VERSION);
  Serial.print(F("$    "));
  Serial.println(some_info);

  if(virtual_serial_client.connected())
  {
    virtual_serial_client.print(F("$FHH:"));
    virtual_serial_client.print(float_hub_id);
    virtual_serial_client.print(F(":"));
    virtual_serial_client.print(FLOATHUB_PROTOCOL_VERSION);
    virtual_serial_client.print(F("$    "));
    virtual_serial_client.println(some_info);
  }

}

void internal_info(String some_info)
{

  //
  // Send only on internal console and preface with FHI (used to send
  // config/settings the esp8266 has to share with the Mega)
  //

  Serial.print(F("$FHI:"));
  Serial.print(float_hub_id);
  Serial.print(F(":"));
  Serial.print(FLOATHUB_PROTOCOL_VERSION);
  Serial.print(F("$    "));
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

  if(virtual_serial_client.connected())
  {
    virtual_serial_client.print(F("$FHD:"));
    virtual_serial_client.print(float_hub_id);
    virtual_serial_client.print(F(":"));
    virtual_serial_client.print(FLOATHUB_PROTOCOL_VERSION);
    virtual_serial_client.print(F("$    "));
    virtual_serial_client.print(some_info);
  }
}

void debug_info(String some_info)
{
  debug_info_core(some_info);
  Serial.println();
  if(virtual_serial_client.connected())
  {
    virtual_serial_client.println();
  }
}

void debug_info(String some_info, float x)
{
  debug_info_core(some_info);
  Serial.println(x);

  if(virtual_serial_client.connected())
  {
    virtual_serial_client.println(x);
  }
}


void debug_info(String some_info, int x)
{
  debug_info_core(some_info);
  Serial.println(x);

  if(virtual_serial_client.connected())
  {
    virtual_serial_client.println(x);
  }
}

void nukeCookie(int which_one)
{
  if(which_one >= MAX_COOKIES)
  {
    which_one = MAX_COOKIES - 1 ;
  }
  if(which_one < 0)
  {
    which_one = 0;
  }
  cookies[which_one].time = 0; 
  cookies[which_one].address = IPAddress(0,0,0,0);
  cookies[which_one].random_number = 0; 
  cookies[which_one].valid = false;
}



//
//	EEPROM routines to factory init, read, and write so we have settings
// persistence between reboots, power cycles.
//

void writeStringToEEPROM(int location, String the_string, unsigned int max_length_not_including_terminating_zero = 32)
{
  int i;
  for(i = 0; i < std::min(the_string.length(), max_length_not_including_terminating_zero); i++)
  {
    EEPROM.write(location + i, the_string.charAt(i));
  }
  EEPROM.write(location + i, '\0');
}

void readStringFromEEPROM(int location, String &the_string, unsigned int max_length_not_including_terminating_zero = 32)
{
  char next_char;
  the_string = "";
  for(int i = 0; i < max_length_not_including_terminating_zero; i++)
  {
    next_char = EEPROM.read(location + i);
    if(next_char == '\0')
    {
      break;
    }
    the_string += next_char;
  }
}

void init_eeprom_memory()
{

  int i;

  //
  //  This function is called only during factory reset or very first startup
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
  //  Set default server
  //


  writeStringToEEPROM(18, F("fdr.floathub.net"));
  EEPROM.write(50, '\0');
  
  //
  //  Set default port of 50003
  //
  
  EEPROM.write(51, 195);
  EEPROM.write(52, 83);

  //
  //  Default public WiFi SSID & Password
  //

  writeStringToEEPROM(53, F("factoryX"));
  EEPROM.write(85, '\0');
  
  writeStringToEEPROM(86, F("factoryX"));
  EEPROM.write(118, '\0');
  
  
  //
  //  Default private WiFi SSID & Password
  //


  byte mac_array[6];
  WiFi.softAPmacAddress(mac_array);

  private_wifi_ssid = "FloatHub_";
  if(mac_array[3] < 16)
  {
    private_wifi_ssid += 0;
  }
  private_wifi_ssid += String(mac_array[3], HEX);
  if(mac_array[4] < 16)
  {
    private_wifi_ssid += 0;
  }
  private_wifi_ssid += String(mac_array[4], HEX);
  if(mac_array[5] < 16)
  {
    private_wifi_ssid += 0;
  }
  private_wifi_ssid += String(mac_array[5], HEX);


  writeStringToEEPROM(119, private_wifi_ssid);
  EEPROM.write(151, '\0');
  
  writeStringToEEPROM(152, F("floathub"));
  EEPROM.write(184, '\0');
  
  //
  //  MDNS .local name 
  //

  writeStringToEEPROM(185, F("floathub"));
  EEPROM.write(217, '\0');
  
  //
  // Is public IP address static (no)
  //

  EEPROM.write(218, 0);

  //
  // Static IP, gateway, mask, dns 
  //

  EEPROM.write(219, 1); EEPROM.write(220, 2); EEPROM.write(221, 3); EEPROM.write(222, 4); 
  EEPROM.write(223, 5); EEPROM.write(224, 6); EEPROM.write(225, 7); EEPROM.write(226, 8); 
  EEPROM.write(227, 255); EEPROM.write(228, 255); EEPROM.write(229, 255); EEPROM.write(230, 0); 
  EEPROM.write(231, 8); EEPROM.write(232, 8); EEPROM.write(233, 8); EEPROM.write(234, 8); 

  //
  //  Set to some kind of default AES key
  //
  
  EEPROM.write(235, 0x00);
  EEPROM.write(236, 0x01);
  EEPROM.write(237, 0x02);
  EEPROM.write(238, 0x03);
  EEPROM.write(239, 0x04);
  EEPROM.write(240, 0x05);
  EEPROM.write(241, 0x06);
  EEPROM.write(242, 0x07);
  EEPROM.write(243, 0x08);
  EEPROM.write(244, 0x09);
  EEPROM.write(245, 0x0A);
  EEPROM.write(246, 0x0B);
  EEPROM.write(247, 0x0C);
  EEPROM.write(248, 0x0D);
  EEPROM.write(249, 0x0E);
  EEPROM.write(250, 0x0F);


  //
  // NMEA muxer flag, port of 2319, private, ais relay, cellular ais relay,
  // force cellular 
  //

  EEPROM.write(251, 1);
  EEPROM.write(252, 9);
  EEPROM.write(253, 15);
  EEPROM.write(324, 0);
  EEPROM.write(325, 1);
  EEPROM.write(326, 0);
  EEPROM.write(327, 0);
 
  //
  // Web interface username and password
  //

  writeStringToEEPROM(254, F("floathub"));
  EEPROM.write(286, '\0');
  
  writeStringToEEPROM(287, F("floathub"));
  EEPROM.write(319, '\0');

  //
  //  Phone home flag (yes)
  //

  EEPROM.write(320, 1);

  //
  //  Virtual serial flag and port of 1923
  //

  EEPROM.write(321, 0);
  EEPROM.write(322, 7);
  EEPROM.write(323, 131);

  //
  //  Do this last to show EEPROM set
  //

  for(i = 0; i < 6; i++)
  {
    EEPROM.write(i, 42);
  }
  
  //
  //  For ESP8266, this does not "take" unless you commit
  //

  EEPROM.commit();
}


void write_eeprom_memory()
{
  int i; 
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
  //  Store server
  //
  
  writeStringToEEPROM(18, float_hub_server);
  
  //
  //  Store port
  //
  
  EEPROM.write(51, highByte(float_hub_server_port));
  EEPROM.write(52, lowByte(float_hub_server_port));
  
  //
  //  Public/Private WiFi SSID & Password
  //

  writeStringToEEPROM( 53, public_wifi_ssid);
  writeStringToEEPROM( 86, public_wifi_password);
  writeStringToEEPROM(119, private_wifi_ssid);
  writeStringToEEPROM(152, private_wifi_password);

  //
  // MDNS .local name
  //

  writeStringToEEPROM(185, mdns_name);


  //
  // public ip static flag
  //

  EEPROM.write(218, public_ip_is_static);

  //
  // Static IP values
  //

  EEPROM.write(219, public_static_ip[0]);   EEPROM.write(220, public_static_ip[1]);     EEPROM.write(221, public_static_ip[2]);     EEPROM.write(222, public_static_ip[3]); 
  EEPROM.write(223, public_static_gate[0]); EEPROM.write(224, public_static_gate[1]);   EEPROM.write(225, public_static_gate[2]);   EEPROM.write(226, public_static_gate[3]); 
  EEPROM.write(227, public_static_mask[0]); EEPROM.write(228, public_static_mask[1]);   EEPROM.write(229, public_static_mask[2]);   EEPROM.write(230, public_static_mask[3]); 
  EEPROM.write(231, public_static_dns[0]);  EEPROM.write(232, public_static_dns[1]);    EEPROM.write(233, public_static_dns[2]);    EEPROM.write(234, public_static_dns[3]); 

  //
  //  Store AES key
  //
  
  for(i = 0; i < 16; i++)
  {
    EEPROM.write(235 + i, float_hub_aes_key[i]);
  }

  //
  // NMEA muxer flag and port
  //

  EEPROM.write(251, nmea_mux_on);
  EEPROM.write(252, highByte(nmea_mux_port));
  EEPROM.write(253, lowByte(nmea_mux_port));
  EEPROM.write(324, nmea_mux_private);
  EEPROM.write(325, relay_ais_data);
  EEPROM.write(326, relay_ais_cellular);
  EEPROM.write(327, force_wifi_connect);
 
  //
  // Web interface username and password
  //

  writeStringToEEPROM(254, web_interface_username);
  writeStringToEEPROM(287, web_interface_password);

  //
  //  Phone home flag
  //

  EEPROM.write(320, phone_home_on);

  //
  //  Virtual serial flag and port of 1923
  //

  EEPROM.write(321, virtual_serial_on);
  EEPROM.write(322, highByte(virtual_serial_port));
  EEPROM.write(323, lowByte(virtual_serial_port));

  
  //
  //  For ESP8266, this does not "take" unless you commit
  //

  EEPROM.commit();

}



void read_eeprom_memory()
{
  int i;
  char next_char;
  byte byte_zero, byte_one, byte_two, byte_three;

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
  EEPROM.commit();
  
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
  //  Server port
  //

  float_hub_server_port  = EEPROM.read(52);
  float_hub_server_port += EEPROM.read(51) * 256;
  

  //
  //  Read Some Strings
  //

  readStringFromEEPROM( 18, float_hub_server);
  readStringFromEEPROM( 53, public_wifi_ssid);
  readStringFromEEPROM( 86, public_wifi_password);
  readStringFromEEPROM(119, private_wifi_ssid);
  readStringFromEEPROM(152, private_wifi_password);
  readStringFromEEPROM(185, mdns_name);
  readStringFromEEPROM(254, web_interface_username);
  readStringFromEEPROM(287, web_interface_password);
  
  //
  // Static IP stuff
  //

  public_ip_is_static = EEPROM.read(218);

  public_static_ip   = IPAddress(EEPROM.read(219), EEPROM.read(220), EEPROM.read(221), EEPROM.read(222));
  public_static_gate = IPAddress(EEPROM.read(223), EEPROM.read(224), EEPROM.read(225), EEPROM.read(226)); 
  public_static_mask = IPAddress(EEPROM.read(227), EEPROM.read(228), EEPROM.read(229), EEPROM.read(230)); 
  public_static_dns  = IPAddress(EEPROM.read(231), EEPROM.read(232), EEPROM.read(233), EEPROM.read(234)); 

  //
  //  Read AES key
  //
  
  for(i = 0; i < 16; i++)
  {
    float_hub_aes_key[i] = EEPROM.read(235 + i);
  }    

  //
  //  NMEA Muxer stuff
  //

  nmea_mux_on = EEPROM.read(251);
  nmea_mux_port  = EEPROM.read(253);
  nmea_mux_port += EEPROM.read(252) * 256;
  nmea_mux_private = EEPROM.read(324);
  relay_ais_data = EEPROM.read(325);
  relay_ais_cellular = EEPROM.read(326);
  force_wifi_connect = EEPROM.read(327);
  
  //
  //  Phone home?
  // 
  
  phone_home_on = EEPROM.read(320);

  //
  //  Virtual Serial
  // 

  virtual_serial_on = EEPROM.read(321);
  virtual_serial_port  = EEPROM.read(323);
  virtual_serial_port += EEPROM.read(322) * 256;
  
}  

bool isAuthenticated()
{
  if (web_server.hasHeader("Cookie"))
  {   
    String cookie = web_server.header("Cookie");
    int cut_spot = cookie.indexOf("FHSESSION=");
    if (cut_spot != -1)
    {
      //
      //  If we find a valid FHSESSION cookie from the registered IP address, then this request is ok/authenticated
      //
 
      cut_spot += 10;
      bool ok = false;
      WiFiClient client = web_server.client();
      for(int i =0 ; i < MAX_COOKIES; i++)
      {
         if( cookie.substring(cut_spot) == String(cookies[i].random_number) &&
             cookies[i].valid == true &&
             client.remoteIP() == cookies[i].address
           )
         {
           //
           //  Authenticated, so we need to "touch" the timestamp for this cookie
           //
          
           cookies[i].time = millis();
  	   #ifdef HTTP_DEBUG_ON
           debug_info(F("Yay, Cookie hitttttt"));
           #endif
           return true;
         }
      }
    }
  }
  #ifdef HTTP_DEBUG_ON
  debug_info(F("Authentication Failed"));
  #endif
  return false;	
}


bool isOnPrivateNetwork()
{
  WiFiClient client = web_server.client();

  if(client.localIP() == WiFi.softAPIP())
  {
    return true;
  }
  return false;
}


void nukeNmeaServer()
{
  for(int i = 0; i < NUMB_NMEA_CLIENTS; i++)
  {
    if(nmea_client[i])
    {
      nmea_client[i].stop();
    }
  }
  if(nmea_server)
  {
    nmea_server->close();
    delete nmea_server;
    nmea_server = 0;	
  }
}


void fireUpNmeaServer()
{
  nukeNmeaServer();
  if(nmea_mux_private)
  {
    nmea_server = new WiFiServer(WiFi.softAPIP(), nmea_mux_port);
  }
  else
  {
    nmea_server = new WiFiServer(nmea_mux_port);
  }
  nmea_server->begin();
  nmea_server->setNoDelay(true);
}


void kickNMEA()
{

  if(nmea_mux_on)
  {
    fireUpNmeaServer();
  }
  else
  {
    nukeNmeaServer();
  }

}


void nukeVirtualSerialServer()
{
  if(virtual_serial_client.connected())
  {
      virtual_serial_client.stop();
  }
  if(virtual_serial_server)
  {
    virtual_serial_server->close();
    delete virtual_serial_server;
  }
  virtual_serial_server = 0;	
}


void fireUpVirtualSerialServer()
{
  nukeVirtualSerialServer();
  virtual_serial_server = new WiFiServer(virtual_serial_port);
  virtual_serial_server->begin();
  virtual_serial_server->setNoDelay(true);
}


void kickVirtualSerial()
{
  if(virtual_serial_on)
  {
    fireUpVirtualSerialServer();
  }
  else
  {
    nukeVirtualSerialServer();
  }
}


void spitOutIPInput(String &page, String name_stub, String label, const IPAddress& address)
{
 
  page += "<label for='" + name_stub + "1' style='width: 140px;'>" + label + "</label>";
  page += "<input type='number' name='" + name_stub + "1' id='" + name_stub + "1' length='3' maxlength='3' style='width: 60px' value='" + String(address[0]) + "' >";
  page += " . <input type='number' name='" + name_stub + "2' length='3' maxlength='3'  style='width: 60px' value='" + String(address[1]) + "' >";
  page += " . <input type='number' name='" + name_stub + "3' length='3' maxlength='3'  style='width: 60px' value='" + String(address[2]) + "' >";
  page += " . <input type='number' name='" + name_stub + "4' length='3' maxlength='3'  style='width: 60px' value='" + String(address[3]) + "' >";
  page += "<br>";
}

void checkPort(unsigned int &the_port)
{
  if(the_port > 65535)
  {
    the_port = 65535;
  }
  if(the_port < 1)
  {
    the_port = 1; 
  }
}


void sendPleaseWait(String where_to_go)
{
  String page = FPSTR(HTTP_INITA);
  page += "<meta http-equiv='refresh' content='8; " + where_to_go + "'>";
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Please Wait</h2>";
  page += "<p style='margin: 20px;'>Update in progress. Will attempt to redirect you in a few seconds ...</p>";

  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}


void handleLogin()
{
  #ifdef HTTP_DEBUG_ON
  debug_info(F("Enter handleLogin()"));
  #endif

  if (web_server.hasArg("DISCONNECT"))
  {
    #ifdef HTTP_DEBUG_ON
    debug_info(F("Disconnection"));
    #endif

    String header = "HTTP/1.1 301 OK\r\nSet-Cookie: FHSESSION=0\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
    web_server.sendContent(header);
    return;
  }
  if(isAuthenticated())
  {
      String header = "HTTP/1.1 301 OK\r\nLocation: /\r\nCache-Control: no-cache\r\n\r\n";
      web_server.sendContent(header);

      #ifdef HTTP_DEBUG_ON
      debug_info(F("redirect already logged in"));
      #endif

      return;
  }
  if (web_server.hasArg("USER") && web_server.hasArg("PASS"))
  {
    if (web_server.arg("USER") == web_interface_username &&  web_server.arg("PASS") == web_interface_password )
    {

      //
      // Find first invalid cookie and set it to authenticate this "session"
      //

      int cookie_to_use = MAX_COOKIES - 1;

      for(int i=0; i < MAX_COOKIES; i++)
      {
        if(!cookies[i].valid)
        {
	    cookie_to_use = i;
            break;
	}
      }

      WiFiClient client = web_server.client();

      cookies[cookie_to_use].valid = true;
      cookies[cookie_to_use].time = millis();
      cookies[cookie_to_use].address = client.remoteIP();
      cookies[cookie_to_use].random_number = random(1000000,10000000);

      String header = "HTTP/1.1 301 OK\r\nSet-Cookie: FHSESSION=" + String(cookies[cookie_to_use].random_number) + "\r\nLocation: /\r\nCache-Control: no-cache\r\n\r\n";
      web_server.sendContent(header);

      #ifdef HTTP_DEBUG_ON
      debug_info("Log in Successful");
      #endif

      return;
    }
  }


  //
  //  Guess we need to show the login page
  //

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);

  page += FPSTR(HTTP_LOGOA);
  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Device Login</h2>";
  page += "<form method='post' action='/login'>";
  page += "<input name='USER' length='32' placeholder='username' autofocus><br>";
  page += "<input name='PASS' length='64' type='password' placeholder='password'><br><br>";
  page += "<button type='submit'>Login</button></form></div>";

  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}


void handleReboot()
{
  #ifdef HTTP_DEBUG_ON
  debug_info(F("Enter handleReboot()"));
  #endif

  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  sendPleaseWait("/");

  delay(200);
  ESP.restart();  
  ESP.wdtDisable();
}


void handleRoot()
{
  #ifdef HTTP_DEBUG_ON
  debug_info(F("Enter handleRoot()"));
  #endif

  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  WiFiClient client = web_server.client();

  #ifdef HTTP_DEBUG_ON
  debug_info("Request from " + client.remoteIP());
  #endif

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);

  String private_address = WiFi.softAPIP().toString();
  String public_address = WiFi.localIP().toString();

  page += FPSTR(HTTP_LOGOA);

  if(isOnPrivateNetwork())
  {
    page += "<h5>" + public_address + " | " + mdns_name + ".local | <a href='http://" + private_address + "'>" + private_address + "</a></h5>";
  }
  else
  {
    page += "<h5><a href='http://" + public_address + "'>" + public_address + "</a> | <a href='http://" + mdns_name + ".local'>" + mdns_name + ".local</a> | " + private_address + "</h5>";
  }


  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Device Settings</h2>";

  page += "<form method='post' action='/public'>";
  page += "<button type='submit'>Public Wireless Settings</button></form>";

  page += "<form method='post' action='/private'>";
  page += "<button type='submit'>Private Wireless Settings</button></form>";


  page += "<form method='post' action='/account'>";
  page += "<button type='submit'>Account Settings</button></form>";

  page += "<form method='post' action='/other'>";
  page += "<button type='submit'>Other Settings</button></form>";

  page += "<form method='post' action='/login'>";
  page += "<button type='submit' name='DISCONNECT'>Logout</button></form>";

  page += "</div>";

  #ifdef CELLULAR_CODE_ON
  if(cellular_link_up)
  {
    page += "<br/><h5>Up | ";
  }
  else
  {
    page += "<br/><h5>Down | ";
  }
  page += cellular_provider + " | " + sim_card_number + "</h5>";
  #endif


  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}


void handlePrivateWireless()
{
  #ifdef HTTP_DEBUG_ON
  debug_info(F("Enter handlePrivateWireless()"));
  #endif

  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  //
  // First see if we received any form submission
  // 

  String error_message = "";

  if(web_server.hasArg("privssid") && web_server.arg("savebutton") == "True")
  {

    //
    //  What is submitted ssid and password; do something only if it has changed
    //

    if(web_server.arg("privpassone") == web_server.arg("privpasstwo"))
    { 
      if(web_server.arg("privssid").length() < 1 )
      {
        error_message += "Network Name cannot be blank.";
      }
      else if(web_server.arg("privpassone").length() < 8) 
      {
	error_message += "Password must be 8+ characters";
      }
      else
      {
        sendPleaseWait("/private");
        private_wifi_ssid = web_server.arg("privssid");
        private_wifi_password = web_server.arg("privpassone");
        WiFi.softAP(private_wifi_ssid.c_str(), private_wifi_password.c_str());
        write_eeprom_memory();
      }
    }
    else
    {
      error_message += "The passwords do not match";
    }

  }

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
  page += FPSTR(HTTP_LOGOA);
  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Private Wireless Network</h2>";
  page += "<h4>" + error_message + "</h4>";

  page += "<form method='post' action='/private'>";
  page += "<label for='privssid'>Network Name: </label>";
  page += "<input name='privssid' value='" + private_wifi_ssid + "' autofocus><br>";
  page += "<label for='privpassone'>Password: </label>";
  page += "<input name='privpassone' type='password'><br>";
  page += "<label for='privpasstwo'>Repeat: </label>";
  page += "<input name='privpasstwo' type='password'><br>";
  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br>";

  page += FPSTR(HTTP_HOMEB);
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}


void kickMDNS(bool ap_side = false)
{
  int attempts = 0;

  IPAddress which_address = WiFi.localIP();
  if(ap_side)
  {
    which_address = WiFi.softAPIP();
  }

  #ifdef MDNS_DEBUG_ON
  debug_info("MDNS binding " + mdns_name + " --> " + which_address.toString());
  #endif

  while((!MDNS.begin (mdns_name.c_str(), which_address)) && attempts < 12)
  {
    #ifdef MDNS_DEBUG_ON
    debug_info("MDNS died off ... will retry");
    #endif

    delay(500);
    attempts++;
    if(attempts >= 12)
    {
      #ifdef MDNS_DEBUG_ON
      debug_info("Can't get MDNS up at all");
      #endif
    }
  }
}


void kickWiFi(bool ap_only=false)
{
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  delay(500);

  if(public_wifi_ssid == "factoryX" || ap_only == true)
  {
    #ifdef WIFI_DEBUG_ON
    debug_info(F("Kicking AP WiFi"));
    #endif
    tried_public_wifi_recently = false;
    WiFi.setAutoConnect(false);
    //WiFi.mode(WIFI_AP);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(private_wifi_ssid.c_str(), private_wifi_password.c_str());
    kickMDNS(true);
  }
  else
  {
    #ifdef WIFI_DEBUG_ON
    debug_info(F("Kicking Full WiFi"));
    #endif
    tried_public_wifi_recently = true;
    WiFi.setAutoConnect(true);
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(public_wifi_ssid.c_str(), public_wifi_password.c_str());
    WiFi.softAP(private_wifi_ssid.c_str(), private_wifi_password.c_str());
    byte attempts = 0; 
    while (WiFi.status() != WL_CONNECTED && attempts < 24)
    { 
      delay(250);
      attempts++;
    }

    //wifi_station_dhcpc_stop();
    if(public_ip_is_static)
    {
      WiFi.config(public_static_ip, public_static_gate, public_static_mask, public_static_dns);
    }
    //else
    //{
    //  wifi_station_dhcpc_start();
    //}
  }

  called_mdns_after_connection = false;
	  
}


void handlePublicWireless()
{
  #ifdef HTTP_DEBUG_ON
  debug_info(F("Enter handlePublicWireless()"));
  #endif

  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  //
  // First see if we received any form submission
  // 

  String error_message = "";

  if(web_server.hasArg("pubadd") && web_server.arg("savebutton") == "True")
  {

    //
    //  We are autenticated and a form was submitted, better process and act on it
    //

    //
    //  What is submitted ssid and password; do something only if it has changed
    //

    if(web_server.arg("pubpassone") == web_server.arg("pubpasstwo"))
    { 
      if(web_server.arg("pubssid").length() < 1 )
      {
        error_message += "Network Name cannot be blank.";
      }
      else
      {
        if(web_server.arg("pubadd") == "dynamic")
        {
          public_ip_is_static = false;
          // Set Public Wireless for DHCP
          public_wifi_ssid = web_server.arg("pubssid");
          if(web_server.arg("pubpassone").length() > 1 )
          {
              public_wifi_password = web_server.arg("pubpassone");
          }
          
          write_eeprom_memory();
          sendPleaseWait("/public");
 	  kickWiFi();
          return;
        }
        else
        {
          // Set Public Wireless for Static 

          public_ip_is_static = true;
          public_wifi_ssid = web_server.arg("pubssid");
          //
          //  If no password, we're ok. If something set, change to that
          //

          if(web_server.arg("pubpassone").length() > 1 )
          {
              public_wifi_password = web_server.arg("pubpassone");
          }

          public_static_ip   = IPAddress(web_server.arg("local1").toInt(), web_server.arg("local2").toInt(), web_server.arg("local3").toInt(), web_server.arg("local4").toInt());
          public_static_gate = IPAddress(web_server.arg("gate1").toInt(), web_server.arg("gate2").toInt(), web_server.arg("gate3").toInt(), web_server.arg("gate4").toInt());
          public_static_mask = IPAddress(web_server.arg("mask1").toInt(), web_server.arg("mask2").toInt(), web_server.arg("mask3").toInt(), web_server.arg("mask4").toInt());
          public_static_dns  = IPAddress(web_server.arg("dns1").toInt(), web_server.arg("dns2").toInt(), web_server.arg("dns3").toInt(), web_server.arg("dns4").toInt());


          write_eeprom_memory();
          sendPleaseWait("http://" + public_static_ip.toString() + "/public");
          kickWiFi();
          return;
        }
      }
    }
    else
    {
      error_message += "The passwords do not match";
    }

  }

  //
  //  Build an array of available WiFi names, max 12, no dupes.
  //

  String wifi_names[12];
  byte num_names = 0; 

  byte numSsid = WiFi.scanNetworks();
  for (int i = 0; i < numSsid; i++)
  {
    bool match = false;
    for(int j = 0; j < num_names; j++)
    {
      if (wifi_names[j] == String(WiFi.SSID(i)))
      {
        match = true;
        break;
      }
    }
    if(!match && num_names < 12)
    {
      wifi_names[num_names] = String(WiFi.SSID(i));
      num_names++;
    }
  }

  //
  // Now build the page to send
  //

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);

  page += FPSTR(HTTP_LOGOA);

  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Public Wireless Network</h2>";
  page += "<h4>" + error_message + "</h4>";

  page += "<form method='post' action='/public'>";
  page += "<label for='pubssid'>Network Name: </label>";
  page += "<input name='pubssid' value='" + public_wifi_ssid + "' list='available'><br>";

  //
  //  On an html 5 browser there will also be drop down for other values
  //

  page += "<datalist id='available'>";
  for (int i = 0; i < num_names; i++)
  {
    page += "<option>" + wifi_names[i] + "</option>";
  }
  page += "</datalist>";

  page += "<label for='pubpassone'>Password: </label>";
  page += "<input name='pubpassone' type='password' autofocus><br>";
  page += "<label for='pubpasstwo'>Repeat: </label>";
  page += "<input name='pubpasstwo' type='password'><br>";
  page += "<input type='radio' name='pubadd' value='dynamic' id='dynamic' ";
  if(!public_ip_is_static)
  {
    page += "checked ";
  }
  page += "style='width: 20px; vertical-align: middle;'>";
  page += "<label for='dynamic' style='width: 240px; text-align: left;'>Dynamic Addressing (DHCP)</label><br>";
  page += "<input type='radio' name='pubadd' value='static' id='static' ";
  if(public_ip_is_static)
  {
    page += "checked ";
  }
  page += "style='width: 20px; vertical-align: middle;'>";
  page += "<label for='static' style='width: 240px; text-align: left;'>Static Addressing</label><br><br>";

  //
  //	If it's not DHCP, we need local IP, gateway, mask, and dns 
  //

  page += "<div>";


  spitOutIPInput(page, "local", "Local IP Address: ", public_static_ip);
  spitOutIPInput(page, "gate", "Gateway: ", public_static_gate);
  spitOutIPInput(page, "mask", "Mask: ", public_static_mask);
  spitOutIPInput(page, "dns", "DNS: ", public_static_dns);


  page += "</div>";

  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br>";

  page += FPSTR(HTTP_HOMEB);
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}


void handleOther()
{
  #ifdef HTTP_DEBUG_ON
  debug_info(F("Enter handleOther()"));
  #endif

  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  bool nmea_changed = false;
  String error_message = "";

  if(web_server.arg("savebutton") == "True")
  {
    bool something_changed = false;
    if(web_server.arg("lpassone") != web_server.arg("lpasstwo"))
    {
      error_message += F("The password do not match");
    }
    else if(web_server.arg("lpassone").length() > 0) 
    {
      if(web_server.arg("lpassone").length() < 4)
      {
        error_message += F("Password must be 4+ characters");
      }
      else
      {
        something_changed = true;
        web_interface_password = web_server.arg("lpassone");
      }
    }

    if(web_server.arg("lname").length() < 4)
    {
      error_message += "Login must be 4+ characters";
    }
    else
    {
      something_changed = true;
      web_interface_username = web_server.arg("lname");  
    }

    if(web_server.arg("muxon") == "yes" && !nmea_mux_on)
    {
      nmea_mux_on = true;
      something_changed = true;
      nmea_changed = true;
    }
    else if(nmea_mux_on && web_server.arg("muxon") != "yes")
    {
      nmea_mux_on = false;
      something_changed = true;
      nmea_changed = true; 
    }

    if(web_server.arg("muxprivate") == "yes" && !nmea_mux_private)
    {
      nmea_mux_private = true;
      something_changed = true;
      nmea_changed = true;
    }
    else if(nmea_mux_private && web_server.arg("muxprivate") != "yes")
    {
      nmea_mux_private = false;
      something_changed = true;
      nmea_changed = true; 
    }

    if(web_server.arg("relayais") == "yes" && !relay_ais_data)
    {
      relay_ais_data = true;
      something_changed = true;
    }
    else if(relay_ais_data && web_server.arg("relayais") != "yes")
    {
      relay_ais_data = false;
      something_changed = true;
    }

    if(web_server.arg("relayaiscell") == "yes" && !relay_ais_cellular)
    {
      relay_ais_cellular = true;
      something_changed = true;
    }
    else if(relay_ais_cellular && web_server.arg("relayaiscell") != "yes")
    {
      relay_ais_cellular = false;
      something_changed = true;
    }

    if(nmea_mux_port != web_server.arg("muxport").toInt())
    {
      nmea_mux_port = web_server.arg("muxport").toInt();
      checkPort(nmea_mux_port);
      something_changed = true;
      nmea_changed = true;
    }
    if(web_server.arg("localname").length() < 1)
    {
      error_message += ".local name cannot be blank";
    }
    else if(web_server.arg("localname") != mdns_name)
    {
       mdns_name = web_server.arg("localname");
       kickMDNS();
       something_changed = true;
    }

    if(something_changed)
    {
      write_eeprom_memory();
    }
    if(nmea_changed)
    {
      error_message += F("Device reboot recommended for NMEA port changes");
      kickNMEA();
    }
  }


  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
  page += FPSTR(HTTP_LOGOA);
  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Other Settings</h2>";
  page += "<h4>" + error_message + "</h4>";

  page += "<form method='post' action='/other'>";

  page += "<label for='lname'>Device Login: </label>";
  page += "<input name='lname' value='" + web_interface_username + "' autofocus><br>";
  page += "<label for='lpassone'>Device Password: </label>";
  page += "<input name='lpassone' type='password' maxlength='32'><br>";
  page += "<label for='lpasstwo'>Repeat: </label>";
  page += "<input name='lpasstwo' type='password' maxlength='32'><br>";
  page += "<label for='localname'>.local Name: </label>";
  page += "<input name='localname' value='" + mdns_name + "'><br>";

  page += "<label for='muxon'>NMEA Output: </label>";
  page += "<input type='checkbox' name='muxon' value='yes'";
  if(nmea_mux_on)
  {
    page += " checked";
  }
  page += "><br>";
  page += "<label for='muxport'>NMEA Port: </label>";
  page += "<input type='number' name='muxport' length='5' maxlength='5' value='" + String(nmea_mux_port) + "' ><br>";
  page += "<label for='muxprivate'>Only Private NMEA: </label>";
  page += "<input type='checkbox' name='muxprivate' value='yes'";
  if(nmea_mux_private)
  {
    page += " checked";
  }
  page += "><br>";
  page += "<label for='relayais'>Relay AIS: </label>";
  page += "<input type='checkbox' name='relayais' value='yes'";
  if(relay_ais_data)
  {
    page += " checked";
  }
  page += "><br>";

  #ifdef CELLULAR_CODE_ON
  page += "<label for='relayaiscell'>AIS over Cellular: </label>";
  page += "<input type='checkbox' name='relayaiscell' value='yes'";
  if(relay_ais_cellular)
  {
    page += " checked";
  }
  page += "><br>";
  #endif

  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br>";

  page += FPSTR(HTTP_HOMEB);
  if(nmea_changed)
  {
    page += FPSTR(HTTP_RBOOT);
  }
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}

void handleAdvanced()
{
  //
  // No obvious way to arrive at this page so it is "secret", but note that
  // user must still first be authenticated like any other page.
  //

  #ifdef HTTP_DEBUG_ON
  debug_info(F("Enter handleAdvanced()"));
  #endif


  if(!isAuthenticated())
  {
    String header = "HTTP/1.1 301 OK\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
    web_server.sendContent(header);
    return;
  }

  String error_message = "";
  bool virtual_serial_changed = false;


  if(web_server.arg("savebutton") == "True")
  {
    if(web_server.arg("phoneon") == "yes")
    {
      phone_home_on = true;
    }
    else
    {
      phone_home_on = false;
    }
    if(web_server.arg("fhserver").length() < 1)
    {
      error_message += "FloatHub Server cannot be blank";
    }
    else
    {
      float_hub_server = web_server.arg("fhserver");
    }
    float_hub_server_port = web_server.arg("fhport").toInt();
    checkPort(float_hub_server_port);

    if(web_server.arg("vserialon") == "yes" && !virtual_serial_on)
    {
      virtual_serial_on = true;
      virtual_serial_changed = true;
    }
    else if (virtual_serial_on && web_server.arg("vserialon") != "yes")
    {
      virtual_serial_on = false; 
      virtual_serial_changed = true;
    }  

    if(web_server.arg("forcewifi") == "yes" && !force_wifi_connect)
    {
      force_wifi_connect = true;
    }
    else if (force_wifi_connect && web_server.arg("forcewifi") != "yes")
    {
      force_wifi_connect = false; 
    }  

    if(virtual_serial_port != web_server.arg("vsport").toInt())
    {
      virtual_serial_port = web_server.arg("vsport").toInt();
      checkPort(virtual_serial_port);
      virtual_serial_changed = true;
    }
    write_eeprom_memory(); 

    if(virtual_serial_changed)
    {
      error_message += F("Device reboot recommended");
      kickVirtualSerial();
    }
  }

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
  page += FPSTR(HTTP_LOGOA);
  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Advanced (Secret) Settings</h2>";
  page += "<h4>" + error_message + "</h4>";

  page += "<form method='post' action='/advanced'>";
  page += "<label for='phoneon'>Upload Data: </label>";
  page += "<input type='checkbox' name='phoneon' value='yes'";
  if(phone_home_on)
  {
    page += " checked";
  }
  page += "><br>";
  page += "<label for='fhserver'>FloatHub Server: </label>";
  page += "<input name='fhserver' value='" + float_hub_server + "'><br>";
  page += "<label for='fhport'>Port: </label>";
  page += "<input type='number' name='fhport' length='5' maxlength='5' value='" + String(float_hub_server_port) + "' ><br>";
  page += "<label for='vserialon'>Virtual Terminal: </label>";
  page += "<input type='checkbox' name='vserialon' value='yes'";
  if(virtual_serial_on)
  {
    page += " checked";
  }
  page += "><br>";
  page += "<label for='vsport'>Port: </label>";
  page += "<input type='number' name='vsport' length='5' maxlength='5' value='" + String(virtual_serial_port) + "' ><br>";
  page += "<label for='forcewifi'>Force Public WiFi: </label>";
  page += "<input type='checkbox' name='forcewifi' value='yes'";
  if(force_wifi_connect)
  {
    page += " checked";
  }
  page += "><br>";
  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br>";

  page += FPSTR(HTTP_HOMEB);
  if(virtual_serial_changed)
  {
    page += FPSTR(HTTP_RBOOT);
  }
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );

}

void handleAccount()
{
  #ifdef HTTP_DEBUG_ON
  debug_info(F("Enter handleAccount()"));
  #endif

  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  //
  // First see if we received any form submission
  // 

  String error_message = "";


  if(web_server.hasArg("deviceid") && web_server.arg("savebutton") == "True")
  {

    if(web_server.arg("deviceid").length() != 8)
    {
      error_message += "Device ID must be exactly 8 characters";
    }
    else
    {
      bool chars_ok = true;
      for(int i = 0; i < 8; i++)
      {
        byte tester = web_server.arg("deviceid").charAt(i);
        if(tester < 48 || (tester > 57 && tester < 65) || (tester > 90 && tester < 97 ) || tester > 122)
	{
	  chars_ok = false;
	}
      }
      if(!chars_ok)
      {
        error_message += "Invalid characters in Device ID";
      }
      else
      {
        if(web_server.arg("aeskey").length() != 32)
        {
          error_message += "Security Key must be exactly 32 characters";
        }
        else
        {
          chars_ok = true;
          for(int i = 0; i < 8; i++)
          {
            byte tester = web_server.arg("aeskey").charAt(i);
            if(tester < 48 || (tester > 57 && tester < 65) || (tester > 70 && tester < 97 ) || tester > 102)
	    {
	      chars_ok = false;
	    }
          }
          if(!chars_ok)
          {
            error_message += "Invalid characters in Security Key";
          }
          else
          {
            // Wow, we survived everything
            float_hub_id = web_server.arg("deviceid");
            
	    String temp_string = web_server.arg("aeskey");
	    temp_string.toLowerCase();

            for(int i = 0; i < 16; i++)
            {
              int new_value = 0;
              char left_byte = temp_string.charAt(i * 2);
              if(left_byte <= '9')
              {
                new_value = (left_byte - '0' ) * 16;
              }
              else
              {   
                new_value = (left_byte - 'a' + 10) * 16;
              }
              char right_byte = temp_string.charAt((i * 2) + 1);
    
              if (right_byte <='9')
              {
                new_value += right_byte - '0';
              } 
              else
              {   
                new_value += right_byte - 'a' + 10;
              }

              float_hub_aes_key[i] = new_value;
            }
	    write_eeprom_memory();
          }
        }
      }
    } 
  }

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
  page += FPSTR(HTTP_LOGOA);
  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Account Settings</h2>";
  page += "<h4>" + error_message + "</h4>";

  page += "<form method='post' action='/account'>";
  page += "<label for='deviceid'>Device ID: </label>";
  page += "<input name='deviceid' value='" + float_hub_id + "' style='width: 250px; font-size: 9pt;' autofocus><br>";
  page += "<label for='aeskey'>Security Key: </label>";
  page += "<input name='aeskey' value='";

  for(int i = 0; i < 16; i++)
  {
    if(float_hub_aes_key[i] < 16)
    {
      page += "0";
    }
    page += String(float_hub_aes_key[i], HEX);
  }

  page += "' style='width: 250px; font-size: 9pt;'><br>";
  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br>";

  page += FPSTR(HTTP_HOMEB);
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );

}


void handleLogo()
{
  web_server.sendHeader(F("Cache-Control"), F("max-age=3600"));
  web_server.send_P(200, "image/png", FLOATHUB_LOGO, sizeof(FLOATHUB_LOGO) );
}


void handleFavicon()
{
  web_server.sendHeader(F("Cache-Control"), F("max-age=3600"));
  web_server.send_P(200, "image/ico", FLOATHUB_FAVICON, sizeof(FLOATHUB_FAVICON) );
}


#ifdef FILE_SERVE_ON
void handleFile()
{
  #ifdef HTTP_DEBUG_ON
  debug_info(F("Enter handleFile()"));
  #endif

  String page = "";
  if(web_server.hasArg("f"))
  {
      File f = SPIFFS.open("/" + web_server.arg("f"), "r");
      if(!f)
      {
        page = F("No such file or dromedary");
      }
      else
      {
        page = "Opened file, contents are \"";
        while(f.available())
        {
          page += (char) f.read();
        }
        f.close();
        page += "\"<br><br>";
        if(web_server.hasArg("d") && web_server.arg("d") == "1")
        {
	  page += "Attempting to delete ...<br>";
          if (SPIFFS.remove("/" + web_server.arg("f")))
          {
            page += "SUCCESS!";
          }
          else
          {
            page += "DRAT!";
          }

        }
      }
      f.close();
    String page = FPSTR(HTTP_INITA);
  }

  web_server.send ( 200, "text/html", page );

}
#endif


void signalBusy()
{
  digitalWrite(serial_ready_pin, LOW);
}


void signalIdle()
{
  digitalWrite(serial_ready_pin, HIGH);
}


void displayUptime()
{
  unsigned long uptime = millis() / 1000.0;
  if (uptime > 86400)
  {
    help_info(String(F(" Uptime: ")) + (uptime / 86400.0) + F(" days"));
  }
  else if (uptime > 3600)
  {
    help_info(String(F(" Uptime: ")) + (uptime / 3600.0) + F(" hours"));
  }
  else if (uptime > 60)
  {
    help_info(String(F(" Uptime: ")) + (uptime / 60.0) + F(" mins"));
  }
  else
  {
    help_info(String(F(" Uptime: ")) + uptime + F(" secs"));
  }
}

void displayCurrentVariables()
{
  String new_message = "v=";
  new_message += FLOATHUB_PROTOCOL_VERSION;
  new_message += ".";
  new_message += FLOATHUB_ENCRYPT_VERSION;
  
  new_message += ",m=";
  new_message += FLOATHUB_MODEL_DESCRIPTION;
  new_message += ",b=";
  new_message += boot_counter;
  help_info(new_message);
  

  
  help_info(String(F("i=")) + float_hub_id); 
  help_info(String(F("s=")) + float_hub_server); 
  help_info(String(F("p=")) + float_hub_server_port); 

  new_message = "k=";
  for(int i = 0; i < 16; i++)
  {
    if(float_hub_aes_key[i] < 16)
    {
      new_message += "0";
    }
    new_message += String(float_hub_aes_key[i], HEX);
  }
  help_info(new_message);


  help_info(String(F("w=")) + public_wifi_ssid); 
  help_info(String(F("W=")) + public_wifi_password); 
  help_info(String(F("x=")) + private_wifi_ssid); 
  help_info(String(F("X=")) + private_wifi_password); 
  help_info(String(F("d=")) + mdns_name); 
  help_info(String(F("F=")) + public_ip_is_static); 
  help_info(String(F("I=")) + public_static_ip.toString()); 
  help_info(String(F("G=")) + public_static_gate.toString()); 
  help_info(String(F("M=")) + public_static_mask.toString()); 
  help_info(String(F("D=")) + public_static_dns.toString()); 
  help_info(String(F("N=")) + nmea_mux_on); 
  help_info(String(F("n=")) + nmea_mux_port); 
  help_info(String(F("e=")) + nmea_mux_private); 
  help_info(String(F("r=")) + relay_ais_data); 
  #ifdef CELLULAR_CODE_ON
  help_info(String(F("R=")) + relay_ais_cellular); 
  #endif
  help_info(String(F("C=")) + force_wifi_connect); 
  help_info(String(F("u=")) + web_interface_username); 
  help_info(String(F("U=")) + web_interface_password); 
  help_info(String(F("P=")) + phone_home_on); 
  help_info(String(F("Z=")) + virtual_serial_on); 
  help_info(String(F("z=")) + virtual_serial_port); 

  help_info(String(F("WiFi-IP: ")) + WiFi.localIP().toString()); 
  help_info(String(F("  AP-IP: ")) + WiFi.softAPIP().toString()); 

  displayUptime();
  
  #ifdef CELLULAR_CODE_ON
  help_info(String(F("   Cell: ")) + cellular_link_up); 
  help_info(String(F("   Name: ")) + cellular_provider); 
  help_info(String(F("    Sim: ")) + sim_card_number); 
  #endif

}


void showFileList(bool actually_show = true)
{
  low_file_pointer = 0; 
  high_file_pointer = 0;
  if(!filesystem_is_working)
  {
    return;
  }
  if(true)
  {
    help_info(F("XXXXX BEG FILE LISTING XXXXX")); 
  }
  String file_name;
  int file_name_number = 0;
  startNoInterrupts();
  Dir dir = SPIFFS.openDir("/");
  while(dir.next())
  {
    file_name = String(dir.fileName()).substring(1);
    //if(actually_show)
    //{
    //  help_info(file_name);
    //}
    if(file_name != "floathub.txt")
    {
      file_name_number = file_name.toInt();
      if(file_name_number > 0)
      {
        if(low_file_pointer  == 0)
        {
          low_file_pointer = file_name_number;
        }
        else if(file_name_number < low_file_pointer)
        {
	  low_file_pointer = file_name_number;
        }
        if(file_name_number > high_file_pointer)
        {
          high_file_pointer = file_name_number; 
        }
      }
    }
  }
  endNoInterrupts();
  if(actually_show)
  {
    help_info(String("LP=") + low_file_pointer + String(", HP=") + high_file_pointer); 
    help_info(F("XXXXX END FILE LISTING XXXXX")); 
  }
}


bool nukeOldestFile()
{
  startNoInterrupts();
  if(!SPIFFS.remove("/" + String(low_file_pointer)))
  {
    #ifdef FILE_DEBUG_ON
    debug_info(F("Can't delete file"), (int) low_file_pointer);
    #endif
    endNoInterrupts();
    return false;
  }
  endNoInterrupts();
  low_file_pointer += 1; 
  return true;
}


bool initFileSystem()
{
  help_info(F("Formatting new filesystem"));
  startNoInterrupts();
  if(!SPIFFS.format())
  {
    help_info(F("BROKEN FILE SYSTEM A"));
    filesystem_is_working = false;
    endNoInterrupts();
    return false;
  }
  File checker = SPIFFS.open("/floathub.txt","w");
  if(!checker)
  {
    help_info(F("BROKEN FILE SYSTEM B"));
    filesystem_is_working = false;
    endNoInterrupts();
    return false;
  }
  checker.print("424242424242");
  checker.close();
  endNoInterrupts();
  filesystem_is_working = true;
  return true;
}


void popMessageQueue()
{

  if(!filesystem_is_working)
  {
    return;
  }

  if(low_file_pointer > 0)
  {
    latest_message_to_send == "";
    while(latest_message_to_send.length() == 0 && low_file_pointer <= high_file_pointer && low_file_pointer > 0)
    {
      startNoInterrupts();
      File f = SPIFFS.open("/" + String(low_file_pointer), "r");
      if(!f)
      {
        endNoInterrupts();
        #ifdef FILE_DEBUG_ON
        debug_info("Bad File/Mess. Rebuild");
        #endif
        if(!nukeOldestFile())
        {
	   #ifdef FILE_DEBUG_ON
           debug_info(F("popMess failed to Nuke, BAD"));
           #endif
           initFileSystem();
           low_file_pointer = 0;
           high_file_pointer = 0;
        }
        if(low_file_pointer > high_file_pointer)
        {
          low_file_pointer = 0;
          high_file_pointer = 0;
	}
      }
      else
      {
        while(f.available())
        {
          latest_message_to_send += (char) f.read();
        }
        f.close();
        endNoInterrupts();
        if(latest_message_to_send.length() < 1)
        {
          #ifdef FILE_DEBUG_ON
          debug_info("Empty Message in Rebuild attempt");
          #endif
          if(!nukeOldestFile())
          {
	    #ifdef FILE_DEBUG_ON
            debug_info(F("popM failed to Nuke, BAD"));
            #endif
            initFileSystem();
            low_file_pointer = 0;
            high_file_pointer = 0;
          }
          if(low_file_pointer > high_file_pointer)
          {
            low_file_pointer = 0;
            high_file_pointer = 0;
	  }
        }
        else
        {
          latest_message_from_file = true;
        }
      }  
    }
  }
}



#ifdef CELLULAR_CODE_ON

void cellular_callback(uint8_t * data, size_t len)
{
  if(busy_doing_spiffs_stuff)
  {
    return;
  }
  //
  // We have received this message, describing the status of the cellular
  // modem
  //	

  String message = String((char *)data);
  #ifdef CELL_DEBUG_ON
  cellular_debug_string = String(F("Cell in: ")) + message;
  #endif


  //
  // The message can be anything like:
  //
  //	"LU 123456 T-Mobile" = Link Up, provider is tmobile
  //	"LR 123456 AT&T"     = Link good, waiting for more to add to current message
  //    "LQ 123456 Orange"   = Link good, but message still in queue
  //    "NL"          = No Link
  //
  // 
  // For user feedback, nice to know state of cellular connection, which is
  // good as long as first character of message is an L
  // 

  if(message.length() > 1 && message.charAt(0) == 'L')
  {
    cellular_link_up = true;
    cellular_link_timestamp = millis();
    if(message.length() > 3)
    {
      sim_card_number = message.substring(3, message.indexOf(' ', 3));
      cellular_provider = message.substring(message.indexOf(' ', 3) + 1);
    }
    else
    {
      cellular_provider = F("Unknown");
      sim_card_number = F("Unknown");
    }
    if(message.length() > 2 && message.charAt(1) == 'U')
    {
      cellular_link_ready = true;
    }
    else
    {
      cellular_link_ready = false;
    }
  }
  else
  {
    cellular_link_up = false;
    cellular_link_ready = false;
    cellular_provider = F("None");
    sim_card_number = F("None");
    SPISlave.setData("");
    return;
  }

  //
  // Still here? Then we want to use cellular data if we can
  //

  if(latest_cellular_message_to_send.length() > 0)
  {
    if(latest_cellular_message_to_send.length() > 32)
    {
      String sub_message = latest_cellular_message_to_send.substring(0,32);
      SPISlave.setData(sub_message.c_str());
      latest_cellular_message_to_send = latest_cellular_message_to_send.substring(32);
    }
    else
    {
      SPISlave.setData(latest_cellular_message_to_send.c_str());
      latest_cellular_message_to_send = "";
    }
  }
  else
  {
    SPISlave.setData("");
  }
}

void setup_spi_slave_to_cellular()
{

  //
  // Every 1/10th of a second, we get a message from the Electron/Cellular
  // modem (running as SPI Master) that informs us of it's status.  We
  // handle that incoming data (called on an interrupt) with this function:
  //

  SPISlave.onData(cellular_callback);

  //
  // Call library setup, set our own flag
  //

  SPISlave.begin();
  cellular_link_up = false;
  cellular_link_timestamp = 0;
  cellular_provider = F("Unknown");
}

#endif

void setupFileSystem()
{
  startNoInterrupts();
  SPIFFS.begin();

  //
  //  See if we can read our floathub.txt file. If we can't, this is a
  // virgin filesystem and needs to be formatted
  //

  File checker = SPIFFS.open("/floathub.txt","r");
  
  if(!checker)
  {
    endNoInterrupts();
    initFileSystem();
    return;
  }

  checker.close();
  endNoInterrupts();
  filesystem_is_working = true;

  showFileList();

}


void setup(void)
{

  //
  //  Turn off watchdog
  //

  ESP.wdtDisable();

  // 
  // Reserve some variable space
  //

  console_read_buffer.reserve(MAX_CONSOLE_BUFFER);
  virtual_serial_read_buffer.reserve(MAX_CONSOLE_BUFFER);
  latest_message_to_send.reserve(MAX_LATEST_MESSAGE_SIZE);
  wifi_read_buffer.reserve(MAX_WIFI_READ_BUFFER_SIZE);
  latest_message_to_send = "";
  wifi_read_buffer = "";
  #ifdef CELL_DEBUG_ON
  cellular_debug_string.reserve(MAX_LATEST_MESSAGE_SIZE);
  cellular_debug_string = "";
  #endif



  //
  //  Set up communicatinons to the Mega
  //

  Serial.begin ( 115200 );
  serial_ready_pin = 5;
  pinMode(serial_ready_pin, OUTPUT);
  signalBusy();

  //
  //  Initial state for SPIFFS
  //

  busy_doing_spiffs_stuff = false;


  //
  // Seed the random number generator so our cookie values are reasonably (pseudo-)random
  //

  randomSeed(analogRead(A0) * ESP.getCycleCount());

  for(int i=0; i < MAX_COOKIES; i++)
  {
    nukeCookie(i);
  }

  //
  //  Handle EEPROM logic for persistant settings (if the first 5 bytes of
  //  EEPROM memory are not all set to 42, then this is a completely
  //  unitialized device)
  //
  
  EEPROM.begin(400);
  int a = 0;
  for(int i = 0; i < 6; i++)
  {
    a = EEPROM.read(i);
    if(a != 42)
    {
      debug_info("Virgin Module");
      init_eeprom_memory();
      break;
    }
  }

  read_eeprom_memory();

  help_info("");
  displayCurrentVariables();

  //
  //  Setup filesystem (do some checks, format if first time run, etc.)
  //

  setupFileSystem();

  //
  // Fire up the WiFi
  //

  kickWiFi(true);

  //
  // Bring up the web server
  //

  web_server.on ( "/", handleRoot );
  web_server.on ( "/logo.png", handleLogo );
  web_server.on ( "/favicon.ico", handleFavicon );
  web_server.on ( "/login", handleLogin );
  web_server.on ( "/public", handlePublicWireless );
  web_server.on ( "/private", handlePrivateWireless );
  web_server.on ( "/other", handleOther );
  web_server.on ( "/advanced", handleAdvanced );
  web_server.on ( "/account", handleAccount );
  web_server.on ( "/reboot", handleReboot );
  web_server.onNotFound(handleLogin);
  #ifdef FILE_SERVE_ON
  web_server.on ( "/file", handleFile ); 
  #endif

  //
  //  Set which headers to keep track of
  //

  const char * headerkeys[] = {"User-Agent","Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys)/sizeof(char*);
  web_server.collectHeaders(headerkeys, headerkeyssize );

  web_server.begin();

  delay(1000);


  //
  // Bring up nmea_server if need be
  //

  kickNMEA();

  //
  // Bring up Virtual Serial Server
  //

  kickVirtualSerial();

  //
  // If we have some, pop off an old message
  //

  if(low_file_pointer > 0)
  {
    popMessageQueue();
    latest_message_from_file = true;
  }

  //
  // Set watchdog timestamp
  //

  watchdog_timestamp = millis();


  //
  //  If we are celullar, need to set up some SPI Slave methods
  //
  #ifdef CELLULAR_CODE_ON
  setup_spi_slave_to_cellular();
  latest_cellular_message_to_send.reserve(MAX_LATEST_MESSAGE_SIZE);
  latest_cellular_message_to_send = "";
  #endif
}


bool openFdr()
{


  // Open a connection to the FloatHub Data Receiver server (usually fdr.floathub.net)

  #ifdef WIFI_DEBUG_ON
  debug_info(F("Opening fdr socket ..."));
  #endif
  if(fdr_client.connect(float_hub_server.c_str(), float_hub_server_port))
  {
    
    fdr_client.setNoDelay(true);
    #ifdef WIFI_DEBUG_ON
    debug_info(F("Success "));
    #endif
    return true;
  }
  else
  {
    #ifdef WIFI_DEBUG_ON
    debug_info(F("Failure "));
    #endif
  }

  return false;
}



void echoNMEA(String a_message)
{
  int how_many; 

  if(nmea_mux_on)
  {
    for(int i = 0; i < NUMB_NMEA_CLIENTS; i++)
    {
      if(nmea_client[i])
      {
        nmea_client[i].println(a_message);
      }
    }
  }

  //
  // If we're phoning home AND relay AIS is set on AND this looks like AIS,
  // and we're not backed up with regular messages, send pretty much a raw
  // AIS string up to the fdr server
  //

  #ifdef CELLULAR_CODE_ON
  if(
      (
        WiFi.status() == WL_CONNECTED ||
        ( cellular_link_up && cellular_link_ready && relay_ais_cellular && latest_cellular_message_to_send.length() == 0 )
      )                              &&
      phone_home_on == true          &&
      relay_ais_data == true         &&
      low_file_pointer < 10	     &&
      high_file_pointer < 10         &&
      a_message.indexOf('!') == 0    &&
      (
        a_message.indexOf('A') == 1  ||
        a_message.indexOf('B') == 1  ||
        a_message.indexOf('S') == 1  
      )
     )
  #else
  if(
      WiFi.status() == WL_CONNECTED  &&
      phone_home_on == true          &&
      relay_ais_data == true         &&
      low_file_pointer < 10	     &&
      high_file_pointer < 10         &&
      a_message.indexOf('!') == 0    &&
      (
        a_message.indexOf('A') == 1  ||
        a_message.indexOf('B') == 1  ||
        a_message.indexOf('S') == 1  
      )
     )
  #endif
  {
    String string_to_send = F("$FHO:");
    string_to_send += float_hub_id + ":" ;
    string_to_send += String(FLOATHUB_PROTOCOL_VERSION) + "$,";
    string_to_send += a_message;

    #ifdef AISR_DEBUG_ON
    debug_info(String("AIS relay: ") + string_to_send);
    #endif

    //
    // We have to encode and markup as FHS 
    //

    int i = 0;
    unsigned int cipher_length, base64_length;
    aes.set_key (float_hub_aes_key, 128) ;
    for(i = 0; i < 16; i++)
    {
      iv[i] = random(0, 256);
      volatile_iv[i] = iv[i];
    }

    for(i = 0; i < string_to_send.length(); i++)
    {
      plain_text[i] = string_to_send.charAt(i);
    }
    cipher_length = i;

    //
    //  AES encrypted messages should end on a 16 byte (128 bit) boundary
    //

    if(cipher_length % 16 == 0)
    {
      for(i = 0; i < 16; i++)
      {
        plain_text[cipher_length + i] = 16;
      }
      cipher_length += 16;
    }
    else
    {
      for(i = 0; i < 16 - (cipher_length % 16); i++)
      {
        plain_text[cipher_length + i] = 16 - (cipher_length % 16);
      }
      cipher_length += i;
    }

    //
    //  Now right length, we can encrypt
    //

    aes.cbc_encrypt (plain_text, cipher_text,  cipher_length / 4, volatile_iv) ;

    //
    //  Now we reuse the plain text array to store cipher with the
    //  initialization vector at the beginning
    //

  
    for(i = 0; i < 16; i++)
    {
      plain_text[i] = iv[i];
    }
    for(i = 0; i < cipher_length; i++)
    {
      plain_text[16 + i] = cipher_text[i];
    }
  

    //
    //  Now convert that long line of bytes in plain to base 64, recycling the cipher array to hold it
    //
  
    base64_length = base64_encode( (char *) cipher_text, (char *) plain_text, cipher_length + 16);
    cipher_text[base64_length] = '\0';

    //
    // Assemble into FHS
    //

    String fhs_message_to_send = F("$FHS:");
    fhs_message_to_send += float_hub_id;
    fhs_message_to_send += F(":");
    fhs_message_to_send += FLOATHUB_ENCRYPT_VERSION;
    fhs_message_to_send += F("$,");
    for(i = 0; i < base64_length; i++)
    {
      fhs_message_to_send += (char) cipher_text[i];
    }
    fhs_message_to_send += "\r\n";

    #ifdef AISR_DEBUG_ON
    debug_info(String("AIS FHS: ") + fhs_message_to_send);
    #endif

    if( openFdr() && fdr_client.connected())
    {
      how_many = fdr_client.write(fhs_message_to_send.c_str(), fhs_message_to_send.length());
      #ifdef AISR_DEBUG_ON
      if(how_many == fhs_message_to_send.length())
      {
        debug_info("AIS relay seemed to all go");
      }
      else
      {
        debug_info("AIS relay only sent ", how_many);
      }
      #endif
      fdr_client.stop();
    }
    #ifdef CELLULAR_CODE_ON
    else if ( cellular_link_up && cellular_link_ready && relay_ais_cellular && latest_cellular_message_to_send.length() == 0 )
    {
      latest_cellular_message_to_send = fhs_message_to_send;
    }
    #endif

    //
    //  Clean up?
    // 

    aes.clean();  //  Not sure if that does anything useful or not.   

  }
}


void pushMessageQueue(String a_message)
{
  if(filesystem_is_working)
  {
    //
    // First check how much space we have left
    //

    #ifdef FILE_DEBUG_ON
    debug_info(F("Free file space ..."));
    #endif
    FSInfo fs_info;
    startNoInterrupts();
    SPIFFS.info(fs_info);
    endNoInterrupts();

    long int space_left = fs_info.totalBytes - fs_info.usedBytes;

    if(space_left > MIN_SPIFF_SPACE)
    {
      #ifdef FILE_DEBUG_ON
      debug_info(F("Enough free: "), (int) space_left);
      debug_info(F("      Total: "), (int) fs_info.totalBytes);
      debug_info(F("       Used: "), (int) fs_info.usedBytes);
      debug_info(F("        LFP: "), (int) low_file_pointer);
      debug_info(F("        HFP: "), (int) high_file_pointer);
      #endif
      
    }
    else
    {
      #ifdef FILE_DEBUG_ON
      debug_info(F("Not enough free: "), (int) space_left);
      debug_info(F("            LFP: "), (int) low_file_pointer);
      debug_info(F("            HFP: "), (int) high_file_pointer);
      #endif
      while(space_left <= MIN_SPIFF_SPACE)
      {
        popMessageQueue();
        if(!nukeOldestFile())
        {
	   #ifdef FILE_DEBUG_ON
           debug_info(F("pushM failed to Nuke, BAD"));
           #endif
           initFileSystem();
           low_file_pointer = 0;
           high_file_pointer = 0;
        }
        startNoInterrupts();
        SPIFFS.info(fs_info);
        endNoInterrupts();
        space_left = fs_info.totalBytes - fs_info.usedBytes;
      } 
    }

    //
    // Create a file called high_file_pointer and write the message inside it
    //

    startNoInterrupts();
    File f = SPIFFS.open("/" + String(high_file_pointer + 1), "w");
    if(!f)
    {
      endNoInterrupts();
      bool keep_going = true;
      while(keep_going)
      {
        popMessageQueue();
        if(!nukeOldestFile())
        {
	  #ifdef FILE_DEBUG_ON
          debug_info(F("pushMess failed to Nuke, BAD"));
          #endif
          initFileSystem();
          low_file_pointer = 0;
          high_file_pointer = 0;
        }
        startNoInterrupts();
        SPIFFS.info(fs_info);
        space_left = fs_info.totalBytes - fs_info.usedBytes;
        f= SPIFFS.open("/" + String(high_file_pointer + 1), "w");
        endNoInterrupts();
        if(f)
        {
          f.print(a_message);
          high_file_pointer += 1;
          keep_going = false;
        }
        if(low_file_pointer > high_file_pointer)
        {
          low_file_pointer = 0;
          high_file_pointer = 0;
          keep_going = false;
          latest_message_to_send = a_message;
          #ifdef FILE_DEBUG_ON
          debug_info(F("Really bad, through out entire file history"));
          #endif
        }
      }
    } 
    else
    {
      f.print(a_message);
      //f.close();
      high_file_pointer += 1;
      if(low_file_pointer == 0)
      {
        low_file_pointer += 1;
      }
    }
    f.close();
    endNoInterrupts();
  }
  else
  {
    //
    //  This is bad, we have nowhere to store previous message that we still
    // have not sent. Best we can is to throw it away and replace with previous messaahe,
    //

    latest_message_to_send = a_message; 
  }
}


void queueMessage(String a_message)
{

  //
  //   Check a few things
  //

  int checksum_start = a_message.lastIndexOf('@');
  if(checksum_start != a_message.length() - 3)
  {
    #ifdef INPT_DEBUG_ON
    debug_info(F("Killed FHS message, no checksum"));
    #endif
    return;
  }

  String checksum_a = a_message.substring(checksum_start + 1);

  a_message = a_message.substring(0, a_message.length() - 3);

  byte a_byte = 0;
  for (int i = 0; i < a_message.length(); i++)
  {
    a_byte = a_byte ^ a_message.charAt(i); 
  }

  String checksum_b = String(a_byte, HEX);
  checksum_b.toUpperCase();
  if(checksum_b.length() < 2)
  {
    checksum_b = String(F("0")) + checksum_b;
  } 

  if(checksum_a != checksum_b)
  {
    #ifdef INPT_DEBUG_ON
    debug_info(String(F("WOW WOW Killed bad FHS: ")) + checksum_a + String(F(" != ")) + checksum_b);
    #endif
    return;
  }

  if(latest_message_to_send.length() == 0)
  {
    #ifdef INPT_DEBUG_ON
    debug_info(F("Queued Message, No File"));
    #endif
    latest_message_to_send = a_message;
    latest_message_from_file = false;
  }
  else 
  {
    #ifdef INPT_DEBUG_ON
    debug_info(F("Pushing Message to File"));
    #endif
    pushMessageQueue(a_message);
  }
}


void nmeaHouseKeeping()
{
  if(!nmea_server || !nmea_mux_on )
  {
    return;
  }

  int i;

  //
  //  Kill any dead nmea clients
  //

  for(i = 0; i < 4; i++)
  {
    if(nmea_client[i] && !nmea_client[i].connected())
    {
      nmea_client[i].stop(); 
    }
  }
  //
  //	Add in any new clients
  //

  while(nmea_server->hasClient())
  { 
    for(i=0; i < 4; i++)
    {
      if(!nmea_client[i])
      {
        break;
      } 
    }
    if(i > 3)
    {
      i = 0;
    }
    nmea_client[i].stop();
    nmea_client[i] = nmea_server->available();
  }
}


void processNewStringValue(String preamble, String &the_string, String new_value)
{
  if(new_value.length() > 32)
  {
    new_value = new_value.substring(0,32);
  }
  the_string = new_value;
  write_eeprom_memory();
  help_info(preamble + the_string);
}


void processNewPortValue(String preamble, unsigned int &the_port, int new_value)
{
  the_port = new_value;
  write_eeprom_memory();
  help_info(preamble + the_port);
}


void processNewFlagValue(String preamble, bool &the_flag, int new_value)
{
  the_flag = new_value;
  write_eeprom_memory();
  help_info(preamble + the_flag);
}


void processNewIPAddress(String preamble, IPAddress &the_address, IPAddress new_address)
{
  the_address = new_address;
  write_eeprom_memory();
  help_info(preamble + the_address.toString());
}


void parseInput(String &the_input)
{
  int i;

  if(the_input.startsWith("E=") && the_input.length() >= 5) // Minimum NMEA sentence length (?)
  {
    echoNMEA(the_input.substring(2));
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("E="))
  {
    debug_info(F("Bad NMEA"));
  }
  #endif
  
  else if(the_input.startsWith("S=") && the_input.length() >= 5) // Minimum FHUB sentence length (?)
  {
    queueMessage(the_input.substring(2));
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("S="))
  {
    debug_info(F("Bad FHx"));
  }
  #endif

  else if(the_input.startsWith("B=")) // Just a debugging message, push it out virtual terminal if there is one 
  {
    if(virtual_serial_on && virtual_serial_client && virtual_serial_client.connected())
    {
      virtual_serial_client.println(the_input.substring(2));
    }
  }

  else if(the_input.startsWith("FactoryResetNow"))
  {
    help_info("Doing factory reset (now) ...");
    init_eeprom_memory();
    read_eeprom_memory();
    initFileSystem();
    delay(500);
    ESP.eraseConfig();
    delay(500);
    ESP.reset();
    ESP.wdtDisable();
  }

  else if(the_input.charAt(0) == 'v')
  {
    displayCurrentVariables();
  }
  else if(the_input.charAt(0) == 'f')
  {
    showFileList();  
  } 

  else if(the_input.startsWith("i=") && the_input.length() >= 10)
  {        
    bool bad_chars = false;
        
    for(i=0; i < 8; i++)
    {
      byte tester = the_input.charAt(2+i);
      if(tester < 48 || (tester > 57 && tester < 65) || (tester > 90 && tester < 97 ) || tester > 122)
      {
        bad_chars = true;
        #ifdef INPT_DEBUG_ON
        debug_info(F("Bad input"));
        #endif
        break;
      }
    }
    if(!bad_chars)
    {
      processNewStringValue("i=", float_hub_id, the_input.substring(2,10));
    }
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("i="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("s=") && the_input.length() >= 3)
  {        
    processNewStringValue("s=", float_hub_server, the_input.substring(2));
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("s="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("p=") && the_input.length() >= 3)
  {
    int new_value = the_input.substring(2).toInt();
    if(new_value > 0 && new_value < 65535)
    { 	
      processNewPortValue("p=", float_hub_server_port, new_value);
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad number"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("p="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("w=") && the_input.length() >= 3)
  {        
    processNewStringValue("w=", public_wifi_ssid, the_input.substring(2));
    kickWiFi();
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("w="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("W=") && the_input.length() >= 10)
  {        
    processNewStringValue("W=", public_wifi_password, the_input.substring(2));
    kickWiFi();
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("W="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("x=") && the_input.length() >= 3)
  {        
    processNewStringValue("x=", private_wifi_ssid, the_input.substring(2));
    kickWiFi();
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("x="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("X=") && the_input.length() >= 10)
  {        
    processNewStringValue("X=", private_wifi_password, the_input.substring(2));
    kickWiFi();
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("X="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("d=") && the_input.length() >= 3)
  {        
    processNewStringValue("d=", mdns_name, the_input.substring(2));
    kickMDNS();
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("d="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("F=") && the_input.length() >= 3)
  {        
    int new_value = the_input.substring(2).toInt();
    if(the_input[2] == '0' || the_input[2] == '1')
    { 	
      processNewFlagValue("F=", public_ip_is_static, new_value);
      kickWiFi();
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad boolean"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("F="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("I=") && the_input.length() >= 9)
  {        
    IPAddress new_address;
    if(new_address.fromString(the_input.substring(2)))
    { 	
      processNewIPAddress("I=", public_static_ip, new_address);
      if(public_ip_is_static)
      {
        kickWiFi();
      }
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad IP"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("I="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("G=") && the_input.length() >= 9)
  {        
    IPAddress new_address;
    if(new_address.fromString(the_input.substring(2)))
    { 	
      processNewIPAddress("G=", public_static_gate, new_address);
      if(public_ip_is_static)
      {
        kickWiFi();
      }
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad IP"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("G="))
  {
    debug_info(F("Short input"));
  }
  #endif
  else if(the_input.startsWith("M=") && the_input.length() >= 9)
  {        
    IPAddress new_address;
    if(new_address.fromString(the_input.substring(2)))
    { 	
      processNewIPAddress("M=", public_static_mask, new_address);
      if(public_ip_is_static)
      {
        kickWiFi();
      }
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad IP"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("M="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("D=") && the_input.length() >= 9)
  {        
    IPAddress new_address;
    if(new_address.fromString(the_input.substring(2)))
    { 	
      processNewIPAddress("D=", public_static_dns, new_address);
      if(public_ip_is_static)
      {
        kickWiFi();
      }
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad IP"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("D="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("k=") && the_input.length() == 34)
  {        
    bool bad_chars = false;
    the_input.toLowerCase();

    for(i=0; i < 32; i++)
    {
      if(!((the_input[i+2] >= '0' && the_input[i+2] <= '9') || (the_input[i+2] >= 'a' && the_input[i+2] <= 'f' )))          
      {
        help_info("Bad input");
        bad_chars = true;
        break;
      }
    }        
    if(!bad_chars)
    {
      String display_string = "k=";
      for(i = 0; i < 16; i++)
      {
        int new_value = 0;
        if (the_input[2 + (i * 2)] <='9')
        {
          new_value = (the_input[2 + (i * 2)] - '0' ) * 16;
        }
        else
        {
          new_value = (the_input[2 + (i * 2)] - 'a' + 10) * 16;
        }
    
        if (the_input[3 + (i * 2)] <='9')
        {
          new_value += the_input[3 + (i * 2)] - '0';
        }
        else
        {
          new_value += the_input[3 + (i * 2)] - 'a' + 10;
        }

        float_hub_aes_key[i] = new_value;
        if(float_hub_aes_key[i] < 16)
        {
          display_string += "0";
        }
        display_string += String(float_hub_aes_key[i], HEX);
      }
      write_eeprom_memory();
      help_info(display_string);
    }    
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("k="))
  {
    debug_info(F("Bad length"));
  }
  #endif


  else if(the_input.startsWith("N=") && the_input.length() >= 3)
  {        
    int new_value = the_input.substring(2).toInt();
    if(the_input[2] == '0' || the_input[2] == '1')
    { 	
      processNewFlagValue("N=", nmea_mux_on, new_value);
      kickNMEA();      
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad boolean"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("N="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("e=") && the_input.length() >= 3)
  {        
    int new_value = the_input.substring(2).toInt();
    if(the_input[2] == '0' || the_input[2] == '1')
    { 	
      processNewFlagValue("e=", nmea_mux_private, new_value);
      kickNMEA();      
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad boolean"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("e="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("C=") && the_input.length() >= 3)
  {        
    int new_value = the_input.substring(2).toInt();
    if(the_input[2] == '0' || the_input[2] == '1')
    { 	
      processNewFlagValue("C=", force_wifi_connect, new_value);
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad boolean"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("C="))
  {
    debug_info(F("Short input"));
  }
  #endif


  else if(the_input.startsWith("r=") && the_input.length() >= 3)
  {        
    int new_value = the_input.substring(2).toInt();
    if(the_input[2] == '0' || the_input[2] == '1')
    { 	
      processNewFlagValue("r=", relay_ais_data, new_value);
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad boolean"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("r="))
  {
    debug_info(F("Short input"));
  }
  #endif

  #ifdef CELLULAR_CODE_ON
  else if(the_input.startsWith("R=") && the_input.length() >= 3)
  {        
    int new_value = the_input.substring(2).toInt();
    if(the_input[2] == '0' || the_input[2] == '1')
    { 	
      processNewFlagValue("r=", relay_ais_cellular, new_value);
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad boolean"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("R="))
  {
    debug_info(F("Short input"));
  }
  #endif
  #endif

  else if(the_input.startsWith("n=") && the_input.length() >= 3)
  {
    int new_value = the_input.substring(2).toInt();
    if(new_value > 0 && new_value < 65535)
    { 	
      processNewPortValue("n=", nmea_mux_port, new_value);
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad number"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("n="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("u=") && the_input.length() >= 6)
  {        
    processNewStringValue("u=", web_interface_username, the_input.substring(2));
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("u="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("U=") && the_input.length() >= 6)
  {        
    processNewStringValue("U=", web_interface_password, the_input.substring(2));
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("U="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("P=") && the_input.length() >= 3)
  {        
    int new_value = the_input.substring(2).toInt();
    if(the_input[2] == '0' || the_input[2] == '1')
    { 	
      processNewFlagValue("P=", phone_home_on, new_value);
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad boolean"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("P="))
  {
    debug_info(F("Short input"));
  }
  #endif
  
  else if(the_input.startsWith("Z=") && the_input.length() >= 3)
  {        
    int new_value = the_input.substring(2).toInt();
    if(the_input[2] == '0' || the_input[2] == '1')
    { 	
      processNewFlagValue("Z=", virtual_serial_on, new_value);
      kickVirtualSerial();
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad boolean"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("Z="))
  {
    debug_info(F("Short input"));
  }
  #endif

  else if(the_input.startsWith("z=") && the_input.length() >= 3)
  {
    int new_value = the_input.substring(2).toInt();
    if(new_value > 0 && new_value < 65535)
    { 	
      processNewPortValue("z=", virtual_serial_port, new_value);
      kickVirtualSerial();
    }
    #ifdef INPT_DEBUG_ON
    else
    {
      debug_info(F("Bad number"));
    }
    #endif
  }
  #ifdef INPT_DEBUG_ON
  else if(the_input.startsWith("z="))
  {
    debug_info(F("Short input"));
  }
  #endif

  #ifdef INPT_DEBUG_ON
  else
  {
    debug_info(String(F("Bad input: ")) + the_input);
  }
  #endif
}


void virtualSerialHouseKeeping()
{
  if(!virtual_serial_server || !virtual_serial_on )
  {
    return;
  }

  if(virtual_serial_client && !virtual_serial_client.connected())
  {
    virtual_serial_client.stop(); 
  }

  //
  //	Add in any new clients
  //

  if(virtual_serial_server->hasClient())
  { 
    if(virtual_serial_client)
    {
      virtual_serial_client.stop(); 
    }
    virtual_serial_client = virtual_serial_server->available();
  }

  //
  //  read anything on it's way in
  //

  char incoming_byte;
  if(virtual_serial_client)
  {
    while(virtual_serial_client.available() && (int) virtual_serial_read_buffer.length() < MAX_CONSOLE_BUFFER)
    {
      incoming_byte = virtual_serial_client.read();
      if(incoming_byte == '\r')
      {
        parseInput(virtual_serial_read_buffer);
        virtual_serial_read_buffer = "";
      }
      else if (incoming_byte == '\n')
      {
        // don't do anything
      }
      else
      {
        virtual_serial_read_buffer += incoming_byte;
      }
    }
    if((int) virtual_serial_read_buffer.length() >= MAX_CONSOLE_BUFFER - 1 )
    {
      virtual_serial_read_buffer = "";
    }
  }
}


bool push_latest_message_out_socket()
{
  int how_many;
  if(fdr_client.connected())
  {
    String string_to_send = latest_message_to_send + "\r\n";
    how_many = fdr_client.write(string_to_send.c_str(), string_to_send.length());
    if(how_many == string_to_send.length())
    {
      return true;
    }
    #ifdef WIFI_DEBUG_ON
    debug_info(String(F("Socket write failed, tried ")) + String(string_to_send.length()) + String(F(", got ")) + String(how_many));
    #endif
    return false;
  }
  #ifdef WIFI_DEBUG_ON
  else
  {
    debug_info(F("socket write called, but not connected"));
  }
  #endif

  return false;
}


void houseKeeping()
{

  #ifdef STAT_DEBUG_ON
  IPAddress local = WiFi.localIP();
  debug_info(String(F("WiFi.status()=")) + WiFi.status() + String(F(", IP address: ")) + local.toString() + String(F(", latest_message=")) + latest_message_to_send) ; 
  #endif

  long now = millis();
  for(int i; i < MAX_COOKIES; i++)
  {
    if(now - cookies[i].time > 60 * 1000 * 8 && cookies[i].valid == true)	// 8 minute cookie timeout
    {
       nukeCookie(i);
    }
  }

  //
  // Check that we have managed to send something at some point in rcent
  // past, otherwise reboot the whole canoodle
  //

  if(millis() - watchdog_timestamp > WATCHDOG_TIME_LIMIT )
  {
    #ifdef WIFI_DEBUG_ON
    debug_info(F("Watchdog hit, rebooting all ..."));
    #endif
    ESP.restart();
  }

  //
  // Check that we have _positively_ been informed that the cellular link is
  // up, otherwise assume it is down
  //
  #ifdef CELLULAR_CODE_ON
  if(millis() - cellular_link_timestamp > 5 * 1000 && cellular_link_up)
  {
    cellular_link_up = false;
    #ifdef CELL_DEBUG_ON
    debug_info("Forced cell link flag to false due to inactivity");
    #endif
  }
  #endif
}


void WiFiHouseKeeping()
{

  wifi_housekeeping_cycle_counter++;
  if(wifi_housekeeping_cycle_counter > 10)
  {
    wifi_housekeeping_cycle_counter = 1;
  }

  #ifdef WIFI_DEBUG_ON
  debug_info(F("WiFi HK Cycle: "), wifi_housekeeping_cycle_counter);
  #endif

  //
  //  If we have public WiFi link, then we don't need to do anything
  //
  if(WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  //
  // OK, we are _not_ connected. Once every ten cycles through here, try to
  // connect if the public wifi is set to anything other than factory
  // default _and_ we can see the public wifi station id in a scan
  //

  if(public_wifi_ssid == "factoryX")
  {
    return;
  }

  //
  //  If we were connected, then we need to go to AP only mode as it looks
  // like whatever we were connected to has gone away
  //

  if (tried_public_wifi_recently == true)
  {
    #ifdef WIFI_DEBUG_ON
    debug_info("WiFi HK: Public Went Away");
    #endif
    kickWiFi(true);
  }

  //
  //	Otherwise, we look for out public station id once every 10 cycles
  // through here and try to connect only if we see it
  //

  if(wifi_housekeeping_cycle_counter == 10 )
  {
    bool saw_public = false;    
    byte numSsid = WiFi.scanNetworks();

    for (int i = 0; i < numSsid; i++)
    {
      if (public_wifi_ssid == String(WiFi.SSID(i)))
      {
        saw_public = true;
        break;
      }
    }

    if(force_wifi_connect)
    {
      #ifdef WIFI_DEBUG_ON
      debug_info(F("WiFi HK: Forced Public"));
      #endif
      kickWiFi();
    }
    else if(saw_public)
    {
      #ifdef WIFI_DEBUG_ON
      debug_info(F("WiFi HK: Trying Public"));
      #endif
      kickWiFi();
    }
    else
    {
      #ifdef WIFI_DEBUG_ON
      debug_info(F("Wifi HK: Skip Public, No Scan"));
      #endif
    }
  }
}


void heartbeatHouseKeeping()
{
  if(heartbeat_cycle == 0)
  {
    internal_info(String(F("i=")) + float_hub_id); 
  }
  else if(heartbeat_cycle == 1)
  {
    String new_message = "k=";
    for(int i = 0; i < 16; i++)
    {
      if(float_hub_aes_key[i] < 16)
      {
        new_message += "0";
      }
      new_message += String(float_hub_aes_key[i], HEX);
    }
    internal_info(new_message);
  }
  else if(heartbeat_cycle == 2)
  {
    int public_wifi_status = 0;
    if(WiFi.status() == WL_CONNECTED)
    {
      public_wifi_status = 1;
    }
    internal_info(String(F("c=")) + public_wifi_status);
  }

  #ifdef CELLULAR_CODE_ON
  else if(heartbeat_cycle == 3)
  {
    internal_info(String(F("d=")) + cellular_link_up); 
  }
  heartbeat_cycle++;
  if(heartbeat_cycle >=4)
  {
    heartbeat_cycle = 0;
  }
  #else
  heartbeat_cycle++;
  if(heartbeat_cycle >=3)
  {
    heartbeat_cycle = 0;
  }
  #endif
}


void fdrHouseKeeping()
{
  if(WiFi.status() == WL_CONNECTED && WiFi.localIP() > 0)
  {
    if(!called_mdns_after_connection)
    {
      called_mdns_after_connection = true;
      kickMDNS();
    }

    //
    //	We are appear to be connected to something. If there's a message to
    // send AND we are not in silent mode (i.e.  if phone_home_on false?),
    // we should send message
    //	

    unsigned long current_timestamp = millis();
    if(current_communication_state == idle && phone_home_on == true)	
    {
      if(latest_message_to_send.length() > 0)
      {
        if(openFdr())
        {
          if(push_latest_message_out_socket())
          {
            #ifdef WIFI_DEBUG_ON
            debug_info(F("wifi Sent FHx"));
            #endif
            current_communication_state = waiting_for_response;
            wifi_read_buffer = "";
            expected_fdr_response_time = millis() + FDR_RESPONSE_MAX ;
          }
          else
          {
            #ifdef WIFI_DEBUG_ON
            debug_info(F("wifi sock prob"));
            #endif
            fdr_client.stop();
          }
        }
        else
        {
          fdr_client.stop();
        }
      }
    }
    else if(current_communication_state == waiting_for_response)
    {
      unsigned long last_read = millis();
      while (fdr_client.connected() && (millis() - last_read < 50))
      {
        while (fdr_client.available() && wifi_read_buffer.length() < MAX_WIFI_READ_BUFFER_SIZE)
        {
          wifi_read_buffer += (char) fdr_client.read();
          last_read = millis();
        }
      }
      if(wifi_read_buffer.indexOf("$FHR$ OK") > -1)
      {
        //
        // Yay! We have succesfully sent something
        //
   
        watchdog_timestamp = millis();

        //
	// Adjust message pointers if need be
        //
        
        if(latest_message_from_file)
        {
          if(low_file_pointer > 0)
          {
            if(!nukeOldestFile())
            {
	      #ifdef FILE_DEBUG_ON
              debug_info(F("HouseK failed to Nuke, BAD"));
              #endif
              initFileSystem();
              low_file_pointer = 0;
              high_file_pointer = 0;
            }
            else if(low_file_pointer > high_file_pointer)
            {
	      // We have emptied the queue ..
              low_file_pointer = 0;
              high_file_pointer = 0;
	    }
          }
	}

        latest_message_to_send = "";
        popMessageQueue();

        if(latest_message_to_send.length() > 0)
        {
          if(push_latest_message_out_socket())
          {
            #ifdef WIFI_DEBUG_ON
            debug_info(F("wifi more FHx"));
            #endif
            current_communication_state = waiting_for_response;
            wifi_read_buffer = "";
          }
          else
          {
            #ifdef WIFI_DEBUG_ON
            debug_info(F("wifi Hangup"));
            #endif
            fdr_client.stop();
            current_communication_state = idle;
          }

        }
        else
        {
          #ifdef WIFI_DEBUG_ON
          debug_info(F("WiFi queue done"));
          #endif
          wifi_read_buffer = "";
          fdr_client.stop();
          current_communication_state = idle;
	}
      }
      else if (millis() > expected_fdr_response_time)
      {
        #ifdef WIFI_DEBUG_ON
        debug_info(F("Died waiting FHR"));
        #endif
        current_communication_state = idle;
        fdr_client.stop();
      }
    }
  }
  else
  {
    called_mdns_after_connection = false;

    //
    //  If we have cellular, and it has been a while 
    //

    #ifdef CELLULAR_CODE_ON
    if ( 
        millis() - watchdog_timestamp > WATCHDOG_USE_CELLULAR_TIME_LIMIT &&
	cellular_link_up == true &&
	cellular_link_ready == true &&
        phone_home_on == true &&
        latest_message_to_send.length() > 0 &&
        latest_cellular_message_to_send.length() == 0
       )
    {
      #ifdef CELL_DEBUG_ON
      debug_info(F("Using cellular for message"));
      #endif
      latest_cellular_message_to_send = latest_message_to_send + "\r\n";
      latest_message_to_send = "";

      if(latest_message_from_file)
      {
        if(low_file_pointer > 0)
        {
          if(!nukeOldestFile())
          {
	    #ifdef FILE_DEBUG_ON
            debug_info(F("HouseK failed to Nuke, BAD"));
            #endif
            initFileSystem();
            low_file_pointer = 0;
            high_file_pointer = 0;
          }
          else if(low_file_pointer > high_file_pointer)
          {
	    // We have emptied the queue ..
            low_file_pointer = 0;
            high_file_pointer = 0;
	  }
        }
      }
      popMessageQueue();
    }
    #endif
  }
}


void readConsole()
{

  signalIdle();
  unsigned long right_right_now = millis();

  //
  //  Signal we are available, then wait up to 2 full seconds for data to come
  //

  while((Serial.available() && (int) console_read_buffer.length() < MAX_CONSOLE_BUFFER) || (millis() - right_right_now) < 2000UL)
  //while((Serial.available() && (int) console_read_buffer.length() < MAX_CONSOLE_BUFFER))
  {
    if(Serial.available() && (int) console_read_buffer.length() < MAX_CONSOLE_BUFFER)
    {
      int incoming_byte = Serial.read();
      if(incoming_byte == '\r')
      { 
        signalBusy();
        right_right_now -= 2000UL;
        parseInput(console_read_buffer);
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
  }

  if((int) console_read_buffer.length() >= MAX_CONSOLE_BUFFER - 1 )
  {
    #ifdef INPT_DEBUG_ON
    debug_info("==== beg console nuke ====");
    debug_info(console_read_buffer);
    debug_info("==== end console nuke ====");
    #endif
    console_read_buffer = "";
  }
  signalBusy();

  //
  // We are here frequently, but outside of interrupts, so if there happens
  // to be some cellular debugging output, show it
  //

  #ifdef CELL_DEBUG_ON
  if(cellular_debug_string.length() > 0)
  {
    debug_info(cellular_debug_string);
    cellular_debug_string = "";
  }
  #endif

  
}


void loop(void)
{
  //
  //  Obviously this is main execution loop. There are a number of values we read and actions we take base on timing
  //
   
  unsigned long current_timestamp = millis();
 
  if(current_timestamp - heartbeat_previous_timestamp >  heartbeat_interval)
  {
    heartbeat_previous_timestamp = current_timestamp;
    heartbeatHouseKeeping();
  }
  if(current_timestamp - fdr_communications_previous_timestamp >  fdr_communications_interval)
  {
    fdr_communications_previous_timestamp = current_timestamp;
    fdrHouseKeeping();
  }
  if(current_timestamp - console_previous_timestamp > console_read_interval)
  {
    console_previous_timestamp = current_timestamp;
    //unsigned long duration = millis();
    readConsole();
    //help_info(String(F("readConsole took ")) +  ((millis() - duration )/ 1000.00));
  }
  if(current_timestamp - house_keeping_previous_timestamp >  house_keeping_interval)
  {
    house_keeping_previous_timestamp = current_timestamp;
    houseKeeping();
    //displayUptime();
  }
  if(current_timestamp - wifi_housekeeping_previous_timestamp >  wifi_housekeeping_interval)
  {
    wifi_housekeeping_previous_timestamp = current_timestamp;
    WiFiHouseKeeping();
  }
  if(current_timestamp - nmea_housekeeping_previous_timestamp > nmea_housekeeping_interval)
  {
    nmea_housekeeping_previous_timestamp = current_timestamp;
    nmeaHouseKeeping();
  } 
  if(current_timestamp - virtual_serial_housekeeping_previous_timestamp > virtual_serial_housekeeping_interval)
  {
    virtual_serial_housekeeping_previous_timestamp = current_timestamp;
    virtualSerialHouseKeeping();
  } 
  if(current_timestamp - network_previous_timestamp > network_interval)
  {
    network_previous_timestamp = current_timestamp;
    web_server.handleClient();
  }
}



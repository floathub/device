/*

 FloatHub ESP8266 Code
 (c) 2015-2016 Modiot Labs
 
*/


/*

   Many parts of this were based on logic from WiFi Manager by tzapu:

   https://github.com/tzapu/WiFiManager

   which was, in turn, inspired by:

   http://www.esp8266.com/viewtopic.php?f=29&t=2520
   https://github.com/chriscook8/esp-arduino-apboot
   https://github.com/esp8266/Arduino/tree/esp8266/hardware/esp8266com/esp8266/libraries/DNSServer/examples/CaptivePortalAdvanced
   Built by AlexT https://github.com/tzapu

*/



#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include "static.h"


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
//  Global defines
//

#define MAX_COOKIES 10
#define DEBUG_ON


//
//  User settable values (anything in this list can be changed via the web interface)
//


//	Wireless Related
String public_wifi_ssid;
String public_wifi_password;
String private_wifi_ssid;
String private_wifi_password;
String mdns_name;
bool	public_ip_is_static = false;
IPAddress public_static_ip;
IPAddress public_static_gate;
IPAddress public_static_mask;
IPAddress public_static_dns;


//	Account Related 

String        float_hub_id;			// default: outofbox
byte          float_hub_aes_key[16]; 

//	Other Settings

bool	nmea_mux_on;				// default: yes
unsigned int  nmea_mux_port;			// default: 2319
String	web_interface_username;			// default: floathub
String	web_interface_password;			// default: floathub


//	Advanced (hidden) settings

bool	      phone_home_on;
String        float_hub_server;			// default: fdr.floathub.net
unsigned int  float_hub_server_port;		// default: 50003
bool	      virtual_serial_on;		// default: no
unsigned int  virtual_serial_port;		// default: 1923

unsigned long boot_counter;			


//
//	Struct to hold out authentication cookies
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



unsigned long house_keeping_interval     	   = 3000;  // Do house keeping every 3 seconds
unsigned long nmea_housekeeping_interval 	   = 1000;  // Check on nmea connections every second	
unsigned long virtual_serial_housekeeping_interval = 200;   // Check on virtual serial connections every 2/10's of a second

unsigned long house_keeping_previous_timestamp = 0;
unsigned long nmea_housekeeping_previous_timestamp = 0;
unsigned long virtual_serial_housekeeping_previous_timestamp = 0;

//
// Servers of many types and size for such a small little microprocessor :-)
//


ESP8266WebServer  web_server ( 80 );
DNSServer	  dns_server;
WiFiServer       *nmea_server = 0;
WiFiClient        nmea_client[4];
WiFiServer	 *virtual_serial_server = 0;
WiFiClient	  virtual_serial_client;

 
void debug_out(String message)
{
  Serial.print("FHD: ");
  Serial.print(message);
  Serial.print("\r\n");

  if(virtual_serial_client && virtual_serial_client.connected())
  {
    virtual_serial_client.print("FHD: ");
    virtual_serial_client.print(message);
    virtual_serial_client.print("\r\n");
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
  //  This function is called only during factory reset or intial startup
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

  writeStringToEEPROM(53, F("floathub"));
  EEPROM.write(85, '\0');
  
  writeStringToEEPROM(86, F("floathub"));
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
  // NMEA muxer flag and port of 2319
  //

  EEPROM.write(251, 0);
  EEPROM.write(252, 9);
  EEPROM.write(253, 15);
 
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
           debug_out(F("Yay, cookie hitttttt"));
           return true;
         }
      }
    }
  }
  debug_out(F("Authentication Failed"));
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

  for(int i = 0; i < 4; i++)
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
  nmea_server = new WiFiServer(nmea_mux_port);
  nmea_server->begin();
  nmea_server->setNoDelay(true);

}


void nukeVirtualSerialServer()
{

  if(virtual_serial_client)
  {
    virtual_serial_client.stop();
  }
  if(virtual_serial_server)
  {
    virtual_serial_server->close();
    delete virtual_serial_server;
    virtual_serial_server = 0;	
  }

}

void fireUpVirtualSerialServer()
{

  nukeVirtualSerialServer();
  virtual_serial_server = new WiFiServer(virtual_serial_port);
  virtual_serial_server->begin();
  virtual_serial_server->setNoDelay(true);

}




void spitOutIPInput(String &page, String name_stub, String label, const IPAddress& address)
{
 
  page += "<label for='" + name_stub + "1' style='width: 140px;'>" + label + "</label>";
  page += "<input type='number' name='" + name_stub + "1' id='" + name_stub + "1' length='3' maxlength='3' style='width: 60px' value='" + String(address[0]) + "' >";
  page += " . <input type='number' name='" + name_stub + "2' length='3' maxlength='3'  style='width: 60px' value='" + String(address[1]) + "' >";
  page += " . <input type='number' name='" + name_stub + "3' length='3' maxlength='3'  style='width: 60px' value='" + String(address[2]) + "' >";
  page += " . <input type='number' name='" + name_stub + "4' length='3' maxlength='3'  style='width: 60px' value='" + String(address[3]) + "' >";
  page += "<br/>";
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
  page += "<meta http-equiv='refresh' content='8; " + where_to_go + "'/>";
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


  debug_out(F("Enter handleLogin()"));

  if (web_server.hasArg("DISCONNECT"))
  {
    debug_out(F("Disconnection"));
    String header = "HTTP/1.1 301 OK\r\nSet-Cookie: FHSESSION=0\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
    web_server.sendContent(header);
    return;
  }
  if(isAuthenticated())
  {
      String header = "HTTP/1.1 301 OK\r\nLocation: /\r\nCache-Control: no-cache\r\n\r\n";
      web_server.sendContent(header);
      debug_out(F("redirect already logged in"));
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
      debug_out("Log in Successful");
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
  page += "<input name='USER' length='32' placeholder='username'/><br/>";
  page += "<input name='PASS' length='64' type='password' placeholder='password'/><br/><br/>";
  page += "<button type='submit'>Login</button></form></div>";

  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}


void handleReboot()
{
  debug_out(F("Enter handleReboot()"));
  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  sendPleaseWait("/");

  delay(200);
  ESP.restart();  

}




void handleRoot()
{


  debug_out(F("Enter handleRoot()"));
  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  WiFiClient client = web_server.client();
  debug_out("Request from " + client.remoteIP());


  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);

  String private_address;
  IPAddress private_ip = WiFi.softAPIP();
  private_address = String(private_ip[0]) + "." + 
                    String(private_ip[1]) + "." +
                    String(private_ip[2]) + "." +
                    String(private_ip[3]);

  String public_address;
  IPAddress public_ip = WiFi.localIP();
  public_address =  String(public_ip[0]) + "." + 
                    String(public_ip[1]) + "." +
                    String(public_ip[2]) + "." +
                    String(public_ip[3]);

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
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}

void handlePrivateWireless()
{


  debug_out(F("Enter handlePrivateWireless()"));
  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  //
  // First see if we received any from submission
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
  page += "<input name='privssid' value='" + private_wifi_ssid + "'/><br/>";
  page += "<label for='privpassone'>Password: </label>";
  page += "<input name='privpassone' type='password'/><br/>";
  page += "<label for='privpasstwo'>Repeat: </label>";
  page += "<input name='privpasstwo' type='password' /><br/>";
  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br/>";

  page += FPSTR(HTTP_HOMEB);
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}


void kickMDNS()
{
  int attempts = 0;
  while((!MDNS.begin (mdns_name.c_str(), WiFi.localIP())) && attempts < 12)
  {
    debug_out("MDNS died off ... will retry");
    delay(500);
    attempts++;
    if(attempts >= 12)
    {
      debug_out("Can't get MDNS up at all");
    }
  }
}

void handlePublicWireless()
{

  debug_out(F("Enter handlePublicWireless()"));
  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  //
  // First see if we received any from submission
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

          WiFi.disconnect();
          WiFi.mode(WIFI_OFF);


          WiFi.mode(WIFI_AP_STA);
          WiFi.begin(public_wifi_ssid.c_str(), public_wifi_password.c_str());
          byte attempts = 0; 
          while (WiFi.status() != WL_CONNECTED && attempts < 24)
          {
            delay(500);
            attempts++;
          }

	  wifi_station_dhcpc_stop();
	  wifi_station_dhcpc_start();
          delay(500);
	  
          kickMDNS();
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

          sendPleaseWait("http://" + web_server.arg("local1") + "." + web_server.arg("local2") + "." +  web_server.arg("local3") + "." + web_server.arg("local4") + "/public");


          

          WiFi.disconnect();
          WiFi.mode(WIFI_OFF);
          WiFi.mode(WIFI_AP_STA);
          WiFi.begin(public_wifi_ssid.c_str(), public_wifi_password.c_str());
          byte attempts = 0; 
          while (WiFi.status() != WL_CONNECTED && attempts < 24)
          {
            delay(500);
            attempts++;
          }
          WiFi.config(public_static_ip, public_static_gate, public_static_mask, public_static_dns);
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
  page += "<input name='pubssid' value='" + public_wifi_ssid + "' list='available'/><br/>";

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
  page += "<input name='pubpassone' type='password'/><br/>";
  page += "<label for='pubpasstwo'>Repeat: </label>";
  page += "<input name='pubpasstwo' type='password' /><br/>";
  page += "<input type='radio' name='pubadd' value='dynamic' id='dynamic' ";
  if(!public_ip_is_static)
  {
    page += "checked ";
  }
  page += "style='width: 20px; vertical-align: middle;'>";
  page += "<label for='dynamic' style='width: 240px; text-align: left;'>Dynamic Addressing (DHCP)</label><br/>";
  page += "<input type='radio' name='pubadd' value='static' id='static' ";
  if(public_ip_is_static)
  {
    page += "checked ";
  }
  page += "style='width: 20px; vertical-align: middle;'>";
  page += "<label for='static' style='width: 240px; text-align: left;'>Static Addressing</label><br/><br/>";

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
  page += "</form></div><br/>";

  page += FPSTR(HTTP_HOMEB);
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}

void handleOther()
{
  debug_out(F("Enter handleOther()"));
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
      if(nmea_mux_on)
      {
        fireUpNmeaServer();
      }
      else
      {
        nukeNmeaServer();
      }
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
  page += "<label for='muxon'>NMEA Output: </label>";
  page += "<input type='checkbox' name='muxon' value='yes'";
  if(nmea_mux_on)
  {
    page += " checked";
  }
  page += "/><br/>";
  page += "<label for='muxport'>NMEA Port: </label>";
  page += "<input type='number' name='muxport' length='5' maxlength='5' value='" + String(nmea_mux_port) + "' ><br/>";
  page += "<label for='lname'>Device Login: </label>";
  page += "<input name='lname' value='" + web_interface_username + "'/><br/>";
  page += "<label for='lpassone'>Device Password: </label>";
  page += "<input name='lpassone' type='password' maxlength='32'/><br/>";
  page += "<label for='lpasstwo'>Repeat: </label>";
  page += "<input name='lpasstwo' type='password' maxlength='32'/><br/>";
  page += "<label for='localname'>.local Name: </label>";
  page += "<input name='localname' value='" + mdns_name + "'/><br/>";
  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br/>";



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


  debug_out(F("Enter handleAdvanced()"));


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
      if(virtual_serial_on)
      {
        fireUpVirtualSerialServer();
      }
      else
      {
        nukeVirtualSerialServer();
      }
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
  page += "><br/>";
  page += "<label for='fhserver'>FloatHub Server: </label>";
  page += "<input name='fhserver' value='" + float_hub_server + "'/><br/>";
  page += "<label for='fhport'>Port: </label>";
  page += "<input type='number' name='fhport' length='5' maxlength='5' value='" + String(float_hub_server_port) + "' ><br/>";
  page += "<label for='vserialon'>Virtual Terminal: </label>";
  page += "<input type='checkbox' name='vserialon' value='yes'";
  if(virtual_serial_on)
  {
    page += " checked";
  }
  page += "><br/>";
  page += "<label for='vsport'>Port: </label>";
  page += "<input type='number' name='vsport' length='5' maxlength='5' value='" + String(virtual_serial_port) + "' ><br/>";
  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br/>";



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


  debug_out(F("Enter handleAccount()"));
  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  //
  // First see if we received any from submission
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
  page += "<input name='deviceid' value='" + float_hub_id + "' style='width: 250px; font-size: 9pt;'/><br/>";
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

  page += "' style='width: 250px; font-size: 9pt;'/><br/>";
  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br/>";

  page += FPSTR(HTTP_HOMEB);
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );

}

void handleLogo()
{
  web_server.sendContent_P(FLOATHUB_LOGO, sizeof(FLOATHUB_LOGO) );
}

void handleFavicon()
{
  web_server.sendContent_P(FLOATHUB_FAVICON, sizeof(FLOATHUB_FAVICON) );
}


void setup(void)
{
  Serial.begin ( 115200 );

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
      debug_out("Virgin Module");
      init_eeprom_memory();
      break;
    }
  }

  read_eeprom_memory();

  debug_out("");
  debug_out(String("Boot: ") + String(boot_counter) + String(", public_wifi_ssid = ") + public_wifi_ssid);


  //
  // Fire up the WiFi
  //

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(public_wifi_ssid.c_str(), public_wifi_password.c_str());
  WiFi.softAP(private_wifi_ssid.c_str(), private_wifi_password.c_str());
  if(public_ip_is_static)
  {
    WiFi.config(public_static_ip, public_static_gate, public_static_mask, public_static_dns);
  }

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

  //
  //  Set which headers to keep track of
  //

  const char * headerkeys[] = {"User-Agent","Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys)/sizeof(char*);
  web_server.collectHeaders(headerkeys, headerkeyssize );

  web_server.begin();

  delay(1000);


  //
  //	Run MDNS so we can resolve as floathub.local
  //

  kickMDNS();


  //
  // Try and redirect _all_ hostnames on the private network to the Web Server
  //

  dns_server.setErrorReplyCode(DNSReplyCode::NoError);
  dns_server.start(53, "*", WiFi.softAPIP());


  //
  // Bring up nmea_server
  //

  if(nmea_mux_on)
  {
    fireUpNmeaServer();
  }
  else
  {
    nukeNmeaServer();
  }

  //
  // Bring up Virtual Serial Server
  //

  if(virtual_serial_on)
  {
    fireUpVirtualSerialServer();
  }
  else
  {
    nukeVirtualSerialServer();
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
      Serial.print("Nuked nmea client at position ");
      Serial.println(i);
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
    Serial.print("Added nmea client at position ");
    Serial.println(i);
  }

  //
  //  Temp testing
  //

  for(i = 0; i < 4; i++)
  {
    if(nmea_client[i])
    {
      nmea_client[i].println(F("McLovin"));
    }
  }
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

}

void houseKeeping()
{
  debug_out(String(F("WiFi.status()=")) + WiFi.status() + String(F(", IP address: ")) + WiFi.localIP()); 

  long now = millis();
  for(int i; i < MAX_COOKIES; i++)
  {
    if(now - cookies[i].time > 60 * 1000 * 8 && cookies[i].valid == true)	// 8 minute cookie timeout
    {
       //debug_out("Nuked a cookie"
       nukeCookie(i);
    }
  }
}



void loop(void)
{
  /*
      Obviously this is main execution loop. There are a number of values we read and actions we take base on timing
  */
   
  unsigned long current_timestamp = millis();
 
  if(current_timestamp - house_keeping_previous_timestamp >  house_keeping_interval)
  {
    house_keeping_previous_timestamp = current_timestamp;
    houseKeeping();

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

  dns_server.processNextRequest();
  web_server.handleClient();
}



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



unsigned long house_keeping_interval = 3000;		// Do house keeping every 5 seconds
unsigned long house_keeping_previous_timestamp = 0;



ESP8266WebServer web_server ( 80 );
DNSServer	dns_server;

void debug_out(String message)
{
  Serial.print("FHD: ");
  Serial.print(message);
  Serial.print("\r\n");
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
           debug_out(F("Yay, cookie hit"));
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

/*
  if (web_server.hasHeader("Cookie"))
  {   
    Serial.print("Found cookie: ");
    String cookie = web_server.header("Cookie");
    Serial.println(cookie);
  }
*/
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
    page += "<h5>" + public_address + " | floathub.local | <a href='http://" + private_address + "'>" + private_address + "</a></h5>";
  }
  else
  {
    page += "<h5><a href='http://" + public_address + "'>" + public_address + "</a> | <a href='http://floathub.local'>floathub.local</a> | " + private_address + "</h5>";
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
    //  We are autenticated and a form was submitted, better process and act on it
    //

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
	  
          attempts = 0;
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
          return;
        }
        else
        {
          sendPleaseWait("http://" + web_server.arg("local1") + "." + web_server.arg("local2") + "." +  web_server.arg("local3") + "." + web_server.arg("local4") + "/public");
          public_ip_is_static = true;

          // Set Public Wireless for Static 
          public_wifi_ssid = web_server.arg("pubssid");

          //
          //  If no password, we're ok. If something set, change to that
          //

          if(web_server.arg("pubpassone").length() > 1 )
          {
              public_wifi_password = web_server.arg("pubpassone");
          }
          

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
          WiFi.config(
            IPAddress(web_server.arg("local1").toInt(), web_server.arg("local2").toInt(), web_server.arg("local3").toInt(), web_server.arg("local4").toInt()),
            IPAddress(web_server.arg("gate1").toInt(), web_server.arg("gate2").toInt(), web_server.arg("gate3").toInt(), web_server.arg("gate4").toInt()),
            IPAddress(web_server.arg("mask1").toInt(), web_server.arg("mask2").toInt(), web_server.arg("mask3").toInt(), web_server.arg("mask4").toInt()),
            IPAddress(web_server.arg("dns1").toInt(), web_server.arg("dns2").toInt(), web_server.arg("dns3").toInt(), web_server.arg("dns4").toInt())
                     );
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


  spitOutIPInput(page, "local", "Local IP Address: ", WiFi.localIP());
  spitOutIPInput(page, "gate", "Gateway: ", WiFi.gatewayIP());
  spitOutIPInput(page, "mask", "Mask: ", WiFi.subnetMask());
  spitOutIPInput(page, "dns", "DNS: ", WiFi.dnsIP());


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

  String error_message = "";

  if(web_server.arg("savebutton") == "True")
  {
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
        web_interface_password = web_server.arg("lpassone");
      }
    }

    if(web_server.arg("lname").length() < 4)
    {
      error_message += "Login must be 4+ characters";
    }
    else
    {
      web_interface_username = web_server.arg("lname");  
    }

    if(web_server.arg("muxon") == "yes")
    {
      nmea_mux_on = true;
    }
    else
    {
      nmea_mux_on = false;
    }

    nmea_mux_port = web_server.arg("muxport").toInt();
    checkPort(nmea_mux_port);

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
  page += "><br/>";
  page += "<label for='muxport'>NMEA Port: </label>";
  page += "<input type='number' name='muxport' length='5' maxlength='5' value='" + String(nmea_mux_port) + "' ><br/>";
  page += "<label for='lname'>Device Login: </label>";
  page += "<input name='lname' value='" + web_interface_username + "'/><br/>";
  page += "<label for='lpassone'>Device Password: </label>";
  page += "<input name='lpassone' type='password' maxlength='32'/><br/>";
  page += "<label for='lpasstwo'>Repeat: </label>";
  page += "<input name='lpasstwo' type='password' maxlength='32'/><br/>";
  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br/>";



  page += FPSTR(HTTP_HOMEB);
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

    if(web_server.arg("vserialon") == "yes")
    {
      virtual_serial_on = true;
    }
    else
    {
      virtual_serial_on = false; 
    }  

    virtual_serial_port = web_server.arg("vsport").toInt();
    checkPort(virtual_serial_port);
 

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



  //public_wifi_ssid = "SuperMOO";
  public_wifi_ssid = "NoWorky";
  public_wifi_password = "VeryS1lly";

  //public_wifi_ssid = "FunkyChicken";
  //public_wifi_password = "tits";

  float_hub_id = "outofbox";
  for(int i = 0; i < 16; i++)
  {
    float_hub_aes_key[i] = i;
  }

  public_ip_is_static = false;
  public_static_ip   = IPAddress(192,168,1,42);
  public_static_gate = IPAddress(192,168,1,1);
  public_static_mask = IPAddress(255,255,255,0);
  public_static_dns  = IPAddress(192,168,1,1);

  web_interface_username = "floathub";
  web_interface_password = "floathub";
 
  nmea_mux_on = true;
  nmea_mux_port = 2319;

  phone_home_on = true;
  float_hub_server = "fdr.floathub.net";
  float_hub_server_port = 50003;
  virtual_serial_on = false;
  virtual_serial_port = 1923;


  
  //ESP.eraseConfig();
  byte mac_array[6];
  WiFi.softAPmacAddress(mac_array);

/*
  Serial.print("MAC: ");
  Serial.print(mac_array[5],HEX);
  Serial.print(":");
  Serial.print(mac_array[4],HEX);
  Serial.print(":");
  Serial.print(mac_array[3],HEX);
  Serial.print(":");
  Serial.print(mac_array[2],HEX);
  Serial.print(":");
  Serial.print(mac_array[1],HEX);
  Serial.print(":");
  Serial.println(mac_array[0],HEX);
*/

  private_wifi_password = "FloatHub"; // min 8

  private_wifi_ssid = "FloatHub_";
  private_wifi_ssid += String(mac_array[3], HEX);
  private_wifi_ssid += String(mac_array[4], HEX);
  private_wifi_ssid += String(mac_array[5], HEX);


//  debug_out(String("Creating AP with SSID of ") + private_wifi_ssid);
	
//  Serial.print("I think I want to create a WiFi network called: ");
//  Serial.println(private_wifi_ssid);


  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(public_wifi_ssid.c_str(), public_wifi_password.c_str());
  WiFi.softAP(private_wifi_ssid.c_str(), private_wifi_password.c_str());

  web_server.on ( "/", handleRoot );
  web_server.on ( "/logo.png", handleLogo );
  web_server.on ( "/favicon.ico", handleFavicon );
  web_server.on ( "/login", handleLogin );
  web_server.on ( "/public", handlePublicWireless );
  web_server.on ( "/private", handlePrivateWireless );
  web_server.on ( "/other", handleOther );
  web_server.on ( "/advanced", handleAdvanced );
  web_server.on ( "/account", handleAccount );

  //
  //  Set which headers to keep track of
  //

  const char * headerkeys[] = {"User-Agent","Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys)/sizeof(char*);
  web_server.collectHeaders(headerkeys, headerkeyssize );

  web_server.begin();

  delay(1000);

  mdns_name = "floathub";
  int attempts = 0, max_attempts = 10;
  while((!MDNS.begin (mdns_name.c_str(), WiFi.localIP())) && attempts < max_attempts)
  {
    debug_out("MDNS died off ... will retry");
    delay(500);
    attempts++;
    if(attempts >= max_attempts)
    {
      debug_out("Can't get MDNS up at all");
    }
  }

  dns_server.setErrorReplyCode(DNSReplyCode::NoError);
  dns_server.start(53, "*", WiFi.softAPIP());

}


void houseKeeping()
{

  debug_out(String(F("WiFi.status()=")) + WiFi.status() + String(F(", IP address: ")) + WiFi.localIP()); 

  long now = millis();
  for(int i; i < MAX_COOKIES; i++)
  {
    if(now - cookies[i].time > 60 * 1000 * 8 && cookies[i].valid == true)	// 8 minute cookie timeout
    {
       Serial.println("Nuked a cookie");
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
  dns_server.processNextRequest();
  web_server.handleClient();
}



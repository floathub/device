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


//	Account Related - Note that the definitive source for these is the Mega, _not_ the esp8266

String        float_hub_id;			// default: outofbox
String        float_hub_server;			// default: fdr.floathub.net
unsigned int  float_hub_server_port;		// default: 50003
unsigned int  multiplexer_port;			// default: 2319
byte          float_hub_aes_key[16]; 


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



void printCookies()
{
/*
  for(int i=0; i < MAX_COOKIES; i++)
  {
    Serial.print("Cookie " + String(i) + ": ip=");
    Serial.print(cookies[i].address);
    Serial.print(", rand=");
    Serial.print(cookies[i].random_number);
    Serial.print(", time=");
    Serial.print(cookies[i].time);
    Serial.print(", valid=");
    Serial.println(cookies[i].valid);
  }
*/
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
//  Serial.println("Enter isAuthenticated()");
  if (web_server.hasHeader("Cookie"))
  {   
//    Serial.print("Found cookie: ");
    String cookie = web_server.header("Cookie");
//    Serial.println(cookie);
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
//	 Serial.print("Comparing \"");
//         Serial.print(cookie.substring(cut_spot));
//         Serial.print("\" to \"");
//         Serial.print(cookies[i].random_number);
//         Serial.println("\"");
         if( cookie.substring(cut_spot) == String(cookies[i].random_number) &&
             cookies[i].valid == true &&
             client.remoteIP() == cookies[i].address
           )
         {
           //
           //  Authenticated, so we need to "touch" the timestamp for this cookie
           //
          
           cookies[i].time = millis();
           Serial.println("Yay, cookie hit");
           return true;
         }
      }
      Serial.println("Cut spot was there, but no cookie matched");
    }
  }
  Serial.println("Authentication Failed");
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


void sendPleaseWait(String where_to_go)
{

  String page = FPSTR(HTTP_INITA);
  page += "<meta http-equiv='refresh' content='8; " + where_to_go + "'/>";
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
//  page += FPSTR(HTTP_LOGOA);
  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Please Wait</h2>";
  page += "<p style='margin: 20px;'>Update in progress. Will attempt to redirect you in a few seconds ...</p>";

  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );


}

void handleLogin()
{


  Serial.println("Enter handleLogin()");

  if (web_server.hasHeader("Cookie"))
  {   
    Serial.print("Found cookie: ");
    String cookie = web_server.header("Cookie");
    Serial.println(cookie);
  }
  if (web_server.hasArg("DISCONNECT")){
    Serial.println("Disconnection");
    String header = "HTTP/1.1 301 OK\r\nSet-Cookie: FHSESSION=0\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
    web_server.sendContent(header);
    return;
  }
  if(isAuthenticated())
  {
      String header = "HTTP/1.1 301 OK\r\nLocation: /\r\nCache-Control: no-cache\r\n\r\n";
      web_server.sendContent(header);
      Serial.println("redirect already logged in");
      return;
  }
  if (web_server.hasArg("USER") && web_server.hasArg("PASS"))
  {
    Serial.println("Well, made it this far");
    if (web_server.arg("USER") == "admin" &&  web_server.arg("PASS") == "admin" )
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

      Serial.print("Going to set cookie ");
      Serial.println(cookie_to_use);


      WiFiClient client = web_server.client();

      cookies[cookie_to_use].valid = true;
      cookies[cookie_to_use].time = millis();
      cookies[cookie_to_use].address = client.remoteIP();
      cookies[cookie_to_use].random_number = random(1000000,10000000);

      printCookies();

      String header = "HTTP/1.1 301 OK\r\nSet-Cookie: FHSESSION=" + String(cookies[cookie_to_use].random_number) + "\r\nLocation: /\r\nCache-Control: no-cache\r\n\r\n";
      web_server.sendContent(header);
      Serial.println("Log in Successful");
      return;
    }
  }


  //
  //  Guess we need to show login page
  //


  char temp[800];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;



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


  Serial.println("Enter handleRoot()");
  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  char temp[800];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  WiFiClient client = web_server.client();
  Serial.print("I think see a request from ");
  Serial.println(client.remoteIP());


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


  Serial.println("Enter handlePrivateWireless()");
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

    Serial.print("I see the new private ssid as");
    Serial.println(web_server.arg("privssid"));

/*
    if(web_server.arg("pubpassone") == web_server.arg("pubpasstwo"))
    { 
      Serial.println("The passwords do match");
      if(web_server.arg("pubssid").length() < 1 )
      {
        error_message += "Network Name cannot be blank.";
      }
      else
      {
        if(web_server.arg("pubadd") == "dynamic")
        {
          // Set Public Wireless for DHCP
          public_wifi_ssid = web_server.arg("pubssid");
          public_wifi_password = web_server.arg("pubpassone");
          
          sendPleaseWait("/public");

          WiFi.disconnect();
          WiFi.mode(WIFI_OFF);
          WiFi.mode(WIFI_AP_STA);
          WiFi.begin(public_wifi_ssid.c_str(), public_wifi_password.c_str());
          byte attempts = 0; 
          while (WiFi.status() != WL_CONNECTED && attempts < 24)
          {
            Serial.print(".");
            delay(500);
            attempts++;
          }
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
          // Set Public Wireless for Static 
        }
      }
    }
    else
    {
      error_message += "The passwords do not match";
    }
*/
  }

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
  page += FPSTR(HTTP_LOGOA);
  page += FPSTR(HTTP_DIV_B);
  page += "<h2>Private Wireless Network</h2>";

  page += "<form method='post' action='/private'>";
  page += "<label for='privssid'>Network Name: </label>";
  page += "<input name='privssid' value='" + private_wifi_ssid + "'/><br/>";
  page += "<label for='privpassone'>Password: </label>";
  page += "<input name='privpassone' type='password'/><br/>";
  page += "<label for='privpasstwo'>Repeat: </label>";
  page += "<input name='privpasstwo' type='password' /><br/>";
  page += "<button type='submit' name='savebutton' value='True'>Save</button>";
  page += "</form></div><br/>";

  page += "<form method='post' action='/'>";
  page += "<button type='submit'>Back</button></form>";
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}



void handlePublicWireless()
{

  Serial.println("Enter handlePublicWireless()");
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

    Serial.print("I see the new ssid as");
    Serial.println(web_server.arg("pubssid"));

    if(web_server.arg("pubpassone") == web_server.arg("pubpasstwo"))
    { 
      Serial.println("The passwords do match");
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
            Serial.print(".");
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
            Serial.print(".");
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

  page += "<form method='post' action='/'>";
  page += "<button type='submit'>Back</button></form></div>";
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}

void handleOther()
{


  Serial.println("Enter handleOther()");
  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  char temp[800];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
  page += FPSTR(HTTP_LOGOA);
  page += "<h2>Account Settings</h2>";

  page += "<p>I am a silly other form</p>";

  page += "<form method='post' action='/'>";
  page += "<button type='submit'>Home</button></form>";
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}

void handleAdvanced()
{

  //
  // No obvious way to arrive at this page so it is "secret", but note that
  // user must still first be authenticated like any other page.
  //


  Serial.println("Enter handleAdvanced()");
  if(!isAuthenticated())
  {
    String header = "HTTP/1.1 301 OK\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
    web_server.sendContent(header);
    return;
  }

  char temp[800];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
  page += FPSTR(HTTP_LOGOA);

  page += "<h2>Uber Secret Advanced Settings</h2>";

  page += "<p>I am a silly advanced form</p>";

  page += "<form method='post' action='/'>";
  page += "<button type='submit'>Home</button></form>";
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}

void handleAccount()
{


  Serial.println("Enter handleAccount()");
  if(!isAuthenticated())
  {
    web_server.sendContent(FPSTR(HTTP_REDIRECT));
    return;
  }

  char temp[800];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);
  page += FPSTR(HTTP_LOGOA);

  page += "<h2>Account Settings</h2>";

  page += "<p>I am a hedgehog form</p>";

  page += "<form method='post' action='/'>";
  page += "<button type='submit'>Home</button></form>";
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
  //delay(500);
  Serial.println("Hello there");


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


  public_ip_is_static = false;
  public_static_ip   = IPAddress(192,168,1,42);
  public_static_gate = IPAddress(192,168,1,1);
  public_static_mask = IPAddress(255,255,255,0);
  public_static_dns  = IPAddress(192,168,1,1);



  
  //ESP.eraseConfig();
  byte mac_array[6];
  WiFi.softAPmacAddress(mac_array);

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

  private_wifi_password = "FloatHub"; // min 8

  private_wifi_ssid = "FloatHub_";
  private_wifi_ssid += String(mac_array[3], HEX);
  private_wifi_ssid += String(mac_array[4], HEX);
  private_wifi_ssid += String(mac_array[5], HEX);

/*
  char mac_char[18];
  for (int i = 3; i < sizeof(mac_array); ++i)
  {
    Serial.print("mac_char is now 
    sprintf(mac_char,"%s%02x",mac_char,mac_array[i]);
  }

  private_wifi_ssid += String(mac_char);
*/

//  debug_out(String("Creating AP with SSID of ") + private_wifi_ssid);
	
  Serial.print("I think I want to create a WiFi network called: ");
  Serial.println(private_wifi_ssid);


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

  printCookies();

}


void houseKeeping()
{

  Serial.print("WiFi.status()=");
  Serial.print(WiFi.status());
  Serial.print( ", IP address: " );
  Serial.println( WiFi.localIP() );



  long now = millis();
  for(int i; i < MAX_COOKIES; i++)
  {
    if(now - cookies[i].time > 60 * 1000 * 8 && cookies[i].valid == true)	// 8 minute cookie timeout
    {
       printCookies();
       Serial.println("Nuked a cookie");
       nukeCookie(i);
       printCookies();
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





/*


const char *ssid = "SuperMOO";
const char *password = "VeryS1lly";

ESP8266WebServer server ( 80 );

const int led = 13;

void handleRoot() {
	digitalWrite ( led, 1 );
	char temp[400];
	int sec = millis() / 1000;
	int min = sec / 60;
	int hr = min / 60;

	snprintf ( temp, 400,

"<html>\
  <head>\
    <meta http-equiv='refresh' content='5'/>\
    <title>ESP8266 Demo</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Hello from ESP8266!</h1>\
    <p>Uptime: %02d:%02d:%02d</p>\
    <img src=\"/test.svg\" />\
  </body>\
</html>",

		hr, min % 60, sec % 60
	);
	server.send ( 200, "text/html", temp );
	digitalWrite ( led, 0 );
}

void handleNotFound() {
	digitalWrite ( led, 1 );
	String message = "File Not Found Fucking Pumper Lumper Lumpy nutjob\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";

	for ( uint8_t i = 0; i < server.args(); i++ ) {
		message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
	}

	server.send ( 404, "text/plain", message );
	digitalWrite ( led, 0 );
}


void drawGraph() {
  String out = "";
  char temp[100];
  out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"150\">\n";
  out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
  out += "<g stroke=\"black\">\n";
  int y = rand() % 130;
  for (int x = 10; x < 390; x+= 10) {
    int y2 = rand() % 130;
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
    out += temp;
    y = y2;
  }
  out += "</g>\n</svg>\n";

  server.send ( 200, "image/svg+xml", out);
}

void setup ( void ) {
	pinMode ( led, OUTPUT );
	digitalWrite ( led, 0 );
	Serial.begin ( 115200 );
	WiFi.begin ( ssid, password );
	Serial.println ( "" );

	// Wait for connection
	while ( WiFi.status() != WL_CONNECTED ) {
		delay ( 500 );
		Serial.print ( "." );
	}

	Serial.println ( "" );
	Serial.print ( "Connected to " );
	Serial.println ( ssid );
	Serial.print ( "IP address: " );
	Serial.println ( WiFi.localIP() );

	if ( MDNS.begin ( "floathub" ) ) {
		Serial.println ( "MDNS responder started" );
	}

	server.on ( "/", handleRoot );
	server.on ( "/test.svg", drawGraph );
	server.on ( "/inline", []() {
		server.send ( 200, "text/plain", "this works as well" );
	} );
	server.onNotFound ( handleNotFound );
	server.begin();
	Serial.println ( "HTTP server started" );
}

void loop ( void ) {
	server.handleClient();
}

*/
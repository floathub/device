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

#define MAX_COOKIES 10
COOKIES cookies[MAX_COOKIES];


//
//  Global settings and timer values
//

#define DEBUG_ON

String external_wifi_ssid;
String external_wifi_password;

String local_wifi_ssid;
String local_wifi_password;

String mdns_name;

unsigned long house_keeping_interval = 3000;		// Do house keeping every 5 seconds
unsigned long house_keeping_previous_timestamp = 0;



ESP8266WebServer web_server ( 80 );
DNSServer	dns_server;



void printCookies()
{
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

void kick_wifi(void)
{
  WiFi.begin(external_wifi_ssid.c_str(), external_wifi_password.c_str());
  WiFi.softAP(local_wifi_ssid.c_str(), local_wifi_password.c_str());
}


bool isAuthenticated()
{
  Serial.println("Enter isAuthenticated()");
  if (web_server.hasHeader("Cookie"))
  {   
    Serial.print("Found cookie: ");
    String cookie = web_server.header("Cookie");
    Serial.println(cookie);
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
	 Serial.print("Comparing \"");
         Serial.print(cookie.substring(cut_spot));
         Serial.print("\" to \"");
         Serial.print(cookies[i].random_number);
         Serial.println("\"");
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

  WiFiClient client = web_server.client();
  Serial.print("I think see a request from ");
  Serial.println(client.remoteIP());


  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);

  page += "<img src='./logo.png' style='width: 360px;'>";
  page += "<h2>Device Login</h2>";
  page += "<form method='post' action='http://floathub.local/login'>";
  page += "<input name='USER' length='32' placeholder='username'><br/><br/>";
  page += "<input name='PASS' length='64' type='password' placeholder='password'><br/><br/>";
  page += "<button type='submit'>Login</button></form>";

  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}




void handleRoot()
{


  Serial.println("Enter handleRoot()");
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

  WiFiClient client = web_server.client();
  Serial.print("I think see a request from ");
  Serial.println(client.remoteIP());


  String page = FPSTR(HTTP_INITA);
  page += FPSTR(HTTP_TITLE);
  page += FPSTR(HTTP_STYLE);
  page += FPSTR(HTTP_DIV_A);

  page += "<img src='./logo.png' style='width: 360px;'>";
  page += "<h2>Device Settings</h2>";
  page += "<p>Yah, you are logged in</p>";
  page += "<form method='post' action='http://floathub.local/login'>";
  //page += "<hidden name='DISCONNECT' value='YES'>";
  page += "<button type='submit' name='DISCONNECT'>Logout</button></form>";
  page += FPSTR(HTTP_CLOSE);

  web_server.send ( 200, "text/html", page );
}

void handleLogo()
{
  web_server.sendContent_P(FLOATHUB_LOGO, sizeof(FLOATHUB_LOGO) );
}

void handleAdmin()
{
  char temp[400];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  snprintf ( temp, 400,

"<html>\
  <head>\
    <meta http-equiv='refresh' content='5'/>\
    <title>FloatHub Demo</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Hello from FloatHub!</h1>\
    <p>Uptime: %02d:%02d:%02d</p>\
  </body>\
</html>", hr, min % 60, sec % 60 );

  web_server.send ( 200, "text/html", temp );
}


void debug_out(String message)
{
  Serial.print("FHD: ");
  Serial.print(message);
  Serial.print("\r\n");
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



  external_wifi_ssid = "SuperMOO";
  //external_wifi_ssid = "mmoo";
  external_wifi_password = "VeryS1lly";
  
  //ESP.eraseConfig();
  byte mac_array[6];
  WiFi.softAPmacAddress(mac_array);

  local_wifi_ssid = "FloatHub_";
  local_wifi_p4assword = "FloatHub"; // min 8

  char mac_char[18];
  for (int i = 3; i < sizeof(mac_array); ++i)
  {
    sprintf(mac_char,"%s%02x",mac_char,mac_array[i]);
  }

  local_wifi_ssid += String(mac_char);

  debug_out(String("Creating AP with SSID of ") + local_wifi_ssid);
	
  //Serial.print("I think I want to create a WiFi network called: ");
  //Serial.println(local_wifi_ssid);


  WiFi.mode(WIFI_AP_STA);
  kick_wifi();

  web_server.on ( "/", handleRoot );
  web_server.on ( "/logo.png", handleLogo );
  web_server.on ( "/login", handleLogin );

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
  Serial.print ( "IP address: " );
  Serial.println ( WiFi.localIP() );
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


/*

void check_external_wifi()
{
  Serial.println("I am checking external Wifi");

  if(WiFi.status() == WL_CONNECTED)
  {
    #ifdef DEBUG_ON
    Serial.println("WiFi shows connected");
    #endif    
    return;
  }
  if(WiFi.status() == WL_IDLE_STATUS)
  {
    #ifdef DEBUG_ON
    Serial.println("WiFi shows connection attempt in progress");
    #endif    
    return;
  }
  #ifdef DEBUG_ON
  Serial.println("WiFi kicking off connection (lying)");
  #endif    
  //WiFi.begin ( external_wifi_ssid.c_str(), external_wifi_password.c_str());
  
}

*/

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
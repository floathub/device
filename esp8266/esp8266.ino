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
//#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>

//
//  Global settings and timer values
//

#define DEBUG_ON

String external_wifi_ssid;
String external_wifi_password;

String local_wifi_ssid;
String local_wifi_password;

String mdns_name;

unsigned long external_wifi_checking_interval = 3000;		// Check and (re-)connect to upstream/external WiFi network every 3 seconds
unsigned long external_wifi_checking_previous_timestamp = 0;

ESP8266WebServer web_server ( 80 );
DNSServer	dns_server;


void kick_wifi(void)
{
  WiFi.begin(external_wifi_ssid.c_str(), external_wifi_password.c_str());
  WiFi.softAP(local_wifi_ssid.c_str(), local_wifi_password.c_str());
}



void handleRoot()
{
  char temp[400];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  WiFiClient client = web_server.client();
  Serial.print("I think see a request from ");
  Serial.println(client.remoteIP());

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




void setup(void)
{
  Serial.begin ( 115200 );
  //delay(500);
  Serial.println("Hello there");



  ESP.eraseConfig();
  delay(1000);
  wifi_set_phy_mode(PHY_MODE_11B);



  external_wifi_ssid = "SuperMOO";
  //external_wifi_ssid = "mmoo";
  external_wifi_password = "VeryS1lly";
  
  //ESP.eraseConfig();
  byte mac_array[6];
  WiFi.softAPmacAddress(mac_array);

  local_wifi_ssid = "FloatHub_";
  local_wifi_password = "FloatHub"; // min 8

  char mac_char[18];
  for (int i = 3; i < sizeof(mac_array); ++i)
  {
    sprintf(mac_char,"%s%02x",mac_char,mac_array[i]);
  }

  Serial.print("Actual values: ");
  Serial.println(mac_char);

  local_wifi_ssid += String(mac_char);

  Serial.print("I think I want to create a WiFi network called: ");
  Serial.println(local_wifi_ssid);


  WiFi.mode(WIFI_AP_STA);
  kick_wifi();

  web_server.on ( "/", handleRoot );
  web_server.on ( "/admin", handleRoot );
  web_server.begin();

  delay(2000);

  mdns_name = "floathub";
  if(MDNS.begin (mdns_name.c_str(), WiFi.localIP()))
  {
    Serial.print("MDNS seems to be up and running with name of");
    Serial.println(mdns_name.c_str());
  }
  else
  {
    Serial.println("MDNS died a horrible death");
  }

  dns_server.dnsServer->setErrorReplyCode(DNSReplyCode::NoError);
  dns_server->start(DNS_PORT, "*", WiFi.softAPIP());

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
 
  if(current_timestamp - external_wifi_checking_previous_timestamp >  external_wifi_checking_interval)
  {
    external_wifi_checking_previous_timestamp = current_timestamp;
  //  check_external_wifi();
	Serial.print ( "IP address: " );
	Serial.println ( WiFi.localIP() );

  } 
  dns_server->processNextRequest();
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
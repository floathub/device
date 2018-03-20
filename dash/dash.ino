/*

  FloatHub Arduino Code
  (c) 2011-2017 Modiot Labs

  DASH Code to send messages from ESP8266 out via cellular link
  Note that contrary to what you might think, the DASH is the _MASTER_

  Begin in Guinea in March, 2018
 
*/

#include <SPI.h>

//
// Global defines
//

#define SPI_CS L07
#define DEBUG_ON
#define USE_RGB_LED				// Light the Dash's LGB with various colors to indicate cellular status
#define MAX_LATEST_MESSAGE_SIZE 512
#define HOUSEKEEPING_INTERVAL       1000        // Check our network status and do other core tasks once every second
//#define MESSAGE_COMPLETION_LIMIT 60000 	// If we can't assemble a whole message within 1 minute, assume it is garbled and throw out
#define MESSAGE_COMPLETION_LIMIT   10000 	// Testing
#define REGULAR_POLLING_INTERVAL    3000  	// When otherwise idle, poll the ESP8266 for a message every 3 seconds
#define ASSEMBLE_POLLING_INTERVAL    500	// Poll the ESP8266 every half second when we are in the process of assembling a message
//#define SIMULATE_SENDING          2000	// If defined, simulate sending the message out after defined period (debug code without paying data fees) 
#define HOLOGRAM_TOPIC     "FHUBTESTING"	// The "topic" that goes with the message to Hologram's Systems
//
//  Global vars
//

String latest_cellular_message_to_send;
bool latest_message_complete;
unsigned long housekeeping_timestamp = 0;
unsigned long message_completion_timestamp = 0;	
unsigned long regular_polling_timestamp = 0;
unsigned long assemble_polling_timestamp = 0;
int current_cellular_status = -1;
#ifdef SIMULATE_SENDING
unsigned long simulated_sending_timestamp = 0;
#endif

String network_operator;
String sim_card_number;


bool check_message()
{

  int checksum_start = latest_cellular_message_to_send.lastIndexOf('@');
  if(checksum_start != latest_cellular_message_to_send.length() - 3)
  {
    #ifdef DEBUG_ON
    Serial.println(F("Killed message, no checksum"));
    #endif
    return false;
  }

  String checksum_a = latest_cellular_message_to_send.substring(checksum_start + 1);

  latest_cellular_message_to_send = latest_cellular_message_to_send.substring(0, latest_cellular_message_to_send.length() - 3);

  byte a_byte = 0;
  for (int i = 0; i < latest_cellular_message_to_send.length(); i++)
  {
    a_byte = a_byte ^ latest_cellular_message_to_send.charAt(i); 
  }

  String checksum_b = String(a_byte, HEX);
  checksum_b.toUpperCase();
  if(checksum_b.length() < 2)
  {
    checksum_b = String(F("0")) + checksum_b;
  } 

  if(checksum_a != checksum_b)
  {
    #ifdef DEBUG_ON
    Serial.println(String(F("WOW WOW Killed bad message: ")) + checksum_a + String(F(" != ")) + checksum_b);
    #endif
    return false;
  }

  return true;
}

void try_to_send()
{
  //
  //	This blocks if we are not connected, so we try and send only if
  // connected
  //
  
  #ifdef SIMULATE_SENDING

  Serial.println(String("SIM Tranmission:\"") + latest_cellular_message_to_send + "\""); 
  latest_cellular_message_to_send = "";

  #else

  if(latest_cellular_message_to_send.length() > 0 &&
     latest_message_complete &&
     HologramCloud.getConnectionStatus() == CLOUD_CONNECTED)
  {
    if(HologramCloud.sendMessage(latest_cellular_message_to_send, HOLOGRAM_TOPIC))
    {
      #ifdef DEBUG_ON
      Serial.println("Message sent over cellular");
      #endif
      latest_cellular_message_to_send = "";
    }
    else
    {
      #ifdef DEBUG_ON
      Serial.println("Message failed over cellular");
      #endif
    }
  }

  #endif
}

void send_SPI_message(const char * message)
{
  uint8_t i = 0;
  size_t length = strlen(message);
  if( length > 32 )
  {
    #ifdef DEBUG_ON
    Serial.println("Asked to send message longer than 32 bytes!");
    #endif
    length = 32;
  }

  SPI.beginTransaction(L07);
  SPI.transfer(0x02);
  SPI.transfer(0x00);

  while (length-- && i < 32)
  {
    SPI.transfer(message[i++]);
  }

  //
  // Must _always_ pad with zeros to get to 32 bytes
  //
  while (i++ < 32)
  {
    SPI.transfer(0);
  }

  delay(10);

  SPI.transfer(0x03);
  SPI.transfer(0x00);

  char data[33];
  data[32] = 0;
  for(int i = 0; i < 32; i++)
  {
    data[i] = SPI.transfer(0x00);
  }

  SPI.endTransaction();
  
  String return_message = String(data);

  if(return_message.length() > 0)
  {
    //
    // We got something back, so begin assembling timer if we're not
    // already in the middle of that
    //
    if(latest_cellular_message_to_send.length() == 0)
    {
      #ifdef DEBUG_ON
      Serial.println("Reset Timestamp");
      #endif 
      message_completion_timestamp = millis();
    }
    latest_message_complete = false;

    if(latest_cellular_message_to_send.length() + return_message.length() < MAX_LATEST_MESSAGE_SIZE)
    {
      latest_cellular_message_to_send += return_message;
    }
    else
    {
      //
      // We seem to be over our max message size ... no good options really
      //

      latest_cellular_message_to_send = return_message;
    }

    if(latest_cellular_message_to_send.endsWith("\r\n"))
    {
      latest_message_complete = true;
      latest_cellular_message_to_send = latest_cellular_message_to_send.substring(0, latest_cellular_message_to_send.length() - 2);
      #ifdef DEBUG_ON
      Serial.print("Messge Ready: \"");
      Serial.print(latest_cellular_message_to_send);
      Serial.println("\"");
      #endif

      //
      //  Tiny command set
      //
      if(latest_cellular_message_to_send == "reset")
      {
        Serial.println("Asked to Reset ...");
        delay(2000);
        latest_cellular_message_to_send = "";
        HologramCloud.resetSystem();
        Dash.deepSleepSec(2);
      }
      else
      {
        if(check_message())
        {
          try_to_send();
        }
        else
        {
          latest_cellular_message_to_send = "";
        }
      }
    }
  }
}

void poll_esp8266()
{
  String message_to_send;
  if(current_cellular_status == CLOUD_CONNECTED)
  {
    if(latest_cellular_message_to_send.length() == 0)
    {
      message_to_send = "LU "; // link up, send me a message
    }
    else if(latest_message_complete)
    {
      message_to_send = "LQ ";	// link up, message in queue
    }
    else
    {
      message_to_send = "LR "; 	// link up, send me _rest_ of message
    }
  }
  else
  {
     message_to_send = "NL "; 	// No link
  }

  //
  //  Add the SIM Card number
  //

  if(sim_card_number.length() > 22)
  {
    message_to_send += sim_card_number.substring(0, 22) + ' ';
  }
  else
  {
    message_to_send += sim_card_number + ' ';
  }


  //
  //  Add the Operator
  //

  message_to_send += network_operator; 

  //
  // Lop off anything over the 32 character limit
  //

  if(message_to_send.length() > 32)
  {
    message_to_send = message_to_send.substring(0,33);
  }

  #ifdef DEBUG_ON
  Serial.print("Polling ESP8266 with message of \"");
  Serial.print(message_to_send);
  Serial.println("\""); 
  #endif

  send_SPI_message((const char *) message_to_send.c_str());

}

void do_housekeeping()
{
  int new_cellular_status = 0;
  #ifdef DEBUG_ON
  Serial.println("Entering Housekeeping...");
  #endif

  //
  // Get current cellular status
  //

  new_cellular_status = HologramCloud.getConnectionStatus();

  if(current_cellular_status != new_cellular_status)
  {
    current_cellular_status = new_cellular_status;
    #ifdef USE_RGB_LED
    switch (current_cellular_status)
    {
      case CLOUD_REGISTERED:
        HologramCloud.setRGB(0x333300); // Yellowish; almost green
        Dash.setLED(false);
        break;
          
      case CLOUD_CONNECTED:
        HologramCloud.setRGB(0x000066); // Blue, all good
        Dash.setLED(false);
        break;

      case CLOUD_ERR_UNAVAILABLE:
        HologramCloud.setRGB(0x660000); // Red, could not talk to modem, flash LED every second
        Dash.pulseLED(50,1000);
        break;
  
      case CLOUD_ERR_SIM:
        HologramCloud.setRGB(0x006666); // Teal; No valid SIM card
        Dash.setLED(false);
        break;
  
      case CLOUD_ERR_UNREGISTERED:
        HologramCloud.setRGB(0x660000); // Red; Could not register, flash LED every 4 seconds
        Dash.pulseLED(50,4000);
        break;

      case CLOUD_ERR_SIGNAL:
        HologramCloud.setRGB(0x660066); // Purple; no tower/signal in sight
        Dash.setLED(false);
        break;

      case CLOUD_ERR_CONNECT:
        HologramCloud.setRGB(0x660000); // Red, SIM card not active, flash every 2 seconds
        Dash.pulseLED(50,2000);
        break;

      case CLOUD_ERR_MODEM_OFF :
        HologramCloud.setRGB(0x660000); // Red, Modem is off, flash every 6 seconds
        Dash.pulseLED(50,6000);
        break;
    }
  }
  #endif

  #ifdef DEBUG_ON
  Serial.print("Connection Status: ");
  switch (current_cellular_status)
  {
    case CLOUD_REGISTERED:
      Serial.println("CLOUD_REGISTERED");
      break;
          
    case CLOUD_CONNECTED:
      Serial.println("CLOUD_CONNECTED");
      break;

    case CLOUD_ERR_UNAVAILABLE:
      Serial.println("CLOUD_ERR_UNAVAILABLE");
      break;
  
    case CLOUD_ERR_SIM:
      Serial.println("CLOUD_ERR_SIM");
      break;
  
    case CLOUD_ERR_UNREGISTERED:
      Serial.println("CLOUD_ERR_UNREGISTERED");
      break;

    case CLOUD_ERR_SIGNAL:
      Serial.println("CLOUD_ERR_SIGNAL");
      break;

    case CLOUD_ERR_CONNECT:
      Serial.println("CLOUD_ERR_CONNECT");
      break;

    case CLOUD_ERR_MODEM_OFF :
      Serial.println("CLOUD_ERR_MODEM_OFF ");
      break;
  }
  #endif  

  //
  //  Get current network provider and SIM Card number
  //

  network_operator = HologramCloud.getNetworkOperator();
  #ifdef DEBUG_ON
  Serial.print("Network Operator is \"");
  Serial.print(network_operator);
  Serial.println("\"");
  #endif

  sim_card_number = HologramCloud.getICCID();
  sim_card_number.replace(' ','_');
  #ifdef DEBUG_ON
  Serial.print("SIM number is \"");
  Serial.print(sim_card_number);
  Serial.println("\"");
  #endif


  //
  // Regularly try to send
  //

  try_to_send();

}

void setup()
{
  pinMode(L07, OUTPUT);
  digitalWrite(L07, LOW);
  Serial.begin(115200);
  delay(1000);
  SPI.begin();
  delay(5000); // Make sure the ESP8266 has time to boot first

  #ifdef DEBUG
  Serial.println("==== Startin' up after ESP8266 headstart delay ====");
  #endif

  digitalWrite(L07, HIGH);

  latest_cellular_message_to_send.reserve(MAX_LATEST_MESSAGE_SIZE);
  latest_cellular_message_to_send = "";
  latest_message_complete = true;

  current_cellular_status = -1; // Unknown
  network_operator = "Unknown";
  sim_card_number = "Unknown";
}


void loop()
{
  unsigned long current_timestamp = millis();

  if(current_timestamp - housekeeping_timestamp >= HOUSEKEEPING_INTERVAL)
  {
    do_housekeeping();
    housekeeping_timestamp = current_timestamp;
  }

  if(current_timestamp - message_completion_timestamp >= MESSAGE_COMPLETION_LIMIT)
  {
    if(!latest_message_complete)
    {
      #ifdef DEBUG_ON
      Serial.println(String("Timed out on  message that began: ") + latest_cellular_message_to_send);
      #endif
      latest_message_complete = true;
      latest_cellular_message_to_send = "";
    }
    message_completion_timestamp = current_timestamp;
  }

  bool polled_this_round = false;
  if(current_timestamp - regular_polling_timestamp >  REGULAR_POLLING_INTERVAL)
  {
    regular_polling_timestamp = current_timestamp;
    #ifdef DEBUG_ON
    Serial.println("SLOW POLLING");
    #endif
    poll_esp8266();
    polled_this_round = true;
  }

  if(current_timestamp - assemble_polling_timestamp > ASSEMBLE_POLLING_INTERVAL)
  {
    assemble_polling_timestamp = current_timestamp;
    if(!latest_message_complete && !polled_this_round)
    {
      #ifdef DEBUG_ON
      Serial.println("FAST POLLING");
      #endif
      poll_esp8266();
    }
  }

/*

  #ifdef SIMULATE_SENDING
  if(current_timestamp - simulated_sending_timestamp > SIMULATE_SENDING)
  {
    simulated_sending_timestamp = current_timestamp;
    if (latest_cellular_message_to_send.length() > 0 && latest_message_complete == true)
    {
      Serial.println(String("SIM Tranmission:\"") + latest_cellular_message_to_send + "\""); 
      latest_cellular_message_to_send = "";
    }
  }
  #endif
*/


}
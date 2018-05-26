/*

  FloatHub Arduino Code
  (c) 2011-2018 Modiot Labs

  Particle Electron Code to send messages from ESP8266 out via cellular link
  Note that contrary to what you might guess, the Electron is the SPI _MASTER_


*/

//
// This code should run even when cellular can't connect
//

SYSTEM_MODE(MANUAL);
//SYSTEM_THREAD(ENABLED);

//
// Global defines
//

//#define DEBUG_ON
#define MAX_LATEST_MESSAGE_SIZE      512
#define HOUSEKEEPING_INTERVAL        1000      // Check our network status and do other core tasks once every second
#define CONNECTION_ATTEMPT_INTERVAL  10000     // If we don't have a connection, how long do we wait between attempts
#define MESSAGE_COMPLETION_LIMIT     60000     // If we can't assemble a whole message within 1 minute, assume it is garbled and toss it
#define REGULAR_POLLING_INTERVAL     500       // When otherwise idle, poll the ESP8266 for a message every 1/2 of a second
#define ASSEMBLE_POLLING_INTERVAL    100       // Poll the ESP8266 every 1/10 second when we are in the process of assembling
//#define SIMULATE_SENDING           2000      // If defined, simulate sending the message out after defined period (debug cod
#define PARTICLE_TOPIC               "fhub"    // The "topic" that goes with the message to Hologram's Systems
#define WATCHDOG_TIME_LIMIT       3600000UL    // If we do not manage to connect at all after 1 hour, reboot cellular


// Default timeout in milliseconds for the calls straight to the cellular modem
static const system_tick_t DEFAULT_TIMEOUT = 10000;

//
//  Global vars
//

String latest_cellular_message_to_send;
bool latest_message_complete;
unsigned long housekeeping_timestamp = 0;
unsigned long connection_attempt_timestamp = 0; 
unsigned long message_completion_timestamp = 0;
unsigned long regular_polling_timestamp = 0;
unsigned long assemble_polling_timestamp = 0;
unsigned long watchdog_timestamp = 0;
bool current_cellular_status = false;
#ifdef SIMULATE_SENDING
unsigned long simulated_sending_timestamp = 0;
#endif

String network_operator;
String sim_card_number;

unsigned long post_sequence_id; 
unsigned long boot_counter;

//
//  Need this to turn off default charging behaviour
//

PMIC _pmic; 

void try_to_send()
{
  //
  //    This blocks if we are not connected, so we try and send only if
  // connected
  //

  #ifdef SIMULATE_SENDING

  if(latest_cellular_message_to_send.length() > 0 &&
     latest_message_complete)
  {
    #ifdef DEBUG_ON
    Serial.println(String("FAKED Tranmission:\"") + latest_cellular_message_to_send + "\"");
    #endif
    latest_cellular_message_to_send = "";
  }
  
  #else  

  if(latest_cellular_message_to_send.length() > 0 &&
     latest_message_complete &&
     Cellular.ready())
  {
    post_sequence_id += 1;
    EEPROM.put(10, post_sequence_id);
    String topic = PARTICLE_TOPIC;
    String fhubid = latest_cellular_message_to_send.substring(latest_cellular_message_to_send.indexOf(':', 0) + 1, 
                                                               latest_cellular_message_to_send.indexOf(':', 8));
    if(fhubid.length() == 8)
    {
      topic += "/";
      topic += fhubid;
      topic += "/";
      topic += latest_cellular_message_to_send.substring(1, latest_cellular_message_to_send.indexOf(':', 1));
      topic += "/";
      topic += latest_cellular_message_to_send.substring(latest_cellular_message_to_send.indexOf(':', 8) + 1, 
                                                               latest_cellular_message_to_send.indexOf('$', 8));
    }
    else
    {
        topic += "/confused/FHU/0";
    }
    topic += "/";
    topic += String(post_sequence_id);
    
    bool all_good = true;    
    int numb_parts = (latest_cellular_message_to_send.length() / 254 ) + 1;
    for (int part_number = 1; part_number <= numb_parts; part_number ++)
    {
      String part_topic = topic + "/";
      part_topic += String(part_number);
      part_topic += "/";
      part_topic += String(numb_parts);
      String part_data = latest_cellular_message_to_send.substring((part_number - 1) * 254, ((part_number - 1) * 254) + 254);
      if (!Particle.publish(part_topic, latest_cellular_message_to_send, 60, PRIVATE))
      {
        all_good = false;
      }
    }
    
    if(all_good)
    {
      #ifdef DEBUG_ON
      Serial.print("Message succesfully sent over cellular: \"");
      Serial.print(topic);
      Serial.print("\" --> \"");
      Serial.print(latest_cellular_message_to_send);
      Serial.println("\"");
      #endif
      latest_cellular_message_to_send = "";
    }
    else
    {
      #ifdef DEBUG_ON
      Serial.print("Not sure how I managed to fail to send a message");
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

  SPI.beginTransaction(__SPISettings(4*MHZ, MSBFIRST, SPI_MODE0));
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
      #ifdef DEBUG_ON
      Serial.print("Appended/Started message, now at: \"");
      Serial.print(latest_cellular_message_to_send);
      Serial.println("\"");
      #endif
    }
    else
    {
      //
      // We seem to be over our max message size ... no good options really
      //
      latest_cellular_message_to_send = return_message;
      #ifdef DEBUG_ON
      Serial.print("BAD BAD BAD: dropped part of message, now just at: \"");
      Serial.print(latest_cellular_message_to_send);
      Serial.println("\"");
      #endif
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
        System.reset();
      }
      else
      {
        try_to_send();
      }
    }
  }  
}

void poll_esp8266()
{
  String message_to_send;
  if(current_cellular_status == true)
  {
    if(latest_cellular_message_to_send.length() == 0)
    {
      message_to_send = "LU "; // link up, send me a message
    }
    else if(latest_message_complete)
    {
      message_to_send = "LQ ";  // link up, message in queue
    }
    else
    {
      message_to_send = "LR ";  // link up, send me _rest_ of message
    }
  }
  else
  {
     message_to_send = "NL ";   // No link
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



int callbackICCID(int type, const char* buf, int len, char* iccid)
{
  if ((type == TYPE_PLUS) && iccid)
  {
    if (sscanf(buf, "\r\n+CCID: %[^\r]\r\n", iccid) == 1)
      /*nothing*/;
  }
  return WAIT;
}


int callbackOperator(int type, const char* buf, int len, char* operator_name)
{
  if ((type == TYPE_PLUS) && operator_name)
  {
    char intermediate[64];  
    if (sscanf(buf, "\r\n+UDOPN: %*d,\"%[^\"]\"\r\n", intermediate) == 1)
    {
      uint8_t new_value;
      int op_name_position = 0;
      for(int i = 0; i < strlen(intermediate) && i < 64 ; i = i + 4)
      {
        if (intermediate[i+2] <= '9')
        {
          new_value = (intermediate[i+2] - '0' ) * 16;
        }
        else
        {
          new_value = (intermediate[i+2] - 'a' + 10) * 16;
        }

        if (intermediate[i+3] <= '9')
        {
          new_value += intermediate[i+3] - '0';
        }
        else
        {
          new_value += intermediate[i+3] - 'a' + 10;
        }
        if(new_value > 31)
        {
          operator_name[op_name_position++] = (char) new_value;
        }
      }
      operator_name[op_name_position] = '\0';
    }
  }
  return WAIT;
}


void do_housekeeping()
{
  if(Particle.connected())
  {
    watchdog_timestamp = millis();
    #ifdef DEBUG_ON
    Serial.println("IN Particle.process()");
    #endif
    Particle.process();
    connection_attempt_timestamp = millis();
    if(current_cellular_status == false)
    {
      current_cellular_status = true;
      char operator_name[32] = "";
      if ((RESP_OK == Cellular.command(callbackOperator, operator_name, DEFAULT_TIMEOUT, "AT+UDOPN=8\r\n"))
      && (strcmp(operator_name,"") != 0))
      {
        network_operator = String(operator_name);
      }
      else
      {
        network_operator = "Particle";
      }
    }
  }
  else
  {
    current_cellular_status = false;
    network_operator = "Particle";
    if (millis() - connection_attempt_timestamp > CONNECTION_ATTEMPT_INTERVAL)
    {
      #ifdef DEBUG_ON
      Serial.println("ATTEMPTING Particle.connect()");
      #endif
      Particle.connect();
      connection_attempt_timestamp = millis();
    }
  }
    
  #ifdef DEBUG_ON
  Serial.print("Network operator is \"");
  Serial.print(network_operator);
  Serial.println("\"");
  #endif   
  
  //
  //  Get current SIM Card number
  //

  char iccid[32] = "";
  if ((RESP_OK == Cellular.command(callbackICCID, iccid, DEFAULT_TIMEOUT, "AT+CCID\r\n"))
    && (strcmp(iccid,"") != 0))
  {
    sim_card_number = String(iccid);
    sim_card_number.replace(' ','_');
  }
  else
  {
    sim_card_number = "Unknown";
  }
  
  #ifdef DEBUG_ON
  Serial.print("SIM number is \"");
  Serial.print(sim_card_number);
  Serial.println("\"");
  #endif   
 
  //
  // Regularly try to send
  //

  try_to_send();
  
  //
  //
  //
  #ifdef DEBUG_ON
  unsigned long uptime = millis() / 1000.0;
  if (uptime > 86400)
  {
    Serial.println(String(F(" Uptime: ")) + (uptime / 86400.0) + F(" days"));
  }
  else if (uptime > 3600)
  {
    Serial.println(String(F(" Uptime: ")) + (uptime / 3600.0) + F(" hours"));
  }
  else if (uptime > 60)
  {
    Serial.println(String(F(" Uptime: ")) + (uptime / 60.0) + F(" mins"));
  }
  else
  {
    Serial.println(String(F(" Uptime: ")) + uptime + F(" secs"));
  }
  #endif
  
  //
  //  Check watchdog
  //
  
  if(millis() - watchdog_timestamp > WATCHDOG_TIME_LIMIT)
  {
    Serial.println("No connection after hours, rebooting cellular");
    delay(2000);
    System.reset();
  }
  
}


void init_eeprom_memory()
{
  boot_counter = 0;
  post_sequence_id = 0;
  EEPROM.write(0, 42);
  EEPROM.write(1, 42);
  EEPROM.write(2, 42);
  EEPROM.write(3, 42);
  EEPROM.write(4, 42);
  EEPROM.write(5, 42);
  EEPROM.put(6, boot_counter);
  EEPROM.put(10, post_sequence_id);
}

void read_eeprom_memory()
{
  EEPROM.get(6, boot_counter);
  boot_counter += 1;
  EEPROM.put(6, boot_counter);
  EEPROM.get(10, post_sequence_id);
}

void setup() {
  // pinMode(L07, OUTPUT);
  // digitalWrite(L07, LOW);
  Serial.begin(115200);
  delay(1000);
  SPI.begin(SPI_MODE_MASTER);
  delay(5000); // Make sure the ESP8266 has time to boot first

  #ifdef DEBUG_ON
  Serial.println("==== Startin' up after ESP8266 headstart delay ====");
  #endif

  // digitalWrite(L07, HIGH);

  latest_cellular_message_to_send.reserve(MAX_LATEST_MESSAGE_SIZE);
  latest_cellular_message_to_send = "";
  latest_message_complete = true;

  current_cellular_status = false;
  network_operator = "Unknown";
  sim_card_number = "Unknown";
  
  //
  // Tell power management unit to stop trying to charge the LIPO battery, because there ain't one.
  //
  
  

  _pmic.begin();
  _pmic.disableWatchdog();
  _pmic.disableDPDM();
  _pmic.disableCharging();
  _pmic.disableBATFET();
  byte DATA = _pmic.readChargeTermRegister(); 
  DATA &= 0b11000110;
  Wire3.beginTransmission(PMIC_ADDRESS);
  Wire3.write(CHARGE_TIMER_CONTROL_REGISTER);
  Wire3.write(DATA);
  Wire3.endTransmission(true);


  //
  //  Handle EEPROM logic for persistant settings (if the first 6 bytes of
  //  EEPROM memory are not all set to 42, then this is a completely
  //  unitialized device)
  //

  uint8_t a = 0;
  for(uint8_t i = 0; i < 6; i++)
  {
    a = EEPROM.read(i);
    if(a != 42)
    {
      Serial.println("====== Virgin Module ======");
      init_eeprom_memory();
      break;
    }
  }

  read_eeprom_memory();

  #ifdef DEBUG_ON
  Serial.print("====== Boot Count is ");
  Serial.print(boot_counter);
  Serial.println(" ======");
  #endif

  //
  // Zero out intial timestamps
  //
  
  housekeeping_timestamp = 0; 
  connection_attempt_timestamp = 0; 
  message_completion_timestamp = 0;
  regular_polling_timestamp = 0;
  assemble_polling_timestamp = 0;
  watchdog_timestamp = 0;
}

void loop() {

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
      Serial.println(String("Timed out on message that began: ") + latest_cellular_message_to_send);
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
}
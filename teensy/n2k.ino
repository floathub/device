#ifdef N2K_CODE_ON

#include "n2k.h"
//#include "globals.h"
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
//#include <NMEA0183Msg.h>
//#include <math.h>


#define N2K_RAPID_VALID_DURATION	 2000UL		// N2K Rapid data valid for 2 seconds
#define N2K_NORMAL_VALID_DURATION	10000UL		// N2K Normal data valid for 10 seconds



void n2k_setup()
{

  n2k_latitude  = N2kDoubleNA;
  n2k_longitude = N2kDoubleNA;
  n2k_gps_valid = false;
  n2k_location_timestamp = 0;

  n2k_cog = N2kDoubleNA;
  n2k_sog = N2kDoubleNA;
  n2k_cogsog_timestamp = 0;
  
  n2k_variation = N2kDoubleNA;
  n2k_variation_timestamp = 0;

  n2k_deviation = N2kDoubleNA;
  n2k_deviation_timestamp = 0;

  n2k_heading_true = N2kDoubleNA;
  n2k_heading_true_timestamp = 0;
  n2k_heading_magnetic = N2kDoubleNA;
  n2k_heading_magnetic_timestamp = 0;

  n2k_depth  = N2kDoubleNA;
  n2k_offset = N2kDoubleNA;
  n2k_depth_timestamp = 0;

  n2k_stw = N2kDoubleNA;
  n2k_stw_timestamp = 0;

  n2k_siv        = N2kUInt8NA;
  n2k_hdp        = N2kDoubleNA;
  n2k_altitude   = N2kDoubleNA;
  n2k_fix_geosep = N2kDoubleNA;
  //n2k_fix_age    = N2kDoubleNA;
  //n2k_fix_refid  = N2kUInt16NA;
  n2k_fix_timestamp = 0;
  n2k_fix_days_1970 = 0;
  n2k_fix_seconds = 0;
  n2k_fix_valid = false;
  
  n2k_water_temperature = N2kDoubleNA;
  n2k_water_temperature_timestamp = 0;

  n2k_wind_true_speed = N2kDoubleNA;
  n2k_wind_true_direction = N2kDoubleNA;
  n2k_wind_true_timestamp = 0;
  
  n2k_wind_apparent_speed = N2kDoubleNA;
  n2k_wind_apparent_direction = N2kDoubleNA;
  n2k_wind_apparent_timestamp = 0;
  
  for(i=0; i< MAX_N2K_BATTERIES; i++)
  {
    n2k_battery_map[i] = 254;
    n2k_battery_voltage[i] = N2kDoubleNA;
    n2k_battery_timestamp[i] = 0;
  }


  n2k_output_cycle = 0;
  
  // Setup NMEA2000 system

  //#ifndef ARDUINO
  //setvbuf (stdout, NULL, _IONBF, 0); // No buffering on stdout, just send chars as they come.
  //#endif

  //#ifdef ARDUINO
  //#ifdef N2kForward_Stream
  //N2kForward_Stream.begin(N2kForward_Stream_Speed);
  //#endif
  //NMEA0183_Out_Stream.begin(NMEA0183_Out_Stream_Speed);
  //delay(1000); // Give some time for serial to initialize
  //#else
  //#endif

  //#ifdef N2kForward_Stream
  //NMEA2000.SetForwardStream(&N2kForward_Stream);
  //#endif

  //char SnoStr[33];
  //uint32_t SerialNumber=GetSerialNumber();
  //snprintf(SnoStr,32,"%lu",(long unsigned int)SerialNumber);

  NMEA2000.SetProductInformation("FloatHub N2K", // Manufacturer's Model serial code
                                 120, // Manufacturer's product code
                                 "N2K",  // Manufacturer's Model ID
                                 "2.3.1",  // Manufacturer's Software version code
                                 "P9.N2" // Manufacturer's Model version
                                 );
  // Set device information
  
  
  NMEA2000.SetDeviceInformation(7654321, // Unique number. Use e.g. Serial number.
                                130, // Device function=PC Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                2319 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );
  

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,25);
  NMEA2000.EnableForward(false);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  //NMEA2000.AttachMsgHandler(&N2kDataToNMEA0183);
  NMEA2000.SetMsgHandler(HandleNMEA2000Messages);

  NMEA2000.Open();

  // Setup NMEA0183 ports and handlers
  //NMEA0183_Out.SetMessageStream(&NMEA0183_Out_Stream);
  //NMEA0183_Out.Open();
  
}



void HandleNMEA2000Messages(const tN2kMsg &N2kMsg)
{
  
  switch (N2kMsg.PGN) {
    case 127250UL: HandleHeading(N2kMsg); break;
    case 127258UL: HandleVariation(N2kMsg); break;
    case 128259UL: HandleBoatSpeed(N2kMsg); break;
    case 128267UL: HandleDepth(N2kMsg); break;
    case 129025UL: HandlePosition(N2kMsg); break;
    case 129026UL: HandleCOGSOG(N2kMsg); break;
    case 129029UL: HandleGNSS(N2kMsg); break;
    case 130310UL: HandleEnvironment(N2kMsg); break;
    case 130306UL: HandleWind(N2kMsg); break;
    case 127508UL: HandleBattery(N2kMsg); break;
    case 129038UL: 
    case 129039UL: 
    case 129794UL: 
    case 129809UL: 
    case 129810UL: HandleAIS(N2kMsg); break;
  }
}


void HandleHeading(const tN2kMsg &N2kMsg)
{
  Serial.println("Actually here in Handle Heading ...");
  unsigned char SID;
  tN2kHeadingReference HeadingReference;
  double a_heading;
  
  if(ParseN2kPGN127250(N2kMsg,SID, a_heading, n2k_deviation, n2k_variation, HeadingReference))
  {
    if( HeadingReference==N2khr_true )
    {
      if(!N2kIsNA(a_heading))
      {
        n2k_heading_true = a_heading;
        n2k_heading_true_timestamp = millis();
        if (!N2kIsNA(n2k_variation))
        {
          n2k_heading_magnetic = a_heading - n2k_variation;
          n2k_heading_magnetic_timestamp = millis();
          n2k_variation_timestamp = millis();
          if (!N2kIsNA(n2k_deviation))
          {
            n2k_heading_magnetic = n2k_heading_magnetic - n2k_deviation;
            n2k_deviation_timestamp = millis();
          }
        }
      }
    }    
    else if( HeadingReference==N2khr_magnetic )
    {
      if(!N2kIsNA(a_heading))
      {
        n2k_heading_magnetic = a_heading;
        n2k_heading_magnetic_timestamp = millis();
        if (!N2kIsNA(n2k_variation))
        {
          n2k_heading_true = a_heading + n2k_variation;
          n2k_heading_true_timestamp = millis();
          n2k_variation_timestamp = millis();
          if (!N2kIsNA(n2k_deviation))
          {
            n2k_heading_true = n2k_heading_true + n2k_deviation;
            n2k_deviation_timestamp = millis();
          }
        }
      }
    }
  }

/*
  if(ParseN2kPGN127250(N2kMsg, n2k_latitude, n2k_longitude)) 
  {
    n2k_location_timestamp=millis();
  }
*/
}



void HandleVariation(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  tN2kMagneticVariation Source;
  uint16_t variation_days;

  if(ParseN2kMagneticVariation(N2kMsg,SID,Source, variation_days, n2k_variation))
  {
    n2k_variation_timestamp = millis();
  }
}


void HandleBoatSpeed(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double sog;
  tN2kSpeedWaterReferenceType SWRT;

  if(ParseN2kPGN128259(N2kMsg, SID, n2k_stw, sog, SWRT))
  {
    n2k_stw_timestamp = millis();
  }
}


void HandleDepth(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double range;

  if(ParseN2kPGN128267(N2kMsg,SID,n2k_depth, n2k_offset, range))
  {
    n2k_depth_timestamp = millis();
  }
}


void HandlePosition(const tN2kMsg &N2kMsg)
{
  if(ParseN2kPGN129025(N2kMsg, n2k_latitude, n2k_longitude)) 
  {
    n2k_location_timestamp=millis();
  }
}


void HandleCOGSOG(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  tN2kHeadingReference HeadingReference;

  if ( ParseN2kCOGSOGRapid(N2kMsg,SID,HeadingReference,n2k_cog,n2k_sog) )
  {
    n2k_cogsog_timestamp = millis();
    if ( HeadingReference==N2khr_magnetic && !N2kIsNA(n2k_cog) && !N2kIsNA(n2k_variation))
    {
      n2k_cog -= n2k_variation;
    }
    else if (HeadingReference==N2khr_magnetic && N2kIsNA(n2k_variation))
    {
      // We have magnetic COG but no variation, so we *cannot* calculate COG
      n2k_cog = N2kDoubleNA;
    }
  }
}

void HandleGNSS(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  
  tN2kGNSStype GNSStype;
  tN2kGNSSmethod GNSSmethod;
  double PDOP;
  double GeoidalSeparation;
  unsigned char nReferenceStations;
  tN2kGNSStype ReferenceStationType;
  uint16_t ReferenceStationID;
  double AgeOfCorrection;
  double fix_age    = N2kDoubleNA;
  

  if ( ParseN2kGNSS(N2kMsg, SID, n2k_fix_days_1970, n2k_fix_seconds, n2k_latitude, n2k_longitude, n2k_altitude,
                    GNSStype, GNSSmethod, n2k_siv, n2k_hdp, PDOP, n2k_fix_geosep,
                    nReferenceStations, ReferenceStationType, ReferenceStationID, fix_age) ) 
  {
    unsigned long unix_time = ( (unsigned long) n2k_fix_days_1970  * 3600UL * 24UL) + n2k_fix_seconds;

    // If the UNIX time is some time after 2020, and the GNSSmethod is good, this is all valid 
    
    if(unix_time > 1609459199UL && GNSSmethod > 0 && GNSSmethod < 6)
    {
      n2k_fix_valid = true;
      n2k_fix_timestamp = millis();
      setTime(unix_time);
    }
    else
    {
      n2k_hdp        = N2kDoubleNA;
      n2k_altitude   = N2kDoubleNA;
      n2k_fix_geosep = N2kDoubleNA;
      //n2k_fix_age    = N2kDoubleNA;
      //n2k_fix_refid  = N2kUInt16NA;
      n2k_fix_timestamp = 0;
      n2k_fix_valid = false;
      
    }
  }

}

void HandleEnvironment(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double air_temp, pressure;

  if ( ParseN2kPGN130310(N2kMsg,SID,n2k_water_temperature, air_temp, pressure) )
  {
    //Serial.println(n2k_water_temperature);
    n2k_water_temperature_timestamp=millis();
  }
}

void HandleWind(const tN2kMsg &N2kMsg)
{

  unsigned char SID;
  double speed, direction;
  tN2kWindReference reference;

  if ( ParseN2kPGN130306(N2kMsg,SID, speed, direction, reference) )
  {
    if (reference == N2kWind_True_North ||
        reference == N2kWind_Magnetic   ||
        reference == N2kWind_True_boat  ||
        reference == N2kWind_True_water )
    {
      n2k_wind_true_speed = speed;
      n2k_wind_true_direction = direction;
      n2k_wind_true_timestamp = millis();
    }
    else if (reference == N2kWind_Apparent )
    {
      n2k_wind_apparent_speed = speed;
      n2k_wind_apparent_direction = direction;
      n2k_wind_apparent_timestamp = millis();
    }
  }
}

unsigned char get_battery(unsigned char instance)
{
  unsigned char lowest_available = MAX_N2K_BATTERIES;
  
  for(i=0; i < MAX_N2K_BATTERIES; i++)
  {
    if(n2k_battery_map[i] == instance)
    {
      //Serial.print("COWABUNGA: Existing Map ");
      //Serial.print(instance);
      //Serial.print(" to position ");
      //Serial.println(i);
      return i;
    }
    else if (n2k_battery_map[i] == 254 && lowest_available > i)
    {
      lowest_available = i;
    }
  }
  
  if (lowest_available < MAX_N2K_BATTERIES)
  {
    //Serial.print("COWABUNGA: Mapped ");
    //Serial.print(instance);
    //Serial.print(" to position ");
    //Serial.println(lowest_available);
    n2k_battery_map[lowest_available] = instance; 
    return lowest_available;
  }
  
  return MAX_N2K_BATTERIES;
}

void HandleBattery(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  unsigned char instance;
  unsigned char mapped_instance;
  double voltage, current, temperature;

  if ( ParseN2kPGN127508(N2kMsg, instance, voltage, current, temperature, SID) )
  {
    mapped_instance = get_battery(instance);
    if(mapped_instance < MAX_N2K_BATTERIES)
    {
      n2k_battery_voltage[mapped_instance] = voltage;
      n2k_battery_timestamp[mapped_instance] = millis();
    }
  }
}

void HandleAIS(const tN2kMsg &N2kMsg)
{

/*  
  uint8_t MessageID; 
  tN2kAISRepeat Repeat; 
  uint32_t UserID;
  double Latitude;
  double Longitude;
  bool Accuracy;
  bool RAIM;
  uint8_t Seconds;
  double COG;
  double SOG;
  double Heading;
  double ROT;
  tN2kAISNavStatus NavStatus;

  if ( ParseN2kPGN129038(N2kMsg, MessageID, Repeat, UserID, Latitude, Longitude, Accuracy, RAIM, Seconds, COG, SOG, Heading, ROT, NavStatus))
  {
    //Serial.print("COWABUNGA: AIS target going at ");
    //Serial.println(SOG);
    Serial.print("COWABUNGA: AIS byte length of ");
    Serial.println(N2kMsg.DataLen);
  }
*/  

  //char encoded[300];
  //int length = base64_encode(encoded, N2kMsg.Data, N2kMsg.DataLen);
  //encoded[length] = '\0';
  //Serial.print("COWABUNGA: B64 encoded of PGN ");
  //Serial.println(N2kMsg.PGN);
  //Serial.print("length is ");
  //Serial.println(length);
  //Serial.print("COWABUNGA B64: ");
  //Serial.println(encoded);
  
}

void push_out_message(const tNMEA0183Msg &NMEA0183Msg)
{
  //char buffer[MAX_NMEA0183_MSG_BUF_LEN];
  char buffer[MAX_NMEA0183_MSG_BUF_LEN]={NMEA0183Msg.GetPrefix(),0};
  strlcat(buffer, NMEA0183Msg.Sender(), MAX_NMEA0183_MSG_BUF_LEN);
  strlcat(buffer, NMEA0183Msg.MessageCode(), MAX_NMEA0183_MSG_BUF_LEN);
  for(i=0; i<NMEA0183Msg.FieldCount(); i++)
  {
    strlcat(buffer,",", MAX_NMEA0183_MSG_BUF_LEN);
    strlcat(buffer, NMEA0183Msg.Field(i), MAX_NMEA0183_MSG_BUF_LEN);
  }
  char buf[4] = "";
  sprintf(buf,"*%02X",NMEA0183Msg.GetCheckSum());
  strlcat(buffer, buf, MAX_NMEA0183_MSG_BUF_LEN);

  //
  // OK, buffer has our message. Send to ESP (WiFi) if we can
  //

  if(esp8266IsReady())
  {
    Serial7.print(F("E="));
    Serial7.println(buffer);
  }

  //
  // As long as we are in NMEA mode (not console mode), print to the console
  //
  if(console_mode == false)
  {
    Serial.println(buffer);
  }

  //
  // Send to NMEA out screw terminals
  //

  Serial8.println(buffer);

  //
  // If it is GGA or RMC, parse it for Fix, time, etc.
  //
  
  if(strncmp(NMEA0183Msg.MessageCode(), "RMC", 3) == 0 && n2k_gps_valid)
  {
    gps_read_buffer = buffer;
    parse_gps_buffer_as_rmc();
    gps_read_buffer = "";
  }
  else if(strncmp(NMEA0183Msg.MessageCode(), "GGA", 3) == 0  && n2k_gps_valid)
  {
    gps_read_buffer = buffer;
    parse_gps_buffer_as_gga();
    gps_read_buffer = "";
  }
}


void n2k_output()
{

  if (n2k_location_timestamp + N2K_RAPID_VALID_DURATION < millis())
  {
    n2k_gps_valid = false;
    n2k_latitude  = N2kDoubleNA;
    n2k_longitude = N2kDoubleNA;
  }
  else
  {
    n2k_gps_valid = true;
  }
  
  if (n2k_cogsog_timestamp + N2K_RAPID_VALID_DURATION < millis())
  {
    n2k_cog = N2kDoubleNA;
    n2k_sog = N2kDoubleNA;
  }
  
  if (n2k_variation_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_variation = N2kDoubleNA;
  }
  if (n2k_fix_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_siv        = N2kUInt8NA;
    n2k_hdp        = N2kDoubleNA;
    n2k_altitude   = N2kDoubleNA;
    n2k_fix_geosep = N2kDoubleNA;
    //n2k_fix_age    = N2kDoubleNA;
    //n2k_fix_refid  = N2kUInt16NA;
    n2k_fix_valid  = false;
  }

  if (n2k_heading_magnetic_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_heading_magnetic = N2kDoubleNA;
  }
  if (n2k_heading_true_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_heading_true = N2kDoubleNA;
  }
  if (n2k_variation_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_variation = N2kDoubleNA;
  }
  if (n2k_deviation_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_deviation = N2kDoubleNA;
  }
  if (n2k_depth_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_depth = N2kDoubleNA;
    n2k_offset = N2kDoubleNA;
  }
  if (n2k_stw_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_stw = N2kDoubleNA;
  }
  if (n2k_water_temperature_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_water_temperature = N2kDoubleNA;
  }
  if (n2k_wind_true_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_wind_true_speed = N2kDoubleNA;
    n2k_wind_true_direction = N2kDoubleNA;
  }
  if (n2k_wind_apparent_timestamp + N2K_NORMAL_VALID_DURATION < millis())
  {
    n2k_wind_apparent_speed = N2kDoubleNA;
    n2k_wind_apparent_direction = N2kDoubleNA;
  }

  for(i=0; i < MAX_N2K_BATTERIES; i++)
  {
    if(n2k_battery_timestamp[i] + N2K_NORMAL_VALID_DURATION < millis())
    {
      n2k_battery_map[i] = 254;
      n2k_battery_voltage[i] = N2kDoubleNA;
    }
  }
  //
  // Output whatever we can as NMEA0183 based on the N2K data at hand
  //
  
  //Serial.print("COWABUNGA outputting with n2k_fix_valid as ");
  //Serial.println(n2k_fix_valid);
  
  tNMEA0183Msg NMEA0183Msg;
  if(n2k_output_cycle == 0 && n2k_fix_valid)
  {
    if ( NMEA0183SetGGA(NMEA0183Msg, n2k_fix_seconds, n2k_latitude, n2k_longitude,
                        1, n2k_siv, n2k_hdp, n2k_altitude - n2k_fix_geosep, 
                        n2k_fix_geosep, NMEA0183DoubleNA, NMEA0183UInt32NA) )
    {
      //Serial.println("COWABUNGA A");
      push_out_message(NMEA0183Msg);
      //Serial.println("COWABUNGA B");
    }
  }
  else if(n2k_output_cycle == 1 && n2k_fix_valid)
  {
    if ( NMEA0183SetRMC(NMEA0183Msg, n2k_fix_seconds, n2k_latitude, n2k_longitude,
                        n2k_cog, n2k_sog, n2k_fix_days_1970, n2k_variation) )
    {
      //Serial.println("COWABUNGA C");
      push_out_message(NMEA0183Msg);
      //Serial.println("COWABUNGA D");
    }
  }
  else if(n2k_output_cycle == 2)
  {
    if ( !N2kIsNA(n2k_heading_magnetic) && 
         !N2kIsNA(n2k_variation)        && 
         NMEA0183SetHDG(NMEA0183Msg, n2k_heading_magnetic, n2k_deviation, n2k_variation) )
    {
      push_out_message(NMEA0183Msg);
    }
    else
    {
      if ( !N2kIsNA(n2k_heading_true) &&  
           NMEA0183SetHDT(NMEA0183Msg, n2k_heading_true))
      {
        push_out_message(NMEA0183Msg);
      }
      if ( !N2kIsNA(n2k_heading_magnetic) && 
           NMEA0183SetHDM(NMEA0183Msg, n2k_heading_magnetic))
      {
        push_out_message(NMEA0183Msg);
      }
    } 
  }
  else if(n2k_output_cycle == 3)
  {
    if ( !N2kIsNA(n2k_depth) && NMEA0183SetDPT(NMEA0183Msg, n2k_depth, n2k_offset) )
    {
      push_out_message(NMEA0183Msg);
    }
    if ( !N2kIsNA(n2k_depth) && NMEA0183SetDBx(NMEA0183Msg, n2k_depth, n2k_offset) )
    {
      push_out_message(NMEA0183Msg);
    }
  }
  else if(n2k_output_cycle == 4)
  {
    if ( !N2kIsNA(n2k_heading_true)     && 
         !N2kIsNA(n2k_heading_magnetic) && 
         NMEA0183SetVHW(NMEA0183Msg, n2k_heading_true, n2k_heading_magnetic, n2k_stw ) )
    {
      push_out_message(NMEA0183Msg);
    }
  }
  else if(n2k_output_cycle == 5)
  {
    if ( !N2kIsNA(n2k_wind_true_speed) && NMEA0183SetMWV(NMEA0183Msg, RadToDeg(n2k_wind_true_direction), NMEA0183Wind_True, n2k_wind_true_speed) )
    {
      push_out_message(NMEA0183Msg);
    }
    if ( !N2kIsNA(n2k_wind_apparent_speed) && NMEA0183SetMWV(NMEA0183Msg, RadToDeg(n2k_wind_apparent_direction), NMEA0183Wind_Apparent, n2k_wind_apparent_speed) )
    {
      push_out_message(NMEA0183Msg);
    }
  }
  n2k_output_cycle += 1;
  if(n2k_output_cycle >= 6)
  {
    n2k_output_cycle = 0;
  }
  
}


void append_n2k_gps_data_to_string(String &the_string)
{
    if( !N2kIsNA(n2k_latitude) )
    {
      the_string += F(",L:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(n2k_latitude,4,7,temp_string);
      the_string += temp_string;
    }
    
    
    if( !N2kIsNA(n2k_longitude) )
    {
      the_string += F(",O:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(n2k_longitude,4,7,temp_string);
      the_string += temp_string;
    }
    
    if ( !N2kIsNA(n2k_sog) )
    {
      the_string += F(",S:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(msToKnots(n2k_sog),4,2,temp_string);
      the_string += temp_string;
    }

    if ( !N2kIsNA(n2k_cog) )
    {
      the_string += F(",B:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(RadToDeg(n2k_cog),4,2,temp_string);
      the_string += temp_string;
    }

    if ( !N2kIsNA(n2k_altitude) )
    {
      the_string += F(",A:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(n2k_altitude - n2k_fix_geosep,3,2,temp_string);
      the_string += temp_string;
    }

    if ( !N2kIsNA(n2k_hdp) )
    {
      the_string += F(",H:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(n2k_hdp,3,2,temp_string);
      the_string += temp_string;
    }

    if ( !N2kIsNA(n2k_siv) )
    {
      the_string += F(",N:");
      the_string += n2k_siv;
    }
    else
    {
      the_string += F(",N:0");
    }

    if ( !N2kIsNA(n2k_heading_magnetic) )
    {
      the_string += F(",M:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(RadToDeg(n2k_heading_magnetic),4,2,temp_string);
      the_string += temp_string;
    }
    if ( !N2kIsNA(n2k_heading_true) )
    {
      the_string += F(",G:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(RadToDeg(n2k_heading_true),4,2,temp_string);
      the_string += temp_string;
    }
    if ( !N2kIsNA(n2k_depth) )
    {
      the_string += F(",D:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(n2k_depth,4,2,temp_string);
      the_string += temp_string;
    }
    if ( !N2kIsNA(n2k_stw) )
    {
      the_string += F(",R:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(msToKnots(n2k_stw),4,2,temp_string);
      the_string += temp_string;
    }
    if ( !N2kIsNA(n2k_water_temperature) )
    {
      the_string += F(",Y:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(KelvinToF(n2k_water_temperature),4,2,temp_string);
      the_string += temp_string;
    }
/*
    if ( !N2kIsNA(n2k_wind_true_speed) )
    {
      the_string += F(",W:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(n2k_wind_true_speed,4,2,temp_string);
      the_string += temp_string;
    }
    if ( !N2kIsNA(n2k_wind_true_direction) )
    {
      the_string += F(",X:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(RadToDeg(n2k_wind_true_direction),4,2,temp_string);
      the_string += temp_string;
    }
    if ( !N2kIsNA(n2k_wind_apparent_speed) )
    {
      the_string += F(",J:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(n2k_wind_apparent_speed,4,2,temp_string);
      the_string += temp_string;
    }
    if ( !N2kIsNA(n2k_wind_apparent_direction) )
    {
      the_string += F(",K:");
      memset(temp_string, 0, 20 * sizeof(char));
      dtostrf(RadToDeg(n2k_wind_apparent_direction),4,2,temp_string);
      the_string += temp_string;
    }
    for(i=0; i < MAX_N2K_BATTERIES; i++)
    {
      if(n2k_battery_map[i] != 254 && !N2kIsNA(n2k_battery_voltage[i]))
      {
        the_string += F(",E");
        memset(temp_string, 0, 20 * sizeof(char));
        itoa(i,temp_string,10);
        the_string += temp_string;
        the_string += ":";
        memset(temp_string, 0, 20 * sizeof(char));
        dtostrf(n2k_battery_voltage[i],4,2,temp_string);
        the_string += temp_string;
      }
    }
*/
}



#endif

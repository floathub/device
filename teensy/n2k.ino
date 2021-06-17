#ifdef N2K_CODE_ON

#include "n2k.h"
//#include "globals.h"
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
//#include <NMEA0183Msg.h>
//#include <math.h>


#define N2K_RAPID_VALID_DURATION	2000UL		// N2K Rapid data valid for 2 second
#define N2K_NORMAL_VALID_DURATION	7000UL		// N2K Normal data valid for 7 seconds


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

  //
  // Default flags (this will get overwritten fairly quickly by the ESP)
  //

  FLAG_GPS_N2K_TO_NMEA = true;
  FLAG_GPS_NMEA_TO_N2K = true;

  FLAG_ENV_INT_TO_N2K = true;
  FLAG_ENV_INT_TO_NMEA = true;
  FLAG_ENV_N2K_TO_NMEA = true;
  FLAG_ENV_NMEA_TO_N2K = true;

  FLAG_VOL_INT_TO_N2K = true;

  FLAG_NAV_N2K_TO_NMEA = true;
  FLAG_NAV_NMEA_TO_N2K = true;

  FLAG_DEP_N2K_TO_NMEA = true;
  FLAG_DEP_NMEA_TO_N2K = true;

  FLAG_WIN_N2K_TO_NMEA = true;
  FLAG_WIN_NMEA_TO_N2K = true;

  FLAG_AIS_N2K_TO_NMEA = true;
  FLAG_AIS_NMEA_TO_N2K = true;

  
  //
  // Set some buffer sizes
  //

  NMEA2000.SetN2kCANMsgBufSize(100);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(150);

  uint32_t serial_number = random(0, 999999999);
  #if defined ARDUINO_TEENSY41
  serial_number = HW_OCOTP_MAC0 & 0xFFFFFF;
  #endif
  char serial_number_chars[33];
  snprintf(serial_number_chars,32,"%lu",(long unsigned int)serial_number);

  // Product Info
  NMEA2000.SetProductInformation(serial_number_chars, // Manufacturer's Model serial code
                                 120, // Manufacturer's product code
                                 "FloatHub N2K",  // Manufacturer's Model ID
                                 "2.4.0",  // Manufacturer's Software version code
                                 "P9.N3" // Manufacturer's Model version
                                 );
  // Set device information
  
  NMEA2000.SetDeviceInformation(serial_number, // Unique number. Use e.g. Serial number.
                                130, // Device function=PC Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                2319 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); 
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,25);
  NMEA2000.EnableForward(false);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.SetMsgHandler(HandleNMEA2000Messages);

  NMEA2000.Open();

}



void HandleNMEA2000Messages(const tN2kMsg &N2kMsg)
{
  //Serial.print("I am switching on ");
  //Serial.println(N2kMsg.PGN);
  
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

    case 129038UL: HandleAISClassAPosition(N2kMsg); break;
    case 129039UL: HandleAISClassBPosition(N2kMsg); break;
    case 129794UL: HandleAISClassAStatic(N2kMsg); break;
    case 129809UL: HandleAISClassBStaticPartA(N2kMsg); break;
    case 129810UL: HandleAISClassBStaticPartB(N2kMsg); break;

  }
}


void HandleHeading(const tN2kMsg &N2kMsg)
{

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
  unsigned char nReferenceStations;
  tN2kGNSStype ReferenceStationType;
  uint16_t ReferenceStationID;
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
      return i;
    }
    else if (n2k_battery_map[i] == 254 && lowest_available > i)
    {
      lowest_available = i;
    }
  }
  
  if (lowest_available < MAX_N2K_BATTERIES)
  {
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



void push_out_message(const tNMEA0183Msg &NMEA0183Msg, bool send_to_nmea_flag = true)
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
  // OK, buffer has our message. Send to ESP (WiFi) if we can and should
  //


  if(send_to_nmea_flag == true || strncmp(NMEA0183Msg.MessageCode(), "VDM", 3) == 0 || strncmp(NMEA0183Msg.MessageCode(), "VDO", 3) == 0)
  {
    if(esp8266IsReady())
    {
      Serial7.print(F("E="));
      Serial7.println(buffer);
    }
  }


  //
  // As long as we are in NMEA mode (not console mode), print to the console
  //

  if(console_mode == false && send_to_nmea_flag == true)
  {
    Serial.println(buffer);
  }

  //
  // Send to NMEA out screw terminals
  //

  if( send_to_nmea_flag == true)
  {
    Serial8.println(buffer);
  }

  //
  // If it is GGA, RMC, etc., parse it for Fix, time, etc.
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
  else if(
           strncmp(NMEA0183Msg.MessageCode(), "HDG", 3) == 0  ||
           strncmp(NMEA0183Msg.MessageCode(), "HDM", 3) == 0  ||
           strncmp(NMEA0183Msg.MessageCode(), "HDT", 3) == 0  ||
           strncmp(NMEA0183Msg.MessageCode(), "DPT", 3) == 0  ||
           strncmp(NMEA0183Msg.MessageCode(), "MTW", 3) == 0  ||
           strncmp(NMEA0183Msg.MessageCode(), "VHW", 3) == 0  ||
           strncmp(NMEA0183Msg.MessageCode(), "MWV", 3) == 0 
         )
  {
    a_string = nmea_read_buffer;
    nmea_read_buffer = buffer;
    parse_nmea_sentence();
    nmea_read_buffer = a_string; 
  }

}




void HandleAISClassAPosition(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID;
  double _Latitude;
  double _Longitude;
  bool _Accuracy;
  bool _RAIM;
  uint8_t _Seconds;
  double _COG;
  double _SOG;
  double _Heading;
  double _ROT;
  tN2kAISNavStatus _NavStatus;

  uint8_t _MessageType = 1;

  if ( ParseN2kPGN129038(N2kMsg, SID, _Repeat, _UserID, _Latitude, _Longitude, _Accuracy, _RAIM, _Seconds, _COG, _SOG, _Heading, _ROT, _NavStatus ) )
  {
    if ( SetAISClassABMessage1(NMEA0183AISMsg, _MessageType, _Repeat, _UserID, _Latitude, _Longitude, _Accuracy, _RAIM, _Seconds, _COG, _SOG, _Heading, _ROT, _NavStatus ) ) 
    {
      push_out_message(NMEA0183AISMsg, FLAG_AIS_N2K_TO_NMEA);
    }
  }
}

void HandleAISClassBPosition(const tN2kMsg &N2kMsg)
{

  uint8_t _MessageID = 18;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID; 
  double _Latitude;
  double _Longitude;
  bool _Accuracy;
  bool _RAIM;
  uint8_t _Seconds;
  double _COG;
  double _SOG;
  double _Heading;
  tN2kAISUnit _Unit;
  bool _Display, _DSC, _Band, _Msg22, _State;
  tN2kAISMode _Mode;

  if ( ParseN2kPGN129039(N2kMsg, _MessageID, _Repeat, _UserID, _Latitude, _Longitude, _Accuracy, _RAIM, _Seconds, _COG, _SOG, _Heading, _Unit, _Display, _DSC, _Band, _Msg22, _Mode, _State) ) 
  {
     _MessageID = 18;
    if ( SetAISClassBMessage18(NMEA0183AISMsg, _MessageID, _Repeat, _UserID, _Latitude, _Longitude, _Accuracy, _RAIM, _Seconds, _COG, _SOG, _Heading, _Unit, _Display, _DSC, _Band, _Msg22, _Mode, _State) ) 
    {
      push_out_message(NMEA0183AISMsg, FLAG_AIS_N2K_TO_NMEA);
    }
  }
}

void HandleAISClassAStatic(const tN2kMsg &N2kMsg)
{

  uint8_t _MessageID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID;
  uint32_t _IMONumber;
  char _Callsign[8];
  char _Name[21];
  uint8_t _VesselType;
  double _Length;
  double _Beam;
  double _PosRefStbd;
  double _PosRefBow;
  uint16_t _ETAdate;
  double _ETAtime;
  double _Draught;
  char _Destination[21];
  tN2kAISVersion _AISversion;
  tN2kGNSStype _GNSStype;
  tN2kAISTranceiverInfo _AISinfo;
  tN2kAISDTE _DTE;


  if ( ParseN2kPGN129794(N2kMsg, _MessageID, _Repeat, _UserID, _IMONumber, _Callsign, _Name, _VesselType, _Length, _Beam, _PosRefStbd, _PosRefBow, _ETAdate, _ETAtime, _Draught, _Destination, _AISversion, _GNSStype, _DTE, _AISinfo) ) 
  {
    if ( SetAISClassAMessage5(NMEA0183AISMsg, _MessageID, _Repeat, _UserID, _IMONumber, _Callsign, _Name, _VesselType, _Length, _Beam, _PosRefStbd, _PosRefBow, _ETAdate, _ETAtime, _Draught, _Destination, _GNSStype, _DTE ) )
    {
      push_out_message( NMEA0183AISMsg.BuildMsg5Part1(NMEA0183AISMsg), FLAG_AIS_N2K_TO_NMEA );
      push_out_message( NMEA0183AISMsg.BuildMsg5Part2(NMEA0183AISMsg), FLAG_AIS_N2K_TO_NMEA );
    }
  }
}

void HandleAISClassBStaticPartA(const tN2kMsg &N2kMsg)
{
  uint8_t _MessageID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID; 
  char _Name[21];

  if ( ParseN2kPGN129809 (N2kMsg, _MessageID, _Repeat, _UserID, _Name) )
  {
    //tNMEA0183AISMsg NMEA0183AISMsg;
    //
    //  This does not send any message, it just stores the mapping of ID to
    //  ship name in a class vector for later use by a Part B message
    //
    SetAISClassBMessage24PartA(NMEA0183AISMsg, _MessageID, _Repeat, _UserID, _Name);
  }
}

void HandleAISClassBStaticPartB(const tN2kMsg &N2kMsg)
{
  uint8_t _MessageID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID, _MothershipID;
  char _Callsign[8];
  char _Vendor[4];
  uint8_t _VesselType;
  double _Length;
  double _Beam;
  double _PosRefStbd;
  double _PosRefBow;

  if ( ParseN2kPGN129810(N2kMsg, _MessageID, _Repeat, _UserID, _VesselType, _Vendor, _Callsign, _Length, _Beam, _PosRefStbd, _PosRefBow, _MothershipID) )
  {
    if ( SetAISClassBMessage24(NMEA0183AISMsg, _MessageID, _Repeat, _UserID, _VesselType, _Vendor, _Callsign, _Length, _Beam, _PosRefStbd, _PosRefBow, _MothershipID ) )
    {
      push_out_message( NMEA0183AISMsg.BuildMsg24PartA(NMEA0183AISMsg), FLAG_AIS_N2K_TO_NMEA );
      push_out_message( NMEA0183AISMsg.BuildMsg24PartB(NMEA0183AISMsg), FLAG_AIS_N2K_TO_NMEA );
    }
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
  
  tNMEA0183Msg NMEA0183Msg;
  if(n2k_output_cycle == 0 && n2k_fix_valid)
  {
    if ( NMEA0183SetGGA(NMEA0183Msg, n2k_fix_seconds, n2k_latitude, n2k_longitude,
                        1, n2k_siv, n2k_hdp, n2k_altitude - n2k_fix_geosep, 
                        n2k_fix_geosep, NMEA0183DoubleNA, NMEA0183UInt32NA) )
    {
      push_out_message(NMEA0183Msg, FLAG_GPS_N2K_TO_NMEA);
    }
  }
  else if(n2k_output_cycle == 1 && n2k_fix_valid)
  {
    if ( NMEA0183SetRMC(NMEA0183Msg, n2k_fix_seconds, n2k_latitude, n2k_longitude,
                        n2k_cog, n2k_sog, n2k_fix_days_1970, n2k_variation) )
    {
      push_out_message(NMEA0183Msg, FLAG_GPS_N2K_TO_NMEA);
    }
  }
  else if(n2k_output_cycle == 2)
  {
    if ( !N2kIsNA(n2k_heading_magnetic) && 
         !N2kIsNA(n2k_variation)        && 
         NMEA0183SetHDG(NMEA0183Msg, n2k_heading_magnetic, n2k_deviation, n2k_variation) )
    {
      push_out_message(NMEA0183Msg, FLAG_NAV_N2K_TO_NMEA);
    }
    else
    {
      if ( !N2kIsNA(n2k_heading_true) &&  
           NMEA0183SetHDT(NMEA0183Msg, n2k_heading_true))
      {
        push_out_message(NMEA0183Msg, FLAG_NAV_N2K_TO_NMEA);
      }
      if ( !N2kIsNA(n2k_heading_magnetic) && 
           NMEA0183SetHDM(NMEA0183Msg, n2k_heading_magnetic))
      {
        push_out_message(NMEA0183Msg, FLAG_NAV_N2K_TO_NMEA);
      }
    } 
  }
  else if(n2k_output_cycle == 3)
  {
    if ( !N2kIsNA(n2k_depth) && NMEA0183SetDPT(NMEA0183Msg, n2k_depth, n2k_offset) )
    {
      push_out_message(NMEA0183Msg, FLAG_DEP_N2K_TO_NMEA);
    }
    if ( !N2kIsNA(n2k_depth) && NMEA0183SetDBx(NMEA0183Msg, n2k_depth, n2k_offset) )
    {
      push_out_message(NMEA0183Msg, FLAG_DEP_N2K_TO_NMEA);
    }
  }
  else if(n2k_output_cycle == 4)
  {
    if ( !N2kIsNA(n2k_heading_true)     && 
         !N2kIsNA(n2k_heading_magnetic) && 
         NMEA0183SetVHW(NMEA0183Msg, n2k_heading_true, n2k_heading_magnetic, n2k_stw ) )
    {
      push_out_message(NMEA0183Msg, FLAG_NAV_N2K_TO_NMEA);
    }
  }
  else if(n2k_output_cycle == 5)
  {
    if ( !N2kIsNA(n2k_wind_true_speed) && NMEA0183SetMWV(NMEA0183Msg, RadToDeg(n2k_wind_true_direction), NMEA0183Wind_True, n2k_wind_true_speed) )
    {
      push_out_message(NMEA0183Msg, FLAG_WIN_N2K_TO_NMEA);
    }
    if ( !N2kIsNA(n2k_wind_apparent_speed) && NMEA0183SetMWV(NMEA0183Msg, RadToDeg(n2k_wind_apparent_direction), NMEA0183Wind_Apparent, n2k_wind_apparent_speed) )
    {
      push_out_message(NMEA0183Msg, FLAG_WIN_N2K_TO_NMEA);
    }
  }
  else if(n2k_output_cycle == 6)
  {
    //
    //  Build a custom 0183 Message as the library does not natively support MTW
    //
    
    if( !N2kIsNA(n2k_water_temperature) )
    {
      tNMEA0183Msg NMEA0183Msg;
       if ( 
            NMEA0183Msg.Init("MTW", "II") && 
            NMEA0183Msg.AddDoubleField(n2k_water_temperature -  273.15, 1,tNMEA0183Msg::DefDoubleFormat,"C")
          )
       {
         push_out_message(NMEA0183Msg, FLAG_ENV_N2K_TO_NMEA);
       }
    }
  }
  n2k_output_cycle += 1;
  if(n2k_output_cycle >= 7)
  {
    n2k_output_cycle = 0;
  }
}


void append_n2k_gps_data_to_string(String &the_string)
{

    for(i=0; i < MAX_N2K_BATTERIES; i++)
    {
      Serial.print("Battery map ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(n2k_battery_map[i]);
      Serial.print("-->");
      Serial.println(n2k_battery_voltage[i]);
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

}


void possibly_convert_nmea_sentence(String nmea0183_sentence)
{
  Serial.print("COWABUNGA: I should be converting "); Serial.println(nmea0183_sentence);
}


#endif


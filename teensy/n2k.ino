#ifdef N2K_CODE_ON

#include "n2k.h"
//#include "globals.h"
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
#include <AIS.h>
//#include <NMEA0183Msg.h>
//#include <math.h>


#define N2K_RAPID_VALID_DURATION	2000UL		// N2K Rapid data valid for 2 second
#define N2K_NORMAL_VALID_DURATION	7000UL		// N2K Normal data valid for 7 seconds


void n2k_setup()
{
  sequence_id = 0;
  
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

  FLAG_GPS_N2K_TO_NMEA = false;
  FLAG_GPS_NMEA_TO_N2K = false;

  FLAG_ENV_INT_TO_N2K = false;
  FLAG_ENV_INT_TO_NMEA = false;
  FLAG_ENV_N2K_TO_NMEA = false;
  FLAG_ENV_NMEA_TO_N2K = false;

  FLAG_VOL_INT_TO_N2K = false;

  FLAG_NAV_N2K_TO_NMEA = false;
  FLAG_NAV_NMEA_TO_N2K = false;

  FLAG_DEP_N2K_TO_NMEA = false;
  FLAG_DEP_NMEA_TO_N2K = false;

  FLAG_WIN_N2K_TO_NMEA = false;
  FLAG_WIN_NMEA_TO_N2K = false;

  FLAG_AIS_N2K_TO_NMEA = false;
  FLAG_AIS_NMEA_TO_N2K = false;

  
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

char next_sequence_id()
{
  sequence_id += 1;
  if( sequence_id >= 253)
  {
    sequence_id = 0;
  }
  return sequence_id;
}

void HandleNMEA2000Messages(const tN2kMsg &N2kMsg)
{
  //Serial.print("I am switching on ");
  //Serial.println(N2kMsg.PGN);
  
  switch (N2kMsg.PGN) {
    case 126992UL: HandleSystemTime(N2kMsg); break;

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
    n2k_cogsog_timestamp = millis(); if ( HeadingReference==N2khr_magnetic
    && !N2kIsNA(n2k_cog) && !N2kIsNA(n2k_variation)) {
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
    
    // If the UNIX time is some time after 2020, and the GNSSmethod is good, this is all valid 
    
    if(GNSSmethod > 0 && GNSSmethod < 6)
    {
      n2k_fix_valid          = true;
      n2k_location_timestamp = millis();
      n2k_fix_timestamp      = millis();
    }
    else
    {
      n2k_hdp                = N2kDoubleNA;
      n2k_altitude           = N2kDoubleNA;
      n2k_fix_geosep         = N2kDoubleNA;
      n2k_location_timestamp = 0;
      n2k_fix_timestamp      = 0;
      n2k_fix_valid          = false;
    }

    unsigned long unix_time = ( (unsigned long) n2k_fix_days_1970  * 3600UL * 24UL) + n2k_fix_seconds;

    if( unix_time > 1587614400UL )
    {
      setTime(unix_time);
    }
  }

}

void HandleSystemTime(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  uint16_t SystemDate;
  double SystemTime;
  tN2kTimeSource TimeSource;
  
  if ( ParseN2kPGN126992(N2kMsg, SID, SystemDate, SystemTime, TimeSource) )
  {
    Serial.print("COWABUNGA SystemDate="); Serial.println(SystemDate);
    Serial.print("COWABUNGA SystemTime="); Serial.println(SystemTime);
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
    if ( !N2kIsNA(n2k_depth) && NMEA0183SetDBT(NMEA0183Msg, n2k_depth) )
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
  
  //
  //  One other thing we want to do is push out "internal" data (Battery,
  //  Ambient, etc.) as N2K if the appropriate flags are turned on
  //
  
  if(FLAG_ENV_INT_TO_N2K && n2k_output_cycle == 0)
  {
    tN2kMsg N2kMsg;
    double water_temperature_to_send = n2k_water_temperature;
    if(water_temperature_to_send != N2kDoubleNA)
    {
      water_temperature_to_send = CToKelvin(n2k_water_temperature);
    }
    SetN2kPGN130310(N2kMsg, next_sequence_id(), water_temperature_to_send, FToKelvin(temperature), pressure * 3386.0);
    NMEA2000.SendMsg(N2kMsg);

  }
  
  if(FLAG_VOL_INT_TO_N2K && n2k_output_cycle == 1)
  {
    tN2kMsg N2kMsg;
    
    char a_sequence_id = next_sequence_id();
    
    if(battery_one > 1.0)
    {
      SetN2kDCBatStatus(N2kMsg, 1, battery_one, N2kDoubleNA, N2kDoubleNA, a_sequence_id);
      NMEA2000.SendMsg(N2kMsg);
    }

    if(battery_two > 1.0)
    {
      SetN2kDCBatStatus(N2kMsg, 2, battery_two, N2kDoubleNA, N2kDoubleNA, a_sequence_id);
      NMEA2000.SendMsg(N2kMsg);
    }

    if(battery_three > 1.0)
    {
      SetN2kDCBatStatus(N2kMsg, 3, battery_three, N2kDoubleNA, N2kDoubleNA, a_sequence_id);
      NMEA2000.SendMsg(N2kMsg);
    }

    if(charger_one > 1.0)
    {
      SetN2kDCBatStatus(N2kMsg, 4, charger_one, N2kDoubleNA, N2kDoubleNA, a_sequence_id);
      NMEA2000.SendMsg(N2kMsg);
    }

    if(charger_two > 1.0)
    {
      SetN2kDCBatStatus(N2kMsg, 5, charger_two, N2kDoubleNA, N2kDoubleNA, a_sequence_id);
      NMEA2000.SendMsg(N2kMsg);
    }

    if(charger_three > 1.0)
    {
      SetN2kDCBatStatus(N2kMsg, 6, charger_three, N2kDoubleNA, N2kDoubleNA, a_sequence_id);
      NMEA2000.SendMsg(N2kMsg);
    }

  }

  //
  // And push out GPS if we are *not* receiving it over N2K
  //  
  
  if(!n2k_fix_valid)
  {
  
  }
}


/*
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
*/


/*
***************************************************************************
*
* Everything below here is related to converting NMEA0183 message to N2K,
* which is fairly limited but functional (and is mostly copied from Timo
* Lappalainen's examples).
*
***************************************************************************
*/

struct tBoatData {
  unsigned long DaysSince1970;   // Days since 1970-01-01
  
  double TrueHeading,SOG,COG,Variation,Deviation,Depth,Offset,
         GPSTime,// Secs since midnight,
         Latitude, Longitude, Altitude, HDOP, GeoidalSeparation, DGPSAge;
  int GPSQualityIndicator, SatelliteCount, DGPSReferenceStationID;
  bool MOBActivated;

public:
  tBoatData() {
    TrueHeading=0;
    SOG=0;
    COG=0; 
    Variation=0.0;
    Deviation=0.0;
    GPSTime=0;
    Altitude=0;
    HDOP=100000;
    DGPSAge=100000;
    DaysSince1970=0; 
    MOBActivated=false; 
    SatelliteCount=0; 
    DGPSReferenceStationID=0;
  };
};

tBoatData BoatData;

tN2kGNSSmethod GNSMethofNMEA0183ToN2k(int Method)
{
  switch (Method) {
    case 0: return N2kGNSSm_noGNSS;
    case 1: return N2kGNSSm_GNSSfix;
    case 2: return N2kGNSSm_DGNSS;
    default: return N2kGNSSm_noGNSS;  
  }
}



void convertGGA(tNMEA0183Msg nmea_message)
{
  if (NMEA0183ParseGGA_nc(nmea_message,BoatData.GPSTime,BoatData.Latitude,BoatData.Longitude,
                   BoatData.GPSQualityIndicator,BoatData.SatelliteCount,BoatData.HDOP,BoatData.Altitude,BoatData.GeoidalSeparation,
                   BoatData.DGPSAge,BoatData.DGPSReferenceStationID))
  {
    tN2kMsg N2kMsg;
    SetN2kGNSS(N2kMsg, next_sequence_id(), BoatData.DaysSince1970,BoatData.GPSTime,BoatData.Latitude,BoatData.Longitude,BoatData.Altitude,
               N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(BoatData.GPSQualityIndicator),BoatData.SatelliteCount,BoatData.HDOP,0,
               BoatData.GeoidalSeparation,1,N2kGNSSt_GPS,BoatData.DGPSReferenceStationID,BoatData.DGPSAge);
    
    NMEA2000.SendMsg(N2kMsg);
  }
}

void convertRMC(tNMEA0183Msg nmea_message)
{
  NMEA0183ParseRMC_nc(nmea_message, BoatData.GPSTime, BoatData.Latitude, BoatData.Longitude, BoatData.COG, BoatData.SOG, BoatData.DaysSince1970, BoatData.Variation);
}

void convertMTW(tNMEA0183Msg nmea_message)
{
  //
  // Have to custom parse this as the library does not include an MTW parser
  //
  
  float water_temp_in_celcius = atof(nmea_message.Field(0));
  
  tN2kMsg N2kMsg;
  SetN2kPGN130310(N2kMsg, next_sequence_id(), CToKelvin(water_temp_in_celcius), FToKelvin(temperature), pressure * 3386.0);
  NMEA2000.SendMsg(N2kMsg);
}

void convertHDG(tNMEA0183Msg nmea_message)
{
  //
  // Have to custom parse this as the library does not include an HDG parser
  //
  
  float heading_magnetic = atof(nmea_message.Field(0));
  float deviation = atof(nmea_message.Field(1));
  float variation = atof(nmea_message.Field(3));
  
  if(nmea_message.Field(2)[0] == 'W')
    deviation = deviation * -1.0;

  if(nmea_message.Field(4)[0] == 'W')
    variation = variation * -1.0;
    
  BoatData.Deviation = DegToRad(deviation);
  BoatData.Variation = DegToRad(variation);
  // float heading_true = heading_magnetic + deviation + variation; 
    
  tN2kMsg N2kMsg;
  SetN2kMagneticHeading(N2kMsg, next_sequence_id(), DegToRad(heading_magnetic), DegToRad(deviation),  DegToRad(variation));  
  NMEA2000.SendMsg(N2kMsg);
}

void convertHDM(tNMEA0183Msg nmea_message)
{

  double heading_magnetic;

  if (NMEA0183ParseHDM_nc(nmea_message, heading_magnetic))
  {
    tN2kMsg N2kMsg;
    SetN2kMagneticHeading(N2kMsg, next_sequence_id(), heading_magnetic, BoatData.Deviation,  BoatData.Variation);  
    NMEA2000.SendMsg(N2kMsg);
  }
}

void convertHDT(tNMEA0183Msg nmea_message)
{
  if (NMEA0183ParseHDT_nc(nmea_message, BoatData.TrueHeading))
  {
    tN2kMsg N2kMsg;
    SetN2kPGN127250(N2kMsg, next_sequence_id(), BoatData.TrueHeading, BoatData.Deviation,  BoatData.Variation, N2khr_true);  
    NMEA2000.SendMsg(N2kMsg);
  }
}

void convertVHW(tNMEA0183Msg nmea_message)
{
  double heading_magnetic;

  if (NMEA0183ParseVHW_nc(nmea_message, BoatData.TrueHeading, heading_magnetic, BoatData.SOG))
  {
    tN2kMsg N2kMsg;
    SetN2kBoatSpeed(N2kMsg, next_sequence_id(), BoatData.SOG);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void convertDBT(tNMEA0183Msg nmea_message)
{
  BoatData.Depth = atof(nmea_message.Field(2));
  tN2kMsg N2kMsg;
  SetN2kWaterDepth(N2kMsg, next_sequence_id(), BoatData.Depth, BoatData.Offset);
  NMEA2000.SendMsg(N2kMsg);
}

void convertDPT(tNMEA0183Msg nmea_message)
{
  BoatData.Depth = atof(nmea_message.Field(0));
  BoatData.Offset = atof(nmea_message.Field(1));
  tN2kMsg N2kMsg;
  SetN2kWaterDepth(N2kMsg, next_sequence_id(), BoatData.Depth, BoatData.Offset);
  NMEA2000.SendMsg(N2kMsg);
}

void convertMWV(tNMEA0183Msg nmea_message)
{
  double wind_angle, wind_speed;
  tNMEA0183WindReference wind_reference;
  tN2kWindReference n2k_wind_reference = N2kWind_True_boat;
  if(NMEA0183ParseMWV_nc(nmea_message, wind_angle, wind_reference, wind_speed))
  {
  
    if(wind_reference == NMEA0183Wind_Apparent)
    {
      n2k_wind_reference = N2kWind_Apparent;
    }
    
    tN2kMsg N2kMsg;
    SetN2kPGN130306(N2kMsg, next_sequence_id(), wind_speed, wind_angle, n2k_wind_reference);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void convertVDO(tNMEA0183Msg nmea_message)
{

  //
  // This is a little more complicated for a couple of reasons:
  //
  //	1.  The main NMEA0183 Library does not decode the VDM binary
  //	payload, so we have to use an additional library.
  //
  //	2.  VDM messages can be multipart, and we can't exactly run a huge
  //	in memory Redis store to keep previous messages around in order to
  //	match them up.  We just store the *most* recent, and if it does not
  //	match up, we give up.
  //

  uint8_t fragment_number;
  uint8_t total_fragments;
  uint message_sequence_id;
  char channel;
  uint bitstream_length = 128;
  char ascii_encoded_bitstream[128];
  uint fill_bits;
  
  if(NMEA0183ParseVDM_nc(nmea_message, fragment_number, total_fragments, message_sequence_id, channel, bitstream_length, ascii_encoded_bitstream, fill_bits))
  {
    if(bitstream_length > 0)
    {
      if(bitstream_length >= 128)
      {
        bitstream_length = 127;
      }
      ascii_encoded_bitstream[bitstream_length] = '\0';
      if(total_fragments > 1)
      {
        if(fragment_number == 1)
        {
          previous_ais_message_sequence_id = message_sequence_id;
          strncpy(previous_ais_message_ascii_encoded_bitstream, ascii_encoded_bitstream, 64);
          return;
        }
        if(previous_ais_message_sequence_id != message_sequence_id)
        {
          return;
        }
        char temp_ascii_encoded_bitstream[128];
        strncpy(temp_ascii_encoded_bitstream, previous_ais_message_ascii_encoded_bitstream, 64);
        strncat(temp_ascii_encoded_bitstream, ascii_encoded_bitstream, 64);
        strncpy(ascii_encoded_bitstream, temp_ascii_encoded_bitstream, 128);
      }
      
      AIS ais_msg(ascii_encoded_bitstream, fill_bits);
      tN2kMsg N2kMsg;

      if(ais_msg.get_numeric_type() == 1 || ais_msg.get_numeric_type() == 2 ||ais_msg.get_numeric_type() == 3)
      {

        SetN2kAISClassAPosition(N2kMsg, next_sequence_id(), (tN2kAISRepeat) ais_msg.get_repeat(), ais_msg.get_mmsi(), ais_msg.get_latitude() / 60000.0, ais_msg.get_longitude() /60000.0,
                        ais_msg.get_posAccuracy_flag(),  ais_msg.get_raim_flag(), ais_msg.get_timeStamp(), DegToRad(ais_msg.get_COG() / 10.0), ais_msg.get_SOG() / 10.0, 
                        DegToRad(ais_msg.get_HDG() / 10.0), ais_msg.get_rot(), (tN2kAISNavStatus) ais_msg.get_navStatus());
        NMEA2000.SendMsg(N2kMsg);
      }

      else if (ais_msg.get_numeric_type() == 5)
      {
        tmElements_tt tm;
        tm.Hour = ais_msg.get_hour(); tm.Minute = ais_msg.get_minute(); tm.Second = 0;
        tm.Day = ais_msg.get_day(); tm.Month = ais_msg.get_month(); tm.Year = year();
        if(tm.Month < month())
        {
          tm.Year += 1;
        }
        time_t arrival_eta = makeTime(tm);

        tN2kAISTranceiverInfo transceiver_info = N2kaisti_Channel_A_VDL_reception;
        if( channel == 'B')
        {
          transceiver_info = N2kaisti_Channel_B_VDL_reception;
        }
        
        char call_sign[16];
        char ship_name[24];
        char destination[24];
        strncpy(call_sign, ais_msg.get_callsign(), 15);
        strncpy(ship_name, ais_msg.get_shipname(), 23);
        strncpy(destination, ais_msg.get_destination(), 23);
      
        SetN2kPGN129794(N2kMsg, next_sequence_id(), (tN2kAISRepeat) ais_msg.get_repeat(), ais_msg.get_mmsi(), 
                        ais_msg.get_imo(), call_sign, ship_name, ais_msg.get_shiptype(), ais_msg.get_to_bow() + ais_msg.get_to_stern() + 0.0,
                        ais_msg.get_to_port() + ais_msg.get_to_starboard() + 0.0, ais_msg.get_to_starboard() + 0.0, ais_msg.get_to_bow() + 0.0, 
                        arrival_eta % 86400, arrival_eta - (arrival_eta % 8600), 
                        ais_msg.get_draught(), destination, (tN2kAISVersion) ais_msg.get_ais_version(), (tN2kGNSStype) ais_msg.get_epfd(),
                        (tN2kAISDTE) ais_msg.get_dte_flag(), transceiver_info);
        NMEA2000.SendMsg(N2kMsg);

      }

      else if (ais_msg.get_numeric_type() == 18)
      {
        SetN2kAISClassBPosition(N2kMsg, next_sequence_id(), (tN2kAISRepeat) ais_msg.get_repeat(), ais_msg.get_mmsi(), 
                        ais_msg.get_latitude() / 60000.0, ais_msg.get_longitude() /60000.0, ais_msg.get_posAccuracy_flag(),  ais_msg.get_raim_flag(), 
                        ais_msg.get_timeStamp(), DegToRad(ais_msg.get_COG() / 10.0), ais_msg.get_SOG() / 10.0, DegToRad(ais_msg.get_HDG() / 10.0), (tN2kAISUnit) ais_msg.get_cs_flag(),
                        ais_msg.get_display_flag(), ais_msg.get_dsc_flag(), ais_msg.get_band_flag(), ais_msg.get_msg22_flag(), (tN2kAISMode) 0, 0);
        NMEA2000.SendMsg(N2kMsg);
      }                        
      else if (ais_msg.get_numeric_type() == 24)
      {
        if(ais_msg.get_partno() == 0)
        {
          char ship_name[24];
          strncpy(ship_name, ais_msg.get_shipname(), 23);
          SetN2kPGN129809(N2kMsg, next_sequence_id(), (tN2kAISRepeat) ais_msg.get_repeat(), ais_msg.get_mmsi(), ship_name);
          NMEA2000.SendMsg(N2kMsg);
        }
        else if(ais_msg.get_partno() == 1)
        {
        
          char call_sign[16];
          char vendor_id[8];
          strncpy(call_sign, ais_msg.get_callsign(), 15);
          strncpy(vendor_id, ais_msg.get_vendorid(), 7);
          SetN2kPGN129810(N2kMsg, next_sequence_id(), (tN2kAISRepeat) ais_msg.get_repeat(), ais_msg.get_mmsi(),
                          ais_msg.get_shiptype(), vendor_id, call_sign, ais_msg.get_to_bow() + ais_msg.get_to_stern() + 0.0, ais_msg.get_to_port() + ais_msg.get_to_starboard() + 0.0, 
                          ais_msg.get_to_starboard() + 0.0, ais_msg.get_to_bow() + 0.0, ais_msg.get_mothership_mmsi());
          NMEA2000.SendMsg(N2kMsg);
        }
      }                        
    }
  }
}


void possibly_convert_nmea_sentence(const char* nmea0183_sentence)
{

  tNMEA0183Msg nmea_message;
  if(! nmea_message.SetMessage(nmea0183_sentence))
  {
    return; // Some kind of bad data?
  }

  //
  //  GPS/Location Messages
  //
  if(FLAG_GPS_NMEA_TO_N2K && nmea_message.IsMessageCode("GGA"))
  {
    convertGGA(nmea_message);
  }
  else if (FLAG_GPS_NMEA_TO_N2K && nmea_message.IsMessageCode("RMC"))
  {
    convertRMC(nmea_message);
  }

  //
  // Environmental
  //
  else if (FLAG_ENV_NMEA_TO_N2K && nmea_message.IsMessageCode("MTW"))
  {
    convertMTW(nmea_message);
  }

  //
  // Navigation
  //
  else if (FLAG_NAV_NMEA_TO_N2K && nmea_message.IsMessageCode("HDG"))
  {
    convertHDG(nmea_message);
  }
  else if (FLAG_NAV_NMEA_TO_N2K && nmea_message.IsMessageCode("HDM"))
  {
    convertHDM(nmea_message);
  }
  else if (FLAG_NAV_NMEA_TO_N2K && nmea_message.IsMessageCode("HDT"))
  {
    convertHDT(nmea_message);
  }
  else if (FLAG_NAV_NMEA_TO_N2K && nmea_message.IsMessageCode("VHW"))
  {
    convertVHW(nmea_message);
  }

  //
  // Depth
  //
  
  else if (FLAG_DEP_NMEA_TO_N2K && nmea_message.IsMessageCode("DBT"))
  {
    convertDBT(nmea_message);
  }

  else if (FLAG_DEP_NMEA_TO_N2K && nmea_message.IsMessageCode("DPT"))
  {
    convertDPT(nmea_message);
  }

  //
  // Wind
  //

  else if (FLAG_WIN_NMEA_TO_N2K && nmea_message.IsMessageCode("MWV"))
  {
    convertMWV(nmea_message);
  }
  
  //
  // AIS
  //

  else if (FLAG_AIS_NMEA_TO_N2K && ( nmea_message.IsMessageCode("VDO") || nmea_message.IsMessageCode("VDM")))
  {
    convertVDO(nmea_message);
  }

}




#endif


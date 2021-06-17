/* 
* CFrskyTelemetry.cpp
*
* Created: 18/04/2020 10:03:45
* Author: Phil2
*/
#include "Includes.h"
#include "CObjectBuffer.h"



uint32_t Pitch;

CObjectArray<mavlink_statustext_t> CFrskyTelemetry::_statustext_queue(FRSKY_TELEM_PAYLOAD_STATUS_CAPACITY);

// default constructor
CFrskyTelemetry::CFrskyTelemetry(CSoftwareSerial* Uart)
{
	m_Uart =  Uart;
} //CFrskyTelemetry

// default destructor
CFrskyTelemetry::~CFrskyTelemetry()
{
} //~CFrskyTelemetry




void CFrskyTelemetry::Init()
{
}


//====================================================================
// send telemetry data for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
// called by scheduler at 1kHz 1B code received every 52 bytes 
//====================================================================
void CFrskyTelemetry::Update(void)
{
//    uint16_t  AvailableBytes = m_Uart->Available();
	static enum{StartBit,StartBit2}FrState = StartBit;
	bool End = false;
    uint32_t now = Core.millis();


   while(!End) 
	{        if(!m_Uart->Read(&m_Passthrough_s.new_byte))		{			return;		}		switch(FrState)		{			case StartBit:				if(m_Passthrough_s.new_byte == (uint8_t) START_STOP_SPORT)					FrState = StartBit2;				break;			case StartBit2:					FrState = StartBit;					if (m_Passthrough_s.new_byte == SENSOR_ID_28 )			// byte 0x7E is the header of each poll request				{ 					End = true;				}		}    }
	
	
    if (m_Passthrough_s.send_attiandrng)								// skip other data, send attitude (roll, pitch) and range only this iteration
	{ 
		 send_uint32(DIY_FIRST_ID+6, CalcAttAndRng());
        m_Passthrough_s.send_attiandrng = false;						// next iteration, check if we should send something other
    } 
	else 
	{																// send other sensor data if it's time for them, and reset the corresponding timer if sent
        m_Passthrough_s.send_attiandrng = true;						// next iteration, send attitude b/c it needs frequent updates to remain smooth
        if ((now - m_Passthrough_s.params_timer) >= 1000) 
		{
            send_uint32(DIY_FIRST_ID+7, CalcParam());
            m_Passthrough_s.params_timer = Core.millis();
            return;
        }
            
        CheckErrorMessageFlags();								// build message queue for sensor_status_flags
        CheckMessages();											// build message queue for non critical messages
            
//         if((now -m_Passthrough_s.MessageTimer) >800 && !m_Passthrough_s.MessageInProgress)
// 		{
// 			m_Passthrough_s.MessageInProgress = false;
// 		}
// 		if(	m_Passthrough_s.MessageInProgress)
// 		{
			if (get_next_msg_chunk())									// if there's any message in the queue, start sending them chunk by chunk; three times each chunk
			{
				send_uint32(DIY_FIRST_ID, _msg_chunk.chunk);
				return;
			}
//		}
        if ((now - m_Passthrough_s.ap_status_timer) >= 500) 
		{
            if (Config.m_RunningFlags.MAIN_INIT_COMPLETED)			// send ap status only once vehicle has been initialised 
			{  
                send_uint32(DIY_FIRST_ID+1, CalcStatus());
                m_Passthrough_s.ap_status_timer = Core.millis();
            }
            return;
        }
        if ((now - m_Passthrough_s.batt_timer) >= 1000) 
		{
            send_uint32(DIY_FIRST_ID+3, CalcBatteryState(0));
            m_Passthrough_s.batt_timer = Core.millis();
            return;
        }
        if (Sensors.m_BatteryCount > 1) 
		{
            if ((now - m_Passthrough_s.batt_timer2) >= 1000) 
			{
                send_uint32(DIY_FIRST_ID+8, CalcBatteryState(1));
                m_Passthrough_s.batt_timer2 = Core.millis();
                return;
            }
        }
        if ((now - m_Passthrough_s.gps_status_timer) >= 1000) 
		{
            send_uint32(DIY_FIRST_ID+2, CalcGpsStatus());
            m_Passthrough_s.gps_status_timer = Core.millis();
            return;
        }
        if ((now - m_Passthrough_s.home_timer) >= 500) 
		{
            send_uint32(DIY_FIRST_ID+4, CalcHome());
            m_Passthrough_s.home_timer = Core.millis();
            return;
        }
        if ((now - m_Passthrough_s.velandyaw_timer) >= 500) 
		{
            send_uint32(DIY_FIRST_ID+5, calc_velandyaw());
            m_Passthrough_s.velandyaw_timer = Core.millis();
            return;
        }
        if ((now - m_Passthrough_s.gps_latlng_timer) >= 1000) 
		{
            send_uint32(GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(&m_Passthrough_s.send_latitude)); // gps latitude or longitude
            if (!m_Passthrough_s.send_latitude) 
			{ 
                m_Passthrough_s.gps_latlng_timer = Core.millis();			// we've cycled and sent one each of longitude then latitude, so reset the timer
            }
            return;
        }
    }
    // if nothing else needed to be sent, send attitude (roll, pitch) and range data
    send_uint32(DIY_FIRST_ID+6, CalcAttAndRng());
    
}




//===================================================================================
// build up the frame's crc for FrSky SPort protocol (X-receivers)
//===================================================================================
void CFrskyTelemetry::calc_crc(uint8_t byte)
{
    _crc += byte; //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0xFF;
}

//===================================================================================
// send the frame's crc at the end of the frame
// for FrSky SPort protocol (X-receivers)
//===================================================================================
void CFrskyTelemetry::send_crc(void) 
{
    send_byte(0xFF - _crc);
    _crc = 0;
}


//===================================================================================
//  send 1 byte and do byte stuffing
//===================================================================================
// void CFrskyTelemetry::send_byte(uint8_t byte)
// {
// 	// FrSky SPort protocol (X-receivers)
// 	if (byte == START_STOP_SPORT) 
// 	{
// 		m_Uart->WriteTxUnbuffered((uint8_t)0x7D);
// 		m_Uart->WriteTxUnbuffered((uint8_t)0x5E);
// 	} 
// 	else 
// 	{
// 		m_Uart->WriteTxUnbuffered(byte);
// 	}
// 	calc_crc(byte);
//    
// }

//===================================================================================
// send one uint32 frame of FrSky data - for FrSky SPort protocol (X-receivers)
//===================================================================================
// void  CFrskyTelemetry::send_uint32( uint16_t id, uint32_t data)
// {
//     send_byte(0x10); // frame type
//     uint8_t *bytes = (uint8_t*)&id;
//     send_byte(bytes[0]); // LSB
//     send_byte(bytes[1]); // MSB
//     bytes = (uint8_t*)&data;
//     send_byte(bytes[0]); // LSB
//     send_byte(bytes[1]);
//     send_byte(bytes[2]);
//     send_byte(bytes[3]); // MSB
//     send_crc();
// }



//===================================================================================
// send one uint32 frame of FrSky data - for FrSky SPort protocol (X-receivers)
//===================================================================================
void  CFrskyTelemetry::send_uint32( uint16_t id, uint32_t data)
{
	m_TxFrame.Frame.FrameType = 0x10;
	m_TxFrame.Frame.ID = id;
	m_TxFrame.Frame.Data = data;
	
	for(uint8_t i=0;i<8;i++)
	{
		calc_crc(m_TxFrame.Message[i]);
	}	
	m_TxFrame.Frame.crc = 0xFF - _crc;

	m_Uart->WriteTxBuffer(m_TxFrame.Message,8,true);
	Buzzer.ToggleOutput();
}






//=========================================================================================================================================================
// grabs one "chunk" (4 bytes) of the queued message to be transmitted for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//=========================================================================================================================================================

bool CFrskyTelemetry::get_next_msg_chunk(void)
{
    if (_statustext_queue.empty()) 
        return false;

    if (_msg_chunk.repeats == 0)					// if it's the first time get_next_msg_chunk is called for a given chunk 
	{ 
        uint8_t character = 0;
        _msg_chunk.chunk = 0;						// clear the 4 bytes of the chunk buffer

        for (int i = 3; i > -1 && _msg_chunk.char_index < sizeof(_statustext_queue[0]->text); i--) 
		{
            character = _statustext_queue[0]->text[_msg_chunk.char_index++];

            if (!character) 
			{
                break;
            }

            _msg_chunk.chunk |= character << i * 8;
        }

        if (!character || (_msg_chunk.char_index == sizeof(_statustext_queue[0]->text))) 
		{ 
			// we've reached the end of the message (string terminated by '\0' or last character of the string has been processed)
            _msg_chunk.char_index = 0; // reset index to get ready to process the next message

            // add severity which is sent as the MSB of the last three bytes of the last chunk (bits 24, 16, and 8) since a character is on 7 bits
            _msg_chunk.chunk |= (_statustext_queue[0]->severity  & 0x4)<<21;
            _msg_chunk.chunk |= (_statustext_queue[0]->severity  & 0x2)<<14;
            _msg_chunk.chunk |= (_statustext_queue[0]->severity  & 0x1)<<7;
        }
    }

    if (_msg_chunk.repeats++ > 2)				// repeat each message chunk 3 times to ensure transmission
	{ 
	    _msg_chunk.repeats = 0;
	    if (_msg_chunk.char_index == 0)	
		{		// if we're ready for the next message	
		    _statustext_queue.remove(0);
			m_Passthrough_s.MessageTimer = Core.millis();
			m_Passthrough_s.MessageInProgress = false;
		}
    }
    return true;
}


//=========================================================================================================================================================
//  add message to message cue for transmission through FrSky link  for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//=========================================================================================================================================================
void CFrskyTelemetry::QueueMessage(MAV_SEVERITY severity, const char *text)
{
    mavlink_statustext_t statustext{};

    statustext.severity = severity;
    strncpy(statustext.text, text, sizeof(statustext.text));

    // The force push will ensure comm links do not block other comm links forever if they fail. If we push to a full buffer then 
	// we overwrite the oldest entry, effectively removing the block but not until the buffer fills up.
    _statustext_queue.push_force(statustext);
}


//=========================================================================================================================================================
// add sensor_status_flags information to message cue, normally passed as sys_status mavlink messages to the GCS, for transmission through FrSky link
// for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 //========================================================================================================================================================
void CFrskyTelemetry::CheckErrorMessageFlags(void)
{
	uint32_t now = Core.millis();

	if ((now - m_CheckErrorMessageTimer) >= 7000)		// prevent repeat every 5 seconds error is reported one at a time (in order of preference) 
	{	
		if(Config.m_RunningFlags.PANIC_MODE_TRIGGERED)
		{
			QueueMessage(MAV_SEVERITY_CRITICAL, "In Panic Mode");
		}
		if (Config.m_RunningFlags.IMU_FAIL) 
		{
			QueueMessage(MAV_SEVERITY_CRITICAL, "IMU Fail");
		} 

		else if (!Gps.GetStatus() == GPS_FAILED) 
		{
			QueueMessage(MAV_SEVERITY_CRITICAL, "Bad GPS Health");
		} 

// 		else if (!Config.m_RunningFlags.IMU_VALID ) 
// 		{
// 			QueueMessage(MAV_SEVERITY_CRITICAL, "Bad Gyro Health");
// 		} 
// 		else if (!Config.m_RunningFlags.IMU_VALID ) 
// 		{
// 			QueueMessage(MAV_SEVERITY_CRITICAL, "Bad Accel Health");
// 		} 
		else if (Config.m_RunningFlags.COMPASS_FAIL) 
		{
			QueueMessage(MAV_SEVERITY_CRITICAL, "Bad Compass Health");
		} 
		else if (!Config.m_RunningFlags.HEAD_LIDAR_FAIL ) 
		{
			QueueMessage(MAV_SEVERITY_CRITICAL, "Bad LiDAR Health");
		} 
		else if (Config.m_RunningFlags.GEO_FENCE_BREACH > 0) 
		{
			QueueMessage(MAV_SEVERITY_CRITICAL, "Geofence Breach");
		} 
		else if (Config.m_RunningFlags.LOST_RC_SIGNAL > 0) 
		{
			QueueMessage(MAV_SEVERITY_CRITICAL, "No RC Receiver");
		} 
		else if (Config.m_RunningFlags.LOGGING_FAIL > 0) 
		{
			QueueMessage(MAV_SEVERITY_CRITICAL, "Bad Logging");
		}
		m_CheckErrorMessageTimer = now;
	}
}
//===================================================================================
// not so critical Messages
//===================================================================================
void CFrskyTelemetry::CheckMessages(void)
{

    uint32_t now = Core.millis();
    if ((now - m_MessageTimer) >= 10000)	// prevent repeating any  message unless 10 seconds have passed // multiple errors can be reported at a time
	{ 
            
        if (Config.m_RunningFlags.ARMED) 
		{
            GetNavigationState();
            m_MessageTimer = now;
        }
        if (!Config.m_RunningFlags.GYRO_CALIBRATED) 
		{
            QueueMessage(MAV_SEVERITY_CRITICAL, "Giro Not Cal");
            m_MessageTimer = now;
        }
        if (!Config.m_RunningFlags.ACC_CALIBRATED) 
		{
            QueueMessage(MAV_SEVERITY_CRITICAL, "ACC Not Cal");
            m_MessageTimer = now;
        }
        if (!Config.m_RunningFlags.MAG_CALIBRATED) 
		{
            QueueMessage(MAV_SEVERITY_CRITICAL, "Mag Not Cal");
            m_MessageTimer = now;
        }
    }
}


//==================================================================

void CFrskyTelemetry::GetNavigationState()
{
	switch(Navigation.m_NavigationState)
	{
		case eControlByRc:
			 QueueMessage(MAV_SEVERITY_INFO, "Control By RC");
			break;
		case eStartNavigation:
			 QueueMessage(MAV_SEVERITY_INFO, "Starting Navigation");
			break;
		case eOnRouteToNextWapoint:
			 QueueMessage(MAV_SEVERITY_INFO, "On route");
			break;
		case eReturnHome:
			 QueueMessage(MAV_SEVERITY_INFO, "Return to home");
			break;
		case ePositionHold:
			QueueMessage(MAV_SEVERITY_INFO, "Position Hold");
			break;
		default:
			break;	
	}
}




      
//==================================================================
// prepare parameter data
// for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//==================================================================
uint32_t CFrskyTelemetry::CalcParam(void)
{

    uint32_t RetVal = 0;

    // cycle through paramIDs
    if (m_ParamID >= 5) 
	{
        m_ParamID = 0;
    }
    m_ParamID++;
    switch(m_ParamID)
	 {
    case 1:
        RetVal = 10; // MAV_TYPE_SURFACE_BOAT    Surface vessel, boat, ship
        break;
    case 2: // was used to send the battery failsafe voltage
    case 3: // was used to send the battery failsafe capacity in mAh
        break;
    case 4:
        RetVal = (uint32_t)round(5000); // battery pack capacity in mAh
        break;
    case 5:
        RetVal = (uint32_t)round(3000); // battery pack capacity in mAh
        break;
    }
    //Reserve first 8 bits for param ID, use other 24 bits to store parameter value
    RetVal = (m_ParamID << PARAM_ID_OFFSET) | (RetVal & PARAM_VALUE_LIMIT);
    
    return(RetVal);
}

//==================================================================
//  prepare gps latitude/longitude data for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//==================================================================
uint32_t CFrskyTelemetry::calc_gps_latlng(bool *send_latitude)
{
    uint32_t latlng;

    // alternate between latitude and longitude
    if ((*send_latitude) == true) 
	{
        if (Gps.m_Coords.Latitude < 0) 
		{
            latlng = ((labs(Gps.m_Coords.Latitude)/100)*6) | 0x40000000;
        } 
		else 
		{
            latlng = ((labs(Gps.m_Coords.Latitude)/100)*6);
        }
        (*send_latitude) = false;
    } 
	else 
	{
        if (Gps.m_Coords.Longitude < 0) 
		{
            latlng = ((labs(Gps.m_Coords.Longitude)/100)*6) | 0xC0000000;
        } else {
            latlng = ((labs(Gps.m_Coords.Longitude)/100)*6) | 0x80000000;
        }
        (*send_latitude) = true;
    }
    return latlng;
}

//==================================================================
// prepare gps status data
// for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//==================================================================
uint32_t CFrskyTelemetry::CalcGpsStatus(void)
{
    uint32_t GpsStatus;

    
    GpsStatus = (Gps.m_GpsReadings.NumSats < GPS_SATS_LIMIT) ? Gps.m_GpsReadings.NumSats : GPS_SATS_LIMIT;	// number of GPS satellites visible (limit to 15 (0xF)  value is stored on 4 bits)
    
    GpsStatus |= ((Gps.m_GpsReadings.GpsFixType < GPS_STATUS_LIMIT) ? Gps.m_GpsReadings.GpsFixType : GPS_STATUS_LIMIT)<<GPS_STATUS_OFFSET;	// GPS receiver status (limit to 0-3 (0x3) since the value is stored on 2 bits: NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, 
																											// GPS_OK_FIX_3D or GPS_OK_FIX_3D_DGPS or GPS_OK_FIX_3D_RTK_FLOAT or GPS_OK_FIX_3D_RTK_FIXED = 3)
   
    GpsStatus |= PrepareNumber(round(Gps.m_GpsReadings.Hdop * 0.1f),2,1)<<GPS_HDOP_OFFSET;					// GPS horizontal dilution of precision in dm
    
    GpsStatus |= ((Gps.m_GpsReadings.GpsFixType > GPS_STATUS_LIMIT) ? Gps.m_GpsReadings.GpsFixType-GPS_STATUS_LIMIT : 0)<<GPS_ADVSTATUS_OFFSET;	// GPS receiver advanced status (0: no advanced fix, 
																											// 1: GPS_OK_FIX_3D_DGPS, 2: GPS_OK_FIX_3D_RTK_FLOAT, 3: GPS_OK_FIX_3D_RTK_FIXED)
    GpsStatus |= PrepareNumber(round(Gps.m_GpsReadings.Altitude * 0.1f),2,2)<<GPS_ALTMSL_OFFSET;				// Altitude MSL in dm
    return GpsStatus;
}

//==================================================================
// prepare battery data
// for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//==================================================================
uint32_t CFrskyTelemetry::CalcBatteryState(uint8_t instance)
{
    uint32_t BatteryValue;
    float Current, ConsumedMah;
	float BattVolts;
 	if(instance == 0)
 	{
		BattVolts = Sensors.m_SensorValues.VoltsBattery_1;
		Current = 0;														// Sensors.m_SensorValues.Current;
		ConsumedMah = 0; // Sensors.m_SensorValues.MahUsed;
	}
 	else
 	{
 		BattVolts = 0;
 		Current = 1;														//Sensors.m_SensorValues.Current;
 		ConsumedMah = Sensors.m_SensorValues.MahUsed;
 	}
    
    
    BatteryValue = (((uint16_t)round(BattVolts * 10.0f)) & BATT_VOLTAGE_LIMIT);		// battery voltage in decivolts, can have up to a 12S battery (4.25Vx12S = 51.0V)
	BatteryValue |= PrepareNumber(round(Current * 10.0f), 2, 1)<<BATT_CURRENT_OFFSET;
																			// battery current drawn since power on in mAh (limit to 32767 (0x7FFF) since value is stored on 15 bits)
    BatteryValue |= ((ConsumedMah < BATT_TOTALMAH_LIMIT) ? ((uint16_t)round(ConsumedMah) & BATT_TOTALMAH_LIMIT) : BATT_TOTALMAH_LIMIT)<<BATT_TOTALMAH_OFFSET;
    return BatteryValue;
}

//==================================================================
//  prepare various autopilot status data
//  for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//==================================================================
uint32_t CFrskyTelemetry::CalcStatus(void)
{
   
	CMyMath Math;
	uint32_t Status;
	float Temp = 20.1;
    // IMU temperature: offset -19, 0 means temp =< 19°, 63 means temp => 82°
    uint8_t imu_temp = (uint8_t) round(Math.Constrain(Temp, AP_IMU_TEMP_MIN, AP_IMU_TEMP_MAX) - AP_IMU_TEMP_MIN);

    //  mode number (limit to 31 (0x1F) since the value is stored on 5 bits)
    Status = GetMode(Navigation.m_NavigationState) & AP_CONTROL_MODE_LIMIT;
   
    
    Status |= (uint8_t)(0<<AP_SIMPLE_OFFSET);						// simple/super simple modes flags
    Status |= (uint8_t)(0<<AP_SSIMPLE_OFFSET);
    
    Status |= (uint8_t)(0 << FR_FLYING_OFFSET);						// is_flying flag
    
    Status |= (uint8_t)(Config.m_RunningFlags.ARMED)<<FR_ARMED_OFFSET;	// armed flag
    
    Status |= (uint8_t)(Config.m_RunningFlags.BATTERY_FAILSAFE)<<FR_BATT_FS_OFFSET;	// battery failsafe flag
    
    if(!Config.m_RunningFlags.IMU_FAIL)
		Status |= (uint8_t)(1<<AP_EKF_FS_OFFSET);	// bad IMU flag
    
    Status |= imu_temp << AP_IMU_TEMP_OFFSET;							// IMU temperature

    return Status;
}

//==================================================================
// prepare home position related data
// for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//==================================================================
uint32_t CFrskyTelemetry::CalcHome(void)
{
	uint32_t home;
	    
	if (Config.m_CourseWaypoints[0].nLatitide!= 0 || Config.m_CourseWaypoints[0].nLongitude != 0)	// check home_loc is valid
	{
		home = PrepareNumber(round(NavigationFunctions.m_Bearings.DistanceToHome), 3, 2);			 // distance between vehicle and home_loc in meters
		// angle from front of vehicle to the direction of home_loc in 3 degree increments (just in case, limit to 127 (0x7F) since the value is stored on 7 bits)
		home |= (((uint8_t)round(NavigationFunctions.m_Bearings.BoatToHomeBearing * 0.00333f)) & HOME_BEARING_LIMIT)<<HOME_BEARING_OFFSET;
	}
	else
		home = 0;
    
    home |= PrepareNumber(round(10 * 0.1f), 3, 2)<<HOME_ALT_OFFSET;		// altitude above home in decimeters
    return home;
}

//==================================================================
// prepare velocity and yaw data
// for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//==================================================================
uint32_t CFrskyTelemetry::calc_velandyaw(void)
{
    uint16_t Heading;
	float vspd = 0;
    // vertical velocity in dm/s
    uint32_t velandyaw = PrepareNumber(round(vspd * 10), 2, 1);
 
    // horizontal velocity in dm/s (use airspeed if available and enabled - even if not used - otherwise use groundspeed)
 
	 // otherwise send groundspeed estimate from ahrs
        velandyaw |= PrepareNumber(round(1.5 * 10), 2, 1)<<VELANDYAW_XYVEL_OFFSET;
    
    // yaw from [0;360] centidegrees to .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
    Heading = (uint16_t)round(Ahrs.GetHeading()); 
	Heading *= 5;
	Heading &= VELANDYAW_YAW_LIMIT;
	velandyaw |= (uint32_t) Heading<<VELANDYAW_YAW_OFFSET;
    return velandyaw;
}

//==========================================================================================================
// prepare attitude (roll, pitch) and range data for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
//==========================================================================================================
uint32_t CFrskyTelemetry::CalcAttAndRng(void)
{
    uint16_t Range = HeadControl.m_BestCourse.Distance;

    uint32_t AttiAndRng;
	
    // roll from [-90;90] degrees to unsigned .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
    AttiAndRng = (uint16_t)(Ahrs.GetRollD() +90);
	AttiAndRng *=10;
	AttiAndRng &=  ATTIANDRNG_ROLL_LIMIT;
//	attiandrng =900;
    // pitch from [-90;90] degrees to unsigned .2 degree increments [0;900] (just in case, limit to 1023 (0x3FF) since the value is stored on 10 bits)
    Pitch = Ahrs.GetPitchD();
	Pitch +=90;
	Pitch *= 5;
	Pitch &=  ATTIANDRNG_PITCH_LIMIT;
	AttiAndRng |= (Pitch << ATTIANDRNG_PITCH_OFFSET);     
    // range finder measurement in cm
    AttiAndRng |= PrepareNumber(Range, 4, 1)<<ATTIANDRNG_RNGFND_OFFSET;
    return AttiAndRng;
}

/*
 * prepare value for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint16_t CFrskyTelemetry::PrepareNumber(int32_t Number, uint8_t Digits, uint8_t Power)
{
    uint16_t RetVal = 0;
    uint32_t AbsNumber = abs(Number);

    if ((Digits == 2) && (Power == 1)) 
	{ 
        if (AbsNumber < 100)					// number encoded on 8 bits: 7 bits for digits + 1 for 10^power 
		{
            RetVal = AbsNumber<<1;
        } 
		else if (AbsNumber < 1270) 
		{
            RetVal = ((uint8_t)roundf(AbsNumber * 0.1f)<<1)|0x1;
        } 
		else { // transmit max possible value (0x7F x 10^1 = 1270)
            RetVal = 0xFF;
        }
        if (Number < 0) 
		{ // if number is negative, add sign bit in front
            RetVal |= 0x1<<8;
        }
    } 
	else if ((Digits == 2) && (Power == 2)) 
	{ // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
        if (AbsNumber < 100) 
		{
            RetVal = AbsNumber<<2;
        } 
		else if (AbsNumber < 1000) 
		{
            RetVal = ((uint8_t)round(AbsNumber * 0.1f)<<2)|0x1;
        } 
		else if (AbsNumber < 10000) 
		{
            RetVal = ((uint8_t)round(AbsNumber * 0.01f)<<2)|0x2;
        } 
		else if (AbsNumber < 127000) 
		{
            RetVal = ((uint8_t)roundf(AbsNumber * 0.001f)<<2)|0x3;
        } 
		else 
		{ // transmit max possible value (0x7F x 10^3 = 127000)
            RetVal = 0x1FF;
        }
        if (Number < 0) { // if number is negative, add sign bit in front
            RetVal |= 0x1<<9;
        }
    } 
	else if ((Digits == 3) && (Power == 1)) 
	{ // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
        if (AbsNumber < 1000) 
		{
            RetVal = AbsNumber<<1;
        } 
		else if (AbsNumber < 10240) {
            RetVal = ((uint16_t)roundf(AbsNumber * 0.1f)<<1)|0x1;
        } 
		else 
		{ // transmit max possible value (0x3FF x 10^1 = 10240)
            RetVal = 0x7FF;
        }
        if (Number < 0) 
		{ // if number is negative, add sign bit in front
            RetVal |= 0x1<<11;
        }
    } 
	else if ((Digits == 3) && (Power == 2)) 
	{ // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
        if (AbsNumber < 1000) 
		{
            RetVal = AbsNumber<<2;
        } 
		else if (AbsNumber < 10000) 
		{
            RetVal = ((uint16_t)round(AbsNumber * 0.1f)<<2)|0x1;
        } else if (AbsNumber < 100000) 
		{
            RetVal = ((uint16_t)round(AbsNumber * 0.01f)<<2)|0x2;
        } 
		else if (AbsNumber < 1024000) {
            RetVal = ((uint16_t)round(AbsNumber * 0.001f)<<2)|0x3;
        } 
		else 
		{ // transmit max possible value (0x3FF x 10^3 = 127000)
            RetVal = 0xFFF;
        }
        if (Number < 0) 
		{ // if number is negative, add sign bit in front
            RetVal |= 0x1<<12;
        }
    }
    return RetVal;
}

/*
 * format the decimal latitude/longitude to the required degrees/minutes
 * for FrSky D and SPort protocols
 */
// float CFrskyTelemetry::format_gps(float dec)
// {
//     uint8_t dm_deg = (uint8_t) dec;
//     return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
// }



uint8_t CFrskyTelemetry::GetMode(NavigationState_t State)
{
	if(Config.m_RunningFlags.PANIC_MODE_TRIGGERED)
		return(0);
	switch(State)
	{
		case eControlByRc:
			return(1);				// Manual 
		case eStartNavigation:
			return(17);				// Initializing
		case eOnRouteToNextWapoint:
			return(4);				// Steering
//		case eBatteryReturnToHome:
//			return(13);				// SmartRTL
		case eReturnHome:
			return(12);				// Rtl
		case eNoGpsLock:
			return(0);
		case ePositionHold:
			return(5);				// Hold
		default:
			return(10);	
	}

	return(0);
}

/*
transmitter script
-- rover modes
flightModes[0]=""
flightModes[1]="Manual"
flightModes[2]="Acro"
flightModes[3]=""
flightModes[4]="Steering"
flightModes[5]="Hold"
flightModes[6]="Loiter"
flightModes[7]="Follow"
flightModes[8]="Simple"
flightModes[9]=""
flightModes[10]=""
flightModes[11]="Auto"
flightModes[12]="RTL"
flightModes[13]="SmartRTL"
flightModes[14]=""
flightModes[15]=""
flightModes[16]="Guided"
flightModes[17]="Initializing"
flightModes[18]=""
flightModes[19]=""
flightModes[20]=""
flightModes[21]=""
flightModes[22]=""
*/
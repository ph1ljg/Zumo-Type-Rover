/* 
* CUblox.cpp
*
* Created: 20/11/2015 11:27:44
* Author: Phil2
*/

#include "includes.h"

volatile uint8_t GpsDegug1[10];
volatile bool fred = false;


// default constructor
CUblox::CUblox()
{
	m_MesageId				= 0;
	m_MessageClass			= 0;
	m_CfgNeedsSave			= false;
	m_CfgSaved				= false,
	m_UbloxPort				= 255;
	m_UnconfiguredMessages	= CONFIG_ALL;
	m_HardwareGeneration	= 0;
	NextFix					= GPS_NO_FIX;
	m_DisableCounter		= 0;
	m_ExtensionSize			= 0; // 250;	
	m_NewPosition			= false;
	m_NoReceivedHdop		= true;
	m_NewSpeed				= false;

} //CUblox

// default destructor
CUblox::~CUblox()
{
} //~CUblox

bool CUblox::Init(CUart * Uart)
{
	m_Uart = Uart;
//	uint8_t Char;
	ConfigureStart();
	Core.delay(1000);
	m_Uart->Init(38400);
	return(true);
}



bool CUblox::GetNewFrame(unsigned char data)
{
	// State machine state
//	static uint8_t		MessageClass;
	static uint8_t		ChecksumA;				// Packet checksum accumulators
	static uint8_t		ChecksumB;
	static uint16_t		PayloadCounter;
	static enum{Pre1,Pre2,GetClass,GetId,GetLengthLSB,GetLengthMSB,,GetData,CheckChksumA,CheckChksumB}State = Pre1;
	bool parsed = false;

Reset:
	switch(State)
	{
	case Pre1:
		if(PREAMBLE1 == data)
		{
			GpsDegug1[1] = data;
			ChecksumA		=0;
			ChecksumB		=0;
			m_PayloadLength	=0;
			PayloadCounter	=0;
			State = Pre2;
		}
		break;
	case Pre2:
		State = Pre1;
		if (PREAMBLE2 == data)
		{
			GpsDegug1[2] = data;
			State = GetClass;
		}
		break;
	case GetClass:
		m_MessageClass = data;
		if(data >0x0f)
			State = Pre1;
		if(data == 0x0A)
			State = Pre1;
		if(data == 0x06)
			GpsDegug1[3] =data;
		ChecksumB = ChecksumA = data;			// reset the checksum accumulators
		State = GetId;
		break;
	case GetId:
		ChecksumB += (ChecksumA += data);			// checksum byte
		m_MesageId = data;
		State = GetLengthLSB;
		GpsDegug1[4] = data;
		break;
	case GetLengthLSB:
		ChecksumB += (ChecksumA += data);			// checksum byte
		m_PayloadLength = data;						// payload length low byte
		State = GetLengthMSB;
		break;
	case GetLengthMSB:
		State = Pre1;
		ChecksumB += (ChecksumA += data);			// checksum byte
		m_PayloadLength += (uint16_t)(data<<8);		// Payload High Byte
		if(m_MessageClass == 0x0A && m_MesageId == 0x04)
		{
			if (m_PayloadLength > sizeof(GpsRecieveBuffer)+m_ExtensionSize)		// version extension can be large 
				goto Reset;														// payload bigger then Payload Length is noise
		}
		else
		{							
			if (m_PayloadLength > sizeof(GpsRecieveBuffer))
				goto Reset;														// payload bigger then Payload Length is noise
		}
		if (m_PayloadLength == 0)
			goto Reset;
		State = GetData;
		break;
	case GetData:									// Receive message data
		ChecksumB += (ChecksumA += data);			// checksum byte
		if (PayloadCounter < sizeof(GpsRecieveBuffer))
			GpsRecieveBuffer[PayloadCounter] = data;

		if (++PayloadCounter >= m_PayloadLength)
			State = CheckChksumA;
		break;
	case CheckChksumA:
		if (ChecksumA != data) 
		{
			State = Pre1;						// bad checksum
			goto Reset;
		}
		State = CheckChksumB;
		break;
	case CheckChksumB:
		State = Pre1;
		if (ChecksumB != data)
		{
			goto Reset;
		}
		if (Parse())
			parsed = true;
		break;
	} //end switch
	return parsed;
}






bool CUblox::Parse()
{
	if(m_MessageClass == CLASS_ACK)
	{
		HandleACk();
		return(false);
	}
	 if (m_MessageClass == CLASS_CFG) 
	{
		HandleConfig();
		return(false);
	}
	if (m_MessageClass == CLASS_MON)
	{ 
		HandleMon();
		return(false);
	}
    if (m_MessageClass != CLASS_NAV) // below ids fro class NAV
	{
		    UnexpectedMessage();
		    return false;
	 }

	
	switch (m_MesageId)
	{
		case MSG_POSLLH:
			Gps.m_GpsReadings.PosllhTime				= GpsRecieveBuffer.posllh.time;
			Gps.m_GpsPositionReadings.nLongitude		= GpsRecieveBuffer.posllh.longitude;
			Gps.m_GpsPositionReadings.nLatitude			= GpsRecieveBuffer.posllh.latitude;

			Gps.m_GpsPositionReadings.Longitude			= (float)GpsRecieveBuffer.posllh.longitude/10000000.0;
			Gps.m_GpsPositionReadings.Latitude			= (float)GpsRecieveBuffer.posllh.latitude/10000000.0;

			if(GpsRecieveBuffer.posllh.altitude_msl <0)
				Gps.m_GpsReadings.Altitude = 0;
			else	
				Gps.m_GpsReadings.Altitude  				= GpsRecieveBuffer.posllh.altitude_msl / 10 ;// /100;      //alt in m
			m_NewPosition								= true;
			Gps.m_GpsReadings.horizontal_accuracy		= GpsRecieveBuffer.posllh.horizontal_accuracy*1.0e-3f;
			Gps.m_GpsReadings.vertical_accuracy			= GpsRecieveBuffer.posllh.vertical_accuracy*1.0e-3f;
			Gps.m_GpsReadings.have_horizontal_accuracy	= true;
			Gps.m_GpsReadings.have_vertical_accuracy	= true;
			break;
		case MSG_STATUS:
			if (GpsRecieveBuffer.status.fix_status >NAV_STATUS_NO_FIX) 
			{
				if( (GpsRecieveBuffer.status.fix_type == NAV_STATUS__FIX_3D) && (GpsRecieveBuffer.status.fix_status & NAV_STATUS_DGPS_USED)) 
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_DGPS;

				else if( GpsRecieveBuffer.status.fix_type == NAV_STATUS__FIX_3D) 
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_3D;

				else if (GpsRecieveBuffer.status.fix_type == NAV_STATUS__FIX_2D) 
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_3D;

				else if (GpsRecieveBuffer.status.fix_type == NAV_STATUS__FIX_TIME)
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_TIME;

				else
					Gps.m_GpsReadings.GpsFixType = GPS_NO_FIX;
	        }
			else
				Gps.m_GpsReadings.GpsFixType = GPS_NO_FIX;
				
			Gps.m_GpsReadings.DgpsValid		= GpsRecieveBuffer.status.differential_status & NAV_STATUS_DGPS_USED;
			break;
		case MSG_DOP:
			m_NoReceivedHdop = false;
			Gps.m_GpsReadings.Hdop			= GpsRecieveBuffer.dop.hDOP;
			Gps.m_GpsReadings.vdop			= GpsRecieveBuffer.dop.vDOP;
			break;
		case MSG_SOL:
			if (GpsRecieveBuffer.solution.fix_status & NAV_SOL_FIX_IN_LIMITS) 
			{
				if( (GpsRecieveBuffer.solution.fix_type == NAV_SOL__FIX_3D) &&	(GpsRecieveBuffer.solution.fix_status & NAV_SOL_DGPS_USED)) 
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_DGPS;

				else if( GpsRecieveBuffer.solution.fix_type == NAV_SOL__FIX_3D) 
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_3D;

				else if (GpsRecieveBuffer.solution.fix_type == NAV_SOL__FIX_2D) 
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_3D;
	
				else if (GpsRecieveBuffer.solution.fix_type == NAV_SOL__FIX_TIME)
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_TIME;

				else
					Gps.m_GpsReadings.GpsFixType = GPS_NO_FIX;
			}
			else
				Gps.m_GpsReadings.GpsFixType = GPS_NO_FIX;

			if(m_NoReceivedHdop) 
				Gps.m_GpsReadings.Hdop = GpsRecieveBuffer.solution.position_DOP;

			Gps.m_GpsReadings.NumSats    = GpsRecieveBuffer.solution.satellites;
			if (Gps.m_GpsReadings.GpsFixType >= GPS_OK_FIX_3D && Gps.m_GpsReadings.GpsFixType != GPS_OK_FIX_TIME) 
			{
				Gps.m_GpsReadings.LastGpsTimeMs = Core.millis();
				if (Gps.m_GpsReadings.time_week == GpsRecieveBuffer.solution.week &&  Gps.m_GpsReadings.time_week_ms + 200 == GpsRecieveBuffer.solution.time) 
				{
					// we got a 5Hz update. This relies on the way that uBlox gives timestamps that are always multiples of 200 for 5Hz
					m_LastUpdateTime = Gps.m_GpsReadings.LastGpsTimeMs;
				}
				Gps.m_GpsReadings.time_week_ms    = GpsRecieveBuffer.solution.time;
				Gps.m_GpsReadings.time_week       = GpsRecieveBuffer.solution.week;
			}
			break;
		case MSG_PVT:
			Gps.m_GpsPositionReadings.nLongitude		= GpsRecieveBuffer.Pvt.lon;
			Gps.m_GpsPositionReadings.nLatitude			= GpsRecieveBuffer.Pvt.lat;

			Gps.m_GpsPositionReadings.Longitude			= (float)GpsRecieveBuffer.Pvt.lon/10000000.0;
			Gps.m_GpsPositionReadings.Latitude			= (float)GpsRecieveBuffer.Pvt.lat/10000000.0;
			Gps.m_GpsReadings.Altitude				=	GpsRecieveBuffer.Pvt.h_msl /10;

			switch (GpsRecieveBuffer.Pvt.fix_type)
			{
			case 0:
				Gps.m_GpsReadings.GpsFixType = GPS_NO_FIX;
				break;
			case 1:
				Gps.m_GpsReadings.GpsFixType = GPS_NO_FIX;
				break;
			case 2:
				Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_3D;
				break;
			case 3:
				Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_3D;
				if (GpsRecieveBuffer.Pvt.flags & 0b00000010)  // diffsoln
						Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_DGPS;
				if (GpsRecieveBuffer.Pvt.flags & 0b01000000)  // carrsoln - float
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_3D_RTK_FLOAT;
				if (GpsRecieveBuffer.Pvt.flags & 0b10000000)  // carrsoln - fixed
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_3D_RTK_FIXED;
				break;
			case 4:
					Gps.m_GpsReadings.GpsFixType = GPS_OK_FIX_3D;
				break;
			case 5:
				Gps.m_GpsReadings.GpsFixType = GPS_NO_FIX;
				break;
			default:
				Gps.m_GpsReadings.GpsFixType = GPS_NO_FIX;
				break;
			}
			m_NewPosition = true;
			Gps.m_GpsReadings.horizontal_accuracy = GpsRecieveBuffer.Pvt.h_acc*1.0e-3f;
			Gps.m_GpsReadings.vertical_accuracy = GpsRecieveBuffer.Pvt.v_acc*1.0e-3f;
			Gps.m_GpsReadings.have_horizontal_accuracy = true;
			Gps.m_GpsReadings.have_vertical_accuracy = true;
			Gps.m_GpsReadings.NumSats    = GpsRecieveBuffer.Pvt.num_sv;
			m_LastVelTime = GpsRecieveBuffer.Pvt.itow;
			Gps.m_GpsReadings.GroundSpeed = GpsRecieveBuffer.Pvt.gspeed *0.001f;          // m/s
			Gps.m_GpsReadings.GroundCourse = wrap_360(GpsRecieveBuffer.Pvt.head_mot * 1.0e-5f);       // Heading 2D deg * 100000
			Gps.m_GpsReadings.HaveVerticalVelocity = true;
			Gps.m_GpsReadings.velocity.x = GpsRecieveBuffer.Pvt.velN * 0.001f;
			Gps.m_GpsReadings.velocity.y = GpsRecieveBuffer.Pvt.velE * 0.001f;
			Gps.m_GpsReadings.velocity.z = GpsRecieveBuffer.Pvt.velD * 0.001f;
			Gps.m_GpsReadings.have_speed_accuracy = true;
			Gps.m_GpsReadings.SpeedAccuracy = GpsRecieveBuffer.Pvt.s_acc*0.001f;
			m_NewSpeed = true;
			if(m_NoReceivedHdop) 
			{
				Gps.m_GpsReadings.Hdop        = GpsRecieveBuffer.Pvt.p_dop;
				Gps.m_GpsReadings.vdop        = GpsRecieveBuffer.Pvt.p_dop;
			}
			Gps.m_GpsReadings.LastGpsTimeMs = Core.millis();
			break;			
		 case MSG_VELNED:
			m_LastVelTime      = GpsRecieveBuffer.velned.time;
			Gps.m_GpsReadings.HaveVerticalVelocity = true;
			Gps.m_GpsReadings.velocity.x = GpsRecieveBuffer.velned.ned_north * 0.01f;
			Gps.m_GpsReadings.velocity.y = GpsRecieveBuffer.velned.ned_east * 0.01f;
			Gps.m_GpsReadings.velocity.z = GpsRecieveBuffer.velned.ned_down * 0.01f;
			Gps.m_GpsReadings.GroundCourse = wrap_360(degrees(atan2f(Gps.m_GpsReadings.velocity.y, Gps.m_GpsReadings.velocity.x)));
			Gps.m_GpsReadings.GroundSpeed = norm(Gps.m_GpsReadings.velocity.y, Gps.m_GpsReadings.velocity.x);
			Gps.m_GpsReadings.have_speed_accuracy = true;
			Gps.m_GpsReadings.SpeedAccuracy = GpsRecieveBuffer.velned.speed_accuracy*0.01f;
			Gps.m_GpsReadings.HeadingAcu	= GpsRecieveBuffer.velned.heading_accuracy/10000;			// rescaled to deg * 10
			m_NewSpeed = true;
			break;
		case MSG_NAV_SVINFO:
			m_HardwareGeneration = GpsRecieveBuffer.svinfo_header.globalFlags & 0x07;		//HardwareGenerationMask 0x07
			switch (m_HardwareGeneration) 
			{
			case UBLOX_5:
			case UBLOX_6:
//				DebugDisplay.Printf("Wrong Ublox Hardware Version%u\n", m_HardwareGeneration);
				break;
			}
			m_UnconfiguredMessages &= ~CONFIG_CHIP_NO;
			ConfigureMessageRate(CLASS_NAV, MSG_NAV_SVINFO, 0);				// We don't need that anymore 
			break; 
		case MSG_TIMEUTC:
			Gps.m_GpsTimeValues.year		=	GpsRecieveBuffer.ubxNavTimeUtc.year;
			Gps.m_GpsTimeValues.month	=	GpsRecieveBuffer.ubxNavTimeUtc.month;
			Gps.m_GpsTimeValues.day		=	GpsRecieveBuffer.ubxNavTimeUtc.day;
			Gps.m_GpsTimeValues.hour		=	GpsRecieveBuffer.ubxNavTimeUtc.hour;
			Gps.m_GpsTimeValues.minute	=	GpsRecieveBuffer.ubxNavTimeUtc.min;
			Gps.m_GpsTimeValues.seconds	=	GpsRecieveBuffer.ubxNavTimeUtc.sec;
			Gps.m_GpsTimeValues.milliseconds =	GpsRecieveBuffer.ubxNavTimeUtc.MillisecondTimeOfWeek;
			m_UnconfiguredMessages &= ~CONFIG_TIMEUTC;
			m_NewSpeed = true;
			break;
		default:
			return false;
	}

	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (m_NewPosition && m_NewSpeed)
	{
		m_NewSpeed = m_NewPosition = false;
		return true;
	}
	return false;
}




void CUblox::HandleConfig()
{
	switch(m_MesageId) 
	{
	case  MSG_CFG_NAV_SETTINGS:
		Debug("Got settings %u min_elev %d drLimit %u\n",(unsigned)GpsRecieveBuffer.nav_settings.dynModel,(int)GpsRecieveBuffer.nav_settings.minElev,(unsigned)GpsRecieveBuffer.nav_settings.drLimit);
		GpsRecieveBuffer.nav_settings.mask = 0;
		if (Gps.m_Navfilter != GPS_ENGINE_NONE && GpsRecieveBuffer.nav_settings.dynModel != Gps.m_Navfilter) 
		{
			Debug("Changing engine setting from %u to %u\n",(unsigned)GpsRecieveBuffer.nav_settings.dynModel, (unsigned)Gps.m_Navfilter);	
			GpsRecieveBuffer.nav_settings.dynModel = Gps.m_Navfilter;			// we've received the current nav settings, change the engine settings and send them back
			GpsRecieveBuffer.nav_settings.mask |= 1;
		}
		if (Gps.m_MinElevation != -100 &&		GpsRecieveBuffer.nav_settings.minElev !=Gps.m_MinElevation) 
		{
			Debug("Changing min elevation to %d\n", (int)Gps.m_MinElevation);
			GpsRecieveBuffer.nav_settings.minElev = Gps.m_MinElevation;
			GpsRecieveBuffer.nav_settings.mask |= 2;
		}
		if (GpsRecieveBuffer.nav_settings.mask != 0) 
		{
			SendMessage(CLASS_CFG, MSG_CFG_NAV_SETTINGS, &GpsRecieveBuffer.nav_settings,	sizeof(GpsRecieveBuffer.nav_settings));
			m_UnconfiguredMessages |= CONFIG_NAV_SETTINGS;
			m_CfgNeedsSave = true;
		} 
		else 
		{
			m_UnconfiguredMessages &= ~CONFIG_NAV_SETTINGS;
		}
		break;
	case MSG_CFG_GNSS:
		if (Gps.m_GnssMode != 0) 
		{
			ubx_cfg_gnss_t start_gnss = GpsRecieveBuffer.gnss;
			uint8_t gnssCount = 0;
			Debug("Got GNSS Settings %u %u %u %u:\n",(unsigned)GpsRecieveBuffer.gnss.msgVer,(unsigned)GpsRecieveBuffer.gnss.numTrkChHw,
				(unsigned)GpsRecieveBuffer.gnss.numTrkChUse,(unsigned)GpsRecieveBuffer.gnss.numConfigBlocks);
			for(int i = 0; i < GpsRecieveBuffer.gnss.numConfigBlocks; i++) 
			{
				Debug("  %u %u %u 0x%08x\n",(unsigned)GpsRecieveBuffer.gnss.configBlock[i].gnssId,(unsigned)GpsRecieveBuffer.gnss.configBlock[i].resTrkCh,
					(unsigned)GpsRecieveBuffer.gnss.configBlock[i].maxTrkCh,	(unsigned)GpsRecieveBuffer.gnss.configBlock[i].flags);
			}

			for(int i = 0; i < UBLOX_MAX_GNSS_CONFIG_BLOCKS; i++) 
			{
				if((Gps.m_GnssMode & (1 << i)) && i != GNSS_SBAS) 
				{
					gnssCount++;
				}
			}

			for(int i = 0; i < GpsRecieveBuffer.gnss.numConfigBlocks; i++) 
			{
				// Reserve an equal portion of channels for all enabled systems
				if(Gps.m_GnssMode & (1 << GpsRecieveBuffer.gnss.configBlock[i].gnssId)) 
				{
					if(GNSS_SBAS !=GpsRecieveBuffer.gnss.configBlock[i].gnssId) 
					{
						GpsRecieveBuffer.gnss.configBlock[i].resTrkCh = (GpsRecieveBuffer.gnss.numTrkChHw - 3) / (gnssCount * 2);
						GpsRecieveBuffer.gnss.configBlock[i].maxTrkCh = GpsRecieveBuffer.gnss.numTrkChHw;
					}
					else
					{
						GpsRecieveBuffer.gnss.configBlock[i].resTrkCh = 1;
						GpsRecieveBuffer.gnss.configBlock[i].maxTrkCh = 3;
					}
					GpsRecieveBuffer.gnss.configBlock[i].flags = GpsRecieveBuffer.gnss.configBlock[i].flags | 0x00000001;
				}
				else
				      
				{
					GpsRecieveBuffer.gnss.configBlock[i].resTrkCh = 0;
					GpsRecieveBuffer.gnss.configBlock[i].maxTrkCh = 0;
					GpsRecieveBuffer.gnss.configBlock[i].flags = GpsRecieveBuffer.gnss.configBlock[i].flags & 0xFFFFFFFE;
				}
			}
			if (!memcmp(&start_gnss, &GpsRecieveBuffer.gnss, sizeof(start_gnss))) 
			{
				SendMessage(CLASS_CFG, MSG_CFG_GNSS, &GpsRecieveBuffer.gnss, 4 + (8 * GpsRecieveBuffer.gnss.numConfigBlocks));
				m_UnconfiguredMessages |= CONFIG_GNSS;
				m_CfgNeedsSave = true;
			} 
			else 
			{
				m_UnconfiguredMessages &= ~CONFIG_GNSS;
			}
		} 
		else 
		{
			m_UnconfiguredMessages &= ~CONFIG_GNSS;
		}
		break;
	case MSG_CFG_SBAS:
		if(m_UnconfiguredMessages & CONFIG_SBAS)
		{
			if (Gps.m_SbasMode != 2) 
			{
				Debug("Got SBAS settings %u %u %u 0x%x 0x%x\n",	(unsigned)GpsRecieveBuffer.sbas.mode,(unsigned)GpsRecieveBuffer.sbas.usage,(unsigned)GpsRecieveBuffer.sbas.maxSBAS,
				(unsigned)GpsRecieveBuffer.sbas.scanmode2,(unsigned)GpsRecieveBuffer.sbas.scanmode1);
			
				if (GpsRecieveBuffer.sbas.mode != Gps.m_SbasMode) 
				{
					GpsRecieveBuffer.sbas.mode = Gps.m_SbasMode;
					SendMessage(CLASS_CFG, MSG_CFG_SBAS,&GpsRecieveBuffer.sbas,	sizeof(GpsRecieveBuffer.sbas));
					m_UnconfiguredMessages |= CONFIG_SBAS;
					m_CfgNeedsSave = true;
				}
				else 
					m_UnconfiguredMessages &= ~CONFIG_SBAS;
			} 
			else 
				m_UnconfiguredMessages &= ~CONFIG_SBAS;
		}
		break;
	case MSG_CFG_MSG:
		if(m_PayloadLength == sizeof(UbxCfgMsgRate6_t)) 
		{
			// can't verify the setting without knowing the port request the port again
			if(m_UbloxPort >= UBLOX_MAX_PORTS) 
			{
				RequestPort();
				break;
			}
			VerifyRate(GpsRecieveBuffer.msg_rate_6.msg_class, GpsRecieveBuffer.msg_rate_6.msg_id,	GpsRecieveBuffer.msg_rate_6.rates[m_UbloxPort]);
		} 
		else 
		{
			VerifyRate(GpsRecieveBuffer.msg_rate.msg_class, GpsRecieveBuffer.msg_rate.msg_id,	GpsRecieveBuffer.msg_rate.rate);
		}
		break;
	case MSG_CFG_PRT:
		m_UbloxPort = GpsRecieveBuffer.prt.portID;
		break;;
	case MSG_CFG_RATE:
		if(m_UnconfiguredMessages & CONFIG_RATE_NAV)
		{
			if(GpsRecieveBuffer.nav_rate.measure_rate_ms != MEASURE_RATE ||	GpsRecieveBuffer.nav_rate.nav_rate != 1 || GpsRecieveBuffer.nav_rate.timeref != 0) 
			{
				ConfigNavRate();
				m_UnconfiguredMessages |= CONFIG_RATE_NAV;
				m_CfgNeedsSave = true;
			} 
			else 
				m_UnconfiguredMessages &= ~CONFIG_RATE_NAV;
		}
	}
	      
}




void CUblox::HandleACk()
{
	if(m_MesageId == MSG_ACK_ACK) 
	{
		switch(GpsRecieveBuffer.ack.clsID) 
		{
		case CLASS_CFG:
			switch(GpsRecieveBuffer.ack.msgID) 
			{
			case MSG_CFG_CFG:
				m_CfgSaved = true;
				m_CfgNeedsSave = false;
				break;
			case MSG_CFG_GNSS:
				m_UnconfiguredMessages &= ~CONFIG_GNSS;
				break;
			case MSG_CFG_MSG:
				// There is no way to know what MSG config was ack'ed, assume it was the last
				break;
			case MSG_CFG_NAV_SETTINGS:
				m_UnconfiguredMessages &= ~CONFIG_NAV_SETTINGS;
				break;
			case MSG_CFG_RATE:
				m_UnconfiguredMessages &= ~CONFIG_RATE_NAV;
				break;
			case MSG_CFG_SBAS:
				m_UnconfiguredMessages &= ~CONFIG_SBAS;
				break;
			}
			break;
		}
	}
}


bool CUblox::HandleMon()
{
	switch(m_MesageId) 
	{
	case MSG_MON_VER:
		m_UnconfiguredMessages &= ~CONFIG_VERSION;
//		DebugDisplay.GpsVersion();
		break;
	default:
		UnexpectedMessage();
    }
    return false;
        			
}

void CUblox::UnexpectedMessage(void)
{
	Debug("Unexpected message 0x%02x 0x%02x", (unsigned)m_MessageClass, (unsigned)m_MesageId);
	if (++m_DisableCounter == 0) 
	{
		// disable future sends of this message id, but only do this every 256 messages, as some message types can't be disabled 
		Debug("Disabling message 0x%02x 0x%02x", (unsigned)m_MessageClass, (unsigned)m_MesageId);
		ConfigureMessageRate(m_MessageClass, m_MesageId, 0);
	}
}



void CUblox::UpdateChecksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b)
{
	while (len--)
	{
		ck_a += *data;
		ck_b += ck_a;
		data++;
	}
}


void CUblox::RequestNextConfig(void)
{

	// Ensure there is enough space for the largest possible outgoing message
	if (m_Uart->TxAvailable() < (int16_t)(sizeof( ubx_header_t)+sizeof(ubx_cfg_nav_rate_t)+2)) 
		return;			// not enough space - do it next time

	Debug("Un configured messages: %d Current message: %d\n", m_UnconfiguredMessages, m_NextMessage);

	switch (m_NextMessage++) 
	{
	case STEP_RATE_NAV:
		if(m_UnconfiguredMessages &CONFIG_RATE_NAV)
		{
			ConfigNavRate();
			break;
		}
		else
			m_NextMessage++;
	case STEP_RATE_POSLLH:
		if(m_UnconfiguredMessages & CONFIG_RATE_POSLLH)
		{
			if(!ConfigureMessageRate(CLASS_NAV, MSG_POSLLH, RATE_POSLLH)) 
				m_NextMessage--;
			break;
		}
		else
			m_NextMessage++;
	case STEP_RATE_VELNED:
		if(m_UnconfiguredMessages & CONFIG_RATE_VELNED)
		{
			if(!ConfigureMessageRate(CLASS_NAV, MSG_VELNED, RATE_VELNED)) 
				m_NextMessage--;
			break;
		}
		else
			m_NextMessage++;
	case STEP_PORT:
		if(m_UbloxPort >= UBLOX_MAX_PORTS)  
			RequestPort();
		break;
	case STEP_POLL_SVINFO:
		// not required once we know what generation we are on
		if(m_HardwareGeneration == 0) 
		{
			SendMessage(CLASS_NAV, MSG_NAV_SVINFO, 0, 0);
			break;
		}
		else
			m_NextMessage++;
	case STEP_POLL_SBAS:
		if(m_UnconfiguredMessages & CONFIG_SBAS)
		{
			SendMessage(CLASS_CFG, MSG_CFG_SBAS, NULL, 0);
			break;
		}
		else
			m_NextMessage++;
	case STEP_POLL_NAV:
		if(m_UnconfiguredMessages & CONFIG_NAV_SETTINGS)
		{
			SendMessage(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
			break;
		}
		else
			m_NextMessage++;
	case STEP_POLL_GNSS:
		if(m_UnconfiguredMessages & CONFIG_GNSS)
		{
			SendMessage(CLASS_CFG, MSG_CFG_GNSS, NULL, 0);
			break;
		}
		else
			m_NextMessage++;
	case STEP_NAV_RATE:
		if(m_UnconfiguredMessages & CONFIG_RATE_NAV)
			RequestNavigationRate();
		break;
	case STEP_POSLLH:
		if(m_UnconfiguredMessages & CONFIG_RATE_POSLLH)
		{
			if(!RequestMessageRate(CLASS_NAV, MSG_POSLLH)) 
				m_NextMessage--;
			break;
		}
		else
			m_NextMessage++;
	case STEP_STATUS:
		if(m_UnconfiguredMessages & CONFIG_RATE_STATUS)
		{
			if(!RequestMessageRate(CLASS_NAV, MSG_STATUS)) 
				m_NextMessage--;
			break;
		}
		else
			m_NextMessage++;
	case STEP_SOL:
		if(m_UnconfiguredMessages & CONFIG_RATE_SOL)
		{
			if(!RequestMessageRate(CLASS_NAV, MSG_SOL)) 
				m_NextMessage--;
			break;
		}
		else
			m_NextMessage++;
	case STEP_VELNED:
		if(m_UnconfiguredMessages & CONFIG_RATE_VELNED)
		{
			if(!RequestMessageRate(CLASS_NAV, MSG_VELNED)) 
				m_NextMessage--;
			break;
		}
		else
			m_NextMessage++;
	case STEP_DOP:
		if(m_UnconfiguredMessages & CONFIG_RATE_DOP)
		{
			if(! RequestMessageRate(CLASS_NAV, MSG_DOP)) 
				m_NextMessage--;
			break;
		}
		else
			m_NextMessage++;
	case MSG_TIMEUTC:
		if(m_UnconfiguredMessages & CONFIG_TIMEUTC)
		{
			if(!ConfigureMessageRate(CLASS_NAV, MSG_TIMEUTC, RATE_TIMEUTC))
				m_NextMessage--;
			break;
		}
		else
			m_NextMessage++;
	case STEP_SW_VERSION:
		if(m_UnconfiguredMessages & CONFIG_VERSION)
			SendMessage(CLASS_MON, MSG_MON_VER, NULL, 0);
		m_NextMessage = STEP_PORT;								// no need to send the initial rates, move to checking only
		break;
	default:
		m_NextMessage = STEP_RATE_NAV;							// this case should never be reached, do a full reset if it is hit
		break;
	}
}

void CUblox::ConfigNavRate(void)
{
	ubx_cfg_nav_rate_t msg;
	msg.measure_rate_ms = MEASURE_RATE;
	msg.nav_rate        = 1;
	msg.timeref         = 0;     // UTC time
	SendMessage(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}

// configure a UBlox GPS for the given message rate for a specific message class and msg_id
bool CUblox::ConfigureMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    if (m_Uart->TxAvailable() < (int16_t)(sizeof( ubx_header_t)+sizeof(ubx_cfg_msg_rate_t)+2)) 
        return false;

    ubx_cfg_msg_rate_t msg;
    msg.msg_class = msg_class;
    msg.msg_id    = msg_id;
    msg.rate          = rate;
    SendMessage(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
    return true;
}


// Requests the ublox driver to identify what port we are using to communicate
void CUblox::RequestPort(void)
{
	if (m_Uart->TxAvailable() < (int16_t)(sizeof(ubx_header_t)+2)) 
	{
		// not enough space - do it next time
		return;
	}
	SendMessage(CLASS_CFG, MSG_CFG_PRT, NULL, 0);
}


//=======================================================================================
// request the current navigation solution rate
//=======================================================================================
void CUblox::RequestNavigationRate(void)
{
	SendMessage(CLASS_CFG, MSG_CFG_RATE, 0, 0);
}


void CUblox::ConfigureRate(void)
{
	UbxCfgNavRate_t msg;
	msg.measure_rate_ms = MEASURE_RATE;
	msg.nav_rate        = 1;
	msg.timeref         = 0;     // UTC time
	SendMessage(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}

//=======================================================================================
// save gps configurations to non-volatile memory sent until the call of
// this message
//=======================================================================================
void CUblox::SaveCfg()
{
    UbxCfgCfg_t save_cfg;
    save_cfg.clearMask = 0;
    save_cfg.saveMask = SAVE_CFG_ALL;
    save_cfg.loadMask = 0;
    SendMessage(CLASS_CFG, MSG_CFG_CFG, &save_cfg, sizeof(save_cfg));
    m_LastCfgSentTime = Core.millis();
    m_NumCfgSaveTries++;
}


//=======================================================================================
//  requests the given message rate for a specific message class//  and msg_id
//  returns true if it sent the request, 
//=======================================================================================
bool CUblox::RequestMessageRate(uint8_t msg_class, uint8_t msg_id)
{
	ubx_cfg_msg_t msg;
	msg.msg_class = msg_class;
	msg.msg_id    = msg_id;
	SendMessage(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
	return true;

}

void CUblox::SendMessage(uint8_t msg_class, uint8_t msg_id, void *msg, uint16_t Size)
{
	UbxHeader_t header;
	uint8_t ck_a=0, ck_b=0;
	header.preamble1 = PREAMBLE1;
	header.preamble2 = PREAMBLE2;
	header.msg_class = msg_class;
	header.msg_id    = msg_id;
	header.length    = Size;

	UpdateChecksum((uint8_t *)&header.msg_class, sizeof(UbxHeader_t)-2, ck_a, ck_b);
	UpdateChecksum((uint8_t *)msg, Size, ck_a, ck_b);
	m_Uart->ClearTxBuffer();
	m_Uart->WriteTxBuffer ((uint8_t*)&header,sizeof(header),false);
	m_Uart->WriteTxBuffer ((uint8_t*)msg,Size,false);
	m_Uart->WriteTxBuffer ((uint8_t*)&ck_a, 1,false);
	m_Uart->WriteTxBuffer ((uint8_t*)&ck_b, 1);


}

void CUblox::VerifyRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate) 
{
	switch(msg_class) 
	{
	case CLASS_NAV:
		switch(msg_id) 
		{
		case MSG_POSLLH:
			if(rate == RATE_POSLLH) 
				m_UnconfiguredMessages &= ~CONFIG_RATE_POSLLH;
			else 
			{
				ConfigureMessageRate(msg_class, msg_id, RATE_POSLLH);
				m_UnconfiguredMessages |= CONFIG_RATE_POSLLH;
				m_CfgNeedsSave = true;
			}
			break;
		case MSG_STATUS:
			if(rate == RATE_STATUS) 
				m_UnconfiguredMessages &= ~CONFIG_RATE_STATUS;
			else 
			{
				ConfigureMessageRate(msg_class, msg_id, RATE_STATUS);
				m_UnconfiguredMessages |= CONFIG_RATE_STATUS;
				m_CfgNeedsSave = true;
			}
			break;
		case MSG_SOL:
			if(rate == RATE_SOL) 
				m_UnconfiguredMessages &= ~CONFIG_RATE_SOL;
			else 
			{
				ConfigureMessageRate(msg_class, msg_id, RATE_SOL);
				m_UnconfiguredMessages |= CONFIG_RATE_SOL;
				m_CfgNeedsSave = true;
			}
			break;
		case MSG_VELNED:
			if(rate == RATE_VELNED) 
				m_UnconfiguredMessages &= ~CONFIG_RATE_VELNED;
			else 
			{
				ConfigureMessageRate(msg_class, msg_id, RATE_VELNED);
				m_UnconfiguredMessages |= CONFIG_RATE_VELNED;
				m_CfgNeedsSave = true;
			}
			break;
		case MSG_DOP:
			if(rate == RATE_DOP) 
				m_UnconfiguredMessages &= ~CONFIG_RATE_DOP;
			else 
			{
				ConfigureMessageRate(msg_class, msg_id, RATE_DOP);
				m_UnconfiguredMessages |= CONFIG_RATE_DOP;
				m_CfgNeedsSave = true;
			}
			break;
		case MSG_TIMEUTC:
			if(rate == RATE_TIMEUTC)
				m_UnconfiguredMessages &= ~CONFIG_TIMEUTC;
			else
			{
				ConfigureMessageRate(msg_class, msg_id, RATE_TIMEUTC);
				m_UnconfiguredMessages |= CONFIG_TIMEUTC;
				m_CfgNeedsSave = true;
			}
			break;
		}
	break;
	}
}

//const char  Baud115200[]  ={0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0,			//    speed to 115200
//	0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC4, 0x96};


void CUblox::ConfigurePort()
{
	UbxConfigurePort_t UbxConfigurePort;
		UbxConfigurePort.portID			= 0x01;			//Port Identifier Number (= 1 or 2 for UART ports)
		UbxConfigurePort.reserved		= 0x00;			//Reserved
		UbxConfigurePort.txReady		= 0x0000;		//00 00
		UbxConfigurePort.mode			= 0x000008D0;// 0xD0080000;	// 00001000 11010000 bits 7/8 charlen bits 9/10/11 parity 13 stop bits			 ? ?
		UbxConfigurePort.Baud			= 0x0001C200;	//0x00C20100;	// 115200
		UbxConfigurePort.inProtoMask	= 0x0007;		// bit 0 Ubx bit 1 nmea
		UbxConfigurePort.outProtoMask	= 0x0007;		// bit 0 Ubx bit 1 nmea
		UbxConfigurePort.reserved4		= 0x0000;		//	0x00, 0x00
		UbxConfigurePort.reserved5		= 0x0000;		//	0x00, 0x00
		SendMessage(CLASS_CFG, 00, &UbxConfigurePort,	sizeof(UbxConfigurePort_t));

} 


#define UBLOX_SET_BINARY		  "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0003,0001,38400,0*26\r\n"
#define UBLOX_SET_BINARY_RAW_BAUD "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0003,0001,115200,0*1E\r\n"

void CUblox::ConfigureStart()
{
	const char InitialisationString[] = UBLOX_SET_BINARY;
	uint8_t Len = sizeof(InitialisationString);

	m_Uart->WriteTxBuffer((uint8_t*) InitialisationString,Len,true);  // ublox 38400 baud
	m_Uart->ClearRxBuffer();
}












/* 
* COsd.cpp
*
* Created: 08/04/2019 19:17:38
* Author: phil
*/
#include "Includes.h"


volatile uint8_t OsdVal;

CMax7456 Max7456(&Spi,&Uart);

// default constructor
COsd::COsd()
{
	m_Blinker				= 0;
	OneSecTimerSwitch		= 0;
	canswitch				= true;
	ch_raw					= 0;
	ClearOsd				= 0;
	osd_mode				= 0;
	osd_off_switch			= 0;
	converts				= 0;
	converth				= 0;
	spe						= 0;
	high					= 0;
	temps					= 0;
	tempconv				= 1;
	tempconvAdd				= 0;
	distchar				= 0;
	climbchar				= 0;
	tempconv				= 1;
	distconv				= 0;
	wp_target_bearing_rotate_int = 0;
	osd_winddirection		= 0;
	nor_osd_windspeed		= 0;
	m_CurrentPanel			= 0;
	UpdatePanelCords();
	
} //COsd

// default destructor
COsd::~COsd()
{
} //~COsd

void COsd::Init()
{
	Max7456.init();						// Prepare OSD for displaying

	panLogo(); // Display  logo
//	Core.delay(3000);
	m_AllowUpdate = false;
	UpdatePanelCords();
//	DisplayFont();

}

void COsd::Start()
{
	SetPanelState(0);
	Max7456.clear();
	
}

void COsd::Update()
{
	if(m_AllowUpdate)
		writePanels();							// writing enabled panels (check OSD_Panels Tab)

}



void COsd::panLogo()
{
	Max7456.OpenPanel(5,5);
	Max7456.Printf("Mini Rover M4|Version 1.0|Phil Griffin| 2020");
	Max7456.ClosePanel();
	ClearOsd = 1;
}

void COsd::DisplayFont()
{
	int i;
	Max7456.OpenPanel(1,1);
	for(i=0;i<255;i++)
		Max7456.Printf("%c",i);
		
	Max7456.ClosePanel();	
}

void COsd::writePanels()
{
	if(ClearOsd)
	{
		Max7456.clear();
		ClearOsd = false;
	}
		
	if(m_CurrentPanel != 2)
	{
		if((Panel1.PanAlarms.Enable))
			PanAlarms(Panel1.PanAlarms.X, Panel1.PanAlarms.Y); //5x1
	}
	else
	{
		if((Panel3.PanAlarms.Enable))
			PanAlarms(Panel3.PanAlarms.X, Panel3.PanAlarms.Y); //5x1
	}

	switch(m_CurrentPanel)
	{
	case 0:	
		if((Panel1.panPitch.Enable))
			PanPitch(Panel1.panPitch.X, Panel1.panPitch.Y); //5x1

		if(Panel1.panRoll.Enable)	
			PanRoll(Panel1.panRoll.X, Panel1.panRoll.Y); //5x1

		if(Panel1.panBatt.Enable)
			PanBattVolts(Panel1.panBatt.X, Panel1.panBatt.Y); //7x1

		if(Panel1.panVtxVal.Enable)
			PanVtx(Panel1.panVtxVal.X, Panel1.panVtxVal.Y); //7x1

		if(Panel1.panGPSats.Enable)
			panGPSats(Panel1.panGPSats.X, Panel1.panGPSats.Y); //5x1

		if(Panel1.panGPS.Enable)
			panGPS(Panel1.panGPS.X, Panel1.panGPS.Y); //12x3

		if(Panel1.panBatteryPercent.Enable)
			panBatteryPercent(Panel1.panBatteryPercent.X, Panel1.panBatteryPercent.Y,true); //

		if(Panel1.panCOG.Enable)
			panCOG(Panel1.panCOG.X, Panel1.panCOG.Y); //
			
		if(Panel1.panRose.Enable)
			panRose(Panel1.panRose.X, Panel1.panRose.Y);        //13x3
			
		if(Panel1.panHeading.Enable)
			panHeading(Panel1.panHeading.X, Panel1.panHeading.Y); //13x3
			
		if(Panel1.panHomeDis.Enable)
			panHomeDis(Panel1.panHomeDis.X, Panel1.panHomeDis.Y); //13x3
		if(Panel1.panHomeDir.Enable)
			panHomeDir(Panel1.panHomeDir.X, Panel1.panHomeDir.Y); //13x3

		if(Panel1.panTime.Enable)
			panTime(Panel1.panTime.X, Panel1.panTime.Y);

		if(Panel1.panWPDis.Enable)
			panWPDis(Panel1.panWPDis.X, Panel1.panWPDis.Y); //??x??
			
		if(Panel1.panVel.Enable)
			panVel(Panel1.panVel.X, Panel1.panVel.Y); 
			
		if(Panel1.PanBoatSpeed.Enable) 
			panBoatSpeed(Panel1.PanBoatSpeed.X, Panel1.PanBoatSpeed.Y); //
			
		if(Panel1.panThr.Enable) 
			panThr(Panel1.panThr.X, Panel1.panThr.Y); //
			
		if(Panel1.PanNavMod.Enable) 
			PanNavigationMode(Panel1.PanNavMod.X, Panel1.PanNavMod.Y);  //
			
		if(Panel1.panHorizon.Enable) 
			panHorizon(Panel1.panHorizon.X, Panel1.panHorizon.Y); //14x5
			
		if(Panel1.panCur_A.Enable) 
			panCur_A(Panel1.panCur_A.X, Panel1.panCur_A.Y);
			
		if(Panel1.panWindSpeed.Enable) 
			panWindSpeed(Panel1.panWindSpeed.X, Panel1.panWindSpeed.Y);
			
		if(Panel1.panRSSI.Enable) 
			panRSSI(Panel1.panRSSI.X, Panel1.panRSSI.Y); //??x??
			
		if(Panel1.panEff.Enable) 
			panEff(Panel1.panEff.X, Panel1.panEff.Y);
			
		if(Panel1.panTemp.Enable) 
			panTemp(Panel1.panTemp.X, Panel1.panTemp.Y);
			
		if(Panel1.panDistance.Enable)
			panDistance(Panel1.panDistance.X, Panel1.panDistance.Y);
		
		if(Panel1.panMess.Enable)
			panMessage(Panel1.panMess.X, Panel1.panMess.Y);
				
		if(Panel1.PanLogging.Enable)
			PanLogState(Panel1.PanLogging.X, Panel1.PanLogging.Y);	
		
		if(Panel1.panFrontDistance.Enable)
			PanFrontDistance(Panel1.panFrontDistance.X,Panel1.panFrontDistance.Y);
		break;
		
	case 1:
		if((Panel2.panPitch.Enable))
		PanPitch(Panel2.panPitch.X, Panel2.panPitch.Y); //5x1

		if(Panel2.panRoll.Enable)
		PanRoll(Panel2.panRoll.X, Panel2.panRoll.Y); //5x1

		if(Panel2.panBatt.Enable)
		PanBattVolts(Panel2.panBatt.X, Panel2.panBatt.Y); //7x1

		if(Panel2.panGPSats.Enable)
		panGPSats(Panel2.panGPSats.X, Panel2.panGPSats.Y); //5x1

		if(Panel2.panGPS.Enable)
			panGPS(Panel2.panGPS.X, Panel2.panGPS.Y); //12x3

		if(Panel2.panBatteryPercent.Enable)
			panBatteryPercent(Panel2.panBatteryPercent.X, Panel2.panBatteryPercent.Y,true); //

		if(Panel2.panCOG.Enable)
			panCOG(Panel2.panCOG.X, Panel2.panCOG.Y); //
					
		if(Panel2.panRose.Enable)
			panRose(Panel2.panRose.X, Panel2.panRose.Y);        //13x3
					
		if(Panel2.panHeading.Enable)
			panHeading(Panel2.panHeading.X, Panel2.panHeading.Y); //13x3
					
		if(Panel2.panHomeDis.Enable)
		panHomeDis(Panel2.panHomeDis.X, Panel2.panHomeDis.Y); //13x3

		if(Panel2.panHomeDir.Enable)
		panHomeDir(Panel2.panHomeDir.X, Panel2.panHomeDir.Y); //13x3

		if(Panel2.panTime.Enable)
		panTime(Panel2.panTime.X, Panel2.panTime.Y);

		if(Panel2.panWPDis.Enable)
			panWPDis(Panel2.panWPDis.X, Panel2.panWPDis.Y); //??x??
					
		if(Panel2.panVel.Enable)
		panVel(Panel2.panVel.X, Panel2.panVel.Y);
					
		if(Panel2.PanBoatSpeed.Enable)
			panBoatSpeed(Panel2.PanBoatSpeed.X, Panel2.PanBoatSpeed.Y); 
			
		if(Panel2.panThr.Enable)
			panThr(Panel2.panThr.X, Panel2.panThr.Y); 
			
		if(Panel2.PanNavMod.Enable)
			PanNavigationMode(Panel2.PanNavMod.X, Panel2.PanNavMod.Y);  
			
		if(Panel2.panHorizon.Enable)
			panHorizon(Panel2.panHorizon.X, Panel2.panHorizon.Y); //14x5
			
		if(Panel2.panCur_A.Enable)
			panCur_A(Panel2.panCur_A.X, Panel2.panCur_A.Y);
					
		if(Panel2.panWindSpeed.Enable)
			panWindSpeed(Panel2.panWindSpeed.X, Panel2.panWindSpeed.Y);
					
		if(Panel2.panRSSI.Enable)
			panRSSI(Panel2.panRSSI.X, Panel2.panRSSI.Y); //??x??
					
		if(Panel2.panEff.Enable)
			panEff(Panel2.panEff.X, Panel2.panEff.Y);
					
		if(Panel2.panTemp.Enable)
			panTemp(Panel2.panTemp.X, Panel2.panTemp.Y);
					
		if(Panel2.panDistance.Enable)
			panDistance(Panel2.panDistance.X, Panel2.panDistance.Y);
		break;
	case 2:
		if((Panel3.panPitch.Enable))
			PanPitch(Panel3.panPitch.X, Panel3.panPitch.Y); //5x1

		if(Panel3.panRoll.Enable)
			PanRoll(Panel3.panRoll.X, Panel3.panRoll.Y); //5x1

		if(Panel3.panBatt.Enable)
			PanBattVolts(Panel3.panBatt.X, Panel3.panBatt.Y); //7x1

		if(Panel3.panGPSats.Enable)
			panGPSats(Panel3.panGPSats.X, Panel3.panGPSats.Y); //5x1

		if(Panel3.panGPS.Enable)
			panGPS(Panel3.panGPS.X, Panel3.panGPS.Y); //12x3

		if(Panel3.panBatteryPercent.Enable)
			panBatteryPercent(Panel3.panBatteryPercent.X, Panel3.panBatteryPercent.Y,true); //

		if(Panel3.panCOG.Enable)
			panCOG(Panel3.panCOG.X, Panel3.panCOG.Y); //
					
		if(Panel3.panRose.Enable)
			panRose(Panel3.panRose.X, Panel3.panRose.Y);        //13x3
					
		if(Panel3.panHeading.Enable)
			panHeading(Panel3.panHeading.X, Panel3.panHeading.Y); //13x3
					
		if(Panel3.panHomeDis.Enable)
			panHomeDis(Panel3.panHomeDis.X, Panel3.panHomeDis.Y); //13x3

		if(Panel3.panHomeDir.Enable)
			panHomeDir(Panel3.panHomeDir.X, Panel3.panHomeDir.Y); //13x3

		if(Panel3.panTime.Enable)
		panTime(Panel3.panTime.X, Panel3.panTime.Y);

		if(Panel3.panWPDis.Enable)
			panWPDis(Panel3.panWPDis.X, Panel3.panWPDis.Y); //??x??
					
		if(Panel3.panVel.Enable)
		panVel(Panel3.panVel.X, Panel3.panVel.Y);
					
		if(Panel3.PanBoatSpeed.Enable)
			panBoatSpeed(Panel3.PanBoatSpeed.X, Panel3.PanBoatSpeed.Y); 
			
		if(Panel3.panThr.Enable)
			panThr(Panel3.panThr.X, Panel3.panThr.Y); 
			
		if(Panel3.PanNavMod.Enable)
			PanNavigationMode(Panel3.PanNavMod.X, Panel3.PanNavMod.Y);  
			
		if(Panel3.panHorizon.Enable)
			panHorizon(Panel3.panHorizon.X, Panel3.panHorizon.Y); //14x5
			
		if(Panel3.panCur_A.Enable)
			panCur_A(Panel3.panCur_A.X, Panel3.panCur_A.Y);
					
		if(Panel3.panWindSpeed.Enable)
			panWindSpeed(Panel3.panWindSpeed.X, Panel3.panWindSpeed.Y);
					
		if(Panel3.panRSSI.Enable)
			panRSSI(Panel3.panRSSI.X, Panel3.panRSSI.Y); //??x??
					
		if(Panel3.panEff.Enable)
			panEff(Panel3.panEff.X, Panel3.panEff.Y);
					
		if(Panel3.panTemp.Enable)
			panTemp(Panel3.panTemp.X, Panel3.panTemp.Y);
					
		if(Panel3.panDistance.Enable)
			panDistance(Panel3.panDistance.X, Panel3.panDistance.Y);
	}

}




void COsd::PanWaitForTelemetry(int first_col, int first_line)
{
	//panLogo();
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("No Tele data!");
	Max7456.ClosePanel();
}


void COsd::panFdata()
{
	Max7456.OpenPanel(11, 4);
	Max7456.Printf("%c%3i%c%02i|%c%5i%c|%c%5i%c|%c%5i%c|%c%10.6f|%c%10.6f", 0x08,((int)Zumo.m_TotalTripTimeSeconds/60)%60,0x3A
	,(int)Zumo.m_TotalTripTimeSeconds%60, 0x0B, (int)((NavigationFunctions.m_Bearings.DistanceToHome/100) * converth), high, 0x8F, (int)((Zumo.m_TotalDistance)),
	high,0x14,(int)(Gps.m_GpsReadings.GroundCourse/10),0x19,0x12, high, 0x03, (double)Gps.m_Coords.Latitude, 0x04, (double)Gps.m_Coords.Longitude);
	Max7456.ClosePanel();
}



/* **************************************************************** */
// Panel  : PanAlarms
// Needs  : X, Y locations
// Output : Alarm Messages
// Size   : 1 x 7  (rows x chars)
// Status  : done


void COsd::PanAlarms(int first_col, int first_line)
{
//	bool warning[]={0,0,0,0,0,0}; // Make and clear the array
	char AlarmText[20];
	char MessageText[18];
	static uint8_t  Rotation = 0;
	static uint8_t  TimerCount = 0;
//	char TmpStr[80];
	uint8_t AlarmCount = Alarms.GetAlarmCount();  
	
	if (++TimerCount >10)
	{
		TimerCount = 0;
		switch(Rotation)
		{
			case 0:
				if(AlarmCount > 0)
				{
					Alarms.GetAlarmText(0,(char*)AlarmText);
					Rotation++;
				}
				else
				{
					Max7456.Printf("               ");
					return;
				}
				break;
			case 1:
				if(AlarmCount > 1)
				{
					Alarms.GetAlarmText(1,(char*)AlarmText);
					Rotation++;
				}
				else
				{
					Rotation = 0;
					return;
				}
				break;
			case 2:
				if(AlarmCount > 2)
				{
					Alarms.GetAlarmText(2,(char*)AlarmText);
					Rotation++;
				}
				else
				{
					Rotation = 0;
					return;
				}
				break;
			case 3:
				if(AlarmCount > 3)
				{
					Alarms.GetAlarmText(3,(char*)AlarmText);
					Rotation++;
				}
				else
				{
					Rotation = 0;
					return;
				}
				break;
			case 4:
				Rotation = 0;
				if(AlarmCount > 4)
				{
					Alarms.GetAlarmText(4,(char*)AlarmText);
				}
				else
					return;
				break;
		}
		SetAlarmText((char*)AlarmText,(char*)MessageText);

		Max7456.OpenPanel(first_col, first_line);
		Max7456.Printf("%s",MessageText);
		Max7456.ClosePanel();
		
	}

}

void COsd::SetAlarmText(char *Alarm,char *Message)
{
	uint8_t i;
	uint8_t Len = strlen(Alarm);
	memset(Message,0x20,15);
	if(Len>15)
	 Len = 15;
	for(i=0;i<Len;i++)
		Message[i] = Alarm[i];
	Message[16] =0;
}

//=============================================================
// Panel  : SetPanelState
// Needs  : Panel No
// Output : 
//=============================================================
void COsd::SetPanelState(uint8_t State)
{
	
	if(m_CurrentPanel != State)
	{
		Max7456.clear();
		m_CurrentPanel = State;
	}
}


//=============================================================
// Panel  : SetPanelState
// Needs  : Panel No
// Output : 
//=============================================================
void COsd::RotatePanelState()
{
	static uint8_t LastState = 0;
	
	m_CurrentPanel = LastState++;
	if(LastState >2)
		LastState = 0;
}



//====================================================================================
// Panel  : PanPitch
// Needs  : X, Y locations
// Output : -+ value of current Pitch from vehicle with degree symbols and pitch symbol
// Size   : 1 x 6  (rows x chars)
// Status  : done
//====================================================================================

void COsd::PanPitch(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%4i%c%c",Ahrs.GetPitchD(),0x05,0x07);
	Max7456.ClosePanel();
}



//====================================================================================
// Panel  : PanLogState
// Needs  : X, Y locations
// Output : -+ value of current Pitch from vehicle with degree symbols and pitch symbol
// Size   : 1 x 6  (rows x chars)
// Status  : done
//====================================================================================

void COsd::PanLogState(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
//	if(Config.m_RunningFlags.LOGGING_IN_PROGRESS)
//		Max7456.Printf("Logging Active");
//	else
		Max7456.Printf("Logging Off");
	Max7456.ClosePanel();
}



//====================================================================================
// Panel  : PanRoll
// Needs  : X, Y locations
// Output : -+ value of current Roll from vehicle with degree symbols and roll symbol
// Size   : 1 x 6  (rows x chars)
// Status  : done
//====================================================================================

void COsd::PanRoll(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%4i%c%c",Ahrs.GetRollD(),0x05,0x06);
	Max7456.ClosePanel();
}

//====================================================================================
// Panel  : PanBattVolts (Voltage 1)
// Needs  : X, Y locations
// Output : Voltage value as in XX.X and symbol of over all battery status
// Size   : 1 x 8  (rows x chars)
// Status  : done
//====================================================================================

void COsd::PanBattVolts(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("(1)%5.2f%c|(2)%5.2f%c", (double)Sensors.m_SensorValues.VoltsBattery_1, 0x0d, 0.0, 0x0d);
	Max7456.ClosePanel();
}

//====================================================================================
// Panel  : PanVtx ()
// Needs  : X, Y locations
// Output : Vtx Freq and power
// Size   : 1 x 8  (rows x chars)
// Status  : done
//====================================================================================

void COsd::PanVtx(int first_col, int first_line)
{
// 	uint16_t Band;
// 	uint16_t Power;
 	Max7456.OpenPanel(first_col, first_line);
// 	if(Config.m_RunningFlags.VTX_PRESENT)
// 	{
// 		Vtx.GetBandAndPower(Band,Power);
// 		Max7456.Printf("Vtx %d Hz %d mw", Band, Power);
// 	}
// 	else
// 		Max7456.Printf("No Vtx Present", Band, Power);
// 	
 	Max7456.ClosePanel();
}

//====================================================================================
// Panel  : panCur_A
// Needs  : X, Y locations
// Output : Current 
// Size   : 1 x 7Hea  (rows x chars)
// Status  : done
//====================================================================================

void COsd::panCur_A(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%3.2f%c", (float(Sensors.m_SensorValues.Current)), 0x0e);
	Max7456.ClosePanel();
}

//====================================================================================
// Panel  : panBatteryPercent
// Needs  : X, Y locations
// Output : Battery state 
// Size   : 1 x 7  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panBatteryPercent(int first_col, int first_line,bool Type)
{
	Max7456.OpenPanel(first_col, first_line);
	if (Type)
	{
		Max7456.Printf("%c%03d%c", 0x17, Sensors.m_SensorValues.BatteryRemaining, 0x25);
	}
	else
	{
		Max7456.Printf("%c%4.0f%c", 0x17, Sensors.m_SensorValues.MahUsed, 0x01);
	}
	Max7456.ClosePanel();
}



//====================================================================================
// Panel  : panGPSats
// Needs  : X, Y locations
// Output : 1 symbol and number of locked satellites
// Size   : 1 x 5  (rows x chars)
// Status  : done
//====================================================================================

void COsd::panGPSats(int first_col, int first_line)
{
	uint8_t gps_str = 0x2a;
	GPS_Status_t Status;
	Status =  Gps.GetStatus();
	Max7456.OpenPanel(first_col, first_line);
		
	if (Status == GPS_OK_FIX_2D)
		gps_str = 0x1f;
	if (Status == GPS_OK_FIX_3D || Status == GPS_OK_FIX_3D_RTK_FIXED || Status == GPS_OK_FIX_DGPS || Status == GPS_OK_FIX_3D_RTK_FLOAT)
		gps_str = 0x0f;

	if ((eph >= 200) && m_Blinker)
	gps_str = 0x20;

	Max7456.Printf("%c%2i|Hdop %02.2f", gps_str, Gps.m_GpsReadings.NumSats,(float)Gps.m_GpsReadings.Hdop/100);
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : panGPS
// Needs  : X, Y locations
// Output : two row numeric value of current GPS location with LAT/LON symbols as on first char
// Size   : 2 x 12  (rows x chars)
// Status : done
//====================================================================================
void COsd::panGPS(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%4.6f|%4.6f", (double)Gps.m_Coords.Latitude/10000000, (double)Gps.m_Coords.Longitude/10000000);
	Max7456.ClosePanel();
}



//====================================================================================
// Panel  : COG Course Over Ground
// Needs  : X, Y locations
// Output :
// Size   :
// Status : done
//====================================================================================

void COsd::panCOG(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	
	osd_COG_arrow_rotate_int = round(((Gps.m_GpsReadings.GroundCourse/100) - Ahrs.GetHeading())/360.0 * 16.0 +1); //Convert to int 1-16
	if(osd_COG_arrow_rotate_int < 0 )
		osd_COG_arrow_rotate_int += 16;
	if(osd_COG_arrow_rotate_int == 0)
		osd_COG_arrow_rotate_int = 16;
	if(osd_COG_arrow_rotate_int == 17)
		osd_COG_arrow_rotate_int = 1;
	
	if (((Gps.m_GpsReadings.GroundCourse / 100) - Ahrs.GetHeading()) > 180)
	{
		off_course = (Gps.m_GpsReadings.GroundCourse/100 - Ahrs.GetHeading()) - 360;
	}
	else if (((Gps.m_GpsReadings.GroundCourse/100) - Ahrs.GetHeading()) < -180)
	{
		off_course = (Gps.m_GpsReadings.GroundCourse/100 - Ahrs.GetHeading()) + 360;
	}
	else
	{
		off_course = (Gps.m_GpsReadings.GroundCourse/100 - Ahrs.GetHeading());
	}
	
	ShowArrow((uint8_t)osd_COG_arrow_rotate_int,2);
	Max7456.ClosePanel();
}

//====================================================================================
// Panel  : panHeading
// Needs  : X, Y locations
// Output : Symbols with numeric compass heading value
// Size   : 1 x 5  (rows x chars)
// Status  : not ready
//====================================================================================
void COsd::panHeading(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%4.0f%c", (double)Ahrs.GetHeading(), 0x05);
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : panHomeDis
// Needs  : X, Y locations
// Output : Home Symbol with distance to home in meters
// Size   : 1 x 7  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panHomeDis(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
//	NavigationProcessor.Bearings.DistanceToHome = 20000;
	Max7456.Printf("%5.0f%c", (double)(NavigationFunctions.m_Bearings.DistanceToHome/100), 0x0c);
	Max7456.ClosePanel();
}

//====================================================================================
// Panel  : panHomeDir
// Needs  : X, Y locations
// Output : 2 symbols that are combined as one arrow, shows direction to home
// Size   : 1 x 2  (rows x chars)
// Status : not tested
//====================================================================================
void COsd::panHomeDir(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%c",0x0b);
	uint8_t Bearing = round(NavigationFunctions.m_Bearings.BoatToHomeBearing/360.0 * 16.0) + 9; //Convert to int 1-16

	ShowArrow(Bearing,0);
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : panTime
// Needs  : X, Y locations
// Output : Time from start with symbols
// Size   : 1 x 7  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panTime(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%2i%c%02i",((int)Zumo.m_TotalTripTimeSeconds/60)%60,0x3A,(int)Zumo.m_TotalTripTimeSeconds%60);
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : panWPDis
// Needs  : X, Y locations
// Output : W then distance in Km - Distance to next waypoint
// Size   : 1 x 2  (rows x chars)
// Status  : not ready TODO - CHANGE the Waypoint symbol - Now only a W!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//====================================================================================
void COsd::panWPDis(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	int8_t WpTargetBearingRotateInt = 0;
	
	if (Navigation.m_NextWaypoint.StartBearing > 0)
	{
		WpTargetBearingRotateInt = round((wp_target_bearing - osd_heading)/360 *16.0) + 1;
	}
	else if (Navigation.m_NextWaypoint.StartBearing < 0)
	{
		WpTargetBearingRotateInt = round(((360 + wp_target_bearing) - osd_heading)/360 *16.0) + 1;
	}
	if (WpTargetBearingRotateInt < 0)
		WpTargetBearingRotateInt += 16;
	if (WpTargetBearingRotateInt == 0)
		WpTargetBearingRotateInt = 16;
	
	Max7456.Printf("%c%c%2i",0x57, 0x70, Navigation.m_NextWaypointNo);
	ShowArrow((uint8_t)WpTargetBearingRotateInt,0);

	Max7456.Printf("|%4.0f%c|",(float)(NavigationFunctions.m_Bearings.NextWpDistanceCm/10),0x0c);
	Max7456.Printf("Xte%3im|",NavigationFunctions.m_Bearings.m_XtrkError);
	Max7456.Printf("Rel%3i%c",NavigationFunctions.m_Bearings.RelativeBearing,0x05);
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : panVel
// Needs  : X, Y locations
// Output : Velocity value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panVel(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);

	Max7456.Printf("%c", 0x14);
	Max7456.Printf("%3.0f%c",(double)(Gps.m_GpsReadings.GroundSpeed*0.0194384),0x19);
	Max7456.ClosePanel();
}

//====================================================================================
// Panel  : panBoatSpeed
// Needs  : X, Y locations
// Output : Airspeed value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panBoatSpeed(int first_col, int first_line)
{
//	Gps.m_GpsData.GroundSpeed = 1000;
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%3.0f", (double)(Gps.m_GpsReadings.GroundSpeed*0.0194384));
	Max7456.Printf("|Knots");
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : panThr
// Needs  : X, Y locations
// Output : Throttle value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Status  : done
//====================================================================================

void COsd::panThr(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%3.0i%c",RadioControl.m_RadioDataIn[RC_THROTTLE],0x25);
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : panFlightMode
// Needs  : X, Y locations
// Output : 2 symbols, one static name symbol and another that changes by flight modes
// Size   : 1 x 2  (rows x chars)
// Status : done
//====================================================================================
void COsd::PanNavigationMode(int first_col, int first_line)
{
	char String[14];
	Max7456.OpenPanel(first_col, first_line);

	switch(Navigation.m_NavigationState)
	{
	case ePositionHold:
		strcpy(String,"Posit Hold ");
		break;
	case  eReturnHome:
		strcpy(String,"Ret to Home");
		break;
	case  eOnRouteToNextWapoint:
		strcpy(String,"Route to WP");
		break;
	case  eControlByRc:
		strcpy(String,"Radio Ctl  ");
		break;
	default:
		strcpy(String,"Unknown    ");
		
	}
	Max7456.Printf("%c%s", 0x7F,String);
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : pan wind speed
// Needs  : X, Y locations
// Output : Wind direction symbol (arrow) and velocity
// Size   : 1 x 7  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panWindSpeed(int first_col, int first_line)
{
//	uint16_t WindArrowRotateInt;
	
 	Max7456.OpenPanel(first_col, first_line);
// 
// 	if (osd_winddirection < 0)
// 	{
// 		WindArrowRotateInt = round(((Sensors.m_SensorValues.WindDirection + 360) - Attitude.m_AttitudeValues.Heading)/360.0 * 16.0) + 9; //Convert to int 1-16
// 	}
// 	else
// 	{
// 		WindArrowRotateInt = round((Sensors.m_SensorValues.WindDirection - Attitude.m_AttitudeValues.Heading)/360.0 * 16.0) + 9; //Convert to int 1-16
// 	}
// 	if(WindArrowRotateInt > 16 )
// 		WindArrowRotateInt -= 16; //normalize
// 	if(WindArrowRotateInt < 1 )
// 		WindArrowRotateInt += 16; //normalize
// 	
// //	nor_osd_windspeed = osd_windspeed * 0.010 + nor_osd_windspeed * 0.990;
// 	
// 	ShowArrow((uint8_t)WindArrowRotateInt,1); //print data to OSD
// 	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : panRSSI
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panRSSI(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%c%3i%%",0x09,RadioControl.m_LQI);
	Max7456.ClosePanel();
}

//====================================================================================
// Panel  : efficiency
// Needs  : X, Y locations
// Output :
// Size   : 1 x 7Hea  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panEff(int first_col, int first_line)
{
	static 	uint16_t RemainingEstimatedRunTimeSeconds;

	Max7456.OpenPanel(first_col, first_line);
	//Check takeoff just to prevent incidental false readings
	if (motor_armed)
	{
		if(Sensors.m_SensorValues.BatteryRemaining != m_LastBatteryReading)
		{
			RemainingEstimatedRunTimeSeconds = ((float)Sensors.m_SensorValues.BatteryRemaining * Zumo.m_TotalTripTimeSeconds / (m_MaxBatteryReading - Sensors.m_SensorValues.BatteryRemaining)) / 1000;
			m_LastBatteryReading = Sensors.m_SensorValues.BatteryRemaining;
		}
		Max7456.Printf("%c%2i%c%02i", 0x17,((int)RemainingEstimatedRunTimeSeconds/60)%60,0x3A,(int)RemainingEstimatedRunTimeSeconds%60);
	}
	Max7456.ClosePanel();
}

//====================================================================================
// Panel  : ODO
// Needs  : X, Y locations
// Output :
// Size   : 1 x 7Hea  (rows x chars)
// Status  : done
//====================================================================================

void COsd::panDistance(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%c%5.2f%c", 0x8f, Zumo.m_TotalDistance, 0x1B);
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : ODO
// Needs  : X, Y locations
// Output :
// Size   : 1 x 7Hea  (rows x chars)
// Status  : done
//====================================================================================

void COsd::PanFrontDistance(int first_col, int first_line)
{
	uint16_t Dist = HeadControl.m_BestCourse.Distance;
	Max7456.OpenPanel(first_col, first_line);
	if(Dist != 0xffff)
		Max7456.Printf("%03d Cm ",Dist);
	else	
		Max7456.Printf("Flt 000");
	Max7456.ClosePanel();
}





//====================================================================================
// Panel  : pantemp
// Needs  : X, Y locations
// Output :
// Size   : 1 x 7Hea  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panTemp(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	//do_converts();
	Max7456.Printf("%5.1f%c", float(Sensors.m_SensorValues.Temperature), temps);
	Max7456.ClosePanel();
}


//====================================================================================
// Panel  : panHorizon
// Needs  : X, Y locations
// Output : 12 x 4 Horizon line surrounded by 2 cols (left/right rules)
// Size   : 14 x 4  (rows x chars)
// Status  : done
//====================================================================================

void COsd::panHorizon(int first_col, int first_line)
{
	Max7456.OpenPanel(first_col, first_line);
	
	Max7456.Printf("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|\xC6\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xC5\x20|\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20");

	Max7456.ClosePanel();
	ShowHorizon((first_col + 1), first_line);
	
	//Show ground level on  HUD
}


//====================================================================================

// Calculate and show artificial horizon
// used formula: y = m * x + n <=> y = tan(a) * x + n
//====================================================================================
void COsd::ShowHorizon(int start_col, int start_row)
{
	int col, row, pitch_line, middle, hit, subval;
	int roll;
	int line_set = LINE_SET_STRAIGHT__;
//	int line_set_overflow = LINE_SET_STRAIGHT_O;
	int subval_overflow = 9;
	
	// preset the line char attributes
	roll = Ahrs.GetRollD();
	if ((roll >= 0 && roll < 90) || (roll >= -179 && roll < -90))			// positive angle line chars
	{
		roll = roll < 0 ? roll + 179 : roll;
		if (abs(roll) > ANGLE_2)
		{
			line_set = LINE_SET_P___STAG_2;
//			line_set_overflow = LINE_SET_P_O_STAG_2;
			subval_overflow = 7;
		}
		else if (abs(roll) > ANGLE_1)
		{
			line_set = LINE_SET_P___STAG_1;
//			line_set_overflow = LINE_SET_P_O_STAG_1;
			subval_overflow = 8;
		}
	}
	else																// negative angle line chars
	{
		roll = roll > 90 ? roll - 179 : roll;
		if (abs(roll) > ANGLE_2)
		{
			line_set = LINE_SET_N___STAG_2;
//			line_set_overflow = LINE_SET_N_O_STAG_2;
			subval_overflow = 7;
		}
		else if (abs(roll) > ANGLE_1)
		{
			line_set = LINE_SET_N___STAG_1;
//			line_set_overflow = LINE_SET_N_O_STAG_1;
			subval_overflow = 8;
		}
	}
	
	pitch_line = round(tan(-AH_PITCH_FACTOR * Ahrs.GetPitchD()) * AH_TOTAL_LINES) + AH_TOTAL_LINES/2;	// 90 total lines
	for (col=1; col<=AH_COLS; col++)
	{
		middle = col * CHAR_COLS - (AH_COLS/2 * CHAR_COLS) - CHAR_COLS/2;	  // -66 to +66	center X point at middle of each column
		hit = tan(AH_ROLL_FACTOR * Ahrs.GetRollD()) * middle + pitch_line;	          // 1 to 90	calculating hit point on Y plus offset
		if (hit >= 1 && hit <= AH_TOTAL_LINES)
		{
			row = (hit-1) / CHAR_ROWS;						  // 0 to 4 bottom-up
			subval = (hit - (row * CHAR_ROWS) + 1) / (CHAR_ROWS / CHAR_SPECIAL);  // 1 to 9
			
			// print the line char
			Max7456.OpenPanel(start_col + col - 1, start_row + AH_ROWS - row - 1);
			Max7456.Printf("%c", line_set + subval);
//			Max7456.Printf("A");
			
			// check if we have to print an overflow line char
			if (subval >= subval_overflow && row < 4)							// only if it is a char which needs overflow and if it is not the upper most row
			{
//				Max7456.openSingle(start_col + col - 1, start_row + AH_ROWS - row - 2);
//				Max7456.Printf("%c", line_set_overflow + subval - OVERFLOW_CHAR_OFFSET);
			}
			Max7456.ClosePanel();
		}
	}
}






//====================================================================================

// Show those fancy 2 char arrows
//====================================================================================
void COsd::ShowArrow(uint8_t rotate_arrow,uint8_t method)
{
	int arrow_set1 = 0x90;
	//We trust that we receive rotate_arrow [1, 16] so it's no needed (rotate_arrow <= 16) in the if clause
	arrow_set1 += rotate_arrow * 2 - 2;

// 	if(method == 1)
// 		Max7456.Printf("%c%02d%c|%c%c%02d%c",0x1d,(Sensors.m_SensorValues.WindSpeed), 0x19, arrow_set1, arrow_set1 + 1,(Sensors.m_SensorValues.MaxWindSpeed), 0x19);
	if(method == 2)
		Max7456.Printf("%c%c%4i%c", arrow_set1, arrow_set1 + 1, off_course, 0x05);
	else
		Max7456.Printf("%c%c", arrow_set1, arrow_set1 + 1);
}


const char buf_Rule[36] = {	0x82,0x80,0x81,0x80,0x81,0x80,	0x84,0x80,0x81,0x80,0x81,0x80,	0x83,0x80,0x81,0x80,0x81,0x80,0x85,0x80,0x81,0x80,0x81,0x80};

void COsd::SetHeadingPattern(uint8_t *Buffer)
{
	int start;
	start = round((Ahrs.GetHeading() * 24)/360);
	start -= 3;
	if(start < 0)
	start += 24;
	for(int x=0; x <= 10; x++)
	{
		Buffer[x] = buf_Rule[start];
		if(++start > 23)
		start = 0;
	}
	Buffer[7] = '\0';
}



//====================================================================================
// Panel  : panRose
// Needs  : X, Y locations
// Output : a dynamic compass rose that changes along the heading information
// Size   : 2 x 13  (rows x chars)
// Status  : done
//====================================================================================
void COsd::panRose(int first_col, int first_line)
{
	uint8_t		Buffer[12];
	SetHeadingPattern(Buffer);
	Max7456.OpenPanel(first_col, first_line);
	Max7456.Printf("%c%s %c", 0x87, Buffer, 0x87);
//	Max7456.Printf("%c%s%c", 0xc3, Buffer, 0x87);
	Max7456.ClosePanel();
}



/* **************************************************************** */
// Panel  : panWarn
// Needs  : X, Y locations
// Output : Airspeed value from Telemetry with symbols
// Size   : 1 x 7  (rows x chars)
// Status  : done


void COsd::panMessage(int first_col, int first_line)
{
	static uint8_t  Rotation = 0;
	static uint8_t  TimerCount = 0;
	
	Max7456.OpenPanel(first_col, first_line);
	if (++TimerCount > 10)
	{
		switch(Rotation)
		{
			case 0:
				Max7456.Printf("                                 ");
				if(m_Messages[0][0] !=0)
					Rotation++;
				break;
			case 1:
				if(m_Messages[0][0] !=0)
				{
					Max7456.Printf("%s",m_Messages[0]);
					Rotation++;
				}
				else
					Rotation = 0;
				break;
			case 2:
				if(m_Messages[0][0] !=0)
				{
					Max7456.Printf("%s",m_Messages[0]);
					Rotation++;
				}
				else
					Rotation = 0;
				break;
			case 3:
				if(m_Messages[0][0] !=0)
				{
					Max7456.Printf("%s",m_Messages[0]);
					Rotation++;
				}
				else
					Rotation = 0;
			case 4:
				if(m_Messages[0][0] !=0)
				{
					Max7456.Printf("%s",m_Messages[0]);
					Rotation++;
				}
				else
					Rotation = 0;
				break;
		}
		
	}
	Max7456.ClosePanel();
}







void COsd::AddMessage(const char *Message)
{
	char String[28] ={0};
	if(strlen(Message) >25)
	{
		strncpy(String,Message,24);
	}
	else
		strcpy(String,Message);
	if(strlen(Message)  ==0)
	{
		strcpy(String,(const char *)"Bad Format Message");
	}
	
	if(IsInMessageQue(String) <5)
		return;
	
	strcpy(m_Messages[5],m_Messages[4]);			// scroll messages
	strcpy(m_Messages[4],m_Messages[3]);
	strcpy(m_Messages[5],m_Messages[2]);
	strcpy(m_Messages[2],m_Messages[1]);
	strcpy(m_Messages[1],m_Messages[0]);
	strcpy(m_Messages[0],String);
}




uint16_t COsd::IsInMessageQue(char* Message)
{
	if(strcmp(m_Messages[0],Message) ==0)		// check for dupes
		return(0);
	if(strcmp(m_Messages[1],Message) ==0)
		return(1);
	if(strcmp(m_Messages[2],Message) ==0)
		return(2);
	if(strcmp(m_Messages[3],Message) ==0)
		return(3);
	if(strcmp(m_Messages[4],Message) ==0)
		return(4);
	if(strcmp(m_Messages[5],Message) ==0)
		return(5);
	return(255);
}






void COsd::UpdatePanelCords()
{

	Panel1.panPitch.Enable		= true;
	Panel1.panPitch.X			= 23;
	Panel1.panPitch.Y			= 2;
	Panel1.panRoll.Enable		= true;
	Panel1.panRoll.X			= 23;
	Panel1.panRoll.Y			= 1;
	Panel1.panBatt.Enable		= true;
	Panel1.panBatt.X			= 1;
	Panel1.panBatt.Y			= 1;
	Panel1.panVtxVal.X			= 1;
	Panel1.panVtxVal.Y			= 13;
	Panel1.panVtxVal.Enable		= false;
	Panel1.panGPSats.Enable		= true;
	Panel1.panGPSats.X			= 1;
	Panel1.panGPSats.Y			= 10;
	Panel1.panCOG.Enable		= false;
	Panel1.panCOG.X				= 23;
	Panel1.panCOG.Y				= 3;
	Panel1.panGPS.Enable		= false;
	Panel1.panGPS.X				= 1;
	Panel1.panGPS.Y				= 13;
	Panel1.panRose.Enable		= true;
	Panel1.panRose.X			= 11;
	Panel1.panRose.Y			= 1;
	Panel1.panHeading.Enable	= true;
	Panel1.panHeading.X			= 13;
	Panel1.panHeading.Y			= 2;
	Panel1.panHomeDir.Enable	= false;
	Panel1.panHomeDir.X			= 14;
	Panel1.panHomeDir.Y			= 3;
	Panel1.panHomeDis.Enable	= false;
	Panel1.panHomeDis.X			= 22;
	Panel1.panHomeDis.Y			= 1;
	Panel1.panWPDis.Enable		= false;
	Panel1.panWPDis.X			= 1;
	Panel1.panWPDis.Y			= 5;
	Panel1.panRSSI.Enable		= true;
	Panel1.panRSSI.X			= 1;
	Panel1.panRSSI.Y			= 6;
	Panel1.panCur_A.Enable		= true;
	Panel1.panCur_A.X			= 1;
	Panel1.panCur_A.Y			= 3;
	Panel1.panVel.Enable		= false;
	Panel1.panVel.X				= 1;
	Panel1.panVel.Y				= 2;
	Panel1.PanBoatSpeed.Enable	= false;
	Panel1.PanBoatSpeed.X		= 1;
	Panel1.PanBoatSpeed.Y		= 1;
	Panel1.panBatteryPercent.Enable	= true;
	Panel1.panBatteryPercent.X	= 1;
	Panel1.panBatteryPercent.Y	= 4;
	Panel1.panTime.Enable		= true;
	Panel1.panTime.X			= 23;
	Panel1.panTime.Y			= 14;
	Panel1.panThr.Enable		= false;
	Panel1.panThr.X				= 1;
	Panel1.panThr.Y				= 13;
	Panel1.PanNavMod.Enable		= true;
	Panel1.PanNavMod.X			= 1;
	Panel1.PanNavMod.Y			= 8;
	Panel1.panHorizon.Enable	= false;
	Panel1.panHorizon.X			= 8;
	Panel1.panHorizon.Y			= 6;
	Panel1.PanAlarms.Enable		= true;
	Panel1.PanAlarms.X			= 9;
	Panel1.PanAlarms.Y			= 6;
	Panel1.panOff.Enable		= false;
	Panel1.panWindSpeed.Enable	= false;
	Panel1.panWindSpeed.X		= 24;
	Panel1.panWindSpeed.Y		= 5;
	Panel1.panEff.Enable		= false;
	Panel1.panEff.X				= 14;
	Panel1.panEff.Y				= 13;
	Panel1.panTemp.Enable		= false;
	Panel1.panTemp.X			= 22;
	Panel1.panTemp.Y			= 14;
	Panel1.panDistance.Enable	= false;
	Panel1.panDistance.X		= 22;
	Panel1.panDistance.Y		= 8;
	Panel1.panMess.Enable		= false;
	Panel1.panMess.X			= 8;
	Panel1.panMess.Y			= 7;
	Panel1.PanLogging.Enable	= false;
	Panel1.PanLogging.X			= 1;
	Panel1.PanLogging.Y			= 14;
	Panel1.panFrontDistance.Enable = true;
	Panel1.panFrontDistance.X   = 21;
	Panel1.panFrontDistance.Y   = 12;
	
	Panel2.panPitch.Enable		= true;
	Panel2.panPitch.X			= 23;
	Panel2.panPitch.Y			= 2;
	Panel2.panRoll.Enable		= true;
	Panel2.panRoll.X			= 23;
	Panel2.panRoll.Y			= 1;
	Panel2.panBatt.Enable		= true;
	Panel2.panBatt.X			= 1;
	Panel2.panBatt.Y			= 1;
	Panel2.panGPSats.Enable		= true;
	Panel2.panGPSats.X			= 1;
	Panel2.panGPSats.Y			= 13;
	Panel2.panCOG.Enable		= false;
	Panel2.panCOG.X				= 23;
	Panel2.panCOG.Y				= 3;
	Panel2.panGPS.Enable		= false;
	Panel2.panGPS.X				= 1;
	Panel2.panGPS.Y				= 13;
	Panel2.panRose.Enable		= true;
	Panel2.panRose.X			= 11;
	Panel2.panRose.Y			= 1;
	Panel2.panHeading.Enable	= true;
	Panel2.panHeading.X			= 12;
	Panel2.panHeading.Y			= 2;
	Panel2.panHomeDir.Enable	= true;
	Panel2.panHomeDir.X			= 25;
	Panel2.panHomeDir.Y			= 4;
	Panel2.panHomeDis.Enable	= true;
	Panel2.panHomeDis.X			= 22;
	Panel2.panHomeDis.Y			= 5;
	Panel2.panWPDis.Enable		= true;
	Panel2.panWPDis.X			= 1;
	Panel2.panWPDis.Y			= 6;
	Panel2.panRSSI.Enable		= true;
	Panel2.panRSSI.X			= 1;
	Panel2.panRSSI.Y			= 4;
	Panel2.panCur_A.Enable		= true;
	Panel2.panCur_A.X			= 1;
	Panel2.panCur_A.Y			= 3;
	Panel2.panVel.Enable		= false;
	Panel2.panVel.X				= 1;
	Panel2.panVel.Y				= 2;
	Panel2.PanBoatSpeed.Enable	= true;
	Panel2.PanBoatSpeed.X		= 22;
	Panel2.PanBoatSpeed.Y		= 11;
	Panel2.panBatteryPercent.Enable	= true;
	Panel2.panBatteryPercent.X	= 1;
	Panel2.panBatteryPercent.Y	= 2;
	Panel2.panTime.Enable		= true;
	Panel2.panTime.X			= 23;
	Panel2.panTime.Y			= 14;
	Panel2.panThr.Enable		= false;
	Panel2.panThr.X				= 1;
	Panel2.panThr.Y				= 13;
	Panel2.PanNavMod.Enable		= true;
	Panel2.PanNavMod.X			= 11;
	Panel2.PanNavMod.Y			= 13;
	Panel2.panHorizon.Enable	= true;
	Panel2.panHorizon.X			= 8;
	Panel2.panHorizon.Y			= 8;
	Panel2.PanAlarms.Enable		= true;
	Panel2.PanAlarms.X			= 9;
	Panel2.PanAlarms.Y			= 4;
	Panel2.panOff.Enable		= false;
	Panel2.panWindSpeed.Enable	= false;
	Panel2.panWindSpeed.X		= 24;
	Panel2.panWindSpeed.Y		= 4;
	Panel2.panEff.Enable		= false;
	Panel2.panEff.X				= 14;
	Panel2.panEff.Y				= 13;
	Panel2.panTemp.Enable		= false;
	Panel2.panTemp.X			= 22;
	Panel2.panTemp.Y			= 14;
	Panel2.panDistance.Enable	= false;
	Panel2.panDistance.X		= 22;
	Panel2.panDistance.Y		= 8;
	Panel2.panMess.Enable		= false;
	Panel2.panMess.X			= 8;
	Panel2.panMess.Y			= 7;


	Panel3.panPitch.Enable		= false;
	Panel3.panPitch.X			= 23;
	Panel3.panPitch.Y			= 2;
	Panel3.panRoll.Enable		= false;
	Panel3.panRoll.X			= 23;
	Panel3.panRoll.Y			= 1;
	Panel3.panBatt.Enable		= true;
	Panel3.panBatt.X			= 1;
	Panel3.panBatt.Y			= 1;
	Panel3.panGPSats.Enable		= false;
	Panel3.panGPSats.X			= 1;
	Panel3.panGPSats.Y			= 11;
	Panel3.panCOG.Enable		= false;
	Panel3.panCOG.X				= 23;
	Panel3.panCOG.Y				= 3;
	Panel3.panGPS.Enable		= false;
	Panel3.panGPS.X				= 1;
	Panel3.panGPS.Y				= 13;
	Panel3.panRose.Enable		= true;
	Panel3.panRose.X			= 11;
	Panel3.panRose.Y			= 1;
	Panel3.panHeading.Enable	= false;
	Panel3.panHeading.X			= 24;
	Panel3.panHeading.Y			= 8;
	Panel3.panHomeDir.Enable	= false;
	Panel3.panHomeDir.X			= 14;
	Panel3.panHomeDir.Y			= 3;
	Panel3.panHomeDis.Enable	= false;
	Panel3.panHomeDis.X			= 22;
	Panel3.panHomeDis.Y			= 1;
	Panel3.panWPDis.Enable		= false;
	Panel3.panWPDis.X			= 1;
	Panel3.panWPDis.Y			= 5;
	Panel3.panRSSI.Enable		= true;
	Panel3.panRSSI.X			= 1;
	Panel3.panRSSI.Y			= 4;
	Panel3.panCur_A.Enable		= true;
	Panel3.panCur_A.X			= 1;
	Panel3.panCur_A.Y			= 6;
	Panel3.panVel.Enable		= false;
	Panel3.panVel.X				= 1;
	Panel3.panVel.Y				= 2;
	Panel3.PanBoatSpeed.Enable	= true;
	Panel3.PanBoatSpeed.X		= 13;
	Panel3.PanBoatSpeed.Y		= 13;
	Panel3.panBatteryPercent.Enable	= true;
	Panel3.panBatteryPercent.X	= 1;
	Panel3.panBatteryPercent.Y	= 2;
	Panel3.panTime.Enable		= true;
	Panel3.panTime.X			= 23;
	Panel3.panTime.Y			= 14;
	Panel3.panThr.Enable		= false;
	Panel3.panThr.X				= 1;
	Panel3.panThr.Y				= 13;
	Panel3.PanNavMod.Enable		= true;
	Panel3.PanNavMod.X			= 1;
	Panel3.PanNavMod.Y			= 14;
	Panel3.panHorizon.Enable	= false;
	Panel3.panHorizon.X			= 8;
	Panel3.panHorizon.Y			= 6;
	Panel3.PanAlarms.Enable		= true;
	Panel3.PanAlarms.X			= 12;
	Panel3.PanAlarms.Y			= 3;
	Panel3.panOff.Enable		= false;
	Panel3.panWindSpeed.Enable	= false;
	Panel3.panWindSpeed.X		= 24;
	Panel3.panWindSpeed.Y		= 5;
	Panel3.panEff.Enable		= false;
	Panel3.panEff.X				= 14;
	Panel3.panEff.Y				= 13;
	Panel3.panTemp.Enable		= false;
	Panel3.panTemp.X			= 22;
	Panel3.panTemp.Y			= 14;
	Panel3.panDistance.Enable	= false;
	Panel3.panDistance.X		= 22;
	Panel3.panDistance.Y		= 8;
	Panel3.panMess.Enable		= false;
	Panel3.panMess.X			= 8;
	Panel3.panMess.Y			= 7;


// 	Pannel1.overspeed	= 30;
// 	Pannel1.stall		= 0;
// 	Pannel1.battv		= 100; //10Volts
// 	Pannel1.ch_toggle	= 6;
// 	Pannel1.PalNtscMode	= 1;
// 	Pannel1.OsdBrightness = 1;
	



}
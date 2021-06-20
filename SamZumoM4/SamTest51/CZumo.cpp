/* 
* CZumo.cpp
*
* Created: 14/05/2020 12:35:07
* Author: philg
*/

#include "Includes.h"
CMotors MotorControl;
CAnalog Analog;


// default constructor
CZumo::CZumo()
{
} //CZumo

// default destructor
CZumo::~CZumo()
{
} //~CZumo


void CZumo::Init()
{

	EnableMainClocks();
	SetInterupts.Init();
	Buzzer.Init();
//	Uart.Init(115200);
	Uart.Init(57600);
	I2c.init();
	Spi.Init();
	Config.Init();
	Osd.Init();
	SbusUart.Init(100000);
	RadioControl.Init();
	TeleSoftwareSerial.Init(57600);
//	DebugSerial.Init(115200);
	SetDebugSerial(DEBUG_SSERIAL);
//	SetDebugSerial(GUI_SSERIAL);
	DebugDisplay.Init();
	DebugDisplay.Printf("Starting");
	StatusControl.Init();
	HeadControl.Init();
	Imu.Init();
	AttitudeControl.Init();
	Motors.Init();
	Config.m_RunningFlags.ENABLE_WHEEL_SENSORS = true;
//	Gps.Init();
	Sensors.Init();
//	TestLoop();
	DebugSerial.Enable(true);
	HuskyMain.Init();
	
	if(Config.m_RunningFlags.TELEMETRY_ACTIVE)
		TeleSoftwareSerial.Enable(true);
	else
		TeleSoftwareSerial.Enable(false);
	
	Avoidance.Init();
//	Sensors.CalibrateWheelSensors();
	Navigation.Init(false);
	TaskManager.Init();
	TaskManager.EnableTask(TASK_START_TASKS,true);
	Config.m_RunningFlags.AVOIDANCE_ACTIVE = true;
//	Config.m_RunningFlags.ARMED = true;       // Removed PSB 22/05/21
	StatusControl.Startup();
}

// allow for modules to boot up
void CZumo::DelayedSetup()
{
	//	DebugDisplay.Printf(DEBUG_INIT_1_ROW,DEBUG_INIT_1_COL,15,"Init Complete");
	//	Osd.AddMessage((const char *)"Init Complete");

	//	CheckI2cDevices();
	//	DebugDisplay.Init();
	//	TaskManager.EnableTask(TASK_RADAR_UPDATE,true);
	//	TaskManager.EnableTask(TASK_OBSTACLE_AVOIDANCE,true);			// rear and front module
	Osd.m_AllowUpdate = true;
	TaskManager.EnableTask(TASK_DELAYED_SETUP,false);
	TaskManager.EnableTask(TASK_SYSTEM,true);
	Config.m_RunningFlags.MAIN_INIT_COMPLETED = true;

}



void CZumo::StartTasks()
{
	static uint8_t TaskNo = 0;
	switch(TaskNo++)
	{
	case 0:
		TaskManager.EnableTask(TASK_TASK_SYSTEM,true);
		TaskManager.EnableTask(TASK_UPDATE_REPORT_STATUS,true);
		break;
	case	1:
		TaskManager.EnableTask(TASK_UPDATE_SENSORS,true);
		break;
	case 2:
		TaskManager.EnableTask(TASK_CHECK_ALARMS,true);
		break;
	case 3:
		TaskManager.EnableTask(TASK_RADIO_VALUES,true);
		break;
	case 4:
	   	TaskManager.EnableTask(TASK_UPDATE_BEARINGS,true);
		break;
	case 5:
		TaskManager.EnableTask(TASK_NAVIGATION_UPDATE,true);
		break;
	case 6:
		TaskManager.EnableTask(TASK_UPDATE_GUI,true);
		break;
	case 7:
		TaskManager.EnableTask(TASK_UPDATE_GPS,true);
		break;
	case 8:
		TaskManager.EnableTask(TASK_UPDATE_ATTITUDE,true);
		break;
	case 9:
//		TaskManager.EnableTask(TASK_FRONT_RANGE,true);
//		break;
	case 10:
		TaskManager.EnableTask(TASK_UPDATE_OSD,true);
		break;
	case 11:
	  	TaskManager.EnableTask(TASK_UPDATE_MOTOR,true);
		break;
	case 12:
		TaskManager.EnableTask(TASK_UPDATE_AHRS,true);
		break;
	case 13:
		TaskManager.EnableTask(TASK_UPDATE_DEBUG_DISPLAY,true);
		break;
	case 14:
		TaskManager.EnableTask(TASK_FRSKY_TELE_UPDATE,true);
		break;
	case 15:
		TaskManager.EnableTask(TASK_UPDATE_AHRS,true);
		break;
	case 16:
		TaskManager.EnableTask(TASK_AVOIDANCE,true);
		break;
	case 17:
		TaskManager.EnableTask(TASK_HEAD_UPDATE,true);
		break;
	case 18:
		TaskManager.EnableTask(TASK_WHEEL_ENCODERS,true);
		break;
	case 19:
		TaskManager.EnableTask(TASK_IMU_UPDATE,true);
		break;
	case 20:
		TaskManager.EnableTask(TASK_UPDATE_STATUS,true);
		break;
	case 21:
		Osd.m_AllowUpdate = true;
		Config.m_RunningFlags.MAIN_INIT_COMPLETED = true;
		TaskManager.EnableTask(TASK_START_TASKS,false);
//		HeadControl.	StartFrontScan();
	}
}

void CZumo::CheckDeviceStatus()
{
	m_DevicesOnLine = 0;
	if(Config.m_RunningFlags.IMU_FAIL)
		m_DevicesOnLine |= (1<<0);


	if(Config.m_RunningFlags.COMPASS_FAIL )
		m_DevicesOnLine |= (1<<1);

	if(Config.m_RunningFlags.HEAD_LIDAR_FAIL)
		m_DevicesOnLine |= (1<<2);
	
}



void CZumo::SetArmedState(bool State)
{
	if(State != Config.m_RunningFlags.ARMED)
	{
		if(State)
		{
			Config.m_RunningFlags.ARMED = true;
//			Attitude.CalibrateLevel();
			Motors.SetTravelDirection(DIRECTION_FORWARD);
		}
		else
		{
			Motors.AllMotorsStop();
			Config.m_RunningFlags.ARMED = false;

		}
	}
}

void CZumo::SetCourseDataVariables(uint32_t dt)
{
	uint16_t GroundSpeed;
	dt =dt/1000;
	if(m_MaxBatteryReading < Sensors.m_SensorValues.BatteryRemaining)
	m_MaxBatteryReading = Sensors.m_SensorValues.BatteryRemaining;

	if(Gps.m_GpsStatus == GPS_OK_FIX_3D)
	{
		GroundSpeed = Gps.m_GpsReadings.GroundSpeed*0.0194384;
		if (GroundSpeed > 1.0)
		m_TotalDistance += (GroundSpeed * (dt) / 1000.0);
	}
	Sensors.m_SensorValues.MahUsed += (((Sensors.m_SensorValues.Current*1000 ) * dt) * 0.0000002778f);
	Sensors.m_SensorValues.BatteryRemaining = (Sensors.m_SensorValues.MahUsed/m_BatteryCapacityMa)*100;
	Sensors.m_SensorValues.BatteryRemaining =100 - Sensors.m_SensorValues.BatteryRemaining;
	//Set max data
	if (!Config.m_RunningFlags.ARMED)
	{
		m_TotalCourseTimeMilis += dt;
		m_TotalTripTimeSeconds = m_TotalCourseTimeMilis / 1000;
	}
}


void CZumo::SetDebugSerial(EnableSSerial_t Active)
{
	if(m_SSerialSetTo == Active)
	return;
	Config.m_RunningFlags.ENABLE_GUI_CLI_DISPLAY = false;
	Config.m_RunningFlags.TELEMETRY_ACTIVE = false;
	Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY  = true;
	Config.m_RunningFlags.ENABLE_GUI_DISPLAY  = false;
	TeleSoftwareSerial.Enable(false);
	DebugSerial.Enable(false);										// *********
	

	switch(Active)
	{
	case DEBUG_SSERIAL:
		m_SSerialSetTo = DEBUG_SSERIAL;
		DebugSerial.Enable(true);
		Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY  = false;
		break;
	case GUI_SSERIAL:
		m_SSerialSetTo = GUI_SSERIAL;
		DebugSerial.Enable(false);										// *********
		Config.m_RunningFlags.ENABLE_GUI_DISPLAY  = true;
		break;
	case GUI_SSERIAL_DEBUG:
		m_SSerialSetTo = GUI_SSERIAL_DEBUG;
		Config.m_RunningFlags.ENABLE_GUI_CLI_DISPLAY = true;
		DebugSerial.Enable(false);
		break;
	case TELEMETRY_SSERIAL:
		m_SSerialSetTo = TELEMETRY_SSERIAL;
		Config.m_RunningFlags.TELEMETRY_ACTIVE = true;
		TeleSoftwareSerial.Enable(true);
		break;
	case NO_SSERIAL:
		m_SSerialSetTo = NO_SSERIAL;
	}
}





void CZumo::SetPanicTasks()
{
//	Config.m_RunningFlags.PANIC_MODE_TRIGGERED = true;
	TaskManager.EnableTask(TASK_TASK_SYSTEM,false);
//	TaskManager.EnableTask(TASK_UPDATE_LED_SEQUENCE,true);
	TaskManager.EnableTask(TASK_UPDATE_SENSORS,true);
	TaskManager.EnableTask(TASK_CHECK_ALARMS,true);
	TaskManager.EnableTask(TASK_RADIO_VALUES,true);
	TaskManager.EnableTask(TASK_UPDATE_BEARINGS,true);
//	TaskManager.EnableTask(TASK_NAVIGATION_UPDATE,false);
	TaskManager.EnableTask(TASK_UPDATE_GUI,true);
	TaskManager.EnableTask(TASK_UPDATE_GPS,false);
	TaskManager.EnableTask(TASK_UPDATE_ATTITUDE,false);
	TaskManager.EnableTask(TASK_FRONT_RANGE,false);
//	TaskManager.EnableTask(TASK_UPDATE_OSD,true);
//	TaskManager.EnableTask(TASK_UPDATE_MOTOR,false);
	TaskManager.EnableTask(TASK_UPDATE_AHRS,true);
//	TaskManager.EnableTask(TASK_UPDATE_DEBUG_DISPLAY,false);
	TaskManager.EnableTask(TASK_FRSKY_TELE_UPDATE,false);
	TaskManager.EnableTask(TASK_AVOIDANCE,false);
	
}

void CZumo::Panic()
{
	static bool Start = true;
	uint32_t LastTime =0;
	uint32_t NowTime;
	float dt;
	if(Start)
	{
		Zumo.SetArmedState(false);
		TeleSoftwareSerial.SetInterupt(OFF);
	//	DebugSerial.SetInterupt(OFF);
		SetPanicTasks();
		Start = true;
	}
	while(1)
	{
		
		TaskManager.Scheduler();
		NowTime = Core.millis();
		if(Config.m_RunningFlags.LOST_RC_SIGNAL)
		{
			if(Navigation.AutoAvailable() && !Config.m_RunningFlags.I2C_INTERFACE_FAIL)
			{
				Config.m_RunningFlags.PANIC_MODE_TRIGGERED = false;
				return;
			}
		}
		else
		{
			if(!Config.m_RunningFlags.I2C_INTERFACE_FAIL)
			{
				dt = (NowTime - LastTime);
				Steering.Update(dt);
				Config.m_RunningFlags.PANIC_MODE_TRIGGERED = false;
				return;
			}
		}
	}
}

void CZumo::TestLoop()
{
	while(1)
	{
		Core.delay(54);
	}
}




void CZumo::EnableMainClocks()
{
	
	if ( SysTick_Config( SystemCoreClock / 1000 ) )	// Set Systick to 1ms interval, common to all Cortex-M variants
	{
		// Capture error
		while ( 1 ) ;
	}
	NVIC_SetPriority (SysTick_IRQn,  (1 << __NVIC_PRIO_BITS) - 2);  // set Priority for Systick Interrupt (2nd lowest) 

	// Set Main Clocks
	// sercom1,sercom2,Tc0,Tc1

	MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1;
  
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TC2;
  
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3 | MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5;
  
	MCLK->APBDMASK.reg |= MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
		  | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7;

 

}
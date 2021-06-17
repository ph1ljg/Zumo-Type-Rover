/* 
* CRadioControl.cpp
*
* Created: 21/04/2019 19:38:03
* Author: phil
*/
#include "Includes.h"

uint16_t RcCount = 0;

volatile unsigned char SticksTmp = 0;

volatile uint16_t Shift ;
volatile uint16_t Action;
volatile bool State;
volatile uint16_t Columns,FunctionNo;
volatile unsigned char AuxSwitch =0;
volatile uint8_t Mask =0;
volatile uint16_t Rc_Temp[4];
volatile uint32_t RcTest;
volatile uint16_t Channel;


// default constructor
CRadioControl::CRadioControl()
{
	unsigned char i;
	m_DoMenuFuctionIndex = 0;
	m_Rssi = 0;

	for(i=0;i<NO_OF_RC_CHANS;i++)
		m_RadioDataIn[i] = 1500;
	m_SteerType = PILOT_STEER_TYPE_DEFAULT;
} //CRadioControl

// default destructor
CRadioControl::~CRadioControl()
{
} //~CRadioControl

void CRadioControl::Init()
{
	unsigned char i;

	for(i=0;i<5;i++)
	ComputeRCValues();		// fill average filter
	
	for(i=0;i<NO_OF_RC_CHANS;i++)
		m_RadioDataIn[i] = 1500;
	m_RadioDataIn[RC_THROTTLE] = Config.m_MainConfig.FailsafeThrottleValue;	
	m_FailSafe = true;
	m_FailsafeStartTime = Core.millis();

}


void CRadioControl::Update(uint32_t Time)
{
	CMyMath Math;
	ComputeRCValues();
	m_RadioDataIn[RC_AILE] = Math.constrain_int16(m_RadioDataIn[RC_AILE],1000,2000);
	m_RadioDataIn[RC_ELEV] = Math.constrain_int16(m_RadioDataIn[RC_ELEV],1000,2000);
	m_RadioDataIn[RC_THROTTLE] = Math.constrain_int16(m_RadioDataIn[RC_THROTTLE],1000,2000);

	m_Rssi = Math.Map(m_RadioDataIn[RC_CHANNEL_16],1500,2000,0,100);
 	if(!Config.m_RunningFlags.LOST_RC_SIGNAL)
 	{
 		CheckStickFunctions(); 		CheckAuxSwitchFunctions();		CheckModeSwitches();

		m_ControlIn[RC_AILE] = PwmToAngle(RC_AILE);
		m_ControlIn[RC_THROTTLE] = PwmToRange(RC_THROTTLE);
		if(!m_IsDirectionForward)
			m_ControlIn[RC_THROTTLE] *= -1;
 	}
}



// steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
// throttle_out is in the range -100 ~ +100
void CRadioControl::GetRadioInput(float &steering_out, float &throttle_out)
{
	// no RC input means no throttle and centered steering
	if (Config.m_RunningFlags.FAILSAFE_ACTIVE) 
	{
		steering_out = 0;
		throttle_out = 0;
		return;
	}

	// apply RC skid steer mixing
	switch (m_SteerType)
	{
		case PILOT_STEER_TYPE_DEFAULT:
		case PILOT_STEER_TYPE_DIR_REVERSED_WHEN_REVERSING:
		default: 
		{
			throttle_out = m_ControlIn[RC_THROTTLE];
			// by default regular and skid-steering vehicles reverse their rotation direction when backing up
			steering_out = m_ControlIn[RC_AILE];
			if(!Config.m_RunningFlags.IN_REVERSE)
				steering_out   *= -1;
			break;
		}

		case PILOT_STEER_TYPE_TWO_PADDLES: 
		{
			// convert the two radio_in values from skid steering values left paddle from steering input channel, right paddle from throttle input channel
			// steering = left-paddle - right-paddle throttle = average(left-paddle, right-paddle)
			const float left_paddle = m_ControlIn[RC_AILE];
			const float right_paddle = m_ControlIn[RC_THROTTLE];

			throttle_out = 0.5f * (left_paddle + right_paddle) * 100.0f;
			steering_out = (left_paddle - right_paddle) * 0.5f * 4500.0f;
			break;
		}

		case PILOT_STEER_TYPE_DIR_UNCHANGED_WHEN_REVERSING: 
		{
			throttle_out = m_ControlIn[RC_THROTTLE];
			steering_out = m_ControlIn[RC_AILE];
			break;
		}
	}
}






//  perform stick mixing on one channel  This type of stick mixing reduces the influence of the auto
//  controller as it increases the influence of the users stick input,  allowing the user full deflection if needed
int16_t CRadioControl::stick_mixing(uint16_t Channel,const int16_t servo_in)
{
	float ch_inf = (float)(m_RadioDataIn[Channel] - Config.m_MainConfig.RadioTrims[Channel].middle);
	ch_inf = fabsf(ch_inf);
	ch_inf = MIN(ch_inf, 400.0f);
	ch_inf = ((400.0f - ch_inf) / 400.0f);

	int16_t servo_out = servo_in;
	servo_out *= ch_inf;
	servo_out += m_ControlIn[Channel];
	return servo_out;
}




void CRadioControl::CheckModeSwitches()
{
	if(m_RadioDataIn[RC_CHANNEL_11] > 1900)
	{
		Ahrs.SaveCompassCalibration();
	}
}

//=====================================================================================
//==============        compute and Filter the RX data           ======================
//=====================================================================================
void CRadioControl::ComputeRCValues()
{
//	unsigned char Channel;//,i;
	CMyMath Math;
	uint16_t TempByte;
	bool RetVal = SbusDecoder.ParseInputData(m_RadioDataIn);

	RcTest = Core.millis() - m_FailsafeStartTime;	

	if(!RetVal )
	{
		if((Core.millis() - m_FailsafeStartTime) >4000 )
			Config.m_RunningFlags.LOST_RC_SIGNAL = true;
	}
	else
	{
		Config.m_RunningFlags.LOST_RC_SIGNAL = false;
		if(GuiFunctions.m_CalibrateSticks != CALIBRATE_STICKS_NONE )
		{
			if(GuiFunctions.m_CalibrateSticks == CALIBRATE_STICKS)
			{
				Calibrate(m_RadioDataIn,false);
			}
			else
			{
				Calibrate(m_RadioDataIn,true);
				GuiFunctions.m_CalibrateSticks = CALIBRATE_STICKS_NONE;
			}
		}
		m_FailsafeStartTime = Core.millis();
	}

	for(Channel = 0;Channel<8;Channel++)
	{
		Math.Constrain(m_RadioDataIn[Channel],Config.m_MainConfig.RadioTrims[Channel].min,Config.m_MainConfig.RadioTrims[Channel].max);
	}
}



//  convert a pulse width modulation value to a value in the configured  range
int16_t CRadioControl::PwmToRange( uint8_t Channel) const
{
	return PwmToRangeDz(Config.m_RcChannels.DeadZone[Channel],Channel);
}

//  convert a pulse width modulation value to a value in the configured  range, using the specified deadzone
int16_t CRadioControl::PwmToRangeDz(uint16_t _dead_zone, uint8_t Channel) const
{
	CMyMath Math;
	int16_t r_in = Math.constrain_int16(m_RadioDataIn[Channel], Config.m_RcChannels.ChannnelsMin[Channel], Config.m_RcChannels.ChannnelsMax[Channel]);

	if (Config.m_MainConfig.ServoConf[Channel].Reverse)
	{
		r_in = Config.m_RcChannels.ChannnelsMax[Channel] - (r_in - Config.m_RcChannels.ChannnelsMin[Channel]);
	}

	int16_t radio_trim_low = Config.m_RcChannels.ChannnelsMin[Channel] + _dead_zone;

	if (r_in > radio_trim_low)
	{
		return (((int32_t)(100) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(Config.m_RcChannels.ChannnelsMax[Channel] - radio_trim_low));
	}
	return 0;
}



//  return an "angle in centidegrees" ( -4500 to 4500) from
//  the current radio_in value using the specified dead_zone
//
int16_t CRadioControl::PwmToAngle(uint16_t Channel) const
{
	return PwmToAngleDz(Config.m_RcChannels.DeadZone[Channel],Channel);
}


//  return an "angle in centidegrees" (normally -4500 to 4500) from  the current radio_in value using the specified dead_zone
int16_t CRadioControl::PwmToAngleDz(uint16_t DeadZone, uint8_t Channel) const
{
	CMyMath Math;
	int16_t RetVal;
	int16_t radio_trim_high = Config.m_RcChannels.Trim[Channel] + DeadZone;
	int16_t radio_trim_low = Config.m_RcChannels.Trim[Channel] - DeadZone;
	int16_t RcMax = Config.m_RcChannels.ChannnelsMax[Channel];
	int16_t RcMin = Config.m_RcChannels.ChannnelsMin[Channel];

	int16_t reverse_mul = (Config.m_MainConfig.ServoConf[Channel].Reverse ? -1 : 1);

	// don't allow out of range values
	int16_t r_in = Math.constrain_int16(m_RadioDataIn[Channel], RcMin, RcMax);

	if (r_in > radio_trim_high && RcMax != radio_trim_high)
	{
		return reverse_mul * ((int32_t)4500 * (int32_t)(r_in - radio_trim_high)) / (int32_t)(RcMax - radio_trim_high);
	}
	else if (r_in < radio_trim_low && radio_trim_low != RcMin)
	{
		return reverse_mul * ((int32_t)4500 * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_trim_low - RcMin);
	}
	else
		return 0;
}


bool CRadioControl::GetFailSafe()
{
	return(SbusDecoder.m_FailSafe);
}



bool CRadioControl::MidCheck(unsigned char Stick)
{
	if(m_RadioDataIn[Stick] > RC_MID_LOWER && m_RadioDataIn[Stick] < RC_MID_UPPER)
	return(true);
	return(false);
}


void CRadioControl::CheckStickFunctions()
{
	SticksTmp =0;								// throttle low rest mid
	static unsigned char rcDelayCommand;			// this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
	static unsigned char rcSticks = 0;				// this holds the stick position for commands
	unsigned char i;
	uint16_t FunctionNo =0;
	uint8_t Sticks[4];


	// Get sticks positions
	for(i=0;i<4;i++)
	{
		SticksTmp  = SticksTmp << 2;								// in 2 bits Low 1 ce 3 High 2
		if(m_RadioDataIn[i] > RC_MINCHECK && m_RadioDataIn[i] < RC_MAXCHECK)
		{
			SticksTmp |= 0b11;
			Sticks[i]=2;
		}
		else if(m_RadioDataIn[i] > RC_MAXCHECK)					// check for MAX
		{
			SticksTmp |= 0b10;
			Sticks[i] =4;
		}
		else if(m_RadioDataIn[i] < RC_MINCHECK)					// check for MAX
		{
			SticksTmp |= 0b01;
			Sticks[i] =1;
		}
	}
	

	if(SticksTmp == rcSticks)
	{
		if(rcDelayCommand<21)							// Position Held
			rcDelayCommand++;
	}
	else
		rcDelayCommand = 0;
	rcSticks = SticksTmp;
	
	
	if(rcDelayCommand == 20)
	{
		FunctionNo = CheckConfigurableFunctions(Sticks);
		if(FunctionNo <255)
		SetStickFunction(FunctionNo);
		
	}
}

uint16_t CRadioControl::CheckConfigurableFunctions(uint8_t *StickValues)
{
	uint16_t i;
	for(i=0;i<NO_OF_STICK_FUNCTIONS;i++)
	{
		Rc_Temp[0] = (Config.m_MainConfig.SticksSettings[i]);
		Rc_Temp[0]&= 7;
		Rc_Temp[1] = (Config.m_MainConfig.SticksSettings[i]>>3);
		Rc_Temp[1]&= 7;
		Rc_Temp[2] = (Config.m_MainConfig.SticksSettings[i]>>6);
		Rc_Temp[2]&= 7;
		Rc_Temp[3] = (Config.m_MainConfig.SticksSettings[i]>>9);
		Rc_Temp[3]&= 7;
		
		if((StickValues[RC_THROTTLE] == Rc_Temp[0]) && (StickValues[RC_AILE] == Rc_Temp[1]) && (StickValues[RC_ELEV] == Rc_Temp[2]) && (StickValues[RC_RUDDER] == Rc_Temp[3]))
			return(i);
	}
	
	return(0xff);

}


void CRadioControl::CheckAuxSwitchFunctions()
{
	uint8_t i;

	for(i=RC_CHANNEL_5;i<RC_CHANNEL_14;i++)
		ReadAuxChannel(i);
}

void CRadioControl::ReadAuxChannel(uint8_t Channel)
{
	AuxSwitchPosition_t NewPosition;
	uint8_t AuxData =0;
//	if(Channel != 8)
//	  return;
	const AuxSwitchPosition_t OldPosition = GetPreviousSwitchPosition(Channel);
	
	if (!Read3PosSwitch(Channel,&NewPosition))
		return;

	if (NewPosition == OldPosition)
	{
		DebounceCounter[Channel].Count = 0;
		return;
	}
	if (DebounceCounter[Channel].NewPosition != NewPosition)
	{
		DebounceCounter[Channel].NewPosition = NewPosition;
		DebounceCounter[Channel].Count = 0;
	}
	
	
	if (DebounceCounter[Channel].Count++ < 2)					// a value of 2 means we need 3 values in a row with the same  value to activate
	{
		return;
	}

	AuxData = (1 <<NewPosition);
	
	SetAuxFunction(Channel,AuxData);							// debounced; undertake the action:
	SetPreviousSwitchPosition(Channel,NewPosition);
}



void CRadioControl::SetAuxFunction(uint16_t Channel,uint8_t Position)
{
	
	uint8_t Shift = (Channel-4)*3;
	if(Channel > 10)
	Shift -=1;
	for(FunctionNo=0;FunctionNo<NO_OF_AUX_FUNCTIONS;FunctionNo++)
	{
		Mask = ((Config.m_MainConfig.AuxSwitchSettings[FunctionNo] >> Shift)&7);
		if((Mask & Position) != 0 )
			DoAuxFunction(FunctionNo,Position);
	}
}

void CRadioControl::DoAuxFunction(uint8_t FunctionNo,uint8_t Command)
{
	switch(FunctionNo)
	{
	case 0:
		if(Command ==1)
			Osd.SetPanelState(0);
		if(Command ==2)
			Osd.SetPanelState(1);
		if(Command ==4)
			Osd.SetPanelState(2);

		break;
	case 1:
		if(Command == 1)
			Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT = false;
		if(Command == 4)
			Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT = true;
		break;
	case 2:
		if(Command == 1)
			Config.m_RunningFlags.IN_REVERSE = false;
		else	
			Config.m_RunningFlags.IN_REVERSE = true;
		break;
	case 3:
		if(Command ==1)
			Config.m_RunningFlags.IS_MECHAN_CONTROL = false;
		if(Command == 4)	
			Config.m_RunningFlags.IS_MECHAN_CONTROL = true;
		break;
	case 4:
		if(Command ==1)
			Config.m_RunningFlags.RETURN_HOME = false;
		if(Command == 4)
			Config.m_RunningFlags.RETURN_HOME = true;
		break;
	case 5:
		if(Command ==1)
			Config.m_RunningFlags.RC_HEAD_CONTROL = false;
		if(Command == 4)
			Config.m_RunningFlags.RC_HEAD_CONTROL = true;
		break;
	case 6:
		if(Command ==1)
			Config.m_RunningFlags.AVOIDANCE_INHIBIT = false;
		else
			Config.m_RunningFlags.AVOIDANCE_INHIBIT = true;
		break;
	case 7:
		break;
	case 8:
		break;
	case 9:
		break;
	}
}



// read_3pos_switch
bool CRadioControl::Read3PosSwitch(uint8_t Channel,AuxSwitchPosition_t *Position)
{
	const uint16_t Value = m_RadioDataIn[Channel];

	if (Value <= 900 || Value >= 2200)
		return false;

	if (Value < 1200)
		*Position = TRIGGER_LOW;

	else if (Value > 1800)
		*Position = TRIGGER_HIGH;
	
	else
		*Position = TRIGGER_MID;
	return true;
}


void CRadioControl::SetStickFunction(uint16_t Number)
{
	if(Config.m_RunningFlags.ARMED)
	{
		switch(Number)
		{
		case 0:
			break;
		case 1:
			Zumo.SetArmedState(false);
			break;
		case 2:
			break;
		case 3:
			break;
		default:
			break;
		}
	}
	else
	{
		switch(Number)
		{
		case 0:
			Zumo.SetArmedState(true);
			break;
		case 1:
			break;
		case 2:
			break;
		}
		
	}
}



void CRadioControl::Calibrate(uint16_t *ReceivedData,bool Save)
{
	uint8_t i;
	static bool Start = true;
	if(Start)
	{
		for(i=0;i<16;i++)
		{
			Config.m_RcChannels.ChannnelsMax[i] = 0;
			Config.m_RcChannels.ChannnelsMin[i] = 2500;
			Start = false;
		}
	}
	for(i=0;i<16;i++)
	{
		if(ReceivedData[i] > Config.m_RcChannels.ChannnelsMax[i])
			Config.m_RcChannels.ChannnelsMax[i] = ReceivedData[i];

		if(ReceivedData[i] < Config.m_RcChannels.ChannnelsMin[i])
			Config.m_RcChannels.ChannnelsMin[i] = ReceivedData[i];
	}
 	if(Save)
 		Config.WriteRcConfig();

}
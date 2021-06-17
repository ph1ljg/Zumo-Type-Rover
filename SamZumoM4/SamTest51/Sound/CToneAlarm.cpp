/* 
* ToneAlarm.cpp
*
* Created: 11/06/2021 10:08:45
* Author: philg
*/
#include "Includes.h"
#include "CPlayer.h"
#include "CToneAlarm.h"

CPlayer Player;


// Tunes follow the syntax of the Microsoft GWBasic/QBasic PLAY   statement, with some exceptions and extensions.  See https://firmware.ardupilot.org/Tools/ToneTester/
const CToneAlarm::Tone CToneAlarm::_tones[] 
{
	#define AP_NOTIFY_TONE_QUIET_NEG_FEEDBACK 0
	{ "MFT200L4<<<B#A#2", false },
	#define AP_NOTIFY_TONE_LOUD_NEG_FEEDBACK 1
	{ "MFT100L4>B#A#2P8B#A#2", false },
	#define AP_NOTIFY_TONE_QUIET_NEU_FEEDBACK 2
	{ "MFT200L4<B#", false },
	#define AP_NOTIFY_TONE_LOUD_NEU_FEEDBACK 3
	{ "MFT100L4>B#", false },
	#define AP_NOTIFY_TONE_QUIET_POS_FEEDBACK 4
	{ "MFT200L4<A#B#", false },
	#define AP_NOTIFY_TONE_LOUD_POS_FEEDBACK 5
	{ "MFT100L4>A#B#", false },
	#define AP_NOTIFY_TONE_LOUD_READY_OR_FINISHED 6
	{ "MFT100L4>G#6A#6B#4", false },
	#define AP_NOTIFY_TONE_QUIET_READY_OR_FINISHED 7
	{ "MFT200L4<G#6A#6B#4", false },
	#define AP_NOTIFY_TONE_LOUD_ATTENTION_NEEDED 8
	{ "MFT100L4>A#A#A#A#", false },
	#define AP_NOTIFY_TONE_QUIET_ARMING_WARNING 9
	{ "MNT75L1O2G", false },
	#define AP_NOTIFY_TONE_LOUD_WP_COMPLETE 10
	{ "MFT200L8G>C3", false },
	#define AP_NOTIFY_TONE_LOUD_LAND_WARNING_CTS 11
	{ "MBT200L2A-G-A-G-A-G-", true },
	#define AP_NOTIFY_TONE_LOUD_VEHICLE_LOST_CTS 12
	{ "MBT200>A#1", true },
	#define AP_NOTIFY_TONE_LOUD_BATTERY_ALERT_CTS 13
	{ "MFNT255<<A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8", true },
	#define AP_NOTIFY_TONE_QUIET_COMPASS_CALIBRATING_CTS 14
	{ "MBNT255<C16P2", true },
	#define AP_NOTIFY_TONE_WAITING_FOR_THROW 15
	{ "MBNT90L4O2A#O3DFN0N0N0", true},
	#define AP_NOTIFY_TONE_LOUD_1 16
	{ "MFT100L8>B", false},
	#define AP_NOTIFY_TONE_LOUD_2 17
	{ "MFT100L8>BB", false},
	#define AP_NOTIFY_TONE_LOUD_3 18
	{ "MFT100L8>BBB", false},
	#define AP_NOTIFY_TONE_LOUD_4 19
	{ "MFT100L8>BBBB", false},
	#define AP_NOTIFY_TONE_LOUD_5 20
	{ "MFT100L8>BBBBB", false},
	#define AP_NOTIFY_TONE_LOUD_6 21
	{ "MFT100L8>BBBBBB", false},
	#define AP_NOTIFY_TONE_LOUD_7 22
	{ "MFT100L8>BBBBBBB", false},
	#define AP_NOTIFY_TONE_TUNING_START 23
	{ "MFT100L20>C#D#", false},
	#define AP_NOTIFY_TONE_TUNING_SAVE 24
	{ "MFT100L10DBDB>", false},
	#define AP_NOTIFY_TONE_TUNING_ERROR 25
	{ "MFT100L10>BBBBBBBB", false},
	#define AP_NOTIFY_TONE_LEAK_DETECTED 26
	{ "MBT255L8>A+AA-", true},
	#define AP_NOTIFY_TONE_QUIET_SHUTDOWN 27
	{ "MFMST200L32O3ceP32cdP32ceP32c<c>c<cccP8L32>c>c<P32<c<c", false },
	#define AP_NOTIFY_TONE_QUIET_NOT_READY_OR_NOT_FINISHED 28
	{ "MFT200L4<B#4A#6G#6", false },
	#define AP_NOTIFY_TONE_STARTUP 29
	{ "MFT240L8O4aO5dcO4aO5dcO4aO5dcL16dcdcdcdc", false },
	#define AP_NOTIFY_TONE_NO_SDCARD 30
	{ "MNBGG", false },
};

// default constructor
CToneAlarm::CToneAlarm()
{
} //ToneAlarm

// default destructor
CToneAlarm::~CToneAlarm()
{
} //~ToneAlarm

bool CToneAlarm::init()
{
    play_tone(AP_NOTIFY_TONE_STARTUP);
  //  Test();
	return true;
}

// play_tune - play one of the pre-defined tunes
void CToneAlarm::play_tone(const uint8_t tone_index)
{
    uint32_t tnow_ms = Core.millis();
    const Tone &tone_requested = _tones[tone_index];

    if (tone_requested.continuous) 
	{
        _cont_tone_playing = tone_index;
    }

    _tone_playing = tone_index;
    _tone_beginning_ms = tnow_ms;

    play_tune(tone_requested.str);
}

// update -   Should be called at 50Hz

void CToneAlarm::Update()
{
	  static uint32_t LastTime = 0;
	//  xit if buzzer is not enabled
// 	  if (buzzer_enabled() == false) 
// 	  {
// 		  return;
// 	  }
	  check_cont_tone();

	  // check if battery status has changed
	if (Config.m_RunningFlags.BATTERY_1_ALARM_CRITICAL  && (Core.millis() - LastTime)> 3000 ) 
	{
		// battery warning tune
		play_tone(AP_NOTIFY_TONE_LOUD_BATTERY_ALERT_CTS);
		 LastTime = Core.millis();
		return;
	}

	  // check if battery status has changed
	if (Config.m_RunningFlags.LOST_RC_SIGNAL && (Core.millis() - LastTime)> 4000 )  
	{
		  play_tone(AP_NOTIFY_TONE_LOUD_ATTENTION_NEEDED);
		 LastTime = Core.millis();
		  return;
	}




	  if (flags.powering_off) 
	  {
		  play_tone(AP_NOTIFY_TONE_QUIET_SHUTDOWN);
		  return;
	  }

	  if (flags.compass_cal_running) 
	  {
			  play_tone(AP_NOTIFY_TONE_QUIET_COMPASS_CALIBRATING_CTS);
			  play_tone(AP_NOTIFY_TONE_QUIET_POS_FEEDBACK);
	  } 
	  else
	   {
			  if (_cont_tone_playing == AP_NOTIFY_TONE_QUIET_COMPASS_CALIBRATING_CTS) 
			  {
				  stop_cont_tone();
			  }
	  }

	  if (flags.compass_cal_canceled) 
	  {
		  play_tone(AP_NOTIFY_TONE_QUIET_NEU_FEEDBACK);
		  return;
	  }

	  if (flags.initiated_compass_cal) 
	  {
		  play_tone(AP_NOTIFY_TONE_QUIET_NEU_FEEDBACK);
		  return;
	  }

	  if (flags.compass_cal_saved) 
	  {
		  play_tone(AP_NOTIFY_TONE_QUIET_READY_OR_FINISHED);
		  return;
	  }

	  if (flags.compass_cal_failed) 
	  {
		  play_tone(AP_NOTIFY_TONE_QUIET_NEG_FEEDBACK);
		  return;
	  }

	  // don't play other tones if compass cal is running
	  if (flags.compass_cal_running) 
		  return;

	  // notify the user when autotune or mission completes
	  if (flags.autotune_complete || flags.mission_complete) 
		  play_tone(AP_NOTIFY_TONE_LOUD_READY_OR_FINISHED);


	  // notify the user when a waypoint completes
	  if (flags.waypoint_complete) 
		  play_tone(AP_NOTIFY_TONE_LOUD_WP_COMPLETE);

	  // notify the user when their mode change was successful
	  if (flags.user_mode_change) 
			  play_tone(AP_NOTIFY_TONE_LOUD_NEU_FEEDBACK);

	  // notify the user when arming fails
	  if (flags.arming_failed) 
	  {
		  play_tone(AP_NOTIFY_TONE_QUIET_NEG_FEEDBACK);
	  }

	  // notify the user when RC contact is lost
	  if (flags.failsafe_radio ) 
	  {
			  // armed case handled by events.failsafe_mode_change
			  if (!Config.m_RunningFlags.ARMED) 
				  play_tone(AP_NOTIFY_TONE_QUIET_NEG_FEEDBACK);
			 else 
			  play_tone(AP_NOTIFY_TONE_LOUD_POS_FEEDBACK);
	  }

	  // notify the user when pre_arm checks are passing
		  if (flags.pre_arm_check) 
		  {
			  play_tone(AP_NOTIFY_TONE_QUIET_READY_OR_FINISHED);
			  _have_played_ready_tone = true;
		  } else 
		  {
			  // only play sad tone if we've ever played happy tone:
			  if (_have_played_ready_tone) 
				  play_tone(AP_NOTIFY_TONE_QUIET_NOT_READY_OR_NOT_FINISHED);
		  }

	  // check if arming status has changed
	  if (flags.armed != Config.m_RunningFlags.ARMED) 
	  {
		  flags.armed = Config.m_RunningFlags.ARMED;
		  if (flags.armed) 
			  play_tone(AP_NOTIFY_TONE_QUIET_ARMING_WARNING);		 // arming tune
		   else 
		   {
			  // disarming tune
			  play_tone(AP_NOTIFY_TONE_QUIET_NEU_FEEDBACK);
			  if (!flags.leak_detected) 
			  {
				  stop_cont_tone();
			  }
		 }
	  }


// 	  // lost vehicle tone
// 	  if (flags.vehicle_lost != AP_Notify::flags.vehicle_lost) 
// 	  {
// 		  flags.vehicle_lost = AP_Notify::flags.vehicle_lost;
// 		  if (flags.vehicle_lost)
// 		   {
// 			  play_tone(AP_NOTIFY_TONE_LOUD_VEHICLE_LOST_CTS);
// 			  } else {
// 			  stop_cont_tone();
// 		  }
// 	  }
	Player.Update();
}

void CToneAlarm::play_tune(const char *str)
{

    Player.stop();
    strncpy(_tone_buf, str, AP_NOTIFY_TONEALARM_TONE_BUF_SIZE);
    _tone_buf[AP_NOTIFY_TONEALARM_TONE_BUF_SIZE-1] = 0;
    Player.play(_tone_buf);
}

void CToneAlarm::stop_cont_tone()
{
    if (_cont_tone_playing == _tone_playing) 
	{
        play_tune("");
        _tone_playing = -1;
    }
    _cont_tone_playing = -1;
}

void CToneAlarm::check_cont_tone()
{
    uint32_t tnow_ms = Core.millis();
    // if we are supposed to be playing a continuous tone, and it was interrupted, and the interrupting tone has timed out, resume the continuous tone

    if (_cont_tone_playing != -1 && _tone_playing != _cont_tone_playing && tnow_ms-_tone_beginning_ms > AP_NOTIFY_TONEALARM_MAX_TONE_LENGTH_MS) 
        play_tone(_cont_tone_playing);
}



void CToneAlarm::Test()
{
		    flags. armed								= 0;
		    flags.  failsafe_battery				= 1;
		    flags.  parachute_release			= 0;
		    flags.  pre_arm_check				= 0;
		    flags.  failsafe_radio					= 0;
		    flags.  failsafe_gcs						= 0;
		    flags.  vehicle_lost						= 0;
		    flags.  compass_cal_running		= 0;
		    flags.  waiting_for_throw			= 0;
		    flags.  leak_detected					= 0;
		    flags.  powering_off					= 0;
		    flags.  compass_cal_canceled		= 0;
		    flags.  initiated_compass_cal		= 0;
		    flags.  compass_cal_saved			= 0;
		    flags.  compass_cal_failed			= 0;
		    flags.  autotune_complete			= 0;
		    flags.  mission_complete			= 0;
		    flags.  waypoint_complete			= 0;
		    flags.  user_mode_change			= 0;
		    flags.  arming_failed					= 0;

}
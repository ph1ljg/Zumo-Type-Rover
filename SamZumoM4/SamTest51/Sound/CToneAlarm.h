/* 
* ToneAlarm.h
*
* Created: 11/06/2021 10:08:45
* Author: philg
*/


#ifndef __TONEALARM_H__
#define __TONEALARM_H__

// wait 2 seconds before assuming a tone is done and continuing the continuous tone
#define AP_NOTIFY_TONEALARM_MAX_TONE_LENGTH_MS 2000
#define AP_NOTIFY_TONEALARM_TONE_BUF_SIZE 100


class CToneAlarm
{
//variables
public:
protected:
private:
    /// tonealarm_type - Bitmask of states we track
    struct tonealarm_type 
	{
	    uint16_t armed						: 1;    // 0 = disarmed, 1 = armed
	    uint16_t failsafe_battery		: 1;    // 1 if battery failsafe
	    uint16_t parachute_release   : 1;    // 1 if parachute is being released
	    uint16_t pre_arm_check       : 1;    // 0 = failing checks, 1 = passed
	    uint16_t failsafe_radio			: 1;    // 1 if radio failsafe
	    uint16_t failsafe_gcs				: 1;    // 1 if gcs failsafe
	    uint16_t vehicle_lost				: 1;    // 1 if lost copter tone requested
	    uint16_t compass_cal_running   : 1;    // 1 if compass calibration is running
	    uint16_t waiting_for_throw    : 1;    // 1 if waiting for copter throw launch
	    uint16_t leak_detected			: 1;    // 1 if leak detected
	    uint16_t powering_off			: 1;    // 1 if smart battery is powering off
		uint16_t compass_cal_canceled :1;
		uint16_t initiated_compass_cal :1;
		uint16_t compass_cal_saved :1;
		uint16_t compass_cal_failed :1;
		uint16_t autotune_complete :1;
		uint16_t mission_complete :1;
		uint16_t waypoint_complete :1;
		uint16_t user_mode_change :1;
		uint16_t arming_failed :1;
    } flags;
    bool _have_played_ready_tone : 1;

    int8_t _cont_tone_playing;
    int8_t _tone_playing;
    uint32_t _tone_beginning_ms;

    struct Tone 
	{
	    const char *str;
	    const uint8_t continuous : 1;
    };

    const static Tone _tones[];
    char _tone_buf[AP_NOTIFY_TONEALARM_TONE_BUF_SIZE];


//functions
public:
	CToneAlarm();
	~CToneAlarm();
	bool init();
	void play_tone(const uint8_t tone_index);
	void Update();
	void play_tune(const char *str);
	void stop_cont_tone();
	void check_cont_tone();
	void Test();
protected:
private:
	CToneAlarm( const CToneAlarm &c );
	CToneAlarm& operator=( const CToneAlarm &c );

}; //ToneAlarm

#endif //__TONEALARM_H__

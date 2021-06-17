/* 
* COsd.h
*
* Created: 08/04/2019 19:17:39
* Author: phil
*/


#ifndef __COSD_H__
#define __COSD_H__


#define LINE_SET_STRAIGHT__	(0xC7 - 1)		// code of the first MAX7456 straight char -1
#define LINE_SET_STRAIGHT_O	(0xD0 - 3)		// code of the first MAX7456 straight overflow char -3
#define LINE_SET_P___STAG_1	(0xD1 - 1)		// code of the first MAX7456 positive staggered set 1 char -1
#define LINE_SET_P___STAG_2	(0xDA - 1)		// code of the first MAX7456 positive staggered set 2 char -1
#define LINE_SET_N___STAG_1	(0xE3 - 1)		// code of the first MAX7456 negative staggered set 1 char -1
#define LINE_SET_N___STAG_2	(0xEC - 1)		// code of the first MAX7456 negative staggered set 2 char -1
#define LINE_SET_P_O_STAG_1	(0xF5 - 2)		// code of the first MAX7456 positive overflow staggered set 1 char -2
#define LINE_SET_P_O_STAG_2	(0xF9 - 1)		// code of the first MAX7456 positive overflow staggered set 2 char -1
#define LINE_SET_N_O_STAG_1	(0xF7 - 2)		// code of the first MAX7456 negative overflow staggered set 1 char -2
#define LINE_SET_N_O_STAG_2	(0xFC - 1)		// code of the first MAX7456 negative overflow staggered set 2 char -1

#define ANGLE_1			9			// angle above we switch to line set 1
#define ANGLE_2			25			// angle above we switch to line set 2

#define AH_PITCH_FACTOR		0.010471976		// conversion factor for pitch
#define AH_ROLL_FACTOR		0.017453293		// conversion factor for roll
#define AH_COLS			10			// number of artificial horizon columns
#define AH_ROWS			4			// number of artificial horizon rows
#define CHAR_COLS		12			// number of MAX7456 char columns
#define CHAR_ROWS		18			// number of MAX7456 char rows
#define CHAR_SPECIAL		9			// number of MAX7456 special chars for the artificial horizon
#define AH_TOTAL_LINES		AH_ROWS * CHAR_ROWS	// helper define
#define OVERFLOW_CHAR_OFFSET	6			// offset for the overflow subvals

// Panel 8bit REGISTER with BIT positions
// panA_REG Byte has:
//#define Cen_BIT        0
#define Pit_BIT        1
#define Rol_BIT        2
#define BatA_BIT       3
#define Bp_BIT         4
#define GPSats_BIT     5
#define COG_BIT        6
#define GPS_BIT        7

// panB_REG Byte has:
#define Rose_BIT       0
#define Head_BIT       1
//#define MavB_BIT       2
#define HDir_BIT       3
#define HDis_BIT       4
//#define WDir_BIT       5 //
#define WDis_BIT       6 //
#define Time_BIT       7

// panC_REG Byte has:
#define CurA_BIT       0
#define As_BIT         1
#define Alt_BIT        2
#define Vel_BIT        3
#define Thr_BIT        4
#define FMod_BIT       5
#define Hor_BIT        6
#define Halt_BIT       7

// panD_REG Byte has:
#define Warn_BIT       0
#define Off_BIT        1
#define WindS_BIT      2
#define Climb_BIT      3
//#define Tune_BIT       4
#define CALLSIGN_BIT   5
#define RSSI_BIT       6
#define Eff_BIT        7

// panE_REG Byte has:

//#define Ch_BIT         0
#define TEMP_BIT       1
#define DIST_BIT       2

#define NO_OF_PANELS	2


typedef struct
{
	uint8_t X;
	uint8_t Y;
	bool Enable;
}PannelCoords_t;

typedef struct
{

	PannelCoords_t panBatteryPercent;
	PannelCoords_t panPitch; // = { 11,1 };
	PannelCoords_t panRoll; // = { 23,7 };
	PannelCoords_t panBatt; // = { 23,1 };
	PannelCoords_t panVtxVal; // = { 23,1 };
	PannelCoords_t panGPSats; // = { 2,12 };
	PannelCoords_t panGPS; // = { 2,13 };
	PannelCoords_t panCOG; // = { 2,11 };

	//Second 8 set of panels and their X,Y coordinate holders
	PannelCoords_t panRose; // = { 16,13 };
	PannelCoords_t panHeading; // = { 16,12 };
	PannelCoords_t panHomeDir; // = { 0,0 };
	PannelCoords_t panHomeDis; // = { 0,0 };
	PannelCoords_t panWPDis; // = { 23,11 };
	PannelCoords_t panTime;

	// Third set of panels and their X,Y coordinate holders
	PannelCoords_t panCur_A; // = { 23,1 };
	PannelCoords_t panAlt; // = { 0,0 };
	PannelCoords_t panHomeAlt; // = { 0,0 };
	PannelCoords_t panVel; // = { 0,0 };
	PannelCoords_t PanBoatSpeed; // = { 0,0 };
	PannelCoords_t panThr; // = { 0,0 };
	PannelCoords_t PanNavMod; // = { 0,0 };
	PannelCoords_t panHorizon; // = {8,centercalc}

	// Third set of panels and their X,Y coordinate holders
	PannelCoords_t PanAlarms;
	PannelCoords_t panWindSpeed;
	PannelCoords_t panClimb;
	PannelCoords_t panRSSI;
	PannelCoords_t panEff;
	PannelCoords_t panCALLSIGN;
	PannelCoords_t panTemp;
	PannelCoords_t panDistance;
	PannelCoords_t panOff;
	PannelCoords_t panMess;
	PannelCoords_t PanLogging;
	PannelCoords_t panFrontDistance;
	
}Pannel_t;



class COsd
{
//variables
public:
	float        osd_heading ;              // ground course heading from GPS
	float        glide;

	float        osd_alt_abs;               // altitude
	float        osd_alt_rel;               // altitude
	float        osd_alt_gps;               // altitude
	float        osd_airspeed;              // airspeed
	float        osd_windspeed;
	float        osd_windspeedz;
//	float        osd_winddirection;
	uint8_t      osd_satellites_visible;    // number of satellites
	uint8_t      osd_fix_type;              // GPS lock 0-1=no fix, 2=2D, 3=3D
	uint16_t     osd_cog;                   // Course over ground
	uint16_t     off_course;
	uint8_t      osd_got_home;              // tels if got home position or not
	float        osd_home_lat;              // home latitude
	float        osd_home_lon;              // home longitude
	long         osd_home_distance;         // distance from home
	uint8_t      osd_home_direction;        // Arrow direction pointing to home (1-16 to CW loop)
	float        osd_groundspeed;           // ground speed
	//	float        osd_alt_rel;                    // altitude
	int16_t      osd_curr_A;                // Battery A current
	uint8_t      osd_battery_remaining_A;   // 0 to 100 <=> 0 to 1000
	uint8_t      batt_warn_level;
	uint16_t     osd_throttle[4];           //  throttle
	int16_t	    wp_target_bearing;			// Bearing to current MISSION/target in degrees
	int16_t		gps_Hdop;
	uint8_t		nav_state;
	uint8_t		action;
	uint8_t		wp_number;
	uint8_t		nav_error;
	float		tdistance;
	float       mah_used;
	uint32_t	total_flight_time_milis;
	uint16_t    total_flight_time_seconds;
	float        max_home_distance;
	float        max_osd_airspeed;
	float        max_osd_groundspeed;
	float        max_osd_home_alt;
	float        max_osd_windspeed;
	float        nor_osd_windspeed;
	bool  motor_armed;
	uint16_t     eph;
	float   xtrack_error; // Current crosstrack error on x-y plane in meters
	uint16_t     wp_dist; // Distance to active MISSION in meters
	uint8_t      osd_rssi;
	int16_t      rssi; // scaled value 0-100%
	uint16_t	m_RcChannels[10];
	uint16_t     temperature;
	bool         last_armed_status;
	float		vs;
	float        osd_climb;
	uint8_t		buf_show[12];
	// First 8 panels and their X,Y coordinate holders
	bool canswitch;
	uint16_t  ch_raw;
	bool ClearOsd;
	uint8_t osd_mode;
	uint8_t osd_off_switch;
	int8_t osd_COG_arrow_rotate_int;
	float converts;
	float converth;
	uint8_t spe;
	uint8_t high;
	int16_t temps;
	uint8_t tempconv;
	uint16_t tempconvAdd;
	uint8_t distchar;
	uint8_t climbchar;
	uint16_t distconv;
	int8_t wp_target_bearing_rotate_int;
	int8_t osd_wind_arrow_rotate_int;
	float osd_winddirection;
//	float nor_osd_windspeed;
	Pannel_t Panel1;
	Pannel_t Panel2;
	Pannel_t Panel3;
	uint8_t PresentPanel;
	int8_t  m_LastBatteryReading;		// 0 to 100 <=> 0 to 1000
	int8_t  m_MaxBatteryReading;		// 0 to 100 <=> 0 to 1000
	bool m_Blinker;
	bool OneSecTimerSwitch;
	uint8_t   m_CurrentPanel; //0 - Normal OSD; 1 - Flight summary; 2 - No telemetry data (pre-set = 255 to force osd.clear() after boot screen
	char m_Messages[6][26];
	bool m_AllowUpdate;
protected:
private:

//functions
public:
	COsd();
	~COsd();
	void Init();
	void Start();
	void Update();
	void panLogo();
	void DisplayFont();
	void writePanels();
	void PanWaitForTelemetry(int first_col, int first_line);
	void panFdata();
	void PanAlarms(int first_col, int first_line);
	void SetAlarmText(char *Alarm,char *Message);
	void SetPanelState(uint8_t State);
	void RotatePanelState();
	void PanPitch(int first_col, int first_line);
	void PanLogState(int first_col, int first_line);
	void PanRoll(int first_col, int first_line);
	void PanBattVolts(int first_col, int first_line);
	void PanVtx(int first_col, int first_line);
	void panGPSats(int first_col, int first_line);
	void panGPS(int first_col, int first_line);
	void panBatteryPercent(int first_col, int first_line,bool Type);
	void panWPDis(int first_col, int first_line);
	void panVel(int first_col, int first_line);
	void panCOG(int first_col, int first_line);
	void panHeading(int first_col, int first_line);
	void panHomeDis(int first_col, int first_line);
	void panBoatSpeed(int first_col, int first_line);
	void panThr(int first_col, int first_line);
	void PanNavigationMode(int first_col, int first_line);
	void panCur_A(int first_col, int first_line);
	void panWindSpeed(int first_col, int first_line);
	void panRSSI(int first_col, int first_line);
	void panEff(int first_col, int first_line);
	void panDistance(int first_col, int first_line);
	void PanFrontDistance(int first_col, int first_line);
	void panTemp(int first_col, int first_line);
	void panHorizon(int first_col, int first_line);
	void ShowHorizon(int start_col, int start_row);
	void ShowArrow(uint8_t rotate_arrow,uint8_t method);
	void SetHeadingPattern(uint8_t *Buffer);
	void panRose(int first_col, int first_line);
	void panHomeDir(int first_col, int first_line);
	void panTime(int first_col, int first_line);
	void panMessage(int first_col, int first_line);
	void AddMessage(const char *Message);
	uint16_t IsInMessageQue(char* Message);
	void UpdatePanelCords();
protected:
private:
	COsd( const COsd &c );
	COsd& operator=( const COsd &c );

}; //COsd

#endif //__COSD_H__

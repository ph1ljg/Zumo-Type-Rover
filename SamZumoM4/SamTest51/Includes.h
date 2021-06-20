/*
 * Includes.h
 *
 * Created: 19/09/2020 13:22:47
 *  Author: philg
 */ 


#ifndef INCLUDES_H_
#define INCLUDES_H_

//======================= MODEL= ===============================
//#define FOUR_WHEEL_ZUMO
#define TWO_WHEEL_ZUMO
//#define  LARGE_ROVER
//============================================================


#include "sam.h"
#include "samd51.h"
#include "float.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "CMyMath.h"
#include "stdio.h"
#include "Compiler.h"
#include "samd51j19a.h"
#include "string.h"
#include "stdarg.h"
#include "sam.h"
#include "Device_Startup/clocks.h"
#include "stdlib.h"
#include <assert.h>
#include "stddef.h"
#include "vector3.h"
#include "Vector2.h"



#include "Defines.h"
#include "CAlarm.h"
#include "CZumo.h"
#include "CCore.h"
#include "CUart.h"
#include "CSercom.h"
#include "CSetInterupts.h"
#include "CObjectBuffer.h"
#include "Vector3.h"
#include "matrix3.h"
#include "CPid_P.h"
#include "CPid_PID.h"
#include "CPinPeripheral.h"
#include "CSpi.h"
#include "CI2c.h"
#include "CServo.h"
#include "CSoftwareSerial.h"
#include "CDebugDisplay.h"
#include "CWS2812.h"
#include "cconfig.h"
#include "CCirBuffer.h"
#include "Cpwm.h"
#include "CUart.h"
#include "CAnalog.h"
#include "CPCF8574.h"
#include "CGuiFunctions.h"
#include "CMotors.h"
#include "CDebugDisplay.h"
#include "CWS2812.h"
#include "CQSpi.h"
#include "QSpiFlash/CQSpiFlash.h"
#include "QSpiFlashDevices.h"
#include "CNavigation.h"
#include "CNavigationFunctions.h"
#include "CRadioControl.h"
#include "CFrskyTelemetry.h"
#include "CSbusDecoder.h"
#include "CAvoidance.h"
#include "CTaskManager.h"
#include "CAhrs.h"
#include "CSensors.h"
#include "CCmps12.h"
#include "CUblox.h"
#include "CGps.h"
#include "CBN0055.h"
#include "CGuiFunctions.h"
#include "CHeadControl.h"
#include "COsd.h"
#include "CMax7456.h"
//#include "CTrampProtocol.h"
//#include "CNavigationFunctions.h"
#include "CStatusControl.h"
#include "CTFMiniPlus.h"
#include "CGps.h"
#include "CTrampProtocol.h"
#include "CVL53OX.h"
#include "CPidSimp.h"
#include "CBuzzer.h"
#include "CWheelEncoder.h"
#include "CVL53L1X.h"
#include "CObjAvoidDbase.h"
#include "CFence.h"

#include "CBendyRuler.h"
#include "CPathPlanner.h"

#include "CSteering.h"
#include "CL1Control.h"
#include "CHuskyProtocol.h"
#include "CHuskyMain.h"
#include "Imu.h"
#include "CAttitudeControl.h"
#include "CProximity.h"
#include "CStatusControl.h"
#include "CCompass.h"
#include "CLedDriver.h"
#include "CSensors.h"
#include "CTone.h"


extern CPinPeripheral PinPeripheral;
extern CZumo Zumo;
extern CNavigationFunctions NavigationFunctions;
extern CWS2812 WS2812;

//extern CUart Uart0;
extern CCore Core;
extern CI2c I2c;
extern CSoftwareSerial SoftwareSerial;
extern CWS2812 WS2812;
extern CDebugDisplay DebugDisplay;
extern CTaskManager TaskManager;
extern CConfig Config;
extern CGps Gps;
extern CMotors Motors;
extern CSpi Spi_1;
extern CSpi Spi;
extern CAlarm Alarms;
extern COsd Osd;
extern CAvoidance Avoidance;
//extern CPid_PID SteeringRatePid;
extern CNavigation Navigation;
extern CGuiFunctions GuiFunctions;
extern CSensors Sensors;
extern CRadioControl RadioControl;
extern CPidSimp Pid;
extern const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM];
extern CSercom SercomCommon;
extern CMax7456 Max7456;
extern CSoftwareSerial TrampSoftwareSerial;
extern CSoftwareSerial DebugSerial;
extern CSbusDecoder SbusDecoder;
extern CUart Uart;
extern CUart SbusUart;
extern CFrskyTelemetry FrskyTelemetry;
extern CStatusControl StatusControl;
extern CSoftwareSerial CameraSoftwareSerial;
extern CUblox Ublox;
extern CHeadControl HeadControl;
extern CPid_P SteeringAnglePid;
extern CAhrs Ahrs;
extern CBN0055 BN0055;
extern CSoftwareSerial TeleSoftwareSerial;
extern CBuzzer Buzzer;
extern CAvoidance Avoidance;
extern CWheelEncoder WheelEncoder;
extern CSteering Steering;
extern CSetInterupts SetInterupts;
extern CPCF8574 PCF8574;
extern CHuskyMain HuskyMain;
extern CHuskyProtocol HuskyProtocol;
extern CServo VertServo;
extern CServo HorizServo;
extern CImu Imu;
extern CAttitudeControl AttitudeControl;
extern CCompass Compass;
extern CSensors Sensors;
extern CTone Tone;
#endif /* INCLUDES_H_ */

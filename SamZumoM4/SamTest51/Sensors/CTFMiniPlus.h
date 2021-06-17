/* 
* CTFMiniPlus.h
*
* Created: 13/07/2020 18:55:38
* Author: philg
*/


#ifndef __CTFMINIPLUS_H__
#define __CTFMINIPLUS_H__

#define  TFMINPLUS_ADDRESS				0x10
#define TFMINI_MAX_MEASUREMENT_ATTEMPTS 10
#define INTERFACE_SERIAL				0
#define INTERFACE_I2C					1

#define DATA_FRAME_MARKER				0x59
#define DATA_FRAME_LENGTH				9
#define CMD_FRAME_MARKER				0x5A
#define MAX_CMD_RESPONSE_LENGTH			8



typedef struct __attribute__ ((packed))
{
  uint8_t FrameHeader1;
  uint8_t FrameHeader2;
  uint8_t DistanceLower;
  uint8_t DistanceHigher;
  uint8_t StrengthLower;
  uint8_t StrengthHigher;
  uint8_t TempLower;
  uint8_t TempHigher;
  uint8_t ChkSum;
}Response1_t;

typedef struct __attribute__ ((packed))
{
  uint8_t FrameHeader1;
  uint8_t FrameHeader2;
  uint16_t Distance;
  uint16_t Strength;
  uint16_t Temp;
  uint8_t ChkSum;
}TFminiPlusResponse_t;



typedef union  __attribute__ ((packed))
{
	TFminiPlusResponse_t Response;
	uint8_t Buffer[9];
}TfMiniPlusFrame_t;

typedef struct
{
	uint16_t Distance;
	uint16_t Strength;
	uint16_t Temperature;
	bool MeasurementValid;
}TfminiPlusData_t;

class CTFMiniPlus
{
//variables
public:
	TfminiPlusData_t m_Results;
protected:
private:
	uint8_t NumMeasurementAttempts;
	TfMiniPlusFrame_t m_ResponseFrame;
	uint8_t GetVersionCommand[4] = {CMD_FRAME_MARKER, 0x04, 0x01, 0x5F};
	uint8_t ResetCommand[4] = {CMD_FRAME_MARKER, 0x04, 0x02, 0x60};
	uint8_t TriggerDetectionCommand[4] = {CMD_FRAME_MARKER, 0X04, 0x04, 0x62};
	uint8_t RestoreFactorySettingsCommand[4] = {CMD_FRAME_MARKER, 0x04, 0x10, 0x6E};
	uint8_t SaveSettingsCommand[4] = {CMD_FRAME_MARKER, 0x04, 0x11, 0x6F};

//functions
public:
	CTFMiniPlus();
	~CTFMiniPlus();
	void Init();
	void UpDate();
	bool GetRange(TfminiPlusData_t &Data);
	bool UpdateDistance();
	bool GetDistance(uint16_t *Distance);
	bool GetSensorRawTemperature(uint16_t *Temp);
	bool GetSensorTemperature(double * Temp);
	bool GetSignalStrength(uint16_t * Stren);
	bool GetVersion(uint32_t &Version);
	bool SystemReset();
	bool SetFrameRate(uint16_t FrameRate);
	void TriggerDetection();
	bool SetInterface(uint8_t Interface);
	bool SetOutputFormat(uint16_t Format);
	bool SetBaudRate(uint32_t baud);
	bool SetEnabled(bool state);
	bool RestoreFactorySettings();
	bool SaveSettings();
	bool ValidateChecksum( uint8_t dataBuffer[], size_t length);
	uint8_t GenerateChecksum(const uint8_t buffer[], size_t length);
protected:
private:
	CTFMiniPlus( const CTFMiniPlus &c );
	CTFMiniPlus& operator=( const CTFMiniPlus &c );

}; //CTFMiniPlus

#endif //__CTFMINIPLUS_H__

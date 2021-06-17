/* 
* CSeeedLidar.h
*
* Created: 17/06/2019 17:49:34
* Author: phil
*/


#ifndef __TFMINILIDAR_H__
#define __CTFMINILIDAR_H__

#define TFMINI_ADDRESS 0x10

#define TFMINI_FRAME_SIZE    7
// Timeouts
#define TFMINI_MAXBYTESBEFOREHEADER       30
#define TFMINI_MAX_MEASUREMENT_ATTEMPTS   10

// States
#define READY                             0
#define ERROR_SERIAL_NOHEADER             1
#define ERROR_SERIAL_BADCHECKSUM          2
#define ERROR_SERIAL_TOOMANYTRIES         3
#define MEASUREMENT_OK                    10

typedef struct __attribute__ ((packed))
{
  uint8_t FrameHeader1;
  uint8_t FrameHeader2;
  uint16_t Distance;
  uint16_t Strength;
  uint16_t Temp;
  uint8_t ChkSum;
}TFMiniResponse_t;



typedef union  __attribute__ ((packed))
{
	TFMiniResponse_t Response;
	uint8_t Buffer[9];
}TfMiniFrame_t;

typedef struct
{
	uint16_t Distance;
	uint16_t Strength;
	uint16_t Temperature;
	bool MeasurementValid;
}TfminiData_t;

class CTFMini
{
//variables
public:
	uint16_t m_Distance;
	uint16_t m_Strength;
	uint16_t m_Quality;
	bool m_MeasurementValid;
	uint8_t m_ErrorState;
	 uint16_t ErrCount =0;
 	TfminiData_t m_Results;

protected:
private:
	uint8_t NumMeasurementAttempts;
	TfMiniFrame_t m_ResponseFrame;
	uint8_t GetVersionCommand[4] = {CMD_FRAME_MARKER, 0x04, 0x01, 0x5F};
	uint8_t ResetCommand[4] = {CMD_FRAME_MARKER, 0x04, 0x02, 0x60};
	uint8_t TriggerDetectionCommand[4] = {CMD_FRAME_MARKER, 0X04, 0x04, 0x62};
	uint8_t RestoreFactorySettingsCommand[4] = {CMD_FRAME_MARKER, 0x04, 0x10, 0x6E};
	uint8_t SaveSettingsCommand[4] = {CMD_FRAME_MARKER, 0x04, 0x11, 0x6F};

//functions
public:
	CTFMini();
	~CTFMini();
	
	void Init();
	void UpDate();
	bool SetDefault();
	bool GetUpdateDistanceI2c();
	bool ValidateChecksum( uint8_t dataBuffer[], size_t length);
	uint8_t GenerateChecksum(const uint8_t buffer[], size_t length);
	bool UpdateDistance();
	void SetStandardOutputMode();
	void SetConfigMode();
	void SetSingleScanMode();
	void ExternalTrigger();
protected:
private:
	CTFMini( const CTFMini &c );
	CTFMini& operator=( const CTFMini &c );
	

}; //CTfminLidar
#endif //__CTFMINILIDAR_H__

/* 
* HuskyFunctions.h
*
* Created: 27/01/2021 18:43:13
* Author: philg
*/


#ifndef __HUSKYFUNCTIONS_H__
#define __HUSKYFUNCTIONS_H__


enum protocolCommand
{
	COMMAND_REQUEST = 0x20,
	COMMAND_REQUEST_BLOCKS,
	COMMAND_REQUEST_ARROWS,
	COMMAND_REQUEST_LEARNED,
	COMMAND_REQUEST_BLOCKS_LEARNED,
	COMMAND_REQUEST_ARROWS_LEARNED,
	COMMAND_REQUEST_BY_ID,
	COMMAND_REQUEST_BLOCKS_BY_ID,
	COMMAND_REQUEST_ARROWS_BY_ID,

	COMMAND_RETURN_INFO,
	COMMAND_RETURN_BLOCK,
	COMMAND_RETURN_ARROW,

	COMMAND_REQUEST_KNOCK,
	COMMAND_REQUEST_ALGORITHM,

	COMMAND_RETURN_OK,
	COMMAND_REQUEST_CUSTOMNAMES,
	COMMAND_REQUEST_PHOTO,
	COMMAND_REQUEST_SEND_PHOTO,
	COMMAND_REQUEST_SEND_KNOWLEDGES,
	COMMAND_REQUEST_RECEIVE_KNOWLEDGES,
	COMMAND_REQUEST_CUSTOM_TEXT,
	COMMAND_REQUEST_CLEAR_TEXT,

	COMMAND_REQUEST_LEARN,
	COMMAND_REQUEST_FORGET,

	COMMAND_REQUEST_SEND_SCREENSHOT,
	COMMAND_REQUEST_SAVE_SCREENSHOT,
	COMMAND_REQUEST_LOAD_AI_FRAME_FROM_USB,
	COMMAND_REQUEST_IS_PRO,
	COMMAND_REQUEST_FIRMWARE_VERSION = 0x3C,

	COMMAND_REQUEST_SENSOR,
};

typedef struct
{
	uint8_t nameDataSize;
	uint8_t id;
	// uint8_t numOfNames;
	// uint8_t algorithim;
	char dataBuffer[21];
} ProtocolCustomNameHeader_t;

typedef struct
{
	uint8_t *dataBuffer;
} ProtocolReceivedKnowledges_t;

typedef struct
{
	uint16_t x;
	uint8_t y;
	uint8_t textSize;
	uint8_t text[21];
} ProtocolCustomText_t;

typedef struct
{
	uint8_t length;
	uint8_t * data;
} ProtocolFirmwareVersion_t;


typedef	union
	{
		int16_t first;
		int16_t xCenter;
		int16_t xOrigin;
		int16_t protocolSize;
		int16_t algorithmType;
		int16_t requestID;
	}UFirst;
typedef	union
	{
		int16_t second;
		int16_t yCenter;
		int16_t yOrigin;
		int16_t knowledgeSize;
	}USecond;
typedef	union
	{
		int16_t third;
		int16_t width;
		int16_t xTarget;
		int16_t frameNum;
	}UThird;
typedef	union
	{
		int16_t fourth;
		int16_t height;
		int16_t yTarget;
	}UFourth;
typedef	union
	{
		int16_t fifth;
		int16_t ID;
	}UFifth;



typedef struct
{
	uint8_t command;
	
	UFirst	First;
	USecond Second;
	UThird	Third;
	UFourth Fourth;
	UFifth	Fifth;

	ProtocolCustomNameHeader_t customNameHeader;
	ProtocolReceivedKnowledges_t receivedKnowledges;
	ProtocolCustomText_t customText;
	ProtocolFirmwareVersion_t firmwareVersion;
} Protocol_t;

enum protocolAlgorithm
{
	ALGORITHM_FACE_RECOGNITION,
	ALGORITHM_OBJECT_TRACKING,
	ALGORITHM_OBJECT_RECOGNITION,
	ALGORITHM_LINE_TRACKING,
	ALGORITHM_COLOR_RECOGNITION,
	ALGORITHM_TAG_RECOGNITION,
	ALGORITHM_OBJECT_CLASSIFICATION,
};

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

typedef Protocol_t HUSKYLENSResult;


class HuskyFunctions
{
//variables
public:
protected:
private:

//functions
public:
	HuskyFunctions();
	~HuskyFunctions();
	bool begin();
	bool protocolWrite(uint8_t *buffer, int length);
protected:
private:
	unsigned long timeOutDuration = 100;
	unsigned long timeOutTimer;
	int16_t currentIndex = 0;
	Protocol_t protocolCache;

	HuskyFunctions( const HuskyFunctions &c );
	HuskyFunctions& operator=( const HuskyFunctions &c );

}; //HuskyFunctions

#endif //__HUSKYFUNCTIONS_H__

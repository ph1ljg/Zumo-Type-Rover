/* 
* HuskyProtocol.h
*
* Created: 27/01/2021 17:50:20
* Author: philg
*/


#ifndef __HUSKYPROTOCOL_H__
#define __HUSKYPROTOCOL_H__

#define IS_BIG_ENDIAN() (!*(uint8_t *)&(uint16_t){1})


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

typedef struct
{
	uint8_t command;
	union
	{
		int16_t first;
		int16_t xCenter;
		int16_t xOrigin;
		int16_t protocolSize;
		int16_t algorithmType;
		int16_t requestID;
	};
	union
	{
		int16_t second;
		int16_t yCenter;
		int16_t yOrigin;
		int16_t knowledgeSize;
	};
	union
	{
		int16_t third;
		int16_t width;
		int16_t xTarget;
		int16_t frameNum;
	};
	union
	{
		int16_t fourth;
		int16_t height;
		int16_t yTarget;
	};
	union
	{
		int16_t fifth;
		int16_t ID;
	};

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






class CHuskyProtocol
{
//variables
public:
protected:
private:

//functions
public:
	CHuskyProtocol();
	~CHuskyProtocol();
	bool validateCheckSum();
	bool husky_lens_protocol_receive(uint8_t data);
	bool husky_lens_protocol_read_begin(uint8_t command);
	uint8_t husky_lens_protocol_read_uint8();
	int16_t husky_lens_protocol_read_int16();
	int32_t husky_lens_protocol_read_int32();
	float husky_lens_protocol_read_float();
	bool husky_lens_protocol_read_end();

	uint8_t* husky_lens_protocol_write_begin(uint8_t command);
	void husky_lens_protocol_write_uint8(uint8_t content);
	void husky_lens_protocol_write_int16(int16_t content);
	void husky_lens_protocol_write_int32(int32_t content);
	void husky_lens_protocol_write_float(float content);
	void husky_lens_protocol_write_buffer_uint8(uint8_t *content, uint32_t length);
	int husky_lens_protocol_write_end();


protected:
private:
	CHuskyProtocol( const CHuskyProtocol &c );
	CHuskyProtocol& operator=( const CHuskyProtocol &c );

}; //HuskyProtocol

#endif //__HUSKYPROTOCOL_H__

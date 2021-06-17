/* 
* HuskyMain.h
*
* Created: 27/01/2021 18:39:13
* Author: philg
*/


#ifndef __HUSKYMAIN_H__
#define __HUSKYMAIN_H__

#define HUSKY_ADDRESS 0x32




class CHuskyMain
{
	
//variables
public:
protected:
	

private:
	unsigned long timeOutDuration = 100;
	unsigned long timeOutTimer;
	int16_t currentIndex = 0;
	Protocol_t protocolCache;
	Protocol_t protocolInfo;
	Protocol_t *protocolPtr = NULL;
	HUSKYLENSResult resultDefault;

//functions
public:
	CHuskyMain();
	~CHuskyMain();

	bool Init();
	bool request();
	bool request(int16_t ID);
	bool requestBlocks();
	bool requestBlocks(int16_t ID);


	bool requestArrows();
	bool requestArrows(int16_t ID);
	bool requestLearned();
	bool requestBlocksLearned();
	bool requestArrowsLearned();
	int available();
	HUSKYLENSResult read();
	bool isLearned();
	bool isLearned(int ID);
	int16_t frameNumber();
	int16_t countLearnedIDs();
	void setTimeOutDuration(unsigned long timeOutDurationInput){timeOutDuration = timeOutDurationInput;};
	int16_t count(){return protocolInfo.protocolSize;};

	#define PROTOCOL_CREATE(function, type, command)       \
	void protocolWrite##function(Protocol_t &protocol) \
	{                                                  \
		protocolWrite##type(protocol, command);        \
	}                                                  \
	void protocolWrite##function()                     \
	{                                                  \
		Protocol_t protocol;                           \
		protocolWrite##type(protocol, command);        \
	}                                                  \
	bool protocolRead##function(Protocol_t &protocol)  \
	{                                                  \
		return protocolRead##type(protocol, command);  \
	}                                                  \
	bool protocolRead##function()                      \
	{                                                  \
		Protocol_t protocol;                           \
		return protocolRead##type(protocol, command);  \
	}



	PROTOCOL_CREATE(Request, Command, COMMAND_REQUEST)
	PROTOCOL_CREATE(RequestBlocks, Command, COMMAND_REQUEST_BLOCKS)
	PROTOCOL_CREATE(RequestArrows, Command, COMMAND_REQUEST_ARROWS)

	PROTOCOL_CREATE(RequestLearned, Command, COMMAND_REQUEST_LEARNED)
	PROTOCOL_CREATE(RequestBlocksLearned, Command, COMMAND_REQUEST_BLOCKS_LEARNED)
	PROTOCOL_CREATE(RequestArrowsLearned, Command, COMMAND_REQUEST_ARROWS_LEARNED)

	PROTOCOL_CREATE(RequestByID, OneInt16, COMMAND_REQUEST_BY_ID)
	PROTOCOL_CREATE(RequestBlocksByID, OneInt16, COMMAND_REQUEST_BLOCKS_BY_ID)
	PROTOCOL_CREATE(RequestArrowsByID, OneInt16, COMMAND_REQUEST_ARROWS_BY_ID)

	PROTOCOL_CREATE(ReturnInfo, FiveInt16, COMMAND_RETURN_INFO)
	PROTOCOL_CREATE(ReturnBlock, FiveInt16, COMMAND_RETURN_BLOCK)
	PROTOCOL_CREATE(ReturnArrow, FiveInt16, COMMAND_RETURN_ARROW)

	PROTOCOL_CREATE(RequestKnock, Command, COMMAND_REQUEST_KNOCK)
	PROTOCOL_CREATE(RequestAlgorithm, OneInt16, COMMAND_REQUEST_ALGORITHM)

	PROTOCOL_CREATE(ReturnOK, Command, COMMAND_RETURN_OK)

	PROTOCOL_CREATE(RequestLearn, OneInt16, COMMAND_REQUEST_LEARN)
	PROTOCOL_CREATE(RequestForget, Command, COMMAND_REQUEST_FORGET)

	PROTOCOL_CREATE(RequestSensor, FiveInt16, COMMAND_REQUEST_SENSOR)

	PROTOCOL_CREATE(RequestCustomNames, CustomNameHeader, COMMAND_REQUEST_CUSTOMNAMES)
	PROTOCOL_CREATE(RequestPhoto, Command, COMMAND_REQUEST_PHOTO)
	PROTOCOL_CREATE(RequestSendKnowledges, OneInt16, COMMAND_REQUEST_SEND_KNOWLEDGES)
	PROTOCOL_CREATE(RequestReceiveKnowledges, OneInt16, COMMAND_REQUEST_RECEIVE_KNOWLEDGES)
	PROTOCOL_CREATE(RequestCustomText, CustomTextRecv, COMMAND_REQUEST_CUSTOM_TEXT)
	PROTOCOL_CREATE(RequestClearText, Command, COMMAND_REQUEST_CLEAR_TEXT)
	PROTOCOL_CREATE(RequestSaveScreenshot, Command, COMMAND_REQUEST_SAVE_SCREENSHOT)
	PROTOCOL_CREATE(RequestIsPro, OneInt16, COMMAND_REQUEST_IS_PRO)
	PROTOCOL_CREATE(RequestFirmwareVersion, FirmwareVersion, COMMAND_REQUEST_FIRMWARE_VERSION);
	int16_t count(int16_t ID);
	int16_t countBlocks();
	int16_t countBlocks(int16_t ID);
	int16_t countArrows();
	int16_t countArrows(int16_t ID);
	int16_t countLearned();
	int16_t countBlocksLearned();
	int16_t countArrowsLearned();
	HUSKYLENSResult get(int16_t index);
	HUSKYLENSResult get(int16_t ID, int16_t index);
	HUSKYLENSResult getBlock(int16_t index);
	HUSKYLENSResult getBlock(int16_t ID, int16_t index);
	HUSKYLENSResult getArrow(int16_t index);
	HUSKYLENSResult getArrow(int16_t ID, int16_t index);
	HUSKYLENSResult getLearned(int16_t index);
	HUSKYLENSResult getBlockLearned(int16_t index);
	HUSKYLENSResult getArrowLearned(int16_t index);
	bool writeAlgorithm(protocolAlgorithm algorithmType);
	bool writeLearn(int ID);
	bool writeForget();
	bool writeSensor(int sensor0, int sensor1, int sensor2);
	bool setCustomName(char * name,uint8_t id);
	bool savePictureToSDCard();
	bool saveModelToSDCard(int fileNum);
	bool loadModelFromSDCard(int fileNum);
	bool clearCustomText();
	bool customText(char * text,uint16_t x,uint8_t y);
	bool saveScreenshotToSDCard();
	bool isPro();
	bool checkFirmwareVersion();
	bool writeFirmwareVersion(const char* version);
	void protocolWriteCommand(Protocol_t &protocol, uint8_t command);
	bool protocolReadCommand(Protocol_t &protocol, uint8_t command);
	void protocolWriteFiveInt16(Protocol_t &protocol, uint8_t command);
	bool protocolReadFiveInt16(Protocol_t &protocol, uint8_t command);
	void protocolWriteOneInt16(Protocol_t &protocol, uint8_t command);
	bool protocolReadOneInt16(Protocol_t &protocol, uint8_t command);
	bool protocolReadCustomNameHeader(Protocol_t &protocol, uint8_t command);
	void protocolWriteCustomNameHeader(Protocol_t &protocol, uint8_t command);
	bool protocolReadReceivedKnowledges(Protocol_t &protocol, uint8_t command);
	bool protocolWriteReceivedKnowledges(Protocol_t &protocol, uint8_t command);
	bool protocolReadCustomTextRecv(Protocol_t &protocol, uint8_t command);
	bool protocolWriteCustomTextRecv(Protocol_t &protocol, uint8_t command);
	bool protocolReadFirmwareVersion(Protocol_t &protocol, uint8_t command);
	bool protocolWriteFirmwareVersion(Protocol_t &protocol, uint8_t command);
	
	



	void Test();
	void printResult(HUSKYLENSResult result);
protected:
private:
	bool protocolWrite(uint8_t *buffer, int length);
	void timerBegin(){timeOutTimer = Core.millis();};
	bool timerAvailable(){return (Core.millis() - timeOutTimer > timeOutDuration);};
	bool protocolAvailable();	bool processReturn();	bool wait(uint8_t command = 0);	bool readKnock();

	CHuskyMain( const CHuskyMain &c );
	CHuskyMain& operator=( const CHuskyMain &c );
}; //HuskyMain

#endif //__HUSKYMAIN_H__

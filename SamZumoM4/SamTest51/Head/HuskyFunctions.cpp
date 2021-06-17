/* 
* HuskyFunctions.cpp
*
* Created: 27/01/2021 18:43:13
* Author: philg
*/

#include "Includes.h"

// default constructor
HuskyFunctions::HuskyFunctions()
{
	resultDefault.command = -1;
	resultDefault.first = -1;
	resultDefault.second = -1;
	resultDefault.third = -1;
	resultDefault.fourth = -1;
	resultDefault.fifth = -1;

} //HuskyFunctions

// default destructor
HuskyFunctions::~HuskyFunctions()
{
} //~HuskyFunctions


bool HuskyFunctions::begin()
{
	return readKnock();
}



bool HuskyFunctions::protocolWrite(uint8_t *buffer, int length)
{
	return(I2c.WriteBytes(HUSKY_ADDRESS,buffer,length));

}

void HuskyFunctions::timerBegin()
{
	timeOutTimer = millis();
}

bool HuskyFunctions::timerAvailable()
{
	return (millis() - timeOutTimer > timeOutDuration);
}

bool HuskyFunctions::protocolAvailable()
{
	if (wire)
	{
		if (!wire->available())
		{
			wire->requestFrom(0x32, 16);
		}
		while (wire->available())
		{
			int result = wire->read();
			if (husky_lens_protocol_receive(result))
			{
				return true;
			}
		}
	}
	else if (stream)
	{
		while (stream->available())
		{
			int result = stream->read();
			if (husky_lens_protocol_receive(result))
			{
				return true;
			}
		}
	}

	return false;
}

Protocol_t protocolInfo;
Protocol_t *protocolPtr = NULL;

bool HuskyFunctions::processReturn()
{
	currentIndex = 0;
	if (!wait(COMMAND_RETURN_INFO))
	return false;
	protocolReadReturnInfo(protocolInfo);
	protocolPtr = (Protocol_t *)realloc(protocolPtr, max(protocolInfo.protocolSize, 1) * sizeof(Protocol_t));

	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (!wait())
		return false;
		if (protocolReadReturnBlock(protocolPtr[i]))
		continue;
		else if (protocolReadReturnArrow(protocolPtr[i]))
		continue;
		else
		return false;
	}
	return true;
}

HUSKYLENSResult resultDefault;

bool HuskyFunctions::wait(uint8_t command = 0)
{
	timerBegin();
	while (!timerAvailable())
	{
		if (protocolAvailable())
		{
			if (command)
			{
				if (husky_lens_protocol_read_begin(command))
				return true;
			}
			else
			{
				return true;
			}
		}
	}
	return false;
}

bool HuskyFunctions::readKnock()
{
	for (int i = 0; i < 5; i++)
	{
		protocolWriteRequestKnock();
		if (wait(COMMAND_RETURN_OK))
		{
			return true;
		}
	}
	return false;
}

public:



void HuskyFunctions::setTimeOutDuration(unsigned long timeOutDurationInput)
{
	timeOutDuration = timeOutDurationInput;
}

bool HuskyFunctions::request()
{
	protocolWriteRequest();
	return processReturn();
}
bool HuskyFunctions::request(int16_t ID)
{
	Protocol_t protocol;
	protocol.requestID = ID;
	protocolWriteRequestByID(protocol);
	return processReturn();
}

bool HuskyFunctions::requestBlocks()
{
	protocolWriteRequestBlocks();
	return processReturn();
}
bool HuskyFunctions::requestBlocks(int16_t ID)
{
	Protocol_t protocol;
	protocol.requestID = ID;
	protocolWriteRequestBlocksByID(protocol);
	return processReturn();
}

bool HuskyFunctions::requestArrows()
{
	protocolWriteRequestArrows();
	return processReturn();
}
bool requestArrows(int16_t ID)
{
	Protocol_t protocol;
	protocol.requestID = ID;
	protocolWriteRequestArrowsByID(protocol);
	return processReturn();
}
bool HuskyFunctions::requestLearned()
{
	protocolWriteRequestLearned();
	return processReturn();
}
bool HuskyFunctions::requestBlocksLearned()
{
	protocolWriteRequestBlocksLearned();
	return processReturn();
}
bool HuskyFunctions::requestArrowsLearned()
{
	protocolWriteRequestArrowsLearned();
	return processReturn();
}

int available()
{
	int result = count();
	currentIndex = min(currentIndex, result);
	return result - currentIndex;
}

HUSKYLENSResult HuskyFunctions::read()
{
	return (get(currentIndex++));
}

bool HuskyFunctions::isLearned()
{
	return countLearnedIDs();
}

bool HuskyFunctions::isLearned(int ID)
{
	return (ID <= countLearnedIDs());
}

int16_t HuskyFunctions::frameNumber()
{
	return protocolInfo.frameNum;
}

int16_t HuskyFunctions::countLearnedIDs()
{
	return protocolInfo.knowledgeSize;
}

int16_t HuskyFunctions::count()
{
	return protocolInfo.protocolSize;
}
int16_t HuskyFunctions::count(int16_t ID)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].ID == ID)
		counter++;
	}
	return counter;
}

int16_t HuskyFunctions::countBlocks()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_BLOCK)
		counter++;
	}
	return counter;
}
int16_t HuskyFunctions::countBlocks(int16_t ID)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_BLOCK && protocolPtr[i].ID == ID)
		counter++;
	}
	return counter;
}

int16_t HuskyFunctions::countArrows()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_ARROW)
		counter++;
	}
	return counter;
}
int16_t HuskyFunctions::countArrows(int16_t ID)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_ARROW && protocolPtr[i].ID == ID)
		counter++;
	}
	return counter;
}

int16_t HuskyFunctions::countLearned()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].ID)
		counter++;
	}
	return counter;
}
int16_t HuskyFunctions::countBlocksLearned()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_BLOCK && protocolPtr[i].ID)
		counter++;
	}
	return counter;
}
int16_t HuskyFunctions::countArrowsLearned()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_ARROW && protocolPtr[i].ID)
		counter++;
	}
	return counter;
}

HUSKYLENSResult HuskyFunctions::get(int16_t index)
{
	if (index < count())
	{
		return protocolPtr[index];
	}
	return resultDefault;
}
HUSKYLENSResult HuskyFunctions::get(int16_t ID, int16_t index)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].ID == ID)
		if (index == counter++)
		return protocolPtr[i];
	}
	return resultDefault;
}

HUSKYLENSResult HuskyFunctions::getBlock(int16_t index)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_BLOCK)
		if (index == counter++)
		return protocolPtr[i];
	}
	return resultDefault;
}
HUSKYLENSResult HuskyFunctions::getBlock(int16_t ID, int16_t index)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_BLOCK && protocolPtr[i].ID == ID)
		if (index == counter++)
		return protocolPtr[i];
	}
	return resultDefault;
}

HUSKYLENSResult HuskyFunctions::getArrow(int16_t index)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_ARROW)
		if (index == counter++)
		return protocolPtr[i];
	}
	return resultDefault;
}
HUSKYLENSResult HuskyFunctions::getArrow(int16_t ID, int16_t index)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_ARROW && protocolPtr[i].ID == ID)
		if (index == counter++)
		return protocolPtr[i];
	}
	return resultDefault;
}

HUSKYLENSResult HuskyFunctions::getLearned(int16_t index)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].ID)
		if (index == counter++)
		return protocolPtr[i];
	}
	return resultDefault;
}
HUSKYLENSResult HuskyFunctions::getBlockLearned(int16_t index)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_BLOCK && protocolPtr[i].ID)
		if (index == counter++)
		return protocolPtr[i];
	}
	return resultDefault;
}
HUSKYLENSResult HuskyFunctions::getArrowLearned(int16_t index)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_ARROW && protocolPtr[i].ID)
		if (index == counter++)
		return protocolPtr[i];
	}
	return resultDefault;
}

bool HuskyFunctions::writeAlgorithm(protocolAlgorithm algorithmType)
{
	Protocol_t protocol;
	protocol.algorithmType = algorithmType;
	protocolWriteRequestAlgorithm(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool HuskyFunctions::writeLearn(int ID)
{
	Protocol_t protocol;
	protocol.requestID = ID;
	protocolWriteRequestLearn(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool HuskyFunctions::writeForget()
{
	protocolWriteRequestForget();
	return wait(COMMAND_RETURN_OK);
}

bool HuskyFunctions::writeSensor(int sensor0, int sensor1, int sensor2)
{
	Protocol_t protocol;
	protocol.first = sensor0;
	protocol.second = sensor1;
	protocol.third = sensor2;
	protocolWriteRequestSensor(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool HuskyFunctions::setCustomName(char * name,uint8_t id)
{
	const char* nameC = name;
	Protocol_t protocol;
	protocol.customNameHeader.nameDataSize=strlen(nameC);
	protocol.customNameHeader.id=id;
	if(protocol.customNameHeader.nameDataSize > 20)
	{
		return false;
	}
	memcpy(protocol.customNameHeader.dataBuffer,nameC,protocol.customNameHeader.nameDataSize);
	protocolWriteRequestCustomNames(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool HuskyFunctions::savePictureToSDCard()
{
	Protocol_t protocol;
	protocolWriteRequestPhoto(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool saveModelToSDCard(int fileNum){
	Protocol_t protocol;
	protocol.first=fileNum;
	protocolWriteRequestSendKnowledges(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool loadModelFromSDCard(int fileNum){
	Protocol_t protocol;
	protocol.first=fileNum;
	protocolWriteRequestReceiveKnowledges(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool clearCustomText(){
	Protocol_t protocol;
	protocolWriteRequestClearText(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool customText(String text,uint16_t x,uint8_t y){
	const char* textC = text.c_str();
	Protocol_t protocol;
	protocol.customText.textSize=strlen(textC);
	if(protocol.customText.textSize>20){
		return false;
	}
	protocol.customText.x=x;
	protocol.customText.y=y;
	memcpy(protocol.customText.text,textC,protocol.customText.textSize);
	protocolWriteRequestCustomText(protocol);
	return wait(COMMAND_RETURN_OK);
}
	

bool saveScreenshotToSDCard(){
	Protocol_t protocol;
	protocolWriteRequestSaveScreenshot(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool isPro(){
	Protocol_t protocolRequest;
	Protocol_t protocolResonse;
	protocolWriteRequestIsPro(protocolRequest);
	wait(COMMAND_RETURN_INFO);
	if(protocolReadRequestIsPro(protocolResonse)){
		return protocolResonse.first;
		}else{
		return false;
	}
}

#define HUSKYLENS_FIRMWARE_VERSION "0.4.1"
bool checkFirmwareVersion(){
	writeFirmwareVersion(HUSKYLENS_FIRMWARE_VERSION);
}

bool writeFirmwareVersion(String version)
{
	Protocol_t protocol;
	uint8_t length = version.length();
	uint8_t data[length + 2] = {length};
	version.toCharArray((char *)data + 1, length + 1);
	protocol.firmwareVersion.length = length + 1;
	protocol.firmwareVersion.data = data;
	protocolWriteRequestFirmwareVersion(protocol);
	return wait(COMMAND_RETURN_OK);
}
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void protocolWriteCommand(Protocol_t &protocol, uint8_t command)
{
	protocol.command = command;
	uint8_t *buffer = husky_lens_protocol_write_begin(protocol.command);
	int length = husky_lens_protocol_write_end();
	protocolWrite(buffer, length);
}

bool protocolReadCommand(Protocol_t &protocol, uint8_t command)
{
	if (husky_lens_protocol_read_begin(command))
	{
		protocol.command = command;
		husky_lens_protocol_read_end();
		return true;
	}
	else
	{
		return false;
	}
}

void protocolWriteFiveInt16(Protocol_t &protocol, uint8_t command)
{
	protocol.command = command;
	uint8_t *buffer = husky_lens_protocol_write_begin(protocol.command);
	husky_lens_protocol_write_int16(protocol.first);
	husky_lens_protocol_write_int16(protocol.second);
	husky_lens_protocol_write_int16(protocol.third);
	husky_lens_protocol_write_int16(protocol.fourth);
	husky_lens_protocol_write_int16(protocol.fifth);
	int length = husky_lens_protocol_write_end();
	protocolWrite(buffer, length);
}

bool protocolReadFiveInt16(Protocol_t &protocol, uint8_t command)
{
	if (husky_lens_protocol_read_begin(command))
	{
		protocol.command = command;
		protocol.first = husky_lens_protocol_read_int16();
		protocol.second = husky_lens_protocol_read_int16();
		protocol.third = husky_lens_protocol_read_int16();
		protocol.fourth = husky_lens_protocol_read_int16();
		protocol.fifth = husky_lens_protocol_read_int16();
		husky_lens_protocol_read_end();
		return true;
	}
	else
	{
		return false;
	}
}

void protocolWriteOneInt16(Protocol_t &protocol, uint8_t command)
{
	protocol.command = command;
	uint8_t *buffer = husky_lens_protocol_write_begin(protocol.command);
	husky_lens_protocol_write_int16(protocol.first);
	int length = husky_lens_protocol_write_end();
	protocolWrite(buffer, length);
}

bool protocolReadOneInt16(Protocol_t &protocol, uint8_t command)
{
	if (husky_lens_protocol_read_begin(command))
	{
		protocol.command = command;
		protocol.first = husky_lens_protocol_read_int16();
		husky_lens_protocol_read_end();
		return true;
	}
	else
	{
		return false;
	}
}

bool protocolReadCustomNameHeader(Protocol_t &protocol, uint8_t command)
{

	if (husky_lens_protocol_read_begin(command))
	{
		protocol.command = command;
		protocol.customNameHeader.id = husky_lens_protocol_read_uint8();
		protocol.customNameHeader.nameDataSize = husky_lens_protocol_read_uint8();
		for (int i = 0; i < protocol.customNameHeader.nameDataSize; i++)
		{
			if (i > 20)
			{
				break;
			}
			protocol.customNameHeader.dataBuffer[i] = husky_lens_protocol_read_uint8();
		}
		protocol.customNameHeader.dataBuffer[20] = 0x00;
		husky_lens_protocol_read_end();
		return true;
	}
	else
	{
		return false;
	}
}

void protocolWriteCustomNameHeader(Protocol_t &protocol, uint8_t command)
{
	protocol.command = command;
	uint8_t *buffer = husky_lens_protocol_write_begin(protocol.command);
	husky_lens_protocol_write_uint8(protocol.customNameHeader.id);
	husky_lens_protocol_write_uint8(protocol.customNameHeader.nameDataSize);
	for(int i=0;i<20;i++){
		husky_lens_protocol_write_uint8(protocol.customNameHeader.dataBuffer[i]);
	}
	husky_lens_protocol_write_uint8(0x0);
	int length = husky_lens_protocol_write_end();
	protocolWrite(buffer, length);
}

bool protocolReadReceivedKnowledges(Protocol_t &protocol, uint8_t command)
{

	if (husky_lens_protocol_read_begin(command))
	{

		return true;
	}
	else
	{
		return false;
	}
}

bool protocolWriteReceivedKnowledges(Protocol_t &protocol, uint8_t command)
{
	if (husky_lens_protocol_read_begin(command))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool protocolReadCustomTextRecv(Protocol_t &protocol, uint8_t command)
{

	if (husky_lens_protocol_read_begin(command))
	{
		protocol.command = command;
		protocol.customText.textSize = husky_lens_protocol_read_uint8();
		protocol.customText.x = husky_lens_protocol_read_uint8() + husky_lens_protocol_read_uint8();
		protocol.customText.y = husky_lens_protocol_read_uint8();
		char buf[protocol.customText.textSize];
		for (int i = 0; i < protocol.customText.textSize; i++)
		{
			if (i > 20)
			{
				break;
			}
			protocol.customText.text[i] = husky_lens_protocol_read_uint8();
		}
		protocol.customText.text[20] = 0x00;
		//Null terminate the string
		protocol.customText.text[protocol.customText.textSize] = 0x00;
		husky_lens_protocol_read_end();
		return true;
	}
	else
	{
		return false;
	}
}

bool protocolWriteCustomTextRecv(Protocol_t &protocol, uint8_t command)
{
	protocol.command = command;
	uint8_t *buffer = husky_lens_protocol_write_begin(protocol.command);
	husky_lens_protocol_write_uint8(protocol.customText.textSize);
	if(protocol.customText.x >= 255){
		husky_lens_protocol_write_uint8(0xFF);
		}else{
		husky_lens_protocol_write_uint8(0x00);
	}
	husky_lens_protocol_write_uint8(protocol.customText.x & 0xFF);
	husky_lens_protocol_write_uint8(protocol.customText.y);
	for(int i=0;i<protocol.customText.textSize;i++){
		husky_lens_protocol_write_uint8(protocol.customText.text[i]);
	}
	husky_lens_protocol_write_uint8(0x0);
	int length = husky_lens_protocol_write_end();
	protocolWrite(buffer, length);
	return true;
}

bool protocolReadFirmwareVersion(Protocol_t &protocol, uint8_t command)
{

	if (husky_lens_protocol_read_begin(command))
	{
		protocol.command = command;
		protocol.firmwareVersion.length = husky_lens_protocol_read_uint8();
		char buf[protocol.firmwareVersion.length];
		for (int i = 0; i < protocol.firmwareVersion.length; i++)
		{
			if (i > 20)
			{
				break;
			}
			protocol.firmwareVersion.data[i] = husky_lens_protocol_read_uint8();
		}
		protocol.firmwareVersion.data[20] = 0x00;
		protocol.firmwareVersion.data[protocol.firmwareVersion.length] = 0x00;
		husky_lens_protocol_read_end();
		return true;
	}
	else
	{
		return false;
	}
}

bool protocolWriteFirmwareVersion(Protocol_t &protocol, uint8_t command)
{
	protocol.command = command;
	uint8_t *buffer = husky_lens_protocol_write_begin(protocol.command);
	husky_lens_protocol_write_buffer_uint8(protocol.firmwareVersion.data, protocol.firmwareVersion.length);
	int length = husky_lens_protocol_write_end();
	for(int i=0; i<protocol.firmwareVersion.length; i++)
	{
		Serial.println(protocol.firmwareVersion.data[i]);
	}
	protocolWrite(buffer, length);
	return true;
}

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
PROTOCOL_CREATE(RequestFirmwareVersion, FirmwareVersion, COMMAND_REQUEST_FIRMWARE_VERSION)


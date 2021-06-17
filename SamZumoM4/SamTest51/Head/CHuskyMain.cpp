/* 
* HuskyMain.cpp
*
* Created: 27/01/2021 18:39:13
* Author: philg
*/

#include "Includes.h"

CHuskyMain HuskyMain;

// default constructor
CHuskyMain::CHuskyMain()
{
	resultDefault.command = -1;
	resultDefault.first = -1;
	resultDefault.second = -1;
	resultDefault.third = -1;
	resultDefault.fourth = -1;
	resultDefault.fifth = -1;

} //HuskyMain

// default destructor
CHuskyMain::~CHuskyMain()
{
} //~HuskyMain



//HUSKYLENS green line >> SDA; blue line >> SCL
void printResult(HUSKYLENSResult result);

bool CHuskyMain::Init() 
{
	return readKnock();

}

bool CHuskyMain::protocolWrite(uint8_t *buffer, int length)
{
	return(I2c.WriteBytes(HUSKY_ADDRESS,buffer,length));	
}


bool CHuskyMain::protocolAvailable()
{
	uint8_t Buffer[16];
	if(I2c.ReadBytes(HUSKY_ADDRESS,Buffer,16) == 16)

	for(int i =0;i<16;i++)
	{
		if (HuskyProtocol.husky_lens_protocol_receive(Buffer[i]))
		{
			return true;
		}
	}
	return false;
}


bool CHuskyMain::processReturn()
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
	return(true);
}


bool CHuskyMain::wait(uint8_t command)
{
	timerBegin();
	while (!timerAvailable())
	{
		if (protocolAvailable())    // read 16 bytes
		{
			if (command)
			{
				if (HuskyProtocol.husky_lens_protocol_read_begin(command)) // 
					return true;
			}
			else
				return true;
		}
	}
	return false;
}

bool CHuskyMain::readKnock()
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

bool CHuskyMain::request()
{
	protocolWriteRequest();
	return processReturn();
}

bool CHuskyMain::request(int16_t ID)
{
	Protocol_t protocol;
	protocol.requestID = ID;
	protocolWriteRequestByID(protocol);
	return processReturn();
}

bool CHuskyMain::requestBlocks()
{
	protocolWriteRequestBlocks();
	return processReturn();
}

bool CHuskyMain::requestBlocks(int16_t ID)
{
	Protocol_t protocol;
	protocol.requestID = ID;
	protocolWriteRequestBlocksByID(protocol);
	return processReturn();
}

bool CHuskyMain::requestArrows()
{
	protocolWriteRequestArrows();
	return processReturn();
}
bool CHuskyMain::requestArrows(int16_t ID)
{
	Protocol_t protocol;
	protocol.requestID = ID;
	protocolWriteRequestArrowsByID(protocol);
	return processReturn();
}
bool CHuskyMain::requestLearned()
{
	protocolWriteRequestLearned();
	return processReturn();
}
bool CHuskyMain::requestBlocksLearned()
{
	protocolWriteRequestBlocksLearned();
	return processReturn();
}
bool CHuskyMain::requestArrowsLearned()
{
	protocolWriteRequestArrowsLearned();
	return processReturn();
}

int CHuskyMain::available()
{
	int result = count();
	currentIndex = min(currentIndex, result);
	return result - currentIndex;
}

HUSKYLENSResult CHuskyMain::read()
{
	return (get(currentIndex++));
}

bool CHuskyMain::isLearned()
{
	return countLearnedIDs();
}

bool CHuskyMain::isLearned(int ID)
{
	return (ID <= countLearnedIDs());
}

int16_t CHuskyMain::frameNumber()
{
	return protocolInfo.frameNum;
}

int16_t CHuskyMain::countLearnedIDs()
{
	return protocolInfo.knowledgeSize;
}

int16_t CHuskyMain::count(int16_t ID)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].ID == ID)
		counter++;
	}
	return counter;
}

int16_t CHuskyMain::countBlocks()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_BLOCK)
		counter++;
	}
	return counter;
}
int16_t CHuskyMain::countBlocks(int16_t ID)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_BLOCK && protocolPtr[i].ID == ID)
		counter++;
	}
	return counter;
}

int16_t CHuskyMain::countArrows()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_ARROW)
		counter++;
	}
	return counter;
}
int16_t CHuskyMain::countArrows(int16_t ID)
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_ARROW && protocolPtr[i].ID == ID)
		counter++;
	}
	return counter;
}

int16_t CHuskyMain::countLearned()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].ID)
		counter++;
	}
	return counter;
}
int16_t CHuskyMain::countBlocksLearned()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_BLOCK && protocolPtr[i].ID)
		counter++;
	}
	return counter;
}
int16_t CHuskyMain::countArrowsLearned()
{
	int16_t counter = 0;
	for (int i = 0; i < protocolInfo.protocolSize; i++)
	{
		if (protocolPtr[i].command == COMMAND_RETURN_ARROW && protocolPtr[i].ID)
		counter++;
	}
	return counter;
}

HUSKYLENSResult CHuskyMain::get(int16_t index)
{
	if (index < count())
	{
		return protocolPtr[index];
	}
	return resultDefault;
}
HUSKYLENSResult CHuskyMain::get(int16_t ID, int16_t index)
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

HUSKYLENSResult CHuskyMain::getBlock(int16_t index)
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
HUSKYLENSResult CHuskyMain::getBlock(int16_t ID, int16_t index)
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

HUSKYLENSResult CHuskyMain::getArrow(int16_t index)
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
HUSKYLENSResult CHuskyMain::getArrow(int16_t ID, int16_t index)
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

HUSKYLENSResult CHuskyMain::getLearned(int16_t index)
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
HUSKYLENSResult CHuskyMain::getBlockLearned(int16_t index)
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
HUSKYLENSResult CHuskyMain::getArrowLearned(int16_t index)
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

bool CHuskyMain::writeAlgorithm(protocolAlgorithm algorithmType)
{
	Protocol_t protocol;
	protocol.algorithmType = algorithmType;
	protocolWriteRequestAlgorithm(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::writeLearn(int ID)
{
	Protocol_t protocol;
	protocol.requestID = ID;
	protocolWriteRequestLearn(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::writeForget()
{
	protocolWriteRequestForget();
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::writeSensor(int sensor0, int sensor1, int sensor2)
{
	Protocol_t protocol;
	protocol.first = sensor0;
	protocol.second = sensor1;
	protocol.third = sensor2;
	protocolWriteRequestSensor(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::setCustomName(char * name,uint8_t id)
{
	const char* nameC = name;
	Protocol_t protocol;
	protocol.customNameHeader.nameDataSize=strlen(nameC);
	protocol.customNameHeader.id=id;
	if(protocol.customNameHeader.nameDataSize > 20)
		return false;
	memcpy(protocol.customNameHeader.dataBuffer,nameC,protocol.customNameHeader.nameDataSize);
	protocolWriteRequestCustomNames(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::savePictureToSDCard()
{
	Protocol_t protocol;
	protocolWriteRequestPhoto(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::saveModelToSDCard(int fileNum)
{
	Protocol_t protocol;
	protocol.first=fileNum;
	protocolWriteRequestSendKnowledges(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::loadModelFromSDCard(int fileNum){
	Protocol_t protocol;
	protocol.first=fileNum;
	protocolWriteRequestReceiveKnowledges(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::clearCustomText(){
	Protocol_t protocol;
	protocolWriteRequestClearText(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::customText(char * text,uint16_t x,uint8_t y)
{
	const char* textC = text;
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
	

bool CHuskyMain::saveScreenshotToSDCard()
{
	Protocol_t protocol;
	protocolWriteRequestSaveScreenshot(protocol);
	return wait(COMMAND_RETURN_OK);
}

bool CHuskyMain::isPro()
{
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
bool CHuskyMain::checkFirmwareVersion()
{
	return(writeFirmwareVersion(HUSKYLENS_FIRMWARE_VERSION));
}

bool CHuskyMain::writeFirmwareVersion(const char * version)
{
	Protocol_t protocol;
	uint8_t length = strlen(version);
	protocol.firmwareVersion.length = length + 1;
	protocol.firmwareVersion.data = (uint8_t *) version;
	protocolWriteRequestFirmwareVersion(protocol);
	return wait(COMMAND_RETURN_OK);
}
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////

	void CHuskyMain::protocolWriteCommand(Protocol_t &protocol, uint8_t command)
	{
		protocol.command = command;
		uint8_t *buffer = HuskyProtocol.husky_lens_protocol_write_begin(protocol.command);
		int length = HuskyProtocol.husky_lens_protocol_write_end();
		protocolWrite(buffer, length);
	}

	bool CHuskyMain::protocolReadCommand(Protocol_t &protocol, uint8_t command)
	{
		if (HuskyProtocol.husky_lens_protocol_read_begin(command))
		{
			protocol.command = command;
			HuskyProtocol.husky_lens_protocol_read_end();
			return true;
		}
		else
		{
			return false;
		}
	}

	void CHuskyMain::protocolWriteFiveInt16(Protocol_t &protocol, uint8_t command)
	{
		protocol.command = command;
		uint8_t *buffer = HuskyProtocol.husky_lens_protocol_write_begin(protocol.command);
		HuskyProtocol.husky_lens_protocol_write_int16(protocol.first);
		HuskyProtocol.husky_lens_protocol_write_int16(protocol.second);
		HuskyProtocol.husky_lens_protocol_write_int16(protocol.third);
		HuskyProtocol.husky_lens_protocol_write_int16(protocol.fourth);
		HuskyProtocol.husky_lens_protocol_write_int16(protocol.fifth);
		int length = HuskyProtocol.husky_lens_protocol_write_end();
		protocolWrite(buffer, length);
	}

	bool CHuskyMain::protocolReadFiveInt16(Protocol_t &protocol, uint8_t command)
	{
		if (HuskyProtocol.husky_lens_protocol_read_begin(command))
		{
			protocol.command = command;
			protocol.first = HuskyProtocol.husky_lens_protocol_read_int16();
			protocol.second = HuskyProtocol.husky_lens_protocol_read_int16();
			protocol.third = HuskyProtocol.husky_lens_protocol_read_int16();
			protocol.fourth = HuskyProtocol.husky_lens_protocol_read_int16();
			protocol.fifth = HuskyProtocol.husky_lens_protocol_read_int16();
			HuskyProtocol.husky_lens_protocol_read_end();
			return true;
		}
		else
		{
			return false;
		}
	}

	void CHuskyMain::protocolWriteOneInt16(Protocol_t &protocol, uint8_t command)
	{
		protocol.command = command;
		uint8_t *buffer = HuskyProtocol.husky_lens_protocol_write_begin(protocol.command);
		HuskyProtocol.husky_lens_protocol_write_int16(protocol.first);
		int length = HuskyProtocol.husky_lens_protocol_write_end();
		protocolWrite(buffer, length);
	}

	bool CHuskyMain::protocolReadOneInt16(Protocol_t &protocol, uint8_t command)
	{
		if (HuskyProtocol.husky_lens_protocol_read_begin(command))
		{
			protocol.command = command;
			protocol.first = HuskyProtocol.husky_lens_protocol_read_int16();
			HuskyProtocol.husky_lens_protocol_read_end();
			return true;
		}
		else
		{
			return false;
		}
	}

	bool CHuskyMain::protocolReadCustomNameHeader(Protocol_t &protocol, uint8_t command)
	{

		if (HuskyProtocol.husky_lens_protocol_read_begin(command))
		{
			protocol.command = command;
			protocol.customNameHeader.id = HuskyProtocol.husky_lens_protocol_read_uint8();
			protocol.customNameHeader.nameDataSize = HuskyProtocol.husky_lens_protocol_read_uint8();
			for (int i = 0; i < protocol.customNameHeader.nameDataSize; i++)
			{
				if (i > 20)
				{
					break;
				}
				protocol.customNameHeader.dataBuffer[i] = HuskyProtocol.husky_lens_protocol_read_uint8();
			}
			protocol.customNameHeader.dataBuffer[20] = 0x00;
			HuskyProtocol.husky_lens_protocol_read_end();
			return true;
		}
		else
		{
			return false;
		}
	}

	void CHuskyMain::protocolWriteCustomNameHeader(Protocol_t &protocol, uint8_t command)
	{
		protocol.command = command;
		uint8_t *buffer = HuskyProtocol.husky_lens_protocol_write_begin(protocol.command);
		HuskyProtocol.husky_lens_protocol_write_uint8(protocol.customNameHeader.id);
		HuskyProtocol.husky_lens_protocol_write_uint8(protocol.customNameHeader.nameDataSize);
		for(int i=0;i<20;i++)
		{
			HuskyProtocol.husky_lens_protocol_write_uint8(protocol.customNameHeader.dataBuffer[i]);
		}
		HuskyProtocol.husky_lens_protocol_write_uint8(0x0);
		int length = HuskyProtocol.husky_lens_protocol_write_end();
		protocolWrite(buffer, length);
	}

	bool CHuskyMain::protocolReadReceivedKnowledges(Protocol_t &protocol, uint8_t command)
	{

		if (HuskyProtocol.husky_lens_protocol_read_begin(command))
		{

			return true;
		}
		else
		{
			return false;
		}
	}

	bool CHuskyMain::protocolWriteReceivedKnowledges(Protocol_t &protocol, uint8_t command)
	{
		if (HuskyProtocol.husky_lens_protocol_read_begin(command))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool CHuskyMain::protocolReadCustomTextRecv(Protocol_t &protocol, uint8_t command)
	{

		if (HuskyProtocol.husky_lens_protocol_read_begin(command))
		{
			protocol.command = command;
			protocol.customText.textSize = HuskyProtocol.husky_lens_protocol_read_uint8();
			protocol.customText.x = HuskyProtocol.husky_lens_protocol_read_uint8() + HuskyProtocol.husky_lens_protocol_read_uint8();
			protocol.customText.y = HuskyProtocol.husky_lens_protocol_read_uint8();
			char buf[protocol.customText.textSize];
			for (int i = 0; i < protocol.customText.textSize; i++)
			{
				if (i > 20)
				{
					break;
				}
				protocol.customText.text[i] = HuskyProtocol.husky_lens_protocol_read_uint8();
			}
			protocol.customText.text[20] = 0x00;
			//Null terminate the string
			protocol.customText.text[protocol.customText.textSize] = 0x00;
			HuskyProtocol.husky_lens_protocol_read_end();
			return true;
		}
		else
		{
			return false;
		}
	}

	bool CHuskyMain::protocolWriteCustomTextRecv(Protocol_t &protocol, uint8_t command)
	{
		protocol.command = command;
		uint8_t *buffer = HuskyProtocol.husky_lens_protocol_write_begin(protocol.command);
		HuskyProtocol.husky_lens_protocol_write_uint8(protocol.customText.textSize);
		if(protocol.customText.x >= 255)
		{
			HuskyProtocol.husky_lens_protocol_write_uint8(0xFF);
		}
		else
		{
			HuskyProtocol.husky_lens_protocol_write_uint8(0x00);
		}
		HuskyProtocol.husky_lens_protocol_write_uint8(protocol.customText.x & 0xFF);
		HuskyProtocol.husky_lens_protocol_write_uint8(protocol.customText.y);
		for(int i=0;i<protocol.customText.textSize;i++)
		{
			HuskyProtocol.husky_lens_protocol_write_uint8(protocol.customText.text[i]);
		}
		HuskyProtocol.husky_lens_protocol_write_uint8(0x0);
		int length = HuskyProtocol.husky_lens_protocol_write_end();
		protocolWrite(buffer, length);
		return true;
	}

	bool CHuskyMain::protocolReadFirmwareVersion(Protocol_t &protocol, uint8_t command)
	{

		if (HuskyProtocol.husky_lens_protocol_read_begin(command))
		{
			protocol.command = command;
			protocol.firmwareVersion.length = HuskyProtocol.husky_lens_protocol_read_uint8();
			char buf[protocol.firmwareVersion.length];
			for (int i = 0; i < protocol.firmwareVersion.length; i++)
			{
				if (i > 20)
				{
					break;
				}
				protocol.firmwareVersion.data[i] = HuskyProtocol.husky_lens_protocol_read_uint8();
			}
			protocol.firmwareVersion.data[20] = 0x00;
			protocol.firmwareVersion.data[protocol.firmwareVersion.length] = 0x00;
			HuskyProtocol.husky_lens_protocol_read_end();
			return true;
		}
		else
		{
			return false;
		}
	}

	bool CHuskyMain::protocolWriteFirmwareVersion(Protocol_t &protocol, uint8_t command)
	{
		protocol.command = command;
		uint8_t *buffer = HuskyProtocol.husky_lens_protocol_write_begin(protocol.command);
		HuskyProtocol.husky_lens_protocol_write_buffer_uint8(protocol.firmwareVersion.data, protocol.firmwareVersion.length);
		int length = HuskyProtocol.husky_lens_protocol_write_end();
		for(int i=0; i<protocol.firmwareVersion.length; i++)
		{
			printf("%c\n",protocol.firmwareVersion.data[i]);
		}
		protocolWrite(buffer, length);
		return true;
	}





void CHuskyMain::Test() 
{
    if (!request()) 
		DebugDisplay.Printf("Fail to request data from HUSKYLENS, recheck the connection!\n");
    else if(!isLearned()) 
		DebugDisplay.Printf("Nothing learned, press learn button on HUSKYLENS to learn one!\n");
    else if(!available()) 
		DebugDisplay.Printf("No block or arrow appears on the screen!\n");
    else
    {
		while(1)
		{
			if (request())
			{
				while (available())
				{
					HUSKYLENSResult result = read();
					printResult(result);
				}
			} 
			Core.delay(300);    
		}
    }
}

void CHuskyMain::printResult(HUSKYLENSResult result)
{
    if (result.command == COMMAND_RETURN_BLOCK){
       DebugDisplay.Printf("Block:xCenter= %d ,yCenter= %d, width=%d ,height= %d ,ID= %d\n",result.xCenter,result.yCenter,result.width,result.height,result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
       DebugDisplay.Printf("Arrow:xOrigin= %d ,yOrigin= %d, xTarget=%d ,yTarget= %d ,ID= %d\n",result.xOrigin,result.yOrigin,result.xTarget,result.yTarget,result.ID);
    }
    else
	{
        DebugDisplay.Printf("Object unknown!");
    }
}







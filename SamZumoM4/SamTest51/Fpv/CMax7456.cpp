/* 
* CMax7456.cpp
*
* Created: 07/04/2019 17:07:22
* Author: phil
*/

#include "Includes.h"

volatile uint8_t OsdDebug;

// default constructor
CMax7456::CMax7456(CSpi* Spi,CUart *uart)
{
	m_Spi = Spi;
	m_Uart = uart;
} //CMax7456

// default destructor
CMax7456::~CMax7456()
{
} //~CMax7456



//------------------ init ---------------------------------------------------
void CMax7456::init()
{   
//	uint8_t Value = 1;

// 	Core.delay(500);
 	SET_MAX7456_RS_PORT_ENABLE;
 	MAX7456_RESET_HIGH;
 	SET_MAX7456_CS_PORT_ENABLE;
 	MAX7456_SELECT_HIGH;
	Core.delay(1500);
	//read black level register
	uint8_t osdbl_r = ReadRegister(MAX7456_OSDBL_reg_read);
	
	while(osdbl_r == 0)
	{
		MAX7456_RESET_LOW;
		Core.delay(500);
		MAX7456_RESET_HIGH;
		Core.delay(150);
		osdbl_r = ReadRegister(MAX7456_OSDBL_reg_read);
		
	}
	setMode(1);

//	while( Value == 1)
//	{
//		Value = ReadRegister(MAX7456_STAT_reg_read);
//		if(Value &= (1<<ResetMode))
//			Reinit();
		Core.delay(150);	
//	}

	WriteRegister(MAX7456_VM0_reg,MAX7456_RESET | MAX7456_MODE_MASK_PAL);
	Core.delay(200);
   WriteRegister(MAX7456_VM0_reg, VIDEO_MODE_PAL);

	//set black level
	uint8_t osdbl_w = (osdbl_r & 0xef); //Set bit 4 to zero 11101111
	WriteRegister(MAX7456_OSDBL_reg,osdbl_w);	
	Core.delay(50);
	setBrightness();
	Core.delay(50);
	// define sync (auto,int,ext) and	// making sure the Max7456 is enabled
	Core.delay(50);
	control(1);
	Core.delay(50);
//	DisplayFont();
//	DisplayRegisters();
//	 UploadFont();
}

void CMax7456::init2()
{
//	uint8_t Reset = 2;
	SET_MAX7456_CS_PORT_ENABLE;
	SET_MAX7456_RS_PORT_ENABLE;
	MAX7456_SELECT_HIGH;
	MAX7456_RESET_HIGH;

	detectMode();

	//read black level register
	
	MAX7456_SELECT_LOW;

	m_Spi->Transfer(MAX7456_OSDBL_reg_read);//black level read register
	uint8_t osdbl_r = m_Spi->Transfer(0xff);
	m_Spi->Transfer(MAX7456_VM0_reg);
	MAX7456_RESET_LOW;
	MAX7456_SELECT_HIGH;

//	Reset != video_mode;
//	m_Spi.Transfer(MAX7456_VM0_reg | Reset);		// force soft reset on Max7456
	Core.delay(50);
	MAX7456_RESET_HIGH;
	Core.delay(100);
	//set black level
	uint8_t osdbl_w = (osdbl_r & 0xef); //Set bit 4 to zero 11101111
	MAX7456_SELECT_LOW;
	m_Spi->Transfer(MAX7456_OSDBL_reg); //black level write register
	m_Spi->Transfer(osdbl_w);

	 m_Spi->Transfer(MAX7456_VM0_reg);
	 OsdDebug = m_Spi->Transfer(0xff);

	MAX7456_SELECT_HIGH;

	setBrightness();
	// define sync (auto,int,ext) and
	// making sure the Max7456 is enabled
	control(1);
}

//------------------ Detect Mode (PAL/NTSC) ---------------------------------

void CMax7456::detectMode()
{
	//read STAT and auto detect Mode PAL/NTSC
	MAX7456_SELECT_LOW;
	m_Spi->Transfer(MAX7456_STAT_reg_read);//status register
	uint8_t osdstat_r = m_Spi->Transfer(0xff);
	if ((0b00000001 & osdstat_r) == 1)
	{ //PAL
		setMode(1);
	}
	else if((0b00000010 & osdstat_r) == 1)
	{ //NTSC
		setMode(0);
	}
	//If no signal was detected so it uses EEPROM config
	else
	{
//		if (Config. OsdConfig.PalNtscMode == 0)
//		{ //NTSC
//			setMode(0);
//		}
//		else 
//		{ //PAL
			setMode(1);
//		}
	}
	MAX7456_SELECT_HIGH;
}

//------------------ Set Brightness  ---------------------------------
void CMax7456::setBrightness()
{

	uint8_t blevel = 3;//Config.OsdConfig.OsdBrightness;

	if(blevel == 0) //low brightness
	blevel = MAX7456_WHITE_level_80;
	else if(blevel == 1)
		blevel = MAX7456_WHITE_level_90;
	else if(blevel == 2)
		blevel = MAX7456_WHITE_level_100;
	else if(blevel == 3) //high brightness
		blevel = MAX7456_WHITE_level_120;
	else
	blevel = MAX7456_WHITE_level_80; //low brightness if bad value
	MAX7456_SELECT_LOW;
	// set all rows to same character white level, 90%
	for (uint8_t x = 0x0; x < 0x10; x++)
	{
		m_Spi->Transfer(x + 0x10);
		m_Spi->Transfer(blevel);
	}
	MAX7456_SELECT_HIGH;
}

//------------------ Set Mode (PAL/NTSC) ------------------------------------

void CMax7456::setMode(int themode)
{
	switch(themode)
	{
	case 0:
		m_VideoMode = MAX7456_MODE_MASK_NTCS;
		m_VideoCenter = MAX7456_CENTER_NTSC;
		break;
	case 1:
		m_VideoMode = MAX7456_MODE_MASK_PAL;
		m_VideoCenter = MAX7456_CENTER_PAL;
		break;
	}
}

//------------------ Get Mode (PAL 0/NTSC 1) --------------------------------

int CMax7456::getMode()
{
	switch(m_VideoMode)
	{
	case MAX7456_MODE_MASK_NTCS:
		return 0;
		break;
	case MAX7456_MODE_MASK_PAL:
		return 1;
		break;
	}
	return 0;
}

//------------------ Get Center (PAL/NTSC) ----------------------------------

int CMax7456::getCenter()
{
	return m_VideoCenter; //first line for center panel
}

//------------------ plug ---------------------------------------------------

void CMax7456::plug()
{
	MAX7456_SELECT_LOW;
}

//------------------ clear ---------------------------------------------------

void CMax7456::clear()
{
	// clear the screen
	MAX7456_SELECT_LOW;
	m_Spi->Transfer(MAX7456_DMM_reg);
	m_Spi->Transfer(MAX7456_CLEAR_display);
	MAX7456_SELECT_HIGH;
}
//------------------ write single char ---------------------------------------------

void
CMax7456::openSingle(uint8_t st_col, uint8_t st_row)
{
	unsigned int linepos;
	uint8_t CharAddressHi, CharAddressLow;

	if(st_row != 0)
	{
		m_StartColumn = st_col;
		m_StartRow = st_row;
		m_Column = st_col;
		m_Row = st_row;
	}
	
	//find [start address] position
	linepos = m_Row*30+m_Column;
	
	// divide 16 bits into hi & lo byte
	CharAddressHi = linepos >> 8;
	CharAddressLow = linepos;
	
	WriteRegister(MAX7456_DMAH_reg,CharAddressHi); // set start address high
	WriteRegister(MAX7456_DMAL_reg,CharAddressLow); // set start address low
//	WriteRegister(MAX7456_DMM_reg,MAX7456_INCREMENT_auto);
}



//================= open panel =================================================

void CMax7456::OpenPanel(uint8_t st_col, uint8_t st_row)
{
	unsigned int linepos;
	uint8_t  CharAddressHi, CharAddressLow;
	if(st_row != 0)
	{
		m_StartColumn = st_col;
		m_StartRow = st_row;
		m_Column = st_col;
		m_Row = st_row;
	}

	
	
	linepos = m_Row*30+m_Column;							//find [start address] position
	
	
	CharAddressHi = linepos >> 8;					// divide 16 bits into hi & lo byte
	CharAddressLow = linepos;

	
	WriteRegister(MAX7456_DMAH_reg,CharAddressHi);
	WriteRegister(MAX7456_DMAL_reg,CharAddressLow);
	WriteRegister(MAX7456_DMM_reg,MAX7456_INCREMENT_auto);
}

//------------------ close panel ---------------------------------------------

void CMax7456::ClosePanel(void)
{
	write(MAX7456_END_string); //This is needed "trick" to finish auto increment
	m_Row++; //only after finish the auto increment the new row will really act as desired
}


void CMax7456::Printf(const char * fmt,  ... )
{
	uint8_t i;
	int ret;
	char buffer[80];
	va_list ap;

	va_start(ap, fmt);
	ret = vsprintf(buffer, fmt, ap);
	va_end(ap);
	MAX7456_SELECT_LOW;
	if (ret > 0)
	{
		for(i=0;i<ret;i++)
		write(buffer[i]);
	}
	MAX7456_SELECT_HIGH;
	
}
uint8_t CMax7456::ReadRegister(uint8_t Register)
{
	uint8_t Data;
	MAX7456_SELECT_LOW;
	m_Spi->Transfer(Register);
	Data = m_Spi->Transfer(0xff);
	MAX7456_SELECT_HIGH;
	return(Data);

}

void CMax7456::WriteRegister(uint8_t Register,uint8_t Data)
{
	MAX7456_SELECT_LOW;
	m_Spi->Transfer(Register);
	m_Spi->Transfer(Data);
	MAX7456_SELECT_HIGH;

}


//------------------ write ---------------------------------------------------

size_t CMax7456::write(uint8_t c)
{
	if(c == '|')
	{
		ClosePanel(); //It does all needed to finish auto increment and change current row
		OpenPanel(0,0); //It does all needed to re-enable auto increment
	}
	
	else
	{
		MAX7456_SELECT_LOW;
		m_Spi->Transfer(c);
		MAX7456_SELECT_HIGH;
	}
	return 1;
}

//---------------------------------

void CMax7456::control(uint8_t ctrl)
{
	switch(ctrl)
	{
	case 0:
		WriteRegister(MAX7456_VM0_reg,MAX7456_DISABLE_display | m_VideoMode);
		break;
	case 1:
		WriteRegister(MAX7456_VM0_reg,(MAX7456_ENABLE_display_vert | m_VideoMode) | MAX7456_SYNC_autosync);
		break;
	}
}

void CMax7456::write_NVM(int font_count, uint8_t *character_bitmap)
{
	uint8_t x;
	uint8_t CharAddressHi;
	uint8_t screen_char;
	CharAddressHi = font_count;

	MAX7456_SELECT_LOW;
	m_Spi->Transfer(MAX7456_VM0_reg);
	m_Spi->Transfer(MAX7456_DISABLE_display);

	m_Spi->Transfer(MAX7456_CMAH_reg); // set start address high
	m_Spi->Transfer(CharAddressHi);

	for(x = 0; x < NVM_ram_size; x++) // write out 54 (out of 64) bytes of character to shadow ram
	{
		screen_char = character_bitmap[x];
		m_Spi->Transfer(MAX7456_CMAL_reg); // set start address low
		m_Spi->Transfer(x);
		m_Spi->Transfer(MAX7456_CMDI_reg);
		m_Spi->Transfer(screen_char);
	}

	// transfer a 54 bytes from shadow ram to NVM
	m_Spi->Transfer(MAX7456_CMM_reg);
	m_Spi->Transfer(WRITE_nvr);
	
	// wait until bit 5 in the status register returns to 0 (12ms)
	while ((m_Spi->Transfer(MAX7456_STAT_reg_read) & STATUS_reg_nvr_busy) != 0x00);

	m_Spi->Transfer(MAX7456_VM0_reg); // turn on screen next vertical
	m_Spi->Transfer(MAX7456_ENABLE_display_vert);
	MAX7456_SELECT_HIGH;
}


void CMax7456::UploadFont()
{
	uint16_t byte_count = 0;
	uint8_t InData;
	uint8_t bit_count =0;
	uint8_t incomingByte;
	static uint8_t CrlfCount;
	
	// move these local 
	uint8_t character_bitmap[0x40];
	int font_count = 0;
	
	m_Uart->ClearBuffer(&m_Uart->m_RecieveQue);
	
	while (1)
	{

		if(m_Uart->Read(&InData))
		{
			if (InData == '\n' || InData == '\r')
			{
				CrlfCount++;
			}
			else
			{
				CrlfCount = 0;
			}
			if (CrlfCount == 3)
			{
				break;
			}
		}
	}

	clear();

	m_Uart->WriteTxUnbuffered((uint8_t*)"RFF\n",4);

	while(font_count < 256)
	{
		if(m_Uart->Read(&incomingByte))
		{
			switch(incomingByte) // parse and decode mcm file
			{
			case 0x0d: // carriage return, end of line
				//Serial.println("cr");
				if (bit_count == 8)
				{
					byte_count++;
					character_bitmap[byte_count] = 0;
				}
				bit_count = 0;
				break;
			case 0x30: // ascii '0'
			case 0x31: // ascii '1'
				character_bitmap[byte_count] = character_bitmap[byte_count] << 1;
				if(incomingByte == 0x31)
					character_bitmap[byte_count] += 1;
				bit_count++;
				break;
			default:
				break;
			}

			// we have one completed character
			// write the character to NVM
			if(byte_count == 64)
			{
				write_NVM(font_count, character_bitmap);
				byte_count = 0;
				font_count++;
				m_Uart->WriteTxUnbuffered((uint8_t*)"CD\n",3);
			}
		}
	}
	while(!m_Uart->Read(&incomingByte))
	Core.delay(10);
	m_Uart->WriteTxUnbuffered((uint8_t*)"End\n",4);
}


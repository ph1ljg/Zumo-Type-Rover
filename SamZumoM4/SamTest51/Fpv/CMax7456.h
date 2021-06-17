/* 
* CMax7456.h
*
* Created: 07/04/2019 17:07:22
* Author: phil
*/


#ifndef __CMAX7456_H__
#define __CMAX7456_H__

#define SET_MAX7456_CS_PORT_ENABLE	PinPeripheral.SetPinMode(PORTB,8,OUTPUT)
#define MAX7456_SELECT_LOW			PinPeripheral.SetPin(PORTB,8,LOW)
#define MAX7456_SELECT_HIGH			PinPeripheral.SetPin(PORTB,8,HIGH)


#define SET_MAX7456_RS_PORT_ENABLE	PinPeripheral.SetPinMode(PORTA,2,OUTPUT)
#define MAX7456_RESET_LOW			PinPeripheral.SetPin(PORTA,2,LOW)
#define MAX7456_RESET_HIGH			PinPeripheral.SetPin(PORTA,2,HIGH)


														// PC8	
#define MAX7456_VSYNC_PULUP //PORTD |= (1<<PORTD2) //2//INT0
#define MAX7456_VSYNC_DIR	//DDRD &= ~(1<<DDD2) //6//SS

#define NTSC 0
#define PAL 1
#define MAX7456_MODE_MASK_PAL 0x40 //PAL mask 01000000
#define MAX7456_CENTER_PAL 0x8

#define MAX7456_MODE_MASK_NTCS 0x00 //NTSC mask 00000000 ("|" will do nothing)
#define MAX7456_CENTER_NTSC 0x6

//MAX7456 reg read addresses
#define MAX7456_OSDBL_reg_read 0xec //black level
#define MAX7456_STAT_reg_read  0xa0 //0xa[X] Status

//MAX7456 reg write addresses
#define MAX7456_VM0_reg   0x00
#define MAX7456_VM1_reg   0x01
#define MAX7456_DMM_reg   0x04
#define MAX7456_DMAH_reg  0x05
#define MAX7456_DMAL_reg  0x06
#define MAX7456_DMDI_reg  0x07
#define MAX7456_OSDM_reg  0x0c //not used. Is to set mix
#define MAX7456_OSDBL_reg 0x6c //black level

//MAX7456 reg write addresses to recording NVM process
#define MAX7456_CMM_reg   0x08
#define MAX7456_CMAH_reg  0x09
#define MAX7456_CMAL_reg  0x0a
#define MAX7456_CMDI_reg  0x0b

//DMM commands
#define MAX7456_CLEAR_display 0x04
#define MAX7456_CLEAR_display_vert 0x06

#define MAX7456_INCREMENT_auto 0x03
#define MAX7456_SETBG_local 0x20 //00100000 force local BG to defined brightness level VM1[6:4]

#define MAX7456_END_string 0xff

//VM0 commands mixed with mode NTSC or PAL mode
#define MAX7456_ENABLE_display_vert 0x0c //mask with NTSC/PAL
#define MAX7456_RESET 0x02 //mask with NTSC/PAL
#define MAX7456_DISABLE_display 0x00 //mask with NTSC/PAL

//VM0 command modifiers
#define MAX7456_SYNC_autosync 0x10
#define MAX7456_SYNC_internal 0x30
#define MAX7456_SYNC_external 0x20
//VM1 command modifiers
#define MAX7456_WHITE_level_80 0x03
#define MAX7456_WHITE_level_90 0x02
#define MAX7456_WHITE_level_100 0x01
#define MAX7456_WHITE_level_120 0x00

#define NVM_ram_size 0x36
#define WRITE_nvr 0xa0
#define STATUS_reg_nvr_busy 0x20

//If PAL
#ifdef isPAL
#define MAX7456_screen_size 480 //16x30
#define MAX7456_screen_rows 0x10
#else
#define MAX7456_screen_size 390 //13x30
#define MAX7456_screen_rows 0x0D
#endif
#define VIDEO_MODE_PAL              0x40

class CMax7456
{
//variables
public:
protected:
private:
	uint8_t m_StartColumn;
	uint8_t m_StartRow;
	uint8_t m_Column;
	uint8_t m_Row;
	uint8_t m_VideoMode;
	uint8_t m_VideoCenter;
	CSpi *m_Spi;
	CUart *m_Uart;
//functions
public:
	CMax7456(CSpi* Spi,CUart *uart);
	~CMax7456();
	void UploadFont();
	void init();
	void init2();
	void detectMode();
	void setBrightness();
	void setMode(int themode);
	int getMode();
	int getCenter();
	void plug();
	void clear();
	void OpenPanel(uint8_t st_col, uint8_t st_row);
	void ClosePanel(void);
	void openSingle(uint8_t x, uint8_t y);
	void Printf(const char * fmt, ... );
	uint8_t ReadRegister(uint8_t Register);
	void WriteRegister(uint8_t Register,uint8_t Data);
	size_t write(uint8_t c);
	void control(uint8_t ctrl);
	void write_NVM(int font_count, uint8_t *character_bitmap);

protected:
private:
	CMax7456( const CMax7456 &c );
	CMax7456& operator=( const CMax7456 &c );

}; //CMax7456

#endif //__CMAX7456_H__

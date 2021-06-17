/* 
* CI2c.cpp
*
* Created: 18/05/2020 14:22:53
* Author: philg
*/

#include "includes.h"

uint8_t DebugAddress1; 
uint8_t DebugAddress2; 
// default constructor
CI2c::CI2c()
{
	m_I2cErrorsTotal = 0;

} //CI2c

// default destructor
CI2c::~CI2c()
{
} //~CI2c



CI2c::CI2c(const void *const s, EPortType port, uint8_t pinSDA, uint8_t pinSCL)
{
   m_SercomRegStruct = (Sercom *)s;
  m_PortNoSDA = pinSDA;
  m_PortNoSCL = pinSCL;
  m_Port = port;
  
}

void CI2c::init(void) 
{
  //Master Mode
  InitMaster(TWI_CLOCK);
  Enable();

  
  PinPeripheral.SetPeripheral(m_Port,m_PortNoSDA, PIO_INPUT_PULLUP);
  PinPeripheral.SetPeripheral(m_Port,m_PortNoSCL, PIO_INPUT_PULLUP);

  PinPeripheral.SetPeripheral(m_Port,m_PortNoSDA, PIO_SERCOM);
  PinPeripheral.SetPeripheral(m_Port,m_PortNoSCL, PIO_SERCOM);
}


void CI2c::Reset()
{
  //I2CM OR I2CS, no matter SWRST is the same bit.

  //Setting the Software bit to 1
  m_SercomRegStruct->I2CM.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(m_SercomRegStruct->I2CM.CTRLA.bit.SWRST || m_SercomRegStruct->I2CM.SYNCBUSY.bit.SWRST);
}

void CI2c::Enable()
{
  // I2C Master and Slave modes share the ENABLE bit function.

 	m_SercomRegStruct->I2CM.CTRLA.bit.ENABLE = 1 ;					// Enable the I2C master mode

  while ( m_SercomRegStruct->I2CM.SYNCBUSY.bit.ENABLE != 0 )		// Waiting the enable bit from SYNCBUSY is equal to 0;
  {
  }
  
  m_SercomRegStruct->I2CM.STATUS.bit.BUSSTATE = 1 ;					// Setting bus idle mode

  while ( m_SercomRegStruct->I2CM.SYNCBUSY.bit.SYSOP != 0 )			// Wait the SYSOP bit from SYNCBUSY coming back to 0
  {
  }
}

void CI2c::disableWIRE()
{
  // I2C Master and Slave modes share the ENABLE bit function.

  // Enable the I2C master mode
  m_SercomRegStruct->I2CM.CTRLA.bit.ENABLE = 0 ;

  while ( m_SercomRegStruct->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }
}

void CI2c::initSlaveWIRE( uint8_t ucAddress, bool enableGeneralCall )
{
  CSercom Ser;
  // Initialize the peripheral clock and interruption
  Ser.InitClockNVIC(m_SercomRegStruct,SERCOM_CLOCK_SOURCE_48M) ;
  Reset() ;

  // Set slave mode
  m_SercomRegStruct->I2CS.CTRLA.bit.MODE = I2C_SLAVE_OPERATION;

  m_SercomRegStruct->I2CS.ADDR.reg = SERCOM_I2CS_ADDR_ADDR( ucAddress & 0x7Ful ) | // 0x7F, select only 7 bits
                          SERCOM_I2CS_ADDR_ADDRMASK( 0x00ul );          // 0x00, only match exact address
  if (enableGeneralCall) {
    m_SercomRegStruct->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_GENCEN;                   // enable general call (address 0x00)
  }

  // Set the interrupt register
  m_SercomRegStruct->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_PREC |   // Stop
                              SERCOM_I2CS_INTENSET_AMATCH | // Address Match
                              SERCOM_I2CS_INTENSET_DRDY ;   // Data Ready

  while ( m_SercomRegStruct->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY to come back to 0
  }
}

void CI2c::InitMaster( uint32_t baudrate )
{
  // Initialize the peripheral clock and interruption
 CSercom Ser;
 Ser.InitClockNVIC(m_SercomRegStruct,SERCOM_CLOCK_SOURCE_48M) ;

  Reset() ;

  // Set master mode and enable SCL Clock Stretch mode (stretch after ACK bit)
  m_SercomRegStruct->I2CM.CTRLA.reg =  SERCOM_I2CM_CTRLA_MODE( I2C_MASTER_OPERATION )/* |
                            SERCOM_I2CM_CTRLA_SCLSM*/ ;

  // Enable Smart mode and Quick Command
  //I2CM.CTRLB.reg =  SERCOM_I2CM_CTRLB_SMEN /*| SERCOM_I2CM_CTRLB_QCEN*/ ;


  // Enable all interrupts
//  I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_ERROR ;

  // Synchronous arithmetic baudrate
  m_SercomRegStruct->I2CM.BAUD.bit.BAUD = SERCOM_FREQ_REF / ( 2 * baudrate) - 1 ;
}

void CI2c::prepareNackBitWIRE( void )
{
  if(isMasterWIRE()) 
  {
    // Send a NACK
    m_SercomRegStruct->I2CM.CTRLB.bit.ACKACT = 1;
  } 
  else 
  {
    m_SercomRegStruct->I2CS.CTRLB.bit.ACKACT = 1;
  }
}

void CI2c::prepareAckBitWIRE( void )
{
  if(isMasterWIRE()) 
  {
    // Send an ACK
    m_SercomRegStruct->I2CM.CTRLB.bit.ACKACT = 0;
  } 
  else 
  {
    m_SercomRegStruct->I2CS.CTRLB.bit.ACKACT = 0;
  }
}

void CI2c::prepareCommandBitsWire(uint8_t cmd)
{
  if(isMasterWIRE()) 
  {
    m_SercomRegStruct->I2CM.CTRLB.bit.CMD = cmd;

    while(m_SercomRegStruct->I2CM.SYNCBUSY.bit.SYSOP)
    {
      // Waiting for synchronization
    }
  } 
  else 
  {
    m_SercomRegStruct->I2CS.CTRLB.bit.CMD = cmd;
  }
}

bool CI2c::startTransmissionWIRE(uint8_t address, SercomWireReadWriteFlag flag)
{
  // 7-bits address + 1-bits R/W
  address = (address << 0x1ul) | flag;

  // Wait idle or owner bus mode
   while ( !isBusIdleWIRE() && !isBusOwnerWIRE() );

  // Send start and address
  m_SercomRegStruct->I2CM.ADDR.bit.ADDR = address;

  // Address Transmitted
  if ( flag == WIRE_WRITE_FLAG ) // Write mode
  {
    while( !m_SercomRegStruct->I2CM.INTFLAG.bit.MB )   // PSB paused here while out of control
    {
      // Wait transmission complete
    }
  }
  else  // Read mode
  {
    while( !m_SercomRegStruct->I2CM.INTFLAG.bit.SB )
    {
        // If the slave NACKS the address, the MB bit will be set.
        // In that case, send a stop condition and return false.
        if (m_SercomRegStruct->I2CM.INTFLAG.bit.MB) {
            m_SercomRegStruct->I2CM.CTRLB.bit.CMD = 3; // Stop condition
            return false;
        }
      // Wait transmission complete
    }

    // Clean the 'Slave on Bus' flag, for further usage.
    //I2CM.INTFLAG.bit.SB = 0x1ul;
  }


  //ACK received (0: ACK, 1: NACK)
  if(m_SercomRegStruct->I2CM.STATUS.bit.RXNACK)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool CI2c::sendDataMasterWIRE(uint8_t data)
{
	uint32_t StartTime = Core.millis();
	//Send data
	m_SercomRegStruct->I2CM.DATA.bit.DATA = data;

	//Wait transmission successful
	while(!m_SercomRegStruct->I2CM.INTFLAG.bit.MB) 
	{
		if((Core.millis() - StartTime)>10)
			return(false);
    
		// If a bus error occurs, the MB bit may never be set. Check the bus error bit and bail if it's set.
		if (m_SercomRegStruct->I2CM.STATUS.bit.BUSERR) 
		{
			return false;
		}
	}

	//Problems on line? nack received?
	if(m_SercomRegStruct->I2CM.STATUS.bit.RXNACK)
		return false;
	else
		return true;
}

bool CI2c::sendDataSlaveWIRE(uint8_t data)
{
  //Send data
  m_SercomRegStruct->I2CS.DATA.bit.DATA = data;

  //Problems on line? nack received?
  if(!m_SercomRegStruct->I2CS.INTFLAG.bit.DRDY || m_SercomRegStruct->I2CS.STATUS.bit.RXNACK)
    return false;
  else
    return true;
}

bool CI2c::isMasterWIRE( void )
{
  return m_SercomRegStruct->I2CS.CTRLA.bit.MODE == I2C_MASTER_OPERATION;
}

bool CI2c::isSlaveWIRE( void )
{
  return m_SercomRegStruct->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION;
}

bool CI2c::isBusIdleWIRE( void )
{
  return m_SercomRegStruct->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE;
}

bool CI2c::isBusOwnerWIRE( void )
{
  return m_SercomRegStruct->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE;
}

bool CI2c::isDataReadyWIRE( void )
{
  return m_SercomRegStruct->I2CS.INTFLAG.bit.DRDY;
}

bool CI2c::isStopDetectedWIRE( void )
{
  return m_SercomRegStruct->I2CS.INTFLAG.bit.PREC;
}

bool CI2c::isRestartDetectedWIRE( void )
{
  return m_SercomRegStruct->I2CS.STATUS.bit.SR;
}

bool CI2c::isAddressMatch( void )
{
  return m_SercomRegStruct->I2CS.INTFLAG.bit.AMATCH;
}

bool CI2c::isMasterReadOperationWIRE( void )
{
  return m_SercomRegStruct->I2CS.STATUS.bit.DIR;
}

bool CI2c::isRXNackReceivedWIRE( void )
{
  return m_SercomRegStruct->I2CM.STATUS.bit.RXNACK;
}

int CI2c::Available( void )
{
  if(isMasterWIRE())
    return m_SercomRegStruct->I2CM.INTFLAG.bit.SB;
  else
    return m_SercomRegStruct->I2CS.INTFLAG.bit.DRDY;
}

uint8_t CI2c::readDataWIRE( void )
{
  if(isMasterWIRE())
  {
    while( m_SercomRegStruct->I2CM.INTFLAG.bit.SB == 0 )
    {
      // Waiting complete receive
    }

    return m_SercomRegStruct->I2CM.DATA.bit.DATA ;
  }
  else
  {
    return m_SercomRegStruct->I2CS.DATA.reg ;
  }
}





void CI2c::SetClock(uint32_t baudrate) 
{
  disableWIRE();
  InitMaster(baudrate);
  Enable();
}

// void CI2c::end() 
// {
//   disableWIRE();
// }


bool CI2c::ReadRegister(uint8_t SlaveAddress, uint8_t Register, uint8_t *Bytes, uint8_t Size)
{
	uint8_t i = 0;
	if ( !startTransmissionWIRE( SlaveAddress, WIRE_WRITE_FLAG ) )
	{
		prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		UpdateErrors();
		return(false) ;  // Address error
	}
	if ( !sendDataMasterWIRE( Register ) )
	{
		prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		UpdateErrors();
		return(false) ;  // Nack or error
	}
	
	 if(startTransmissionWIRE(SlaveAddress, WIRE_READ_FLAG))
	 {
		 // Read first data
	 Bytes[i++] = readDataWIRE();

		 if(Size>1)
		 {
			 // Connected to slave
			 for (uint8_t byteRead = 1; byteRead < Size; ++byteRead)
			 {
				 prepareAckBitWIRE();                          // Prepare Acknowledge
				 prepareCommandBitsWire(WIRE_MASTER_ACT_READ); // Prepare the ACK command for the slave
				 Bytes[i++] = readDataWIRE();          // Read data and send the ACK
			 }
		 }
		 prepareNackBitWIRE();                           // Prepare NACK to stop slave transmission
		 //readDataWIRE();                               // Clear data register to send NACK
		 prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);   // Send Stop
	 }

	return(true);
}

bool CI2c::ReadRegisterAdd16(uint8_t SlaveAddress, uint16_t Register, uint8_t *Bytes, uint8_t Size)
{
	uint8_t i = 0;
	uint8_t* pAddress =(uint8_t*)&Register;
	DebugAddress1 = pAddress[0];
	DebugAddress2 = pAddress[1]	;
	if ( !startTransmissionWIRE(SlaveAddress, WIRE_WRITE_FLAG ))
	{
		prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		UpdateErrors();
		return(false) ;  // Address error
	}
	if ( !sendDataMasterWIRE( pAddress[1] ) ||  !sendDataMasterWIRE( pAddress[0] ))
	{
		prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		UpdateErrors();
		return(false) ;  // Nack or error
	}
	
	 if(startTransmissionWIRE(SlaveAddress, WIRE_READ_FLAG))
	 {
		 // Read first data
	 Bytes[i++] = readDataWIRE();

		 if(Size>1)
		 {
			 // Connected to slave
			 for (uint8_t byteRead = 1; byteRead < Size; ++byteRead)
			 {
				 prepareAckBitWIRE();                          // Prepare Acknowledge
				 prepareCommandBitsWire(WIRE_MASTER_ACT_READ); // Prepare the ACK command for the slave
				 Bytes[i++] = readDataWIRE();          // Read data and send the ACK
			 }
		 }
		 prepareNackBitWIRE();                           // Prepare NACK to stop slave transmission
		 //readDataWIRE();                               // Clear data register to send NACK
		 prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);   // Send Stop
	 }

	return(true);
}



uint8_t CI2c::ReadBytes(uint8_t address, uint8_t *Bytes,uint8_t Size)
{
  size_t byteRead = 0;


  if(startTransmissionWIRE(address, WIRE_READ_FLAG))
  {
	// Read first data
	Bytes[0] = readDataWIRE();

	if(Size>1)
	{
		// Connected to slave
		for (byteRead = 1; byteRead < Size; ++byteRead)
		{
			prepareAckBitWIRE();                          // Prepare Acknowledge
			prepareCommandBitsWire(WIRE_MASTER_ACT_READ); // Prepare the ACK command for the slave
			Bytes[byteRead] = readDataWIRE();          // Read data and send the ACK
		}
	}
	prepareNackBitWIRE();                           // Prepare NACK to stop slave transmission
	//readDataWIRE();                               // Clear data register to send NACK
	prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);   // Send Stop
	}

	return byteRead;
}

bool CI2c::WriteByte(uint8_t SlaveAddress , uint8_t Data )
{
	return(WriteBytes( SlaveAddress , &Data,1 ));
}

bool CI2c::WriteBytes(uint8_t SlaveAddress , uint8_t *Data,unsigned int DataLength )
{
	uint8_t i;
	// Start I2C transmission
	if ( !startTransmissionWIRE( SlaveAddress, WIRE_WRITE_FLAG ) )
	{
		prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		UpdateErrors();
		return(false) ;  // Address error
	}

	// Send all buffer
	for(i=0;i<DataLength;i++)
	{
		// Trying to send data
		if ( !sendDataMasterWIRE( Data[i] ) )
		{
			prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
			return 3 ;  // Nack or error
		}
	}
	prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
	return(true);
}


bool CI2c::WriteRegisterByte(uint8_t SlaveAddress,uint8_t SlaveRegister , uint8_t Data)
{
	return(WriteRegisterBytes(SlaveAddress,SlaveRegister ,&Data,1));
}

bool CI2c::WriteRegisterBytes(uint8_t SlaveAddress,uint8_t SlaveRegister , uint8_t *Data,unsigned int DataLength)
{
	uint8_t i;
	uint8_t ii = DataLength -1;
	if ( !startTransmissionWIRE( SlaveAddress, WIRE_WRITE_FLAG ) )		// Start I2C transmission
	{
		prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		UpdateErrors();
		return(false) ;  // Address error
	}

	// Send all buffer
	sendDataMasterWIRE( SlaveRegister);
	for(i=0;i<DataLength;i++)
	{
		// Trying to send data
		if ( !sendDataMasterWIRE( Data[ii--] ) )
		{
			prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
			return 3 ;  // Nack or error
		}
	}
	prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
	return(true);
}

bool CI2c::WriteRegisterBytesAdd16(uint8_t SlaveAddress,uint16_t SlaveRegister , uint8_t *Data,unsigned int DataLength)
{
	uint8_t i;
	uint8_t* pAddress = (uint8_t*)&SlaveRegister;
	uint8_t ii = DataLength -1;
	if ( !startTransmissionWIRE( SlaveAddress, WIRE_WRITE_FLAG ) )		// Start I2C transmission
	{
		prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		UpdateErrors();
		return(false) ;  // Address error
	}

	// Send all buffer
	sendDataMasterWIRE( pAddress[1]);
	sendDataMasterWIRE( pAddress[0]);
	for(i=0;i<DataLength;i++)
	{
		// Trying to send data
		if ( !sendDataMasterWIRE( Data[ii--] ) )
		{
			prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
			return 3 ;  // Nack or error
		}
	}
	prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
	return(true);
}





void CI2c::UpdateErrors()
{
	if(m_I2cErrorsTotal++ > 100)
		Config.m_RunningFlags.I2C_INTERFACE_FAIL = true;
}

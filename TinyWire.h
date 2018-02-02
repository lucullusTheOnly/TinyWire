/*****************************************************************************
*
*
* File              TinyWire.h
* Date              Saturday, 10/29/17
* Composed by 		lucullus
*
*
* Wrapperclass for the new composed TinyWire library for I2C communication on ATTiny's
* The complete library is based on the librarys TinyWireM and TinyWireS, which were originally posted by
* BroHogan and then modified by jkl. I recomposed their code to unite slave and master functionality similar
* to the behavior of the Wire library on an Arduino
*
*
* NOTE! - It's very important to use pullups on the SDA & SCL lines! More so than with the Wire lib.
*
*
*
* TODO:
* - Reenable the random read function from TinyWireM (I don't really understand how it works and what it is for)
*
*  This library is free software; you can redistribute it and/or modify it under the
*  terms of the GNU General Public License as published by the Free Software
*  Foundation; either version 2.1 of the License, or any later version.
*  This program is distributed in the hope that it will be useful, but WITHOUT ANY
*  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
*  PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*********************************************************************************/

#ifndef tiny_twi_h
#define tiny_twi_h
	#include <inttypes.h>
	
	//********** Master Error Codes **********//
	// Note these have been renumbered from the Atmel Apps Note. Most likely errors are now
	//		lowest numbers so they're easily recognized as LED flashes.
	#define USI_TWI_NO_DATA             0x08  // Transmission buffer is empty
	#define USI_TWI_DATA_OUT_OF_BOUND   0x09  // Transmission buffer is outside SRAM space
	#define USI_TWI_UE_START_CON        0x07  // Unexpected Start Condition
	#define USI_TWI_UE_STOP_CON         0x06  // Unexpected Stop Condition
	#define USI_TWI_UE_DATA_COL         0x05  // Unexpected Data Collision (arbitration)
	#define USI_TWI_NO_ACK_ON_DATA      0x02  // The slave did not acknowledge  all data
	#define USI_TWI_NO_ACK_ON_ADDRESS   0x01  // The slave did not acknowledge  the address
	#define USI_TWI_MISSING_START_CON   0x03  // Generated Start Condition not detected on bus
	#define USI_TWI_MISSING_STOP_CON    0x04  // Generated Stop Condition not detected on bus
	#define USI_TWI_BAD_MEM_READ	    0x0A  // Error during external memory read
	#define USI_TWI_BUS_BUSY            0x0B  // Another Master is using the bus
	#define USI_TWI_ARBITRATION_LOST    0x0C  // The master lost the arbitration due to the transmission of another master

	//********** Class Definition **********//
	class TinyTwi
	{
		private:
		bool master_mode=false;
		bool temp_master_mode=false;
		uint8_t slave_addr=0;
		static void (*user_onRequest)(void);
    	static void (*user_onReceive)(int);
    	static void onRequestService(void);
    	static void onReceiveService(int numBytes);

		public:
		TinyTwi();
		void begin();
		void begin(uint8_t I2C_SLAVE_ADDR);
		uint8_t read();
		uint8_t receive();
		uint8_t available();
		uint8_t send(uint8_t data);
		uint8_t write(uint8_t data);
		void beginTransmission(uint8_t slaveAddr);
		uint8_t endTransmission();
		uint8_t requestFrom(uint8_t slaveAddr, uint8_t numBytes);
		void end();
		
		void onReceive( void (*)(int) );
    	void onRequest( void (*)(void) );
	};

	extern TinyTwi TinyWire;

#endif

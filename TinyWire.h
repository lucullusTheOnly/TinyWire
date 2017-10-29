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
* For
*
*
* TODO:
* - Reenable the random read function from TinyWireM (I don't really understand how it works and what it is for)
* - Calling an user event function on slave receive doesn't work right, because the end of data transmit is not
*   reliable indicated by a Stop- or Restart-Condition (Sending from an Arduino Nano doesn't seem to trigger the
*   interrupt)
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
		uint8_t available();
		void send(uint8_t data);
		void beginTransmission(uint8_t slaveAddr);
		uint8_t endTransmission();
		uint8_t requestFrom(uint8_t slaveAddr, uint8_t numBytes);
		void end();
		
		//void onReceive( void (*)(int) ); // doesn't work currently
    	void onRequest( void (*)(void) );
	};

	extern TinyTwi TinyWire;

#endif
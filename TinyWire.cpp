/*****************************************************************************
*
*
* File              TinyWire.cpp
* Date              Saturday, 10/29/17
* Composed by 		lucullus
*
*
*  **** See TinyWire.h for Credits and Usage information ****
*
*
*  This library is free software; you can redistribute it and/or modify it under the
*  terms of the GNU General Public License as published by the Free Software
*  Foundation; either version 2.1 of the License, or any later version.
*  This program is distributed in the hope that it will be useful, but WITHOUT ANY
*  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
*  PARTICULAR PURPOSE.  See the GNU General Public License for more details.
******************************************************************************/

#ifndef tiny_twi_cpp
#define tiny_twi_cpp

	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include "TinyWire.h"
	#include "twi.h"

	void (*TinyTwi::user_onRequest)(void);
	void (*TinyTwi::user_onReceive)(int);

	TinyTwi::TinyTwi(){
	}

	/*---------------------------------------------------
	 Initialize USI and library for master function
	-----------------------------------------------------*/
	void TinyTwi::end(){
		Twi_end();
	}

	void TinyTwi::begin(){
		master_mode = true;
		Twi_attachSlaveTxEvent(onRequestService);
  		Twi_attachSlaveRxEvent(onReceiveService);
		Twi_master_init();
	}

	/*---------------------------------------------------
	 Initialize USI and library for slave function
	 Parameter: uint8_t slave address
	-----------------------------------------------------*/
	void TinyTwi::begin(uint8_t I2C_SLAVE_ADDR){
		master_mode = false;
		slave_addr = I2C_SLAVE_ADDR;
		Twi_attachSlaveTxEvent(onRequestService);
  		Twi_attachSlaveRxEvent(onReceiveService);
		Twi_slave_init(I2C_SLAVE_ADDR);
	}

	uint8_t TinyTwi::send(uint8_t data){
		if(master_mode || temp_master_mode){
			return Twi_master_send(data);
		} else{
			return Twi_slave_send(data);
		}
	}

	uint8_t TinyTwi::write(uint8_t data){
		return send(data);
	}

	uint8_t TinyTwi::read(){
		return Twi_receive();
	}

	uint8_t TinyTwi::receive(){
		return read();
	}

	uint8_t TinyTwi::available(){
		return Twi_available();
	}

	void TinyTwi::beginTransmission(uint8_t slaveAddr)
	{
		Twi_master_beginTransmission(slaveAddr);
	}

	uint8_t TinyTwi::endTransmission()
	{
		uint8_t temp;

		if(!master_mode){
			temp_master_mode = true;
			Twi_master_init();
		}

		temp = Twi_master_endTransmission();
		if(temp_master_mode){
			temp_master_mode = false;
			begin(slave_addr);
		}
		return temp;
	}

	uint8_t TinyTwi::requestFrom(uint8_t slaveAddr, uint8_t numBytes)
	{
		uint8_t temp;
		if(!master_mode){
			temp_master_mode = true;
			Twi_master_init();
		}
		temp = Twi_master_requestFrom(slaveAddr, numBytes);
		if(temp_master_mode){
			temp_master_mode = false;
			begin(slave_addr);
		}
		return temp;
	}

	// behind the scenes function that is called when data is received
	void TinyTwi::onReceiveService(int numBytes)
	{
	  // don't bother if user hasn't registered a callback
	  if(!user_onReceive){
	    return;
	  }
	  // alert user program
	  user_onReceive(numBytes);
	}

	// behind the scenes function that is called when data is requested
	void TinyTwi::onRequestService(void)
	{
	  // don't bother if user hasn't registered a callback
	  if(!user_onRequest){
	    return;
	  }
	  // alert user program
	  user_onRequest();
	}

	void TinyTwi::onReceive( void (*function)(int) )
	{
		user_onReceive = function;
	}

    void TinyTwi::onRequest( void (*function)(void) )
    {
    	user_onRequest = function;
	}

	/*-----------------------------------------------------
	 Preinstantiate TinyWire Object
	-------------------------------------------------------*/
	TinyTwi TinyWire = TinyTwi();

#endif

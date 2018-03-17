/*****************************************************************************
*
*
* File              twi.h
* Date              Saturday, 10/29/17
* Composed by 		lucullus
*
* This library gives the functionality to provide a nearly full I2C communication with both master and slave mode
* through the USI hardware of an ATTiny (since it has no dedicated hardware for this purpose)
* The master functionality is done directly (not in an ISR) and in case of a requestFrom blocking. The slave
* functionality uses interrupt routines for detecting start and stop conditions and the overflow of the USI counter,
* which denotes a transmitted byte or a (N)ACK
*
* This is a composed library of TinyWireM and TinyWireS, to provide simultaneous slave and master configuration like
* the Wire library on an Arduino. The original libraries were posted by BroHogan and modified by jkl and the credit
* for writing the interface belongs to them.
*
* TODO:
* - Reenable the random read function from TinyWireM (I don't really understand how it works and what it is for)
*
*
*  This library is free software; you can redistribute it and/or modify it under the
*  terms of the GNU General Public License as published by the Free Software
*  Foundation; either version 2.1 of the License, or any later version.
*  This program is distributed in the hope that it will be useful, but WITHOUT ANY
*  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
*  PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*/

#ifndef __USI_TWI_H__
#define __USI_TWI_H__

/*--------------------------------------------------------------
 prototypes
----------------------------------------------------------------*/
void Twi_end();

void Twi_slave_init(uint8_t slave_addr);
uint8_t Twi_slave_send(uint8_t databyte);
uint8_t Twi_receive(void);
uint8_t Twi_available(void);

void Twi_master_init(void);
void Twi_master_beginTransmission(uint8_t slave_addr);
uint8_t Twi_master_send(uint8_t data);
uint8_t Twi_master_endTransmission();
unsigned char USI_TWI_Get_State_Info( void );
uint8_t Twi_master_requestFrom(uint8_t slave_addr, uint8_t numBytes);
void Twi_attachSlaveRxEvent( void (*function)(int) );
void Twi_attachSlaveTxEvent( void (*function)(void) );

/*--------------------------------------------------------------
 buffer definitions
----------------------------------------------------------------*/
// permitted RX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256

#define TWI_RX_BUFFER_SIZE  ( 32 ) // jjg was 16
#define TWI_RX_BUFFER_MASK  ( TWI_RX_BUFFER_SIZE - 1 )

#if ( TWI_RX_BUFFER_SIZE & TWI_RX_BUFFER_MASK )
#  error TWI RX buffer size is not a power of 2
#endif

// permitted TX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256

#define TWI_TX_BUFFER_SIZE ( 32 ) // jjg was 16
#define TWI_TX_BUFFER_MASK ( TWI_TX_BUFFER_SIZE - 1 )

#if ( TWI_TX_BUFFER_SIZE & TWI_TX_BUFFER_MASK )
#  error TWI TX buffer size is not a power of 2
#endif

/*--------------------------------------------------------------
 Master definitions
----------------------------------------------------------------*/
// Defines controlling timing limits - SCL <= 100KHz.
#define SYS_CLK   1000.0  // [kHz]	Default for ATtiny2313

// For use with _delay_us()
#define T2_TWI    5 		// >4,7us
#define T4_TWI    4 		// >4,0us

// Defines error code generating
//#define PARAM_VERIFICATION
//#define NOISE_TESTING
#define BUS_ARBITRATION
#define SIGNAL_VERIFY		// This should probably be on always.

/****************************************************************************
  Bit definitions
****************************************************************************/
#define TWI_READ_BIT  0       // Bit position for R/W bit in "address byte".
#define TWI_ADR_BITS  1       // Bit position for LSB of the slave address bits in the init byte.
#define TWI_NACK_BIT  0       // Bit position for (N)ACK bit.
#define USI_SEND         0              // indicates sending to TWI
#define USI_RCVE         1              // indicates receiving from TWI
#define USI_BUF_SIZE    16              // bytes in message buffer

// Note these have been renumbered from the Atmel Apps Note. Most likely errors are now
//		lowest numbers so they're easily recognized as LED flashes.
#define USI_TWI_NO_DATA             0x08  // Transmission buffer is empty
#define USI_TWI_DATA_OUT_OF_BOUND   0x09  // Transmission buffer is outside SRAM space
#define USI_TWI_UE_START_CON        0x07  // Unexpected Start Condition
#define USI_TWI_UE_STOP_CON         0x06  // Unexpected Stop Condition
#define USI_TWI_UE_DATA_COL         0x05  // Unexpected Data Collision
#define USI_TWI_NO_ACK_ON_DATA      0x02  // The slave did not acknowledge  all data
#define USI_TWI_NO_ACK_ON_ADDRESS   0x01  // The slave did not acknowledge  the address
#define USI_TWI_MISSING_START_CON   0x03  // Generated Start Condition not detected on bus
#define USI_TWI_MISSING_STOP_CON    0x04  // Generated Stop Condition not detected on bus
#define USI_TWI_BAD_MEM_READ	      0x0A  // Error during external memory read
#define USI_TWI_BUS_BUSY            0x0B  // Another Master is using the bus
#define USI_TWI_ARBITRATION_LOST    0x0C  // The master lost the arbitration due to the transmission of another master

#define TRUE  1
#define FALSE 0

/*--------------------------------------------------------------
 device dependent definitions
----------------------------------------------------------------*/
#if defined( __AVR_ATtiny2313__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB5
#  define PORT_USI_SCL        PB7
#  define PIN_USI_SDA         PINB5
#  define PIN_USI_SCL         PINB7
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATtiny24__ ) | \
     defined( __AVR_ATtiny44__ ) | \
     defined( __AVR_ATtiny84__ )
#  define DDR_USI             DDRA
#  define PORT_USI            PORTA
#  define PIN_USI             PINA
#  define PORT_USI_SDA        PA6
#  define PORT_USI_SCL        PA4
#  define PIN_USI_SDA         PINA6
#  define PIN_USI_SCL         PINA4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_STR_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#  define PIN_CHANGE_INTERRUPT_ENABLE PCIE0
#  define PIN_CHANGE_FLAG     PCIF0
#  define PIN_CHANGE_MASK     PCMSK0
#  define GENERAL_INTERRUPT_MASK GIMSK
#  define GENERAL_INTERRUPT_FLAGS GIFR
#endif

#if defined( __AVR_ATtiny25__ ) | \
     defined( __AVR_ATtiny45__ ) | \
     defined( __AVR_ATtiny85__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF //was USICIF jjg
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#  define PIN_CHANGE_INTERRUPT_ENABLE PCIE
#  define PIN_CHANGE_FLAG     PCIF
#  define PIN_CHANGE_MASK     PCMSK
#  define GENERAL_INTERRUPT_MASK GIMSK
#  define GENERAL_INTERRUPT_FLAGS GIFR
#endif

#if defined( __AVR_ATtiny26__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_STRT_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny261__ ) | \
     defined( __AVR_ATtiny461__ ) | \
     defined( __AVR_ATtiny861__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATmega165__ ) | \
     defined( __AVR_ATmega325__ ) | \
     defined( __AVR_ATmega3250__ ) | \
     defined( __AVR_ATmega645__ ) | \
     defined( __AVR_ATmega6450__ ) | \
     defined( __AVR_ATmega329__ ) | \
     defined( __AVR_ATmega3290__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATmega169__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#endif

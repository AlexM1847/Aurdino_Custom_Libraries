/*
 * DYNAMIC_DMX_SERIAL.h	-	A Aurduino Library for sending and receiving DMX using a variable serail port
 * Accompanied Files:
 * 						Dynamic_DMX_Serial.cpp
 * 						????
 * 						????
 *
 * THE ORIGINAL CODE BUILT by Matthias Hertel (http://www.mathertel.de)
 * THE ORIGINAL CODE file "DMXSERIAL", code and documentation at http://www.mathertel.de/Arduino
 * THE ORIGINAL WORK is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
 * 26.02.2020 & Earlier notes have been earased from this document and can be found in the original.
 *
 * THIS VERSION BUILT on DMXSerial Version 1.5.0
 * THIS VERSION BUILT by Alexander Marino
 * NOTES:
 * 			This Library will only work on AVR Styled Boards.
 *
 * 25.07.2011 creation of the DYNAMIC_DMX_SERIAL library.
 *
 *
 *
// - - - - -
 */

#ifndef DYNAMIC_DMX_SERIAL_H
#define DYNAMIC_DMX_SERIAL_H

#include <avr/io.h>

	// -----  Constants -----
	#define DMXSERIAL_MAX 512	// MAX DMX Channles

	#define DMXMODEPIN 2		// AURDUINO PIN 2 for controlling the data direction is the default value.
	#define DmxModeOut HIGH  	// set the level to HIGH for outgoing data direction
	#define DmxModeIn  LOW   	// set the level to LOW  for incoming data direction

	#define DMXPROBE_RECV_MAX 50 // standard maximum of waiting for a DMX packet in DMXPROBE mode.

	// ----- Enumerations -----
		// REQUIRED TO SET MODE TYPE //
	typedef enum {
	  DMXNone,			// unspecified
	  DMXController,	// always sending
	  DMXReceiver,		// always listening
	  DMXProbe			// send and receive upon Request
	  } DMXMode;
	  	// REQUIRED TO TELL SERIAL PORT TYPE //
	  typedef enum {
		 SerialInterface0 = 0,		// Serial 0
		 SerialInterface1 = 1,		// Serial 1
		 SerialInterface2 = 2,		// Serial 2
		 SerialInterface3 = 3,		// Serial 3
	 } SerialInterface;


	// ----- C Format Declertions -----
	  	  // WHAT DOES THIS DO? //
	  extern "C" {
	    typedef void (*dmxUpdateFunction)(void);
	  }

// ------ Library Class -----
  /* Description:		Aurduino Library to Send and Recive DMX using a max chip
   *
   * OG DESCRIPTION
   * 	he library works unchanged with the Arduino 2009, UNO, MEGA 2560 and Leonardo boards. <br />
	*	The Arduino MEGA 2560 boards use the serial port 0 on pins 0 an 1. <br />
	*	The Arduino Leonardo will use serial port 1, also on pins 0 an 1. (on the 32u4 boards the first USART is USART1) <br />
	* 	This is consistent with the Layout of the Arduino DMX Shield http://www.mathertel.de/Arduino/DMXShield.aspx.
	   */
class DynamicDMXSerialClass
{
	public:
		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.Init
		//	Description:	This function is used to initialize the class with std initilization
		//	Input:			int mode
		//	Return:			void
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		void Init (int mode);


		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.Init
		//	Description:	This function is intended Initialize the specified mode
		//						including a specific mode pin and int SerialInterface
		//	Input:			int mode, int modePin
		//	Return:			void
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		void Init (int mode, int modePin);

		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.MaxChannel
		//	Description:	@brief Set the maximum used channel for DMXController mode.
		//	Input:			int channel
		//	Return:			void
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		void MaxChannel (int channel);

		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.Read
		//	Description:	Read the channle
		//	Input:			int channel
		//	Return:			uint8_t value
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		uint8_t Read (int channel);

		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.Write
		//	Description:	Write a value to a channle
		//	Input:			uint16_t channel to write to, uint8_t value to write
		//	Return:			Void
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		void Write (uint16_t channel, uint8_t value);

		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.getBuffer
		//	Description:	Gets a pointer to the DMX Buffer
		//	Input:			Void
		//	Return:			uint8_t * pointer to buffer
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		uint8_t *GetBuffer();

		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.noDataSince
		//	Description:	return the duration since data was received.
		//	Input:			Void
		//	Return:			return unsigned long milliseconds since last pdata package.
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		unsigned long NoDataSince();


		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.dataUpdated
		//	Description:	check for changed data
		//	Input:			Void
		//	Return:			bool of recived data
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		bool DataUpdated();

		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.ResetUpdate
		//	Description:	Briefe reset once updated data
		//	Input:			Void
		//	Return:			Void
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		void ResetUpdated();

		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.Receive
		//	Description:	Actively wait for an incoming DMX packet.
		//	Input:			Void
		//	Return:			Bool, True or false of data or timeout
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		bool Receive();

		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.Receive
		//	Description:	Actively wait for an incoming DMX packet for a specified millisecond
		//	Input:			uint8_t time to wait in MS
		//	Return:			Bool, True or false of data or timeout
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		bool Receive(uint8_t wait);

		///////////////////////////////////////////////////////////////////////////////////////////
		//	Name:			.Term
		//	Description:	Terminate the operation mode
		//	Input:			Void
		//	Return:			Void
		//	Dependancies:	DMXSerial Include
		//	Notes:			None.
		///////////////////////////////////////////////////////////////////////////////////////////
		void Term();
private:
	  // Not used.
	  // all private information is in the global _dmxXXX variables for speed and code size optimization.
	  // @see DMXSerial.cpp.
};

#endif /* DYNAMIC_DMX_SERIAL_H */

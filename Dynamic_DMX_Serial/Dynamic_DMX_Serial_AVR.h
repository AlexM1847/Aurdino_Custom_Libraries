// - - - - -
// Dynamic_DMX_Serial_AVR.h - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// DMXSerial_avr.h: Hardware specific functions for AVR processors like ATmega168 and ATmega328 used in Aurduino UNO.
// Also supported boards are Leonardo and Mega2560.
//
// Copyright (c) 2011-2020 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// - - - - -

// global variables and functions are prefixed with "_DMX_"

// ----- ATMega specific hardware related functions -----

#ifndef DYNAMIC_DMX_SERIAL_AVR_H
#define DYNAMIC_DMX_SERIAL_AVR_H

	#define DMX_USE_PORT3

	#if defined(DMXFORMAT) && defined(ARDUINO_ARCH_AVR)

		#include "Arduino.h"
		#include "Dynamic_DMX_Serial.h"
		#include "avr/io.h"
		#include "avr/interrupt.h"

		// Board Selection
		#if	defined(USART_RXC_vect)
			// USED ON ATmega8 CHIPS
			// Boards with and 8 Bit SERIAL like ATmega8 boards
			#define UCSRnA UCSRA	// Control and Status Register A
			#define TXCn TXC		// Transmit buffer clear

			#define UCSRnB UCSRB 	// USART Control and Status Register B

			#define RXCIEn RXCIE	// Enable Receive Complete Interrupt
			#define TXCIEn TXCIE	// Enable Transmission Complete Interrupt
			#define UDRIEn UDRIE	// Enable Data Register Empty Interrupt
			#define RXENn RXEN		// Enable Receiving
			#define TXENn TXEN		// Enable Sending

			#define UCSRnC UCSRC	// Control and Status Register C
			#define USBSn USBS		// Stop bit select 0=1bit, 1=2bits
			#define UCSZn0 UCSZ0	// Character size 00=5, 01=6, 10=7, 11=8 bits
			#define UPMn0 UPM0		// Parity setting 00=N, 10=E, 11=O

			#define UBRRnH UBRRH	// USART Baud Rate Register High
			#define UBRRnL UBRRL	// USART Baud Rate Register Low

			#define UDRn UDR		// USART Data Register
			#define UDREn UDRE		// USART Data Ready
			#define FEn FE			// Frame Error

			#define USARTn_RX_vect USART_RXC_vect 		// Interrupt Data received
			#define USARTn_TX_vect USART_TXC_vect		// Interrupt Data sent
			#define USARTn_UDRE_vect USART_UDRE_vect	// Interrupt Data Register empty

		#elif defined(USART_RX_vect)
			// USED ON SINGLE SERIAL ATMega168p / ATmega328p CHIPS
			// SINGLE SERIAL ONLY ('USART_RX_VECT' has just 'USART' not 'USART0'
			#define UCSRnA UCSR0A
			#define RXCn RXC0
			#define TXCn TXC0
			#define UCSRnB UCSR0B
			#define RXCIEn RXCIE0
			#define TXCIEn TXCIE0
			#define UDRIEn UDRIE0
			#define RXENn RXEN0
			#define TXENn TXEN0
			#define UCSRnC UCSR0C
			#define USBSn USBS0
			#define UCSZn0 UCSZ00
			#define UPMn0 UPM00
			#define UBRRnH UBRR0H
			#define UBRRnL UBRR0L
			#define UDRn UDR0
			#define UDREn UDRE0
			#define FEn FE0
			#define USARTn_RX_vect USART_RX_vect
			#define USARTn_TX_vect USART_TX_vect
			#define USARTn_UDRE_vect USART_UDRE_vect

		#elif defined(DMX_USE_PORT3) && defined(USART3_RX_vect)
			// SERIAL PORT 3 CASE ON ATMega2600 STYLE BOARDS
			// Multiple Serial Ports
			#define UCSRnA UCSR3A	// Control and Status Register A
			#define RXCn RXC3		// Recive Buffer Clear
			#define TXCn TXC3		// Transmit Buffer Clear

			#define UCSRnB UCSR3B	// Control and Status of Register B
			#define RXCIEn RXCIE3	// Enable Receive Complete Interrupt
			#define TXCIEn TXCIE3	// Enable Transmission Complete Interrupt
			#define UDRIEn UDRIE3	// Enable Data Register Empty Interrupt
			#define RXENn RXEN3		// Enable Receiving
			#define TXENn TXEN3		// Enable Transmittinbg

			#define UCSRnC UCSR3C	// Control and Status of Register B

			#define USBSn USBS3		// Stop bit select 0=1bit, 1=2bits
			#define UCSZn0 UCSZ30	// Character size 00=5, 01=6, 10=7, 11=8 bits
			#define UPMn0 UPM30		// Parity setting 00=N, 10=E, 11=O

			#define UBRRnH UBRR3H	// USART Baud Rate Register High
			#define UBRRnL UBRR3L	// USART Baud Rate Register LOW

			#define UDRn UDR3		// USART Data Register
			#define UDREn UDRE3		// USART Data Ready
			#define FEn FE3			// Frame Error

			#define USARTn_RX_vect USART3_RX_vect		// Interrupt Data received
			#define USARTn_TX_vect USART3_TX_vect		// Interrupt Data Transmitted
			#define USARTn_UDRE_vect USART3_UDRE_vect	// Interrupt Data Register empty

		#elif defined(DMX_USE_PORT2) && defined(USART2_RX_vect)
			// SERIAL PORT 2 CASE ON ATMega2600 STYLE BOARDS
			// Multiple Serial Ports
			#define UCSRnA UCSR2A	// Control and Status Register A
			#define RXCn RXC2		// Recive Buffer Clear
			#define TXCn TXC2		// Transmit Buffer Clear

			#define UCSRnB UCSR2B	// Control and Status of Register B
			#define RXCIEn RXCIE2	// Enable Receive Complete Interrupt
			#define TXCIEn TXCIE2	// Enable Transmission Complete Interrupt
			#define UDRIEn UDRIE2	// Enable Data Register Empty Interrupt
			#define RXENn RXEN2		// Enable Receiving
			#define TXENn TXEN2		// Enable Transmittinbg

			#define UCSRnC UCSR2C	// Control and Status of Register B

			#define USBSn USBS2		// Stop bit select 0=1bit, 1=2bits
			#define UCSZn0 UCSZ20	// Character size 00=5, 01=6, 10=7, 11=8 bits
			#define UPMn0 UPM20		// Parity setting 00=N, 10=E, 11=O

			#define UBRRnH UBRR2H	// USART Baud Rate Register High
			#define UBRRnL UBRR2L	// USART Baud Rate Register LOW

			#define UDRn UDR2		// USART Data Register
			#define UDREn UDRE2		// USART Data Ready
			#define FEn FE2			// Frame Error

			#define USARTn_RX_vect USART2_RX_vect		// Interrupt Data received
			#define USARTn_TX_vect USART2_TX_vect		// Interrupt Data Transmitted
			#define USARTn_UDRE_vect USART2_UDRE_vect	// Interrupt Data Register empty

		#elif defined(DMX_USE_PORT1) && defined(USART1_RX_vect)
			// SERIAL PORT 1 CASE ON ATMega2600 STYLE BOARDS
			// Multiple Serial Ports
			#define UCSRnA UCSR1A	// Control and Status Register A
			#define RXCn RXC1		// Recive Buffer Clear
			#define TXCn TXC1		// Transmit Buffer Clear

			#define UCSRnB UCSR1B	// Control and Status of Register B
			#define RXCIEn RXCIE1	// Enable Receive Complete Interrupt
			#define TXCIEn TXCIE1	// Enable Transmission Complete Interrupt
			#define UDRIEn UDRIE1	// Enable Data Register Empty Interrupt
			#define RXENn RXEN1		// Enable Receiving
			#define TXENn TXEN1		// Enable Transmittinbg

			#define UCSRnC UCSR1C	// Control and Status of Register B

			#define USBSn USBS1		// Stop bit select 0=1bit, 1=2bits
			#define UCSZn0 UCSZ10	// Character size 00=5, 01=6, 10=7, 11=8 bits
			#define UPMn0 UPM10		// Parity setting 00=N, 10=E, 11=O

			#define UBRRnH UBRR1H	// USART Baud Rate Register High
			#define UBRRnL UBRR1L	// USART Baud Rate Register LOW

			#define UDRn UDR1		// USART Data Register
			#define UDREn UDRE1		// USART Data Ready
			#define FEn FE1			// Frame Error

			#define USARTn_RX_vect USART1_RX_vect		// Interrupt Data received
			#define USARTn_TX_vect USART1_TX_vect		// Interrupt Data Transmitted
			#define USARTn_UDRE_vect USART1_UDRE_vect	// Interrupt Data Register empty

		#elif defined(USART0_RX_vect)
			// SERIAL PORT 0CASE ON ATMega2600 STYLE BOARDS (DEFAULT)
			// Multiple Serial Ports
			#define UCSRnA UCSR0A	// Control and Status Register A
			#define RXCn RXC0		// Recive Buffer Clear
			#define TXCn TXC0		// Transmit Buffer Clear

			#define UCSRnB UCSR0B	// Control and Status of Register B
			#define RXCIEn RXCIE0	// Enable Receive Complete Interrupt
			#define TXCIEn TXCIE0	// Enable Transmission Complete Interrupt
			#define UDRIEn UDRIE0	// Enable Data Register Empty Interrupt
			#define RXENn RXEN0		// Enable Receiving
			#define TXENn TXEN0		// Enable Transmittinbg

			#define UCSRnC UCSR0C	// Control and Status of Register B

			#define USBSn USBS0		// Stop bit select 0=1bit, 1=2bits
			#define UCSZn0 UCSZ00	// Character size 00=5, 01=6, 10=7, 11=8 bits
			#define UPMn0 UPM00		// Parity setting 00=N, 10=E, 11=O

			#define UBRRnH UBRR0H	// USART Baud Rate Register High
			#define UBRRnL UBRR0L	// USART Baud Rate Register LOW

			#define UDRn UDR0		// USART Data Register
			#define UDREn UDRE0		// USART Data Ready
			#define FEn FE0			// Frame Error

			#define USARTn_RX_vect USART0_RX_vect		// Interrupt Data received
			#define USARTn_TX_vect USART0_TX_vect		// Interrupt Data Transmitted
			#define USARTn_UDRE_vect USART0_UDRE_vect	// Interrupt Data Register empty
		#endif

		// ----- ATMega specific Hardware abstraction functions -----

		// calculate prescaler from baud rate and cpu clock rate at compile time.
		// This is a processor specific formular from the datasheet.
		// It implements rounding of ((clock / 16) / baud) - 1.
		#define CalcPreScale(B) (((((F_CPU) / 8) / (B)) - 1) / 2)

		const int32_t _DMX_dmxPreScale = CalcPreScale(DMXSPEED); // BAUD prescale factor for DMX speed.
		const int32_t _DMX_breakPreScale = CalcPreScale(BREAKSPEED); // BAUD prescale factor for BREAK speed.

		// initialize mode independent registers.
		void _DMX_init() {
		  // 04.06.2012: use normal speed operation
		  UCSRnA = 0;
		} // _DMX_init()


		/// Initialize the Hardware UART serial port registers to the required mode.
		void _DMX_setMode(DMXUARTMode mode) {
		  if (mode == DMXUARTMode::OFF) {
			UCSRnB = 0;

		  } else if (mode == DMXUARTMode::RONLY) {
			// assign the baud_setting to the USART Baud Rate Register
			UBRRnH = _DMX_dmxPreScale >> 8;
			UBRRnL = _DMX_dmxPreScale;
			// enable USART functions RX, TX, Interrupts
			UCSRnB = (1 << RXENn);
			// stop bits and character size
			UCSRnC = DMXREADFORMAT; // accept data packets after first stop bit

		  } else if (mode == DMXUARTMode::RDATA) {
			UBRRnH = _DMX_dmxPreScale >> 8;
			UBRRnL = _DMX_dmxPreScale;
			UCSRnB = (1 << RXENn) | (1 << RXCIEn);
			UCSRnC = DMXREADFORMAT; // accept data packets after first stop bit

		  } else if (mode == DMXUARTMode::TBREAK) {
			UBRRnH = _DMX_breakPreScale >> 8;
			UBRRnL = _DMX_breakPreScale;
			UCSRnB = ((1 << TXENn) | (1 << TXCIEn));
			UCSRnC = BREAKFORMAT;

		  } else if (mode == DMXUARTMode::TDATA) {
			UBRRnH = _DMX_dmxPreScale >> 8;
			UBRRnL = _DMX_dmxPreScale;
			UCSRnB = ((1 << TXENn) | (1 << UDRIEn));
			UCSRnC = DMXFORMAT; // send with 2 stop bits for compatibility

		  } else if (mode == DMXUARTMode::TDONE) {
			UBRRnH = _DMX_dmxPreScale >> 8;
			UBRRnL = _DMX_dmxPreScale;
			UCSRnB = ((1 << TXENn) | (1 << TXCIEn));
			UCSRnC = DMXFORMAT; // send with 2 stop bits for compatibility
		  } // if
		} // _DMX_setMode()


		// flush all incomming data packets in the queue
		void _DMX_flush() {
		  uint8_t voiddata;
		  while (UCSRnA & (1 << RXCn)) {
			voiddata = UDRn; // get data
		  }
		}


		// send the next byte after current byte was sent completely.
		inline void _DMX_writeByte(uint8_t data) {
		  // putting data into buffer sends the data
		  UDRn = data;
		} // _DMX_writeByte


		// This Interrupt Service Routine is called when a byte or frame error was received.
		// In DMXController mode this interrupt is disabled and will not occur.
		// In DMXReceiver mode when a byte or frame error was received
		ISR(USARTn_RX_vect) {
		  uint8_t rxferr = (UCSRnA & (1 << FEn)); // get state before data!
		  uint8_t rxdata = UDRn; // get data
		  _DMXReceived(rxdata, rxferr);
		} // ISR(USARTn_RX_vect)


		// Interrupt service routines that are called when the actual byte was sent.
		ISR(USARTn_TX_vect) {
		  _DMXTransmitted();
		} // ISR(USARTn_TX_vect)


		// this interrupt occurs after data register was emptied by handing it over to the shift register.
		ISR(USARTn_UDRE_vect) {
		  _DMXTransmitted();
		} // ISR(USARTn_UDRE_vect)

	#endif //defined(DMXFORMAT) && defined(ARDUINO_ARCH_AVR)

#endif /* DYNAMIC_DMX_SERIAL_AVR_H */

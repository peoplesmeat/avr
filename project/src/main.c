/*
	AVR Software-Uart Demo-Application
	Version 0.3, 4/2007

	by Martin Thomas, Kaiserslautern, Germany
	<eversmith@heizung-thomas.de>
	http://www.siwawi.arubi.uni-kl.de/avr_projects
*/

/*
Test environment/settings:
- avr-gcc 4.1.1/avr-libc 1.4.5 (WinAVR 1/2007)
- Atmel ATtiny85 @ 1MHz internal R/C
- 2400bps

AVR Memory Usage (-Os)
----------------
Device: attiny85

Program:     874 bytes (10.7% Full)
(.text + .data + .bootloader)

Data:         52 bytes (10.2% Full)
(.data + .bss + .noinit)

*/

// #define WITH_STDIO_DEMO

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "softuart.h"
#include <delay.h>
#include "sensors.h"
#include "uart.h"

#define STATUS_LED 0
#define BAUD 9600

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

void ioinit()
{
    DDRB = 0b11101111; //PB4 = MISO
    DDRC = 0b11111111; //
    DDRD = 0b11111111; //PORTD (RX on PD0)
}

int main(void)
{
	char c;
	static const char pstring[] PROGMEM =
		"adapted for Atmel AVR and this demo by Martin Thomas\r\n";

	ioinit();
	uart_init((UART_BAUD_SELECT((BAUD),F_OSC)));
	softuart_init();
	softuart_turn_rx_on(); /* redundant - on by default */

	sei();

	uart_puts_P("Powered On...\r\n\n");
	softuart_puts_P( "\r\nSoftuart Demo-Application\r\n" );    // "implicit" PSTR
	delay_ms(1000);
	softuart_puts_p( PSTR("generic softuart driver code by Colin Gittins\r\n") ); // explicit PSTR
	delay_ms(1000);
	softuart_puts_p( pstring ); // pstring defined with PROGMEM
	delay_ms(1000);
	softuart_puts( "--\r\n" );  // string "from RAM"
	delay_ms(1000);

	//printf("Searching for Sensors\n");
	int nSensors = sensors_search();

	while (nSensors<2)
	{
		softuart_puts_p( "No enough DS18X20 Sensor(s) available... retrying:\r\n" );
		delay_ms(3000);
		nSensors = sensors_search();
	}

	char out[64];
	sprintf(out, "Found %d sensors", nSensors);
	softuart_puts(out);    // "implicit" PSTR
	delay_ms(5000);
	softuart_putchar(0xFE);
	softuart_putchar(0x1);
	for (;;) {

		//softuart_puts_P( " Measure " );
		sbi(PORTC, STATUS_LED);
		uint16_t decicelsius = make_sensor_measurement(0);
		cbi(PORTC, STATUS_LED);

		softuart_putchar(0xFE);
		softuart_putchar(0x80);

		sprintf(out, "%d.%d C %d.%d F",
				(int)(decicelsius/10) ,
				(decicelsius%10),
				(int)((decicelsius)*9)/50 + 32,
				(int)( ((decicelsius*9)/5) %10) );
		softuart_puts(out);
		uart_puts_P("Reading Sensor 1 ");
		uart_puts(out);
		uart_puts_P("\r\n");

		sbi(PORTC, STATUS_LED);
		decicelsius = make_sensor_measurement(1);
		cbi(PORTC, STATUS_LED);

		softuart_putchar(0xFE);
		softuart_putchar(0xC0);

		sprintf(out, "%d.%d C %d.%d F",
				(int)(decicelsius/10) ,
				(decicelsius%10),
				(int)((decicelsius)*9)/50 + 32,
				(int)( ((decicelsius*9)/5) %10) );
		softuart_puts(out);

		uart_puts_P("Reading Sensor 2 ");
		uart_puts(out);
		uart_puts_P("\r\n");

		delay_ms(2000);

	}

	return 0; /* never reached */
}

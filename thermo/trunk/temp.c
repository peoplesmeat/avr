

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "onewire.h"
#include "ds18x20.h"

#define BAUD 9600
//#define BAUD 19200
#define MYUBRR (F_OSC/16/BAUD-1)

#define FALSE 0
#define TRUE 1

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

static int uart_putchar(char c, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

int atod();

#define MAXSENSORS 5

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

#define STATUS_LED 0
#define TEMP_SENS 1
void ioinit(void)
{

	    //1 = output, 0 = input
    DDRB = 0b11101111; //PB4 = MISO 
    DDRC = 0b11111111; //
    DDRD = 0b11111111; //PORTD (RX on PD0)
		
    //USART Baud rate: 9600
    UBRR0H = MYUBRR >> 8;
    UBRR0L = MYUBRR;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);	
		
		stdout = &mystdout; //Required for printf init
		
	  OCR1A   = 15625; // Set CTC compare value to 1Hz at 16MHz AVR clock, with a prescaler of 1024 
	  TCCR1B |= ((1 << CS10) | (1 << CS12)); // Start timer at Fcpu/64 
	  TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode
    //TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt 
	  	 
	 sei();
}

uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	printf( "Scanning Bus for DS18X20\n" );
	
	nSensors = 0;
	
	for( diff = OW_SEARCH_FIRST; 
		diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; )
	{
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			printf( "No Sensor found\n" );
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			printf( "Bus Error\n" );
			break;
		}
		
		for (i=0;i<OW_ROMCODE_SIZE;i++)
			gSensorIDs[nSensors][i]=id[i];
		
		nSensors++;
	}
	
	return nSensors;
}

int lightOn = 0; 
int main (void)
{
   char ReceivedByte;
	 int nSensors;
	 
	 ioinit(); 
	 printf("Searching for sensors\n"); 
	 sbi(PORTC, STATUS_LED);
	 nSensors= search_sensors();
	 printf("Found %d Sensors\n", nSensors);
	 for (;;) // Loop forever
   {
		 
	 }
	 /*
	adc_init();
	delay_us(1 * 1000000);
	FindDevices();
	delay_us(1 * 1000000);
	FindDevices();
   while(1) 
	 {
		delay_us(1 * 1000000);
		  int k; 
			printf("d:%d",atod()); 
			if (lightOn)
				cbi(PORTC, STATUS_LED);
			else
				sbi(PORTC, STATUS_LED);
		lightOn = ~lightOn;
	 }

	 
	 sbi(PORTC, STATUS_LED);
	 delay_ms(1000); 
	 cbi(PORTC, STATUS_LED); 
	 delay_ms(100); 
	 sbi(PORTC, STATUS_LED);*/

	 
 /*  for (;;) // Loop forever
   {
      while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
      ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived"

      while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it
      UDR0 = ReceivedByte; // Echo back the received byte back to the computer

			if (ReceivedByte == '\r') 
			{
				while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it	
				UDR0 = '\n'; // Echo back the received byte back to the computer			
			}
				
	 }*/
	 
 
  for (;;) // Loop forever
   {
		 //cbi(PORTC, STATUS_LED);
		 //delay_ms(100); 
		 //sbi(PORTC, STATUS_LED); 
         // Do nothing - echoing is handled by the ISR instead of in the main loop
		 //delay_ms(5000); 
   }   

}

static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    
    return 0;
}



//ISR(_VECTOR(19))
ISR(USART_RX_vect) 
{

   char ReceivedByte;
   ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived"
   UDR0 = ReceivedByte; // Echo back the received byte back to the computer
	 //delay_ms(350); 
	 
}

ISR(TIMER1_COMPA_vect)
{ 
	/*
	 if (lightOn)
		 cbi(PORTC, STATUS_LED);
	 else
		 sbi(PORTC, STATUS_LED);
	 lightOn = ~lightOn;
	 printf("Timer");
	 */
}



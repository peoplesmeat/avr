#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <onewire.h>
#include <ds18x20.h>
#include <delay.h>

#define MAXSENSORS 2
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

/*uint8_t ** sensor_getIds() {
	return &gSensorIDs[0];
}*/


uint8_t sensors_search(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;

	//uart_puts_P( "\r\nScanning Bus for DS18X20\r\n" );

	nSensors = 0;

	for( diff = OW_SEARCH_FIRST;
		diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; )
	{
		DS18X20_find_sensor( &diff, &id[0] );

		if( diff == OW_PRESENCE_ERR ) {
		//	uart_puts_P( "No Sensor found\r\n" );
			break;
		}

		if( diff == OW_DATA_ERR ) {
		//	uart_puts_P( "Bus Error\r\n" );
			break;
		}

		for (i=0;i<OW_ROMCODE_SIZE;i++)
			gSensorIDs[nSensors][i]=id[i];

		nSensors++;
	}

	return nSensors;
}

uint16_t make_sensor_measurement(int sensor)
{
	uint8_t subzero, cel, cel_frac_bits;

	//uart_puts_P( "\r\nConvert_T for all Sensors and Read Sensor by Sensor\r\n" );
	if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL )
		== DS18X20_OK)
	{
		delay_ms(DS18B20_TCONV_12BIT);

		if ( DS18X20_read_meas( &gSensorIDs[sensor][0], &subzero,
							&cel, &cel_frac_bits) == DS18X20_OK )
		{
			//uart_put_temp(subzero, cel, cel_frac_bits);
			return DS18X20_temp_to_decicel(subzero, cel, cel_frac_bits);
		}
		else
		{
			return -5;
		}
	}
	else
	{
		return -2;
	//	uart_puts_P("Start meas. failed (short circuit?)");
	}
}

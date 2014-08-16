/*
 * sensors.h
 *
 *  Created on: Nov 28, 2008
 *      Author: davis
 */

#ifndef SENSORS_H_
#define SENSORS_H_



uint8_t sensors_search(void);
uint16_t make_sensor_measurement(int sensor);
uint8_t ** sensor_getIds() ;

#endif /* SENSORS_H_ */

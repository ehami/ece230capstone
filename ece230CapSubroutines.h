/*
 * ece230CapSubroutines.h
 *
 *  Created on: May 13, 2021
 *      Author: sextonom
 */

#ifndef ECE230CAPSUBROUTINES_H_
#define ECE230CAPSUBROUTINES_H_



#endif /* ECE230CAPSUBROUTINES_H_ */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>


void removeChar(char* string, int i);
void PlayAllNotesOnce(void);
void PlaySendNotification(void);
void PlayRecieveNotification(void);
void DisplayProjectName(char* L1, char* L2);

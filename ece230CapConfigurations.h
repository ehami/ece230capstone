/*
 * ece230Project5subroutines.h
 *
 *  Created on: Apr 25, 2021
 *      Author: song
 */

#ifndef ECE230PROJECT5SUBROUTINES_H_
#define ECE230PROJECT5SUBROUTINES_H_

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>

void ConfigureClocks(void);
void ConfigureA4ATD(void);
void ConfigurePort3Interrupts(void);
void ConfigureTimerA1InterruptOneSecond(void);
void ConfigureTimerA0PWM(void);
void ConfigureTimerA2Interrupt(void);


#define USB_EUSCI_MODULE 	EUSCI_A0_BASE
#define BT_EUSCI_MODULE 	EUSCI_A1_BASE

void configureUsbSerial();
void configureBluetoothSerial();


#endif /* ECE230PROJECT5SUBROUTINES_H_ */

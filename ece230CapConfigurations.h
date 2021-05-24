/*
 * ece230CapstoneConfigurations.h
 *
 *  Created on: May 10, 2021
 *      Author: song
 */

#ifndef ECE230CAPSTONECONFIGURATIONS_H_
#define ECE230CAPSTONECONFIGURATIONS_H_

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>

void ConfigureClocks(void);
void ConfigureTimerA0PWM(void);
void ConfigureTimerA1Interrupt(void);
void ConfigureTimerA2Interrupt(void);
void ConfigurePort3Interrupts(void);
void ConfigureA4ATD(void);
void configureUsbSerial();
void configureBluetoothSerial();
void configurePortTwo();

#endif /* ECE230CAPSTONECONFIGURATIONS_H_ */

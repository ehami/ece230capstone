/*
 * ece230Project5Subroutines.c
 *
 *  Created on: April 26, 2021
 *      Author: song
 */
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "ece230CapConfigurations.h"

#define beatPeriod      64000
#define Rest 0x1

//Use External 48MHz oscillator
//Set MCLK at 12 MHz for CPU
//Set SMCLK at 12 MHz for Timer A0
//Set ACLK at 128kHz for Timer A1
void ConfigureClocks(void) {
    /* Configuring pins for peripheral/crystal usage and LED for output */
        MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
                GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
        MAP_CS_setExternalClockSourceFrequency(32000,48000000);
          /* Starting HFXT in non-bypass mode without a timeout. Before we start
           * we have to change VCORE to 1 to support the 48MHz frequency */
          MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
          MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
          MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
          MAP_CS_startHFXT(false);
          /* Initializing MCLK to HFXT (effectively 48MHz) */
          MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
          /*Timer A0 uses SMCLK */
          MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);


          /* Timer A1 clock source set up at 128kHz. Timer overflow occurs at 0.5 Second*/
          MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
          MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

void ConfigureA4ATD() {
    /* Initializing ADC (MCLK/1/4) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4,
            0);

    /* Configuring GPIOs (6.0 A15) (6.1 A14) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN0,
    GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN1,
        GPIO_TERTIARY_MODULE_FUNCTION);


    /* Configuring ADC Memory */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
    //Choose A15 on P6.0 false on differential mode
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
           ADC_INPUT_A5, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS,
               ADC_INPUT_A4, false);

    /* Configuring Sample Timer */
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    /* Enabling/Toggling Conversion */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

}

#define TIMER_PERIOD    10000 // change this count to generate one second

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,              // ACLK=128KHz
        TIMER_A_CLOCKSOURCE_DIVIDER_2,          // prescaler
        TIMER_PERIOD,                           // TAxCCR1 value
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

void ConfigureTimerA1InterruptOneSecond(void){
    /* Configuring Timer_A0 for Up Mode */
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    /* Enabling interrupts and starting the timer */
//    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableInterrupt(INT_TA1_N);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

}

//set up Port 3 pins 5-7 to generate interrupt on falling edge
void ConfigurePort3Interrupts(void)
{
    // Pin7 - Enter character
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN7);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN7);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN7);
    // Pin6 - Delete Character
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN6);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN6);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN6);
    // Pin5 - Send Message
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN5);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN5);
    // Pin0 - Switch
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN0);
    // Enable Port3 Interrupt
    MAP_Interrupt_enableInterrupt(INT_PORT3);
}


////Map Pin P2.4 to Timer A CCP0 compare output
//const uint8_t port_mapping[] =
//{
//        PMAP_NONE, PMAP_NONE, PMAP_NONE, PMAP_NONE, PMAP_TA0CCR0A, PMAP_NONE,
//        PMAP_NONE, PMAP_NONE
//};

/* Timer_A 0 Up Configuration Parameter */
const Timer_A_UpModeConfig upConfigNote =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_3,          // SMCLK/3 = 4MHz
        Rest,                                   // 5000 tick period
        TIMER_A_TAIE_INTERRUPT_ENABLE,          // Enable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};


const Timer_A_UpModeConfig upConfigBeat =
{
        TIMER_A_CLOCKSOURCE_ACLK,               // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // ACLK/1 = 128kHz
        beatPeriod,                             // 5000 tick period
        TIMER_A_TAIE_INTERRUPT_ENABLE,          // Enable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};


// Timer_A 0 Compare Configuration Parameter
// to toggle CCR0 output pin at Timer A0 PERIOD interval
const Timer_A_CompareModeConfig compareModeConfig =
{
         TIMER_A_CAPTURECOMPARE_REGISTER_0,         // Use CCR0
         TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,  // Disable CCR interrupt
         TIMER_A_OUTPUTMODE_TOGGLE,                 // Toggle output
         Rest
};

//set up for Timer A2
//to generate interrupt at full count from 0 to 0xFFFF
const Timer_A_ContinuousModeConfig continuousModeConfig = {
         TIMER_A_CLOCKSOURCE_ACLK,              // ACLK Clock Source
         TIMER_A_CLOCKSOURCE_DIVIDER_2,        // ACLK/1 = 128kHz
         TIMER_A_TAIE_INTERRUPT_DISABLE,        // Enable Timer Interrupt
         TIMER_A_DO_CLEAR                       // Clear Value
};

//note waveform generation with Timer A0 up mode and CCR0 compare toggle
//map CCR0 output to P2.4 for speaker output
void ConfigureTimerA0PWM(void) {
    /* Remapping  TACCR0 to P2.4 */
//    MAP_PMAP_configurePorts((const uint8_t *) port_mapping, PMAP_P2MAP, 1,
//            PMAP_DISABLE_RECONFIGURATION);	// set in BT UART Config below
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
            GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Initialize compare registers to generate PWM1 */
   MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareModeConfig);

   /* Configuring Timer_A0 for Up Mode and starting */
   MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfigNote);
   MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

//interrupt interval is 2^16/128kHz=0.5 seconds
//this is duration of one note or beat
void ConfigureTimerA2Interrupt(void) {
    /* Configuring Timer_A1 for Continuous Mode */
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &upConfigBeat);
    /* Enabling interrupts and starting the timer */
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_disableInterrupt(INT_TA2_0);
    /* Start counter */
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    MAP_Timer_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                    beatPeriod);

}



const uint8_t portMapP2[] = {
        PMAP_NONE, 		// 0
		PMAP_NONE,
		PMAP_NONE,
		PMAP_UCA1RXD, 	// P2.3 (34): RXD
		PMAP_TA0CCR0A,		// 4: TODO: Set to timer for speaker
		PMAP_UCA1TXD, 	// P2.5 (19): TXD
		PMAP_NONE,
		PMAP_NONE
};


void configureUsbSerial() {
	/*
	 * 38400 8N1, SMCLK = 12 MHz
	 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	 *
	 */
	const eUSCI_UART_ConfigV1 usbUartConfig = {
	        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          		// SMCLK Clock Source
	        19,                                     			// Pre
	        8,                                       		// Mod 1
	        85,                                       		// Mod 2
	        EUSCI_A_UART_NO_PARITY,                  		// No Parity
	        EUSCI_A_UART_LSB_FIRST,                  		// LSB First
	        EUSCI_A_UART_ONE_STOP_BIT,               		// One stop bit
	        EUSCI_A_UART_MODE,                       		// UART mode
	        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling = 1
	        EUSCI_A_UART_8_BIT_LEN                  		// 8 bit data length
	};

	MAP_PMAP_configurePorts((const uint8_t*) portMapP2, PMAP_P2MAP, 1, PMAP_DISABLE_RECONFIGURATION);

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
    		GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3,
			GPIO_PRIMARY_MODULE_FUNCTION
	);


    MAP_UART_initModule(USB_EUSCI_MODULE, &usbUartConfig);
    MAP_UART_enableModule(USB_EUSCI_MODULE);


    MAP_UART_enableInterrupt(USB_EUSCI_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
}


void configureBluetoothSerial() {
	/*
	 * 38400 8N1, SMCLK = 12 MHz
	 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	 *
	 */
	const eUSCI_UART_ConfigV1 btUartConfig = {
	        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          		// SMCLK Clock Source
	        19,                                     			// Pre
	        8,                                       		// Mod 1
	        85,                                       		// Mod 2
	        EUSCI_A_UART_NO_PARITY,                  		// No Parity
	        EUSCI_A_UART_LSB_FIRST,                  		// LSB First
	        EUSCI_A_UART_ONE_STOP_BIT,               		// One stop bit
	        EUSCI_A_UART_MODE,                       		// UART mode
	        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling = 1
	        EUSCI_A_UART_8_BIT_LEN                  		// 8 bit data length
	};

	MAP_PMAP_configurePorts((const uint8_t*) portMapP2, PMAP_P2MAP, 1, PMAP_DISABLE_RECONFIGURATION);

	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
				GPIO_PORT_P2,
				GPIO_PIN3 | GPIO_PIN5,	// TODO: Config pin 4 for speaker
				GPIO_PRIMARY_MODULE_FUNCTION
	);


    MAP_UART_initModule(BT_EUSCI_MODULE, &btUartConfig);
    MAP_UART_enableModule(BT_EUSCI_MODULE);


    MAP_UART_enableInterrupt(BT_EUSCI_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
}

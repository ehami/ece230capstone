/*
 * ece230CapConfigurations.c
 *
 * Clock, Timer, and Interrupt configuration functions.
 *
 *  Created on: May 10, 2021
 *      Author: Olivia Sexton & Eric Hamilton
 */
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>

#define beatPeriod          20000               // Count for period length of a Note
#define Rest                0x1                 // Default Frequency for PWM Wave
#define TIMER_PERIOD        10000               // Analog Input Capture Interrupt Count
#define USB_EUSCI_MODULE    EUSCI_A0_BASE
#define BT_EUSCI_MODULE     EUSCI_A1_BASE


/*
 * Use External 48MHz oscillator
 * Set MCLK at 12 MHz for CPU
 * Set SMCLK at 12 MHz for Timer A0, USB-UART, BT-UART
 * Set ACLK at 128kHz for Timer A0, Timer A1, Timer A2
 */
void ConfigureClocks(void) {
    // Configuring pins for peripheral/crystal usage and LED for output
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_PJ,
            GPIO_PIN3 | GPIO_PIN2,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_CS_setExternalClockSourceFrequency(32000, 48000000);
    // Starting HFXT in non-bypass mode without a timeout. Before we start
    // we have to change VCORE to 1 to support the 48MHz frequency
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    MAP_CS_startHFXT(false);
    // Initializing MCLK to HFXT (effectively 48MHz)
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
    // Timer A0, USB-UART, BT-UART use SMCLK
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
    // Timer A0, Timer A1, Timer A2 use ACLK set up at 128kHz. Timer overflow occurs at 0.5 Second
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}


/* Timer_A 0 Up Mode Configuration Parameter */
const Timer_A_UpModeConfig upConfigNote = {
        TIMER_A_CLOCKSOURCE_SMCLK,                  // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_3,              // SMCLK/3 = 4MHz
        Rest,                                       // 5000 tick period
        TIMER_A_TAIE_INTERRUPT_ENABLE,              // Enable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,         // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                            // Clear value
        };

/*
 * Timer_A 0 Compare Configuration Parameter
 * to toggle CCR0 output pin at Timer A0 PERIOD interval
 */
const Timer_A_CompareModeConfig compareModeConfig = {
        TIMER_A_CAPTURECOMPARE_REGISTER_0,          // Use CCR0
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE,                  // Toggle output
        Rest
        };

/*
 * Note waveform generation with Timer A0 up mode and CCR0 compare toggle
 */
void ConfigureTimerA0PWM(void) {
    // Configure Pin 2.4 for PWM Output
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P2,
            GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);

    // Initialize compare registers to generate PWM1
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareModeConfig);

    // Configuring Timer_A0 for Up Mode and starting
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfigNote);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig = {
        TIMER_A_CLOCKSOURCE_ACLK,               // ACLK=128KHz
        TIMER_A_CLOCKSOURCE_DIVIDER_2,          // Prescaler
        TIMER_PERIOD,                           // TAxCCR1 value
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
        };

/*
 * Configure Timer A1
 *      - Up Mode
 *      - CCR1
 *      - 0.15s
 *      - INT_TA1_N
 */
void ConfigureTimerA1Interrupt(void) {
    // Configuring Timer_A1 for Up Mode
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    // Enabling interrupt and starting the timer
    MAP_Interrupt_enableInterrupt(INT_TA1_N);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}


/* Timer_A 2 Up Mode Configuration Parameter */
const Timer_A_UpModeConfig upConfigBeat = {
        TIMER_A_CLOCKSOURCE_ACLK,                   // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,              // ACLK/1 = 128kHz
        beatPeriod,                                 // 5000 tick period
        TIMER_A_TAIE_INTERRUPT_ENABLE,              // Enable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,         // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                            // Clear value
        };

/*
 * Configure timer to interrupt at the end of each beat
 */
void ConfigureTimerA2Interrupt(void)
{
    // Configuring Timer_A2 for Up Mode
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &upConfigBeat);
    // Disable Interrupt - To be enabled when notification sound is played
    MAP_Interrupt_disableInterrupt(INT_TA2_0);
    // Start counter
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
    // Set Compare Value
    MAP_Timer_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                beatPeriod);
}


/*
 * Configure Port 3 to generate interrupt on falling edge
 *      - Pin 7: Enter character on button press
 *      - Pin 6: Delete character on button press
 *      - Pin 5: Send Message on Button Press
 *      - Pin 0: Switch input for character input
 */
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

/*
 * Configure Analog input pins - ADC
 *      P5.0 - A5 - MEM1 - Capital Character Potentiometer Selection
 *      P5.1 - A4 - MEM0 - Special Character Potentiometer Selection
 */
void ConfigureA4ATD() {
    // Initializing ADC (MCLK/1/4)
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4,
                         0);

    // Configuring GPIOs (5.0 A15) (5.1 A14)
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN0,
    GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN1,
    GPIO_TERTIARY_MODULE_FUNCTION);

    // Configuring ADC Memory
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
    // Choose A5 on P5.0 false on differential mode
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                        ADC_INPUT_A5, false);
    // Choose A4 on P5.1 false on differential mode
    MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                        ADC_INPUT_A4, false);

    // Configuring Sample Timer
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    // Enabling/Toggling Conversion
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
}


/* Port mapping for RXD/TXD for Bluetooth, and Speaker PWM Wave */
const uint8_t portMapP2[] = {
        PMAP_NONE, 		// 0
        PMAP_NONE,
        PMAP_NONE,
        PMAP_UCA1RXD, 	// P2.3 (34): RXD
        PMAP_TA0CCR0A,  // 2.4 (38): Speaker PWM Wave
        PMAP_UCA1TXD, 	// P2.5 (19): TXD
        PMAP_NONE,
        PMAP_NONE
        };

/* Port 2 USB UART Configuration Parameter */
const eUSCI_UART_ConfigV1 usbUartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                 // SMCLK Clock Source
        19,                                             // Pre
        8,                                              // Mod 1
        85,                                             // Mod 2
        EUSCI_A_UART_NO_PARITY,                         // No Parity
        EUSCI_A_UART_LSB_FIRST,                         // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                      // One stop bit
        EUSCI_A_UART_MODE,                              // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling = 1
        EUSCI_A_UART_8_BIT_LEN                          // 8 bit data length
        };

/*
 * 38400 8N1, SMCLK = 12 MHz
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 *
 */
void configureUsbSerial()
{
    MAP_PMAP_configurePorts((const uint8_t*) portMapP2, PMAP_P2MAP, 1,
    PMAP_DISABLE_RECONFIGURATION);

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3,
            GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_UART_initModule(USB_EUSCI_MODULE, &usbUartConfig);
    MAP_UART_enableModule(USB_EUSCI_MODULE);

    MAP_UART_enableInterrupt(USB_EUSCI_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
}

/* Port 2 Bluetooth UART Configuration Parameter */
const eUSCI_UART_ConfigV1 btUartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                 // SMCLK Clock Source
        19,                                             // Pre
        8,                                              // Mod 1
        85,                                             // Mod 2
        EUSCI_A_UART_NO_PARITY,                         // No Parity
        EUSCI_A_UART_LSB_FIRST,                         // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                      // One stop bit
        EUSCI_A_UART_MODE,                              // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling = 1
        EUSCI_A_UART_8_BIT_LEN                          // 8 bit data length
        };


/*
 * 38400 8N1, SMCLK = 12 MHz
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 *
 */
void configureBluetoothSerial() {
    MAP_PMAP_configurePorts((const uint8_t*) portMapP2, PMAP_P2MAP, 1,
    PMAP_DISABLE_RECONFIGURATION);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P2,
            GPIO_PIN3 | GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_UART_initModule(BT_EUSCI_MODULE, &btUartConfig);
    MAP_UART_enableModule(BT_EUSCI_MODULE);

    MAP_UART_enableInterrupt(BT_EUSCI_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
}

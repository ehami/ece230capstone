//Project Name: ece230Project5ATDLCDtemplate
//Jianjian Song
//4-25-2021
//This is a template for intermediate implementation of Project #5
//students are free to revise this code
/*******************************************************************************
 * Project 5 Intermediate implementation
 *  Get analog voltage value from A15 on P6.0 every second with Timer Up CCR0 interrupt
 *  and display both digital and analog  values on LCD
 *
 *  ACLK = TACLK = 32768Hz, MCLK = SMCLK = DCO = 3MHz
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST     P6.6 (36) |-->Register Select RS
 *            |        P6.7 (35) |-->E Clock
 *            |                  |
 *            |          Port 4  |--> Data Lines
 *            |                  |
 *  P5.0 A5-->|           P1.0   |-->LED1
 *  P5.1 A4-->|                  |
 *
 ******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <lcd8bitsece230.h>
#include <ece230CapConfigurations.h>
#include <ece230CapSubroutines.h>
static uint16_t characterSelectionfromA5;
static uint8_t index = 0;
static char currentChar;
static char currentMessage[] = "                ";	// 16 + NT
static bool sendMessageFlag = false;
static bool receiveMessageFlag = false;


//#define VDD 26
#define RESOLUTION 14

#define VDD_CHAR 26
#define VDD_SPECIAL 33

#define DELAYCOUNT 1000000  //Two second delay at 12MHz MCLK

static bool hasBtRxMessageBeenShownYet = true;

static int usbIndex = 0;
static int btIndex = 0;

static char blankLine[] = "                ";
static char usbStr[] 	= "                ";	// 16x ' ' + 1x '\0'
static char btStr[] 	= "                ";	// 16x ' ' + 1x '\0'
static char line2buf[] 	= "                ";

void displayRxMessage(char* message) {

	int i = 0;
	for(i = 0; i < 16; i++) {
		line2buf[i] = message[i];
	}
	receiveMessageFlag = true;

	i = 0;
	while(message[i] != '\n' && message[i] != '\r' && message[i] != '\0') {
		MAP_UART_transmitData(USB_EUSCI_MODULE, message[i++]);
	}

	MAP_UART_transmitData(USB_EUSCI_MODULE, '\n');
}

void sendMessage(char* message) {
	// TODO: Input validation
	int i = 0;
	while(message[i] != '\n' && message[i] != '\r' && message[i] != '\0') {
		MAP_UART_transmitData(BT_EUSCI_MODULE, message[i++]);
	}

	MAP_UART_transmitData(BT_EUSCI_MODULE, '\n');

	PlaySendNotification();
	sendMessageFlag = true;
}


int main(void) {
    currentChar = ' ';
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    /* Enabling the FPU for floating point operation */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();

    /* Configuring P1.0 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    ConfigureClocks();

    ConfigureA4ATD();
    ConfigurePort3Interrupts();

    lcd8bits_init();

    DisplayProjectName("Bluetooth", "Messaging");


    configureUsbSerial();
    configureBluetoothSerial();

    ConfigureTimerA0PWM();

    PlayAllNotesOnce();

//    ConfigureTimerA2Interrupt();



    ConfigureTimerA1InterruptOneSecond();

    lcd_clear();

    MAP_Interrupt_enableMaster(); //not necessary

    int count;


    while(1)
    {
        if (sendMessageFlag) {
            lcd_SetLineNumber(FirstLine);
            lcd_puts("  Message Sent  ");
            for(count = 0; count < 400000; count++);
            count = 0;
            while (count < 16) {
                currentMessage[count] = ' ';
                count++;
            }
            sendMessageFlag = false;
            index = 0;
        }

        if (receiveMessageFlag) {
            lcd_SetLineNumber(SecondLine);
            lcd_puts("Message Received");
            for(count = 0; count < 400000; count++);
            receiveMessageFlag = false;
        }
    	// Update current message string
    	currentMessage[index] = characterSelectionfromA5;
    	//set LCD display on Line 1
    	lcd_SetLineNumber(FirstLine);
    	//display digital on Line 1 of LCD
    	lcd_puts(currentMessage);
    	for(count = 0; count < 50000; count++);

    	lcd_SetLineNumber(SecondLine);
    	lcd_puts(line2buf);
    	for(count = 0; count < 50000; count++);
    } //end while(1)
} //end main()

//Timer A1 interrupt handler to read analog voltage every second - Character selection
void TA1_N_IRQHandler(void) {
    // Save ADC14 values
    if (ROM_GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN0)) {
        characterSelectionfromA5 = MAP_ADC14_getResult(ADC_MEM1) * VDD_SPECIAL / (1<<RESOLUTION) + 32;
    } else {
        characterSelectionfromA5 = MAP_ADC14_getResult(ADC_MEM0) * VDD_CHAR / (1<<RESOLUTION) + 65;
    }
    // Toggle Conversion
    MAP_ADC14_toggleConversionTrigger();
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_1);
}

//set note half period count every 500ms
void TA2_0_IRQHandler(void) {
    // Set new note compare value
//    MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
//                                TwinkleTwinkleLittleStar[noteIndex]);
//    // increment song index
//    noteIndex = (noteIndex + 1) % NoteCNT;
//    // clear flag
//    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

//  external button press interrupt
void PORT3_IRQHandler(void) {
    int count;
    for(count=0;count<1000; count++);
    //get input interrupt flags of these enabled pins only
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);

    if (status & GPIO_PIN7) { // enter character
        if (index != 15) {
            index++;
        }
        currentChar = currentMessage[index];
    } else if (status & GPIO_PIN6) { // delete character
        if (index != 0) {
            removeChar(currentMessage, index);
            index--;
        }
    } else if (status & GPIO_PIN5) { // send message
    	sendMessage(currentMessage);
    }
}



/* USB UART ISR */
void EUSCIA0_IRQHandler() {
    uint32_t status = MAP_UART_getEnabledInterruptStatus(USB_EUSCI_MODULE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) {
    	char usbChar = (char) MAP_UART_receiveData(USB_EUSCI_MODULE);

    	if (usbChar == '\r') {
    	    // do nothing
    	} if (usbChar == '\n') {
    		usbIndex = 0;
    		sendMessage(usbStr);
    	} else {
    		if (usbIndex > 15) {
    			usbIndex = 1;
    			usbStr[0] = '~';
    		}

    		usbStr[usbIndex++] = usbChar;
    	}
    }
}


/* Bluetooth UART ISR */
void EUSCIA1_IRQHandler() {
    uint32_t status = MAP_UART_getEnabledInterruptStatus(BT_EUSCI_MODULE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) {
    	char btChar = (char) MAP_UART_receiveData(BT_EUSCI_MODULE);

    	if (btChar == '\r') {
    		// do nothing
    	} if (btChar == '\n') {
    		btIndex = 0;
    		displayRxMessage(btStr);
    		PlayReceiveNotification();
    		int i;
    		for(i = 0; i < 16; i++) {
    			btStr[i] = ' ';
    		}

    	} else {
    		if (btIndex > 15) {
    			btStr[0] = '~';
    			btIndex = 1;
    		}

    		btStr[btIndex++] = btChar;
    	}

    }
}

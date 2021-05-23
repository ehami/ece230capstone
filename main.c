//Project Name: ece230Capstone
//Jianjian Song
//4-25-2021
//This is a template for intermediate implementation of Project #5
//students are free to revise this code
/*
 *  ece230Capstone: Bluetooth messaging
 *
 *  LCD 8-bit display on Port 4
 *  Potentiometer analog input on Pins 5.0 and 5.1
 *  Digital input on Pins 3.0, 3.5, 3.6, 3.7
 *  Speaker PWM Wave Output on Pin 2.4
 *  Bluetooth Communication RXD and TXD on pins 2.3 and 2.5
 *
 *  Program is used to communicate between 2 paired Bluetooth chips.
 *  Main User interaction is done through the LCD.
 *  Line 1 is for the use to compose a message to send to the receiver.
 *  Line 2 is for the received message.
 *  All messages are 16 characters.
 *  Potentiometers are used to change characters.
 *  Buttons are used to enter/delete characters, and send message.
 *  Switch used to change type of character input.
 *  Speaker plays notification after a message is sent or received.
 *  USB connection to PC is used for terminal communication.
 *
 *  ACLK = TACLK = 128kHz, MCLK = SMCLK = DCO = 12MHz
 *
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

static uint16_t characterSelectionfromA5;
static uint8_t index = 0;
static int usbIndex = 0;
static int btIndex = 0;
static char currentMessage[] = "                ";	// 16 + NT
static char usbStr[] = "                ";  // 16x ' ' + 1x '\0'
static char btStr[] = "                ";   // 16x ' ' + 1x '\0'
static char line2buf[] = "                ";
static bool sendMessageFlag = false;
static bool receiveMessageFlag = false;
static bool notificationFinished = false;
static char noteIndex = 0;

#define USB_EUSCI_MODULE    EUSCI_A0_BASE
#define BT_EUSCI_MODULE     EUSCI_A1_BASE

//12MHz system clock and 3:1 prescaler
//4MHz Timer A clock
//Half period count = (12,000,000/Note frequency)/2
#define SystemClock     12000000                   //Hz
#define Prescaler       3                       //Timer A prescaler
#define TimerAClock     SystemClock/Prescaler

#define beatPeriod      64000
#define RESOLUTION      14         // 14 Bit Resolution
#define VDD_CHAR        26         // 26 Characters
#define VDD_SPECIAL     33         // 33 Special Characters
#define DELAYCOUNT      1000000    //Two second delay at 12MHz MCLK

//rest note, one shot signal of 1/12MHz pulse width is generated,
//which is not audible
#define Rest            0x1

#define FrequencyG3     196 //Hz
#define FrequencyA3     220 //Hz
#define FrequencyB3     247 //Hz
#define FrequencyC4     262 //Hz
#define FrequencyD4     294 //Hz
#define FrequencyE4     330 //Hz
#define FrequencyF4     349 //Hz

#define NOTEG3  TimerAClock/FrequencyG3/2
#define NOTEA3  TimerAClock/FrequencyA3/2
#define NOTEB3  TimerAClock/FrequencyB3/2
#define NOTEC4  TimerAClock/FrequencyC4/2
#define NOTED4  TimerAClock/FrequencyD4/2
#define NOTEE4  TimerAClock/FrequencyE4/2
#define NOTEF4  TimerAClock/FrequencyF4/2

const uint16_t NotesSequence[] = { NOTEG3, NOTEA3, NOTEB3, NOTEC4, NOTED4, NOTEE4, NOTEF4, Rest };
const uint16_t SendSequence[] = { Rest, NOTEB3, Rest };
const uint16_t RecieveSequence[] = { Rest, NOTEB3, Rest, NOTEE4, Rest };

///Play the note list once to test the notes sound correct
void PlayAllNotesOnce(void) {
   // char noteIndex = 0;
    unsigned int delay;
    while (noteIndex < 8)
    {
        MAP_Timer_A_setCompareValue(TIMER_A0_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                    NotesSequence[noteIndex]);
        noteIndex = noteIndex + 1;
        for (delay = 200000; delay != 0; delay--)
            ;
    } //end while()
} //end PlayAllNotesOnce(void)

/*
 * @param char* string
 * @param int   i
 *
 * Remove character from string at index i
 */
void removeChar(char* string, int i)
{
    while (i < 15)
    {
        string[i] = string[i + 1];
        i++;
    }
    string[15] = ' ';
}

/*
 * Write 'Bluetooth Messaging' to LCD Display
 */
void DisplayProjectName(void)
{
    int count;
    for (count = 0; count < 500; count++)
        ;
    lcd_SetLineNumber(FirstLine);
    lcd_puts("Bluetooth");
    lcd_SetLineNumber(SecondLine);
    lcd_puts("Messaging");
}


void displayRxMessage(char *message) {

    int i = 0;
    for (i = 0; i < 16; i++)
    {
        line2buf[i] = message[i];
    }
    receiveMessageFlag = true;
    MAP_Interrupt_enableInterrupt(INT_TA2_0);

    i = 0;
    while (message[i] != '\n' && message[i] != '\r' && message[i] != '\0')
    {
        MAP_UART_transmitData(USB_EUSCI_MODULE, message[i++]);
    }

    MAP_UART_transmitData(USB_EUSCI_MODULE, '\n');
}

void sendMessage(char *message) {
    // TODO: Input validation
    int i = 0;
    while (message[i] != '\n' && message[i] != '\r' && message[i] != '\0')
    {
        MAP_UART_transmitData(BT_EUSCI_MODULE, message[i++]);
    }

    MAP_UART_transmitData(BT_EUSCI_MODULE, '\n');

    sendMessageFlag = true;
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
}

int main(void)
{
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

    DisplayProjectName();

    configureUsbSerial();
    configureBluetoothSerial();

    ConfigureTimerA0PWM();

    PlayAllNotesOnce();

    ConfigureTimerA1Interrupt();

    ConfigureTimerA2Interrupt();

    lcd_clear();

    MAP_Interrupt_enableMaster(); //not necessary

    int count;

    while (1)
    {
        if (sendMessageFlag) { // Write Message Sent to LCD
            lcd_SetLineNumber(FirstLine);
            lcd_puts("  Message Sent  ");
            for (count = 0; count < 400000; count++)
                ;
            // Wait for Notification Sound to Finish
            while(!notificationFinished);
            notificationFinished = false;
            // Clear First Line of LCD
            count = 0;
            while (count < 16)
            {
                currentMessage[count] = ' ';
                count++;
            }
            sendMessageFlag = false;
            // Reset index
            index = 0;
        }

        if (receiveMessageFlag) { // Write Message Received to LCD
            lcd_SetLineNumber(SecondLine);
            lcd_puts("Message Received");
            for (count = 0; count < 400000; count++)
                ;
            receiveMessageFlag = false;
            // Wait for Notification Sound to finish
            while(!notificationFinished);
            notificationFinished = false;
        }
        // Update current message string
        currentMessage[index] = characterSelectionfromA5;
        // Display currentMessage on Line 1 of LCD
        lcd_SetLineNumber(FirstLine);
        lcd_puts(currentMessage);
        for (count = 0; count < 50000; count++);
        // Display recievedMessage on Line 2 of LCD
        lcd_SetLineNumber(SecondLine);
        lcd_puts(line2buf);
        for (count = 0; count < 50000; count++);
    } //end while(1)
} //end main()

/*
 *
 */
//Timer A1 interrupt handler to read analog voltage every second - Character selection
void TA1_N_IRQHandler(void) {
    // Save ADC14 values
    if (ROM_GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN0)) {
        characterSelectionfromA5 = MAP_ADC14_getResult(ADC_MEM1) * VDD_SPECIAL
                / (1 << RESOLUTION) + 32;
    } else {
        characterSelectionfromA5 = MAP_ADC14_getResult(ADC_MEM0) * VDD_CHAR
                / (1 << RESOLUTION) + 65;
    }
    // Toggle Conversion
    MAP_ADC14_toggleConversionTrigger();
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
    TIMER_A_CAPTURECOMPARE_REGISTER_1);
}

//set note half period count every 500ms
void TA2_0_IRQHandler(void)
{
    // Set new note compare value
    if (sendMessageFlag && (noteIndex < 3)) { // Play Send Notification
        MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                    SendSequence[noteIndex]);
        noteIndex++;
    } else if (receiveMessageFlag && (noteIndex < 5)) { // Play Recieve Notification
        MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                    RecieveSequence[noteIndex]);
        noteIndex++;
    } else { // Notification Finished: reset index and disable interrupt
        MAP_Interrupt_disableInterrupt(INT_TA2_0);
        noteIndex = 0;
        notificationFinished = true;
    }
    // Clear Interrupt Flag
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

/*
 * Event Based Interrupt: Button Press
 *      - Pin 7: Enter Character
 *      - Pin 6: Delete Character
 *      - Pin 5: Send Message
 */
void PORT3_IRQHandler(void)
{
    int count;
    for (count = 0; count < 1000; count++);
    //get input interrupt flags of these enabled pins only
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);

    if (status & GPIO_PIN7) { // enter character
        if (index != 15) {
            index++;
        }
    } else if (status & GPIO_PIN6) { // delete character
        if (index != 0) {
            removeChar(currentMessage, index);
            index--;
        }
    }
    else if (status & GPIO_PIN5) { // send message
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
        }

        if (usbChar == '\n') {
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
void EUSCIA1_IRQHandler()
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(BT_EUSCI_MODULE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) {
        char btChar = (char) MAP_UART_receiveData(BT_EUSCI_MODULE);

        if (btChar == '\r') {
            // do nothing
        }

        if (btChar == '\n') {
            btIndex = 0;
            displayRxMessage(btStr);
            int i;
            for (i = 0; i < 16; i++) {
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

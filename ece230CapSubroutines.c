/*
 * ece230CapSubroutines.c
 *
 *  Created on: May 13, 2021
 *      Author: sextonom
 */
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <lcd8bitsece230.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

//12MHz system clock and 3:1 prescaler
//4MHz Timer A clock
//Half period count = (12,000,000/Note frequency)/2
#define SystemClock     12000000                   //Hz
#define Prescaler       3                       //Timer A prescaler
#define TimerAClock     SystemClock/Prescaler

#define beatPeriod      64000

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


//rest note, one shot signal of 1/12MHz pulse width is generated,
//which is not audible
#define Rest 0x1
#define NoteCNT 48

const uint16_t NotesSequence[] = {NOTEG3, NOTEA3, NOTEB3, NOTEC4, NOTED4, NOTEE4, NOTEF4, Rest, NULL};
const uint16_t SendSequence[] = {NOTEB3, Rest};
const uint16_t RecieveSequence[] = {NOTEB3, Rest, NOTEE4, Rest};

const uint16_t TwinkleTwinkleLittleStar[] = {NOTEG3, NOTEG3, NOTED4, NOTED4, NOTEE4, NOTEE4, NOTED4, Rest,
                                             NOTEC4, NOTEC4, NOTEB3, NOTEB3, NOTEA3, NOTEA3, NOTEG3, Rest,
                                             NOTED4, NOTED4, NOTEC4, NOTEC4, NOTEB3, NOTEB3, NOTEA3, Rest,
                                             NOTED4, NOTED4, NOTEC4, NOTEC4, NOTEB3, NOTEB3, NOTEA3, Rest,
                                             NOTEG3, NOTEG3, NOTED4, NOTED4, NOTEE4, NOTEE4, NOTED4, Rest,
                                             NOTEC4, NOTEC4, NOTEB3, NOTEB3, NOTEA3, NOTEA3, NOTEG3, Rest
                                            };


// remove character from string
void removeChar(char* string, int i) {
    while (i < 15) {
        string[i] = string[i+1];
        i++;
    }
    string[15] = ' ';
}

void DisplayProjectName(char* L1, char* L2) {
    int count;
    for(count=0;count<500; count++);
    lcd_SetLineNumber(FirstLine);
    lcd_puts(L1);
    lcd_SetLineNumber(SecondLine);
    lcd_puts(L2);
}

///Play the note list once to test the notes sound correct
void PlayAllNotesOnce(void) {
    char noteIndex = 0;
    unsigned int delay;
    while(NotesSequence[noteIndex]!=NULL) {
        MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                NotesSequence[noteIndex]);
        noteIndex=noteIndex+1;
        for(delay = 200000; delay != 0; delay--) {}
    } //end while()
} //end PlayAllNotesOnce(void)

void PlaySendNotification(void) {
    char noteIndex = 0;
    unsigned int delay;
    while (noteIndex < 2) {
        MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                        SendSequence[noteIndex]);
        noteIndex++;
        for(delay = 200000; delay != 0; delay--) {}
    }
}

void PlayRecieveNotification(void) {
    char noteIndex = 0;
    unsigned int delay;
    while (noteIndex < 4) {
        MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                    RecieveSequence[noteIndex]);
        noteIndex++;
        for(delay = 200000; delay != 0; delay--) {}
    }
}

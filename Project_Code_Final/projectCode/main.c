/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 *
 * main.c
 *
 * Out of Box Demo for the MSP-EXP430FR4133
 * Main loop, initialization, and interrupt service routines
 *
 * September 2014
 * E. Chen
 *
 ******************************************************************************/

#include "main.h"
#include "hal_LCD.h"
#include "stdio.h"

// Backup Memory variables to track states through LPM3.5
volatile unsigned int s1buttonDebounce = 0;
volatile unsigned int s2buttonDebounce = 0;
volatile unsigned char holdCount[6] = "000000";
volatile unsigned int hold = 0;
volatile unsigned int actualMode = 0;
volatile unsigned int mode0work = 0;
volatile unsigned int mode1work = 0;
volatile unsigned int Count = 0;

// i messed up yall, ss[0] is least significant so u have to write 50 as 05, same with the others
volatile unsigned char ss[2] = "00";
volatile unsigned char min[2] = "00";
volatile unsigned char hr[2] = "21";
volatile unsigned char AmPm[2] = "MA";
volatile unsigned int AmPmINT = 0; //0 for AM, 1 for PM
volatile unsigned int alarmAmPmINT = 0; //0 for AM, 1 for PM
volatile unsigned char alarmss[2] = "00";
volatile unsigned char alarmmin[2] = "00";
volatile unsigned char alarmhr[2] = "00";
volatile unsigned char alarmAmPm[2] = "00";
volatile unsigned int timeToPill = 0;
volatile unsigned int userTurnedAlarmOff = 0;
volatile unsigned int beep = 0;
volatile unsigned int butt1 = 0; //flag for button 1 being pressed
volatile unsigned int butt2 = 0; //flag for button 1 being pressed
volatile unsigned int enteredAlarm = 0; //checks to see if the user has entered the menu to set the alarm

// TimerA0 UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 2MHz
        30000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};

void editAlarm(int goBackHome)
{
    enteredAlarm++;
    int confirm = 0;
    butt1 = 0;
    butt2 = 0;
    displayScrollText("SETTING ALARM MODE");
    showChar('P', pos6); //least significant digit, pos6 is rightmost
    showChar('A', pos5);
    showChar(' ', pos4);
    showChar(' ', pos3);
    showChar('R', pos2);
    showChar('H', pos1);
    __delay_cycles(2000000);
    while (confirm == 0)
    {
        if (butt1)
        {
            alarmhr[0]++;
            if (alarmhr[0] > '9')
            {
                alarmhr[1]++;
                alarmhr[0] = '0';
            }
            if (alarmhr[1]-'0' == 1 && alarmhr[0]-'0' == 2) //from 11:59 to 12:00
            {
                alarmAmPmINT = (alarmAmPmINT == 1) ? 0 : 1; //flips the half of day
                if (alarmAmPmINT) //if 1 then it's PM
                {
                    alarmAmPm[1] = 'P';
                }
                else //if 0 then it's AM
                {
                    alarmAmPm[1] = 'A';
                }
            }
            else if (alarmhr[1]-'0' == 1 && alarmhr[0]-'0' > 2) // if 12:59 to 13:00 -> 01:00
            {
                alarmhr[0] = '1';
                alarmhr[1] = '0';
            }
            butt1 = 0;
        }
        else if (butt2)
        {
            butt2 = 0;
            confirm++;
        }
        showChar(alarmAmPm[0], pos6); //least significant digit, pos6 is rightmost
        showChar(alarmAmPm[1], pos5);
        showChar(alarmmin[0], pos4);
        showChar(alarmmin[1], pos3);
        showChar(alarmhr[0], pos2);
        showChar(alarmhr[1], pos1);
    }

    showChar(' ', pos6); //least significant digit, pos6 is rightmost
    showChar(' ', pos5);
    showChar(' ', pos4);
    showChar('N', pos3);
    showChar('I', pos2);
    showChar('M', pos1);
    __delay_cycles(2000000);
    while (confirm == 1)
    {
        if (butt1)
        {
            alarmmin[0]++;
            if (alarmmin[0] > '9')
            {
                alarmmin[0] = '0';
                alarmmin[1]++;
            }
            if (alarmmin[1] >= '6')
            {
                alarmmin[1] = '0';
            }
            butt1 = 0;
        }
        else if (butt2)
        {
            butt2 = 0;
            confirm++;
        }
        showChar(alarmAmPm[0], pos6); //least significant digit, pos6 is rightmost
        showChar(alarmAmPm[1], pos5);
        showChar(alarmmin[0], pos4);
        showChar(alarmmin[1], pos3);
        showChar(alarmhr[0], pos2);
        showChar(alarmhr[1], pos1);
    }
    displayScrollText("ALARM SET");
    if (goBackHome)
        actualMode = 0;
    alarmss[0] = alarmss[1] = '0';
}

void editClock(void)
{
    int confirm = 0;
    butt1 = 0;
    butt2 = 0;
    displayScrollText("SETTING CLOCK MODE");
    showChar('P', pos6); //least significant digit, pos6 is rightmost
    showChar('A', pos5);
    showChar(' ', pos4);
    showChar(' ', pos3);
    showChar('R', pos2);
    showChar('H', pos1);
    __delay_cycles(2000000);
    while (confirm == 0) //we need 2 confirms before breaking out of the edit clock function, first one edits hour and AMPM, second one edits minute
    {
        if (butt1) //
        {
            hr[0]++;
            if (hr[0] > '9')
            {
                hr[1]++;
                hr[0] = '0';
            }
            if (hr[1]-'0' == 1 && hr[0]-'0' == 2) //from 11:59 to 12:00
            {
                AmPmINT = (AmPmINT == 1) ? 0 : 1; //flips the half of day
                if (AmPmINT) //if 1 then it's PM
                {
                    AmPm[1] = 'P';
                }
                else //if 0 then it's AM
                {
                    AmPm[1] = 'A';
                }
            }
            else if (hr[1]-'0' == 1 && hr[0]-'0' > 2) // if 12:59 to 13:00 -> 01:00
            {
                hr[0] = '1';
                hr[1] = '0';
            }
            butt1 = 0;
        }
        else if (butt2)
        {
            butt2 = 0;
            confirm++;
        }
        showChar(AmPm[0], pos6); //least significant digit, pos6 is rightmost
        showChar(AmPm[1], pos5);
        showChar(min[0], pos4);
        showChar(min[1], pos3);
        showChar(hr[0], pos2);
        showChar(hr[1], pos1);
    }
    ss[0] = ss[1] = '0';
    showChar(' ', pos6); //least significant digit, pos6 is rightmost
    showChar(' ', pos5);
    showChar(' ', pos4);
    showChar('N', pos3);
    showChar('I', pos2);
    showChar('M', pos1);
    __delay_cycles(2000000);
    while (confirm == 1)
    {
        if (butt1)
        {
            min[0]++;
            if (min[0] > '9')
            {
                min[0] = '0';
                min[1]++;
            }
            if (min[1] >= '6')
            {
                min[1] = '0';
            }
            butt1 = 0;
        }
        else if (butt2)
        {
            butt2 = 0;
            confirm++;
        }
        showChar(AmPm[0], pos6); //least significant digit, pos6 is rightmost
        showChar(AmPm[1], pos5);
        showChar(min[0], pos4);
        showChar(min[1], pos3);
        showChar(hr[0], pos2);
        showChar(hr[1], pos1);
    }
    displayScrollText("CLOCK SET");
    ss[0] = ss[1] = '0';
}

int areEqual(void)
{
    int areEqual = 1; //innocent till proven guilty
    if (AmPm[0] != alarmAmPm[0])
        areEqual = 0;
    if (AmPm[1] != alarmAmPm[1])
        areEqual = 0;
    if (ss[0] != alarmss[0])
        areEqual = 0;
    if (ss[1] != alarmss[1])
        areEqual = 0;
    if (min[0] != alarmmin[0])
        areEqual = 0;
    if (min[1] != alarmmin[1])
        areEqual = 0;
    if (hr[0] != alarmhr[0])
        areEqual = 0;
    if (hr[1] != alarmhr[1])
        areEqual = 0;
    return areEqual;
}

/*
 * main.c
 */
int main(void) {
    // Stop Watchdog timer
    WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);     // Stop WDT

        // Initializations
        Init_GPIO();
        Init_Clock();
        Init_LCD();
        Init_RTC();
        RTC_start(RTC_BASE, RTC_CLOCKSOURCE_XT1CLK);

        GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);

        __enable_interrupt();

        while(1)
        {
            while(actualMode == 0)
            {
                int holdCountNum;
                char newCount[6] = {holdCount[5], holdCount[4], holdCount[3], holdCount[2], holdCount[1], holdCount[0]};
                sscanf(newCount, "%d", &holdCountNum); //for the beep handler
                showChar(holdCount[0], pos6); //least significant digit, pos6 is rightmost
                showChar(holdCount[1], pos5);
                showChar(holdCount[2], pos4);
                showChar(holdCount[3], pos3);
                showChar(holdCount[4], pos2);
                showChar(holdCount[5], pos1);

                if (holdCountNum < 5 || areEqual() == 1){ //two checks, one for if the pill count is too low, and second is when the alarm goes off
                    beep = 1;
                    P4OUT |= BIT0;    // Turn LED2 On
                    if (enteredAlarm > 0 && areEqual() == 1)
                    {
                        timeToPill = 1; //alarm turns on
                        P4OUT |= BIT0;    // Turn LED2 On
                    }
                }
                else if (holdCountNum >= 5 && timeToPill == 0){ //here, both the pill count must be sufficient, and the alarm must be turned off
                    beep = 0;
                    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
                }
            }
            while(actualMode == 1)
            {
                //show clock
                P4OUT &= ~BIT0;
                showChar(AmPm[0], pos6); //least significant digit, pos6 is rightmost
                showChar(AmPm[1], pos5);
                showChar(min[0], pos4);
                showChar(min[1], pos3);
                showChar(hr[0], pos2);
                showChar(hr[1], pos1);
                if (butt1) //edit clock
                    editClock();
                else if (butt2) //edit alarm
                {
                    alarmAmPmINT = AmPmINT;
                    alarmAmPm[0] = AmPm[0];
                    alarmAmPm[1] = AmPm[1];
                    alarmmin[0] = min[0];
                    alarmmin[1] = min[1];
                    alarmhr[0] = hr[0];
                    alarmhr[1] = hr[1];
                    editAlarm(1); //0 for not returning to pill counting screen, 1 to go back to it
                }
            }
        }
}

/*
 * GPIO Initialization
 */
void Init_GPIO()
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    // Configure button S1 (P1.2) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // Configure button S2 (P2.6) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

    // Set P4.1 and P4.2 as Secondary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_P4,
           GPIO_PIN1 + GPIO_PIN2,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

/*
 * Clock System Initialization
 */
void Init_Clock()
{
    // Intializes the XT1 crystal oscillator
    CS_turnOnXT1(CS_XT1_DRIVE_1);
}

/*
 * Real Time Clock Counter Initialization
 */
void Init_RTC()
{
    // Set RTC modulo to 327-1 to trigger interrupt every ~10 ms
    RTC_setModulo(RTC_BASE, 326);
    RTC_enableInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT);
}

/*
 * PORT1 Interrupt Service Routine
 * Handles S1 button press interrupt
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case P1IV_NONE : break;
        case P1IV_P1IFG0 : break;
        case P1IV_P1IFG1 : break;
        case P1IV_P1IFG2 :
            if ((s1buttonDebounce) == 0 && (P2IN & BIT6))
            {
                s1buttonDebounce = 1;                        // First high to low transition
                P1OUT |= BIT0;    // Turn LED1 On
                hold = 0;
                if (actualMode == 0) //if regular pill Counting is active
                {
                    holdCount[0]++;
                    if (holdCount[0] == 58)
                    {
                        holdCount[0] = '0';
                        holdCount[1]++;
                    }
                    if (holdCount[1] == 58)
                    {
                        holdCount[0] = '0';
                        holdCount[1] = '0';
                        holdCount[2]++;
                    }
                    if (holdCount[2] == 58)
                    {
                        holdCount[0] = '0';
                        holdCount[1] = '0';
                        holdCount[2] = '0';
                        holdCount[3]++;
                    }
                    if (holdCount[3] == 58)
                    {
                        holdCount[0] = '0';
                        holdCount[1] = '0';
                        holdCount[2] = '0';
                        holdCount[3] = '0';
                        holdCount[4]++;
                    }
                    if (holdCount[4] == 58)
                    {
                        holdCount[0] = '0';
                        holdCount[1] = '0';
                        holdCount[2] = '0';
                        holdCount[3] = '0';
                        holdCount[4] = '0';
                        holdCount[5]++;
                    }
                    if (holdCount[5] == 58)
                    {
                        holdCount[0] = '0';
                        holdCount[1] = '0';
                        holdCount[2] = '0';
                        holdCount[3] = '0';
                        holdCount[4] = '0';
                        holdCount[5] = '0';
                        holdCount[0]++;
                    }
                }
                if (actualMode == 1) //if menu is active
                {
                    if (!(P1IN & BIT2) && !(P2IN & BIT6))
                    {
                        actualMode = 0;
                        mode0work = 1;
                        mode1work = 0;
                        s1buttonDebounce = 0;                                   // Clear button debounce
                        s2buttonDebounce = 0;                                   // Clear button debounce
                    }
                    else
                        butt1 = (butt1 == 1) ? 0 : 1;
                }

                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }
        case P1IV_P1IFG3 : break;
        case P1IV_P1IFG4 : break;
        case P1IV_P1IFG5 : break;
        case P1IV_P1IFG6 : break;
        case P1IV_P1IFG7 : break;
    }
}

/*
 * PORT2 Interrupt Service Routine
 * Handles S2 button press interrupt
 */
#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    switch(__even_in_range(P2IV, P2IV_P2IFG7))
    {
        case P2IV_NONE : break;
        case P2IV_P2IFG0 : break;
        case P2IV_P2IFG1 : break;
        case P2IV_P2IFG2 : break;
        case P2IV_P2IFG3 : break;
        case P2IV_P2IFG4 : break;
        case P2IV_P2IFG5 : break;
        case P2IV_P2IFG6 :
//            if ((s2buttonDebounce) == 0 && !(P1IN & BIT2)) //if button 1 has been pressed and now button 2 is being pressed
//            {
//                beep = 0;
//                timeToPill = 0;
//            }
            if ((s2buttonDebounce) == 0 && (P1IN & BIT2)) //if only button 2 is pressed (button 1 is released)
            {
                s2buttonDebounce = 1;                        // First high to low transition
                P4OUT |= BIT0;    // Turn LED2 On
                hold = 0;
                if (actualMode == 0) //regular pill Counting mode
                {
                    if (timeToPill == 1)
                        timeToPill = 0;
                    if (holdCount[0] == 48)
                    {
                        if (holdCount[1] == 48)
                        {
                            if (holdCount[2] == 48)
                            {
                                if (holdCount[3] == 48)
                                {
                                    if (holdCount[4] == 48)
                                    {
                                        if (holdCount[5] == 48)
                                        {
                                            displayScrollText("NO PILLS");
                                        }
                                        else
                                        {
                                            holdCount[5]--;
                                            holdCount[4] = '9';
                                            holdCount[3] = '9';
                                            holdCount[2] = '9';
                                            holdCount[1] = '9';
                                            holdCount[0] = '9';
                                        }
                                    }
                                    else
                                    {
                                        holdCount[4]--;
                                        holdCount[3] = '9';
                                        holdCount[2] = '9';
                                        holdCount[1] = '9';
                                        holdCount[0] = '9';
                                    }
                                }
                                else
                                {
                                    holdCount[3]--;
                                    holdCount[2] = '9';
                                    holdCount[1] = '9';
                                    holdCount[0] = '9';
                                }
                            }
                            else
                            {
                                holdCount[2]--;
                                holdCount[1] = '9';
                                holdCount[0] = '9';
                            }
                        }
                        else
                        {
                            holdCount[1]--;
                            holdCount[0] = '9';
                        }
                    }
                    else
                    {
                        holdCount[0]--;
                    }
                }
                if (actualMode == 1) //if menu is active
                {
                    if (!(P1IN & BIT2) && !(P2IN & BIT6))
                    {
                        actualMode = 0;
                        mode0work = 1;
                        mode1work = 0;
                        s1buttonDebounce = 0;                                   // Clear button debounce
                        s2buttonDebounce = 0;                                   // Clear button debounce
                    }
                    else
                        butt2 = (butt2 == 1) ? 0 : 1;
                }
                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }

        case P2IV_P2IFG7 : break;
    }
}

/*
 * Timer A0 Interrupt Service Routine
 * Used as button debounce timer
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
    // Both button S1 & S2 held down
    if (!(P1IN & BIT2) && !(P2IN & BIT6))
    {
        hold++;
        if (hold == 40)
        {
            // Stop Timer A0
            Timer_A_stop(TIMER_A0_BASE);

            // Change mode
            if (actualMode == 0)
            {
                holdCount[0]++;
                if (holdCount[0] == 58)
                {
                    holdCount[0] = '0';
                    holdCount[1]++;
                }
                if (holdCount[1] == 58)
                {
                    holdCount[0] = '0';
                    holdCount[1] = '0';
                    holdCount[2]++;
                }
                if (holdCount[2] == 58)
                {
                    holdCount[0] = '0';
                    holdCount[1] = '0';
                    holdCount[2] = '0';
                    holdCount[3]++;
                }
                if (holdCount[3] == 58)
                {
                    holdCount[0] = '0';
                    holdCount[1] = '0';
                    holdCount[2] = '0';
                    holdCount[3] = '0';
                    holdCount[4]++;
                }
                if (holdCount[4] == 58)
                {
                    holdCount[0] = '0';
                    holdCount[1] = '0';
                    holdCount[2] = '0';
                    holdCount[3] = '0';
                    holdCount[4] = '0';
                    holdCount[5]++;
                }
                if (holdCount[5] == 58)
                {
                    holdCount[0] = '0';
                    holdCount[1] = '0';
                    holdCount[2] = '0';
                    holdCount[3] = '0';
                    holdCount[4] = '0';
                    holdCount[5] = '0';
                    holdCount[0]++;
                }
                actualMode = 1;
                mode1work = 1;
                mode0work = 0;
                s1buttonDebounce = 0;                                   // Clear button debounce
                s2buttonDebounce = 0;                                   // Clear button debounce
            }
            else
            {
                actualMode = 0;
                mode0work = 1;
                mode1work = 0;
                s1buttonDebounce = 0;                                   // Clear button debounce
                s2buttonDebounce = 0;                                   // Clear button debounce
            }
            __bic_SR_register_on_exit(LPM3_bits);                // exit LPM3
        }
    }

    // Button S1 released
    if (P1IN & BIT2)
    {
        s1buttonDebounce = 0;                                   // Clear button debounce
        P1OUT &= ~BIT0;
    }

    // Button S2 released
    if (P2IN & BIT6)
    {
        s2buttonDebounce = 0;                                   // Clear button debounce
        P4OUT &= ~BIT0;
    }

    // Both button S1 & S2 released
    if ((P1IN & BIT2) && (P2IN & BIT6))
    {
        // Stop timer A0
        Timer_A_stop(TIMER_A0_BASE);
        s1buttonDebounce = 0;                                   // Clear button debounce
        s2buttonDebounce = 0;                                   // Clear button debounce
        if (actualMode == 1)
            if (mode1work == 0)
                __bic_SR_register_on_exit(LPM3_bits);                // exit LPM3
//        __bic_SR_register_on_exit(LPM3_bits);                // exit LPM3
    }
}

/*
 * RTC Interrupt Service Routine
 * Wakes up every ~10 milliseconds to update stopwatch
 */
#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
{
    switch(__even_in_range(RTCIV, RTCIV_RTCIF))
    {
        case RTCIV_NONE : break;
        case RTCIV_RTCIF:
            if (actualMode == 0 || actualMode == 1)
            {
                mode0work = 1;
                // Since RTC runs at 32768 Hz and isn't fast enough to Count 10 ms exactly
                // offset RTC Counter every 100 10ms intervals to add up to 1s
                // (327 * 32) + (328 * 68) = 32768
                if((Count)==31)
                {
                    // Set RTC to interrupt after 328 XT1 cycles
                    RTC_setModulo(RTC_BASE, 327);
                }
                else if((Count)==35 && beep)
                {
                    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
                }
                else if((Count)==40 && beep)
                {
                    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
                }
                else if((Count)==45 && beep)
                {
                    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
                }
                else if((Count)==50 && beep)
                {
                    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
                }
                else if((Count)==99) //this is the 1 second mark
                {
                    // Set RTC to interrupt after 327 XT1 cycles
                    RTC_setModulo(RTC_BASE, 326);
                    if (ss[0] >= '9')
                    {
                        ss[0] = '0';
                        ss[1]++;
                    }
                    else
                    {
                        ss[0]++;
                    }
                    (Count)=0; //reset to 0
                }
                if (ss[1] >= '6')
                {
                    min[0]++;
                    if (min[0] > '9')
                    {
                        min[0] = '0';
                        min[1]++;
                    }
                    ss[1] = '0';
                }
                if (min[1] >= '6')
                {
                    hr[0]++;
                    if (hr[0] > '9')
                    {
                        hr[1]++;
                        hr[0] = '0';
                    }
                    if (hr[1]-'0' == 1 && hr[0]-'0' == 2) //from 11:59 to 12:00
                    {
                        AmPmINT = (AmPmINT == 1) ? 0 : 1; //flips the half of day
                        if (AmPmINT) //if 1 then it's PM
                        {
                            AmPm[1] = 'P';
                        }
                        else //if 0 then it's AM
                        {
                            AmPm[1] = 'A';
                        }
                    }
                    else if (hr[1]-'0' == 1 && hr[0]-'0' > 2) // if 12:59 to 13:00 -> 01:00
                    {
                        hr[0] = '1';
                        hr[1] = '0';
                    }
                    min[1] = '0';
                }
                (Count)++;
                __bic_SR_register_on_exit(LPM3_bits);            // exit LPM3
            }
            break;
    }
}

/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
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
//****************************************************************************
//
// main.c - MSP-EXP432P401R + Educational Boosterpack MkII - Joystick
//
//          Displays raw 14-bit ADC measurements for X/Y axis of Joystick
//
//****************************************************************************

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>

/* Graphic library context */
Graphics_Context g_sContext;

/* ADC results buffer */
static uint16_t resultsBuffer[2];

int win;
    int board[6][7];
    int row;
    int col;
    int maxRow = 5;
    int maxCol = 6;
    int player = 0;
    int i = 0;
    int j = 0;

/*
 * Main function
 */
int main(void)
{
    //joystick simple GPIO
    P4->SEL0 &= ~0x02;
    P4->SEL1 &= ~0x02;
    P4->DIR &= ~0x02;
    P4->REN |= 0x02;
    P4OUT |= 0x02;

    P5->SEL1 &= ~0x02;//set P5.1 simple GPIO(set to 0)
    P5->SEL0 &= ~0x02;//P5.1 simple GPIO(set to 0)
    P5->DIR &= ~0x02;//set P5.1 as inputs(set to 0)
    P5->REN |= 0x02;// active pull resistors(set to 1)
    P5OUT  |= 0x02;//pull up resistors(set to 1)

    P3->SEL1 &= ~0x20;//set P3.5 simple GPIO(set to 0)
            P3->SEL0 &= ~0x20;//P3.5 simple GPIO(set to 0)
            P3->DIR &= ~0x20;//set P3.5 as inputs(set to 0)
            P3->REN |= 0x20;// active pull resistors(set to 1)
            P3OUT  |= 0x20;//pull up resistors(set to 1)
    /* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);

    /* Configures Pin 6.0 and 4.4 as ADC input */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
         * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
     *  is complete and enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT1);

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    //coordinates for circle
    int x = 64;
    int y = 64;
    int width = 128;
    int height = 128;
    int i = 1;
    int player = 0;


    int slot1 = width / 7;
    int slot2 = 2 * width / 7;
    int slot3 = 3 * width / 7;
    int slot4 = 4 * width / 7;
    int slot5 = 5 * width / 7;
    int slot6 = 6 * width / 7;
    int slot7 = width;
    int slot1count = height - 10;
    int slot2count = height - 10;
    int slot3count = height - 10;
    int slot4count = height - 10;
    int slot5count = height - 10;
    int slot6count = height - 10;
    int slot7count = height - 10;

    int col1Count = 0;
    int col2Count = 0;
    int col3Count = 0;
    int col4Count = 0;
    int col5Count = 0;
    int col6Count = 0;
    int col7Count = 0;


    for(i = 1; i < 7; i++)
        {
            Graphics_drawLineV(&g_sContext, i * height / 7, 0 , width);
        }
    for(i = 1; i < 6; i++)
        {
            Graphics_drawLineH(&g_sContext, 0, width, i * height / 6);
        }
    //draws circle in direction of joystick
    while(win == 0)
    {
        MAP_PCM_gotoLPM0();
        if(player == 0)
        {
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
        }
        if(player == 1)
        {
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
        }
                Graphics_drawCircle(&g_sContext, x, y - 61, 5);
                Graphics_fillCircle(&g_sContext, x, y - 61, 5);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
                Graphics_drawCircle(&g_sContext, x, y - 61, 5);
                Graphics_fillCircle(&g_sContext, x, y - 61, 5);
                x = ADC14_getResult(ADC_MEM0) / 117;
                //y =  (64 - ADC14_getResult(ADC_MEM1) / 117) + 64;
                MAP_PCM_gotoLPM0();
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                for(i = 1; i < 7; i++)
                {
                    Graphics_drawLineV(&g_sContext, i * height / 7, 0, width);
                }
                for(i = 1; i < 6; i++)
                {
                    Graphics_drawLineH(&g_sContext, 0, width, i * height / 6);
                }

                if(player == 0)
                {
                    if((~P5IN & 0x02) && (x <= width / 7))
                    {
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                            Graphics_drawCircle(&g_sContext, slot1 - 9, slot1count, 6);
                            Graphics_fillCircle(&g_sContext, slot1 - 9, slot1count, 6);
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                            slot1count = slot1count - height / 6;
                            player = !player;
                            col1Count++;
                            board[0][5 - col1Count] = 1;
                    }
                    if((~P5IN & 0x02) && (x >= width / 7 && x <= 2 * width / 7))
                    {
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                            Graphics_drawCircle(&g_sContext, slot2 - 9, slot2count, 6);
                            Graphics_fillCircle(&g_sContext, slot2 - 9, slot2count, 6);
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                            slot2count = slot2count - height / 6;
                            player = !player;
                            col2Count++;
                            board[1][5 - col2Count] = 1;
                    }
                    if((~P5IN & 0x02) && (x >= 2 * width / 7 && x <= 3 * width / 7))
                    {
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                            Graphics_drawCircle(&g_sContext, slot3 - 9, slot3count, 6);
                            Graphics_fillCircle(&g_sContext, slot3 - 9, slot3count, 6);
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                            slot3count = slot3count - height / 6;
                            player = !player;
                            col3Count++;
                            board[2][5 - col3Count] = 1;
                    }
                    if((~P5IN & 0x02) && (x >= 3 * width / 7 && x <= 4 * width / 7))
                    {
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                            Graphics_drawCircle(&g_sContext, slot4 - 9, slot4count, 6);
                            Graphics_fillCircle(&g_sContext, slot4 - 9, slot4count, 6);
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                            slot4count = slot4count - height / 6;
                            player = !player;
                            col4Count++;
                            board[3][5 - col4Count] = 1;
                    }
                    if((~P5IN & 0x02) && (x >= 4 * width / 7 && x <= 5 * width / 7))
                    {
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                            Graphics_drawCircle(&g_sContext, slot5 - 9, slot5count, 6);
                            Graphics_fillCircle(&g_sContext, slot5 - 9, slot5count, 6);
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                            slot5count = slot5count - height / 6;
                            player = !player;
                            col5Count++;
                            board[4][5 - col5Count] = 1;
                    }
                    if((~P5IN & 0x02) && (x >= 5 * width / 7 && x <= 6 * width / 7))
                    {
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                            Graphics_drawCircle(&g_sContext, slot6 - 9, slot6count, 6);
                            Graphics_fillCircle(&g_sContext, slot6 - 9, slot6count, 6);
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                            slot6count = slot6count - height / 6;
                            player = !player;
                            col6Count++;
                            board[5][5 - col6Count] = 1;
                    }
                    if((~P5IN & 0x02) && (x >= 6 * width / 7 && x <= width))
                    {
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                            Graphics_drawCircle(&g_sContext, slot7 - 9, slot7count, 6);
                            Graphics_fillCircle(&g_sContext, slot7 - 9, slot7count, 6);
                            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                            slot7count = slot7count - height / 6;
                            player = !player;
                            col7Count++;
                            board[6][5 - col7Count] = 1;
                    }
                }

                if(player == 1)
                {
                    if((~P3IN & 0x20) && (x <= width / 7))
                {
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                        Graphics_drawCircle(&g_sContext, slot1 - 9, slot1count, 6);
                        Graphics_fillCircle(&g_sContext, slot1 - 9, slot1count, 6);
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                        slot1count = slot1count - height / 6;
                        player = !player;
                        col1Count++;
                        board[0][5 - col1Count] = 1;

                }
                if((~P3IN & 0x20) && (x >= width / 7 && x <= 2 * width / 7))
                {
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                        Graphics_drawCircle(&g_sContext, slot2 - 9, slot2count, 6);
                        Graphics_fillCircle(&g_sContext, slot2 - 9, slot2count, 6);
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                        slot2count = slot2count - height / 6;
                        player = !player;
                        col2Count++;
                        board[1][5 - col2Count] = 1;
                }
                if((~P3IN & 0x20) && (x >= 2 * width / 7 && x <= 3 * width / 7))
                {
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                        Graphics_drawCircle(&g_sContext, slot3 - 9, slot3count, 6);
                        Graphics_fillCircle(&g_sContext, slot3 - 9, slot3count, 6);
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                        slot3count = slot3count - height / 6;
                        player = !player;
                        col3Count++;
                        board[2][5 - col3Count] = 1;
                }
                if((~P3IN & 0x20) && (x >= 3 * width / 7 && x <= 4 * width / 7))
                {
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                        Graphics_drawCircle(&g_sContext, slot4 - 9, slot4count, 6);
                        Graphics_fillCircle(&g_sContext, slot4 - 9, slot4count, 6);
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                        slot4count = slot4count - height / 6;
                        player = !player;
                        col4Count++;
                        board[3][5 - col4Count] = 1;
                }
                if((~P3IN & 0x20) && (x >= 4 * width / 7 && x <= 5 * width / 7))
                {
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                        Graphics_drawCircle(&g_sContext, slot5 - 9, slot5count, 6);
                        Graphics_fillCircle(&g_sContext, slot5 - 9, slot5count, 6);
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                        slot5count = slot5count - height / 6;
                        player = !player;
                        col5Count++;
                        board[4][5 - col5Count] = 1;
                }
                if((~P3IN & 0x20) && (x >= 5 * width / 7 && x <= 6 * width / 7))
                {
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                        Graphics_drawCircle(&g_sContext, slot6 - 9, slot6count, 6);
                        Graphics_fillCircle(&g_sContext, slot6 - 9, slot6count, 6);
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                        slot6count = slot6count - height / 6;
                        player = !player;
                        col6Count++;
                        board[5][5 - col6Count] = 1;
                }
                if((~P3IN & 0x20) && (x >= 6 * width / 7 && x <= width))
                {
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                        Graphics_drawCircle(&g_sContext, slot7 - 9, slot7count, 6);
                        Graphics_fillCircle(&g_sContext, slot7 - 9, slot7count, 6);
                        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                        slot7count = slot7count - height / 6;
                        player = !player;
                        col7Count++;
                        board[6][5 - col7Count] = 1;
                }
            }
    }
}

/**
    // horizontalCheck
    for (int j = 0; j <  ; j++ )
    {
        for (int i = 0; i < maxCol; i++)
        {
            if (board[i][j] == player && board[i][j+1] == player && board[i][j+2] == player && board[i][j+3] == player)
            {
                return true;
            }
        }
    }
    // verticalCheck
    for (int i = 0; i<maxCol-3 ; i++ )
    {
        for (int j = 0; j<maxRow; j++)
        {
            if (board[i][j] == player && board[i+1][j] == player && board[i+2][j] == player && board[i+3][j] == player)
            {
                return true;
            }
        }
    }
    // ascendingDiagonalCheck
    for (int i=3; i<maxCol; i++)
    {
        for (int j=0; j<maxRow-3; j++)
        {
            if (board[i][j] == player && board[i-1][j+1] == player && board[i-2][j+2] == player && board[i-3][j+3] == player)
            {
                return true;
            }
        }
    }
    // descendingDiagonalCheck
    for (int i=3; i<maxCol; i++)
    {
        for (int j=3; j<maxWidth; j++)
        {
            if (board[i][j] == player && board[i-1][j-1] == player && board[i-2][j-2] == player && board[i-3][j-3] == player)
            {
                return true;
            }
        }
    }
    return false;
}
/**
/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if(status & ADC_INT1)
    {
        /* Store ADC14 conversion results */
        resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = ADC14_getResult(ADC_MEM1);
    }
}

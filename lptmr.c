/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_lptmr.h"
#include "fsl_gpio.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "lcd.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_LPTMR_BASE LPTMR0
#define DEMO_LPTMR_IRQn LPTMR0_IRQn
#define LPTMR_LED_HANDLER LPTMR0_IRQHandler
/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
/* Define LPTMR microseconds counts value */
#define LPTMR_USEC_COUNT 1000000U
#define LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile uint32_t lptmrCounter = 0U;
int status_bt1 = 0;
int status_bt2 = 0;
int status = 0;
int select = 0;
int rep = 0;
uint32_t secCounter = 0U;
uint32_t minCounter = 0U;
uint32_t hourCounter = 0U;

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

/*******************************************************************************
 * Code
 ******************************************************************************/
void LPTMRIntHandler(void)
{
    LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);
    lptmrCounter++;
    LED_TOGGLE();
    /*
     * Workaround for TWR-KV58: because write buffer is enabled, adding
     * memory barrier instructions to make sure clearing interrupt flag completed
     * before go out ISR
     */
    __DSB();
    __ISB();
}

// Switches
void bt1_init() {
 SIM->COPC = 0;       // Desactiva el Watchdog
 SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;  // Conecta el reloj al puerto C

 //SW1_POS y SW2 son los puertos de los botones.
 PORTC->PCR[3] |= PORT_PCR_MUX(1); // Activa el GPIO
 PORTC->PCR[3] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
 PORTC->PCR[3] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

 // IRQ
 PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
 NVIC_SetPriority(31, 0); // Prioridad de la interrupcion 31
 NVIC_EnableIRQ(31);   // Activa la interrupcion
}

void bt2_init() {
 SIM->COPC = 0;       // Desactiva el Watchdog
 SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;  // Conecta el reloj al puerto C

 PORTC->PCR[12] |= PORT_PCR_MUX(1); // Activa el GPIO
 PORTC->PCR[12] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
 PORTC->PCR[12] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

 // IRQ
 PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
 NVIC_SetPriority(31, 0); // Prioridad de la interrupcion 31
 NVIC_EnableIRQ(31);   // Activa la interrupcion
}

void led_green_init()
{
 SIM->COPC = 0;         // Desactiva el Watchdog
 SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;    // Conecta el reloj al puerto D
 PORTD->PCR[5] = PORT_PCR_MUX(1); // Configura los pines necesarios como GPIO
 GPIOD->PDDR |= (1 << 5);   // Se configura como pin de salida
 GPIOD->PSOR |= (1 << 5);   // Se pone a 1 el pin de salida
}

void led_green_on(void)
{
 GPIOD->PCOR |= (1 << 5);
}

void led_green_off(void) {
 GPIOD->PSOR |= (1 << 5);
}

void PORTDIntHandler(void) {
  int pressed_switch = PORTC->ISFR;
  PORTC->ISFR = 0xFFFFFFFF; // Clear IRQ
        // SW1
        if (status == 0) {
		if(pressed_switch == (0x8)) {
		    rep = 0;
		    status_bt1 = 1;
		    status = 1;
                    led_green_on();
		} else
		  status_bt1 = 0;
		if(pressed_switch == (0x1000)) {
			if (rep == 0)
				rep = 1;
			else rep = 0;
		}
	} else {
		if(pressed_switch == (0x8)) {
		    status_bt1 = 1;
		    if (select == 0)
			select = 1;
		    else {
			status = 0;
  			led_green_off();
 			select = 0;
		    }
		} else
			status_bt1 = 0;
		if(pressed_switch == (0x1000)) {
		    status_bt2 = 1;
		    if (select == 0) {
			if (hourCounter > 22) {
		      	  hourCounter = 0U;
		    	} else
			  hourCounter++;
		    } else {
			if (minCounter > 58) {
		      	  minCounter = 0U;
		    	} else
			  minCounter++;
		    }
		} else
			status_bt2 = 0;	
		if ((status_bt1 == 1) && (status_bt2 == 1)) {
			status = 0;
  			led_green_off();
 			select = 0;
	    	}		
	}
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t currentCounter = 0U;
    lptmr_config_t lptmrConfig;

    irclk_ini(); // Enable internal ref clk to use by LCD
    lcd_ini();

    LED_INIT();
    led_green_init();

    bt1_init();
    bt2_init();

    /* Board pin, clock, debug console init */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Configure LPTMR */
    
     lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     lptmrConfig.enableFreeRunning = false;
     lptmrConfig.bypassPrescaler = true;
     lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     
    LPTMR_GetDefaultConfig(&lptmrConfig);

    /* Initialize the LPTMR */
    LPTMR_Init(DEMO_LPTMR_BASE, &lptmrConfig);

    /*
     * Set timer period.
     * Note : the parameter "ticks" of LPTMR_SetTimerPeriod should be equal or greater than 1.
    */
    LPTMR_SetTimerPeriod(DEMO_LPTMR_BASE, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(DEMO_LPTMR_BASE, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(DEMO_LPTMR_IRQn);

    PRINTF("Low Power Timer Example\r\n");

    /* Start counting */
    LPTMR_StartTimer(DEMO_LPTMR_BASE);
    while (1)
    {
	if (((secCounter % 60) == 0) && (secCounter > 0)) {
	  secCounter = 0U;
	  if (minCounter > 58) {
    	    minCounter = 0U;  
	    hourCounter++;  
	  } else
	    minCounter++;
	  if (hourCounter > 23) {
		hourCounter = 0U;
	  }
	}
	if (rep == 0)
		lcd_display_time(hourCounter, minCounter);
	else lcd_display_time(minCounter, secCounter);
        if (currentCounter != lptmrCounter)
        {
            currentCounter = lptmrCounter;
	    secCounter++;
            PRINTF("LPTMR interrupt No.%d \r\n", currentCounter);
        }
    }
}

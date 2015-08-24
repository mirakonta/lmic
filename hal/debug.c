/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/


#include "debug.h"
#include "../lmic/lmic.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_leuart.h"

#if defined PA52
	#define PORT_TXD	gpioPortD	//TXD = PD4
	#define PIN_TXD		4U
	#define PORT_RXD	gpioPortD	//RXD = PD5
	#define PIN_RXD		5U

	#define PORT_SPK	gpioPortC	//REED3 = PC15 TILT = SPK
	#define PIN_SPK		15U
#elif defined PA53
	#define PORT_TXD	gpioPortD	//TXD = PD4
	#define PIN_TXD		4U
	#define PORT_RXD	gpioPortD	//RXD = PD5
	#define PIN_RXD		5U

	#define PORT_SPK	gpioPortC	//REED3 = PC15 TILT = SPK
	#define PIN_SPK		15U
#endif

void debug_init ()
{
	GPIO_PinModeSet(PORT_SPK, PIN_SPK, gpioModePushPull, 0);

	LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);
	CMU_ClockEnable(cmuClock_LEUART0, true);

	init.refFreq =  0; //14MHz / 2 pre-scaled by 4 = 1750000;
	init.enable =   leuartDisable;
	init.baudrate = 9600;
	init.databits = leuartDatabits8;
	init.parity =   leuartNoParity;
	init.stopbits = leuartStopbits1;

	// Reseting and initializing LEUART0
	LEUART_Reset(LEUART0);
	LEUART_Init(LEUART0, &init);

	GPIO_PinModeSet(PORT_TXD, PIN_TXD, gpioModePushPull, 1);
	GPIO_PinModeSet(PORT_RXD, PIN_RXD, gpioModeInput, 0);

	LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN | LEUART_ROUTE_LOCATION_LOC0;
	LEUART0->CMD = LEUART_CMD_TXDIS | LEUART_CMD_RXDIS | LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX;
	LEUART0->CMD = LEUART_CMD_TXEN | LEUART_CMD_RXEN;

	// Eventually enable UART
	LEUART_Enable(LEUART0, leuartEnable);

	// print banner
    debug_str("\r\n============= DEBUG STARTED =============\r\n");
}

void debug_led (u1_t val)
{
	if (val)
		GPIO_PinOutSet(PORT_SPK, PIN_SPK);
	else
		GPIO_PinOutClear(PORT_SPK, PIN_SPK);
}

void debug_char (u1_t c)
{
	LEUART_Tx(LEUART0, c);
}

void debug_hex (u1_t b) {
    debug_char("0123456789ABCDEF"[b>>4]);
    debug_char("0123456789ABCDEF"[b&0xF]);
}

void debug_buf (const u1_t* buf, u2_t len) {
    while(len--) {
        debug_hex(*buf++);
        debug_char(' ');
    }
    debug_char('\r');
    debug_char('\n');
}

void debug_uint (u4_t v) {
    for(s1_t n=24; n>=0; n-=8) {
        debug_hex(v>>n);
    }
}

void debug_str (const u1_t* str) {
    while(*str) {
        debug_char(*str++);
    }
}

void debug_val (const u1_t* label, u4_t val) {
    debug_str(label);
    debug_uint(val);
    debug_char('\r');
    debug_char('\n');
}

void debug_event (int ev) {
    static const u1_t* evnames[] = {
        [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
        [EV_BEACON_FOUND]   = "BEACON_FOUND",
        [EV_BEACON_MISSED]  = "BEACON_MISSED",
        [EV_BEACON_TRACKED] = "BEACON_TRACKED",
        [EV_JOINING]        = "JOINING",
        [EV_JOINED]         = "JOINED",
        [EV_RFU1]           = "RFU1",
        [EV_JOIN_FAILED]    = "JOIN_FAILED",
        [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
        [EV_TXCOMPLETE]     = "TXCOMPLETE",
        [EV_LOST_TSYNC]     = "LOST_TSYNC",
        [EV_RESET]          = "RESET",
        [EV_RXCOMPLETE]     = "RXCOMPLETE",
        [EV_LINK_DEAD]      = "LINK_DEAD",
        [EV_LINK_ALIVE]     = "LINK_ALIVE",
    };
    debug_str(evnames[ev]);
    debug_char('\r');
    debug_char('\n');
}

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

#include "../../lmic/lmic.h"
#include "../../hal/debug.h"

// LMIC application callbacks not used in his example
void os_getArtEui (u1_t* buf) {
}

void os_getDevEui (u1_t* buf) {
}

void os_getDevKey (u1_t* buf) {
}

void onEvent (ev_t ev) {
}

// counter
static int cnt = 0;

// log text to USART and toggle LED
static void initfunc (osjob_t* job) {
    // say hello
    debug_str("Hello World!\r\n");
    // log counter
    debug_val("cnt = ", cnt);
    // toggle LED
    debug_led(++cnt & 1);
    // reschedule job every second
    os_setTimedCallback(job, os_getTime()+sec2osticks(1), initfunc);
}

// application entry point
int main ()
{
	osjob_t initjob;
	u4_t time;

	debug_init();	// initialize debug library

    /*hal_init();
    while(1)
	{
		debug_str("\r\nT=");
		time = hal_ticks();
		debug_hex(time>>24);
		debug_hex(time>>16);
		debug_hex(time>>8);
		debug_hex(time>>0);
	}*/

	os_init();	// initialize runtime env

	os_setCallback(&initjob, initfunc);	// setup initial job

	os_runloop();	// execute scheduled jobs and events


	return 0;	// (not reached)
}

/*******************************************************************************
 * Copyright (c) 2014 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

#include "lmic.h"

// counter
static int cnt = 0;

// log text to USART and toggle LED
static void initfunc (osjob_t* job) {
    // say hello
    DEBUG_STR("Hello World!\r\n");
    // log counter
    DEBUG_VAL("cnt = ", cnt);
    // toggle LED
    DEBUG_LED(++cnt & 1);
    // reschedule job every second
    os_setTimedCallback(job, os_getTime()+sec2osticks(1), initfunc);
}

// application entry point
void main () {
    osjob_t initjob;

    // initialize runtime env
    os_init();
    // setup initial job
    os_setCallback(&initjob, initfunc);
    // execute scheduled jobs and events
    os_runloop();
    // (not reached)
}

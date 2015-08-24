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

// sensor functions
extern void initsensor(void);
extern u2_t readsensor(void);


//////////////////////////////////////////////////
// CONFIGURATION (FOR APPLICATION CALLBACKS BELOW)
//////////////////////////////////////////////////

// application router ID (LSBF)
static const u1_t APPEUI[8]  = { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// unique device ID (LSBF)
static const u1_t DEVEUI[8]  = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// device-specific AES key (derived from device EUI)
static const u1_t DEVKEY[16] = { 0xE7, 0x63, 0x2E, 0x1C, 0xF3, 0x61, 0x7E, 0xAD, 0xF5, 0xC0, 0xBC, 0x7E, 0x38, 0xEA, 0x09, 0xA8  };

static const u1_t nwkKey[16] = { 0xE7, 0x63, 0x2E, 0x1C, 0xF3, 0x61, 0x7E, 0xAD, 0xF5, 0xC0, 0xBC, 0x7E, 0x38, 0xEA, 0x09, 0xA8  };

static const u1_t artKey[16] = { 0xE7, 0x63, 0x2E, 0x1C, 0xF3, 0x61, 0x7E, 0xAD, 0xF5, 0xC0, 0xBC, 0x7E, 0x38, 0xEA, 0x09, 0xA8  };


//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
}

//////////////////////////////////////////////////
// UTILITY JOB
//////////////////////////////////////////////////
static osjob_t reportjob;

// report sensor value every minute
static void reportfunc (osjob_t* j) {
    // read sensor
    u2_t val = readsensor();
    debug_val("val = ", val);
    // prepare and schedule data for transmission
    LMIC.frame[0] = val << 8;
    LMIC.frame[1] = val;
    LMIC_setTxData2(1, LMIC.frame, 2, 1); // (port 1, 2 bytes, unconfirmed)
    os_setTimedCallback(j, os_getTime()+sec2osticks(5), reportfunc);     // reschedule job in 5 seconds

}

static osjob_t alivejob;

static void alivefunc(osjob_t* j)
{
	debug_str("\r\nalive\r\n");
	os_setTimedCallback(j, os_getTime()+sec2osticks(1), alivefunc);     // reschedule job in 5 seconds
}
//////////////////////////////////////////////////
// MAIN - INITIALIZATION AND STARTUP
//////////////////////////////////////////////////

// initial job
static void initfunc (osjob_t* j) {
    // intialize sensor hardware
    initsensor();
    // reset MAC state
    LMIC_reset();
    // start joining
    LMIC_setSession(0x12345678, 0xB710566C, nwkKey, artKey);
    // kick-off periodic sensor job
    alivefunc(&alivejob);
    reportfunc(&reportjob);

    //LMIC_startJoining();
    // init done - onEvent() callback will be invoked...
}


// application entry point
int main () {
    osjob_t initjob;

    // initialize runtime env
    os_init();
    // initialize debug library
    debug_init();
    // setup initial job
    os_setCallback(&initjob, initfunc);
    // execute scheduled jobs and events
    os_runloop();
    // (not reached)
    return 0;
}




//////////////////////////////////////////////////
// LMIC EVENT CALLBACK
//////////////////////////////////////////////////

void onEvent (ev_t ev) {
    debug_event(ev);

    switch(ev) {

      // network joined, session established
      case EV_JOINED:
          // switch on LED
          debug_led(1);
          // kick-off periodic sensor job
          reportfunc(&reportjob);
          break;
    }
}

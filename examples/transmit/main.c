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

//////////////////////////////////////////////////
// CONFIGURATION (FOR APPLICATION CALLBACKS BELOW)
//////////////////////////////////////////////////

// application router ID (LSBF)
static const u1_t APPEUI[8]  = { 0xBE, 0x7A, 0x00, 0x0B, 0xE7, 0xA0, 0x00, 0x40 };//0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// unique device ID (LSBF)
static const u1_t DEVEUI[8]  = { 0xBE, 0x7A, 0x00, 0x0B, 0xE7, 0xA0, 0x00, 0x45 };//0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

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
// MAIN - INITIALIZATION AND STARTUP
//////////////////////////////////////////////////

// initial job
static void initfunc (osjob_t* j) {

    LMIC_reset();	// reset MAC state

   LMIC_setSession(0x12345678, 0xB710566C, nwkKey, artKey);

   //LMIC_startJoining();	// start joining
   // init done - onEvent() callback will be invoked...

/*void LMIC_setSession (u4_t netid, devaddr_t(u4_t) devaddr, xref2u1_t nwkKey, xref2u1_t artKey) {
LMIC.netid = netid;
LMIC.devaddr = devaddr;
if( nwkKey != (xref2u1_t)0 )
os_copyMem(LMIC.nwkKey, nwkKey, 16);
if( artKey != (xref2u1_t)0 )
os_copyMem(LMIC.artKey, artKey, 16);*/

debug_str("\r\nSending");
// immediately prepare next transmission
LMIC.frame[0] = LMIC.snr;
// schedule transmission (port 1, datalen 1, no ack requested)
LMIC_setTxData2(1, LMIC.frame, 1, 0);
// (will be sent as soon as duty cycle permits)

LMIC_sendAlive();
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

void onEvent (ev_t ev)
{
    debug_event(ev);

    switch(ev)
    {
   
      // network joined, session established
      case EV_JOINED:
          debug_val("\r\nnetid = ", LMIC.netid);

          // immediately prepare next transmission
          debug_str("\r\nSending");
          LMIC.frame[0] = LMIC.snr;
          LMIC_setTxData2(1, LMIC.frame, 1, 0);	// schedule transmission (port 1, datalen 1, no ack requested)
    	  break;
        
      // scheduled data sent (optionally data received)
      case EV_TXCOMPLETE:
          if(LMIC.dataLen) { // data received in rx slot after tx
        	  debug_str("\r\nReceived:");
              debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
          }
          // immediately prepare next transmission
          LMIC.frame[0] = LMIC.snr;
          debug_val("\r\nSending:", LMIC.frame[0]);
			// schedule transmission (port 1, datalen 1, no ack requested)
          LMIC_setTxData2(1, LMIC.frame, 1, 0);
          break;
    }
}

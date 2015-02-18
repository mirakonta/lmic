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

#ifndef _hal_hpp_
#define _hal_hpp_

/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
void hal_init ();

/*
 * drive radio NSS pin (0=low, 1=high).
 */
void hal_pin_nss (u1_t val);

/*
 * drive radio RX/TX pins (0=rx, 1=tx).
 */
void hal_pin_rxtx (u1_t val);

/*
 * control radio RST pin (0=low, 1=high, 2=floating)
 */
void hal_pin_rst (u1_t val);

/*
 * perform 8-bit SPI transaction with radio.
 *   - write given byte 'outval'
 *   - read byte and return value
 */
u1_t hal_spi (u1_t outval);

/*
 * disable all CPU interrupts.
 *   - might be invoked nested 
 *   - will be followed by matching call to hal_enableIRQs()
 */
void hal_disableIRQs ();

/*
 * enable CPU interrupts.
 */
void hal_enableIRQs ();

/*
 * put system and CPU in low-power mode, sleep until interrupt.
 */
void hal_sleep ();

/*
 * return 32-bit system time in ticks.
 */
u4_t hal_ticks ();

/*
 * busy-wait until specified timestamp (in ticks) is reached.
 */
void hal_waitUntil (u4_t time);

/*
 * check and rewind timer for target time.
 *   - return 1 if target time is close
 *   - otherwise rewind timer for target time or full period and return 0
 */
u1_t hal_checkTimer (u4_t targettime);

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void hal_failed ();

//////////////////////////////////////////////////////////////////////
#ifdef CFG_DEBUG
void debug_init ();
void debug_led (u1_t val);
void debug_char (u1_t c);
void debug_hex (u1_t b);
void debug_buf (const u1_t* buf, u2_t len);
void debug_uint (u4_t v);
void debug_str (const u1_t* str);
void debug_event (int ev);
void debug_val (const u1_t* label, u4_t val);
#define DEBUG_INIT()   debug_init()
#define DEBUG_LED(v)   debug_led(v)
#define DEBUG_CHAR(c)  debug_char(c)
#define DEBUG_HEX(b)   debug_hex(b)
#define DEBUG_BUF(b,l) debug_buf(b,l)
#define DEBUG_UINT(v)  debug_uint(v)
#define DEBUG_STR(s)   debug_str(s)
#define DEBUG_EVENT(e) debug_event(e)
#define DEBUG_VAL(l,v) debug_val(l, v)
#else // CFG_DEBUG
#define DEBUG_INIT()
#define DEBUG_LED(v)
#define DEBUG_CHAR(c)
#define DEBUG_HEX(b)
#define DEBUG_BUF(b,l)
#define DEBUG_UINT(v)
#define DEBUG_STR(s)
#define DEBUG_EVENT(e)
#define DEBUG_VAL(l,v)
#endif // CFG_DEBUG

#endif // _hal_hpp_

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

#define PAMBL_SYMS 8
#define MINRX_SYMS 5
#define PAMBL_FSK  5
#define PRERX_FSK  1
#define RXLEN_FSK  (1+5+2)

#define BCN_INTV_osticks       sec2osticks(BCN_INTV_sec)
#define TXRX_GUARD_osticks     ms2osticks(TXRX_GUARD_ms)
#define JOIN_GUARD_osticks     ms2osticks(JOIN_GUARD_ms)
#define DELAY_DNW1_osticks     sec2osticks(DELAY_DNW1)
#define DELAY_DNW2_osticks     sec2osticks(DELAY_DNW2)
#define DELAY_JACC1_osticks    sec2osticks(DELAY_JACC1)
#define DELAY_JACC2_osticks    sec2osticks(DELAY_JACC2)
#define DELAY_EXTDNW2_osticks  sec2osticks(DELAY_EXTDNW2)
#define BCN_RESERVE_osticks    ms2osticks(BCN_RESERVE_ms)
#define BCN_GUARD_osticks      ms2osticks(BCN_GUARD_ms)
#define BCN_WINDOW_osticks     ms2osticks(BCN_WINDOW_ms)
#define AIRTIME_BCN_osticks    us2osticks(AIRTIME_BCN)

DEFINE_LMIC;
DECL_ON_LMIC_EVENT;


// Fwd decls.
static void engineUpdate();
static void startScan ();


// ================================================================================
// BEG OS - default implementations for certain OS suport functions

#if !HAS_os_calls

#ifndef os_rlsbf2
u2_t os_rlsbf2 (xref2cu1_t buf) {
    return (u2_t)(buf[0] | (buf[1]<<8));
}
#endif

#ifndef os_rlsbf4
u4_t os_rlsbf4 (xref2cu1_t buf) {
    return (u4_t)(buf[0] | (buf[1]<<8) | ((u4_t)buf[2]<<16) | ((u4_t)buf[3]<<24));
}
#endif


#ifndef os_rmsbf4
u4_t os_rmsbf4 (xref2cu1_t buf) {
    return (u4_t)(buf[3] | (buf[2]<<8) | ((u4_t)buf[1]<<16) | ((u4_t)buf[0]<<24));
}
#endif


#ifndef os_wlsbf2
void os_wlsbf2 (xref2u1_t buf, u2_t v) {
    buf[0] = v;
    buf[1] = v>>8;
}
#endif

#ifndef os_wlsbf4
void os_wlsbf4 (xref2u1_t buf, u4_t v) {
    buf[0] = v;
    buf[1] = v>>8;
    buf[2] = v>>16;
    buf[3] = v>>24;
}
#endif

#ifndef os_wmsbf4
void os_wmsbf4 (xref2u1_t buf, u4_t v) {
    buf[3] = v;
    buf[2] = v>>8;
    buf[1] = v>>16;
    buf[0] = v>>24;
}
#endif

#ifndef os_getBattLevel
u1_t os_getBattLevel () {
    return MCMD_DEVS_BATT_NOINFO;
}
#endif

#ifndef os_crc16
static const u2_t CRC_TABLE[] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
u2_t os_crc16 (xref2u1_t data, uint len) {
    u2_t fcs = 0;
    for( uint u=0; u<len; u++ ) {
	u1_t b = data[u];
	fcs = (CRC_TABLE[(fcs ^ b) & 0xFF] ^ ((fcs >> 8) & 0xFF));
    }
    return fcs;
}
#endif

#endif // !HAS_os_calls

// END OS - default implementations for certain OS suport functions
// ================================================================================

// ================================================================================
// BEG AES

static void micB0 (u4_t devaddr, u4_t seqno, int dndir, int len) {
    os_clearMem(AESaux,16);
    AESaux[0]  = 0x49;
    AESaux[5]  = dndir?1:0;
    AESaux[15] = len;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
}


static int aes_verifyMic (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    return os_aes(AES_MIC, pdu, len) == os_rmsbf4(pdu+len);
}


static void aes_appendMic (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    // MSB because of internal structure of AES
    os_wmsbf4(pdu+len, os_aes(AES_MIC, pdu, len));
}


static void aes_appendMic0 (xref2u1_t pdu, int len) {
    os_getDevKey(AESkey);
    os_wmsbf4(pdu+len, os_aes(AES_MIC|AES_MICNOAUX, pdu, len));  // MSB because of internal structure of AES
}


static int aes_verifyMic0 (xref2u1_t pdu, int len) {
    os_getDevKey(AESkey);
    return os_aes(AES_MIC|AES_MICNOAUX, pdu, len) == os_rmsbf4(pdu+len);
}


static void aes_encrypt (xref2u1_t pdu, int len) {
    os_getDevKey(AESkey);
    os_aes(AES_ENC, pdu, len);
}


static void aes_cipher (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t payload, int len) {
    if( len <= 0 )
	return;
    os_clearMem(AESaux, 16);
    AESaux[0] = AESaux[15] = 1; // mode=cipher / dir=down / block counter=1
    AESaux[5] = dndir?1:0;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
    os_copyMem(AESkey,key,16);
    os_aes(AES_CTR, payload, len);
}


static void aes_sessKeys (u2_t devnonce, xref2cu1_t artnonce, xref2u1_t nwkkey, xref2u1_t artkey) {
    os_clearMem(nwkkey, 16);
    nwkkey[0] = 0x01;
    os_copyMem(nwkkey+1, artnonce, LEN_ARTNONCE+LEN_NETID);
    os_wlsbf2(nwkkey+1+LEN_ARTNONCE+LEN_NETID, devnonce);
    os_copyMem(artkey, nwkkey, 16);
    artkey[0] = 0x02;

    os_getDevKey(AESkey);
    os_aes(AES_ENC, nwkkey, 16);
    os_getDevKey(AESkey);
    os_aes(AES_ENC, artkey, 16);
}

// END AES
// ================================================================================


// ================================================================================
// BEG LORA

#if CFG_eu868 // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF9 ? maxFrameLens[(dr)] : 0xFF)
const u1_t maxFrameLens [] = { 64,64,64,123 };

const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    (u1_t)MAKERPS(SF12, BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF11, BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF9,  BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF8,  BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF7,  BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF7,  BW250, CR_4_5, 0, 0),
    (u1_t)MAKERPS(FSK,  BW125, CR_4_5, 0, 0),
    ILLEGAL_RPS
};

static const s1_t TXPOWLEVELS[] = {
    20, 14, 11, 8, 5, 2, 0,0, 0,0,0,0, 0,0,0,0
};
#define pow2dBm(mcmd_ladr_p1) (TXPOWLEVELS[(mcmd_ladr_p1&MCMD_LADR_POW_MASK)>>MCMD_LADR_POW_SHIFT])

#elif CFG_us915 // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF11CR ? maxFrameLens[(dr)] : 0xFF)
const u1_t maxFrameLens [] = { 24,66,142,255,255,255,255,255,  66,142 };

const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    MAKERPS(SF12, BW500, CR_4_5, 0, 0),
    MAKERPS(SF11, BW500, CR_4_5, 0, 0),
    MAKERPS(SF10, BW500, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS
};

#define pow2dBm(mcmd_ladr_p1) ((s1_t)(30 - (((mcmd_ladr_p1)&MCMD_LADR_POW_MASK)<<1)))

#endif // ================================================

static const u1_t SENSITIVITY[7][3] = {
    // ------------bw----------
    // 125kHz    250kHz    500kHz
    { 141-109,  141-109, 141-109 },  // FSK
    { 141-127,  141-124, 141-121 },  // SF7
    { 141-129,  141-126, 141-123 },  // SF8
    { 141-132,  141-129, 141-126 },  // SF9
    { 141-135,  141-132, 141-129 },  // SF10
    { 141-138,  141-135, 141-132 },  // SF11
    { 141-141,  141-138, 141-135 }   // SF12
};

int getSensitivity (rps_t rps) {
    return -141 + SENSITIVITY[getSf(rps)][getBw(rps)];
}

ostime_t calcAirTime (rps_t rps, u1_t plen) {
    u1_t bw = getBw(rps);  // 0,1,2 = 125,250,500kHz
    u1_t sf = getSf(rps);  // 0=FSK, 1..6 = SF7..12
    if( sf == FSK ) {
	return (plen+/*preamble*/5+/*syncword*/2+/*len*/1+/*crc*/2) * /*bits/byte*/8
	    * (s4_t)OSTICKS_PER_SEC / /*kbit/s*/50000;
    }
    u1_t sfx = 4*(sf+(7-SF7));
    u1_t q = sfx - (sf >= SF11 ? 8 : 0);
    int tmp = 8*plen - sfx + 28 + (getNocrc(rps)?0:16) - (getIh(rps)?20:0);
    if( tmp > 0 ) {
	tmp = (tmp + q - 1) / q;
	tmp *= getCr(rps)+5;
	tmp += 8;
    } else {
	tmp = 8;
    }
    tmp = (tmp<<2) + /*preamble*/49 /* 4 * (8 + 4.25) */;
    // bw = 125000 = 15625 * 2^3
    //      250000 = 15625 * 2^4
    //      500000 = 15625 * 2^5
    // sf = 7..12
    //
    // osticks =  tmp * OSTICKS_PER_SEC * 1<<sf / bw
    //
    // 3 => counter reduced divisor 125000/8 => 15625
    // 2 => counter 2 shift on tmp
    sfx = sf+(7-SF7) - (3+2) - bw;
    int div = 15625;
    if( sfx > 4 ) {
	// prevent 32bit signed int overflow in last step
	div >>= sfx-4;
	sfx = 4;
    }
    // Need 32bit arithmetic for this last step
    return (((ostime_t)tmp << sfx) * OSTICKS_PER_SEC + div/2) / div;
}

extern inline s1_t  rssi2s1 (int v);
extern inline int   s12rssi (s1_t v);
extern inline float  s12snr (s1_t v);
extern inline s1_t   snr2s1 (double v);
extern inline int   getRssi (rxqu_t*rxq);
extern inline void  setRssi (rxqu_t*rxq, int v);

extern inline rps_t updr2rps (dr_t dr);
extern inline rps_t dndr2rps (dr_t dr);
extern inline int isFasterDR (dr_t dr1, dr_t dr2);
extern inline int isSlowerDR (dr_t dr1, dr_t dr2);
extern inline dr_t  incDR    (dr_t dr);
extern inline dr_t  decDR    (dr_t dr);
extern inline dr_t  assertDR (dr_t dr);
extern inline dr_t  validDR  (dr_t dr);
extern inline dr_t  lowerDR  (dr_t dr, u1_t n);

extern inline sf_t  getSf    (rps_t params);
extern inline rps_t setSf    (rps_t params, sf_t sf);
extern inline bw_t  getBw    (rps_t params);
extern inline rps_t setBw    (rps_t params, bw_t cr);
extern inline cr_t  getCr    (rps_t params);
extern inline rps_t setCr    (rps_t params, cr_t cr);
extern inline int   getNocrc (rps_t params);
extern inline rps_t setNocrc (rps_t params, int nocrc);
extern inline int   getIh    (rps_t params);
extern inline rps_t setIh    (rps_t params, int ih);
extern inline rps_t makeRps  (sf_t sf, bw_t bw, cr_t cr, int ih, int nocrc);
extern inline int   sameSfBw (rps_t r1, rps_t r2);

// END LORA
// ================================================================================


// Adjust DR for TX retries
//  - indexed by retry count
//  - return steps to lower DR
static const u1_t DRADJUST[2+TXCONF_ATTEMPTS] = {
    // normal frames - 1st try / no retry
    0,
    // confirmed frames
    0,0,0,0,0,1,1,1,2
};


// Table below defines the size of one symbol as
//   symtime = 256us * 2^T(sf,bw)
// 256us is called one symunit. 
//                 SF: 	 				
//  	BW:	 |__7___8___9__10__11__12
//  	125kHz	 |  2   3   4   5   6   7
//  	250kHz	 |  1   2   3   4   5   6
//  	500kHz	 |  0   1   2   3   4   5
//  
// Times for half symbol per DR
// Per DR table to minimize rounding errors
static const ostime_t DR2HSYM_osticks[] = {
#if CFG_eu868
#define dr2hsym(dr) (DR2HSYM_osticks[(dr)])
    us2osticksRound(128<<7),  // DR_SF12
    us2osticksRound(128<<6),  // DR_SF11
    us2osticksRound(128<<5),  // DR_SF10
    us2osticksRound(128<<4),  // DR_SF9
    us2osticksRound(128<<3),  // DR_SF8
    us2osticksRound(128<<2),  // DR_SF7
    us2osticksRound(128<<1),  // DR_SF7B
    us2osticksRound(80)       // FSK -- not used (time for 1/2 byte)
#elif CFG_us915
#define dr2hsym(dr) (DR2HSYM_osticks[(dr)&7])  // map DR_SFnCR -> 0-6
    us2osticksRound(128<<5),  // DR_SF10   DR_SF12CR
    us2osticksRound(128<<4),  // DR_SF9    DR_SF11CR
    us2osticksRound(128<<3),  // DR_SF8    DR_SF10CR
    us2osticksRound(128<<2),  // DR_SF7    DR_SF9CR
    us2osticksRound(128<<1),  // DR_SF8C   DR_SF8CR
    us2osticksRound(128<<0)   // ------    DR_SF7CR
#endif
};


static ostime_t calcRxWindow (u1_t secs, dr_t dr) {
    ostime_t rxoff, err;
    if( secs==0 ) {
	// aka 128 secs (next becaon)
	rxoff = LMIC.drift;
	err = LMIC.lastDriftDiff;
    } else {
	// scheduled RX window within secs into current beacon period
	rxoff = (LMIC.drift * (ostime_t)secs) >> BCN_INTV_exp;
	err = (LMIC.lastDriftDiff * (ostime_t)secs) >> BCN_INTV_exp;
    }
    u1_t rxsyms = MINRX_SYMS;
    err += (ostime_t)LMIC.maxDriftDiff * LMIC.missedBcns;
    LMIC.rxsyms = MINRX_SYMS + (err / dr2hsym(dr));

    return (rxsyms-PAMBL_SYMS) * dr2hsym(dr) + rxoff;
}


// Setup beacon RX parameters assuming we have an error of ms (aka +/-(ms/2))
static void calcBcnRxWindowFromMillis (u1_t ms, bit_t ini) {
    if( ini ) {
	LMIC.drift = 0;
	LMIC.maxDriftDiff = 0;
	LMIC.missedBcns = 0;
	LMIC.bcninfo.flags |= BCN_NODRIFT|BCN_NODDIFF;
    }
    ostime_t hsym = dr2hsym(DR_BCN);
    LMIC.bcnRxsyms = MINRX_SYMS + ms2osticksCeil(ms) / hsym;
    LMIC.bcnRxtime = LMIC.bcninfo.txtime + BCN_INTV_osticks - (LMIC.bcnRxsyms-PAMBL_SYMS) * hsym;
}


// Setup scheduled RX window (ping/multicast slot)
static void rxschedInit (xref2rxsched_t rxsched) {
    os_clearMem(AESkey,16);
    os_clearMem(LMIC.frame+8,8);
    os_wlsbf4(LMIC.frame, LMIC.bcninfo.time);
    os_wlsbf4(LMIC.frame+4, LMIC.devaddr);
    os_aes(AES_ENC,LMIC.frame,16);
    u1_t intvExp = rxsched->intvExp;
    ostime_t off = os_rlsbf2(LMIC.frame) & (0x0FFF >> (7 - intvExp)); // random offset (slot units)
    rxsched->rxbase = (LMIC.bcninfo.txtime +
		       BCN_RESERVE_osticks +
		       ms2osticks(BCN_SLOT_SPAN_ms * off)); // random offset osticks
    rxsched->slot   = 0;
    rxsched->rxtime = rxsched->rxbase - calcRxWindow(/*secs BCN_RESERVE*/2+(1<<intvExp),rxsched->dr);
    rxsched->rxsyms = LMIC.rxsyms;
}


static bit_t rxschedNext (xref2rxsched_t rxsched, ostime_t cando) {
  again:
    if( rxsched->rxtime - cando >= 0 )
	return 1;
    u1_t slot;
    if( (slot=rxsched->slot) >= 128 )
	return 0;
    u1_t intv = 1<<rxsched->intvExp;
    if( (rxsched->slot = (slot += (intv))) >= 128 )
	return 0;
    rxsched->rxtime = rxsched->rxbase
	+ ((BCN_WINDOW_osticks * (ostime_t)slot) >> BCN_INTV_exp)
	- calcRxWindow(/*secs BCN_RESERVE*/2+slot+intv,rxsched->dr);
    rxsched->rxsyms = LMIC.rxsyms;
    goto again;
}


static ostime_t rndDelay (u1_t secSpan) {
    u2_t r = os_getRndU2();
    ostime_t delay = r;
    if( delay > OSTICKS_PER_SEC )
	delay = r % (u2_t)OSTICKS_PER_SEC;
    if( secSpan > 0 )
	delay += ((u1_t)r % secSpan) * OSTICKS_PER_SEC;
    return delay;
}


static void txDelay (ostime_t reftime, u1_t secSpan) {
    reftime += rndDelay(secSpan);
    if( LMIC.globalDutyRate == 0  ||  (reftime - LMIC.globalDutyAvail) > 0 ) {
	LMIC.globalDutyAvail = reftime;
	LMIC.opmode |= OP_RNDTX;
    }
}


static void setDrJoin (u1_t reason, u1_t dr) {
    EV(drChange, INFO, (e_.reason    = reason,
			e_.deveui    = MAIN::CDEV->getEui(),
			e_.dr        = dr|DR_PAGE,
			e_.txpow     = LMIC.adrTxPow,
			e_.prevdr    = LMIC.datarate|DR_PAGE,
			e_.prevtxpow = LMIC.adrTxPow));
    LMIC.datarate = dr;
    DO_DEVDB(updateDatarate, dr);
}


static void setDrTxpow (u1_t reason, u1_t dr, s1_t pow) {
    EV(drChange, INFO, (e_.reason    = reason,
			e_.deveui    = MAIN::CDEV->getEui(),
			e_.dr        = dr|DR_PAGE,
			e_.txpow     = pow,
			e_.prevdr    = LMIC.datarate|DR_PAGE,
			e_.prevtxpow = LMIC.adrTxPow));
    if( pow != KEEP_TXPOW )
	LMIC.adrTxPow = pow;
    LMIC.datarate = dr;
    DO_DEVDB(updateDatarate, dr);
    LMIC.opmode |= OP_NEXTCHNL;
}


void LMIC_stopPingable () {
    LMIC.opmode &= ~(OP_PINGABLE|OP_PINGINI);
}


void LMIC_setPingable (u1_t intvExp) {
    // Change setting
    LMIC.ping.intvExp = (intvExp & 0x7);
    LMIC.opmode |= OP_PINGABLE;
    // App may call LMIC_enableTracking() explicitely before
    // Otherwise tracking is implicitly enabled here
    if( (LMIC.opmode & (OP_TRACK|OP_SCAN)) == 0  &&  LMIC.bcninfoTries == 0 )
	LMIC_enableTracking(0);
}


#if CFG_eu868
// ================================================================================
//
// BEG: EU868 related stuff
//
enum { BAND_MILLI=0, BAND_CENTI=1, BAND_DECI=2 };
static const u4_t iniChannelFreq[12] = {
    // Join frequencies and duty cycle limit (0.1%)
    EU868_F1|BAND_MILLI, EU868_F2|BAND_MILLI, EU868_F3|BAND_MILLI,
    EU868_J4|BAND_MILLI, EU868_J5|BAND_MILLI, EU868_J6|BAND_MILLI,
    // Default operational frequencies
    EU868_F1|BAND_CENTI, EU868_F2|BAND_CENTI, EU868_F3|BAND_CENTI,
    EU868_F4|BAND_MILLI, EU868_F5|BAND_MILLI, EU868_F6|BAND_DECI
};

static void initDefaultChannels (bit_t join) {
    LMIC.channelMap = 0x3F;
    u1_t su = join ? 0 : 6;
    for( u1_t fu=0; fu<6; fu++,su++ ) {
	LMIC.channelFreq[fu] = iniChannelFreq[su];
	LMIC.channelDrs[fu]  = DR_SF12 | (DR_SF7<<4);
    }
    if( !join )
	LMIC.channelDrs[5] = DR_SF12 | (DR_FSK<<4);

    LMIC.bands[BAND_MILLI].txcap = 1000;  // 0.1%
    LMIC.bands[BAND_MILLI].txpow = 14;
    LMIC.bands[BAND_CENTI].txcap = 100;   // 1%
    LMIC.bands[BAND_CENTI].txpow = 14;
    LMIC.bands[BAND_DECI ].txcap = 10;    // 10%
    LMIC.bands[BAND_DECI ].txpow = 27;
    LMIC.bands[BAND_MILLI].avail = 
    LMIC.bands[BAND_CENTI].avail =
    LMIC.bands[BAND_DECI ].avail = os_getTime();
}

static u4_t convFreq (xref2u1_t ptr) {
    u4_t freq = (os_rlsbf4(ptr-1) >> 8) * 100;
    if( freq >= EU868_FREQ_MIN && freq <= EU868_FREQ_MAX )
	freq = 0;
    return freq;
}

static bit_t setupChannel (u1_t chidx, u4_t freq, int drs) {
    if( chidx >= MAX_CHANNELS )
	return 0;
    if( freq >= 869400000 && freq <= 869650000 )
	freq |= BAND_DECI;  // 10% 27dBm
    else if( (freq >= 868000000 && freq <= 868600000) ||
	     (freq >= 869700000 && freq <= 870000000)  )
	freq |= BAND_CENTI;  // 1% 14dBm
    //else 
    //  freq |= BAND_MILLI;  // 0.1% 14dBm
    LMIC.channelFreq[chidx] = freq;
    LMIC.channelDrs [chidx] = drs < 0 ? (DR_SF12|(DR_SF7<<4)) : drs;
    return 1;
}

static u1_t mapChannels (u1_t chpage, u2_t chmap) {
    if( chpage != 0 || (chmap & ~((u2_t)(1<<MAX_CHANNELS)-1)) != 0 )
	return 0;  // illegal input
    for( u1_t chnl=0; chnl<MAX_CHANNELS; chnl++ ) {
	if( (chmap & (1<<chnl)) != 0 && LMIC.channelFreq[chnl] == 0 )
	    chmap &= ~(1<<chnl); // ignore - channel is not defined
    }
    LMIC.channelMap = chmap;
    return 1;
}


static void updateTx (ostime_t txbeg) {
    u4_t freq = LMIC.channelFreq[LMIC.txChnl];
    // Update global/band specific duty cycle stats
    ostime_t airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
    // Update channel/global duty cycle stats
    xref2band_t band = &LMIC.bands[freq & 0x3];
    LMIC.freq  = freq & ~(u4_t)3;
    LMIC.txpow = band->txpow;
    band->avail = txbeg + airtime * band->txcap;
    // Update obsolete avail's to prevent rollover
    // (needed for devices that send rarely - e.g. once/twice a day)
    for( u1_t b=0; b<MAX_BANDS; b++ ) {
	if( LMIC.bands[b].avail - txbeg < 0 )
	    LMIC.bands[b].avail = txbeg;
    }
    if( LMIC.globalDutyRate != 0 )
	LMIC.globalDutyAvail = txbeg + (airtime<<LMIC.globalDutyRate);
}

static ostime_t nextTx (ostime_t now) {
    // If we do not find a suitable channel stop sending (misconfigured device)
    ostime_t mintime = now + /*10h*/36000*OSTICKS_PER_SEC;
    s1_t     bmask   = 0;
    for( u1_t b=0; b<MAX_BANDS; b++ ) {
	xref2band_t band = &LMIC.bands[b];
	if( band->txcap == 0 ) // band not setup
	    continue;
	if( now - band->avail >= 0 ) {
	    bmask = (bmask < 0 ? 0 : bmask) | (1<<b);
	}
	else if( mintime - band->avail > 0 ) {
	    mintime = band->avail;
	}
    }
    if( bmask == 0 )
	return mintime;
    u1_t chnl, ccnt;
    u2_t cset, mask;
    chnl = cset = ccnt = 0;
    mask = 1;
    while( mask && mask <= LMIC.channelMap ) {
	// Channel is enabled AND and in a ready band and can handle the datarate
	if( (mask & LMIC.channelMap) != 0  &&  (bmask & (1 << (LMIC.channelFreq[chnl] & 0x3))) != 0) {
	    u1_t drs = LMIC.channelDrs[chnl];
	    if( !(isSlowerDR((dr_t)LMIC.datarate, (dr_t)(drs & 0xF)) ||   // lowest datarate
		  isFasterDR((dr_t)LMIC.datarate, (dr_t)(drs >>  4))) ) { // fastest datarate
		cset |= mask;
		ccnt++;
	    }
	}
	mask <<= 1;
	chnl++;
    }
    if( ccnt == 0 ) // No eligible channel - misconfigured device?
	return mintime;
    ccnt = os_getRndU2() % ccnt;
    mask = 1;
    chnl = 0;
    do {
	if( (cset & mask) != 0 ) {
	    if( ccnt==0 ) {
		LMIC.txChnl = chnl;
		return now;
	    }
	    ccnt--;
	}
	mask <<= 1;
	chnl++;
    } while(1);
}

static void setBcnRxParams () {
    LMIC.dataLen = 0;
    LMIC.freq = LMIC.channelFreq[LMIC.bcnChnl] & ~(u4_t)3;
    LMIC.rps  = setIh(setNocrc(dndr2rps((dr_t)DR_BCN),1),LEN_BCN);
}

#define setRx1Params() /*LMIC.freq/rps remain unchanged*/

static void initJoinLoop () {
    LMIC.txChnl = os_getRndU1() % 3;
    LMIC.adrTxPow = 14;
    setDrJoin(DRCHG_SET, DR_SF7);
    initDefaultChannels(1);
    ASSERT((LMIC.opmode & OP_NEXTCHNL)==0);
    LMIC.txend = LMIC.bands[BAND_MILLI].avail;
}


static ostime_t nextJoinState () {
    // Try 869.x and then 864.x with same DR
    // If both fail try next lower datarate
    LMIC.opmode &= ~OP_NEXTCHNL;
    LMIC.txend = LMIC.bands[BAND_MILLI].avail;
    if( LMIC.txChnl < 3 ) {
	LMIC.txChnl += 3;
    } else {
	if( LMIC.datarate == DR_SF12 ) {
	    setDrJoin(DRCHG_NOJACC, DR_SF7);
	    // We tried one round of FSK..SF12
	    // Now pick new random channel and insert a random delay and try another round.
	    LMIC.txChnl = os_getRndU1() % 3;
	    if( LMIC.txCnt < 5 )
		LMIC.txCnt += 1;
	    // Delay by 7,15,31,63,127,255 secs
	    return rndDelay((4<<LMIC.txCnt)-1) | 1;   // |1 - trigger EV_JOIN_FAILED event
	}
	LMIC.txChnl = LMIC.txChnl==5 ? 0 : LMIC.txChnl-2;
	setDrJoin(DRCHG_NOJACC, decDR((dr_t)LMIC.datarate));
    }
    // Avoid collision with JOIN ACCEPT being sent by GW (but we missed it)
    return 3*OSTICKS_PER_SEC;
}

//
// END: EU868 related stuff
//
// ================================================================================
#elif CFG_us915
// ================================================================================
//
// BEG: US915 related stuff
//

static void initDefaultChannels () {
    for( u1_t i=0; i<4; i++ )
	LMIC.channelMap[i] = 0xFFFF;
    LMIC.channelMap[4] = 0x00FF;
}

static u4_t convFreq (xref2u1_t ptr) {
    u4_t freq = (os_rlsbf4(ptr-1) >> 8) * 100;
    if( freq >= US915_FREQ_MIN && freq <= US915_FREQ_MAX )
	freq = 0;
    return freq;
}

static bit_t setupChannel (u1_t chidx, u4_t freq, int drs) {
    if( chidx < 72 || chidx >= 72+MAX_XCHANNELS )
	return 0; // channels 0..71 are hardwired
    chidx -= 72;
    LMIC.xchFreq[chidx] = freq;
    LMIC.xchDrs[chidx] = drs < 0 ? (DR_SF10|(DR_SF8C<<4)) : drs;
    LMIC.channelMap[chidx>>4] |= (1<<(chidx&0xF));
    return 1;
}


static u1_t mapChannels (u1_t chpage, u2_t chmap) {
    if( chpage == MCMD_LADR_CHP_125ON || chpage == MCMD_LADR_CHP_125OFF ) {
	u2_t en125 = chpage == MCMD_LADR_CHP_125ON ? 0xFFFF : 0x0000;
	for( u1_t u=0; u<4; u++ )
	    LMIC.channelMap[u] = en125;
	LMIC.channelMap[64/16] = chmap;
    } else {
	if( chpage >= (72+MAX_XCHANNELS+15)/16 )
	    return 0;
	LMIC.channelMap[chpage] = chmap;
    }
    return 1;
}

static void updateTx (ostime_t txbeg) {
    u1_t chnl = LMIC.txChnl;
    if( chnl < 64 ) {
	LMIC.freq = US915_125kHz_UPFBASE + chnl*US915_125kHz_UPFSTEP;
	LMIC.txpow = 30;
	return;
    }
    LMIC.txpow = 26;
    if( chnl < 64+8 ) {
	LMIC.freq = US915_500kHz_UPFBASE + (chnl-64)*US915_500kHz_UPFSTEP;
    } else {
	ASSERT(chnl < 64+8+MAX_XCHANNELS);
	LMIC.freq = LMIC.xchFreq[chnl-72];
    }

    // Update global duty cycle stats
    if( LMIC.globalDutyRate != 0 ) {
	ostime_t airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
	LMIC.globalDutyAvail = txbeg + (airtime<<LMIC.globalDutyRate);
    }
}

// US does not have duty cycling - return now as earliest TX time
#define nextTx(now) (_nextTx(),(now))
static void _nextTx () {
    u1_t  chnl  = LMIC.txChnl;
    u1_t  cnt   = 0;
    bit_t bw500 = (LMIC.datarate >= DR_SF8C);

    if( (LMIC.chRnd & 0xF) == 0 ) {
	chnl += LMIC.chRnd>>4;
	LMIC.chRnd = os_getRndU1();
    }
    LMIC.chRnd--;
  again:
    chnl += 1;
    if( bw500 ) {
	// Only channels 64..71
	chnl = 64+(chnl&0x7);
	// At least one 500kHz channel must be enabled
	// - otherwise MAC should not have used such a datarate
	ASSERT((LMIC.channelMap[64/16]&0xFF)!=0x00);
	if( ++cnt == 8 )
	    return; // no appropriate channel enabled stay where we are
    } else {
	// At least one 125kHz channel must be enabled
	// - otherwise MAC should not have used DR_SF10-7
	ASSERT((LMIC.channelMap[0]|LMIC.channelMap[1]|LMIC.channelMap[2]|LMIC.channelMap[3])!=0x0000);
	chnl = chnl&0x3F;
	if( ++cnt == 64 )
	    return; // no appropriate channel enabled stay where we are
    }
    if( (LMIC.channelMap[(chnl >> 4)] & (1<<(chnl&0xF))) == 0 )
	goto again;  // not enabled
    LMIC.txChnl = chnl;
}

static void setBcnRxParams () {
    LMIC.dataLen = 0;
    LMIC.freq = US915_500kHz_DNFBASE + LMIC.bcnChnl*US915_500kHz_DNFSTEP;
    LMIC.rps  = setIh(setNocrc(dndr2rps((dr_t)DR_BCN),1),LEN_BCN);
}

#define setRx1Params() { 						\
    LMIC.freq = US915_500kHz_DNFBASE + (LMIC.txChnl & 0x7) * US915_500kHz_DNFSTEP; \
    if( /* TX datarate */LMIC.rxsyms < DR_SF8C )				\
	LMIC.rxsyms += DR_SF10CR - DR_SF10;				\
    else if( LMIC.rxsyms == DR_SF8C )					\
	LMIC.rxsyms = DR_SF7CR;						\
    LMIC.rps = dndr2rps(LMIC.rxsyms);					\
}

static void initJoinLoop () {
    LMIC.txChnl = os_getRndU1() & 0x3F;
    LMIC.adrTxPow = 20;
    ASSERT((LMIC.opmode & OP_NEXTCHNL)==0);
    LMIC.txend = os_getTime();
    setDrJoin(DRCHG_SET, DR_SF7);
}

static ostime_t nextJoinState () {
    // Try the following:
    //   SF7/8/9/10  on a random channel 0..63
    //   SF8C        on a random channel 64..71
    //
    LMIC.opmode &= ~OP_NEXTCHNL;
    LMIC.txend = os_getTime();
    if( LMIC.datarate != DR_SF8C ) {
	LMIC.txChnl = 64+(LMIC.txChnl&7);
	LMIC.datarate = DR_SF8C;
    } else {
	LMIC.txChnl = os_getRndU1() & 0x3F;
	LMIC.datarate = DR_SF7 - (LMIC.txCnt++ & 3);
	if( (LMIC.txCnt & 3) == 0 ) {
	    // Tried SF10/SF8C
	    // Delay by 7,15,31,63,127,255 secs
	    return rndDelay((4<<(LMIC.txCnt>>2))-1) | 1;   // |1 - trigger EV_JOIN_FAILED event
	}
    }
    // Always wait 500ms in case there was a TX and we just missed it.
    // Sending immediately would not be picked up because GW is still busy txing missed frame.
    return (OSTICKS_PER_SEC/2) & ~1;
}

//
// END: US915 related stuff
//
// ================================================================================
#else
#error Unsupported frequency band!
#endif


static void runEngineUpdate (xref2osjob_t osjob) {
    engineUpdate();
}


static void reportEvent (ev_t ev) {
    EV(devCond, INFO, (e_.reason = EV::devCond_t::LMIC_EV,
		       e_.eui    = MAIN::CDEV->getEui(),
		       e_.info   = ev));
    ON_LMIC_EVENT(ev);
    engineUpdate();
}


static void stateJustJoined () {
    LMIC.seqnoDn     = LMIC.seqnoUp = 0;
    LMIC.rejoinCnt   = 0;
    LMIC.dnConf      = LMIC.adrChanged = LMIC.ladrAns = LMIC.devsAns = 0;
    LMIC.moreData    = LMIC.dn2Ans = LMIC.snchAns = LMIC.dutyCapAns = 0;
    LMIC.pingSetAns  = 0;
    LMIC.upRepeat    = 0;
    LMIC.adrAckReq   = LINK_CHECK_INIT;
    LMIC.dn2Dr       = DR_DNW2;
    LMIC.dn2Freq     = FREQ_DNW2;
    LMIC.bcnChnl     = CHNL_BCN;
    LMIC.ping.freq   = FREQ_PING;
    LMIC.ping.dr     = DR_PING;
}


// ================================================================================
// Decoding frames


// Decode beacon  - do not overwrite bcninfo unless we have a match!
static int decodeBeacon () {
    ASSERT(LMIC.dataLen == LEN_BCN); // implicit header RX guarantees this
    xref2u1_t d = LMIC.frame;
    if( os_rlsbf2(&d[OFF_BCN_CRC1]) != os_crc16(d, OFF_BCN_CRC1) )
	return 0;   // first (common) part fails CRC check
    // First set of fields is ok
    u4_t bcnnetid = os_rlsbf4(&d[OFF_BCN_NETID]) & 0xFFFFFF;
    if( bcnnetid != LMIC.netid )
	return -1;  // not the beacon we're looking for

    LMIC.bcninfo.flags &= ~(BCN_PARTIAL|BCN_FULL);
    // Match - update bcninfo structure
    os_copyMem((xref2u1_t)&LMIC.bcninfo.rxq, (xref2u1_t)&LMIC.rxq, SIZEOFEXPR(LMIC.rxq));
    LMIC.bcninfo.txtime = LMIC.rxtime - AIRTIME_BCN_osticks;
    LMIC.bcninfo.time   = os_rlsbf4(&d[OFF_BCN_TIME]);
    LMIC.bcninfo.flags |= BCN_PARTIAL;

    // Check 2nd set
    if( os_rlsbf2(&d[OFF_BCN_CRC2]) != os_crc16(d,OFF_BCN_CRC2) )
	return 1;
    // Second set of fields is ok
    LMIC.bcninfo.lat    = (s4_t)os_rlsbf4(&d[OFF_BCN_LAT-1]) >> 8; // read as signed 24-bit
    LMIC.bcninfo.lon    = (s4_t)os_rlsbf4(&d[OFF_BCN_LON-1]) >> 8; // ditto
    LMIC.bcninfo.info   = d[OFF_BCN_INFO];
    LMIC.bcninfo.flags |= BCN_FULL;
    return 2;
}


static bit_t decodeFrame () {
    xref2u1_t d = LMIC.frame;
    u1_t hdr    = d[0];
    u1_t ftype  = hdr & HDR_FTYPE;
    int  dlen   = LMIC.dataLen;
    if( dlen < OFF_DAT_OPTS+4 ||
	(hdr & HDR_MAJOR) != HDR_MAJOR_V1 ||
	(ftype != HDR_FTYPE_DADN  &&  ftype != HDR_FTYPE_DCDN) ) {
	// Basic sanity checks failed
	EV(specCond, WARN, (e_.reason = EV::specCond_t::UNEXPECTED_FRAME,
			    e_.eui    = MAIN::CDEV->getEui(),
			    e_.info   = dlen < 4 ? 0 : os_rlsbf4(&d[dlen-4]),
			    e_.info2  = hdr + (dlen<<8)));
      norx:
	LMIC.dataLen = 0;
	return 0;
    }
    // Validate exact frame length
    // Note: device address was already read+evaluated in order to arrive here.
    int  fct   = d[OFF_DAT_FCT];
    u4_t addr  = os_rlsbf4(&d[OFF_DAT_ADDR]);
    u4_t seqno = os_rlsbf2(&d[OFF_DAT_SEQNO]);
    int  olen  = fct & FCT_OPTLEN;
    int  ackup = (fct & FCT_ACK) != 0 ? 1 : 0;   // ACK last up frame
    int  poff  = OFF_DAT_OPTS+olen;
    int  pend  = dlen-4;  // MIC

    if( addr != LMIC.devaddr ) {
	EV(specCond, WARN, (e_.reason = EV::specCond_t::ALIEN_ADDRESS,
			    e_.eui    = MAIN::CDEV->getEui(),
			    e_.info   = addr,
			    e_.info2  = LMIC.devaddr));
	goto norx;
    }
    if( poff > pend ) {
	EV(specCond, ERR, (e_.reason = EV::specCond_t::CORRUPTED_FRAME,
			   e_.eui    = MAIN::CDEV->getEui(),
			   e_.info   = 0x1000000 + (poff-pend) + (fct<<8) + (dlen<<16)));
	goto norx;
    }

    int port = -1;
    int replayConf = 0;

    if( pend > poff )
	port = d[poff++];

    seqno = LMIC.seqnoDn + (s2_t)(seqno - LMIC.seqnoDn);

    if( !aes_verifyMic(LMIC.nwkKey, LMIC.devaddr, seqno, /*dn*/1, d, pend) ) {
	EV(specCond, ERR, (e_.reason = EV::specCond_t::CORRUPTED_MIC,
			   e_.eui    = MAIN::CDEV->getEui(),
			   e_.info   = Base::lsbf4(&d[pend]),
			   e_.info2  = seqno));
	goto norx;
    }
    if( seqno < LMIC.seqnoDn ) {
	if( (s4_t)seqno > (s4_t)LMIC.seqnoDn ) {
	    EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_ROLL_OVER,
				e_.eui    = MAIN::CDEV->getEui(),
				e_.info   = LMIC.seqnoDn, 
				e_.info2  = seqno));
	    goto norx;
	}
	if( seqno != LMIC.seqnoDn-1 || !LMIC.dnConf || ftype != HDR_FTYPE_DCDN ) {
	    EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_OBSOLETE,
				e_.eui    = MAIN::CDEV->getEui(),
				e_.info   = LMIC.seqnoDn, 
				e_.info2  = seqno));
	    goto norx;
	}
	// Replay of previous sequence number allowed only if
	// previous frame and repeated both requested confirmation
	replayConf = 1;
    }
    else {
	if( seqno > LMIC.seqnoDn ) {
	    EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_SKIP,
				e_.eui    = MAIN::CDEV->getEui(),
				e_.info   = LMIC.seqnoDn, 
				e_.info2  = seqno));
	}
	LMIC.seqnoDn = seqno+1;  // next number to be expected
	DO_DEVDB(updateSeqnoDn, LMIC.seqnoDn);
	// DN frame requested confirmation - provide ACK once with next UP frame
	LMIC.dnConf = (ftype == HDR_FTYPE_DCDN ? FCT_ACK : 0);
    }

    if( LMIC.dnConf || (fct & FCT_MORE) )
	LMIC.opmode |= OP_POLL;

    // We heard from network
    LMIC.adrChanged = 0;
    LMIC.adrAckReq = LINK_CHECK_INIT;

    // Process OPTS
    int m = LMIC.rxq.rssi - RSSI_OFF - getSensitivity(LMIC.rps);
    LMIC.margin = m < 0 ? 0 : m > 254 ? 254 : m;

    xref2u1_t opts = &d[OFF_DAT_OPTS];
    int oidx = 0;
    while( oidx < olen ) {
	switch( opts[oidx] ) {
	case MCMD_LCHK_ANS: {
	    //int gwmargin = opts[oidx+1];
	    //int ngws = opts[oidx+2];
	    oidx += 3;
	    continue;
	}
	case MCMD_LADR_REQ: {
	    u1_t p1     = opts[oidx+1];            // txpow + DR
	    u2_t chmap  = os_rlsbf2(&opts[oidx+2]);// list of enabled channel
	    u1_t chpage = opts[oidx+4] & MCMD_LADR_CHPAGE_MASK;     // channel page
	    u1_t uprpt  = opts[oidx+4] & MCMD_LADR_REPEAT_MASK;     // up repeat count
	    oidx += 5;

	    LMIC.ladrAns = 0x80 |     // Include an answer into next frame up
		MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK;
	    if( !mapChannels(chpage, chmap) )
		LMIC.ladrAns &= ~MCMD_LADR_ANS_CHACK;
	    dr_t dr = (dr_t)(p1>>MCMD_LADR_DR_SHIFT);
	    if( !validDR(dr) ) {
		LMIC.ladrAns &= ~MCMD_LADR_ANS_DRACK;
		dr = (dr_t)LMIC.datarate;
		EV(specCond, ERR, (e_.reason = EV::specCond_t::BAD_MAC_CMD,
				   e_.eui    = MAIN::CDEV->getEui(),
				   e_.info   = Base::lsbf4(&d[pend]),
				   e_.info2  = Base::msbf4(&opts[oidx-4])));
	    }
	    LMIC.upRepeat = uprpt;
	    setDrTxpow(DRCHG_NWKCMD, dr, pow2dBm(p1));
	    LMIC.adrChanged = 1;  // Trigger an ACK to NWK
	    continue;
	}
	case MCMD_DEVS_REQ: {
	    LMIC.devsAns = 1;
	    oidx += 1;
	    continue;
	}
	case MCMD_DN2P_SET: {
	    dr_t dr = (dr_t)(opts[oidx+1] & 0x0F);
	    u4_t freq = convFreq(&opts[oidx+2]);
	    oidx += 5;
	    LMIC.dn2Ans = 0x80;   // answer pending
	    if( validDR(dr) )
		LMIC.dn2Ans |= MCMD_DN2P_ANS_DRACK;
	    if( freq != 0 )
		LMIC.dn2Ans |= MCMD_DN2P_ANS_CHACK;
	    if( LMIC.dn2Ans == (0x80|MCMD_DN2P_ANS_DRACK|MCMD_DN2P_ANS_CHACK) ) {
		LMIC.dn2Dr = dr;
		LMIC.dn2Freq = freq;
		DO_DEVDB(updateDn2,LMIC.dn2Dr,LMIC.dn2Freq);
	    }
	    continue;
	}
	case MCMD_DCAP_REQ: {
	    u1_t cap = opts[oidx+1];
	    oidx += 2;
	    // A value cap=0xFF means device is OFF unless enabled again manually
	    // We just set duty cap to 0xF which is 0.003% -- pretty much off.
	    // We don't check 0xF0 bits if cap!=0xFF
	    LMIC.globalDutyRate  = cap & 0xF;
	    LMIC.globalDutyAvail = os_getTime();
	    DO_DEVDB(updateDutyCap,cap);
	    LMIC.dutyCapAns = 1;
	    continue;
	}
	case MCMD_SNCH_REQ: {
	    u1_t chidx = opts[oidx+1];  // channel
	    u4_t freq  = convFreq(&opts[oidx+2]); // freq
	    u1_t drs   = opts[oidx+5];  // datarate span
	    LMIC.snchAns = 0x80;
	    if( freq != 0 && setupChannel(chidx,freq,drs) )
		LMIC.snchAns |= MCMD_SNCH_ANS_DRACK|MCMD_SNCH_ANS_FQACK;
	    oidx += 6;
	}
	case MCMD_PING_SET: {
	    u4_t freq = convFreq(&opts[oidx+1]);
	    oidx += 4;
	    u1_t flags = 0x80;
	    if( freq != 0 ) {
		flags |= MCMD_PING_ANS_FQACK;
		LMIC.ping.freq = freq;
		DO_DEVDB(updateClassB,LMIC.ping.intvExp,freq,LMIC.ping.dr);
	    }
	    LMIC.pingSetAns = flags;
	    continue;
	}
	case MCMD_BCNI_ANS: {
	    // Ignore if tracking already enabled
	    if( (LMIC.opmode & OP_TRACK) == 0 ) {
		LMIC.bcnChnl = opts[oidx+3];
		// Enable tracking - bcninfoTries
		LMIC.opmode |= OP_TRACK;
		// Cleared later in txComplete handling - triggers EV_BEACON_FOUND
		ASSERT(LMIC.bcninfoTries!=0);
		// Setup RX parameters
		LMIC.bcninfo.txtime =  (LMIC.rxtime
				       - calcAirTime(LMIC.rps, LMIC.dataLen)
				       + ms2osticks(os_rlsbf2(&opts[oidx+1]) * 10)
				       + ms2osticksCeil(5)
				       - BCN_INTV_osticks);
		LMIC.bcninfo.flags = 0;  // txtime above cannot be used as reference (BCN_PARTIAL|BCN_FULL cleared)
		calcBcnRxWindowFromMillis(10,1);  // error of +/-5 ms 

		EV(lostFrame, INFO, (e_.reason  = EV::lostFrame_t::MCMD_BCNI_ANS,
				     e_.eui     = MAIN::CDEV->getEui(),
				     e_.lostmic = Base::lsbf4(&d[pend]),
				     e_.info    = (LMIC.missedBcns |
						   (osticks2us(LMIC.bcninfo.txtime + BCN_INTV_osticks
							       - LMIC.bcnRxtime) << 8)),
				     e_.time    = MAIN::CDEV->ostime2ustime(LMIC.bcninfo.txtime + BCN_INTV_osticks)));
	    }
	    oidx += 4;
	    continue;
	}
	}
	EV(specCond, ERR, (e_.reason = EV::specCond_t::BAD_MAC_CMD,
			   e_.eui    = MAIN::CDEV->getEui(),
			   e_.info   = Base::lsbf4(&d[pend]),
			   e_.info2  = Base::msbf4(&opts[oidx])));
	break;
    }
    if( oidx != olen ) {
	EV(specCond, ERR, (e_.reason = EV::specCond_t::CORRUPTED_FRAME,
			   e_.eui    = MAIN::CDEV->getEui(),
			   e_.info   = 0x1000000 + (oidx) + (olen<<8)));
    }

    if( !replayConf ) {
	// Handle payload only if not a replay
	// Decrypt payload - if any
	if( port >= 0  &&  pend-poff > 0 )
	    aes_cipher(port <= 0 ? LMIC.nwkKey : LMIC.artKey, LMIC.devaddr, seqno, /*dn*/1, d+poff, pend-poff);

	EV(dfinfo, DEBUG, (e_.deveui  = MAIN::CDEV->getEui(),
			   e_.devaddr = LMIC.devaddr,
			   e_.seqno   = seqno,
			   e_.flags   = (port < 0 ? EV::dfinfo_t::NOPORT : 0) | EV::dfinfo_t::DN,
			   e_.mic     = Base::lsbf4(&d[pend]),
			   e_.hdr     = d[LORA::OFF_DAT_HDR],
			   e_.fct     = d[LORA::OFF_DAT_FCT],
			   e_.port    = port,
			   e_.plen    = dlen,
			   e_.olen    = olen,
			   memcpy(e_.opts, opts, olen)));
    } else {
	EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_REPLAY,
			    e_.eui    = MAIN::CDEV->getEui(),
			    e_.info   = Base::lsbf4(&d[pend]),
			    e_.info2  = seqno));
    }

    if( // NWK acks but we don't have a frame pending
	(ackup && LMIC.txCnt == 0) ||
	// We sent up confirmed and we got a response in DNW1/DNW2
	// BUT it did not carry an ACK - this should never happen
	// Do not resend and assume frame was not ACKed.
	(!ackup && LMIC.txCnt != 0) ) {
	EV(specCond, ERR, (e_.reason = EV::specCond_t::SPURIOUS_ACK,
			   e_.eui    = MAIN::CDEV->getEui(),
			   e_.info   = seqno,
			   e_.info2  = ackup));
    }

    if( LMIC.txCnt != 0 ) // we requested an ACK
	LMIC.txrxFlags |= ackup ? TXRX_ACK : TXRX_NACK;

    if( port < 0 ) {
	LMIC.txrxFlags |= TXRX_NOPORT;
	LMIC.dataBeg = LMIC.dataLen = 0;
    } else {
	LMIC.dataBeg = poff;
	LMIC.dataLen = pend-poff;
    }
    return 1;
}


// ================================================================================
// TX/RX transaction support


static void setupRx2 () {
    LMIC.txrxFlags = TXRX_DNW2;
    LMIC.rps = dndr2rps(LMIC.dn2Dr);
    LMIC.freq = LMIC.dn2Freq;
    LMIC.dataLen = 0;
    os_radio(RADIO_RX);
}


static void schedRx2 (ostime_t delay, osjobcb_t func) {
    // Add 1.5 symbols we need 5 out of 8. Try to sync 1.5 symbols into the preamble.
    LMIC.rxtime = LMIC.txend + delay + (PAMBL_SYMS-MINRX_SYMS)*dr2hsym(LMIC.dn2Dr);
    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, func);
}

static void setupRx1 (osjobcb_t func) {
    LMIC.txrxFlags = TXRX_DNW1;
    // Turn LMIC.rps from TX over to RX
    LMIC.rps = setNocrc(LMIC.rps,1);
    LMIC.dataLen = 0;
    LMIC.osjob.func = func;
    os_radio(RADIO_RX);
}


// Called by HAL once TX complete and delivers exact end of TX time stamp in LMIC.rxtime
static void txDone (ostime_t delay, osjobcb_t func) {
    // NOTE: LMIC.rxsyms carries the modified DR for TX (LMIC.datarate is the base DR - [CONFIRM mode])
    u1_t dr = LMIC.rxsyms;    // rxschedInit - would overwrite rxsyms
    if( (LMIC.opmode & (OP_TRACK|OP_PINGABLE|OP_PINGINI)) == (OP_TRACK|OP_PINGABLE) ) {
	rxschedInit(&LMIC.ping);    // note: reuses LMIC.frame buffer!
	LMIC.opmode |= OP_PINGINI;
    }
    // Change RX frequency / rps (US only) before we increment txChnl
    setRx1Params();
    // LMIC.rxsyms carries the TX datarate (can be != LMIC.datarate [confirm retries etc.])
    // Setup receive - LMIC.rxtime is preloaded with 1.5 symbols offset to tune
    // into the middle of the 8 symbols preamble.
#if CFG_eu868
    if( /* TX datarate */LMIC.rxsyms == DR_FSK ) {
	LMIC.rxtime = LMIC.txend + delay - PRERX_FSK*us2osticksRound(160);
	LMIC.rxsyms = RXLEN_FSK;
    }
    else
#endif
    {
	LMIC.rxtime = LMIC.txend + delay + (PAMBL_SYMS-MINRX_SYMS)*dr2hsym(dr);
	LMIC.rxsyms = MINRX_SYMS;
    }
    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, func);
}


// ======================================== Join frames


static void onJoinFailed (xref2osjob_t osjob) {
    // Notify app - must call LMIC_reset() to stop joining
    // otherwise join procedure continues.
    reportEvent(EV_JOIN_FAILED);
}


static bit_t processJoinAccept () {
    ASSERT(LMIC.txrxFlags != TXRX_DNW1 || LMIC.dataLen != 0);
    ASSERT((LMIC.opmode & OP_TXRXPEND)!=0);

    if( LMIC.dataLen == 0 ) {
      nojoinframe:
	if( (LMIC.opmode & OP_JOINING) == 0 ) {
	    ASSERT((LMIC.opmode & OP_REJOIN) != 0);
	    // REJOIN attempt for roaming
	    LMIC.opmode &= ~(OP_REJOIN|OP_TXRXPEND);
	    if( LMIC.rejoinCnt < 10 )
		LMIC.rejoinCnt++;
	    reportEvent(EV_REJOIN_FAILED);
	    return 1;
	}
	LMIC.opmode &= ~OP_TXRXPEND;
	ostime_t delay = nextJoinState();
	EV(devCond, DEBUG, (e_.reason = EV::devCond_t::NO_JACC,
			    e_.eui    = MAIN::CDEV->getEui(),
			    e_.info   = LMIC.datarate|DR_PAGE,
			    e_.info2  = osticks2ms(delay)));
	// Build next JOIN REQUEST with next engineUpdate call
	// Optionally, report join failed.
	// Both after a random/chosen amount of ticks.
	os_setTimedCallback(&LMIC.osjob, os_getTime()+delay,
			    (delay&1) != 0
			    ? FUNC_ADDR(onJoinFailed)      // one JOIN iteration done and failed
			    : FUNC_ADDR(runEngineUpdate)); // next step to be delayed
	return 1;
    }
    u1_t hdr  = LMIC.frame[0];
    u1_t dlen = LMIC.dataLen;
    u4_t mic  = os_rlsbf4(&LMIC.frame[dlen-4]); // safe before modified by encrypt!
    if( (dlen != LEN_JA && dlen != LEN_JAEXT)
	|| (hdr & (HDR_FTYPE|HDR_MAJOR)) != (HDR_FTYPE_JACC|HDR_MAJOR_V1) ) {
	EV(specCond, ERR, (e_.reason = EV::specCond_t::UNEXPECTED_FRAME,
			   e_.eui    = MAIN::CDEV->getEui(),
			   e_.info   = dlen < 4 ? 0 : mic,
			   e_.info2  = hdr + (dlen<<8)));
      badframe:
	goto nojoinframe;
    }
    aes_encrypt(LMIC.frame+1, dlen-1);
    if( !aes_verifyMic0(LMIC.frame, dlen-4) ) {
	EV(specCond, ERR, (e_.reason = EV::specCond_t::JOIN_BAD_MIC,
			   e_.info   = mic));
	goto badframe;
    }

    u4_t addr = os_rlsbf4(LMIC.frame+OFF_JA_DEVADDR);
    LMIC.devaddr = addr;
    LMIC.netid = os_rlsbf4(&LMIC.frame[OFF_BCN_NETID-1]) >> 8;

#if CFG_eu868
    initDefaultChannels(0);
#endif
    if( dlen > LEN_JA ) {
	dlen = OFF_CFLIST;
#if CFG_eu868
	u1_t chidx=3;
#elif CFG_us915
	u1_t chidx=72;
#endif
	for( ; chidx<8; chidx++, dlen+=3 )
	    setupChannel(chidx, os_rlsbf4(&LMIC.frame[dlen-1]) >> 8, -1);
    }

    // already incremented when JOIN REQ got sent off
    aes_sessKeys(LMIC.devNonce-1, &LMIC.frame[OFF_JA_ARTNONCE], LMIC.nwkKey, LMIC.artKey);
    DO_DEVDB(updateJoinAcc, LMIC.devaddr, LMIC.nwkKey, LMIC.artKey);

    EV(joininfo, INFO, (e_.arteui  = MAIN::CDEV->getArtEui(),
			e_.deveui  = MAIN::CDEV->getEui(),
			e_.devaddr = LMIC.devaddr,
			e_.oldaddr = oldaddr,
			e_.nonce   = LMIC.devNonce-1,
			e_.mic     = mic,
			e_.flags   = ((LMIC.opmode & OP_REJOIN) != 0
				      ? EV::joininfo_t::REJOIN_ACCEPT
				      : EV::joininfo_t::ACCEPT)));
    
    ASSERT((LMIC.opmode & (OP_JOINING|OP_REJOIN))!=0);
    if( (LMIC.opmode & OP_REJOIN) != 0 ) {
	LMIC.datarate = lowerDR(LMIC.datarate, LMIC.rejoinCnt);
    }
    LMIC.opmode &= ~(OP_JOINING|OP_TRACK|OP_REJOIN|OP_TXRXPEND|OP_PINGINI) | OP_NEXTCHNL;
    stateJustJoined();
    reportEvent(EV_JOINED);
    return 1;
}

static void processRx1Jacc (xref2osjob_t osjob) {
    if( LMIC.dataLen == 0 )
	LMIC.txrxFlags = 0;  // nothing in 1st/2nd DN slot
    processJoinAccept();
}

static void setupRx1Jacc (xref2osjob_t osjob) {
    setupRx1(FUNC_ADDR(processRx1Jacc));
}


static void jreqDone (xref2osjob_t osjob) {
    txDone(DELAY_JACC1_osticks, FUNC_ADDR(setupRx1Jacc));
}

// ======================================== Data frames

// Fwd decl.
static bit_t processDnData();

static void processRx2DnData (xref2osjob_t osjob) {
    if( LMIC.dataLen == 0 )
	LMIC.txrxFlags = 0;  // nothing in 1st/2nd DN slot
    processDnData();
}


static void setupRx2DnData (xref2osjob_t osjob) {
    LMIC.osjob.func = FUNC_ADDR(processRx2DnData);
    setupRx2();
}


static void processRx1DnData (xref2osjob_t osjob) {
    if( LMIC.dataLen == 0 || !processDnData() )
	schedRx2(DELAY_DNW2_osticks, FUNC_ADDR(setupRx2DnData));
}


static void setupRx1DnData (xref2osjob_t osjob) {
    setupRx1(FUNC_ADDR(processRx1DnData));
}


static void updataDone (xref2osjob_t osjob) {
    txDone(DELAY_DNW1_osticks, FUNC_ADDR(setupRx1DnData));
}

// ======================================== 


static void buildDataFrame () {
    bit_t txdata = ((LMIC.opmode & (OP_TXDATA|OP_POLL)) != OP_POLL);
    u1_t dlen = txdata ? LMIC.pendTxLen : 0;

    // Piggyback MAC options
    // Prioritize by importance
    int  end = OFF_DAT_OPTS;
    if( (LMIC.opmode & (OP_TRACK|OP_PINGABLE)) == (OP_TRACK|OP_PINGABLE) ) {
	// Indicate pingability in every UP frame
	LMIC.frame[end] = MCMD_PING_IND;
	LMIC.frame[end+1] = LMIC.ping.dr | (LMIC.ping.intvExp<<4);
	end += 2;
    }
    if( LMIC.dutyCapAns ) {
	LMIC.frame[end] = MCMD_DCAP_ANS;
	end += 1;
	LMIC.dutyCapAns = 0;
    }
    if( LMIC.dn2Ans ) {
	LMIC.frame[end+0] = MCMD_DN2P_ANS;
	LMIC.frame[end+1] = LMIC.dn2Ans & ~MCMD_DN2P_ANS_RFU;
	end += 2;
	LMIC.dn2Ans = 0;
    }
    if( LMIC.devsAns ) {  // answer to device status
	LMIC.frame[end+0] = MCMD_DEVS_ANS;
	LMIC.frame[end+1] = LMIC.margin;
	LMIC.frame[end+2] = os_getBattLevel();
	end += 3;
	LMIC.devsAns = 0;
    }
    if( LMIC.ladrAns ) {  // answer to ADR change
	LMIC.frame[end+0] = MCMD_LADR_ANS;
	LMIC.frame[end+1] = LMIC.ladrAns & ~MCMD_LADR_ANS_RFU;
	end += 2;
	LMIC.ladrAns = 0;
    }
    if( LMIC.bcninfoTries > 0 ) {
	LMIC.frame[end] = MCMD_BCNI_REQ;
	end += 1;
    }
    if( LMIC.adrChanged ) {
	LMIC.adrAckReq = LMIC.adrAckReq < 0 ? 0 : LMIC.adrAckReq;
	LMIC.adrChanged = 0;
    }
    if( LMIC.pingSetAns != 0 ) {
	LMIC.frame[end+0] = MCMD_PING_ANS;
	LMIC.frame[end+1] = LMIC.pingSetAns & ~MCMD_PING_ANS_RFU;
	end += 2;
	LMIC.pingSetAns = 0;
    }
    if( LMIC.snchAns ) {
	LMIC.frame[end+0] = MCMD_SNCH_ANS;
	LMIC.frame[end+1] = LMIC.snchAns;
	end += 2;
	LMIC.snchAns = 0;
    }
    ASSERT(end <= OFF_DAT_OPTS+16);

    u1_t flen = end + (txdata ? 5+dlen : 4);
    if( flen > MAX_LEN_FRAME ) {
	// Options and payload too big - delay payload
	txdata = 0;
	flen = end+4;
    }
    LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DAUP | HDR_MAJOR_V1;
    LMIC.frame[OFF_DAT_FCT] = (LMIC.dnConf | LMIC.adrEnabled
			      | (LMIC.adrAckReq >= 0 ? FCT_ADRARQ : 0)
			      | (end-OFF_DAT_OPTS));
    os_wlsbf4(LMIC.frame+OFF_DAT_ADDR,  LMIC.devaddr);

    if( LMIC.txCnt == 0 ) {
	LMIC.seqnoUp += 1;
	DO_DEVDB(updateSeqnoUp, LMIC.seqnoUp);
    } else {
	EV(devCond, INFO, (e_.reason = EV::devCond_t::RE_TX,
			   e_.eui    = MAIN::CDEV->getEui(),
			   e_.info   = LMIC.seqnoUp-1,
			   e_.info2  = (LMIC.txCnt+1) | (DRADJUST[LMIC.txCnt+1] << 8) | ((LMIC.datarate|DR_PAGE)<<16)));
    }
    os_wlsbf2(LMIC.frame+OFF_DAT_SEQNO, LMIC.seqnoUp-1);

    // Clear pending DN confirmation
    LMIC.dnConf = 0;

    if( txdata ) {
	if( LMIC.pendTxConf ) {
	    // Confirmed only makes sense if we have a payload (or at least a port)
	    LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DCUP | HDR_MAJOR_V1;
	    LMIC.txCnt += 1;
	}
	LMIC.frame[end] = LMIC.pendTxPort;
	os_copyMem(LMIC.frame+end+1, LMIC.pendTxData, dlen);
	aes_cipher(LMIC.pendTxPort==0 ? LMIC.nwkKey : LMIC.artKey,
		   LMIC.devaddr, LMIC.seqnoUp-1,
		   /*up*/0, LMIC.frame+end+1, dlen);
    }
    aes_appendMic(LMIC.nwkKey, LMIC.devaddr, LMIC.seqnoUp-1, /*up*/0, LMIC.frame, flen-4);

    EV(dfinfo, DEBUG, (e_.deveui  = MAIN::CDEV->getEui(),
		       e_.devaddr = LMIC.devaddr,
		       e_.seqno   = LMIC.seqnoUp-1,
		       e_.flags   = (LMIC.pendTxPort < 0 ? EV::dfinfo_t::NOPORT : EV::dfinfo_t::NOP),
		       e_.mic     = Base::lsbf4(&LMIC.frame[flen-4]),
		       e_.hdr     = LMIC.frame[LORA::OFF_DAT_HDR],
		       e_.fct     = LMIC.frame[LORA::OFF_DAT_FCT],
		       e_.port    = LMIC.pendTxPort,
		       e_.plen    = txdata ? dlen : 0,
		       e_.olen    = end-LORA::OFF_DAT_OPTS,
		       memcpy(e_.opts, LMIC.frame+LORA::OFF_DAT_OPTS, end-LORA::OFF_DAT_OPTS)));
    LMIC.dataLen = flen;
}


// Callback from HAL during scan mode or when job timer expires.
static void onBcnRx (xref2osjob_t job) {
    // If we arrive via job timer make sure to put radio to rest.
    os_radio(RADIO_RST);
    os_clearCallback(&LMIC.osjob);
    if( LMIC.dataLen == 0 ) {
	// Nothing received - timeout
	LMIC.opmode &= ~(OP_SCAN | OP_TRACK);
	reportEvent(EV_SCAN_TIMEOUT);
	return;
    }
    if( decodeBeacon() <= 0 ) {
	// Something is wrong with the beacon - continue scan
	LMIC.dataLen = 0;
	os_radio(RADIO_RXON);
	os_setTimedCallback(&LMIC.osjob, LMIC.bcninfo.txtime, FUNC_ADDR(onBcnRx));
	return;
    }
    // Found our 1st beacon
    // We don't have a previous beacon to calc some drift - assume
    // an max error of 13ms = 128sec*100ppm which is roughly +/-100ppm
    calcBcnRxWindowFromMillis(13,1);
    LMIC.opmode &= ~OP_SCAN;          // turn SCAN off
    LMIC.opmode |=  OP_TRACK;         // auto enable tracking
    reportEvent(EV_BEACON_FOUND);    // can be disabled in callback
}


// Enable receiver to listen to incoming beacons
// netid defines when scan stops (any or specific beacon)
// This mode ends with events: EV_SCAN_TIMEOUT/EV_SCAN_BEACON
// Implicitely cancels any pending TX/RX transaction.
// Also cancels an onpoing joining procedure.
static void startScan () {
    ASSERT(LMIC.devaddr!=0 && (LMIC.opmode & OP_JOINING)==0);
    if( (LMIC.opmode & OP_SHUTDOWN) != 0 )
	return;
    // Cancel onging TX/RX transaction
    LMIC.txCnt = LMIC.dnConf = LMIC.bcninfo.flags = 0;
    LMIC.opmode = (LMIC.opmode | OP_SCAN) & ~(OP_TXRXPEND);
    setBcnRxParams();
    LMIC.rxtime = LMIC.bcninfo.txtime = os_getTime() + sec2osticks(BCN_INTV_sec+1);
    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime, FUNC_ADDR(onBcnRx));
    os_radio(RADIO_RXON);
}


bit_t LMIC_enableTracking (u1_t tryBcnInfo) {
    if( (LMIC.opmode & (OP_SCAN|OP_TRACK|OP_SHUTDOWN)) != 0 )
	return 0;  // already in progress or failed to enable
    // If BCN info requested from NWK then app has to take are
    // of sending data up so that MCMD_BCNI_REQ can be attached.
    if( (LMIC.bcninfoTries = tryBcnInfo) == 0 )
	startScan();
    return 1;  // enabled
}


void LMIC_disableTracking () {
    LMIC.opmode &= ~(OP_SCAN|OP_TRACK);
    LMIC.bcninfoTries = 0;
    engineUpdate();
}


// ================================================================================
//
// Join stuff
//
// ================================================================================

static void buildJoinRequest (u1_t ftype) {
    // Do not use pendTxData since we might have a pending
    // user level frame in there. Use RX holding area instead.
    xref2u1_t d = LMIC.frame;
    d[OFF_JR_HDR] = ftype;
    os_getArtEui(d + OFF_JR_ARTEUI);
    os_getDevEui(d + OFF_JR_DEVEUI);
    os_wlsbf2(d + OFF_JR_DEVNONCE, LMIC.devNonce);
    aes_appendMic0(d, OFF_JR_MIC);

    EV(joininfo,INFO,(e_.deveui  = MAIN::CDEV->getEui(),
		      e_.arteui  = MAIN::CDEV->getArtEui(),
		      e_.nonce   = LMIC.devNonce,
		      e_.oldaddr = LMIC.devaddr,
		      e_.mic     = Base::lsbf4(&d[LORA::OFF_JR_MIC]),
		      e_.flags   = ((LMIC.opmode & OP_REJOIN) != 0
				    ? EV::joininfo_t::REJOIN_REQUEST
				    : EV::joininfo_t::REQUEST)));
    LMIC.dataLen = LEN_JR;
    LMIC.devNonce++;
    DO_DEVDB(updateDevNonce, LMIC.devNonce);
}

static void startJoining (xref2osjob_t osjob) {
    reportEvent(EV_JOINING);
}

// Start join procedure if not already joined.
bit_t LMIC_startJoining () {
    if( LMIC.devaddr == 0 ) {
	// There should be no TX/RX going on
	ASSERT((LMIC.opmode & (OP_POLL|OP_TXRXPEND)) == 0);
	// Cancel scanning
	LMIC.opmode &= ~(OP_SCAN|OP_REJOIN|OP_LINKDEAD|OP_NEXTCHNL);
	// Setup state
	LMIC.rejoinCnt = LMIC.txCnt = 0;
	initJoinLoop();
	LMIC.opmode |= OP_JOINING;
	// reportEvent will call engineUpdate which then starts sending JOIN REQUESTS
	os_setCallback(&LMIC.osjob, FUNC_ADDR(startJoining));
	return 1;
    }
    return 0;
}


// ================================================================================
//
//
//
// ================================================================================

static void processPingRx (xref2osjob_t osjob) {
    if( LMIC.dataLen != 0 ) {
	LMIC.txrxFlags = TXRX_PING;
	if( decodeFrame() ) {
	    reportEvent(EV_RXCOMPLETE);
	    return;
	}
    }
    // Pick next ping slot
    engineUpdate();
}


static bit_t processDnData () {
    ASSERT((LMIC.opmode & OP_TXRXPEND)!=0);

    if( LMIC.dataLen == 0 ) {
      norx:
	if( LMIC.txCnt != 0 ) {
	    if( LMIC.txCnt <= TXCONF_ATTEMPTS ) {
		// Schedule another retransmission
		txDelay(LMIC.rxtime, RETRY_PERIOD_secs);
		LMIC.opmode &= ~OP_TXRXPEND;
		engineUpdate();
		return 1;
	    }
	    LMIC.txrxFlags = TXRX_NACK | TXRX_NOPORT;
	} else {
	    // Nothing received - implies no port
	    LMIC.txrxFlags = TXRX_NOPORT;
	}
	LMIC.adrAckReq += 1;
	LMIC.dataBeg = LMIC.dataLen = 0;
      txcomplete:
	LMIC.opmode &= ~(OP_TXDATA|OP_TXRXPEND);
	if( (LMIC.txrxFlags & (TXRX_DNW1|TXRX_DNW2|TXRX_PING)) != 0  &&  (LMIC.opmode & OP_LINKDEAD) != 0 ) {
	    LMIC.opmode &= ~OP_LINKDEAD;
	    reportEvent(EV_LINK_ALIVE);
	}
	reportEvent(EV_TXCOMPLETE);
	// If we haven't heard from NWK in a while although we asked for a sign
	// assume link is dead - notify application and keep going
	if( LMIC.adrAckReq > LINK_CHECK_DEAD ) {
	    // We haven't heard from NWK for some time although
	    // We asked for a response for some time - assume we're disconnected. Lower DR one notch.
	    EV(devCond, ERR, (e_.reason = EV::devCond_t::LINK_DEAD,
			      e_.eui    = MAIN::CDEV->getEui(),
			      e_.info   = LMIC.adrAckReq));
	    setDrTxpow(DRCHG_NOADRACK, decDR((dr_t)LMIC.datarate), KEEP_TXPOW);
	    LMIC.adrAckReq = LINK_CHECK_CONT;
	    LMIC.opmode |= OP_REJOIN|OP_LINKDEAD;
	    reportEvent(EV_LINK_DEAD);
	}
	// If this falls to zero the NWK did not answer our MCMD_BCNI_REQ commands - try full scan
	if( LMIC.bcninfoTries > 0 ) {
	    if( (LMIC.opmode & OP_TRACK) != 0 ) {
		reportEvent(EV_BEACON_FOUND);
		LMIC.bcninfoTries = 0;
	    }
	    else if( --LMIC.bcninfoTries == 0 ) {
		startScan();   // NWK did not answer - try scan
	    }
	}
	return 1;
    }
    if( !decodeFrame() ) {
	if( (LMIC.txrxFlags & TXRX_DNW1) != 0 )
	    return 0;
	goto norx;
    }
    // If we received a frame reset counter
    LMIC.adrAckReq = LINK_CHECK_INIT;
    LMIC.rejoinCnt = 0;
    goto txcomplete;
}


static void processBeacon (xref2osjob_t osjob) {
    ostime_t lasttx = LMIC.bcninfo.txtime;   // save here - decodeBeacon might overwrite
    u1_t flags = LMIC.bcninfo.flags;
    ev_t ev;

    if( LMIC.dataLen != 0 && decodeBeacon() >= 1 ) {
	ev = EV_BEACON_TRACKED;
	if( (flags & (BCN_PARTIAL|BCN_FULL)) == 0 ) {
	    // We don't have a previous beacon to calc some drift - assume
	    // an max error of 13ms = 128sec*100ppm which is roughly +/-100ppm
	    calcBcnRxWindowFromMillis(13,0);
	    goto rev;
	}
	// We have a previous BEACON to calculate some drift
	s2_t drift = BCN_INTV_osticks - (LMIC.bcninfo.txtime - lasttx);
	if( LMIC.missedBcns > 0 ) {
	    drift = LMIC.drift + (drift - LMIC.drift) / (LMIC.missedBcns+1);
	}
	if( (LMIC.bcninfo.flags & BCN_NODRIFT) == 0 ) {
	    s2_t diff = LMIC.drift - drift;
	    if( diff < 0 ) diff = -diff;
	    LMIC.lastDriftDiff = diff;
	    if( LMIC.maxDriftDiff < diff )
		LMIC.maxDriftDiff = diff;
	    LMIC.bcninfo.flags &= ~BCN_NODDIFF;
	}
	LMIC.drift = drift;
	LMIC.missedBcns = LMIC.rejoinCnt = 0;
	LMIC.bcninfo.flags &= ~BCN_NODRIFT;
	EV(devCond,INFO,(e_.reason = EV::devCond_t::CLOCK_DRIFT,
			 e_.info   = drift,
			 e_.info2  = /*occasion BEACON*/0));
	ASSERT((LMIC.bcninfo.flags & (BCN_PARTIAL|BCN_FULL)) != 0);
    } else {
	ev = EV_BEACON_MISSED;
	LMIC.bcninfo.txtime += BCN_INTV_osticks - LMIC.drift;
	LMIC.bcninfo.time   += BCN_INTV_sec;
	LMIC.missedBcns++;
	// Delay any possible TX after surmised beacon - it's there although we missed it
	txDelay(LMIC.bcninfo.txtime + BCN_RESERVE_osticks, 4);
	if( LMIC.missedBcns > MAX_MISSED_BCNS )
	    LMIC.opmode |= OP_REJOIN;  // try if we can roam to another network
	if( LMIC.bcnRxsyms > MAX_RXSYMS ) {
	    LMIC.opmode &= ~(OP_TRACK|OP_PINGABLE|OP_PINGINI|OP_REJOIN);
	    reportEvent(EV_LOST_TSYNC);
	    return;
	}
    }
    LMIC.bcnRxtime = LMIC.bcninfo.txtime + BCN_INTV_osticks - calcRxWindow(0,DR_BCN);
    LMIC.bcnRxsyms = LMIC.rxsyms;    
  rev:
    if( (LMIC.opmode & OP_PINGINI) != 0 )
	rxschedInit(&LMIC.ping);  // note: reuses LMIC.frame buffer!
    reportEvent(ev);
}


static void startRxBcn (xref2osjob_t osjob) {
    LMIC.osjob.func = FUNC_ADDR(processBeacon);
    os_radio(RADIO_RX);
}


static void startRxPing (xref2osjob_t osjob) {
    LMIC.osjob.func = FUNC_ADDR(processPingRx);
    os_radio(RADIO_RX);
}


// Decide what to do next for the MAC layer of a device
static void engineUpdate () {
    // Check for ongoing state: scan or TX/RX transaction
    if( (LMIC.opmode & (OP_SCAN|OP_TXRXPEND|OP_SHUTDOWN)) != 0 ) 
	return;

    if( LMIC.devaddr == 0 && (LMIC.opmode & OP_JOINING) == 0 ) {
	LMIC_startJoining();
	return;
    }

    ostime_t now    = os_getTime();
    ostime_t rxtime = 0;
    ostime_t txbeg  = 0;

    if( (LMIC.opmode & OP_TRACK) != 0 ) {
	// We are tracking a beacon
	ASSERT( now + RX_RAMPUP - LMIC.bcnRxtime <= 0 );
	rxtime = LMIC.bcnRxtime - RX_RAMPUP;
    }

    if( (LMIC.opmode & (OP_JOINING|OP_REJOIN|OP_TXDATA|OP_POLL)) != 0 ) {
	// Need to TX some data...
	// Assuming txChnl points to channel which first becomes available again.
	bit_t jacc = ((LMIC.opmode & (OP_JOINING|OP_REJOIN)) != 0 ? 1 : 0);
	// Find next suitable channel and return availability time
	if( (LMIC.opmode & OP_NEXTCHNL) != 0 ) {
	    txbeg = LMIC.txend = nextTx(now);
	    LMIC.opmode &= ~OP_NEXTCHNL;
	} else {
	    txbeg = LMIC.txend;
	}
	// Delayed TX or waiting for duty cycle?
	if( (LMIC.globalDutyRate != 0 || (LMIC.opmode & OP_RNDTX) != 0)  &&  (txbeg - LMIC.globalDutyAvail) < 0 )
	    txbeg = LMIC.globalDutyAvail;
	// If we're tracking a beacon...
	// then make sure TX-RX transaction is complete before beacon
	if( (LMIC.opmode & OP_TRACK) != 0 &&
	    txbeg + (jacc ? JOIN_GUARD_osticks : TXRX_GUARD_osticks) - rxtime > 0 ) {
	    // Not enough time to complete TX-RX before beacon - postpone after beacon.
	    // In order to avoid clustering of postponed TX right after beacon randomize start!
	    txDelay(rxtime + BCN_RESERVE_osticks, 16);
	    txbeg = 0;
	    goto checkrx;
	}
	// Earliest possible time vs overhead to setup radio
	if( txbeg - (now + TX_RAMPUP) < 0 ) {
	    // We could send right now!
	    dr_t txdr = (dr_t)LMIC.datarate;
	    if( jacc ) {
		u1_t ftype;
		if( (LMIC.opmode & OP_REJOIN) != 0 ) {
		    txdr = lowerDR(txdr, LMIC.rejoinCnt);
		    ftype = HDR_FTYPE_REJOIN;
		} else {
		    ftype = HDR_FTYPE_JREQ;
		}
		buildJoinRequest(ftype);
		LMIC.osjob.func = FUNC_ADDR(jreqDone);
	    } else {
		if( LMIC.seqnoUp == 0xFFFFFFFF ) {
		    // Roll over of up seq counter
		    EV(specCond, ERR, (e_.reason = EV::specCond_t::UPSEQNO_ROLL_OVER,
				       e_.eui    = MAIN::CDEV->getEui()));
		    // Rerun join procedure - start with current datarate
		    LMIC.devaddr = 0;
		    LMIC_startJoining();
		    reportEvent(EV_RESET);
		    return;
		}
		buildDataFrame();
		// NOTE: a channel decision for LMIC.txChnl above will never be rendered invalid
		// by chosing a slower datarate (it could be invalided ONLY by a faster datarate).
		txdr = lowerDR(txdr, DRADJUST[LMIC.txCnt]);
		LMIC.osjob.func = FUNC_ADDR(updataDone);
	    }
	    LMIC.rps    = setCr(updr2rps(txdr), (cr_t)LMIC.errcr);
	    LMIC.rxsyms = txdr;  // carry TX datarate (can be != LMIC.datarate) over to txDone/setupRx1
	    LMIC.opmode = (LMIC.opmode & ~(OP_POLL|OP_RNDTX)) | OP_TXRXPEND | OP_NEXTCHNL;
	    updateTx(txbeg);
	    os_radio(RADIO_TX);
	    return;
	}
	// Cannot yet TX
	if( (LMIC.opmode & OP_TRACK) == 0 )
	    goto txdelay; // We don't track the beacon - nothing else to do - so wait for the time to TX
	// Consider RX tasks
	if( txbeg == 0 ) // zero indicates no TX pending
	    txbeg += 1;  // TX delayed by one tick (insignificant amount of time)
    } else {
	// No TX pending - no scheduled RX
	if( (LMIC.opmode & OP_TRACK) == 0 )
	    return;
    }

    // Are we pingable?
  checkrx:
    if( (LMIC.opmode & OP_PINGINI) != 0 ) {
	// One more RX slot in this beacon period?
	if( rxschedNext(&LMIC.ping, now+RX_RAMPUP) ) {
	    if( txbeg != 0  &&  (txbeg - LMIC.ping.rxtime) < 0 )
		goto txdelay;
	    LMIC.rxsyms  = LMIC.ping.rxsyms;
	    LMIC.rxtime  = LMIC.ping.rxtime;
	    LMIC.freq    = LMIC.ping.freq;
	    LMIC.rps     = dndr2rps(LMIC.ping.dr);
	    LMIC.dataLen = 0;
	    ASSERT(LMIC.rxtime - now+RX_RAMPUP >= 0 );
	    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, FUNC_ADDR(startRxPing));
	    return;
	}
	// no - just wait for the beacon
    }

    if( txbeg != 0  &&  (txbeg - rxtime) < 0 )
	goto txdelay;

    setBcnRxParams();
    LMIC.rxsyms = LMIC.bcnRxsyms;
    LMIC.rxtime = LMIC.bcnRxtime;
    if( now - rxtime >= 0 ) {
	LMIC.osjob.func = FUNC_ADDR(processBeacon);
	os_radio(RADIO_RX);
	return;
    }
    os_setTimedCallback(&LMIC.osjob, rxtime, FUNC_ADDR(startRxBcn));
    return;

  txdelay:
    EV(devCond, INFO, (e_.reason = EV::devCond_t::TX_DELAY,
		       e_.eui    = MAIN::CDEV->getEui(),
		       e_.info   = osticks2ms(txbeg-now),
		       e_.info2  = LMIC.seqnoUp-1));
    os_setTimedCallback(&LMIC.osjob, txbeg-TX_RAMPUP, FUNC_ADDR(runEngineUpdate));
}


void LMIC_setAdrMode (bit_t enabled) {
    LMIC.adrEnabled = enabled ? FCT_ADREN : 0;
}


//  Should we have/need an ext. API like this?
void LMIC_setDrTxpow (dr_t dr, s1_t txpow) {
    setDrTxpow(DRCHG_SET, dr, txpow);
}


void LMIC_shutdown () {
    os_clearCallback(&LMIC.osjob);
    os_radio(RADIO_RST);
    LMIC.opmode |= OP_SHUTDOWN;
}


void LMIC_reset () {
    EV(devCond, INFO, (e_.reason = EV::devCond_t::LMIC_EV,
		       e_.eui    = MAIN::CDEV->getEui(),
		       e_.info   = EV_RESET));
    os_radio(RADIO_RST);
    os_clearCallback(&LMIC.osjob);

    os_clearMem((xref2u1_t)&LMIC,SIZEOFEXPR(LMIC));
    LMIC.devaddr     = 0;
    LMIC.devNonce    = os_getRndU2();
    LMIC.opmode      = OP_NONE;
    LMIC.errcr       = CR_4_5;
    LMIC.adrEnabled  = FCT_ADREN;
    LMIC.dn2Dr       = DR_DNW2;   // we need this for 2ns DN window of join accept
    LMIC.dn2Freq     = FREQ_DNW2; // ditto
#if CFG_us915
    initDefaultChannels();
#endif
}


void LMIC_init () {
    LMIC.opmode = OP_SHUTDOWN;
}


void LMIC_clrTxData () {
    LMIC.opmode &= ~(OP_TXDATA|OP_TXRXPEND|OP_POLL);
    LMIC.pendTxLen = 0;
    if( (LMIC.opmode & (OP_JOINING|OP_SCAN)) != 0 ) // do not interfere with JOINING
	return;
    os_clearCallback(&LMIC.osjob);
    os_radio(RADIO_RST);
    engineUpdate();
}


void LMIC_setTxData () {
    LMIC.opmode |= OP_TXDATA;
    if( (LMIC.opmode & OP_JOINING) == 0 )
	LMIC.txCnt = 0;             // cancel any ongoing TX/RX retries
    engineUpdate();
}


//
int LMIC_setTxData2 (u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed) {
    if( dlen > SIZEOFEXPR(LMIC.pendTxData) )
	return -2;
    if( data != (xref2u1_t)0 )
	os_copyMem(LMIC.pendTxData, data, dlen);
    LMIC.pendTxConf = confirmed;
    LMIC.pendTxPort = port;
    LMIC.pendTxLen  = dlen;
    LMIC_setTxData();
    return 0;
}


// Send a payload-less message to signal device is alive
void LMIC_sendAlive () {
    LMIC.opmode |= OP_POLL;
    engineUpdate();
}


// Check if other networks are around.
void LMIC_tryRejoin () {
    LMIC.opmode |= OP_REJOIN;
    engineUpdate();
}


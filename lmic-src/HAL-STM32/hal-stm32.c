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
#include "stm32l1xx.h"

// HAL STATE
static struct {
    int irqlevel;
    u4_t ticks;
} HAL;

// -----------------------------------------------------------------------------
// GPIOCFG macros
#define GPIOCFG_AF_MASK        0x000F
#define GPIOCFG_MODE_SHIFT      4
#define GPIOCFG_MODE_MASK      (3<<GPIOCFG_MODE_SHIFT)
#define GPIOCFG_MODE_INP       (0<<GPIOCFG_MODE_SHIFT)
#define GPIOCFG_MODE_OUT       (1<<GPIOCFG_MODE_SHIFT)
#define GPIOCFG_MODE_ALT       (2<<GPIOCFG_MODE_SHIFT)
#define GPIOCFG_MODE_ANA       (3<<GPIOCFG_MODE_SHIFT)
#define GPIOCFG_OSPEED_SHIFT    6
#define GPIOCFG_OSPEED_MASK    (3<<GPIOCFG_OSPEED_SHIFT)
#define GPIOCFG_OSPEED_400kHz  (0<<GPIOCFG_OSPEED_SHIFT)
#define GPIOCFG_OSPEED_2MHz    (1<<GPIOCFG_OSPEED_SHIFT)
#define GPIOCFG_OSPEED_10MHz   (2<<GPIOCFG_OSPEED_SHIFT)
#define GPIOCFG_OSPEED_40MHz   (3<<GPIOCFG_OSPEED_SHIFT)
#define GPIOCFG_OTYPE_SHIFT     8
#define GPIOCFG_OTYPE_MASK     (1<<GPIOCFG_OTYPE_SHIFT)
#define GPIOCFG_OTYPE_PUPD     (0<<GPIOCFG_OTYPE_SHIFT)
#define GPIOCFG_OTYPE_OPEN     (1<<GPIOCFG_OTYPE_SHIFT)
#define GPIOCFG_PUPD_SHIFT      9
#define GPIOCFG_PUPD_MASK      (3<<GPIOCFG_PUPD_SHIFT)
#define GPIOCFG_PUPD_NONE      (0<<GPIOCFG_PUPD_SHIFT)
#define GPIOCFG_PUPD_PUP       (1<<GPIOCFG_PUPD_SHIFT)
#define GPIOCFG_PUPD_PDN       (2<<GPIOCFG_PUPD_SHIFT)
#define GPIOCFG_PUPD_RFU       (3<<GPIOCFG_PUPD_SHIFT)

#define GPIO_IRQ_MASK          0x38
#define GPIO_IRQ_FALLING       0x20
#define GPIO_IRQ_RISING        0x28

// GPIO by port number (A=0, B=1, ..)
#define GPIOx(no) ((GPIO_TypeDef*) (GPIOA_BASE + (no)*(GPIOB_BASE-GPIOA_BASE)))

#define GPIO_AF_BITS        4     // width of bit field
#define GPIO_AF_MASK        0x0F  // mask in AFR[0/1]
#define GPIO_AFRLR(i)       ((i)>>3)
#define GPIO_AF_PINi(i,af)  ((af)<<(((i)&7)*GPIO_AF_BITS))
#define GPIO_AF_set(gpio,i,af) ((gpio)->AFR[GPIO_AFRLR(i)] = \
				((gpio)->AFR[GPIO_AFRLR(i)] \
				 & ~GPIO_AF_PINi(i,GPIO_AF_MASK)) \
				|   GPIO_AF_PINi(i,af))

static void hw_cfg_pin (GPIO_TypeDef* gpioport, u1_t pin, u2_t gpiocfg) {
    u1_t pin2 = 2*pin;

    GPIO_AF_set(gpioport, pin, gpiocfg & GPIOCFG_AF_MASK);
    gpioport->MODER   = (gpioport->MODER   & ~(3 << pin2)) | (((gpiocfg >> GPIOCFG_MODE_SHIFT  ) & 3) << pin2);
    gpioport->OSPEEDR = (gpioport->OSPEEDR & ~(3 << pin2)) | (((gpiocfg >> GPIOCFG_OSPEED_SHIFT) & 3) << pin2);
    gpioport->OTYPER  = (gpioport->OTYPER  & ~(1 << pin )) | (((gpiocfg >> GPIOCFG_OTYPE_SHIFT ) & 1) << pin );
    gpioport->PUPDR   = (gpioport->PUPDR   & ~(3 << pin2)) | (((gpiocfg >> GPIOCFG_PUPD_SHIFT  ) & 3) << pin2);
}

static void hw_cfg_extirq (u1_t portidx, u1_t pin, u1_t irqcfg) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // make sure module is on

    // configure external interrupt (set 4-bit portidx A-G for every pin 0-15)
    u4_t tmp1 = (pin & 0x3) << 2;
    u4_t tmp2 = ((u4_t)0x0F) << tmp1;
    SYSCFG->EXTICR[pin >> 2] = (SYSCFG->EXTICR[pin >> 2] & ~tmp2) | (((u4_t)portidx) << tmp1);

    // configure trigger and enable irq
    u4_t mask = (u4_t)(1 << pin);
    EXTI->RTSR &= ~mask; // clear trigger
    EXTI->FTSR &= ~mask; // clear trigger
    switch(irqcfg & GPIO_IRQ_MASK) {
      case GPIO_IRQ_RISING:   EXTI->RTSR |= mask; break; // trigger at rising edge
      case GPIO_IRQ_FALLING:  EXTI->FTSR |= mask; break; // trigger at falling edge
    }
    EXTI->IMR  |= mask;  // enable IRQ (pin x for all ports)

    // configure the NVIC
    u1_t channel = (pin < 5) ? (EXTI0_IRQn+pin) : ((pin<10) ? EXTI9_5_IRQn : EXTI15_10_IRQn);
    NVIC->IP[channel] = 0x70; // interrupt priority
    NVIC->ISER[channel>>5] = 1<<(channel&0x1F);  // set enable IRQ
}


// -----------------------------------------------------------------------------
// I/O

#ifdef CFG_sx1276mb1_board

#define NSS_PORT           1 // NSS: PB6, sx1276
#define NSS_PIN            6  // sx1276: PB6

#define TX_PORT            2 // TX:  PC1
#define TX_PIN             1

#define RST_PORT           0 // RST: PA0
#define RST_PIN            0

#define DIO0_PORT          0 // DIO0: PA10, sx1276   (line 1 irq handler)
#define DIO0_PIN           10
#define DIO1_PORT          1 // DIO1: PB3, sx1276  (line 10-15 irq handler)
#define DIO1_PIN           3
#define DIO2_PORT          1 // DIO2: PB5, sx1276  (line 10-15 irq handler)
#define DIO2_PIN           5

static const u1_t outputpins[] = { NSS_PORT, NSS_PIN, TX_PORT, TX_PIN  };
static const u1_t inputpins[]  = { DIO0_PORT, DIO0_PIN, DIO1_PORT, DIO1_PIN, DIO2_PORT, DIO2_PIN };

#elif CFG_wimod_board

// output lines
#define NSS_PORT           1 // NSS: PB0, sx1272
#define NSS_PIN            0

#define TX_PORT            0 // TX:  PA4
#define TX_PIN             4
#define RX_PORT            2 // RX:  PC13
#define RX_PIN            13
#define RST_PORT           0 // RST: PA2
#define RST_PIN            2

// input lines
#define DIO0_PORT          1 // DIO0: PB1   (line 1 irq handler)
#define DIO0_PIN           1
#define DIO1_PORT          1 // DIO1: PB10  (line 10-15 irq handler)
#define DIO1_PIN          10
#define DIO2_PORT          1 // DIO2: PB11  (line 10-15 irq handler)
#define DIO2_PIN          11

static const u1_t outputpins[] = { NSS_PORT, NSS_PIN, TX_PORT, TX_PIN, RX_PORT, RX_PIN };
static const u1_t inputpins[]  = { DIO0_PORT, DIO0_PIN, DIO1_PORT, DIO1_PIN, DIO2_PORT, DIO2_PIN };

#else
#error Missing CFG_sx1276mb1_board/CFG_wimod_board!
#endif

static void setpin (GPIO_TypeDef* gpioport, u1_t pin, u1_t state) {
    gpioport->ODR     = (gpioport->ODR     & ~(1 << pin))  | ((state & 1) << pin );
}

static void hal_io_init () {
    // clock enable for GPIO ports A,B,C
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    // configure output lines and set to low state
    for(u1_t i=0; i<sizeof(outputpins); i+=2) {
        hw_cfg_pin(GPIOx(outputpins[i]), outputpins[i+1], GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
        setpin(GPIOx(outputpins[i]), outputpins[i+1], 0);
    }

    // configure input lines and register IRQ
    for(u1_t i=0; i<sizeof(inputpins); i+=2) {
        hw_cfg_pin(GPIOx(inputpins[i]), inputpins[i+1], GPIOCFG_MODE_INP | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_OPEN);
        hw_cfg_extirq(inputpins[i], inputpins[i+1], GPIO_IRQ_RISING);
}
}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (u1_t val) {
    ASSERT(val == 1 || val == 0);
#ifndef CFG_sx1276mb1_board
    setpin(GPIOx(RX_PORT), RX_PIN, ~val);
#endif
    setpin(GPIOx(TX_PORT), TX_PIN, val);
}


// set radio NSS pin to given value
void hal_pin_nss (u1_t val) {
    setpin(GPIOx(NSS_PORT), NSS_PIN, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if(val == 0 || val == 1) { // drive pin
        hw_cfg_pin(GPIOx(RST_PORT), RST_PIN, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
	setpin(GPIOx(RST_PORT), RST_PIN, val);
    } else { // keep pin floating
        hw_cfg_pin(GPIOx(RST_PORT), RST_PIN, GPIOCFG_MODE_INP | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_OPEN);
    }
}

extern void radio_irq_handler(u1_t dio);

// generic EXTI IRQ handler for all channels
static void EXTI_IRQHandler () {
    // DIO 0
    if((EXTI->PR & (1<<DIO0_PIN)) != 0) { // pending
	EXTI->PR = (1<<DIO0_PIN); // clear irq
        // invoke radio handler (on IRQ!)
        radio_irq_handler(0);
    }
    // DIO 1
    if((EXTI->PR & (1<<DIO1_PIN)) != 0) { // pending
	EXTI->PR = (1<<DIO1_PIN); // clear irq
        // invoke radio handler (on IRQ!)
        radio_irq_handler(1);
    }
    // DIO 2
    if((EXTI->PR & (1<<DIO2_PIN)) != 0) { // pending
	EXTI->PR = (1<<DIO2_PIN); // clear irq
        // invoke radio handler (on IRQ!)
        radio_irq_handler(2);
    }
#ifdef CFG_EXTI_IRQ_HANDLER
    // invoke user-defined interrupt handler
    {
        extern void CFG_EXTI_IRQ_HANDLER();
        CFG_EXTI_IRQ_HANDLER();
    }
#endif // CFG_EXTI_IRQ_HANDLER
}

void EXTI0_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI1_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI2_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI3_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI4_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI15_10_IRQHandler () {
    EXTI_IRQHandler();
}

// -----------------------------------------------------------------------------
// SPI

// for sx1272 and 1276

#define SCK_PORT   0 // SCK:  PA5
#define SCK_PIN    5
#define MISO_PORT  0 // MISO: PA6
#define MISO_PIN   6
#define MOSI_PORT  0 // MOSI: PA7
#define MOSI_PIN   7

#define GPIO_AF_SPI1        0x05

static void hal_spi_init () {
    // enable clock for SPI interface 1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // use alternate function SPI1 (SCK, MISO, MOSI)
    hw_cfg_pin(GPIOx(SCK_PORT),  SCK_PIN,  GPIOCFG_MODE_ALT | GPIOCFG_OSPEED_40MHz | GPIO_AF_SPI1 | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
    hw_cfg_pin(GPIOx(MISO_PORT), MISO_PIN, GPIOCFG_MODE_ALT | GPIOCFG_OSPEED_40MHz | GPIO_AF_SPI1 | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
    hw_cfg_pin(GPIOx(MOSI_PORT), MOSI_PIN, GPIOCFG_MODE_ALT | GPIOCFG_OSPEED_40MHz | GPIO_AF_SPI1 | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
    
    // configure and activate the SPI (master, internal slave select, software slave mgmt)
    // (use default mode: 8-bit, 2-wire, no crc, MSBF, PCLK/2, CPOL0, CPHA0)
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE;
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    SPI1->DR = out;
    while( (SPI1->SR & SPI_SR_RXNE ) == 0);
    return SPI1->DR; // in
}


// -----------------------------------------------------------------------------
// TIME
static void hal_time_init () {
    PWR->CR |= PWR_CR_DBP; // disable write protect
    RCC->CSR |= RCC_CSR_LSEON; // switch on low-speed oscillator @32.768kHz
    while( (RCC->CSR & RCC_CSR_LSERDY) == 0 ); // wait for it...

    RCC->APB2ENR   |= RCC_APB2ENR_TIM9EN;     // enable clock to TIM9 peripheral 
    RCC->APB2LPENR |= RCC_APB2LPENR_TIM9LPEN; // enable clock to TIM9 peripheral also in low power mode
    RCC->APB2RSTR  |= RCC_APB2RSTR_TIM9RST;   // reset TIM9 interface
    RCC->APB2RSTR  &= ~RCC_APB2RSTR_TIM9RST;  // reset TIM9 interface

    TIM9->SMCR = TIM_SMCR_ECE; // external clock enable (source clock mode 2) with no prescaler and no filter

    NVIC->IP[TIM9_IRQn] = 0x70; // interrupt priority
    NVIC->ISER[TIM9_IRQn>>5] = 1<<(TIM9_IRQn&0x1F);  // set enable IRQ

    // enable update (overflow) interrupt
    TIM9->DIER |= TIM_DIER_UIE;
    
    // Enable timer counting
    TIM9->CR1 = TIM_CR1_CEN;
}

u4_t hal_ticks () {
    hal_disableIRQs();
    u4_t t = HAL.ticks;
    u2_t cnt = TIM9->CNT;
    if( (TIM9->SR & TIM_SR_UIF) ) {
	// Overflow before we read CNT?
	// Include overflow in evaluation but
	// leave update of state to ISR once interrupts enabled again
	cnt = TIM9->CNT;
	t++;
    }
    hal_enableIRQs();
    return (t<<16)|cnt;
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static u2_t deltaticks (u4_t time) {
    u4_t t = hal_ticks();
    s4_t d = time - t;
    if( d<=0 ) return 0;    // in the past
    if( (d>>16)!=0 ) return 0xFFFF; // far ahead
    return (u2_t)d;
}

void hal_waitUntil (u4_t time) {
    while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    u2_t dt;
    TIM9->SR &= ~TIM_SR_CC2IF; // clear any pending interrupts
    if((dt = deltaticks(time)) < 5) { // event is now (a few ticks ahead)
        TIM9->DIER &= ~TIM_DIER_CC2IE; // disable IE
        return 1;
    } else { // rewind timer (fully or to exact time))
        TIM9->CCR2 = TIM9->CNT + dt;   // set comparator
        TIM9->DIER |= TIM_DIER_CC2IE;  // enable IE
        TIM9->CCER |= TIM_CCER_CC2E;   // enable capture/compare uint 2
        return 0;
    }
}
  
void TIM9_IRQHandler () {
    if(TIM9->SR & TIM_SR_UIF) { // overflow
        HAL.ticks++;
    }
    if((TIM9->SR & TIM_SR_CC2IF) && (TIM9->DIER & TIM_DIER_CC2IE)) { // expired
        // do nothing, only wake up cpu
    }
    TIM9->SR = 0; // clear IRQ flags
}

// -----------------------------------------------------------------------------
// IRQ
void hal_disableIRQs () {
    __disable_irq();
    HAL.irqlevel++;
}

void hal_enableIRQs () {
    if(--HAL.irqlevel == 0) {
        __enable_irq();
    }
}

void hal_sleep () {
    // low power sleep mode
    PWR->CR |= PWR_CR_LPSDSR;
    // suspend execution until IRQ, regardless of the CPSR I-bit
    __WFI();
}

// -----------------------------------------------------------------------------

void hal_init () {
    memset(&HAL, 0x00, sizeof(HAL));
    hal_disableIRQs();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();

    DEBUG_INIT();

    hal_enableIRQs();
}

void hal_failed () {
    // HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1);
}

//////////////////////////////////////////////////////////////////////
// DEBUG CODE BELOW (use CFG_DEBUG)
//////////////////////////////////////////////////////////////////////
#ifdef CFG_DEBUG

#define LED_PORT        GPIOA // use GPIO PA8 (LED4 on IMST, P11/PPS/EXT1_10/GPS6 on Blipper)
#define LED_PIN         8
#define USART_TX_PORT   GPIOA
#define USART_TX_PIN    9
#define GPIO_AF_USART1  0x07

void debug_init () {
    // configure LED pin as output
    hw_cfg_pin(LED_PORT, LED_PIN, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
    debug_led(0);

    // configure USART1 (115200/8N1, tx-only)
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    hw_cfg_pin(USART_TX_PORT, USART_TX_PIN, GPIOCFG_MODE_ALT|GPIOCFG_OSPEED_40MHz|GPIOCFG_OTYPE_PUPD|GPIOCFG_PUPD_PUP|GPIO_AF_USART1);
    USART1->BRR = 277; // 115200
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE; // usart+transmitter enable

    // print banner
    debug_str("\r\n============== DEBUG STARTED ==============\r\n");
}

void debug_led (u1_t val) {
    setpin(LED_PORT, LED_PIN, val);
}

void debug_char (u1_t c) {
    while( !(USART1->SR & USART_SR_TXE) );    
    USART1->DR = c;
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
#endif // CFG_DEBUG


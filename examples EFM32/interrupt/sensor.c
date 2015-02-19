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

#define GPIOx(no) ((GPIO_TypeDef*) (GPIOA_BASE + (no)*(GPIOB_BASE-GPIOA_BASE)))

// use PB12 (DIP switch 1)
#define INP_PORT 1
#define INP_PIN 12

static void cfg_input_pin (GPIO_TypeDef* port, u1_t pin) {
    unsigned char pin2 = 2*pin;
    port->MODER   &= ~(3 << pin2); // MODE  INP   = 0
    port->OSPEEDR |= (3 << pin2);  // SPEED 40MHz = 3
    port->OTYPER  |= (1 << pin);   // OTYPE OPEN  = 1
}

static void cfg_extirq_pin (u1_t portidx, u1_t pin) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // make sure module is on

    // configure external interrupt (set 4-bit portidx A-G for every pin 0-15)
    u4_t tmp1 = (pin & 0x3) << 2;
    u4_t tmp2 = ((u4_t)0x0F) << tmp1;
    SYSCFG->EXTICR[pin >> 2] = (SYSCFG->EXTICR[pin >> 2] & ~tmp2) | (((u4_t)portidx) << tmp1);

    // configure trigger and enable irq
    u4_t mask = (u4_t)(1 << pin);
    EXTI->RTSR |= mask; // trigger at rising edge
    EXTI->FTSR |= mask; // trigger at falling edge
    EXTI->IMR  |= mask; // enable IRQ (pin x for all ports)

    // configure the NVIC
    u1_t channel = (pin < 5) ? (EXTI0_IRQn+pin) : ((pin<10) ? EXTI9_5_IRQn : EXTI15_10_IRQn);
    NVIC->IP[channel] = 0x70; // interrupt priority
    NVIC->ISER[channel>>5] = 1<<(channel&0x1F);  // set enable IRQ
}

static osjob_t irqjob;

// use DIP1 as sensor value
void initsensor (osjobcb_t callback) {
    // configure input
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; // clock enable port B
    cfg_input_pin(GPIOx(INP_PORT), INP_PIN);
    cfg_extirq_pin(INP_PORT, INP_PIN);
    // save application callback
    irqjob.func = callback;
}

// read PB12
u2_t readsensor () {
    return ((GPIOB->IDR & (1 << INP_PIN)) != 0);
}

// called by EXTI_IRQHandler
// (set preprocessor option CFG_EXTI_IRQ_HANDLER=sensorirq)
void sensorirq () {
    if((EXTI->PR & (1<<INP_PIN)) != 0) { // pending
	EXTI->PR = (1<<INP_PIN); // clear irq
        // run application callback function in 50ms (debounce)
        os_setTimedCallback(&irqjob, os_getTime()+ms2osticks(50), irqjob.func);
    }
}

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

static void cfg_input_pin (GPIO_TypeDef* port, u1_t pin) {
    unsigned char pin2 = 2*pin;
    port->MODER   &= ~(3 << pin2); // MODE  INP   = 0
    port->OSPEEDR |= (3 << pin2);  // SPEED 40MHz = 3
    port->OTYPER  |= (1 << pin);   // OTYPE OPEN  = 1
}

// use PB12 (DIP switch 1) as sensor value
void initsensor () {
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; // clock enable port B
    cfg_input_pin(GPIOB, 12); // PB12
}

// read PB12
u2_t readsensor () {
    return ((GPIOB->IDR & (1 << 12)) != 0);
}

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

#include "../lmic/lmic.h"

#include "em_chip.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_usart.h"

// -----------------------------------------------------------------------------
// I/O
#if defined PA52
	#define PORT_DIO0			gpioPortB	//DIO0 = PB11
	#define PIN_DIO0			11
	#define PORT_DIO1			gpioPortC	//DIO1 = PC1
	#define PIN_DIO1			1
	#define PORT_DIO2			gpioPortC	//DIO2 = PC0
	#define PIN_DIO2			0
	#define PORT_DIO3			gpioPortA	//DIO3 = PA2
	#define PIN_DIO3			2
	#define PORT_DIO4			gpioPortA	//DIO4 = PA1
	#define PIN_DIO4			1
	#define PORT_DIO5			gpioPortA	//DIO5 = PA0
	#define PIN_DIO5			0
	#define USART_USED                USART0
	#define USART_LOCATION            USART_ROUTE_LOCATION_LOC0
	#define USART_CLK                 cmuClock_USART0
	#define PIN_SPI_TX                10			//MOSI = PE10
	#define PORT_SPI_TX               gpioPortE
	#define PIN_SPI_RX                11			//MISO = PE11
	#define PORT_SPI_RX               gpioPortE
	#define PIN_SPI_CLK               12			//SCK = PE12
	#define PORT_SPI_CLK              gpioPortE
	#define PIN_SPI_CS                13			//CSN = PE13
	#define PORT_SPI_CS               gpioPortE
#elif defined PA53
	#define PORT_DIO0			gpioPortC	//DIO0 = PC1
	#define PIN_DIO0			1U
	#define PORT_DIO1			gpioPortC	//DIO1 = PC0
	#define PIN_DIO1			0U
	#define PORT_DIO2			gpioPortA	//DIO2 = PA2
	#define PIN_DIO2			2U
	#define PORT_DIO4			gpioPortA	//DIO4 = PA1
	#define PIN_DIO4			1U
	#define PORT_DIO5			gpioPortA	//DIO5 = PA0
	#define PIN_DIO5			0U
	#define USART_USED                USART0
	#define USART_LOCATION            USART_ROUTE_LOCATION_LOC0
	#define USART_CLK                 cmuClock_USART0
	#define PIN_SPI_TX                10			//MOSI = PE10
	#define PORT_SPI_TX               gpioPortE
	#define PIN_SPI_RX                11			//MISO = PE11
	#define PORT_SPI_RX               gpioPortE
	#define PIN_SPI_CLK               12			//SCK = PE12
	#define PORT_SPI_CLK              gpioPortE
	#define PIN_SPI_CS                13			//CSN = PE13
	#define PORT_SPI_CS               gpioPortE
#else
#error Missing board!
#endif

// HAL state
static struct
{
    //int irqlevel;
    u4_t ticks;
} HAL;

// -----------------------------------------------------------------------------
// I/O

static void hal_io_init ()
{
	//Enable module clocks
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_PinModeSet(PORT_DIO0, PIN_DIO0, gpioModeInputPull, 0);	//DIO0=PayLoadReady
	GPIO_PinModeSet(PORT_DIO1, PIN_DIO1, gpioModeInputPull, 0);	//DIO1=FifoLevel
	GPIO_PinModeSet(PORT_DIO2, PIN_DIO2, gpioModeInputPull, 0);	//DIO2=SyncAddr
	#if !defined PA53
	GPIO_PinModeSet(PORT_DIO3, PIN_DIO3, gpioModeInputPull, 0);	//DIO3=FifoEmpty
	#endif
	GPIO_PinModeSet(PORT_DIO4, PIN_DIO4, gpioModeInputPull, 0);	//DIO4=PreambleDetect/RSSI
	GPIO_PinModeSet(PORT_DIO5, PIN_DIO5, gpioModeInputPull, 0);	//DIO5=ModeReady

	GPIO_IntConfig(PORT_DIO0, PIN_DIO0, true, false, true);
	GPIO_IntConfig(PORT_DIO1, PIN_DIO1, true, false, true);
	GPIO_IntConfig(PORT_DIO2, PIN_DIO2, true, false, true);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (u1_t val)
{
	//not used in PA52 nor PA53
}


// set radio NSS pin to given value
void hal_pin_nss (u1_t val)
{
	if (val)
		GPIO_PinOutSet(PORT_SPI_CS, PIN_SPI_CS);	//  SPI Disable
	else
		GPIO_PinOutClear(PORT_SPI_CS, PIN_SPI_CS);	//  SPI Enable (Active Low)
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val)
{
	#if defined PA52
		//not used in PA52
	#elif defined PA53
	if(val == 0 || val == 1) { // drive pin
        hw_cfg_pin(GPIOx(RST_PORT), RST_PIN, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
        hw_set_pin(GPIOx(RST_PORT), RST_PIN, val);
    } else { // keep pin floating
        hw_cfg_pin(GPIOx(RST_PORT), RST_PIN, GPIOCFG_MODE_INP | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_OPEN);
    }
	#endif
}

extern void radio_irq_handler(u1_t dio);

void GPIO_ODD_IRQHandler(void)	//impar
 {
	u4_t i = GPIO_IntGet();
	if (i & 1<<PIN_DIO0)
		radio_irq_handler(0);
	else if (i & 1<<PIN_DIO1)
		radio_irq_handler(1);
	GPIO_IntClear(0xAAAA);
 }

void GPIO_EVEN_IRQHandler(void)	//par
 {
	u4_t i = GPIO_IntGet();
	if (i & 1<<PIN_DIO2)
		radio_irq_handler(2);
	GPIO_IntClear(0x5555);
 }

static void hal_spi_init ()
{
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

	/* Enable module clocks */
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(USART_CLK, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	/* Configure SPI pins */
	GPIO_PinModeSet(PORT_SPI_TX, PIN_SPI_TX, gpioModePushPull, 0);
	GPIO_PinModeSet(PORT_SPI_RX, PIN_SPI_RX, gpioModeInputPull, 0);
	GPIO_PinModeSet(PORT_SPI_CLK,PIN_SPI_CLK,gpioModePushPull, 0);
	GPIO_PinModeSet(PORT_SPI_CS, PIN_SPI_CS, gpioModePushPull, 1);

	/* Reset USART just in case */
	USART_Reset(USART_USED);

	init.clockMode = usartClockMode0;

	/* Configure to use SPI master with manual CS */
	init.refFreq = 0;
	init.baudrate = 1000000;
	init.msbf = true;	/* Send most significant bit first. */
	USART_InitSync(USART_USED, &init);

	/* Enabling pins and setting location, SPI CS not enable */
	USART_USED->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | USART_LOCATION;
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out)
{
	/* For every byte sent, one is received */
	USART_Tx(USART_USED, out);
	return USART_Rx(USART_USED);
}


// -----------------------------------------------------------------------------
// TIME
static uint8_t       rtcInitialized = 0;    /**< 1 if rtc is initialized */
static uint32_t      rtcFreq;               /**< RTC Frequence. 32.768 kHz */

/***************************************************************************//**
 * @brief RTC Interrupt Handler, invoke callback function if defined.
 ******************************************************************************/
void RTC_IRQHandler(void)
{
	if (RTC_IntGet() & RTC_IF_OF)
	{
		HAL.ticks ++;
	}

    if(RTC_IntGet() & RTC_IF_COMP0) // expired
    {
        // do nothing, only wake up cpu
    }
	RTC_IntClear(_RTC_IF_MASK); // clear IRQ flags
}


static void hal_time_init ()
{
	RTC_Init_TypeDef init;

	rtcInitialized = 1;

	/* Ensure LE modules are accessible */
	CMU_ClockEnable(cmuClock_CORELE, true);

	/* Enable LFACLK in CMU (will also enable oscillator if not enabled) */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

	/* Use the prescaler to reduce power consumption. */
	CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

	rtcFreq = CMU_ClockFreqGet(cmuClock_RTC);

	/* Enable clock to RTC module */
	CMU_ClockEnable(cmuClock_RTC, true);

	init.enable   = false;
	init.debugRun = false;
	init.comp0Top = false;
	//init.comp0Top = true; /* Count to max before wrapping */
	RTC_Init(&init);

	/* Disable interrupt generation from RTC0 */
	RTC_IntDisable(_RTC_IF_MASK);

	/* Enable interrupts */
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Enable RTC */
	RTC_Enable(true);

	RTC_IntEnable(RTC_IF_OF);	//Enable interrupt on overflow
}

u4_t hal_ticks ()
{
    hal_disableIRQs();
    u4_t t = HAL.ticks;
    u4_t cnt = RTC_CounterGet();
	if (RTC_IntGet() & RTC_IF_OF)	// Overflow before we read CNT?
	{
        cnt = RTC_CounterGet();
        t ++;	// Include overflow in evaluation but leave update of state to ISR once interrupts enabled again
    }
    hal_enableIRQs();
    return (t<<24)|cnt;
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static u2_t deltaticks (u4_t time)
{
    u4_t t = hal_ticks();
    s4_t d = time - t;
    if( d<=0 ) return 0;    // in the past
    if( (d>>16)!=0 ) return 0xFFFF; // far ahead
    return (u2_t)d;
}

void hal_waitUntil (u4_t time)
{
    while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time)
{
    u2_t dt;
	RTC_IntClear(RTC_IF_COMP0);		//clear any pending interrupts
    if((dt = deltaticks(time)) < 5) // event is now (a few ticks ahead)
    {
    	RTC_IntDisable(RTC_IF_COMP0);	// disable IE
        return 1;
    }
    else // rewind timer (fully or to exact time))
    {
    	RTC_CompareSet(0, RTC_CounterGet() + dt);   // set comparator
    	RTC_IntEnable(RTC_IF_COMP0);  // enable IE
        return 0;
    }
}
  


// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs ()
{
	INT_Disable();
}

void hal_enableIRQs ()
{
	INT_Enable();
}

void hal_sleep ()
{
	EMU_EnterEM2(false);
}

// -----------------------------------------------------------------------------

void hal_init ()
{
    CHIP_Init();

    memset(&HAL, 0x00, sizeof(HAL));
    hal_disableIRQs();

    hal_io_init();	// configure radio I/O and interrupt handler

    hal_spi_init();	// configure radio SPI

    hal_time_init();	// configure timer and interrupt handler

    hal_enableIRQs();
}

void hal_failed ()
{
	debug_led(1);
	// HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1);
}


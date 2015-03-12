#include "lmic.h"

/* CMSIS package */
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_emu.h"
#include "em_rmu.h"
#include "em_rtc.h"
#include "rtcdrv.h"


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

//static void hw_cfg_pin (GPIO_TypeDef* gpioport, u1_t pin, u2_t gpiocfg) {
//    u1_t pin2 = 2*pin;
//
//    GPIO_AF_set(gpioport, pin, gpiocfg & GPIOCFG_AF_MASK);
//    gpioport->MODER   = (gpioport->MODER   & ~(3 << pin2)) | (((gpiocfg >> GPIOCFG_MODE_SHIFT  ) & 3) << pin2);
//    gpioport->OSPEEDR = (gpioport->OSPEEDR & ~(3 << pin2)) | (((gpiocfg >> GPIOCFG_OSPEED_SHIFT) & 3) << pin2);
//    gpioport->OTYPER  = (gpioport->OTYPER  & ~(1 << pin )) | (((gpiocfg >> GPIOCFG_OTYPE_SHIFT ) & 1) << pin );
//    gpioport->PUPDR   = (gpioport->PUPDR   & ~(3 << pin2)) | (((gpiocfg >> GPIOCFG_PUPD_SHIFT  ) & 3) << pin2);
//}

//static void hw_cfg_extirq (u1_t portidx, u1_t pin, u1_t irqcfg) {
//    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // make sure module is on
//
//    // configure external interrupt (set 4-bit portidx A-G for every pin 0-15)
//    u4_t tmp1 = (pin & 0x3) << 2;
//    u4_t tmp2 = ((u4_t)0x0F) << tmp1;
//    SYSCFG->EXTICR[pin >> 2] = (SYSCFG->EXTICR[pin >> 2] & ~tmp2) | (((u4_t)portidx) << tmp1);
//
//    // configure trigger and enable irq
//    u4_t mask = (u4_t)(1 << pin);
//    EXTI->RTSR &= ~mask; // clear trigger
//    EXTI->FTSR &= ~mask; // clear trigger
//    switch(irqcfg & GPIO_IRQ_MASK) {
//      case GPIO_IRQ_RISING:   EXTI->RTSR |= mask; break; // trigger at rising edge
//      case GPIO_IRQ_FALLING:  EXTI->FTSR |= mask; break; // trigger at falling edge
//    }
//    EXTI->IMR  |= mask;  // enable IRQ (pin x for all ports)
//
//    // configure the NVIC
//    u1_t channel = (pin < 5) ? (EXTI0_IRQn+pin) : ((pin<10) ? EXTI9_5_IRQn : EXTI15_10_IRQn);
//    NVIC->IP[channel] = 0x70; // interrupt priority
//    NVIC->ISER[channel>>5] = 1<<(channel&0x1F);  // set enable IRQ
//}


// -----------------------------------------------------------------------------
// I/O

#ifdef  TG_STK

#define USART_USED                USART1
#define USART_LOCATION            USART_ROUTE_LOCATION_LOC1
#define USART_CLK                 cmuClock_USART1
#define PIN_SPI_TX                0				//MOSI = PD0
#define PORT_SPI_TX               gpioPortD
#define PIN_SPI_RX                1				//MISO = PD1
#define PORT_SPI_RX               gpioPortD
#define PIN_SPI_CLK               2				//SCK = PD2
#define PORT_SPI_CLK              gpioPortD
#define PIN_SPI_CS                3				//CSN = PD3
#define PORT_SPI_CS               gpioPortD

#define PORT_FEM_CPS		gpioPortD	//FEM_CPS = PD4
#define PIN_FEM_CPS			4
#define PORT_FEM_CTX		gpioPortD	//FEM_CTX = PD8
#define PIN_FEM_CTX			8
#define PORT_DIO0			gpioPortC //gpioPortB //DIO0 = PC13
#define PIN_DIO0			13 //11
#define PORT_DIO1			gpioPortC	//DIO1 = PC5
#define PIN_DIO1			5
#define PORT_DIO2			gpioPortB	//DIO2 = PB12
#define PIN_DIO2			12
#define PORT_DIO3			gpioPortC	//DIO3 = PC12
#define PIN_DIO3			12
#define PORT_DIO4			gpioPortC	//DIO4 = PC7
#define PIN_DIO4			7
#define PORT_DIO5			gpioPortC	//DIO5 = PC6
#define PIN_DIO5			6
#define PORT_GPIO1			gpioPortA
#define PIN_GPIO1			12

#else
#error Missing CFG_sx1276mb1_board/CFG_wimod_board!
#endif

static void setpin (GPIO_Port_TypeDef port, unsigned int pin, u1_t state)
{
	if (state)
		GPIO_PinOutSet(port, pin);
	else
		GPIO_PinOutClear(port, pin);
}

static void hal_io_init ()
{
	/* Chip errata */
	CHIP_Init();

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_HFPER, true);

	GPIO_PinModeSet(PORT_DIO0, PIN_DIO0, gpioModeInput, 0);
	GPIO_PinModeSet(PORT_DIO1, PIN_DIO1, gpioModeInput, 0);
	GPIO_PinModeSet(PORT_DIO2, PIN_DIO2, gpioModeInput, 0);
	GPIO_PinModeSet(PORT_DIO3, PIN_DIO3, gpioModeInput, 0);
	GPIO_PinModeSet(PORT_DIO4, PIN_DIO4, gpioModeInput, 0);
	GPIO_PinModeSet(PORT_DIO5, PIN_DIO5, gpioModeInput, 0);
	GPIO_PinModeSet(PORT_FEM_CPS, PIN_FEM_CPS, gpioModePushPullDrive, 1);
	GPIO_PinModeSet(PORT_FEM_CTX, PIN_FEM_CTX, gpioModePushPullDrive, 0);
}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (u1_t val)
{
    ASSERT(val == 1 || val == 0);
}


// set radio NSS pin to given value
void hal_pin_nss (u1_t val)
{
    setpin(PORT_SPI_CS, PIN_SPI_CS, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val)
{

}

extern void radio_irq_handler(u1_t dio);

// generic EXTI IRQ handler for all channels
//static void EXTI_IRQHandler ()
//{
//    // DIO 0
//    if((EXTI->PR & (1<<DIO0_PIN)) != 0) { // pending
//	EXTI->PR = (1<<DIO0_PIN); // clear irq
//        // invoke radio handler (on IRQ!)
//        radio_irq_handler(0);
//    }
//    // DIO 1
//    if((EXTI->PR & (1<<DIO1_PIN)) != 0) { // pending
//	EXTI->PR = (1<<DIO1_PIN); // clear irq
//        // invoke radio handler (on IRQ!)
//        radio_irq_handler(1);
//    }
//    // DIO 2
//    if((EXTI->PR & (1<<DIO2_PIN)) != 0) { // pending
//	EXTI->PR = (1<<DIO2_PIN); // clear irq
//        // invoke radio handler (on IRQ!)
//        radio_irq_handler(2);
//    }
//#ifdef CFG_EXTI_IRQ_HANDLER
//    // invoke user-defined interrupt handler
//    {
//        extern void CFG_EXTI_IRQ_HANDLER();
//        CFG_EXTI_IRQ_HANDLER();
//    }
//#endif // CFG_EXTI_IRQ_HANDLER
//}

//void EXTI0_IRQHandler () {
//    EXTI_IRQHandler();
//}
//
//void EXTI1_IRQHandler () {
//    EXTI_IRQHandler();
//}
//
//void EXTI2_IRQHandler () {
//    EXTI_IRQHandler();
//}
//
//void EXTI3_IRQHandler () {
//    EXTI_IRQHandler();
//}
//
//void EXTI4_IRQHandler () {
//    EXTI_IRQHandler();
//}
//
//void EXTI9_5_IRQHandler () {
//    EXTI_IRQHandler();
//}
//
//void EXTI15_10_IRQHandler () {
//    EXTI_IRQHandler();
//}

// -----------------------------------------------------------------------------
// SPI

#define GPIO_AF_SPI1        0x05

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
	/* For now, configure SPI for worst case 32MHz clock in order to work for all */
	/* configurations. */
	init.refFreq = 32000000;
	init.baudrate = 800000;//5000000;
	init.msbf = true;	/* Send most significant bit first. */
	USART_InitSync(USART_USED, &init);

	/* Enabling pins and setting location, SPI CS not enable */
	USART_USED->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | USART_LOCATION;

	/* Enabling TX and RX */
	//  USART_USED->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
	/* Clear previous interrupts */
	//  USART_USED->IFC = _USART_IFC_MASK;
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
/******************************************************************************
 * @brief RTC CallBack
 *****************************************************************************/
void RTCDRV_CallBack(void)
{
	HAL.ticks++;
	RTC_IntEnable(RTC_IF_COMP0);	// Enable RTC interrupt from COMP0
	RTC_Enable(true);	//Enable RTC
}

static void hal_time_init ()
{
	SystemCoreClockUpdate(); //Ensure core frequency has been updated
	RTCDRV_Setup(cmuSelect_LFXO, cmuClkDiv_1);
	RTCDRV_Trigger(1, RTCDRV_CallBack);
}

u4_t hal_ticks () {
    hal_disableIRQs();
    u4_t t = HAL.ticks;
    u2_t cnt = RTC->CNT;
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
//    u2_t dt;
//    TIM9->SR &= ~TIM_SR_CC2IF; // clear any pending interrupts
    if(deltaticks(time) < 5) { // event is now (a few ticks ahead)
//        TIM9->DIER &= ~TIM_DIER_CC2IE; // disable IE
        return 1;
    } else { // rewind timer (fully or to exact time))
//        TIM9->CCR2 = TIM9->CNT + dt;   // set comparator
//        TIM9->DIER |= TIM_DIER_CC2IE;  // enable IE
//        TIM9->CCER |= TIM_CCER_CC2E;   // enable capture/compare uint 2
        return 0;
    }
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

void hal_sleep ()
{
	EMU_EnterEM2(false);	// Enter EM2 and wait for RTC interrupt
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
	GPIO_PinModeSet(PORT_GPIO1, PIN_GPIO1, gpioModePushPull, 0);
    debug_led(0);

    // configure USART1 (115200/8N1, tx-only)
//    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
//    hw_cfg_pin(USART_TX_PORT, USART_TX_PIN, GPIOCFG_MODE_ALT|GPIOCFG_OSPEED_40MHz|GPIOCFG_OTYPE_PUPD|GPIOCFG_PUPD_PUP|GPIO_AF_USART1);
//    USART1->BRR = 277; // 115200
//    USART1->CR1 = USART_CR1_UE | USART_CR1_TE; // usart+transmitter enable

    // print banner
    debug_str("\r\n============== DEBUG STARTED ==============\r\n");
}

void debug_led (u1_t val) {
    setpin(PORT_GPIO1, PIN_GPIO1, val);
}

void debug_char (u1_t c) {
//    while( !(USART1->SR & USART_SR_TXE) );
//    USART1->DR = c;
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


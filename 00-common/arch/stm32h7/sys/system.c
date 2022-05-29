#include <stdint.h>
#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/lptimer.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"

uint32_t SystemCoreClock =    480000000UL;
uint32_t rcc_apb1_frequency = 120000000UL;
uint32_t rcc_apb2_frequency = 120000000UL;

#define STDOUT_UART_BAUDRATE    115200UL

#define VECT_TAB_OFFSET         0x00000000

static void STDOUTInit(void)
{
        uint8_t shift = RCC_D2CCIP2R_USART234578SEL_SHIFT;
	uint32_t mask = RCC_D2CCIP2R_USARTSEL_MASK;
	uint32_t reg32 = RCC_D2CCIP2R & ~(mask << shift);
	RCC_D2CCIP2R = reg32 | (RCC_D2CCIP2R_USARTSEL_HSI << shift);

        uint32_t temp = rcc_apb1_frequency;
        rcc_apb1_frequency = 64000000UL;

        rcc_periph_clock_enable(RCC_GPIOD);
        rcc_periph_clock_enable(RCC_USART3);

        gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO8);
        gpio_set_af(GPIOD, GPIO_AF7, GPIO9);

	usart_set_baudrate(USART3, STDOUT_UART_BAUDRATE);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	usart_enable(USART3);

        rcc_apb1_frequency = temp;
}

/*
static void ClockInit(void) {
    struct rcc_pll_config rcc_config = {
	.sysclock_source = RCC_PLL,
	.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,

	uint32_t hse_frequency;
	struct pll_config {
		uint8_t divm;
		uint16_t divn;
		uint8_t divp;
		uint8_t divq;
		uint8_t divr;
	} pll1, pll2, pll3;
	uint8_t core_pre;
	uint8_t hpre;
	uint8_t ppre1;
	uint8_t ppre2;
	uint8_t ppre3;
	uint8_t ppre4;
	uint8_t flash_waitstates;
	enum pwr_vos_scale voltage_scale;
	enum pwr_sys_mode power_mode;
	uint8_t smps_level;
};
}

*/
/*
static void ClockInit(void)
{
        SCB_CPACR |= ((3UL << 10*2)|(3UL << 11*2));

        rcc_osc_on(RCC_LSI);
        rcc_wait_for_osc_ready(RCC_LSI);

	rcc_osc_on(RCC_HSI16);
        rcc_wait_for_osc_ready(RCC_HSI16);

	flash_prefetch_enable();
	flash_set_ws(4);
	flash_dcache_enable();
	flash_icache_enable();

        rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSI16, 4, 40, 0, 0, RCC_PLLCFGR_PLLR_DIV2);
	rcc_osc_on(RCC_PLL);
        rcc_wait_for_osc_ready(RCC_PLL);

        rcc_periph_clock_enable(RCC_SYSCFG);

        rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
	rcc_wait_for_sysclk_status(RCC_PLL);

   //     rcc_ahb_frequency = SystemCoreClock;
//	rcc_apb1_frequency = SystemCoreClock;
	//rcc_apb2_frequency = SystemCoreClock;

        PWR_CR1 &= ~PWR_CR1_LPR;
        PWR_CR1 &= ~PWR_CR1_LPMS_MASK;
        PWR_CR1 |= PWR_CR1_LPMS_STOP_1;
}
*/
/*
static void LPTIMInit(void)
{
        uint8_t shift = RCC_CCIPR_LPTIM1SEL_SHIFT;
	uint32_t mask = RCC_CCIPR_LPTIMxSEL_MASK;
	uint32_t reg32 = RCC_CCIPR & ~(mask << shift);
	RCC_CCIPR = reg32 | (RCC_CCIPR_LPTIMxSEL_LSI << shift);

        rcc_periph_clock_enable(RCC_LPTIM1);

        lptimer_set_internal_clock_source(LPTIM1);
        lptimer_enable_trigger(LPTIM1, LPTIM_CFGR_TRIGEN_SW);
        lptimer_set_prescaler(LPTIM1, LPTIM_CFGR_PRESC_32);
        lptimer_set_period(LPTIM1, 0xffff);

        nvic_set_priority(NVIC_LPTIM1_IRQ, 0xFF);
        nvic_enable_irq(NVIC_LPTIM1_IRQ);
}
*/

void SystemInit(void)
{
        //ClockInit();
        STDOUTInit();
        //LPTIMInit();
}

void system_pre_sleep(uint32_t *expected_time)
{
    /*
        while (usart_get_flag(USART3, USART_FLAG_TC) == 0) {};

        lptimer_enable(LPTIM1);
        lptimer_set_counter(LPTIM1, 0);
        lptimer_set_compare(LPTIM1, *expected_time);
        lptimer_clear_flag(LPTIM1, LPTIM_ICR_CMPMCF);
        lptimer_enable_irq(LPTIM1, LPTIM_IER_CMPMIE);
        lptimer_start_counter(LPTIM1, LPTIM_CR_SNGSTRT);

        SCB_SCR |= ((uint32_t)SCB_SCR_SLEEPDEEP);
        */
}

void system_post_sleep(uint32_t *expected_time)
{
    /*
        SCB_SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP);

        lptimer_disable_irq(LPTIM1, LPTIM_IER_CMPMIE);

        uint16_t elapsed = lptimer_get_counter(LPTIM1);
        lptimer_disable(LPTIM1);

        *expected_time = (elapsed < *expected_time) ? elapsed : *expected_time;

        ClockInit();
        */
}

uint32_t g_runtime_stat_timer_overflow_cntr;

void system_config_runtime_stat_timer( void )
{
        rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_reset_pulse(RST_TIM2);

	timer_set_prescaler(TIM2, (rcc_apb1_frequency / (configTICK_RATE_HZ * 100UL)));
	timer_set_period(TIM2, 65535);
	timer_enable_counter(TIM2);
	timer_enable_irq(TIM2, TIM_DIER_UIE);

        nvic_set_priority(NVIC_TIM2_IRQ, 0x01);
        nvic_enable_irq(NVIC_TIM2_IRQ);
}

uint32_t system_get_runtime_stat_timer_count(void)
{
        return (g_runtime_stat_timer_overflow_cntr << 16) + timer_get_counter(TIM2);
}

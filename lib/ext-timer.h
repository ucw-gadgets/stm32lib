/*
 *	Timer Functions Missing from LibOpenCM3
 *
 *	(c) 2019 Martin Mareš <mj@ucw.cz>
 */

#ifndef _EXT_TIMER_H
#define _EXT_TIMER_H

#include <libopencm3/stm32/timer.h>

static inline bool timer_is_counter_enabled(u32 timer)
{
	return TIM_CR1(timer) & TIM_CR1_CEN;
}

static inline void timer_enable_dma_cc1(u32 timer)
{
	TIM_DIER(timer) |= TIM_DIER_CC1DE;
}

static inline void timer_disable_dma_cc1(u32 timer)
{
	TIM_DIER(timer) &= ~TIM_DIER_CC1DE;
}

#endif

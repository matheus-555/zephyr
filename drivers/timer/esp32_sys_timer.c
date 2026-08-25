/*
 * Copyright (c) 2021-2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc/soc_caps.h>
#include <soc/soc.h>

#include <hal/systimer_hal.h>
#include <hal/systimer_ll.h>
#include <esp_private/systimer.h>
#include <rom/ets_sys.h>
#include <esp_attr.h>
#include <esp_cpu.h>

#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys/clock.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/spinlock.h>

#include "esp32_sys_timer.h"

#define CYC_PER_TICK ((uint32_t)((uint64_t)sys_clock_hw_cycles_per_sec()	\
			      / (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC))
#define MAX_CYC   0xffffffffu
#define MAX_TICKS ((MAX_CYC - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY 1

#if defined(CONFIG_TEST)
const int32_t z_sys_timer_irq_for_test = DT_IRQN(DT_NODELABEL(systimer0));
#endif

#define TICKLESS IS_ENABLED(CONFIG_TICKLESS_KERNEL)

#ifdef CONFIG_SMP
static inline uint32_t sys_timer_alarm_id(void)
{
	return (esp_cpu_get_core_id() == 0) ? SYSTIMER_ALARM_OS_TICK_CORE0
					    : SYSTIMER_ALARM_OS_TICK_CORE1;
}
#else
#define sys_timer_alarm_id() SYSTIMER_ALARM_OS_TICK_CORE0
#endif

#if defined(CONFIG_PM)
static uint64_t systimer_pre_idle;
static uint64_t lptim_pre_idle;
static bool timeout_idle;
#endif

/* Systimer HAL layer object */
static systimer_hal_context_t systimer_hal;

static void set_systimer_alarm(uint64_t time)
{
	uint32_t alarm_id = sys_timer_alarm_id();
	systimer_counter_value_t alarm = {.val = time};

	systimer_hal_select_alarm_mode(&systimer_hal, alarm_id, SYSTIMER_ALARM_MODE_ONESHOT);
	systimer_ll_enable_alarm(systimer_hal.dev, alarm_id, false);
	systimer_ll_set_alarm_target(systimer_hal.dev, alarm_id, alarm.val);
	systimer_ll_apply_alarm_value(systimer_hal.dev, alarm_id);
	systimer_ll_enable_alarm(systimer_hal.dev, alarm_id, true);
	systimer_ll_enable_alarm_int(systimer_hal.dev, alarm_id, true);
}

static uint64_t get_systimer_alarm(void)
{
	return systimer_hal_get_counter_value(&systimer_hal, SYSTIMER_COUNTER_OS_TICK);
}

/* Counter value sampled by sys_clock_disable(), reported by the read below
 * once the hardware is gone.
 */
static uint64_t stopped_cycles;

/*
 * A free-running counter and a one-shot alarm: a COMPARE_ORDERED backend. The
 * counter is 52 bits wide, read back as a 64-bit value. Every SoC this driver
 * builds for defines SYSTIMER_LL_ALARM_MISS_COMPENSATE, so an alarm armed at or
 * behind the counter is raised at once instead of being lost, and the target
 * needs no minimum distance.
 */
#define TIMER_CORE_BACKEND_COMPARE_ORDERED
#define TIMER_CORE_COUNTER_WIDTH 52

static inline uint64_t timer_driver_cycle_get(void)
{
	/* sys_clock_disable() deinitializes the HAL and clears dev; a stray core
	 * read afterwards must not dereference it. Report the value the counter
	 * stopped at, so the elapsed span holds instead of stepping backwards.
	 */
	if (systimer_hal.dev == NULL) {
		return stopped_cycles;
	}
	return get_systimer_alarm();
}

static void timer_driver_set_compare(uint64_t cycles)
{
	/* Same post-sys_clock_disable() guard as timer_driver_cycle_get(): once
	 * the HAL is deinitialized and dev cleared, a stray arm must not
	 * dereference it.
	 */
	if (systimer_hal.dev == NULL) {
		return;
	}

	set_systimer_alarm(cycles);
}

#include "system_timer_generic.h"

static void IRAM_ATTR sys_timer_isr(void *arg)
{
	ARG_UNUSED(arg);
	uint32_t alarm_id = sys_timer_alarm_id();

	systimer_ll_clear_alarm_int(systimer_hal.dev, alarm_id);

	timer_core_announce();
}

void sys_clock_disable(void)
{
	stopped_cycles = get_systimer_alarm();

	systimer_ll_enable_alarm(systimer_hal.dev, SYSTIMER_ALARM_OS_TICK_CORE0, false);
	systimer_ll_enable_alarm_int(systimer_hal.dev, SYSTIMER_ALARM_OS_TICK_CORE0, false);
#if defined(CONFIG_SMP) && (DT_NUM_IRQS(DT_NODELABEL(systimer0)) > 1)
	/* The secondary core's private alarm was armed in smp_timer_init(). */
	systimer_ll_enable_alarm(systimer_hal.dev, SYSTIMER_ALARM_OS_TICK_CORE1, false);
	systimer_ll_enable_alarm_int(systimer_hal.dev, SYSTIMER_ALARM_OS_TICK_CORE1, false);
#endif
	systimer_hal_deinit(&systimer_hal);
}

#if defined(CONFIG_PM)
void sys_clock_idle_enter(uint32_t ticks)
{
	sys_clock_set_timeout(ticks, false);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL) || systimer_hal.dev == NULL) {
		return;
	}

	uint64_t timeout_us = k_ticks_to_us_ceil64(ticks);

	lptim_pre_idle = esp32_lptim_hook_on_lpm_entry(timeout_us);
	systimer_pre_idle = get_systimer_alarm();
	timeout_idle = true;
}

void sys_clock_idle_exit(void)
{
	if (!timeout_idle) {
		return;
	}

	k_spinlock_key_t key = sys_clock_lock();

	uint64_t lptim_now = esp32_lptim_hook_on_lpm_exit();
	uint64_t systimer_now = get_systimer_alarm();
	uint64_t lptim_diff = lptim_now - lptim_pre_idle;
	uint64_t systimer_diff = systimer_now - systimer_pre_idle;
	uint64_t expected_cycles =
		(lptim_diff * sys_clock_hw_cycles_per_sec()) / esp32_lptim_hook_get_freq();
	uint64_t missed_cycles = 0;

	if (expected_cycles > systimer_diff) {
		missed_cycles = expected_cycles - systimer_diff;
	}

	uint64_t new_value =
		systimer_hal_get_counter_value(&systimer_hal, SYSTIMER_COUNTER_OS_TICK) +
		missed_cycles;

	/* Compensate systimer missed cycles while in sleep */
	systimer_ll_set_counter_value(systimer_hal.dev, SYSTIMER_COUNTER_OS_TICK, new_value);
	systimer_ll_apply_counter_value(systimer_hal.dev, SYSTIMER_COUNTER_OS_TICK);

	timeout_idle = false;

	/* Announce OS ticks as systimer remained stalled while in light sleep */
	timer_core_announce_from(key);
}
#endif

static int sys_clock_driver_init(void)
{
	int ret;

	ret = esp_intr_alloc(
		DT_IRQ_BY_IDX(DT_NODELABEL(systimer0), 0, irq),
		ESP_PRIO_TO_FLAGS(DT_IRQ_BY_IDX(DT_NODELABEL(systimer0), 0, priority)) |
			ESP_INT_FLAGS_CHECK(DT_IRQ_BY_IDX(DT_NODELABEL(systimer0), 0, flags)) |
			ESP_INTR_FLAG_IRAM,
		sys_timer_isr, NULL, NULL);

	if (ret != 0) {
		return ret;
	}

	systimer_hal_init(&systimer_hal);
	systimer_hal_connect_alarm_counter(&systimer_hal,
		SYSTIMER_ALARM_OS_TICK_CORE0, SYSTIMER_COUNTER_OS_TICK);

	systimer_hal_enable_counter(&systimer_hal, SYSTIMER_COUNTER_OS_TICK);
	systimer_hal_counter_can_stall_by_cpu(&systimer_hal, SYSTIMER_COUNTER_OS_TICK, 0, true);
#if defined(CONFIG_SMP)
	systimer_hal_counter_can_stall_by_cpu(&systimer_hal, SYSTIMER_COUNTER_OS_TICK, 1, true);
#endif

	/* Seed the announce baseline from the systimer and arm the first tick. */
	timer_core_init();
	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);

#if defined(CONFIG_SMP) && (DT_NUM_IRQS(DT_NODELABEL(systimer0)) > 1)
static struct k_spinlock lock;

/*
 * Runs on a secondary CPU (via smp_init_top()). The shared OS_TICK
 * counter is already running thanks to the boot CPU; connect this
 * core's private alarm to it, route the alarm's interrupt source to
 * this core (esp_intr_alloc() binds to the calling core) and arm the
 * first timeout. DT interrupt index 1 is the core-1 alarm source,
 * so this is only compiled for SoCs describing both alarms.
 */
void smp_timer_init(void)
{
	int ret = esp_intr_alloc(
		DT_IRQ_BY_IDX(DT_NODELABEL(systimer0), 1, irq),
		ESP_PRIO_TO_FLAGS(DT_IRQ_BY_IDX(DT_NODELABEL(systimer0), 1, priority)) |
			ESP_INT_FLAGS_CHECK(DT_IRQ_BY_IDX(DT_NODELABEL(systimer0), 1, flags)) |
			ESP_INTR_FLAG_IRAM,
		sys_timer_isr, NULL, NULL);

	__ASSERT_NO_MSG(ret == 0);
	ARG_UNUSED(ret);

	/*
	 * The boot CPU's tick ISR is already running and performs the same
	 * read-modify-writes on the shared SYSTIMER conf/int_ena registers
	 * under this lock; take it so the alarm setup below cannot lose an
	 * update against the ISR.
	 */
	k_spinlock_key_t key = k_spin_lock(&lock);

	systimer_hal_connect_alarm_counter(&systimer_hal, SYSTIMER_ALARM_OS_TICK_CORE1,
					   SYSTIMER_COUNTER_OS_TICK);

	set_systimer_alarm(get_systimer_alarm() + CYC_PER_TICK);
	k_spin_unlock(&lock, key);
}
#endif /* CONFIG_SMP && DT_NUM_IRQS(systimer0) > 1 */

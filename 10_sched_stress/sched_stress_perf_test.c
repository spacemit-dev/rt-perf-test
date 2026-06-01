/*
 * Scheduler stress / multi-task interference performance test
 *
 * This test runs a 1 kHz control-like realtime loop while several background
 * interference threads consume CPU, yield, sleep, and use IPC. It reports loop
 * jitter, compute cost, deadline misses, IPC wakeup latency, and background
 * worker activity so mixed-load scheduling behavior can be observed.
 */

#include <rtthread.h>
#include <rthw.h>
#include <spacemit_sdk_soc.h>

#define SCHED_STRESS_DURATION_MS       10000U
#define SCHED_STRESS_PERIOD_MS         1U
#define SCHED_STRESS_STACK_SIZE        4096
#define SCHED_STRESS_CTRL_PRIO         7
#define SCHED_STRESS_IPC_PRIO          8
#define SCHED_STRESS_BUSY_PRIO         15
#define SCHED_STRESS_YIELD_PRIO        15
#define SCHED_STRESS_SLEEP_PRIO        15
#define SCHED_STRESS_TIMESLICE         5
#define SCHED_STRESS_PRINT_INTERVAL    1000U
#define SCHED_STRESS_PRINT_PROGRESS    0
#define SCHED_STRESS_BUSY_WORK         1600U
#define SCHED_STRESS_CTRL_WORK         320U
#define SCHED_STRESS_BUSY_YIELD_EVERY  16U
#define SCHED_STRESS_ABS_DIFF(a, b)    (((a) > (b)) ? ((a) - (b)) : ((b) - (a)))

typedef struct {
    rt_uint64_t min_ticks;
    rt_uint64_t max_ticks;
    rt_uint64_t total_ticks;
    rt_uint32_t samples;
} sched_stress_stat_t;

typedef struct {
    rt_uint32_t control_loops;
    rt_uint32_t busy_loops;
    rt_uint32_t yield_loops;
    rt_uint32_t sleep_loops;
    rt_uint32_t ipc_signals;
    rt_uint32_t ipc_wakeups;
    rt_uint32_t ipc_timeouts;
    rt_uint32_t miss_1500us;
    rt_uint32_t miss_2000us;
    rt_uint32_t miss_5000us;
} sched_stress_counters_t;

static volatile rt_bool_t sched_stress_running;
static volatile rt_uint64_t sched_stress_ipc_signal_ticks;
static struct rt_semaphore sched_stress_sem;
static sched_stress_stat_t sched_stress_jitter_stat;
static sched_stress_stat_t sched_stress_period_stat;
static sched_stress_stat_t sched_stress_compute_stat;
static sched_stress_stat_t sched_stress_ipc_stat;
static sched_stress_counters_t sched_stress_counters;

static rt_uint64_t sched_stress_now_ticks(void)
{
    return SysTimer_GetLoadValue();
}

static rt_uint32_t sched_stress_ticks_to_us(rt_uint64_t ticks)
{
    return (rt_uint32_t)(ticks * 1000000ULL / SOC_TIMER_FREQ);
}

static void sched_stress_stat_reset(sched_stress_stat_t *stat)
{
    stat->min_ticks = 0xffffffffffffffffULL;
    stat->max_ticks = 0ULL;
    stat->total_ticks = 0ULL;
    stat->samples = 0U;
}

static void sched_stress_stat_add(sched_stress_stat_t *stat, rt_uint64_t ticks)
{
    if (ticks < stat->min_ticks) {
        stat->min_ticks = ticks;
    }
    if (ticks > stat->max_ticks) {
        stat->max_ticks = ticks;
    }
    stat->total_ticks += ticks;
    stat->samples++;
}

static void sched_stress_print_stat(const char *name, const sched_stress_stat_t *stat)
{
    rt_uint64_t avg_ticks;

    if (stat->samples == 0U) {
        rt_kprintf("[SCHED] %-16s no sample\n", name);
        return;
    }

    avg_ticks = stat->total_ticks / stat->samples;
    rt_kprintf("[SCHED] %-16s samples=%u min=%u us avg=%u us max=%u us\n",
               name,
               stat->samples,
               sched_stress_ticks_to_us(stat->min_ticks),
               sched_stress_ticks_to_us(avg_ticks),
               sched_stress_ticks_to_us(stat->max_ticks));
}

static float sched_stress_control_work(rt_uint32_t seed)
{
    float x = (float)(seed & 0xffU) * 0.00390625f;
    float y = 0.25f;
    rt_uint32_t i;

    for (i = 0; i < SCHED_STRESS_CTRL_WORK; i++) {
        x = x * 1.00017f + y * 0.00091f + (float)(i & 7U) * 0.00013f;
        y = y * 0.99983f - x * 0.00037f + 0.00011f;
        if (x > 8.0f) {
            x *= 0.125f;
        }
        if (y < -8.0f) {
            y *= -0.125f;
        }
    }

    return x + y;
}

static void sched_stress_busy_worker(void *parameter)
{
    volatile rt_uint32_t sink = 0U;
    (void)parameter;

    while (sched_stress_running) {
        rt_uint32_t i;

        for (i = 0; i < SCHED_STRESS_BUSY_WORK; i++) {
            sink += (i ^ sink) + 0x9e3779b9U;
        }
        sched_stress_counters.busy_loops++;
        if ((sched_stress_counters.busy_loops % SCHED_STRESS_BUSY_YIELD_EVERY) == 0U) {
            rt_thread_yield();
        }
    }
}

static void sched_stress_yield_worker(void *parameter)
{
    volatile rt_uint32_t sink = 0U;
    (void)parameter;

    while (sched_stress_running) {
        rt_uint32_t i;

        for (i = 0; i < (SCHED_STRESS_BUSY_WORK / 4U); i++) {
            sink += i + (sink << 1);
        }
        sched_stress_counters.yield_loops++;
        rt_thread_yield();
    }
}

static void sched_stress_sleep_worker(void *parameter)
{
    (void)parameter;

    while (sched_stress_running) {
        sched_stress_counters.sleep_loops++;
        rt_thread_mdelay(2);
    }
}

static void sched_stress_ipc_worker(void *parameter)
{
    (void)parameter;

    while (sched_stress_running) {
        if (rt_sem_take(&sched_stress_sem, rt_tick_from_millisecond(20)) == RT_EOK) {
            rt_uint64_t signal_ticks = sched_stress_ipc_signal_ticks;

            if (signal_ticks != 0ULL) {
                sched_stress_stat_add(&sched_stress_ipc_stat,
                                      sched_stress_now_ticks() - signal_ticks);
                sched_stress_ipc_signal_ticks = 0ULL;
            }
            sched_stress_counters.ipc_wakeups++;
        } else {
            sched_stress_counters.ipc_timeouts++;
        }
    }
}

static void sched_stress_print_progress(rt_uint32_t elapsed_ms)
{
    rt_kprintf("[SCHED] t=%u ms loops=%u miss=%u ipc=%u/%u busy=%u yield=%u sleep=%u\n",
               elapsed_ms,
               sched_stress_counters.control_loops,
               sched_stress_counters.miss_1500us,
               sched_stress_counters.ipc_wakeups,
               sched_stress_counters.ipc_signals,
               sched_stress_counters.busy_loops,
               sched_stress_counters.yield_loops,
               sched_stress_counters.sleep_loops);
}

static void sched_stress_thread_entry(void *parameter)
{
    rt_thread_t busy_thread;
    rt_thread_t yield_thread;
    rt_thread_t sleep_thread;
    rt_thread_t ipc_thread;
    rt_uint64_t period_ticks = (rt_uint64_t)SOC_TIMER_FREQ * SCHED_STRESS_PERIOD_MS / 1000ULL;
    rt_uint64_t last_ticks;
    rt_uint32_t elapsed_ms = 0U;
    rt_uint32_t next_print_ms = SCHED_STRESS_PRINT_INTERVAL;
    volatile float guard = 0.0f;

    (void)parameter;

    rt_memset(&sched_stress_counters, 0, sizeof(sched_stress_counters));
    sched_stress_stat_reset(&sched_stress_jitter_stat);
    sched_stress_stat_reset(&sched_stress_period_stat);
    sched_stress_stat_reset(&sched_stress_compute_stat);
    sched_stress_stat_reset(&sched_stress_ipc_stat);
    sched_stress_ipc_signal_ticks = 0ULL;

    if (rt_sem_init(&sched_stress_sem, "schsem", 0, RT_IPC_FLAG_PRIO) != RT_EOK) {
        rt_kprintf("[SCHED] Failed to init semaphore\n");
        return;
    }

    sched_stress_running = RT_TRUE;

    busy_thread = rt_thread_create("schbusy", sched_stress_busy_worker, RT_NULL,
                                   SCHED_STRESS_STACK_SIZE, SCHED_STRESS_BUSY_PRIO,
                                   SCHED_STRESS_TIMESLICE);
    yield_thread = rt_thread_create("schyield", sched_stress_yield_worker, RT_NULL,
                                    SCHED_STRESS_STACK_SIZE, SCHED_STRESS_YIELD_PRIO,
                                    SCHED_STRESS_TIMESLICE);
    sleep_thread = rt_thread_create("schsleep", sched_stress_sleep_worker, RT_NULL,
                                    SCHED_STRESS_STACK_SIZE, SCHED_STRESS_SLEEP_PRIO,
                                    SCHED_STRESS_TIMESLICE);
    ipc_thread = rt_thread_create("schipc", sched_stress_ipc_worker, RT_NULL,
                                  SCHED_STRESS_STACK_SIZE, SCHED_STRESS_IPC_PRIO,
                                  SCHED_STRESS_TIMESLICE);

    if (busy_thread == RT_NULL || yield_thread == RT_NULL ||
        sleep_thread == RT_NULL || ipc_thread == RT_NULL) {
        rt_kprintf("[SCHED] Failed to create worker thread\n");
        sched_stress_running = RT_FALSE;
        rt_sem_detach(&sched_stress_sem);
        return;
    }

    rt_kprintf("\n[SCHED] Scheduler stress test started\n");
    rt_kprintf("[SCHED] duration=%u ms, control period=%u ms\n",
               SCHED_STRESS_DURATION_MS, SCHED_STRESS_PERIOD_MS);
    rt_kprintf("[SCHED] workers: busy(prio=%u), yield(prio=%u), sleep(prio=%u), ipc(prio=%u)\n\n",
               SCHED_STRESS_BUSY_PRIO,
               SCHED_STRESS_YIELD_PRIO,
               SCHED_STRESS_SLEEP_PRIO,
               SCHED_STRESS_IPC_PRIO);

    rt_thread_startup(busy_thread);
    rt_thread_startup(yield_thread);
    rt_thread_startup(sleep_thread);
    rt_thread_startup(ipc_thread);

    last_ticks = sched_stress_now_ticks();
    while (elapsed_ms < SCHED_STRESS_DURATION_MS) {
        rt_uint64_t loop_start;
        rt_uint64_t loop_end;
        rt_uint64_t now_ticks;
        rt_uint64_t interval_ticks;

        rt_thread_mdelay(SCHED_STRESS_PERIOD_MS);
        loop_start = sched_stress_now_ticks();
        interval_ticks = loop_start - last_ticks;
        sched_stress_stat_add(&sched_stress_period_stat, interval_ticks);
        sched_stress_stat_add(&sched_stress_jitter_stat,
                              SCHED_STRESS_ABS_DIFF(interval_ticks, period_ticks));
        if (interval_ticks > (period_ticks + period_ticks / 2ULL)) {
            sched_stress_counters.miss_1500us++;
        }
        if (interval_ticks > (period_ticks * 2ULL)) {
            sched_stress_counters.miss_2000us++;
        }
        if (interval_ticks > (period_ticks * 5ULL)) {
            sched_stress_counters.miss_5000us++;
        }

        guard += sched_stress_control_work(sched_stress_counters.control_loops);
        loop_end = sched_stress_now_ticks();
        sched_stress_stat_add(&sched_stress_compute_stat, loop_end - loop_start);

        sched_stress_ipc_signal_ticks = sched_stress_now_ticks();
        sched_stress_counters.ipc_signals++;
        rt_sem_release(&sched_stress_sem);

        sched_stress_counters.control_loops++;
        now_ticks = sched_stress_now_ticks();
        last_ticks = now_ticks;
        elapsed_ms += SCHED_STRESS_PERIOD_MS;

        if (SCHED_STRESS_PRINT_PROGRESS && elapsed_ms >= next_print_ms) {
            sched_stress_print_progress(elapsed_ms);
            next_print_ms += SCHED_STRESS_PRINT_INTERVAL;
        }
    }

    sched_stress_running = RT_FALSE;
    rt_sem_release(&sched_stress_sem);
    rt_thread_mdelay(20);
    rt_sem_detach(&sched_stress_sem);

    rt_kprintf("\n[SCHED] Scheduler stress test finished, guard=%d\n", (int)(guard * 1000.0f));
    sched_stress_print_stat("period", &sched_stress_period_stat);
    sched_stress_print_stat("1ms jitter", &sched_stress_jitter_stat);
    sched_stress_print_stat("ctrl compute", &sched_stress_compute_stat);
    sched_stress_print_stat("ipc wakeup", &sched_stress_ipc_stat);
    rt_kprintf("[SCHED] counters loops=%u miss>1.5ms=%u miss>2ms=%u miss>5ms=%u ipc_signal=%u ipc_wakeup=%u ipc_timeout=%u\n",
               sched_stress_counters.control_loops,
               sched_stress_counters.miss_1500us,
               sched_stress_counters.miss_2000us,
               sched_stress_counters.miss_5000us,
               sched_stress_counters.ipc_signals,
               sched_stress_counters.ipc_wakeups,
               sched_stress_counters.ipc_timeouts);
    rt_kprintf("[SCHED] workers busy=%u yield=%u sleep=%u\n\n",
               sched_stress_counters.busy_loops,
               sched_stress_counters.yield_loops,
               sched_stress_counters.sleep_loops);
}

static void sched_stress_perf_test(int argc, char **argv)
{
    rt_thread_t thread;
    (void)argc;
    (void)argv;

    if (rt_thread_find("schedpf") != RT_NULL) {
        rt_kprintf("[SCHED] Background test already running\n");
        return;
    }

    thread = rt_thread_create("schedpf",
                              sched_stress_thread_entry,
                              RT_NULL,
                              SCHED_STRESS_STACK_SIZE,
                              SCHED_STRESS_CTRL_PRIO,
                              SCHED_STRESS_TIMESLICE);
    if (thread == RT_NULL) {
        rt_kprintf("[SCHED] Failed to create background thread\n");
        return;
    }

    rt_kprintf("[SCHED] Background test thread started\n");
    rt_thread_startup(thread);
}

#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(sched_stress_perf_test, sched_perf, run_scheduler_stress_performance_test);

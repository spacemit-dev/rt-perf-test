/*
 * RT latency performance test
 *
 * Includes:
 * - 1 ms periodic task jitter
 * - multi-thread priority preemption latency
 * - semaphore/mutex/mailbox/message queue wakeup latency
 * - interrupt-to-thread wakeup latency approximation by hard timer callback
 * - RT-Thread timer accuracy
 */

#include <rtthread.h>
#include <rthw.h>
#include <spacemit_sdk_soc.h>

#define RTLAT_SAMPLE_COUNT          1000U
#define RTLAT_STACK_SIZE            4096
#define RTLAT_CTRL_PRIO             9
#define RTLAT_WORKER_PRIO           8
#define RTLAT_LOW_PRIO              15
#define RTLAT_TIMESLICE             10
#define RTLAT_1MS_PERIOD_MS         1
#define RTLAT_1MS_WARMUP_COUNT      20U
#define RTLAT_TIMER_PERIOD_MS       1
#define RTLAT_PREEMPT_ROUNDS        1000U
#define RTLAT_IPC_ROUNDS            1000U
#define RTLAT_MBOX_SIZE             8U
#define RTLAT_PRINT_EACH_RECORD     0
#define RTLAT_PRINT_RECORDS         0
#ifdef RT_USING_MESSAGEQUEUE
#define RTLAT_MQ_MSG_SIZE           sizeof(rt_uint32_t)
#define RTLAT_MQ_POOL_SIZE          (RTLAT_MQ_MSG_SIZE * 8U + sizeof(void *) * 8U)
#endif

#define RTLAT_ABS_DIFF(a, b)        (((a) > (b)) ? ((a) - (b)) : ((b) - (a)))

struct rtlat_stat
{
    rt_uint64_t min_ticks;
    rt_uint64_t max_ticks;
    rt_uint64_t total_ticks;
    rt_uint64_t *records;
    rt_uint32_t max_records;
    rt_uint32_t samples;
};

static struct rt_semaphore rtlat_sem;
static struct rt_mutex rtlat_mutex;
static struct rt_mailbox rtlat_mb;
#ifdef RT_USING_MESSAGEQUEUE
static struct rt_messagequeue rtlat_mq;
#endif
static struct rt_timer rtlat_timer;
static struct rt_timer rtlat_irq_timer;
static rt_ubase_t rtlat_mb_pool[RTLAT_MBOX_SIZE];
#ifdef RT_USING_MESSAGEQUEUE
static rt_uint8_t rtlat_mq_pool[RTLAT_MQ_POOL_SIZE];
#endif
static rt_uint64_t rtlat_signal_ticks;
static rt_uint64_t rtlat_irq_signal_ticks;
static rt_uint32_t rtlat_timer_count;
static struct rtlat_stat rtlat_timer_stat;
static volatile rt_bool_t rtlat_running;

static rt_uint64_t rtlat_1ms_records[RTLAT_SAMPLE_COUNT];
static rt_uint64_t rtlat_preempt_records[RTLAT_PREEMPT_ROUNDS];
static rt_uint64_t rtlat_sem_records[RTLAT_IPC_ROUNDS];
static rt_uint64_t rtlat_mutex_records[RTLAT_IPC_ROUNDS];
static rt_uint64_t rtlat_mbox_records[RTLAT_IPC_ROUNDS];
#ifdef RT_USING_MESSAGEQUEUE
static rt_uint64_t rtlat_mq_records[RTLAT_IPC_ROUNDS];
#endif
static rt_uint64_t rtlat_irq_records[RTLAT_SAMPLE_COUNT];
static rt_uint64_t rtlat_timer_records[RTLAT_SAMPLE_COUNT];

static struct rtlat_stat rtlat_1ms_stat = { .records = rtlat_1ms_records, .max_records = RTLAT_SAMPLE_COUNT };
static struct rtlat_stat rtlat_preempt_stat = { .records = rtlat_preempt_records, .max_records = RTLAT_PREEMPT_ROUNDS };
static struct rtlat_stat rtlat_ipc_stats[4] = {
    { .records = rtlat_sem_records, .max_records = RTLAT_IPC_ROUNDS },
    { .records = rtlat_mutex_records, .max_records = RTLAT_IPC_ROUNDS },
    { .records = rtlat_mbox_records, .max_records = RTLAT_IPC_ROUNDS },
#ifdef RT_USING_MESSAGEQUEUE
    { .records = rtlat_mq_records, .max_records = RTLAT_IPC_ROUNDS },
#else
    { .records = RT_NULL, .max_records = 0U },
#endif
};
static struct rtlat_stat rtlat_irq_stat = { .records = rtlat_irq_records, .max_records = RTLAT_SAMPLE_COUNT };

static rt_uint64_t rtlat_now_ticks(void)
{
    return SysTimer_GetLoadValue();
}

static rt_uint32_t rtlat_ticks_to_us(rt_uint64_t ticks)
{
    return (rt_uint32_t)(ticks * 1000000ULL / SOC_TIMER_FREQ);
}

static void rtlat_stat_reset(struct rtlat_stat *stat)
{
    stat->min_ticks = 0xffffffffffffffffULL;
    stat->max_ticks = 0ULL;
    stat->total_ticks = 0ULL;
    stat->samples = 0U;
}

static void rtlat_stat_add(struct rtlat_stat *stat, rt_uint64_t ticks)
{
    if (stat->samples < stat->max_records && stat->records != RT_NULL) {
        stat->records[stat->samples] = ticks;
    }

#if RTLAT_PRINT_EACH_RECORD
    rt_kprintf("[RTLAT] record=%u us\n", rtlat_ticks_to_us(ticks));
#endif

    if (ticks < stat->min_ticks) {
        stat->min_ticks = ticks;
    }
    if (ticks > stat->max_ticks) {
        stat->max_ticks = ticks;
    }
    stat->total_ticks += ticks;
    stat->samples++;
}

static void rtlat_print_stat(const char *name, const struct rtlat_stat *stat)
{
    rt_uint64_t avg_ticks;

    if (stat->samples == 0U) {
        rt_kprintf("[RTLAT] %-18s no sample\n", name);
        return;
    }

    avg_ticks = stat->total_ticks / stat->samples;
    rt_kprintf("[RTLAT] %-18s samples=%u min=%u us avg=%u us max=%u us\n",
               name,
               stat->samples,
               rtlat_ticks_to_us(stat->min_ticks),
               rtlat_ticks_to_us(avg_ticks),
               rtlat_ticks_to_us(stat->max_ticks));
}

static void rtlat_print_records(const char *name, const struct rtlat_stat *stat)
{
    rt_uint32_t i;
    rt_uint32_t count = stat->samples;

    if (stat->records == RT_NULL || stat->max_records == 0U) {
        return;
    }

    if (count > stat->max_records) {
        count = stat->max_records;
    }

    rt_kprintf("[RTLAT] %-18s records(us):", name);
    for (i = 0; i < count; i++) {
        if ((i % 16U) == 0U) {
            rt_kprintf("\n[RTLAT]   %04u:", i);
        }
        rt_kprintf(" %u", rtlat_ticks_to_us(stat->records[i]));
    }
    rt_kprintf("\n");
}

static void rtlat_print_result(const char *name, const struct rtlat_stat *stat)
{
    rtlat_print_stat(name, stat);
#if RTLAT_PRINT_RECORDS
    rtlat_print_records(name, stat);
#else
    (void)name;
#endif
}

static void rtlat_preempt_worker(void *parameter)
{
    struct rtlat_stat *stat = (struct rtlat_stat *)parameter;

    while (rtlat_running) {
        rt_sem_take(&rtlat_sem, RT_WAITING_FOREVER);
        if (!rtlat_running) {
            break;
        }
        if (rtlat_signal_ticks != 0ULL) {
            rtlat_stat_add(stat, rtlat_now_ticks() - rtlat_signal_ticks);
            rtlat_signal_ticks = 0ULL;
        }
    }
}

static void rtlat_low_busy_worker(void *parameter)
{
    volatile rt_uint32_t spin = 0U;
    (void)parameter;

    while (rtlat_running) {
        spin++;
        if ((spin & 0x3ffU) == 0U) {
            rt_thread_yield();
        }
    }
}

static void rtlat_ipc_worker(void *parameter)
{
    struct rtlat_stat *stat = (struct rtlat_stat *)parameter;
    rt_ubase_t value;
#ifdef RT_USING_MESSAGEQUEUE
    rt_uint32_t msg;
#endif

    while (rtlat_running) {
        if (rt_sem_take(&rtlat_sem, RT_WAITING_FOREVER) == RT_EOK) {
            if (!rtlat_running) {
                break;
            }
            rtlat_stat_add(&stat[0], rtlat_now_ticks() - rtlat_signal_ticks);
            rtlat_signal_ticks = 0ULL;
        }
        if (rt_mutex_take(&rtlat_mutex, RT_WAITING_FOREVER) == RT_EOK) {
            if (!rtlat_running) {
                rt_mutex_release(&rtlat_mutex);
                break;
            }
            rtlat_stat_add(&stat[1], rtlat_now_ticks() - rtlat_signal_ticks);
            rtlat_signal_ticks = 0ULL;
            rt_mutex_release(&rtlat_mutex);
        }
        if (rt_mb_recv(&rtlat_mb, &value, RT_WAITING_FOREVER) == RT_EOK) {
            if (!rtlat_running) {
                break;
            }
            rtlat_stat_add(&stat[2], rtlat_now_ticks() - rtlat_signal_ticks);
            rtlat_signal_ticks = 0ULL;
        }
#ifdef RT_USING_MESSAGEQUEUE
        if (rt_mq_recv(&rtlat_mq, &msg, sizeof(msg), RT_WAITING_FOREVER) == RT_EOK) {
            if (!rtlat_running) {
                break;
            }
            rtlat_stat_add(&stat[3], rtlat_now_ticks() - rtlat_signal_ticks);
            rtlat_signal_ticks = 0ULL;
        }
#endif
    }
}

static void rtlat_irq_timer_cb(void *parameter)
{
    (void)parameter;

    rtlat_irq_signal_ticks = rtlat_now_ticks();
    rt_sem_release(&rtlat_sem);
}

static void rtlat_irq_worker(void *parameter)
{
    struct rtlat_stat *stat = (struct rtlat_stat *)parameter;

    while (rtlat_running) {
        rt_sem_take(&rtlat_sem, RT_WAITING_FOREVER);
        if (!rtlat_running) {
            break;
        }
        if (rtlat_irq_signal_ticks != 0ULL) {
            rtlat_stat_add(stat, rtlat_now_ticks() - rtlat_irq_signal_ticks);
            rtlat_irq_signal_ticks = 0ULL;
        }
    }
}

static void rtlat_accuracy_timer_cb(void *parameter)
{
    rt_uint64_t *last_ticks = (rt_uint64_t *)parameter;
    rt_uint64_t now = rtlat_now_ticks();
    rt_uint64_t period_ticks = (rt_uint64_t)SOC_TIMER_FREQ * RTLAT_TIMER_PERIOD_MS / 1000ULL;

    if (*last_ticks != 0ULL) {
        rtlat_stat_add(&rtlat_timer_stat, RTLAT_ABS_DIFF(now - *last_ticks, period_ticks));
    }
    *last_ticks = now;
    rtlat_timer_count++;
}

static void rtlat_test_1ms_jitter(void)
{
    rt_uint64_t last_ticks;
    rt_uint64_t now_ticks;
    rt_uint64_t period_ticks = (rt_uint64_t)SOC_TIMER_FREQ * RTLAT_1MS_PERIOD_MS / 1000ULL;
    rt_uint32_t i;

    rtlat_stat_reset(&rtlat_1ms_stat);

    for (i = 0; i < RTLAT_1MS_WARMUP_COUNT; i++) {
        rt_thread_mdelay(RTLAT_1MS_PERIOD_MS);
    }

    last_ticks = rtlat_now_ticks();

    for (i = 0; i < RTLAT_SAMPLE_COUNT; i++) {
        rt_thread_mdelay(RTLAT_1MS_PERIOD_MS);
        now_ticks = rtlat_now_ticks();
        rtlat_stat_add(&rtlat_1ms_stat, RTLAT_ABS_DIFF(now_ticks - last_ticks, period_ticks));
        last_ticks = now_ticks;
    }
}

static void rtlat_test_preempt(void)
{
    rt_thread_t high_thread;
    rt_thread_t low_thread;
    rt_uint32_t i;

    rtlat_stat_reset(&rtlat_preempt_stat);
    rt_sem_init(&rtlat_sem, "rtlsem", 0, RT_IPC_FLAG_PRIO);
    rtlat_running = RT_TRUE;

    high_thread = rt_thread_create("rtlp_hi", rtlat_preempt_worker, &rtlat_preempt_stat,
                                   RTLAT_STACK_SIZE, RTLAT_WORKER_PRIO, RTLAT_TIMESLICE);
    low_thread = rt_thread_create("rtlp_lo", rtlat_low_busy_worker, RT_NULL,
                                  RTLAT_STACK_SIZE, RTLAT_LOW_PRIO, RTLAT_TIMESLICE);
    if (high_thread == RT_NULL || low_thread == RT_NULL) {
        rt_kprintf("[RTLAT] preempt test thread create failed\n");
        rtlat_running = RT_FALSE;
        rt_sem_detach(&rtlat_sem);
        return;
    }

    rt_thread_startup(high_thread);
    rt_thread_startup(low_thread);
    rt_thread_mdelay(10);

    for (i = 0; i < RTLAT_PREEMPT_ROUNDS; i++) {
        rtlat_signal_ticks = rtlat_now_ticks();
        rt_sem_release(&rtlat_sem);
        while (rtlat_signal_ticks != 0ULL) {
            rt_thread_yield();
        }
    }

    rtlat_running = RT_FALSE;
    rt_sem_release(&rtlat_sem);
    rt_thread_mdelay(10);
    rt_sem_detach(&rtlat_sem);
}

static void rtlat_test_ipc(void)
{
    rt_thread_t worker;
    rt_uint32_t i;
#ifdef RT_USING_MESSAGEQUEUE
    rt_uint32_t msg = 0x5a5a5a5aU;
#endif

    for (i = 0; i < 4U; i++) {
        rtlat_stat_reset(&rtlat_ipc_stats[i]);
    }

    rt_sem_init(&rtlat_sem, "rtlsm2", 0, RT_IPC_FLAG_PRIO);
    rt_mutex_init(&rtlat_mutex, "rtlmtx", RT_IPC_FLAG_PRIO);
    rt_mb_init(&rtlat_mb, "rtlmb", rtlat_mb_pool, RTLAT_MBOX_SIZE, RT_IPC_FLAG_PRIO);
#ifdef RT_USING_MESSAGEQUEUE
    rt_mq_init(&rtlat_mq, "rtlmq", rtlat_mq_pool, RTLAT_MQ_MSG_SIZE,
               sizeof(rtlat_mq_pool), RT_IPC_FLAG_PRIO);
#endif
    rtlat_running = RT_TRUE;

    worker = rt_thread_create("rtlipc", rtlat_ipc_worker, rtlat_ipc_stats,
                              RTLAT_STACK_SIZE, RTLAT_WORKER_PRIO, RTLAT_TIMESLICE);
    if (worker == RT_NULL) {
        rt_kprintf("[RTLAT] ipc test thread create failed\n");
        rtlat_running = RT_FALSE;
        goto out;
    }

    rt_mutex_take(&rtlat_mutex, RT_WAITING_FOREVER);
    rt_thread_startup(worker);
    rt_thread_mdelay(10);

    for (i = 0; i < RTLAT_IPC_ROUNDS; i++) {
        rtlat_signal_ticks = rtlat_now_ticks();
        rt_sem_release(&rtlat_sem);
        while (rtlat_signal_ticks != 0ULL) {
            rt_thread_yield();
        }

        rtlat_signal_ticks = rtlat_now_ticks();
        rt_mutex_release(&rtlat_mutex);
        while (rtlat_signal_ticks != 0ULL) {
            rt_thread_yield();
        }
        rt_mutex_take(&rtlat_mutex, RT_WAITING_FOREVER);

        rtlat_signal_ticks = rtlat_now_ticks();
        rt_mb_send(&rtlat_mb, 1U);
        while (rtlat_signal_ticks != 0ULL) {
            rt_thread_yield();
        }

#ifdef RT_USING_MESSAGEQUEUE
        rtlat_signal_ticks = rtlat_now_ticks();
        rt_mq_send(&rtlat_mq, &msg, sizeof(msg));
        while (rtlat_signal_ticks != 0ULL) {
            rt_thread_yield();
        }
#endif
    }

    rtlat_running = RT_FALSE;
    rtlat_signal_ticks = 0ULL;
    rt_sem_release(&rtlat_sem);
    rt_mutex_release(&rtlat_mutex);
    rt_mb_send(&rtlat_mb, 0U);
#ifdef RT_USING_MESSAGEQUEUE
    rt_mq_send(&rtlat_mq, &msg, sizeof(msg));
#endif
    rt_thread_mdelay(10);

out:
#ifdef RT_USING_MESSAGEQUEUE
    rt_mq_detach(&rtlat_mq);
#endif
    rt_mb_detach(&rtlat_mb);
    rt_mutex_detach(&rtlat_mutex);
    rt_sem_detach(&rtlat_sem);
}

static void rtlat_test_irq_wakeup(void)
{
    rt_thread_t worker;
    rt_tick_t timer_tick = rt_tick_from_millisecond(RTLAT_TIMER_PERIOD_MS);

    rtlat_stat_reset(&rtlat_irq_stat);
    rt_sem_init(&rtlat_sem, "rtlirq", 0, RT_IPC_FLAG_PRIO);
    rtlat_running = RT_TRUE;
    rtlat_irq_signal_ticks = 0ULL;

    worker = rt_thread_create("rtlirq", rtlat_irq_worker, &rtlat_irq_stat,
                              RTLAT_STACK_SIZE, RTLAT_WORKER_PRIO, RTLAT_TIMESLICE);
    if (worker == RT_NULL) {
        rt_kprintf("[RTLAT] irq wakeup test thread create failed\n");
        rtlat_running = RT_FALSE;
        rt_sem_detach(&rtlat_sem);
        return;
    }

    rt_timer_init(&rtlat_irq_timer, "rtlirqtm", rtlat_irq_timer_cb, RT_NULL,
                  timer_tick, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    rt_thread_startup(worker);
    rt_timer_start(&rtlat_irq_timer);

    while (rtlat_irq_stat.samples < RTLAT_SAMPLE_COUNT) {
        rt_thread_mdelay(1);
    }

    rt_timer_stop(&rtlat_irq_timer);
    rtlat_running = RT_FALSE;
    rt_sem_release(&rtlat_sem);
    rt_thread_mdelay(10);
    rt_timer_detach(&rtlat_irq_timer);
    rt_sem_detach(&rtlat_sem);
}

static void rtlat_test_timer_accuracy(void)
{
    rt_uint64_t last_ticks = 0ULL;
    rt_tick_t timer_tick = rt_tick_from_millisecond(RTLAT_TIMER_PERIOD_MS);

    rtlat_timer_stat.records = rtlat_timer_records;
    rtlat_timer_stat.max_records = RTLAT_SAMPLE_COUNT;
    rtlat_stat_reset(&rtlat_timer_stat);
    rtlat_timer_count = 0U;
    rt_timer_init(&rtlat_timer, "rtltimer", rtlat_accuracy_timer_cb, &last_ticks,
                  timer_tick, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    rt_timer_start(&rtlat_timer);

    while (rtlat_timer_count < (RTLAT_SAMPLE_COUNT + 1U)) {
        rt_thread_mdelay(1);
    }

    rt_timer_stop(&rtlat_timer);
    rt_timer_detach(&rtlat_timer);
}

static void rtlat_print_all_results(void)
{
    rt_kprintf("\n[RTLAT] RT latency performance test results\n");
    rtlat_print_result("1ms task jitter", &rtlat_1ms_stat);
    rtlat_print_result("prio preempt", &rtlat_preempt_stat);
    rtlat_print_result("semaphore", &rtlat_ipc_stats[0]);
    rtlat_print_result("mutex", &rtlat_ipc_stats[1]);
    rtlat_print_result("mailbox", &rtlat_ipc_stats[2]);
#ifdef RT_USING_MESSAGEQUEUE
    rtlat_print_result("message queue", &rtlat_ipc_stats[3]);
#else
    rt_kprintf("[RTLAT] %-18s skipped: RT_USING_MESSAGEQUEUE is disabled\n", "message queue");
#endif
    rtlat_print_result("irq wakeup", &rtlat_irq_stat);
    rtlat_print_result("timer accuracy", &rtlat_timer_stat);
}

static void rtlat_thread_entry(void *parameter)
{
    rt_kprintf("\n[RTLAT] RT latency performance test started\n");
    rt_kprintf("[RTLAT] samples=%u, period=%u ms\n\n", RTLAT_SAMPLE_COUNT, RTLAT_1MS_PERIOD_MS);

    rtlat_test_1ms_jitter();
    rtlat_test_preempt();
    rtlat_test_ipc();
    rtlat_test_irq_wakeup();
    rtlat_test_timer_accuracy();
    rtlat_print_all_results();

    rt_kprintf("\n[RTLAT] RT latency performance test finished\n");
    (void)parameter;
}

static void rtlat_perf_test(int argc, char **argv)
{
    rt_thread_t thread;
    (void)argc;
    (void)argv;

    if (rt_thread_find("rtlat") != RT_NULL) {
        rt_kprintf("[RTLAT] Background test already running\n");
        return;
    }

    thread = rt_thread_create("rtlat",
                              rtlat_thread_entry,
                              RT_NULL,
                              RTLAT_STACK_SIZE,
                              RTLAT_CTRL_PRIO,
                              RTLAT_TIMESLICE);
    if (thread == RT_NULL) {
        rt_kprintf("[RTLAT] Failed to create background thread\n");
        return;
    }

    rt_thread_startup(thread);
    rt_kprintf("[RTLAT] Background test thread started\n");
}

MSH_CMD_EXPORT_ALIAS(rtlat_perf_test, rtlat_perf, run_rt_latency_performance_test);

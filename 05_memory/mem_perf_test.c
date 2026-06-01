/*
 * Memory / Cache performance test
 *
 * 用于补齐小核性能测试中的内存访问维度：顺序读写、rt_memcpy、rt_memset、
 * stride 访问、伪随机访问以及 DCache flush/invalidate 开销统计。
 */

#include <rtdevice.h>
#include <rtthread.h>
#include <rthw.h>
#include <spacemit_sdk_soc.h>

#define MEM_PERF_BUFFER_SIZE        (64 * 1024)
#define MEM_PERF_BLOCK_SIZE         (16 * 1024)
#define MEM_PERF_SMALL_BLOCK_SIZE   64
#define MEM_PERF_RANDOM_ENTRIES     4096
#define MEM_PERF_STRIDE_BYTES       64
#define MEM_PERF_PRINT_INTERVAL     100
#define MEM_PERF_MAX_LOOPS          100
#define MEM_PERF_FIXED_PERIOD_MS    10

static rt_uint8_t *mem_src;
static rt_uint8_t *mem_dst;
static rt_uint32_t *mem_random_index;
static volatile rt_uint32_t mem_sink;

typedef struct {
    rt_uint32_t loop_count;
} mem_perf_thread_param_t;

typedef struct {
    rt_uint64_t seq_read_ticks;
    rt_uint64_t seq_write_ticks;
    rt_uint64_t seq_copy_ticks;
    rt_uint64_t memcpy_ticks;
    rt_uint64_t memset_ticks;
    rt_uint64_t small_copy_ticks;
    rt_uint64_t stride_ticks;
    rt_uint64_t random_ticks;
    rt_uint64_t dcache_flush_ticks;
    rt_uint64_t dcache_inval_ticks;
} mem_perf_stat_t;

static void mem_perf_fill_pattern(void)
{
    rt_uint32_t i;

    for (i = 0; i < MEM_PERF_BUFFER_SIZE; i++) {
        mem_src[i] = (rt_uint8_t)((i * 37U + 11U) & 0xffU);
        mem_dst[i] = (rt_uint8_t)((i * 13U + 7U) & 0xffU);
    }

    for (i = 0; i < MEM_PERF_RANDOM_ENTRIES; i++) {
        rt_uint32_t value = i * 1103515245U + 12345U;

        value ^= value >> 16;
        value *= 1664525U;
        mem_random_index[i] = value % MEM_PERF_BUFFER_SIZE;
    }
}

static rt_uint32_t mem_perf_seq_read(void)
{
    rt_uint32_t sum = 0;
    rt_uint32_t i;

    for (i = 0; i < MEM_PERF_BLOCK_SIZE; i++) {
        sum += mem_src[i];
    }

    return sum;
}

static void mem_perf_seq_write(rt_uint32_t seed)
{
    rt_uint32_t i;

    for (i = 0; i < MEM_PERF_BLOCK_SIZE; i++) {
        mem_dst[i] = (rt_uint8_t)(seed + i);
    }
}

static void mem_perf_seq_copy(void)
{
    rt_uint32_t i;

    for (i = 0; i < MEM_PERF_BLOCK_SIZE; i++) {
        mem_dst[i] = mem_src[i];
    }
}

static rt_uint32_t mem_perf_stride_read(void)
{
    rt_uint32_t sum = 0;
    rt_uint32_t i;

    for (i = 0; i < MEM_PERF_BUFFER_SIZE; i += MEM_PERF_STRIDE_BYTES) {
        sum += mem_src[i];
    }

    return sum;
}

static rt_uint32_t mem_perf_random_read(void)
{
    rt_uint32_t sum = 0;
    rt_uint32_t i;

    for (i = 0; i < MEM_PERF_RANDOM_ENTRIES; i++) {
        sum += mem_src[mem_random_index[i]];
    }

    return sum;
}

static void mem_perf_small_copy(void)
{
    rt_uint32_t offset;

    for (offset = 0; offset < MEM_PERF_BLOCK_SIZE; offset += MEM_PERF_SMALL_BLOCK_SIZE) {
        rt_memcpy(&mem_dst[offset], &mem_src[offset], MEM_PERF_SMALL_BLOCK_SIZE);
    }
}

static rt_uint32_t mem_perf_ticks_to_us(rt_uint64_t ticks)
{
    return (rt_uint32_t)(ticks * 1000000ULL / SOC_TIMER_FREQ);
}

static rt_uint32_t mem_perf_kbps(rt_uint32_t bytes, rt_uint32_t us)
{
    if (us == 0U) {
        return 0U;
    }

    return (rt_uint32_t)(((rt_uint64_t)bytes * 1000000ULL) / ((rt_uint64_t)us * 1024ULL));
}

static void mem_perf_print_bandwidth(const char *name, rt_uint32_t bytes_per_loop, rt_uint32_t total_us)
{
    rt_uint32_t avg_us = total_us / MEM_PERF_PRINT_INTERVAL;
    rt_uint32_t kbps = mem_perf_kbps(bytes_per_loop * MEM_PERF_PRINT_INTERVAL, total_us);

    rt_kprintf("[MEM] %-10s total_us=%u, avg_us=%u, throughput=%u KB/s\n",
               name, total_us, avg_us, kbps);
}

static void mem_perf_one_loop(mem_perf_stat_t *stat, rt_uint32_t loop_count)
{
    rt_uint64_t start;
    rt_uint64_t end;

    start = SysTimer_GetLoadValue();
    mem_sink += mem_perf_seq_read();
    end = SysTimer_GetLoadValue();
    stat->seq_read_ticks += end - start;

    start = SysTimer_GetLoadValue();
    mem_perf_seq_write(loop_count);
    end = SysTimer_GetLoadValue();
    stat->seq_write_ticks += end - start;

    start = SysTimer_GetLoadValue();
    mem_perf_seq_copy();
    end = SysTimer_GetLoadValue();
    stat->seq_copy_ticks += end - start;

    start = SysTimer_GetLoadValue();
    rt_memcpy(mem_dst, mem_src, MEM_PERF_BLOCK_SIZE);
    end = SysTimer_GetLoadValue();
    stat->memcpy_ticks += end - start;

    start = SysTimer_GetLoadValue();
    rt_memset(mem_dst, (int)(loop_count & 0xffU), MEM_PERF_BLOCK_SIZE);
    end = SysTimer_GetLoadValue();
    stat->memset_ticks += end - start;

    start = SysTimer_GetLoadValue();
    mem_perf_small_copy();
    end = SysTimer_GetLoadValue();
    stat->small_copy_ticks += end - start;

    start = SysTimer_GetLoadValue();
    mem_sink += mem_perf_stride_read();
    end = SysTimer_GetLoadValue();
    stat->stride_ticks += end - start;

    start = SysTimer_GetLoadValue();
    mem_sink += mem_perf_random_read();
    end = SysTimer_GetLoadValue();
    stat->random_ticks += end - start;

    start = SysTimer_GetLoadValue();
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, mem_dst, MEM_PERF_BLOCK_SIZE);
    end = SysTimer_GetLoadValue();
    stat->dcache_flush_ticks += end - start;

    start = SysTimer_GetLoadValue();
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, mem_dst, MEM_PERF_BLOCK_SIZE);
    end = SysTimer_GetLoadValue();
    stat->dcache_inval_ticks += end - start;
}

static void mem_perf_print_result(const mem_perf_stat_t *stat, rt_uint32_t loop_count)
{
    rt_uint32_t seq_read_us = mem_perf_ticks_to_us(stat->seq_read_ticks);
    rt_uint32_t seq_write_us = mem_perf_ticks_to_us(stat->seq_write_ticks);
    rt_uint32_t seq_copy_us = mem_perf_ticks_to_us(stat->seq_copy_ticks);
    rt_uint32_t memcpy_us = mem_perf_ticks_to_us(stat->memcpy_ticks);
    rt_uint32_t memset_us = mem_perf_ticks_to_us(stat->memset_ticks);
    rt_uint32_t small_copy_us = mem_perf_ticks_to_us(stat->small_copy_ticks);
    rt_uint32_t stride_us = mem_perf_ticks_to_us(stat->stride_ticks);
    rt_uint32_t random_us = mem_perf_ticks_to_us(stat->random_ticks);
    rt_uint32_t flush_us = mem_perf_ticks_to_us(stat->dcache_flush_ticks);
    rt_uint32_t inval_us = mem_perf_ticks_to_us(stat->dcache_inval_ticks);

    rt_kprintf("\n[MEM] Loop %u memory/cache performance result\n", loop_count);
    mem_perf_print_bandwidth("seq_read", MEM_PERF_BLOCK_SIZE, seq_read_us);
    mem_perf_print_bandwidth("seq_write", MEM_PERF_BLOCK_SIZE, seq_write_us);
    mem_perf_print_bandwidth("seq_copy", MEM_PERF_BLOCK_SIZE, seq_copy_us);
    mem_perf_print_bandwidth("memcpy", MEM_PERF_BLOCK_SIZE, memcpy_us);
    mem_perf_print_bandwidth("memset", MEM_PERF_BLOCK_SIZE, memset_us);
    mem_perf_print_bandwidth("smallcopy", MEM_PERF_BLOCK_SIZE, small_copy_us);
    mem_perf_print_bandwidth("stride", MEM_PERF_BUFFER_SIZE / MEM_PERF_STRIDE_BYTES, stride_us);
    mem_perf_print_bandwidth("random", MEM_PERF_RANDOM_ENTRIES, random_us);
    rt_kprintf("[MEM] dcache_flush total_us=%u, avg_us=%u\n",
               flush_us, flush_us / MEM_PERF_PRINT_INTERVAL);
    rt_kprintf("[MEM] dcache_inval total_us=%u, avg_us=%u, sink=%u\n\n",
               inval_us, inval_us / MEM_PERF_PRINT_INTERVAL, mem_sink);
}

static void mem_perf_thread_entry(void *parameter)
{
    mem_perf_thread_param_t *param = (mem_perf_thread_param_t *)parameter;
    mem_perf_stat_t stat;

    rt_memset(&stat, 0, sizeof(stat));
    mem_perf_fill_pattern();

    rt_kprintf("\n[MEM] Background memory/cache performance test started\n");
    rt_kprintf("[MEM] block=%u bytes, buffer=%u bytes, stride=%u bytes\n",
               MEM_PERF_BLOCK_SIZE, MEM_PERF_BUFFER_SIZE, MEM_PERF_STRIDE_BYTES);
    rt_kprintf("[MEM] Includes sequential, memcpy/memset, stride, random and DCache ops\n");
    rt_kprintf("[MEM] Run %u loops then exit\n\n", MEM_PERF_MAX_LOOPS);

    while (param->loop_count < MEM_PERF_MAX_LOOPS) {
        mem_perf_one_loop(&stat, param->loop_count);
        param->loop_count++;

        if ((param->loop_count % MEM_PERF_PRINT_INTERVAL) == 0U) {
            mem_perf_print_result(&stat, param->loop_count);
            rt_memset(&stat, 0, sizeof(stat));
        }

        if (param->loop_count < MEM_PERF_MAX_LOOPS) {
            rt_thread_mdelay(MEM_PERF_FIXED_PERIOD_MS);
        }
    }

    rt_kprintf("[MEM] Test finished after %u loops\n", param->loop_count);
    rt_free(mem_src);
    rt_free(mem_dst);
    rt_free(mem_random_index);
    rt_free(param);
    mem_src = RT_NULL;
    mem_dst = RT_NULL;
    mem_random_index = RT_NULL;
}

static void mem_perf_test(int argc, char **argv)
{
    rt_thread_t thread;
    mem_perf_thread_param_t *param;
    (void)argc;
    (void)argv;

    if (rt_thread_find("mem_perf") != RT_NULL) {
        rt_kprintf("[MEM] Background test already running\n");
        return;
    }

    mem_src = (rt_uint8_t *)rt_malloc(MEM_PERF_BUFFER_SIZE);
    mem_dst = (rt_uint8_t *)rt_malloc(MEM_PERF_BUFFER_SIZE);
    mem_random_index = (rt_uint32_t *)rt_malloc(sizeof(rt_uint32_t) * MEM_PERF_RANDOM_ENTRIES);
    param = (mem_perf_thread_param_t *)rt_malloc(sizeof(mem_perf_thread_param_t));

    if (mem_src == RT_NULL || mem_dst == RT_NULL || mem_random_index == RT_NULL || param == RT_NULL) {
        rt_kprintf("[MEM] Failed to allocate memory\n");
        if (mem_src != RT_NULL) {
            rt_free(mem_src);
            mem_src = RT_NULL;
        }
        if (mem_dst != RT_NULL) {
            rt_free(mem_dst);
            mem_dst = RT_NULL;
        }
        if (mem_random_index != RT_NULL) {
            rt_free(mem_random_index);
            mem_random_index = RT_NULL;
        }
        if (param != RT_NULL) {
            rt_free(param);
        }
        return;
    }

    param->loop_count = 0;
    thread = rt_thread_create("mem_perf",
                              mem_perf_thread_entry,
                              param,
                              4096,
                              10,
                              21);
    if (thread == RT_NULL) {
        rt_kprintf("[MEM] Failed to create background thread\n");
        rt_free(mem_src);
        rt_free(mem_dst);
        rt_free(mem_random_index);
        rt_free(param);
        mem_src = RT_NULL;
        mem_dst = RT_NULL;
        mem_random_index = RT_NULL;
        return;
    }

    rt_thread_startup(thread);
    rt_kprintf("[MEM] Background test thread started\n");
}

MSH_CMD_EXPORT_ALIAS(mem_perf_test, mem_perf, run_memory_cache_performance_test);

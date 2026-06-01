/*
 * FIR/IIR + FFT 信号处理性能测试
 *
 * 用于模拟传感器/音频链路中的分块信号处理：多通道信号生成、FIR 滤波、
 * 二阶 IIR 滤波以及 256 点 radix-2 FFT 幅值统计，评估浮点计算性能。
 */

#include <math.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <spacemit_sdk_soc.h>

#define SIGNAL_BLOCK_SIZE       256
#define SIGNAL_FIR_TAPS         32
#define SIGNAL_CHANNELS         2
#define SIGNAL_TEST_ITERATIONS  5000000
#define SIGNAL_WARMUP_ITERATIONS 1000
#define SIGNAL_FIXED_PERIOD_MS  1
#define SIGNAL_PRINT_INTERVAL   1000
#define SIGNAL_TWO_PI           6.2831853071795864f
#define SIGNAL_SAMPLE_RATE      48000.0f

typedef struct {
    float fir_coeff[SIGNAL_FIR_TAPS];
    float fir_delay[SIGNAL_CHANNELS][SIGNAL_FIR_TAPS];
    float iir_b0;
    float iir_b1;
    float iir_b2;
    float iir_a1;
    float iir_a2;
    float iir_x1[SIGNAL_CHANNELS];
    float iir_x2[SIGNAL_CHANNELS];
    float iir_y1[SIGNAL_CHANNELS];
    float iir_y2[SIGNAL_CHANNELS];
    float input[SIGNAL_CHANNELS][SIGNAL_BLOCK_SIZE];
    float fir_out[SIGNAL_CHANNELS][SIGNAL_BLOCK_SIZE];
    float iir_out[SIGNAL_CHANNELS][SIGNAL_BLOCK_SIZE];
    float fft_real[SIGNAL_BLOCK_SIZE];
    float fft_imag[SIGNAL_BLOCK_SIZE];
    float magnitude[SIGNAL_BLOCK_SIZE / 2];
    float phase;
    float energy;
    float peak;
    float centroid;
} signal_processor_t;

rt_inline float signal_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static rt_int32_t signal_to_fixed_4(float value)
{
    float scaled = value * 10000.0f;

    return (rt_int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void signal_print_fixed_4(rt_int32_t value)
{
    rt_int32_t integer = value / 10000;
    rt_int32_t fraction = value % 10000;

    if (fraction < 0) {
        fraction = -fraction;
    }

    if (value < 0 && integer == 0) {
        rt_kprintf("-");
    }

    rt_kprintf("%d.%04d", integer, fraction);
}

static void signal_init(signal_processor_t *processor)
{
    static const float fir_coeff[SIGNAL_FIR_TAPS] = {
        -0.001822f, -0.002480f, -0.002121f, 0.000000f,
         0.004327f,  0.009113f,  0.010675f, 0.003892f,
        -0.013429f, -0.034211f, -0.041580f, -0.014596f,
         0.065684f,  0.180596f,  0.286927f, 0.330089f,
         0.286927f,  0.180596f,  0.065684f, -0.014596f,
        -0.041580f, -0.034211f, -0.013429f, 0.003892f,
         0.010675f,  0.009113f,  0.004327f, 0.000000f,
        -0.002121f, -0.002480f, -0.001822f, -0.000853f
    };
    int ch;
    int i;

    for (i = 0; i < SIGNAL_FIR_TAPS; i++) {
        processor->fir_coeff[i] = fir_coeff[i];
    }

    for (ch = 0; ch < SIGNAL_CHANNELS; ch++) {
        processor->iir_x1[ch] = 0.0f;
        processor->iir_x2[ch] = 0.0f;
        processor->iir_y1[ch] = 0.0f;
        processor->iir_y2[ch] = 0.0f;
        for (i = 0; i < SIGNAL_FIR_TAPS; i++) {
            processor->fir_delay[ch][i] = 0.0f;
        }
        for (i = 0; i < SIGNAL_BLOCK_SIZE; i++) {
            processor->input[ch][i] = 0.0f;
            processor->fir_out[ch][i] = 0.0f;
            processor->iir_out[ch][i] = 0.0f;
        }
    }

    processor->iir_b0 = 0.020083f;
    processor->iir_b1 = 0.040167f;
    processor->iir_b2 = 0.020083f;
    processor->iir_a1 = -1.561018f;
    processor->iir_a2 = 0.641352f;
    processor->phase = 0.0f;
    processor->energy = 0.0f;
    processor->peak = 0.0f;
    processor->centroid = 0.0f;
}

static void signal_generate_block(signal_processor_t *processor, rt_uint32_t block_index)
{
    float base_phase = processor->phase;
    float freq0 = 780.0f + 120.0f * sinf((float)(block_index % 2048) * 0.00306796f);
    float freq1 = 2400.0f + 320.0f * cosf((float)(block_index % 1024) * 0.00613592f);
    float step0 = SIGNAL_TWO_PI * freq0 / SIGNAL_SAMPLE_RATE;
    float step1 = SIGNAL_TWO_PI * freq1 / SIGNAL_SAMPLE_RATE;
    int i;

    for (i = 0; i < SIGNAL_BLOCK_SIZE; i++) {
        float phase0 = base_phase + step0 * (float)i;
        float phase1 = base_phase * 0.73f + step1 * (float)i;
        float modulation = 0.65f + 0.20f * sinf(phase0 * 0.03125f);
        float noise = (float)(((int)(block_index * 37U + (rt_uint32_t)i * 17U) % 101) - 50) * 0.0009f;
        float sample = modulation * sinf(phase0)
                       + 0.28f * cosf(phase1)
                       + 0.11f * sinf(phase0 * 3.0f + 0.17f)
                       + noise;

        processor->input[0][i] = sample;
        processor->input[1][i] = sample * 0.72f + 0.23f * sinf(phase1 * 0.5f) - noise * 0.5f;
    }

    processor->phase += step0 * (float)SIGNAL_BLOCK_SIZE;
    while (processor->phase >= SIGNAL_TWO_PI) {
        processor->phase -= SIGNAL_TWO_PI;
    }
}

static void signal_fir_filter(signal_processor_t *processor)
{
    int ch;
    int i;
    int tap;

    for (ch = 0; ch < SIGNAL_CHANNELS; ch++) {
        for (i = 0; i < SIGNAL_BLOCK_SIZE; i++) {
            float acc = processor->input[ch][i] * processor->fir_coeff[0];

            for (tap = SIGNAL_FIR_TAPS - 1; tap > 0; tap--) {
                processor->fir_delay[ch][tap] = processor->fir_delay[ch][tap - 1];
                acc += processor->fir_delay[ch][tap] * processor->fir_coeff[tap];
            }
            processor->fir_delay[ch][0] = processor->input[ch][i];
            processor->fir_out[ch][i] = acc;
        }
    }
}

static void signal_iir_filter(signal_processor_t *processor)
{
    int ch;
    int i;

    for (ch = 0; ch < SIGNAL_CHANNELS; ch++) {
        for (i = 0; i < SIGNAL_BLOCK_SIZE; i++) {
            float x0 = processor->fir_out[ch][i];
            float y0 = processor->iir_b0 * x0
                       + processor->iir_b1 * processor->iir_x1[ch]
                       + processor->iir_b2 * processor->iir_x2[ch]
                       - processor->iir_a1 * processor->iir_y1[ch]
                       - processor->iir_a2 * processor->iir_y2[ch];

            processor->iir_x2[ch] = processor->iir_x1[ch];
            processor->iir_x1[ch] = x0;
            processor->iir_y2[ch] = processor->iir_y1[ch];
            processor->iir_y1[ch] = y0;
            processor->iir_out[ch][i] = y0;
        }
    }
}

static void signal_fft_prepare(signal_processor_t *processor)
{
    int i;

    for (i = 0; i < SIGNAL_BLOCK_SIZE; i++) {
        float window = 0.5f - 0.5f * cosf(SIGNAL_TWO_PI * (float)i / (float)(SIGNAL_BLOCK_SIZE - 1));
        processor->fft_real[i] = (processor->iir_out[0][i] + processor->iir_out[1][i] * 0.5f) * window;
        processor->fft_imag[i] = 0.0f;
    }
}

static void signal_fft_compute(signal_processor_t *processor)
{
    unsigned int i;
    unsigned int j = 0;
    unsigned int len;

    for (i = 1; i < SIGNAL_BLOCK_SIZE; i++) {
        unsigned int bit = SIGNAL_BLOCK_SIZE >> 1;

        while ((j & bit) != 0U) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;

        if (i < j) {
            float temp_real = processor->fft_real[i];
            float temp_imag = processor->fft_imag[i];
            processor->fft_real[i] = processor->fft_real[j];
            processor->fft_imag[i] = processor->fft_imag[j];
            processor->fft_real[j] = temp_real;
            processor->fft_imag[j] = temp_imag;
        }
    }

    for (len = 2; len <= SIGNAL_BLOCK_SIZE; len <<= 1) {
        float angle = -SIGNAL_TWO_PI / (float)len;
        float wlen_real = cosf(angle);
        float wlen_imag = sinf(angle);
        unsigned int start;

        for (start = 0; start < SIGNAL_BLOCK_SIZE; start += len) {
            float w_real = 1.0f;
            float w_imag = 0.0f;
            unsigned int half = len >> 1;

            for (i = 0; i < half; i++) {
                unsigned int even = start + i;
                unsigned int odd = even + half;
                float odd_real = processor->fft_real[odd] * w_real - processor->fft_imag[odd] * w_imag;
                float odd_imag = processor->fft_real[odd] * w_imag + processor->fft_imag[odd] * w_real;
                float even_real = processor->fft_real[even];
                float even_imag = processor->fft_imag[even];
                float next_w_real;

                processor->fft_real[even] = even_real + odd_real;
                processor->fft_imag[even] = even_imag + odd_imag;
                processor->fft_real[odd] = even_real - odd_real;
                processor->fft_imag[odd] = even_imag - odd_imag;

                next_w_real = w_real * wlen_real - w_imag * wlen_imag;
                w_imag = w_real * wlen_imag + w_imag * wlen_real;
                w_real = next_w_real;
            }
        }
    }
}

static void signal_analyze_spectrum(signal_processor_t *processor)
{
    float energy = 0.0f;
    float weighted = 0.0f;
    float peak = 0.0f;
    int i;

    for (i = 1; i < SIGNAL_BLOCK_SIZE / 2; i++) {
        float real = processor->fft_real[i];
        float imag = processor->fft_imag[i];
        float mag = sqrtf(real * real + imag * imag);

        processor->magnitude[i] = mag;
        energy += mag;
        weighted += mag * (float)i;
        if (mag > peak) {
            peak = mag;
        }
    }

    processor->energy = energy / (float)(SIGNAL_BLOCK_SIZE / 2);
    processor->peak = peak;
    processor->centroid = (energy > 0.000001f) ? (weighted / energy) : 0.0f;
}

static void signal_additional_compute(signal_processor_t *processor)
{
    float covariance[2][2];
    float feature[4];
    float norm = 0.0f;
    int i;

    covariance[0][0] = 0.0f;
    covariance[0][1] = 0.0f;
    covariance[1][0] = 0.0f;
    covariance[1][1] = 0.0f;

    for (i = 0; i < SIGNAL_BLOCK_SIZE; i++) {
        covariance[0][0] += processor->iir_out[0][i] * processor->iir_out[0][i];
        covariance[0][1] += processor->iir_out[0][i] * processor->iir_out[1][i];
        covariance[1][0] += processor->iir_out[1][i] * processor->iir_out[0][i];
        covariance[1][1] += processor->iir_out[1][i] * processor->iir_out[1][i];
    }

    feature[0] = sqrtf(signal_absf(covariance[0][0]) + 1.0f);
    feature[1] = sqrtf(signal_absf(covariance[1][1]) + 1.0f);
    feature[2] = sinf(processor->centroid * 0.02f) + cosf(processor->peak * 0.001f);
    feature[3] = processor->energy * 0.01f + processor->peak * 0.001f;

    for (i = 0; i < 4; i++) {
        norm += feature[i] * feature[i];
    }
    processor->energy += sqrtf(norm) * 0.0001f;
}

static void signal_step(signal_processor_t *processor, rt_uint32_t block_index)
{
    signal_generate_block(processor, block_index);
    signal_fir_filter(processor);
    signal_iir_filter(processor);
    signal_fft_prepare(processor);
    signal_fft_compute(processor);
    signal_analyze_spectrum(processor);
    signal_additional_compute(processor);
}

typedef struct {
    rt_uint32_t sample_index;
} signal_perf_thread_param_t;

static void signal_perf_thread_entry(void *parameter)
{
    signal_perf_thread_param_t *param = (signal_perf_thread_param_t *)parameter;
    signal_processor_t *processor;
    rt_uint64_t compute_timer_ticks = 0;

    rt_kprintf("\n[SIGNAL] Background FIR/IIR + FFT performance test started\n");
    rt_kprintf("[SIGNAL] compute frequency: 1000 Hz\n");
    rt_kprintf("[SIGNAL] block size: %d samples, FIR taps: %d, FFT points: %d\n",
               SIGNAL_BLOCK_SIZE, SIGNAL_FIR_TAPS, SIGNAL_BLOCK_SIZE);
    rt_kprintf("[SIGNAL] Use another shell command to interact with MSH while test runs\n\n");

    processor = rt_malloc(sizeof(signal_processor_t));
    if (processor == RT_NULL) {
        rt_kprintf("[SIGNAL] Failed to allocate processor state\n");
        rt_free(param);
        return;
    }

    signal_init(processor);

    while (1) {
        rt_uint64_t compute_start;
        rt_uint64_t compute_end;

        compute_start = SysTimer_GetLoadValue();
        signal_step(processor, param->sample_index);
        compute_end = SysTimer_GetLoadValue();
        compute_timer_ticks += compute_end - compute_start;

        param->sample_index++;
        if ((param->sample_index % SIGNAL_PRINT_INTERVAL) == 0) {
            rt_uint64_t compute_total_us;
            rt_uint32_t compute_avg_us;

            compute_total_us = compute_timer_ticks * 1000000ULL / SOC_TIMER_FREQ;
            compute_avg_us = (rt_uint32_t)(compute_total_us / SIGNAL_PRINT_INTERVAL);

            rt_kprintf("[SIGNAL] Sample %u: energy=", param->sample_index);
            signal_print_fixed_4(signal_to_fixed_4(processor->energy));
            rt_kprintf(", peak=");
            signal_print_fixed_4(signal_to_fixed_4(processor->peak));
            rt_kprintf(", centroid=");
            signal_print_fixed_4(signal_to_fixed_4(processor->centroid));
            rt_kprintf(", compute_total_us=%llu, compute_avg_us=%u\n",
                       (unsigned long long)compute_total_us, compute_avg_us);
            compute_timer_ticks = 0;
        }
        rt_thread_mdelay(SIGNAL_FIXED_PERIOD_MS);
    }
}

static void signal_perf_test(int argc, char **argv)
{
    rt_thread_t thread;
    signal_perf_thread_param_t *param;
    (void)argc;
    (void)argv;

    if (rt_thread_find("sig_perf") != RT_NULL) {
        rt_kprintf("[SIGNAL] Background test already running\n");
        return;
    }

    param = rt_malloc(sizeof(signal_perf_thread_param_t));
    if (param == RT_NULL) {
        rt_kprintf("[SIGNAL] Failed to allocate memory for background thread\n");
        return;
    }

    param->sample_index = 0;

    thread = rt_thread_create("sig_perf",
                              signal_perf_thread_entry,
                              param,
                              8192,
                              10,
                              21);
    if (thread == RT_NULL) {
        rt_kprintf("[SIGNAL] Failed to create background thread\n");
        rt_free(param);
        return;
    }

    rt_thread_startup(thread);
    rt_kprintf("[SIGNAL] Background test thread started\n");
}

MSH_CMD_EXPORT_ALIAS(signal_perf_test, signal_perf, run_fir_iir_fft_performance_test);

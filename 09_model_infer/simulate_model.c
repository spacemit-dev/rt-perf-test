/*
 * Model inference performance test
 *
 * Auto-generated GEMM/ReLU workload from dance_0605.onnx, packaged as a
 * background small-core performance test command.
 */

#include <rtdevice.h>
#include <rtthread.h>
#include <spacemit_sdk_soc.h>

#define MODEL_INFER_LAYER_COUNT      4
#define MODEL_INFER_FIXED_PERIOD_MS  1
#define MODEL_INFER_PRINT_INTERVAL   1000
#define MODEL_INFER_PREVIEW_SIZE     8
#define MODEL_INFER_INPUT_MAX        380
#define MODEL_INFER_WEIGHT_MAX       (380 * 512)
#define MODEL_INFER_BIAS_MAX         512
#define MODEL_INFER_BUFFER_MAX       512

typedef struct {
    int m;
    int k;
    int n;
    int has_bias;
    const char *name;
} GemmLayer;

typedef struct {
    rt_uint32_t sample_index;
} model_infer_thread_param_t;

static const GemmLayer g_model_layers[MODEL_INFER_LAYER_COUNT] = {
    {1, 380, 512, 1, "/actor_module/module/module.0/Gemm"},
    {1, 512, 256, 1, "/actor_module/module/module.2/Gemm"},
    {1, 256, 128, 1, "/actor_module/module/module.4/Gemm"},
    {1, 128, 23, 1, "/actor_module/module/module.6/Gemm"},
};

static float g_model_input[MODEL_INFER_INPUT_MAX];
static float g_model_weight[MODEL_INFER_WEIGHT_MAX];
static float g_model_bias[MODEL_INFER_BIAS_MAX];
static float g_model_buffer_a[MODEL_INFER_BUFFER_MAX];
static float g_model_buffer_b[MODEL_INFER_BUFFER_MAX];

static void model_print_fixed_4(rt_int32_t value)
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

static rt_int32_t model_to_fixed_4(float value)
{
    float scaled = value * 10000.0f;

    return (rt_int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void fill_data(float *data, int size, float scale)
{
    int i;

    for (i = 0; i < size; ++i) {
        data[i] = ((float)((i % 23) - 11)) * scale;
    }
}

static void gemm(const GemmLayer *layer, const float *input, const float *weight, const float *bias, float *output)
{
    int row;
    int col;
    int kk;

    for (row = 0; row < layer->m; ++row) {
        for (col = 0; col < layer->n; ++col) {
            float sum = layer->has_bias ? bias[col] : 0.0f;
            for (kk = 0; kk < layer->k; ++kk) {
                sum += input[row * layer->k + kk] * weight[kk * layer->n + col];
            }
            output[row * layer->n + col] = sum;
        }
    }
}

static void relu(float *data, int size)
{
    int i;

    for (i = 0; i < size; ++i) {
        if (data[i] < 0.0f) {
            data[i] = 0.0f;
        }
    }
}

static rt_err_t model_infer_once(float preview[MODEL_INFER_PREVIEW_SIZE], int *preview_size)
{
    float *current = RT_NULL;
    int current_size = 0;
    int i;

    for (i = 0; i < MODEL_INFER_LAYER_COUNT; ++i) {
        const GemmLayer *layer = &g_model_layers[i];
        const int input_size = layer->m * layer->k;
        const int weight_size = layer->k * layer->n;
        const int bias_size = layer->n;
        const int output_size = layer->m * layer->n;
        float *input = RT_NULL;
        float *bias = layer->has_bias ? g_model_bias : RT_NULL;
        float *output = (i & 1) ? g_model_buffer_b : g_model_buffer_a;

        if (input_size > MODEL_INFER_BUFFER_MAX && i != 0) {
            rt_kprintf("[MODEL] input buffer overflow before layer %d\n", i);
            return -RT_ERROR;
        }
        if (i == 0 && input_size > MODEL_INFER_INPUT_MAX) {
            rt_kprintf("[MODEL] first input buffer overflow\n");
            return -RT_ERROR;
        }
        if (weight_size > MODEL_INFER_WEIGHT_MAX || bias_size > MODEL_INFER_BIAS_MAX || output_size > MODEL_INFER_BUFFER_MAX) {
            rt_kprintf("[MODEL] static buffer overflow at layer %d\n", i);
            return -RT_ERROR;
        }

        if (i == 0) {
            input = g_model_input;
            fill_data(input, input_size, 0.03125f);
        } else {
            input = current;
            if (current_size != input_size) {
                rt_kprintf("[MODEL] shape mismatch before layer %d\n", i);
                return -RT_ERROR;
            }
        }

        fill_data(g_model_weight, weight_size, 0.015625f);
        if (bias != RT_NULL) {
            fill_data(bias, bias_size, 0.0078125f);
        }

        gemm(layer, input, g_model_weight, bias, output);
        if (i != MODEL_INFER_LAYER_COUNT - 1) {
            relu(output, output_size);
        }

        current = output;
        current_size = output_size;
    }

    if (current != RT_NULL && current_size > 0) {
        int count = (current_size < MODEL_INFER_PREVIEW_SIZE) ? current_size : MODEL_INFER_PREVIEW_SIZE;

        for (i = 0; i < count; ++i) {
            preview[i] = current[i];
        }
        *preview_size = count;
    } else {
        *preview_size = 0;
    }

    return RT_EOK;
}

static void model_infer_print_workload(void)
{
    rt_uint64_t total_macs = 0;
    rt_uint64_t total_flops = 0;
    int i;

    for (i = 0; i < MODEL_INFER_LAYER_COUNT; ++i) {
        const GemmLayer *layer = &g_model_layers[i];
        rt_uint64_t layer_macs = (rt_uint64_t)layer->m * layer->k * layer->n;
        rt_uint64_t layer_flops = 2ULL * layer->m * layer->k * layer->n;

        total_macs += layer_macs;
        total_flops += layer_flops;
        rt_kprintf("[MODEL] layer %d: %s, Gemm(%d x %d) * (%d x %d), MACs=%llu, FLOPs=%llu\n",
                   i,
                   layer->name,
                   layer->m, layer->k,
                   layer->k, layer->n,
                   (unsigned long long)layer_macs,
                   (unsigned long long)layer_flops);
    }

    rt_kprintf("[MODEL] Total MACs=%llu, Total FLOPs=%llu\n",
               (unsigned long long)total_macs,
               (unsigned long long)total_flops);
}

static void model_infer_perf_thread_entry(void *parameter)
{
    model_infer_thread_param_t *param = (model_infer_thread_param_t *)parameter;
    rt_uint64_t compute_timer_ticks = 0;
    rt_err_t result = RT_EOK;

    rt_kprintf("\n[MODEL] Background model inference performance test started\n");
    rt_kprintf("[MODEL] compute frequency: 1000 Hz\n");
    rt_kprintf("[MODEL] Use another shell command to interact with MSH while test runs\n\n");
    model_infer_print_workload();

    while (1) {
        rt_uint64_t compute_start;
        rt_uint64_t compute_end;
        float preview[MODEL_INFER_PREVIEW_SIZE];
        int preview_size = 0;

        compute_start = SysTimer_GetLoadValue();
        result = model_infer_once(preview, &preview_size);
        compute_end = SysTimer_GetLoadValue();
        compute_timer_ticks += compute_end - compute_start;

        if (result != RT_EOK) {
            rt_kprintf("[MODEL] inference failed, result=%d\n", result);
            rt_free(param);
            return;
        }

        param->sample_index++;
        if ((param->sample_index % MODEL_INFER_PRINT_INTERVAL) == 0) {
            rt_uint64_t compute_total_us;
            rt_uint32_t compute_avg_us;
            int i;

            compute_total_us = compute_timer_ticks * 1000000ULL / SOC_TIMER_FREQ;
            compute_avg_us = (rt_uint32_t)(compute_total_us / MODEL_INFER_PRINT_INTERVAL);

            rt_kprintf("[MODEL] Sample %u: output preview:", param->sample_index);
            for (i = 0; i < preview_size; ++i) {
                rt_kprintf(" ");
                model_print_fixed_4(model_to_fixed_4(preview[i]));
            }
            rt_kprintf(", compute_total_us=%llu, compute_avg_us=%u\n",
                       (unsigned long long)compute_total_us, compute_avg_us);
            compute_timer_ticks = 0;
        }
        rt_thread_mdelay(MODEL_INFER_FIXED_PERIOD_MS);
    }
}

static void model_infer_perf_test(int argc, char **argv)
{
    rt_thread_t thread;
    model_infer_thread_param_t *param;
    (void)argc;
    (void)argv;

    if (rt_thread_find("model_perf") != RT_NULL) {
        rt_kprintf("[MODEL] Background test already running\n");
        return;
    }

    param = rt_malloc(sizeof(model_infer_thread_param_t));
    if (param == RT_NULL) {
        rt_kprintf("[MODEL] Failed to allocate memory for background thread\n");
        return;
    }

    param->sample_index = 0;

    thread = rt_thread_create("model_perf",
                              model_infer_perf_thread_entry,
                              param,
                              8192,
                              10,
                              21);
    if (thread == RT_NULL) {
        rt_kprintf("[MODEL] Failed to create background thread\n");
        rt_free(param);
        return;
    }

    rt_thread_startup(thread);
    rt_kprintf("[MODEL] Background test thread started\n");
}

MSH_CMD_EXPORT_ALIAS(model_infer_perf_test, model_perf, run_model_inference_performance_test);

/*
 * EKF / Kalman 性能测试
 *
 * 用于执行更接近 IMU 融合场景的矩阵版 EKF 迭代，评估浮点计算性能。
 */

#include <math.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <spacemit_sdk_soc.h>

#define EKF_STATE_DIM          9
#define EKF_MEAS_DIM           9
#define EKF_TEST_ITERATIONS    5000000
#define EKF_WARMUP_ITERATIONS  1000
#define EKF_FIXED_PERIOD_MS    1
#define EKF_PRINT_INTERVAL     1000

typedef struct {
    float x[EKF_STATE_DIM];
    float p[EKF_STATE_DIM][EKF_STATE_DIM];
    float q[EKF_STATE_DIM][EKF_STATE_DIM];
    float r[EKF_MEAS_DIM][EKF_MEAS_DIM];
    float dt;
} ekf_filter_t;

rt_inline float fast_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static rt_int32_t ekf_to_fixed_4(float value)
{
    float scaled = value * 10000.0f;

    return (rt_int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void ekf_print_fixed_4(rt_int32_t value)
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

static void ekf_additional_compute(const ekf_filter_t *filter, const float measurement[EKF_MEAS_DIM])
{
    float omega = filter->x[0] * 0.1f - filter->x[1] * 0.05f + filter->x[2] * 0.02f;
    float c = cosf(omega);
    float s = sinf(omega);
    float rot[EKF_STATE_DIM][EKF_STATE_DIM];
    float temp[EKF_STATE_DIM][EKF_STATE_DIM];
    float delta[EKF_STATE_DIM];
    int i, j, k;

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            rot[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    rot[0][0] = c;
    rot[0][1] = -s;
    rot[1][0] = s;
    rot[1][1] = c;

    for (i = 0; i < EKF_STATE_DIM; i++) {
        delta[i] = filter->x[i] * 0.1f + measurement[i % EKF_MEAS_DIM] * 0.05f;
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            temp[i][j] = 0.0f;
            for (k = 0; k < EKF_STATE_DIM; k++) {
                temp[i][j] += rot[i][k] * filter->p[k][j];
            }
        }
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        float acc = 0.0f;
        for (j = 0; j < EKF_STATE_DIM; j++) {
            acc += temp[i][j] * delta[j];
        }
        delta[i] = sqrtf(fast_absf(acc) + 1.0f) + acc * 0.001f;
    }

    (void)delta;
}

static void ekf_init(ekf_filter_t *filter)
{
    int i, j;

    for (i = 0; i < EKF_STATE_DIM; i++) {
        filter->x[i] = 0.0f;
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            filter->p[i][j] = (i == j) ? 1.0f : 0.0f;
            filter->q[i][j] = (i == j) ? (0.0005f + 0.0002f * (float)i) : 0.0f;
        }
    }

    for (i = 0; i < EKF_MEAS_DIM; i++) {
        for (j = 0; j < EKF_MEAS_DIM; j++) {
            filter->r[i][j] = (i == j) ? (0.05f + 0.01f * (float)i) : 0.0f;
        }
    }

    filter->dt = 0.01f;
}

static void ekf_step(ekf_filter_t *filter, const float measurement[EKF_MEAS_DIM])
{
    float dt = filter->dt;
    float x_pred[EKF_STATE_DIM];
    float f[EKF_STATE_DIM][EKF_STATE_DIM];
    float p_pred[EKF_STATE_DIM][EKF_STATE_DIM];
    float temp_fp[EKF_STATE_DIM][EKF_STATE_DIM];
    float y[EKF_MEAS_DIM];
    float hp[EKF_MEAS_DIM][EKF_STATE_DIM];
    float s[EKF_MEAS_DIM][EKF_MEAS_DIM];
    float s_inv[EKF_MEAS_DIM][EKF_MEAS_DIM];
    float pht[EKF_STATE_DIM][EKF_MEAS_DIM];
    float k[EKF_STATE_DIM][EKF_MEAS_DIM];
    float kh[EKF_STATE_DIM][EKF_STATE_DIM];
    float i_kh[EKF_STATE_DIM][EKF_STATE_DIM];
    float innovation_norm = 0.0f;
    int i;
    int j;
    int m;

    for (i = 0; i < EKF_STATE_DIM; i++) {
        float drift = (i + 1 < EKF_STATE_DIM) ? filter->x[i + 1] : filter->x[i] * 0.1f;
        x_pred[i] = filter->x[i] + dt * drift + 0.0005f * fast_absf(filter->x[i]);
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            f[i][j] = (i == j) ? 1.0f : 0.0f;
        }
        if (i + 1 < EKF_STATE_DIM) {
            f[i][i + 1] = dt;
        }
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            temp_fp[i][j] = 0.0f;
            for (m = 0; m < EKF_STATE_DIM; m++) {
                temp_fp[i][j] += f[i][m] * filter->p[m][j];
            }
        }
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            p_pred[i][j] = filter->q[i][j];
            for (m = 0; m < EKF_STATE_DIM; m++) {
                p_pred[i][j] += temp_fp[i][m] * f[j][m];
            }
        }
    }

    for (i = 0; i < EKF_MEAS_DIM; i++) {
        y[i] = measurement[i] - x_pred[i];
    }

    for (i = 0; i < EKF_MEAS_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            hp[i][j] = p_pred[i][j];
        }
    }

    for (i = 0; i < EKF_MEAS_DIM; i++) {
        for (j = 0; j < EKF_MEAS_DIM; j++) {
            s[i][j] = hp[i][j];
        }
        s[i][i] += filter->r[i][i];
    }

    for (i = 0; i < EKF_MEAS_DIM; i++) {
        for (j = 0; j < EKF_MEAS_DIM; j++) {
            s_inv[i][j] = 0.0f;
        }
        s_inv[i][i] = 1.0f / s[i][i];
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_MEAS_DIM; j++) {
            pht[i][j] = p_pred[i][j];
            k[i][j] = pht[i][j] * s_inv[j][j];
        }
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        filter->x[i] = x_pred[i];
        for (m = 0; m < EKF_MEAS_DIM; m++) {
            filter->x[i] += k[i][m] * y[m];
        }
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            kh[i][j] = (i < EKF_STATE_DIM && j < EKF_MEAS_DIM) ? k[i][j] : 0.0f;
        }
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            i_kh[i][j] = -kh[i][j];
        }
        i_kh[i][i] += 1.0f;
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        for (j = 0; j < EKF_STATE_DIM; j++) {
            filter->p[i][j] = 0.0f;
            for (m = 0; m < EKF_STATE_DIM; m++) {
                filter->p[i][j] += i_kh[i][m] * p_pred[m][j];
            }
        }
    }

    for (i = 0; i < EKF_STATE_DIM; i++) {
        innovation_norm += fast_absf(y[i % EKF_MEAS_DIM]);
    }
    for (i = 0; i < EKF_STATE_DIM; i++) {
        filter->x[i] += 0.0001f * innovation_norm * fast_absf(filter->x[i]);
    }

    ekf_additional_compute(filter, measurement);
}

static void generate_measurement(int index, float measurement[EKF_MEAS_DIM])
{
    int saw = index % 97;
    float base = (float)(saw - 48) * 0.0625f;
    float ripple = (float)((index % 11) - 5) * 0.015f;
    float accel_wave = (float)((index % 23) - 11) * 0.0125f;
    float gyro_wave = (float)((index % 13) - 6) * 0.011f;
    float mag_wave = (float)((index % 17) - 8) * 0.008f;
    int i;

    for (i = 0; i < EKF_MEAS_DIM; i++) {
        float factor = 1.0f + 0.1f * (float)i;
        measurement[i] = base * factor
                         + ripple * (0.5f + 0.05f * (float)i)
                         + accel_wave * (((i & 1) != 0) ? 0.8f : 1.2f)
                         + gyro_wave * ((float)(i % 3) * 0.03f)
                         + mag_wave * ((float)(i % 4) * 0.02f)
                         + fast_absf(base) * 0.05f;
    }
}

typedef struct {
    rt_uint32_t sample_index;
} ekf_perf_thread_param_t;

static void ekf_perf_thread_entry(void *parameter)
{
    ekf_perf_thread_param_t *param = (ekf_perf_thread_param_t *)parameter;
    ekf_filter_t filter;
    float measurement[EKF_MEAS_DIM];
    rt_uint64_t compute_timer_ticks = 0;

    rt_kprintf("\n[EKF] Background Kalman performance test started\n");
    rt_kprintf("[EKF] compute frequency: 1000 Hz\n");
    rt_kprintf("[EKF] Use another shell command to interact with MSH while test runs\n\n");

    ekf_init(&filter);

    while (1) {
        rt_uint64_t compute_start;
        rt_uint64_t compute_end;

        compute_start = SysTimer_GetLoadValue();
        generate_measurement((int)param->sample_index, measurement);
        ekf_step(&filter, measurement);
        compute_end = SysTimer_GetLoadValue();
        compute_timer_ticks += compute_end - compute_start;

        param->sample_index++;
        if ((param->sample_index % EKF_PRINT_INTERVAL) == 0) {
            rt_uint64_t compute_total_us;
            rt_uint32_t compute_avg_us;

            compute_total_us = compute_timer_ticks * 1000000ULL / SOC_TIMER_FREQ;
            compute_avg_us = (rt_uint32_t)(compute_total_us / EKF_PRINT_INTERVAL);

            rt_kprintf("[EKF] Sample %u: x[0]=", param->sample_index);
            ekf_print_fixed_4(ekf_to_fixed_4(filter.x[0]));
            rt_kprintf(", x[1]=");
            ekf_print_fixed_4(ekf_to_fixed_4(filter.x[1]));
            rt_kprintf(", x[2]=");
            ekf_print_fixed_4(ekf_to_fixed_4(filter.x[2]));
            rt_kprintf(", compute_total_us=%llu, compute_avg_us=%u\n",
                       (unsigned long long)compute_total_us, compute_avg_us);
            compute_timer_ticks = 0;
        }
        rt_thread_mdelay(EKF_FIXED_PERIOD_MS);
    }
}

static void ekf_perf_test(int argc, char **argv)
{
    rt_thread_t thread;
    ekf_perf_thread_param_t *param;
    (void)argc;
    (void)argv;

    if (rt_thread_find("ekf_perf") != RT_NULL) {
        rt_kprintf("[EKF] Background test already running\n");
        return;
    }

    param = rt_malloc(sizeof(ekf_perf_thread_param_t));
    if (param == RT_NULL) {
        rt_kprintf("[EKF] Failed to allocate memory for background thread\n");
        return;
    }

    param->sample_index = 0;

    thread = rt_thread_create("ekf_perf",
                              ekf_perf_thread_entry,
                              param,
                              8192,
                              10,
                              21);
    if (thread == RT_NULL) {
        rt_kprintf("[EKF] Failed to create background thread\n");
        rt_free(param);
        return;
    }

    rt_thread_startup(thread);
    rt_kprintf("[EKF] Background test thread started\n");
}


MSH_CMD_EXPORT_ALIAS(ekf_perf_test, ekf_perf, run_kalman_filter_performance_test);

/*
 * FOC 控制环计算性能测试
 *
 * 用于模拟 PMSM/BLDC 电机 FOC 电流环：采样生成、Clarke/Park 变换、
 * d/q 轴 PI 控制、反 Park、SVPWM 以及简化电机状态更新，评估浮点计算性能。
 */

#include <math.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <spacemit_sdk_soc.h>

#define FOC_TEST_ITERATIONS    5000000
#define FOC_WARMUP_ITERATIONS  1000
#define FOC_FIXED_PERIOD_MS    1
#define FOC_PRINT_INTERVAL     1000
#define FOC_SQRT3              1.7320508075688772f
#define FOC_INV_SQRT3          0.5773502691896258f
#define FOC_TWO_PI             6.2831853071795864f
#define FOC_DC_BUS_VOLTAGE     24.0f
#define FOC_MAX_VOLTAGE        13.856406f
#define FOC_MOTOR_PHASE_R      0.35f
#define FOC_MOTOR_PHASE_L      0.0007f
#define FOC_MOTOR_FLUX         0.018f
#define FOC_MOTOR_INERTIA      0.00018f
#define FOC_MOTOR_DAMPING      0.00008f
#define FOC_POLE_PAIRS         7.0f

typedef struct {
    float kp;
    float ki;
    float integrator;
    float out_min;
    float out_max;
} foc_pi_t;

typedef struct {
    float theta_e;
    float omega_e;
    float id;
    float iq;
    float vd;
    float vq;
    float duty_a;
    float duty_b;
    float duty_c;
    float id_ref;
    float iq_ref;
    float torque;
    float dt;
    foc_pi_t pi_d;
    foc_pi_t pi_q;
} foc_controller_t;

rt_inline float foc_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float foc_clampf(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static rt_int32_t foc_to_fixed_4(float value)
{
    float scaled = value * 10000.0f;

    return (rt_int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void foc_print_fixed_4(rt_int32_t value)
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

static float foc_wrap_angle(float angle)
{
    while (angle >= FOC_TWO_PI) {
        angle -= FOC_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += FOC_TWO_PI;
    }
    return angle;
}

static void foc_generate_phase_current(int index,
                                       float theta_e,
                                       float id,
                                       float iq,
                                       float *ia,
                                       float *ib,
                                       float *ic)
{
    float sin_t = sinf(theta_e);
    float cos_t = cosf(theta_e);
    float i_alpha = id * cos_t - iq * sin_t;
    float i_beta = id * sin_t + iq * cos_t;
    float ripple = (float)((index % 31) - 15) * 0.0025f;
    float sensor_bias = (float)((index % 17) - 8) * 0.0015f;

    *ia = i_alpha + ripple;
    *ib = -0.5f * i_alpha + 0.5f * FOC_SQRT3 * i_beta - ripple * 0.4f + sensor_bias;
    *ic = -0.5f * i_alpha - 0.5f * FOC_SQRT3 * i_beta - ripple * 0.6f - sensor_bias;
}

static float foc_pi_update(foc_pi_t *pi, float error, float dt)
{
    float proportional = pi->kp * error;
    float candidate_integrator = pi->integrator + pi->ki * error * dt;
    float output;

    candidate_integrator = foc_clampf(candidate_integrator, pi->out_min, pi->out_max);
    output = proportional + candidate_integrator;
    output = foc_clampf(output, pi->out_min, pi->out_max);

    if (output > pi->out_min && output < pi->out_max) {
        pi->integrator = candidate_integrator;
    } else if ((output >= pi->out_max && error < 0.0f) || (output <= pi->out_min && error > 0.0f)) {
        pi->integrator = candidate_integrator;
    }

    return output;
}

static void foc_svpwm(float v_alpha,
                      float v_beta,
                      float vbus,
                      float *duty_a,
                      float *duty_b,
                      float *duty_c)
{
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.5f * FOC_SQRT3 * v_beta;
    float vc = -0.5f * v_alpha - 0.5f * FOC_SQRT3 * v_beta;
    float vmax = va;
    float vmin = va;
    float v_offset;

    if (vb > vmax) {
        vmax = vb;
    }
    if (vc > vmax) {
        vmax = vc;
    }
    if (vb < vmin) {
        vmin = vb;
    }
    if (vc < vmin) {
        vmin = vc;
    }

    v_offset = -0.5f * (vmax + vmin);
    *duty_a = foc_clampf(0.5f + (va + v_offset) / vbus, 0.02f, 0.98f);
    *duty_b = foc_clampf(0.5f + (vb + v_offset) / vbus, 0.02f, 0.98f);
    *duty_c = foc_clampf(0.5f + (vc + v_offset) / vbus, 0.02f, 0.98f);
}

static void foc_additional_compute(const foc_controller_t *ctrl)
{
    float abc[3];
    float transform[3][3];
    float harmonic[3];
    float norm = 0.0f;
    int i;
    int j;

    abc[0] = ctrl->duty_a - 0.5f;
    abc[1] = ctrl->duty_b - 0.5f;
    abc[2] = ctrl->duty_c - 0.5f;

    for (i = 0; i < 3; i++) {
        float angle = ctrl->theta_e + (float)i * 2.0943951023931953f;
        transform[i][0] = cosf(angle);
        transform[i][1] = sinf(angle);
        transform[i][2] = 1.0f + 0.05f * (float)i;
    }

    for (i = 0; i < 3; i++) {
        harmonic[i] = 0.0f;
        for (j = 0; j < 3; j++) {
            harmonic[i] += transform[i][j] * abc[j];
        }
        norm += harmonic[i] * harmonic[i];
    }

    harmonic[0] = sqrtf(norm + foc_absf(ctrl->torque) + 1.0f);
    harmonic[1] = sinf(harmonic[0] * 0.1f) + cosf(norm * 0.05f);
    harmonic[2] = harmonic[0] * harmonic[1] * 0.001f;

    (void)harmonic;
}

static void foc_init(foc_controller_t *ctrl)
{
    ctrl->theta_e = 0.0f;
    ctrl->omega_e = 80.0f;
    ctrl->id = 0.0f;
    ctrl->iq = 0.0f;
    ctrl->vd = 0.0f;
    ctrl->vq = 0.0f;
    ctrl->duty_a = 0.5f;
    ctrl->duty_b = 0.5f;
    ctrl->duty_c = 0.5f;
    ctrl->id_ref = 0.0f;
    ctrl->iq_ref = 2.0f;
    ctrl->torque = 0.0f;
    ctrl->dt = 0.001f;

    ctrl->pi_d.kp = 1.8f;
    ctrl->pi_d.ki = 180.0f;
    ctrl->pi_d.integrator = 0.0f;
    ctrl->pi_d.out_min = -FOC_MAX_VOLTAGE;
    ctrl->pi_d.out_max = FOC_MAX_VOLTAGE;

    ctrl->pi_q.kp = 1.8f;
    ctrl->pi_q.ki = 220.0f;
    ctrl->pi_q.integrator = 0.0f;
    ctrl->pi_q.out_min = -FOC_MAX_VOLTAGE;
    ctrl->pi_q.out_max = FOC_MAX_VOLTAGE;
}

static void foc_step(foc_controller_t *ctrl, int sample_index)
{
    float ia;
    float ib;
    float ic;
    float i_alpha;
    float i_beta;
    float id_meas;
    float iq_meas;
    float sin_t;
    float cos_t;
    float id_error;
    float iq_error;
    float voltage_norm;
    float scale;
    float v_alpha;
    float v_beta;
    float load_torque;
    float electrical_accel;
    float speed_command;

    speed_command = 80.0f + 24.0f * sinf((float)(sample_index % 4096) * 0.00153398f);
    ctrl->iq_ref = 2.0f + 1.2f * sinf((float)(sample_index % 2048) * 0.00306796f);
    ctrl->id_ref = -0.15f * cosf((float)(sample_index % 1024) * 0.00613592f);
    ctrl->omega_e += (speed_command - ctrl->omega_e) * 0.0008f;
    ctrl->theta_e = foc_wrap_angle(ctrl->theta_e + ctrl->omega_e * ctrl->dt);

    foc_generate_phase_current(sample_index, ctrl->theta_e, ctrl->id, ctrl->iq, &ia, &ib, &ic);

    i_alpha = ia;
    i_beta = (ia + 2.0f * ib) * FOC_INV_SQRT3;
    sin_t = sinf(ctrl->theta_e);
    cos_t = cosf(ctrl->theta_e);
    id_meas = i_alpha * cos_t + i_beta * sin_t;
    iq_meas = -i_alpha * sin_t + i_beta * cos_t;

    id_error = ctrl->id_ref - id_meas;
    iq_error = ctrl->iq_ref - iq_meas;
    ctrl->vd = foc_pi_update(&ctrl->pi_d, id_error, ctrl->dt);
    ctrl->vq = foc_pi_update(&ctrl->pi_q, iq_error, ctrl->dt);

    voltage_norm = sqrtf(ctrl->vd * ctrl->vd + ctrl->vq * ctrl->vq);
    if (voltage_norm > FOC_MAX_VOLTAGE) {
        scale = FOC_MAX_VOLTAGE / voltage_norm;
        ctrl->vd *= scale;
        ctrl->vq *= scale;
    }

    v_alpha = ctrl->vd * cos_t - ctrl->vq * sin_t;
    v_beta = ctrl->vd * sin_t + ctrl->vq * cos_t;
    foc_svpwm(v_alpha, v_beta, FOC_DC_BUS_VOLTAGE, &ctrl->duty_a, &ctrl->duty_b, &ctrl->duty_c);

    ctrl->id += ((ctrl->vd - FOC_MOTOR_PHASE_R * ctrl->id + ctrl->omega_e * FOC_MOTOR_PHASE_L * ctrl->iq) / FOC_MOTOR_PHASE_L) * ctrl->dt;
    ctrl->iq += ((ctrl->vq - FOC_MOTOR_PHASE_R * ctrl->iq - ctrl->omega_e * (FOC_MOTOR_PHASE_L * ctrl->id + FOC_MOTOR_FLUX)) / FOC_MOTOR_PHASE_L) * ctrl->dt;
    ctrl->id = foc_clampf(ctrl->id, -20.0f, 20.0f);
    ctrl->iq = foc_clampf(ctrl->iq, -20.0f, 20.0f);

    ctrl->torque = 1.5f * FOC_POLE_PAIRS * FOC_MOTOR_FLUX * ctrl->iq;
    load_torque = 0.04f + 0.015f * sinf((float)(sample_index % 513) * 0.01225f);
    electrical_accel = FOC_POLE_PAIRS * (ctrl->torque - load_torque - FOC_MOTOR_DAMPING * ctrl->omega_e) / FOC_MOTOR_INERTIA;
    ctrl->omega_e += electrical_accel * ctrl->dt;
    ctrl->omega_e = foc_clampf(ctrl->omega_e, 10.0f, 420.0f);

    (void)ic;
    foc_additional_compute(ctrl);
}

typedef struct {
    rt_uint32_t sample_index;
} foc_perf_thread_param_t;

static void foc_perf_thread_entry(void *parameter)
{
    foc_perf_thread_param_t *param = (foc_perf_thread_param_t *)parameter;
    foc_controller_t controller;
    rt_uint64_t compute_timer_ticks = 0;

    rt_kprintf("\n[FOC] Background FOC control-loop performance test started\n");
    rt_kprintf("[FOC] compute frequency: 1000 Hz\n");
    rt_kprintf("[FOC] Use another shell command to interact with MSH while test runs\n\n");

    foc_init(&controller);

    while (1) {
        rt_uint64_t compute_start;
        rt_uint64_t compute_end;

        compute_start = SysTimer_GetLoadValue();
        foc_step(&controller, (int)param->sample_index);
        compute_end = SysTimer_GetLoadValue();
        compute_timer_ticks += compute_end - compute_start;

        param->sample_index++;
        if ((param->sample_index % FOC_PRINT_INTERVAL) == 0) {
            rt_uint64_t compute_total_us;
            rt_uint32_t compute_avg_us;

            compute_total_us = compute_timer_ticks * 1000000ULL / SOC_TIMER_FREQ;
            compute_avg_us = (rt_uint32_t)(compute_total_us / FOC_PRINT_INTERVAL);

            rt_kprintf("[FOC] Sample %u: id=", param->sample_index);
            foc_print_fixed_4(foc_to_fixed_4(controller.id));
            rt_kprintf(", iq=");
            foc_print_fixed_4(foc_to_fixed_4(controller.iq));
            rt_kprintf(", duty_a=");
            foc_print_fixed_4(foc_to_fixed_4(controller.duty_a));
            rt_kprintf(", duty_b=");
            foc_print_fixed_4(foc_to_fixed_4(controller.duty_b));
            rt_kprintf(", duty_c=");
            foc_print_fixed_4(foc_to_fixed_4(controller.duty_c));
            rt_kprintf(", compute_total_us=%llu, compute_avg_us=%u\n",
                       (unsigned long long)compute_total_us, compute_avg_us);
            compute_timer_ticks = 0;
        }
        rt_thread_mdelay(FOC_FIXED_PERIOD_MS);
    }
}

static void foc_perf_test(int argc, char **argv)
{
    rt_thread_t thread;
    foc_perf_thread_param_t *param;
    (void)argc;
    (void)argv;

    if (rt_thread_find("foc_perf") != RT_NULL) {
        rt_kprintf("[FOC] Background test already running\n");
        return;
    }

    param = rt_malloc(sizeof(foc_perf_thread_param_t));
    if (param == RT_NULL) {
        rt_kprintf("[FOC] Failed to allocate memory for background thread\n");
        return;
    }

    param->sample_index = 0;

    thread = rt_thread_create("foc_perf",
                              foc_perf_thread_entry,
                              param,
                              8192,
                              10,
                              21);
    if (thread == RT_NULL) {
        rt_kprintf("[FOC] Failed to create background thread\n");
        rt_free(param);
        return;
    }

    rt_thread_startup(thread);
    rt_kprintf("[FOC] Background test thread started\n");
}

MSH_CMD_EXPORT_ALIAS(foc_perf_test, foc_perf, run_foc_control_loop_performance_test);

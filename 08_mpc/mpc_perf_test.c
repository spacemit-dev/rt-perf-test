/*
 * MPC 四轮小车闭环控制性能测试
 *
 * 模拟四轮差速/滑移小车的 6 状态线性化预测模型，执行短时域 MPC
 * 滚动优化、控制约束、状态反馈闭环和车辆状态更新，评估浮点计算性能。
 */

#include <math.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <spacemit_sdk_soc.h>

#define MPC_STATE_DIM          6
#define MPC_CTRL_DIM           4
#define MPC_HORIZON            10
#define MPC_FIXED_PERIOD_MS    1
#define MPC_PRINT_INTERVAL     1000
#define MPC_DT                 0.01f
#define MPC_TWO_PI             6.2831853071795864f
#define MPC_HALF_TRACK         0.18f
#define MPC_HALF_WHEELBASE     0.22f
#define MPC_WHEEL_RADIUS       0.065f
#define MPC_MAX_WHEEL_SPEED    24.0f
#define MPC_MAX_WHEEL_ACCEL    120.0f
#define MPC_BODY_MASS          12.5f
#define MPC_YAW_INERTIA        0.95f
#define MPC_DRAG_LINEAR        0.28f
#define MPC_DRAG_YAW           0.18f
#define MPC_GRADIENT_ITERS     4

#define MPC_MIN(a, b)          (((a) < (b)) ? (a) : (b))
#define MPC_MAX(a, b)          (((a) > (b)) ? (a) : (b))

/*
 * state: [x, y, yaw, vx_body, vy_body, yaw_rate]
 * ctrl : [front_left, front_right, rear_left, rear_right] wheel angular velocity
 */
typedef struct {
    float x[MPC_STATE_DIM];
    float u[MPC_CTRL_DIM];
    float u_plan[MPC_HORIZON][MPC_CTRL_DIM];
    float q[MPC_STATE_DIM];
    float r[MPC_CTRL_DIM];
    float dt;
} mpc_controller_t;

rt_inline float mpc_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float mpc_clampf(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float mpc_wrap_angle(float angle)
{
    while (angle > 3.1415926535897932f) {
        angle -= MPC_TWO_PI;
    }
    while (angle < -3.1415926535897932f) {
        angle += MPC_TWO_PI;
    }
    return angle;
}

static rt_int32_t mpc_to_fixed_4(float value)
{
    float scaled = value * 10000.0f;

    return (rt_int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void mpc_print_fixed_4(rt_int32_t value)
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

static void mpc_reference(int index, int horizon, float ref[MPC_STATE_DIM])
{
    float t = (float)(index + horizon) * MPC_DT;
    float path_angle = 0.55f * t;
    float radius = 1.2f + 0.15f * sinf(0.13f * t);
    float speed = 0.55f + 0.12f * cosf(0.21f * t);

    ref[0] = radius * cosf(path_angle);
    ref[1] = radius * sinf(path_angle);
    ref[2] = mpc_wrap_angle(path_angle + 1.5707963267948966f + 0.12f * sinf(0.17f * t));
    ref[3] = speed;
    ref[4] = 0.05f * sinf(0.37f * t);
    ref[5] = 0.55f + 0.04f * cosf(0.19f * t);
}

static void mpc_wheel_to_body(const float u[MPC_CTRL_DIM], float *vx, float *vy, float *wz)
{
    float fl = u[0] * MPC_WHEEL_RADIUS;
    float fr = u[1] * MPC_WHEEL_RADIUS;
    float rl = u[2] * MPC_WHEEL_RADIUS;
    float rr = u[3] * MPC_WHEEL_RADIUS;
    float yaw_scale = MPC_HALF_WHEELBASE + MPC_HALF_TRACK;

    *vx = 0.25f * (fl + fr + rl + rr);
    *vy = 0.25f * (-fl + fr + rl - rr);
    *wz = 0.25f * (-fl + fr - rl + rr) / yaw_scale;
}

static void mpc_model_step(const float state[MPC_STATE_DIM],
                           const float u[MPC_CTRL_DIM],
                           float dt,
                           float next[MPC_STATE_DIM])
{
    float cmd_vx;
    float cmd_vy;
    float cmd_wz;
    float c = cosf(state[2]);
    float s = sinf(state[2]);
    float ax;
    float ay;
    float yaw_accel;
    float coupling;

    mpc_wheel_to_body(u, &cmd_vx, &cmd_vy, &cmd_wz);

    coupling = state[5] * state[4];
    ax = (cmd_vx - state[3]) * (18.0f / MPC_BODY_MASS) - MPC_DRAG_LINEAR * state[3] + coupling;
    ay = (cmd_vy - state[4]) * (16.0f / MPC_BODY_MASS) - MPC_DRAG_LINEAR * state[4] - state[5] * state[3];
    yaw_accel = (cmd_wz - state[5]) * (10.0f / MPC_YAW_INERTIA) - MPC_DRAG_YAW * state[5];

    next[0] = state[0] + dt * (c * state[3] - s * state[4]);
    next[1] = state[1] + dt * (s * state[3] + c * state[4]);
    next[2] = mpc_wrap_angle(state[2] + dt * state[5]);
    next[3] = state[3] + dt * ax;
    next[4] = state[4] + dt * ay;
    next[5] = state[5] + dt * yaw_accel;
}

static void mpc_init(mpc_controller_t *ctrl)
{
    int i;
    int j;

    for (i = 0; i < MPC_STATE_DIM; i++) {
        ctrl->x[i] = 0.0f;
        ctrl->q[i] = 1.0f;
    }
    ctrl->q[0] = 8.0f;
    ctrl->q[1] = 8.0f;
    ctrl->q[2] = 5.0f;
    ctrl->q[3] = 1.5f;
    ctrl->q[4] = 1.2f;
    ctrl->q[5] = 1.0f;

    for (i = 0; i < MPC_CTRL_DIM; i++) {
        ctrl->u[i] = 0.0f;
        ctrl->r[i] = 0.025f + 0.005f * (float)i;
    }

    for (i = 0; i < MPC_HORIZON; i++) {
        for (j = 0; j < MPC_CTRL_DIM; j++) {
            ctrl->u_plan[i][j] = 0.0f;
        }
    }

    ctrl->x[0] = 1.0f;
    ctrl->x[1] = -0.2f;
    ctrl->dt = MPC_DT;
}

static float mpc_rollout_cost(const mpc_controller_t *ctrl,
                              const float initial[MPC_STATE_DIM],
                              const float plan[MPC_HORIZON][MPC_CTRL_DIM],
                              int sample_index)
{
    float state[MPC_STATE_DIM];
    float next[MPC_STATE_DIM];
    float cost = 0.0f;
    int i;
    int j;

    for (i = 0; i < MPC_STATE_DIM; i++) {
        state[i] = initial[i];
    }

    for (i = 0; i < MPC_HORIZON; i++) {
        float ref[MPC_STATE_DIM];
        float prev_u[MPC_CTRL_DIM];

        mpc_reference(sample_index, i + 1, ref);
        for (j = 0; j < MPC_CTRL_DIM; j++) {
            prev_u[j] = (i == 0) ? ctrl->u[j] : plan[i - 1][j];
        }

        mpc_model_step(state, plan[i], ctrl->dt, next);

        for (j = 0; j < MPC_STATE_DIM; j++) {
            float err = next[j] - ref[j];
            if (j == 2) {
                err = mpc_wrap_angle(err);
            }
            cost += ctrl->q[j] * err * err;
        }

        for (j = 0; j < MPC_CTRL_DIM; j++) {
            float du = plan[i][j] - prev_u[j];
            cost += ctrl->r[j] * plan[i][j] * plan[i][j] + 0.12f * du * du;
        }

        for (j = 0; j < MPC_STATE_DIM; j++) {
            state[j] = next[j];
        }
    }

    return cost;
}

static void mpc_shift_plan(mpc_controller_t *ctrl)
{
    int i;
    int j;

    for (i = 0; i < MPC_HORIZON - 1; i++) {
        for (j = 0; j < MPC_CTRL_DIM; j++) {
            ctrl->u_plan[i][j] = ctrl->u_plan[i + 1][j];
        }
    }
    for (j = 0; j < MPC_CTRL_DIM; j++) {
        ctrl->u_plan[MPC_HORIZON - 1][j] = ctrl->u_plan[MPC_HORIZON - 2][j];
    }
}

static void mpc_optimize(mpc_controller_t *ctrl, int sample_index)
{
    float base_state[MPC_STATE_DIM];
    float trial[MPC_HORIZON][MPC_CTRL_DIM];
    float step_size = 1.8f;
    int iter;
    int i;
    int j;
    int k;

    for (i = 0; i < MPC_STATE_DIM; i++) {
        base_state[i] = ctrl->x[i];
    }

    for (i = 0; i < MPC_HORIZON; i++) {
        for (j = 0; j < MPC_CTRL_DIM; j++) {
            trial[i][j] = ctrl->u_plan[i][j];
        }
    }

    for (iter = 0; iter < MPC_GRADIENT_ITERS; iter++) {
        float base_cost = mpc_rollout_cost(ctrl, base_state, trial, sample_index);

        for (i = 0; i < MPC_HORIZON; i++) {
            for (j = 0; j < MPC_CTRL_DIM; j++) {
                float original = trial[i][j];
                float plus_cost;
                float minus_cost;
                float gradient;
                float delta;
                float max_delta;

                trial[i][j] = mpc_clampf(original + step_size, -MPC_MAX_WHEEL_SPEED, MPC_MAX_WHEEL_SPEED);
                plus_cost = mpc_rollout_cost(ctrl, base_state, trial, sample_index);
                trial[i][j] = mpc_clampf(original - step_size, -MPC_MAX_WHEEL_SPEED, MPC_MAX_WHEEL_SPEED);
                minus_cost = mpc_rollout_cost(ctrl, base_state, trial, sample_index);
                trial[i][j] = original;

                gradient = (plus_cost - minus_cost) / (2.0f * step_size + 0.0001f);
                delta = -0.018f * gradient;
                max_delta = MPC_MAX_WHEEL_ACCEL * ctrl->dt * (1.0f + 0.15f * (float)iter);
                delta = mpc_clampf(delta, -max_delta, max_delta);
                trial[i][j] = mpc_clampf(original + delta, -MPC_MAX_WHEEL_SPEED, MPC_MAX_WHEEL_SPEED);
            }
        }

        if (mpc_rollout_cost(ctrl, base_state, trial, sample_index) > base_cost) {
            step_size *= 0.55f;
        } else {
            step_size *= 0.85f;
        }
    }

    for (i = 0; i < MPC_HORIZON; i++) {
        for (j = 0; j < MPC_CTRL_DIM; j++) {
            float prev = (i == 0) ? ctrl->u[j] : trial[i - 1][j];
            float max_step = MPC_MAX_WHEEL_ACCEL * ctrl->dt;

            trial[i][j] = mpc_clampf(trial[i][j], prev - max_step, prev + max_step);
            trial[i][j] = mpc_clampf(trial[i][j], -MPC_MAX_WHEEL_SPEED, MPC_MAX_WHEEL_SPEED);
        }
    }

    for (i = 0; i < MPC_HORIZON; i++) {
        for (j = 0; j < MPC_CTRL_DIM; j++) {
            ctrl->u_plan[i][j] = trial[i][j];
        }
    }

    for (k = 0; k < MPC_CTRL_DIM; k++) {
        ctrl->u[k] = ctrl->u_plan[0][k];
    }
}

static void mpc_additional_compute(const mpc_controller_t *ctrl, int sample_index)
{
    float gram[MPC_CTRL_DIM][MPC_CTRL_DIM];
    float response[MPC_CTRL_DIM];
    float energy = 0.0f;
    int i;
    int j;
    int k;

    for (i = 0; i < MPC_CTRL_DIM; i++) {
        for (j = 0; j < MPC_CTRL_DIM; j++) {
            gram[i][j] = 0.0f;
            for (k = 0; k < MPC_HORIZON; k++) {
                gram[i][j] += ctrl->u_plan[k][i] * ctrl->u_plan[k][j] * (0.001f + 0.0001f * (float)(k + 1));
            }
            if (i == j) {
                gram[i][j] += 1.0f;
            }
        }
    }

    for (i = 0; i < MPC_CTRL_DIM; i++) {
        response[i] = 0.0f;
        for (j = 0; j < MPC_CTRL_DIM; j++) {
            response[i] += gram[i][j] * ctrl->u[j];
        }
        energy += response[i] * response[i];
    }

    response[0] = sqrtf(energy + mpc_absf(ctrl->x[0]) + 1.0f);
    response[1] = sinf(response[0] * 0.07f + (float)(sample_index & 0xff) * 0.001f);
    response[2] = cosf(energy * 0.0003f + ctrl->x[2]);
    response[3] = response[0] * response[1] * response[2] * 0.001f;

    (void)response;
}

static void mpc_step(mpc_controller_t *ctrl, int sample_index)
{
    float next[MPC_STATE_DIM];
    float disturbance[MPC_CTRL_DIM];
    int i;

    mpc_optimize(ctrl, sample_index);

    for (i = 0; i < MPC_CTRL_DIM; i++) {
        disturbance[i] = ctrl->u[i] + 0.18f * sinf((float)(sample_index + i * 17) * 0.013f);
    }
    mpc_model_step(ctrl->x, disturbance, ctrl->dt, next);

    for (i = 0; i < MPC_STATE_DIM; i++) {
        ctrl->x[i] = next[i];
    }

    mpc_additional_compute(ctrl, sample_index);
    mpc_shift_plan(ctrl);
}

typedef struct {
    rt_uint32_t sample_index;
} mpc_perf_thread_param_t;

static void mpc_perf_thread_entry(void *parameter)
{
    mpc_perf_thread_param_t *param = (mpc_perf_thread_param_t *)parameter;
    mpc_controller_t ctrl;
    rt_uint64_t compute_timer_ticks = 0;

    rt_kprintf("\n[MPC] Four-wheel closed-loop MPC performance test started\n");
    rt_kprintf("[MPC] compute frequency: 1000 Hz, horizon=%d, states=%d, controls=%d\n",
               MPC_HORIZON, MPC_STATE_DIM, MPC_CTRL_DIM);
    rt_kprintf("[MPC] Use another shell command to interact with MSH while test runs\n\n");

    mpc_init(&ctrl);

    while (1) {
        rt_uint64_t compute_start;
        rt_uint64_t compute_end;

        compute_start = SysTimer_GetLoadValue();
        mpc_step(&ctrl, (int)param->sample_index);
        compute_end = SysTimer_GetLoadValue();
        compute_timer_ticks += compute_end - compute_start;

        param->sample_index++;
        if ((param->sample_index % MPC_PRINT_INTERVAL) == 0) {
            rt_uint64_t compute_total_us;
            rt_uint32_t compute_avg_us;

            compute_total_us = compute_timer_ticks * 1000000ULL / SOC_TIMER_FREQ;
            compute_avg_us = (rt_uint32_t)(compute_total_us / MPC_PRINT_INTERVAL);

            rt_kprintf("[MPC] Sample %u: pos=(", param->sample_index);
            mpc_print_fixed_4(mpc_to_fixed_4(ctrl.x[0]));
            rt_kprintf(", ");
            mpc_print_fixed_4(mpc_to_fixed_4(ctrl.x[1]));
            rt_kprintf("), yaw=");
            mpc_print_fixed_4(mpc_to_fixed_4(ctrl.x[2]));
            rt_kprintf(", u_fl=");
            mpc_print_fixed_4(mpc_to_fixed_4(ctrl.u[0]));
            rt_kprintf(", u_fr=");
            mpc_print_fixed_4(mpc_to_fixed_4(ctrl.u[1]));
            rt_kprintf(", compute_total_us=%llu, compute_avg_us=%u\n",
                       (unsigned long long)compute_total_us, compute_avg_us);
            compute_timer_ticks = 0;
        }
        rt_thread_mdelay(MPC_FIXED_PERIOD_MS);
    }
}

static void mpc_perf_test(int argc, char **argv)
{
    rt_thread_t thread;
    mpc_perf_thread_param_t *param;
    (void)argc;
    (void)argv;

    if (rt_thread_find("mpc_perf") != RT_NULL) {
        rt_kprintf("[MPC] Background test already running\n");
        return;
    }

    param = rt_malloc(sizeof(mpc_perf_thread_param_t));
    if (param == RT_NULL) {
        rt_kprintf("[MPC] Failed to allocate memory for background thread\n");
        return;
    }

    param->sample_index = 0;

    thread = rt_thread_create("mpc_perf",
                              mpc_perf_thread_entry,
                              param,
                              12288,
                              10,
                              21);
    if (thread == RT_NULL) {
        rt_kprintf("[MPC] Failed to create background thread\n");
        rt_free(param);
        return;
    }

    rt_thread_startup(thread);
    rt_kprintf("[MPC] Background test thread started\n");
}

MSH_CMD_EXPORT_ALIAS(mpc_perf_test, mpc_perf, run_four_wheel_mpc_performance_test);

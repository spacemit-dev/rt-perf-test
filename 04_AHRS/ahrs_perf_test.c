/*
 * 四元数 AHRS 姿态解算性能测试
 *
 * 用于模拟 IMU 姿态融合链路：陀螺仪积分、加速度/磁力计归一化、
 * 梯度下降误差修正、四元数归一化以及欧拉角输出，评估浮点计算性能。
 */

#include <math.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <spacemit_sdk_soc.h>

#define AHRS_TEST_ITERATIONS    5000000
#define AHRS_WARMUP_ITERATIONS  1000
#define AHRS_FIXED_PERIOD_MS    1
#define AHRS_PRINT_INTERVAL     1000
#define AHRS_TWO_PI             6.2831853071795864f
#define AHRS_DEG_PER_RAD        57.29577951308232f
#define AHRS_RAD_PER_DEG        0.0174532925199433f
#define AHRS_BETA               0.055f
#define AHRS_KI                 0.002f

typedef struct {
    float w;
    float x;
    float y;
    float z;
} ahrs_quat_t;

typedef struct {
    ahrs_quat_t q;
    float gyro_bias[3];
    float integral_error[3];
    float roll;
    float pitch;
    float yaw;
    float dt;
    float beta;
} ahrs_filter_t;

typedef struct {
    float gyro[3];
    float accel[3];
    float mag[3];
} ahrs_imu_sample_t;

rt_inline float ahrs_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float ahrs_clampf(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static rt_int32_t ahrs_to_fixed_4(float value)
{
    float scaled = value * 10000.0f;

    return (rt_int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void ahrs_print_fixed_4(rt_int32_t value)
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

static float ahrs_inv_norm3(float x, float y, float z)
{
    float norm_sq = x * x + y * y + z * z;

    if (norm_sq <= 0.000001f) {
        return 0.0f;
    }
    return 1.0f / sqrtf(norm_sq);
}

static void ahrs_normalize_quat(ahrs_quat_t *q)
{
    float inv_norm = 1.0f / sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

    q->w *= inv_norm;
    q->x *= inv_norm;
    q->y *= inv_norm;
    q->z *= inv_norm;
}

static void ahrs_quat_multiply(const ahrs_quat_t *a, const ahrs_quat_t *b, ahrs_quat_t *out)
{
    ahrs_quat_t r;

    r.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    r.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    r.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    r.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
    *out = r;
}

static void ahrs_init(ahrs_filter_t *filter)
{
    int i;

    filter->q.w = 1.0f;
    filter->q.x = 0.0f;
    filter->q.y = 0.0f;
    filter->q.z = 0.0f;
    for (i = 0; i < 3; i++) {
        filter->gyro_bias[i] = 0.0f;
        filter->integral_error[i] = 0.0f;
    }
    filter->roll = 0.0f;
    filter->pitch = 0.0f;
    filter->yaw = 0.0f;
    filter->dt = 0.001f;
    filter->beta = AHRS_BETA;
}

static void ahrs_generate_sample(rt_uint32_t index, ahrs_imu_sample_t *sample)
{
    float t = (float)(index % 6283U) * 0.001f;
    float slow = (float)(index % 4096U) * 0.00153398f;
    float roll = 0.42f * sinf(t * 0.71f) + 0.08f * cosf(slow * 0.37f);
    float pitch = 0.31f * cosf(t * 0.53f) + 0.06f * sinf(slow * 0.29f);
    float yaw = t * 0.19f + 0.20f * sinf(t * 0.17f);
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sp = sinf(pitch);
    float cp = cosf(pitch);
    float sy = sinf(yaw);
    float cy = cosf(yaw);
    float gravity[3];
    float magnetic[3];
    float ripple = (float)((int)(index % 37U) - 18) * 0.00075f;
    float noise = (float)((int)(index % 23U) - 11) * 0.00055f;

    sample->gyro[0] = (0.30f * cosf(t * 0.71f) - 0.015f * sinf(slow * 0.37f)) + ripple;
    sample->gyro[1] = (-0.16f * sinf(t * 0.53f) + 0.017f * cosf(slow * 0.29f)) - ripple * 0.6f;
    sample->gyro[2] = 0.19f + 0.034f * cosf(t * 0.17f) + noise;

    gravity[0] = -sp;
    gravity[1] = sr * cp;
    gravity[2] = cr * cp;

    magnetic[0] = cy * cp + 0.23f * (cy * sp * sr - sy * cr);
    magnetic[1] = sy * cp + 0.23f * (sy * sp * sr + cy * cr);
    magnetic[2] = -sp + 0.23f * cp * sr;

    sample->accel[0] = gravity[0] + 0.018f * sinf(t * 3.7f) + noise;
    sample->accel[1] = gravity[1] + 0.014f * cosf(t * 2.9f) - ripple;
    sample->accel[2] = gravity[2] + 0.010f * sinf(t * 4.3f) + ripple * 0.5f;

    sample->mag[0] = magnetic[0] + 0.006f * sinf(t * 1.7f) - noise;
    sample->mag[1] = magnetic[1] + 0.006f * cosf(t * 1.3f) + ripple;
    sample->mag[2] = magnetic[2] + 0.004f * sinf(t * 1.1f) - ripple * 0.3f;
}

static void ahrs_compute_reference(const ahrs_quat_t *q, float gravity[3], float magnetic[3])
{
    float qw = q->w;
    float qx = q->x;
    float qy = q->y;
    float qz = q->z;
    float hx;
    float hy;
    float bx;
    float bz;

    gravity[0] = 2.0f * (qx * qz - qw * qy);
    gravity[1] = 2.0f * (qw * qx + qy * qz);
    gravity[2] = qw * qw - qx * qx - qy * qy + qz * qz;

    hx = 2.0f * (magnetic[0] * (0.5f - qy * qy - qz * qz)
                 + magnetic[1] * (qx * qy - qw * qz)
                 + magnetic[2] * (qx * qz + qw * qy));
    hy = 2.0f * (magnetic[0] * (qx * qy + qw * qz)
                 + magnetic[1] * (0.5f - qx * qx - qz * qz)
                 + magnetic[2] * (qy * qz - qw * qx));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (magnetic[0] * (qx * qz - qw * qy)
                 + magnetic[1] * (qy * qz + qw * qx)
                 + magnetic[2] * (0.5f - qx * qx - qy * qy));

    magnetic[0] = 2.0f * bx * (0.5f - qy * qy - qz * qz) + 2.0f * bz * (qx * qz - qw * qy);
    magnetic[1] = 2.0f * bx * (qx * qy - qw * qz) + 2.0f * bz * (qw * qx + qy * qz);
    magnetic[2] = 2.0f * bx * (qw * qy + qx * qz) + 2.0f * bz * (0.5f - qx * qx - qy * qy);
}

static void ahrs_additional_compute(const ahrs_filter_t *filter, const ahrs_imu_sample_t *sample)
{
    float rot[3][3];
    float vec[3];
    float fused[3];
    float energy = 0.0f;
    int i;
    int j;

    rot[0][0] = 1.0f - 2.0f * (filter->q.y * filter->q.y + filter->q.z * filter->q.z);
    rot[0][1] = 2.0f * (filter->q.x * filter->q.y - filter->q.w * filter->q.z);
    rot[0][2] = 2.0f * (filter->q.x * filter->q.z + filter->q.w * filter->q.y);
    rot[1][0] = 2.0f * (filter->q.x * filter->q.y + filter->q.w * filter->q.z);
    rot[1][1] = 1.0f - 2.0f * (filter->q.x * filter->q.x + filter->q.z * filter->q.z);
    rot[1][2] = 2.0f * (filter->q.y * filter->q.z - filter->q.w * filter->q.x);
    rot[2][0] = 2.0f * (filter->q.x * filter->q.z - filter->q.w * filter->q.y);
    rot[2][1] = 2.0f * (filter->q.y * filter->q.z + filter->q.w * filter->q.x);
    rot[2][2] = 1.0f - 2.0f * (filter->q.x * filter->q.x + filter->q.y * filter->q.y);

    for (i = 0; i < 3; i++) {
        vec[i] = sample->accel[i] * 0.45f + sample->mag[i] * 0.35f + sample->gyro[i] * 0.20f;
    }

    for (i = 0; i < 3; i++) {
        fused[i] = 0.0f;
        for (j = 0; j < 3; j++) {
            fused[i] += rot[i][j] * vec[j];
        }
        energy += fused[i] * fused[i];
    }

    fused[0] = sqrtf(energy + ahrs_absf(filter->roll) + 1.0f);
    fused[1] = sinf(fused[0] * 0.13f) + cosf(filter->yaw * AHRS_RAD_PER_DEG);
    fused[2] = fused[0] * fused[1] * 0.001f + filter->pitch * 0.0001f;

    (void)fused;
}

static void ahrs_step(ahrs_filter_t *filter, const ahrs_imu_sample_t *sample)
{
    float ax = sample->accel[0];
    float ay = sample->accel[1];
    float az = sample->accel[2];
    float mx = sample->mag[0];
    float my = sample->mag[1];
    float mz = sample->mag[2];
    float gx = sample->gyro[0] - filter->gyro_bias[0];
    float gy = sample->gyro[1] - filter->gyro_bias[1];
    float gz = sample->gyro[2] - filter->gyro_bias[2];
    float inv_norm;
    float ref_gravity[3];
    float ref_mag_input[3];
    float ref_mag[3];
    float error[3];
    float correction_norm;
    ahrs_quat_t omega;
    ahrs_quat_t q_dot;
    float qw;
    float qx;
    float qy;
    float qz;
    int i;

    inv_norm = ahrs_inv_norm3(ax, ay, az);
    if (inv_norm > 0.0f) {
        ax *= inv_norm;
        ay *= inv_norm;
        az *= inv_norm;
    }

    inv_norm = ahrs_inv_norm3(mx, my, mz);
    if (inv_norm > 0.0f) {
        mx *= inv_norm;
        my *= inv_norm;
        mz *= inv_norm;
    }

    ref_mag_input[0] = mx;
    ref_mag_input[1] = my;
    ref_mag_input[2] = mz;
    ahrs_compute_reference(&filter->q, ref_gravity, ref_mag_input);
    ref_mag[0] = ref_mag_input[0];
    ref_mag[1] = ref_mag_input[1];
    ref_mag[2] = ref_mag_input[2];

    error[0] = (ay * ref_gravity[2] - az * ref_gravity[1]) + (my * ref_mag[2] - mz * ref_mag[1]);
    error[1] = (az * ref_gravity[0] - ax * ref_gravity[2]) + (mz * ref_mag[0] - mx * ref_mag[2]);
    error[2] = (ax * ref_gravity[1] - ay * ref_gravity[0]) + (mx * ref_mag[1] - my * ref_mag[0]);

    correction_norm = ahrs_inv_norm3(error[0], error[1], error[2]);
    if (correction_norm > 0.0f) {
        error[0] *= correction_norm;
        error[1] *= correction_norm;
        error[2] *= correction_norm;
    }

    for (i = 0; i < 3; i++) {
        filter->integral_error[i] += error[i] * AHRS_KI * filter->dt;
        filter->integral_error[i] = ahrs_clampf(filter->integral_error[i], -0.03f, 0.03f);
    }

    gx += filter->beta * error[0] + filter->integral_error[0];
    gy += filter->beta * error[1] + filter->integral_error[1];
    gz += filter->beta * error[2] + filter->integral_error[2];

    omega.w = 0.0f;
    omega.x = gx;
    omega.y = gy;
    omega.z = gz;
    ahrs_quat_multiply(&filter->q, &omega, &q_dot);

    filter->q.w += 0.5f * q_dot.w * filter->dt;
    filter->q.x += 0.5f * q_dot.x * filter->dt;
    filter->q.y += 0.5f * q_dot.y * filter->dt;
    filter->q.z += 0.5f * q_dot.z * filter->dt;
    ahrs_normalize_quat(&filter->q);

    qw = filter->q.w;
    qx = filter->q.x;
    qy = filter->q.y;
    qz = filter->q.z;
    filter->roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * AHRS_DEG_PER_RAD;
    filter->pitch = asinf(ahrs_clampf(2.0f * (qw * qy - qz * qx), -1.0f, 1.0f)) * AHRS_DEG_PER_RAD;
    filter->yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * AHRS_DEG_PER_RAD;

    ahrs_additional_compute(filter, sample);
}

typedef struct {
    rt_uint32_t sample_index;
} ahrs_perf_thread_param_t;

static void ahrs_perf_thread_entry(void *parameter)
{
    ahrs_perf_thread_param_t *param = (ahrs_perf_thread_param_t *)parameter;
    ahrs_filter_t filter;
    ahrs_imu_sample_t sample;
    rt_uint64_t compute_timer_ticks = 0;

    rt_kprintf("\n[AHRS] Background quaternion AHRS performance test started\n");
    rt_kprintf("[AHRS] compute frequency: 1000 Hz\n");
    rt_kprintf("[AHRS] Use another shell command to interact with MSH while test runs\n\n");

    ahrs_init(&filter);

    while (1) {
        rt_uint64_t compute_start;
        rt_uint64_t compute_end;

        compute_start = SysTimer_GetLoadValue();
        ahrs_generate_sample(param->sample_index, &sample);
        ahrs_step(&filter, &sample);
        compute_end = SysTimer_GetLoadValue();
        compute_timer_ticks += compute_end - compute_start;

        param->sample_index++;
        if ((param->sample_index % AHRS_PRINT_INTERVAL) == 0) {
            rt_uint64_t compute_total_us;
            rt_uint32_t compute_avg_us;

            compute_total_us = compute_timer_ticks * 1000000ULL / SOC_TIMER_FREQ;
            compute_avg_us = (rt_uint32_t)(compute_total_us / AHRS_PRINT_INTERVAL);

            rt_kprintf("[AHRS] Sample %u: roll=", param->sample_index);
            ahrs_print_fixed_4(ahrs_to_fixed_4(filter.roll));
            rt_kprintf(", pitch=");
            ahrs_print_fixed_4(ahrs_to_fixed_4(filter.pitch));
            rt_kprintf(", yaw=");
            ahrs_print_fixed_4(ahrs_to_fixed_4(filter.yaw));
            rt_kprintf(", q0=");
            ahrs_print_fixed_4(ahrs_to_fixed_4(filter.q.w));
            rt_kprintf(", compute_total_us=%llu, compute_avg_us=%u\n",
                       (unsigned long long)compute_total_us, compute_avg_us);
            compute_timer_ticks = 0;
        }
        rt_thread_mdelay(AHRS_FIXED_PERIOD_MS);
    }
}

static void ahrs_perf_test(int argc, char **argv)
{
    rt_thread_t thread;
    ahrs_perf_thread_param_t *param;
    (void)argc;
    (void)argv;

    if (rt_thread_find("ahrs_perf") != RT_NULL) {
        rt_kprintf("[AHRS] Background test already running\n");
        return;
    }

    param = rt_malloc(sizeof(ahrs_perf_thread_param_t));
    if (param == RT_NULL) {
        rt_kprintf("[AHRS] Failed to allocate memory for background thread\n");
        return;
    }

    param->sample_index = 0;

    thread = rt_thread_create("ahrs_perf",
                              ahrs_perf_thread_entry,
                              param,
                              8192,
                              10,
                              21);
    if (thread == RT_NULL) {
        rt_kprintf("[AHRS] Failed to create background thread\n");
        rt_free(param);
        return;
    }

    rt_thread_startup(thread);
    rt_kprintf("[AHRS] Background test thread started\n");
}

MSH_CMD_EXPORT_ALIAS(ahrs_perf_test, ahrs_perf, run_quaternion_ahrs_performance_test);

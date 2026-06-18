# RT 性能测试套件说明

本目录提供 Spacemit K3 小核 ESOS/RT-Thread 侧的实时性能测试用例，用于覆盖典型控制算法、信号处理、内存访问、实时调度、大小核通信和模型推理等场景。测试用例主要通过 MSH 命令启动，便于在目标板上按需执行和组合测试。

## 目录结构

| 目录/文件 | MSH 命令 | 说明 |
| --- | --- | --- |
| `01_ekf/` | `ekf_perf` | EKF/Kalman 矩阵迭代性能测试，模拟 IMU 融合等浮点计算场景 |
| `02_foc/` | `foc_perf` | FOC 电流环控制性能测试，包含 Clarke/Park、PI、SVPWM 和简化电机状态更新 |
| `03_signal/` | `signal_perf` | FIR/IIR + FFT 信号处理性能测试，模拟多通道传感器或音频链路 |
| `04_AHRS/` | `ahrs_perf` | 四元数 AHRS 姿态解算性能测试 |
| `05_memory/` | `mem_perf` | 内存/缓存访问性能测试，评估拷贝、填充、读写等基础内存操作 |
| `06_rtlat/` | `rtlat_perf` | 实时延迟测试，覆盖调度、抢占、IPC、IRQ 等实时性指标 |
| `07_rpmsg/` | `rpmsg_perf` | 大核 Linux 与小核 ESOS/RT-Thread 的 RPMsg request/echo 往返性能测试，统计 RTT 和吞吐；详细说明见 `07_rpmsg/README.md` |
| `07_rpmsg_send/` | `rpmsg_send_test` | 大核 Linux 发送 DATA、小核返回 `received` ACK 的确认链路测试，统计 ACK RTT 和发送吞吐；详细说明见 `07_rpmsg_send/README.md` |
| `08_mpc/` | `mpc_perf` | 四轮 MPC 闭环控制性能测试 |
| `09_model_infer/` | `model_perf` | 模型推理仿真性能测试 |
| `10_sched_stress/` | `sched_perf` | 调度压力/多任务干扰测试 |
| `main.c` | - | 启动时打印测试套件 banner 和可用命令提示 |
| `SConscript` | - | 构建脚本，将测试用例加入 `os1_rcpu` 构建源文件列表 |

## 构建集成

当前 `SConscript` 中针对 `os1_rcpu` 已加入本目录全部测试源文件：

- `rt-perf-test/main.c`
- `01_ekf/ekf_perf_test.c`
- `02_foc/foc_perf_test.c`
- `03_signal/signal_perf_test.c`
- `04_AHRS/ahrs_perf_test.c`
- `05_memory/mem_perf_test.c`
- `06_rtlat/rtlat_perf_test.c`
- `07_rpmsg/rpmsg_perf_test.c`
- `07_rpmsg_send/rpmsg_send_test.c`
- `08_mpc/mpc_perf_test.c`
- `09_model_infer/simulate_model.c`
- `10_sched_stress/sched_stress_perf_test.c`

构建并启动小核固件后，系统会打印类似信息：

```text
########## RT Performance Test ##########
Use 'ekf_perf' to start the EKF background loop.
Use 'foc_perf' to start the FOC background loop.
...
Fixed 1000 Hz test loops.
```

## 使用方法

在小核 MSH 中输入对应命令即可启动测试，例如：

```text
ekf_perf
foc_perf
signal_perf
ahrs_perf
mem_perf
rtlat_perf
rpmsg_perf
rpmsg_send_test
mpc_perf
model_perf
sched_perf
```

多数算法类测试会创建后台线程执行固定周期循环，并周期性打印执行耗时、循环次数或统计结果。`rpmsg_perf` 和 `rpmsg_send_test` 作为大小核通信服务，需要配合大核 Linux 侧测试程序使用，详见 `07_rpmsg/README.md` 和 `07_rpmsg_send/README.md`。

RPMsg 测试常用命令：

```text
rpmsg_perf          # 启动 echo 服务并开启小核统计打印
rpmsg_perf 0        # 启动 echo 服务但关闭小核统计打印
rpmsg_send_test     # 启动 DATA/ACK 服务但关闭小核统计打印
rpmsg_send_test --print
```

## 测试类型说明

### 算法计算类

包括 `ekf_perf`、`foc_perf`、`signal_perf`、`ahrs_perf`、`mpc_perf`、`model_perf`。这类测试主要用于观察小核在典型实时算法负载下的浮点计算能力、单周期执行时间和长期运行稳定性。多数测试默认使用 `1 ms` 固定周期，模拟 `1000 Hz` 控制或处理循环。

### 内存与缓存类

`mem_perf` 用于评估基础内存操作性能，适合观察不同数据规模、缓存状态和系统负载下的内存访问表现。

### 实时性与调度类

`rtlat_perf` 用于观察实时调度延迟、抢占延迟、IPC 延迟和中断相关延迟；`sched_perf` 用于制造多任务干扰，观察调度器在忙等、让出、睡眠、IPC 等不同任务组合下的行为。

### 大小核通信类

`rpmsg_perf` 创建 `rpmsg:perf_test` 服务，小核收到大核 Linux 发送的数据后立即 echo 完整 frame，用于测量 RPMsg request/echo 往返 RTT 和吞吐。大核侧程序位于 `07_rpmsg/k3_rpmsg_perf.c`。

`rpmsg_send_test` 创建 `rpmsg:send_test` 服务，小核收到 DATA frame 后返回同序号 `received` ACK，用于验证大核发送、小核确认链路和 ACK RTT。大核侧程序位于 `07_rpmsg_send/k3_rpmsg_send.c`。

## 建议测试流程

1. 单独运行一个测试命令，确认基础功能和输出正常。
2. 对算法类测试，先观察默认 `1000 Hz` 周期下是否存在超时或异常波动。
3. 对实时性测试，先在系统空载下运行 `rtlat_perf`，再配合 `sched_perf` 或算法类测试观察干扰影响。
4. 对 RPMsg 测试，先在小核启动 `rpmsg_perf` 或 `rpmsg_send_test`，再在大核 Linux 侧编译并运行对应目录下的 `k3_rpmsg_perf` 或 `k3_rpmsg_send`。
5. 多次重复测试时，尽量保持相同 CPU 频率、系统负载、日志等级和外设状态，便于横向比较。

## 注意事项

- 多数测试会创建后台线程；重复执行同一命令时，代码通常会检测并提示测试已启动。
- 频繁 `rt_kprintf` 会影响实时性和吞吐测试结果，分析性能时应重点关注稳定运行阶段的数据。
- 若需要更改周期、迭代次数或打印间隔，可在对应测试源文件顶部的宏定义中调整。
- 同时运行多个测试会相互干扰，适合压力场景；如果要测单项性能，建议一次只运行一个测试。
- `07_rpmsg/` 和 `07_rpmsg_send/` 涉及大核 Linux 侧程序编译、`/dev/rpmsg_ctrl0` 与 `/dev/rpmsgX` 设备节点，请参考对应目录下的专用说明文档。

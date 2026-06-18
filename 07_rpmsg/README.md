# RPMsg 大小核往返性能测试

本目录用于测试 Spacemit K3 大核 Linux 与小核 ESOS/RT-Thread 之间的数据包往返性能。测试链路为：大核发送一个数据包到小核，小核在 RPMsg 回调中立即 echo 完整数据包，大核收到该包回包后再发送下一个包；RTT 统计为单个包的一发一收往返时间。

## 文件说明

| 文件 | 运行侧 | 说明 |
| --- | --- | --- |
| `rpmsg_perf_test.c` | 小核 RCPU | 创建 `rpmsg:perf_test` 服务；RPMsg 回调校验收到的 frame，并直接 `rpmsg_send()` echo 完整 frame |
| `k3_rpmsg_perf.c` | 大核 Linux | 创建 Linux RPMsg endpoint；一个 pthread 负责按“一发一等回包”发送，一个 pthread 负责接收回包并唤醒发送线程，同时统计 RTT/吞吐 |

## RPMsg 参数

| 参数 | 值 |
| --- | --- |
| 服务名 | `rpmsg:perf_test` |
| 小核地址 | `1002` |
| 大核地址 | `1003` |
| 默认控制设备 | `/dev/rpmsg_ctrl0` |
| 默认数据设备 | `/dev/rpmsg0` |
| RPMsg buffer 长度 | `512 bytes` |
| RPMsg 头长度 | `16 bytes` |
| 最大 frame 长度 | `496 bytes` |
| frame 头长度 | `20 bytes` |
| 最大 payload 长度 | `476 bytes` |

## 小核侧使用

`rpmsg_perf_test.c` 已加入 `bsp/spacemit/applications/SConscript` 的 `os1_rcpu` 构建源文件列表。启动小核后，在 MSH 中执行：

```text
rpmsg_perf [stat]
```

`stat` 参数用于控制是否启动小核统计打印线程：

| 参数 | 说明 |
| --- | --- |
| 不填 / `1` / `on` / `stat` | 启动统计打印线程，保持原默认行为 |
| `0` / `off` / `nostat` / `no-stat` | 不启动统计打印线程，减少小核周期打印对 RTT 的影响 |

正常会看到类似输出：

```text
[RPMSG_PERF] Endpoint created: rpmsg:perf_test (src=1002, dst=1003), max_frame=496, tx_limit=496, rx_limit=496
[RPMSG_PERF] Callback echo enabled, stat thread started
```

如果执行 `rpmsg_perf 0`，最后一行会显示：

```text
[RPMSG_PERF] Callback echo enabled, stat thread disabled
```

启用统计线程时，小核会周期性打印：

```text
[RPMSG_PERF] rx=xxxx pkt/s xxx KB/s, tx=xxxx pkt/s xxx KB/s, bad=0, err=0
```

其中 `bad` 表示非法 frame 数量，`err` 表示回调中 `rpmsg_send` 失败次数。

## 大核侧编译

在 K3 Linux 目标板上编译：

```bash
cd /media/chenzhaoqi/data/tmp/esos_dev/4.0.1-perf/esos/bsp/spacemit/applications/rt-perf-test/07_rpmsg
gcc -Wall -Wextra -O2 -pthread -o k3_rpmsg_perf k3_rpmsg_perf.c
```

交叉编译时将 `gcc` 替换成目标工具链，例如：

```bash
riscv64-unknown-linux-gnu-gcc -Wall -Wextra -O2 -pthread -o k3_rpmsg_perf k3_rpmsg_perf.c
```

## 大核侧运行

先在小核 MSH 启动 `rpmsg_perf`，再在大核 Linux 执行：

```bash
sudo ./k3_rpmsg_perf
```

默认测试参数：

- payload：`128 bytes`
- packet count：`10000`
- 每 `1000` 个回包打印一次进度
- 每个包等待回包超时：`3000 ms`
- 记录 RTT 超过 `300 us` 的包序号，可用 `-w` 修改阈值

示例输出：

```text
RPMsg ready: service=rpmsg:perf_test src=1003 dst=1002 dev=/dev/rpmsgX
echo progress: 1000/10000
...

Summary: sent=10000 echoed=10000 match=yes elapsed=x.xxx s
Details: missing=0 bad_echo=0 duplicate_echo=0 short_write=0 timeout=0 sender_error=0 receiver_error=0
Slow RTT: threshold=300 us count=n
Throughput: tx=xxx.xx KB/s rx_echo=xxx.xx KB/s round_trip=xxx.xx KB/s (frame=148 bytes, payload=128 bytes)
Packet RTT: count=10000 min=xx.xx us avg=xx.xx us max=xx.xx us p50=xx.xx us p90=xx.xx us p99=xx.xx us
```

## 常用参数

```bash
sudo ./k3_rpmsg_perf -n 50000 -s 256 -r 5000 -t 5000
```

| 参数 | 说明 |
| --- | --- |
| `-n <count>` | 发送包数量 |
| `-s <bytes>` | payload 大小，范围 `0..476` |
| `-r <count>` | 每多少个回包打印一次进度 |
| `-t <ms>` | 每个包发送后等待对应回包的超时时间 |
| `-w <us>` | 记录并打印 RTT 超过该阈值的包序号，默认 `300`，设置 `0` 关闭 |
| `-c <device>` | RPMsg control 设备，默认 `/dev/rpmsg_ctrl0` |
| `-d <device>` | RPMsg data 设备，默认自动查找新建的 `/dev/rpmsgX` |

## 测试建议

1. 先使用默认 `128 bytes` payload 确认链路和统计正常。
2. 再分别测试典型 payload：`0`、`32`、`64`、`128`、`256`、`476`。
3. 大核使用独立发送/接收线程，但发送线程每发一个包都会等待对应回包，因此 RTT 是单包一发一收的同步往返时间。
4. 小核在 RPMsg 回调中直接 echo 完整 frame，没有 RX/TX 队列线程，RTT 更接近 RPMsg 往返路径本身。
5. 如果 `missing` 或 `err` 非 0，可降低发送数量、增大超时时间，或检查 RPMsg 设备状态。
6. 如果提示无法打开 `/dev/rpmsg_ctrl0` 或找不到 `/dev/rpmsgX`，请确认小核已执行 `rpmsg_perf`，并确认 Linux RPMsg char 设备已加载。
7. 当前 OpenAMP 默认 `RPMSG_BUFFER_SIZE=512`，扣除 `struct rpmsg_hdr` 的 `16 bytes` 后，单包 frame 上限为 `496 bytes`；本测试自定义 frame 头为 `20 bytes`，因此最大 payload 为 `476 bytes`。
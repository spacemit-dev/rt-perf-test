# RPMsg 大核发送/小核 ACK 测试

本目录用于测试 Spacemit K3 大核 Linux 到小核 ESOS/RT-Thread 的 RPMsg 发送与确认链路。测试链路为：大核向小核发送 DATA frame，小核校验后返回 ACK frame，ACK payload 固定为 `received`；大核侧统计发送数量、ACK 数量、发送吞吐和 ACK RTT。

## 文件说明

| 文件 | 运行侧 | 说明 |
| --- | --- | --- |
| `rpmsg_send_test.c` | 小核 RCPU | 创建 `rpmsg:send_test` 服务；收到大核 DATA frame 后返回同序号 ACK frame，ACK payload 固定为 `received` |
| `k3_rpmsg_send.c` | 大核 Linux | 创建 Linux RPMsg endpoint；发送线程逐包发送 DATA 并等待对应 ACK，接收线程负责接收 ACK、唤醒发送线程并统计 RTT |

## RPMsg 参数

| 参数 | 值 |
| --- | --- |
| 服务名 | `rpmsg:send_test` |
| 小核地址 | `1012` |
| 大核地址 | `1013` |
| 默认控制设备 | `/dev/rpmsg_ctrl0` |
| 默认数据设备 | 自动查找新建的 `/dev/rpmsgX`，也可用 `-d` 指定 |
| RPMsg buffer 长度 | `512 bytes` |
| RPMsg 头长度 | `16 bytes` |
| 最大 frame 长度 | `496 bytes` |
| frame 头长度 | `16 bytes` |
| 最大 payload 长度 | `480 bytes` |
| ACK payload | `received` |

## 小核侧使用

`rpmsg_send_test.c` 已加入 `bsp/spacemit/applications/SConscript` 的 `os1_rcpu` 构建源文件列表。启动小核后，在 MSH 中执行：

```text
rpmsg_send_test
```

默认不启动小核周期统计打印线程，以减少打印对测试结果的影响。如果需要查看小核收包和回 ACK 统计，执行：

```text
rpmsg_send_test --print
```

命令参数：

| 参数 | 说明 |
| --- | --- |
| 不填 | 启动 RPMsg 服务，不启动统计打印线程 |
| `--print` / `-q` | 启动 RPMsg 服务，并启动小核统计打印线程 |
| `--help` / `-h` | 打印命令帮助 |

正常会看到类似输出：

```text
[RPMSG_SEND] Service starting...
[RPMSG_SEND] rpdev ready
[RPMSG_SEND] creating endpoint...
[RPMSG_SEND] Endpoint created: rpmsg:send_test (src=1012, dst=1013), max_frame=496
```

如果未开启统计线程，会额外看到：

```text
[RPMSG_SEND] Stat print thread disabled
```

开启统计线程后，小核每秒打印一次累计收包和回 ACK 统计：

```text
[RPMSG_SEND] rx=1000(+1000), ack=1000(+1000), bad=0, err=0
```

其中 `bad` 表示非法 DATA frame 数量，`err` 表示小核 `rpmsg_send()` ACK 失败次数。

## 大核侧编译

在 K3 Linux 目标板上编译：

```bash
cd rt-perf-test/07_rpmsg_send
gcc -Wall -Wextra -O2 -pthread -o k3_rpmsg_send k3_rpmsg_send.c
```

交叉编译时将 `gcc` 替换成目标工具链，例如：

```bash
riscv64-unknown-linux-gnu-gcc -Wall -Wextra -O2 -pthread -o k3_rpmsg_send k3_rpmsg_send.c
```

## 大核侧运行

先在小核 MSH 启动 `rpmsg_send_test`，再在大核 Linux 执行：

```bash
sudo ./k3_rpmsg_send
```

默认测试参数：

- payload：`128 bytes`
- packet count：`10000`
- 每收到 `1000` 个 ACK 打印一次进度
- 每个包发送后等待对应 ACK 的超时时间：`3000 ms`
- 大核发送线程逐包发送 DATA 并等待 ACK，接收线程接收 ACK 并统计 RTT

示例输出：

```text
RPMsg ready: service=rpmsg:send_test src=1013 dst=1012 dev=/dev/rpmsgX
ack progress: 1000/10000
...

Summary: sent=10000 acked=10000 match=yes elapsed=x.xxx s
Details: missing=0 bad_ack=0 duplicate_ack=0 short_write=0 sender_error=0 receiver_error=0
Timeout ACKs: 0
Send throughput: xxx.xx KB/s (frame=144 bytes, payload=128 bytes)
ACK RTT: count=10000 min=xx.xx us avg=xx.xx us max=xx.xx us p50=xx.xx us p90=xx.xx us p99=xx.xx us
```

## 常用参数

```bash
sudo ./k3_rpmsg_send -n 50000 -s 256 -r 5000 -t 5000
```

| 参数 | 说明 |
| --- | --- |
| `-n <count>` | 发送包数量 |
| `-s <bytes>` | payload 大小，范围 `0..480` |
| `-r <count>` | 每收到多少个 ACK 打印一次进度 |
| `-t <ms>` | 等待 ACK 的超时时间；发送线程用于等待当前包 ACK，接收线程用于发送结束后等待剩余 ACK |
| `-c <device>` | RPMsg control 设备，默认 `/dev/rpmsg_ctrl0` |
| `-d <device>` | RPMsg data 设备；默认自动查找新建的 `/dev/rpmsgX`，指定后直接打开该设备 |
| `-h` / `--help` | 打印帮助 |

## 测试建议

1. 先使用默认 `128 bytes` payload 确认链路、ACK 数量和 RTT 统计正常。
2. 再分别测试典型 payload：`0`、`32`、`64`、`128`、`256`、`480`。
3. 大核发送线程每发一个 DATA frame 都会等待同序号 ACK，因此 `ACK RTT` 表示单包 DATA 到 ACK 的往返确认时间。
4. 小核在 RPMsg 回调中直接 ACK，没有额外 RX/TX 队列线程；需要减少打印干扰时，使用默认 `rpmsg_send_test`，不要加 `--print`。
5. 如果 `missing`、`bad_ack`、`duplicate_ack` 或 `err` 非 0，可降低发送数量、增大 `-t` 超时时间，或检查 RPMsg 设备状态。
6. 如果提示无法打开 `/dev/rpmsg_ctrl0` 或找不到 `/dev/rpmsgX`，请确认小核已执行 `rpmsg_send_test`，并确认 Linux RPMsg char 设备已加载。
7. 当前 OpenAMP 默认 `RPMSG_BUFFER_SIZE=512`，扣除 `struct rpmsg_hdr` 的 `16 bytes` 后，单包 frame 上限为 `496 bytes`；本测试自定义 frame 头为 `16 bytes`，因此最大 payload 为 `480 bytes`。

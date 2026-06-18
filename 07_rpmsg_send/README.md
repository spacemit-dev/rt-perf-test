# RPMsg 大核发送/小核确认测试

本目录用于测试 Spacemit K3 大核 Linux 到小核 ESOS/RT-Thread 的 RPMsg 发送与确认链路。

## 文件说明

| 文件 | 运行侧 | 说明 |
| --- | --- | --- |
| `rpmsg_send_test.c` | 小核 RCPU | 创建 `rpmsg:send_test` 服务，收到大核 DATA frame 后返回 ACK frame，ACK payload 固定为 `received` |
| `k3_rpmsg_send.c` | 大核 Linux | 创建 Linux RPMsg endpoint，一个线程发送 DATA frame，另一个线程接收 ACK，最后打印发送数量与确认数量是否一致 |

## RPMsg 参数

| 参数 | 值 |
| --- | --- |
| 服务名 | `rpmsg:send_test` |
| 小核地址 | `1012` |
| 大核地址 | `1013` |
| 默认控制设备 | `/dev/rpmsg_ctrl0` |
| 默认数据设备 | `/dev/rpmsg0` |
| RPMsg buffer 长度 | `512 bytes` |
| RPMsg 头长度 | `16 bytes` |
| 最大 frame 长度 | `496 bytes` |
| frame 头长度 | `16 bytes` |
| 最大 payload 长度 | `480 bytes` |

## 小核侧使用

小核启动后，在 MSH 中执行：

```text
rpmsg_send_test
```

正常会看到类似输出：

```text
[RPMSG_SEND] Service starting...
[RPMSG_SEND] rpdev ready
[RPMSG_SEND] creating endpoint...
[RPMSG_SEND] Endpoint created: rpmsg:send_test (src=1012, dst=1013), max_frame=496
```

测试过程中小核每秒打印一次累计收包和回 ACK 统计：

```text
[RPMSG_SEND] rx=1000(+1000), ack=1000(+1000), bad=0, err=0
```

## 大核侧编译

在 K3 Linux 目标板上编译：

```bash
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
- 发送线程发送 DATA，接收线程接收 ACK

示例输出：

```text
RPMsg ready: service=rpmsg:send_test src=1013 dst=1012 dev=/dev/rpmsgX
ack progress: 1000/10000
...
Summary: sent=10000 acked=10000 match=yes elapsed=x.xxx s
Details: missing=0 bad_ack=0 duplicate_ack=0 short_write=0 sender_error=0 receiver_error=0
```

## 常用参数

```bash
sudo ./k3_rpmsg_send -n 50000 -s 256 -r 5000
```

| 参数 | 说明 |
| --- | --- |
| `-n <count>` | 发送包数量 |
| `-s <bytes>` | payload 大小，范围 `0..480` |
| `-r <count>` | 每收到多少个 ACK 打印一次进度 |
| `-t <ms>` | 发送线程退出后，接收线程等待剩余 ACK 的空闲超时时间 |
| `-c <device>` | RPMsg control 设备，默认 `/dev/rpmsg_ctrl0` |
| `-d <device>` | RPMsg data 设备，默认 `/dev/rpmsg0` |

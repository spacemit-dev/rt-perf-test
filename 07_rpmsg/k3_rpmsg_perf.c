/*
 * Linux side RPMsg communication performance test for Spacemit K3.
 *
 * It creates endpoint "rpmsg:perf_test", sends fixed-size packets to RCPU,
 * waits for echo, and prints round-trip latency and throughput statistics.
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_RPMSG_CTRL_DEV    "/dev/rpmsg_ctrl0"
#define DEFAULT_RPMSG_DATA_DEV    "/dev/rpmsg0"
#define DEFAULT_SERVICE_NAME      "rpmsg:perf_test"
#define DEFAULT_LOCAL_ADDR        1011U
#define DEFAULT_REMOTE_ADDR       1010U
#define DEFAULT_PAYLOAD_SIZE      128U
#define DEFAULT_PACKET_COUNT      10000U
#define DEFAULT_REPORT_INTERVAL   1000U
#define RPMSG_PERF_MAGIC          0x52504631U
#define RPMSG_PERF_HEADER_SIZE    20U
#define RPMSG_PERF_MAX_FRAME_SIZE 496U

struct rpmsg_endpoint_info {
    char name[32];
    uint32_t src;
    uint32_t dst;
};

#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)

struct rpmsg_perf_frame {
    uint32_t magic;
    uint32_t seq;
    uint64_t send_ns;
    uint32_t payload_len;
    uint8_t payload[RPMSG_PERF_MAX_FRAME_SIZE - RPMSG_PERF_HEADER_SIZE];
};

struct rpmsg_perf_config {
    const char *ctrl_dev;
    const char *data_dev;
    const char *service_name;
    uint32_t local_addr;
    uint32_t remote_addr;
    uint32_t payload_size;
    uint32_t packet_count;
    uint32_t report_interval;
};

static volatile sig_atomic_t stop_requested;

static void signal_handler(int sig)
{
    (void)sig;
    stop_requested = 1;
}

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("\nOptions:\n");
    printf("  -n <count>       Packet count. Default: %u\n", DEFAULT_PACKET_COUNT);
    printf("  -s <bytes>       Payload size, 0..%u. Default: %u\n",
           RPMSG_PERF_MAX_FRAME_SIZE - RPMSG_PERF_HEADER_SIZE,
           DEFAULT_PAYLOAD_SIZE);
    printf("  -r <count>       Report interval in packets. Default: %u\n",
           DEFAULT_REPORT_INTERVAL);
    printf("  -c <device>      RPMsg control device. Default: %s\n", DEFAULT_RPMSG_CTRL_DEV);
    printf("  -d <device>      RPMsg data device. Default: %s\n", DEFAULT_RPMSG_DATA_DEV);
    printf("  -h, --help       Show this help.\n");
}

static void config_init(struct rpmsg_perf_config *cfg)
{
    cfg->ctrl_dev = DEFAULT_RPMSG_CTRL_DEV;
    cfg->data_dev = DEFAULT_RPMSG_DATA_DEV;
    cfg->service_name = DEFAULT_SERVICE_NAME;
    cfg->local_addr = DEFAULT_LOCAL_ADDR;
    cfg->remote_addr = DEFAULT_REMOTE_ADDR;
    cfg->payload_size = DEFAULT_PAYLOAD_SIZE;
    cfg->packet_count = DEFAULT_PACKET_COUNT;
    cfg->report_interval = DEFAULT_REPORT_INTERVAL;
}

static int parse_args(int argc, char **argv, struct rpmsg_perf_config *cfg)
{
    int i;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            cfg->packet_count = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            cfg->payload_size = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            cfg->report_interval = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            cfg->ctrl_dev = argv[++i];
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            cfg->data_dev = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 1;
        } else {
            fprintf(stderr, "Unknown or incomplete option: %s\n", argv[i]);
            print_usage(argv[0]);
            return -1;
        }
    }

    if (cfg->payload_size > RPMSG_PERF_MAX_FRAME_SIZE - RPMSG_PERF_HEADER_SIZE) {
        fprintf(stderr, "payload size too large, max=%u\n",
                RPMSG_PERF_MAX_FRAME_SIZE - RPMSG_PERF_HEADER_SIZE);
        return -1;
    }
    if (cfg->packet_count == 0 || cfg->report_interval == 0) {
        fprintf(stderr, "packet count and report interval must be non-zero\n");
        return -1;
    }

    return 0;
}

static int rpmsg_open(const struct rpmsg_perf_config *cfg, int *ctrl_fd, int *data_fd)
{
    struct rpmsg_endpoint_info epinfo;

    *ctrl_fd = open(cfg->ctrl_dev, O_RDWR);
    if (*ctrl_fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", cfg->ctrl_dev, strerror(errno));
        return -1;
    }

    memset(&epinfo, 0, sizeof(epinfo));
    strncpy(epinfo.name, cfg->service_name, sizeof(epinfo.name) - 1);
    epinfo.src = cfg->local_addr;
    epinfo.dst = cfg->remote_addr;

    if (ioctl(*ctrl_fd, RPMSG_CREATE_EPT_IOCTL, &epinfo) < 0) {
        fprintf(stderr, "create rpmsg endpoint failed: %s\n", strerror(errno));
        close(*ctrl_fd);
        *ctrl_fd = -1;
        return -1;
    }

    *data_fd = open(cfg->data_dev, O_RDWR);
    if (*data_fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", cfg->data_dev, strerror(errno));
        ioctl(*ctrl_fd, RPMSG_DESTROY_EPT_IOCTL);
        close(*ctrl_fd);
        *ctrl_fd = -1;
        return -1;
    }

    printf("RPMsg ready: service=%s src=%u dst=%u\n",
           cfg->service_name, cfg->local_addr, cfg->remote_addr);
    return 0;
}

static void rpmsg_close(int ctrl_fd, int data_fd)
{
    if (data_fd >= 0) {
        close(data_fd);
    }
    if (ctrl_fd >= 0) {
        ioctl(ctrl_fd, RPMSG_DESTROY_EPT_IOCTL);
        close(ctrl_fd);
    }
}

static int read_echo(int fd, struct rpmsg_perf_frame *frame, size_t expected_len)
{
    struct pollfd pfd;
    int ret;

    memset(&pfd, 0, sizeof(pfd));
    pfd.fd = fd;
    pfd.events = POLLIN;

    while (!stop_requested) {
        ret = poll(&pfd, 1, 3000);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            fprintf(stderr, "poll failed: %s\n", strerror(errno));
            return -1;
        }
        if (ret == 0) {
            fprintf(stderr, "timeout waiting echo\n");
            return -1;
        }
        if (pfd.revents & POLLIN) {
            ret = (int)read(fd, frame, RPMSG_PERF_MAX_FRAME_SIZE);
            if (ret < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                fprintf(stderr, "read failed: %s\n", strerror(errno));
                return -1;
            }
            if ((size_t)ret == expected_len) {
                return ret;
            }
            fprintf(stderr, "unexpected echo length: %d, expected=%zu\n", ret, expected_len);
            return -1;
        }
    }

    return -1;
}

int main(int argc, char **argv)
{
    struct rpmsg_perf_config cfg;
    struct rpmsg_perf_frame tx_frame;
    struct rpmsg_perf_frame rx_frame;
    int ctrl_fd = -1;
    int data_fd = -1;
    size_t frame_len;
    uint64_t total_start_ns;
    uint64_t min_rtt_ns = UINT64_MAX;
    uint64_t max_rtt_ns = 0;
    uint64_t total_rtt_ns = 0;
    uint32_t i;
    int parse_ret;

    config_init(&cfg);
    parse_ret = parse_args(argc, argv, &cfg);
    if (parse_ret > 0) {
        return 0;
    }
    if (parse_ret < 0) {
        return 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (rpmsg_open(&cfg, &ctrl_fd, &data_fd) != 0) {
        return 1;
    }

    memset(&tx_frame, 0, sizeof(tx_frame));
    tx_frame.magic = RPMSG_PERF_MAGIC;
    tx_frame.payload_len = cfg.payload_size;
    for (i = 0; i < cfg.payload_size; ++i) {
        tx_frame.payload[i] = (uint8_t)(i & 0xffU);
    }

    frame_len = RPMSG_PERF_HEADER_SIZE + cfg.payload_size;
    total_start_ns = now_ns();

    for (i = 0; i < cfg.packet_count && !stop_requested; ++i) {
        ssize_t wr;
        uint64_t rtt_ns;

        tx_frame.seq = i;
        tx_frame.send_ns = now_ns();

        wr = write(data_fd, &tx_frame, frame_len);
        if (wr < 0) {
            fprintf(stderr, "write failed at seq=%u: %s\n", i, strerror(errno));
            break;
        }
        if ((size_t)wr != frame_len) {
            fprintf(stderr, "short write at seq=%u: %zd/%zu\n", i, wr, frame_len);
            break;
        }

        if (read_echo(data_fd, &rx_frame, frame_len) < 0) {
            break;
        }
        if (rx_frame.magic != RPMSG_PERF_MAGIC || rx_frame.seq != i ||
            rx_frame.payload_len != cfg.payload_size) {
            fprintf(stderr, "bad echo frame: seq=%u magic=0x%x payload=%u\n",
                    rx_frame.seq, rx_frame.magic, rx_frame.payload_len);
            break;
        }

        rtt_ns = now_ns() - rx_frame.send_ns;
        if (rtt_ns < min_rtt_ns) {
            min_rtt_ns = rtt_ns;
        }
        if (rtt_ns > max_rtt_ns) {
            max_rtt_ns = rtt_ns;
        }
        total_rtt_ns += rtt_ns;

        if ((i + 1U) % cfg.report_interval == 0U) {
            double elapsed_s = (double)(now_ns() - total_start_ns) / 1000000000.0;
            double avg_rtt_us = (double)total_rtt_ns / (double)(i + 1U) / 1000.0;
            double throughput_kbps = ((double)frame_len * 2.0 * (double)(i + 1U)) /
                                     elapsed_s / 1024.0;

            printf("[%u/%u] avg_rtt=%.2f us min=%.2f us max=%.2f us throughput=%.2f KB/s\n",
                   i + 1U,
                   cfg.packet_count,
                   avg_rtt_us,
                   (double)min_rtt_ns / 1000.0,
                   (double)max_rtt_ns / 1000.0,
                   throughput_kbps);
        }
    }

    if (i > 0U) {
        double elapsed_s = (double)(now_ns() - total_start_ns) / 1000000000.0;
        printf("\nSummary: packets=%u payload=%u frame=%zu elapsed=%.3f s\n",
               i, cfg.payload_size, frame_len, elapsed_s);
        printf("RTT: avg=%.2f us min=%.2f us max=%.2f us\n",
               (double)total_rtt_ns / (double)i / 1000.0,
               (double)min_rtt_ns / 1000.0,
               (double)max_rtt_ns / 1000.0);
        printf("Full-duplex echo throughput: %.2f KB/s\n",
               ((double)frame_len * 2.0 * (double)i) / elapsed_s / 1024.0);
    }

    rpmsg_close(ctrl_fd, data_fd);
    return 0;
}

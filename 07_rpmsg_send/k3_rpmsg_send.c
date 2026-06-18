/*
 * Linux big-core RPMsg send/ack test for Spacemit K3.
 *
 * One pthread sends DATA frames to the small core. Another pthread receives ACK
 * frames whose payload is "received" and checks whether every sent sequence was
 * acknowledged exactly once.
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_RPMSG_CTRL_DEV       "/dev/rpmsg_ctrl0"
#define DEFAULT_RPMSG_DATA_DEV       "/dev/rpmsg0"
#define DEFAULT_SERVICE_NAME         "rpmsg:send_test"
#define DEFAULT_LOCAL_ADDR           1013U
#define DEFAULT_REMOTE_ADDR          1012U
#define DEFAULT_PAYLOAD_SIZE         128U
#define DEFAULT_PACKET_COUNT         10000U
#define DEFAULT_REPORT_INTERVAL      1000U
#define DEFAULT_IDLE_TIMEOUT_MS      3000U
#define RPMSG_SEND_MAGIC             0x52505331U
#define RPMSG_SEND_TYPE_DATA         1U
#define RPMSG_SEND_TYPE_ACK          2U
#define RPMSG_SEND_HEADER_SIZE       16U
#define RPMSG_SEND_RPMSG_BUFFER_SIZE 512U
#define RPMSG_SEND_RPMSG_HEADER_SIZE 16U
#define RPMSG_SEND_MAX_FRAME_SIZE    (RPMSG_SEND_RPMSG_BUFFER_SIZE - RPMSG_SEND_RPMSG_HEADER_SIZE)
#define RPMSG_SEND_ACK_TEXT          "received"

struct rpmsg_endpoint_info {
    char name[32];
    uint32_t src;
    uint32_t dst;
};

#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)

struct rpmsg_send_frame {
    uint32_t magic;
    uint32_t type;
    uint32_t seq;
    uint32_t payload_len;
    uint8_t payload[RPMSG_SEND_MAX_FRAME_SIZE - RPMSG_SEND_HEADER_SIZE];
};

struct rpmsg_send_config {
    const char *ctrl_dev;
    const char *data_dev;
    const char *service_name;
    uint32_t local_addr;
    uint32_t remote_addr;
    uint32_t payload_size;
    uint32_t packet_count;
    uint32_t report_interval;
    uint32_t idle_timeout_ms;
};

struct rpmsg_dev_snapshot {
    dev_t devs[64];
    size_t count;
};

struct rpmsg_send_shared {
    int data_fd;
    const struct rpmsg_send_config *cfg;
    size_t frame_len;
    uint8_t *acked;
    uint64_t *send_ns;
    uint64_t *rtt_ns;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    uint32_t sent_count;
    uint32_t ack_count;
    uint32_t rtt_count;
    uint32_t bad_ack_count;
    uint32_t duplicate_ack_count;
    uint32_t short_write_count;
    uint64_t total_rtt_ns;
    uint64_t min_rtt_ns;
    uint64_t max_rtt_ns;
    int sender_done;
    int sender_error;
    int receiver_error;
};

struct rpmsg_rtt_summary {
    uint64_t min_ns;
    uint64_t max_ns;
    uint64_t avg_ns;
    uint64_t p50_ns;
    uint64_t p90_ns;
    uint64_t p99_ns;
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
           RPMSG_SEND_MAX_FRAME_SIZE - RPMSG_SEND_HEADER_SIZE,
           DEFAULT_PAYLOAD_SIZE);
    printf("  -r <count>       Receiver report interval. Default: %u\n",
           DEFAULT_REPORT_INTERVAL);
    printf("  -t <ms>          Receiver idle timeout after sender exits. Default: %u\n",
           DEFAULT_IDLE_TIMEOUT_MS);
    printf("  -c <device>      RPMsg control device. Default: %s\n", DEFAULT_RPMSG_CTRL_DEV);
    printf("  -d <device>      RPMsg data device. Default: %s\n", DEFAULT_RPMSG_DATA_DEV);
    printf("  -h, --help       Show this help.\n");
}

static void config_init(struct rpmsg_send_config *cfg)
{
    cfg->ctrl_dev = DEFAULT_RPMSG_CTRL_DEV;
    cfg->data_dev = DEFAULT_RPMSG_DATA_DEV;
    cfg->service_name = DEFAULT_SERVICE_NAME;
    cfg->local_addr = DEFAULT_LOCAL_ADDR;
    cfg->remote_addr = DEFAULT_REMOTE_ADDR;
    cfg->payload_size = DEFAULT_PAYLOAD_SIZE;
    cfg->packet_count = DEFAULT_PACKET_COUNT;
    cfg->report_interval = DEFAULT_REPORT_INTERVAL;
    cfg->idle_timeout_ms = DEFAULT_IDLE_TIMEOUT_MS;
}

static int parse_args(int argc, char **argv, struct rpmsg_send_config *cfg)
{
    int i;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            cfg->packet_count = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            cfg->payload_size = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            cfg->report_interval = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            cfg->idle_timeout_ms = (uint32_t)strtoul(argv[++i], NULL, 0);
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

    if (cfg->payload_size > RPMSG_SEND_MAX_FRAME_SIZE - RPMSG_SEND_HEADER_SIZE) {
        fprintf(stderr, "payload size too large, max=%u\n",
                RPMSG_SEND_MAX_FRAME_SIZE - RPMSG_SEND_HEADER_SIZE);
        return -1;
    }
    if (cfg->packet_count == 0 || cfg->report_interval == 0 || cfg->idle_timeout_ms == 0) {
        fprintf(stderr, "packet count, report interval and timeout must be non-zero\n");
        return -1;
    }

    return 0;
}

static void snapshot_rpmsg_devs(struct rpmsg_dev_snapshot *snapshot)
{
    char path[64];
    struct stat st;
    int i;

    snapshot->count = 0;
    for (i = 0; i < 64; ++i) {
        snprintf(path, sizeof(path), "/dev/rpmsg%d", i);
        if (stat(path, &st) == 0 &&
            snapshot->count < sizeof(snapshot->devs) / sizeof(snapshot->devs[0])) {
            snapshot->devs[snapshot->count++] = st.st_rdev;
        }
    }
}

static int snapshot_has_dev(const struct rpmsg_dev_snapshot *snapshot, dev_t dev)
{
    size_t i;

    for (i = 0; i < snapshot->count; ++i) {
        if (snapshot->devs[i] == dev) {
            return 1;
        }
    }

    return 0;
}

static int find_new_rpmsg_data_dev(const struct rpmsg_dev_snapshot *before,
                                   char *path, size_t path_size)
{
    char candidate[64];
    struct stat st;
    int i;

    for (i = 0; i < 64; ++i) {
        snprintf(candidate, sizeof(candidate), "/dev/rpmsg%d", i);
        if (stat(candidate, &st) != 0) {
            continue;
        }
        if (!snapshot_has_dev(before, st.st_rdev)) {
            snprintf(path, path_size, "%s", candidate);
            return 0;
        }
    }

    return -1;
}

static int wait_new_rpmsg_data_dev(const struct rpmsg_dev_snapshot *before,
                                   char *path, size_t path_size)
{
    uint64_t start_ns = now_ns();

    while (now_ns() - start_ns < 3000000000ULL) {
        if (find_new_rpmsg_data_dev(before, path, path_size) == 0) {
            return 0;
        }
        poll(NULL, 0, 10);
    }

    return -1;
}

static int rpmsg_open(const struct rpmsg_send_config *cfg, int *ctrl_fd, int *data_fd)
{
    struct rpmsg_endpoint_info epinfo;
    struct rpmsg_dev_snapshot before;
    char data_dev[64];
    int flags;

    snapshot_rpmsg_devs(&before);

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

    if (strcmp(cfg->data_dev, DEFAULT_RPMSG_DATA_DEV) == 0) {
        if (wait_new_rpmsg_data_dev(&before, data_dev, sizeof(data_dev)) != 0) {
            fprintf(stderr, "failed to locate rpmsg data device\n");
            close(*ctrl_fd);
            *ctrl_fd = -1;
            return -1;
        }
    } else {
        snprintf(data_dev, sizeof(data_dev), "%s", cfg->data_dev);
    }

    *data_fd = open(data_dev, O_RDWR);
    if (*data_fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", data_dev, strerror(errno));
        close(*ctrl_fd);
        *ctrl_fd = -1;
        return -1;
    }

    flags = fcntl(*data_fd, F_GETFL, 0);
    if (flags < 0 || fcntl(*data_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        fprintf(stderr, "set %s nonblock failed: %s\n", data_dev, strerror(errno));
        ioctl(*data_fd, RPMSG_DESTROY_EPT_IOCTL);
        close(*data_fd);
        *data_fd = -1;
        close(*ctrl_fd);
        *ctrl_fd = -1;
        return -1;
    }

    printf("RPMsg ready: service=%s src=%u dst=%u dev=%s\n",
           cfg->service_name, cfg->local_addr, cfg->remote_addr, data_dev);
    fflush(stdout);
    return 0;
}

static void rpmsg_close(int ctrl_fd, int data_fd)
{
    if (data_fd >= 0) {
        ioctl(data_fd, RPMSG_DESTROY_EPT_IOCTL);
        close(data_fd);
    }
    if (ctrl_fd >= 0) {
        close(ctrl_fd);
    }
}

static int wait_fd_ready(int fd, short events, int timeout_ms, const char *op)
{
    struct pollfd pfd;
    int ret;

    memset(&pfd, 0, sizeof(pfd));
    pfd.fd = fd;
    pfd.events = events;

    while (!stop_requested) {
        ret = poll(&pfd, 1, timeout_ms);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            fprintf(stderr, "poll %s failed: %s\n", op, strerror(errno));
            return -1;
        }
        if (ret == 0) {
            return 0;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            fprintf(stderr, "rpmsg fd error while waiting %s, revents=0x%x\n",
                    op, pfd.revents);
            return -1;
        }
        if (pfd.revents & events) {
            return 1;
        }
    }

    return -1;
}

static int get_sender_done(struct rpmsg_send_shared *shared, uint32_t *sent_count)
{
    int done;

    pthread_mutex_lock(&shared->lock);
    done = shared->sender_done;
    if (sent_count != NULL) {
        *sent_count = shared->sent_count;
    }
    pthread_mutex_unlock(&shared->lock);

    return done;
}

static void make_realtime_deadline(struct timespec *deadline, uint32_t wait_ms)
{
    clock_gettime(CLOCK_REALTIME, deadline);
    deadline->tv_sec += wait_ms / 1000U;
    deadline->tv_nsec += (long)(wait_ms % 1000U) * 1000000L;
    if (deadline->tv_nsec >= 1000000000L) {
        deadline->tv_sec++;
        deadline->tv_nsec -= 1000000000L;
    }
}

static int wait_ack_for_seq(struct rpmsg_send_shared *shared, uint32_t seq)
{
    uint64_t start_ns = now_ns();
    int ret = 0;

    pthread_mutex_lock(&shared->lock);
    while (!shared->acked[seq] && !shared->receiver_error && !stop_requested) {
        struct timespec deadline;

        if ((now_ns() - start_ns) / 1000000ULL >= shared->cfg->idle_timeout_ms) {
            ret = -1;
            break;
        }

        make_realtime_deadline(&deadline, 100U);
        ret = pthread_cond_timedwait(&shared->cond, &shared->lock, &deadline);
        if (ret != 0 && ret != ETIMEDOUT) {
            break;
        }
        ret = 0;
    }
    if (shared->receiver_error || stop_requested) {
        ret = -1;
    }
    pthread_mutex_unlock(&shared->lock);

    return ret;
}

static void *send_thread_entry(void *arg)
{
    struct rpmsg_send_shared *shared = (struct rpmsg_send_shared *)arg;
    const struct rpmsg_send_config *cfg = shared->cfg;
    struct rpmsg_send_frame frame;
    uint32_t seq;

    memset(&frame, 0, sizeof(frame));
    frame.magic = RPMSG_SEND_MAGIC;
    frame.type = RPMSG_SEND_TYPE_DATA;
    frame.payload_len = cfg->payload_size;
    for (seq = 0; seq < cfg->payload_size; ++seq) {
        frame.payload[seq] = (uint8_t)(seq & 0xffU);
    }

    for (seq = 0; seq < cfg->packet_count && !stop_requested; ++seq) {
        ssize_t wr;

        frame.seq = seq;
        while (!stop_requested) {
            pthread_mutex_lock(&shared->lock);
            wr = write(shared->data_fd, &frame, shared->frame_len);
            if (wr >= 0) {
                if ((size_t)wr == shared->frame_len) {
                    shared->send_ns[seq] = now_ns();
                    shared->sent_count++;
                }
                pthread_mutex_unlock(&shared->lock);
                break;
            }
            pthread_mutex_unlock(&shared->lock);
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(stderr, "write failed at seq=%u: %s\n", seq, strerror(errno));
                pthread_mutex_lock(&shared->lock);
                shared->sender_error = 1;
                pthread_mutex_unlock(&shared->lock);
                goto out;
            }
            if (wait_fd_ready(shared->data_fd, POLLOUT, 1000, "tx buffer") < 0) {
                pthread_mutex_lock(&shared->lock);
                shared->sender_error = 1;
                pthread_mutex_unlock(&shared->lock);
                goto out;
            }
        }

        if (stop_requested) {
            break;
        }
        if ((size_t)wr != shared->frame_len) {
            fprintf(stderr, "short write at seq=%u: %zd/%zu\n", seq, wr, shared->frame_len);
            pthread_mutex_lock(&shared->lock);
            shared->short_write_count++;
            shared->sender_error = 1;
            pthread_mutex_unlock(&shared->lock);
            goto out;
        }

        if (wait_ack_for_seq(shared, seq) != 0) {
            fprintf(stderr, "timeout waiting ack at seq=%u\n", seq);
            pthread_mutex_lock(&shared->lock);
            shared->sender_error = 1;
            pthread_mutex_unlock(&shared->lock);
            goto out;
        }
    }

out:
    pthread_mutex_lock(&shared->lock);
    shared->sender_done = 1;
    pthread_cond_broadcast(&shared->cond);
    pthread_mutex_unlock(&shared->lock);
    return NULL;
}

static int check_ack_frame(const struct rpmsg_send_config *cfg,
                           const struct rpmsg_send_frame *frame, size_t len)
{
    size_t ack_len = strlen(RPMSG_SEND_ACK_TEXT);

    if (len < RPMSG_SEND_HEADER_SIZE || len > RPMSG_SEND_MAX_FRAME_SIZE) {
        return -1;
    }
    if (frame->magic != RPMSG_SEND_MAGIC || frame->type != RPMSG_SEND_TYPE_ACK) {
        return -1;
    }
    if (frame->seq >= cfg->packet_count || frame->payload_len != ack_len) {
        return -1;
    }
    if (RPMSG_SEND_HEADER_SIZE + frame->payload_len != len) {
        return -1;
    }
    if (memcmp(frame->payload, RPMSG_SEND_ACK_TEXT, ack_len) != 0) {
        return -1;
    }

    return 0;
}

static void *recv_thread_entry(void *arg)
{
    struct rpmsg_send_shared *shared = (struct rpmsg_send_shared *)arg;
    const struct rpmsg_send_config *cfg = shared->cfg;
    struct rpmsg_send_frame frame;
    uint64_t last_rx_ns = now_ns();
    int sender_done_seen = 0;

    while (!stop_requested) {
        uint32_t sent_count;
        int ready;
        int sender_done;
        ssize_t rd;

        sender_done = get_sender_done(shared, &sent_count);
        pthread_mutex_lock(&shared->lock);
        if (shared->ack_count >= cfg->packet_count ||
            (sender_done && shared->ack_count >= sent_count)) {
            pthread_mutex_unlock(&shared->lock);
            break;
        }
        pthread_mutex_unlock(&shared->lock);

        if (sender_done && !sender_done_seen) {
            last_rx_ns = now_ns();
            sender_done_seen = 1;
        }

        ready = wait_fd_ready(shared->data_fd, POLLIN, 200, "ack");
        if (ready < 0) {
            pthread_mutex_lock(&shared->lock);
            shared->receiver_error = 1;
            pthread_cond_broadcast(&shared->cond);
            pthread_mutex_unlock(&shared->lock);
            break;
        }
        if (ready == 0) {
            if (sender_done &&
                (now_ns() - last_rx_ns) / 1000000ULL >= cfg->idle_timeout_ms) {
                fprintf(stderr, "timeout waiting ack after sender done\n");
                pthread_mutex_lock(&shared->lock);
                shared->receiver_error = 1;
                pthread_cond_broadcast(&shared->cond);
                pthread_mutex_unlock(&shared->lock);
                break;
            }
            continue;
        }

        rd = read(shared->data_fd, &frame, sizeof(frame));
        if (rd < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            fprintf(stderr, "read failed: %s\n", strerror(errno));
            pthread_mutex_lock(&shared->lock);
            shared->receiver_error = 1;
            pthread_cond_broadcast(&shared->cond);
            pthread_mutex_unlock(&shared->lock);
            break;
        }

        last_rx_ns = now_ns();
        pthread_mutex_lock(&shared->lock);
        if (check_ack_frame(cfg, &frame, (size_t)rd) != 0) {
            shared->bad_ack_count++;
        } else if (shared->acked[frame.seq]) {
            shared->duplicate_ack_count++;
        } else {
            uint64_t rtt_ns = 0;

            if (shared->send_ns[frame.seq] != 0U) {
                rtt_ns = last_rx_ns - shared->send_ns[frame.seq];
                shared->rtt_ns[shared->rtt_count++] = rtt_ns;
                shared->total_rtt_ns += rtt_ns;
                if (shared->min_rtt_ns == 0U || rtt_ns < shared->min_rtt_ns) {
                    shared->min_rtt_ns = rtt_ns;
                }
                if (rtt_ns > shared->max_rtt_ns) {
                    shared->max_rtt_ns = rtt_ns;
                }
            }
            shared->acked[frame.seq] = 1;
            shared->ack_count++;
            if (shared->ack_count % cfg->report_interval == 0U) {
                printf("ack progress: %u/%u\n", shared->ack_count, cfg->packet_count);
            }
        }
        pthread_cond_broadcast(&shared->cond);
        pthread_mutex_unlock(&shared->lock);
    }

    return NULL;
}

static uint32_t count_missing_acks(const uint8_t *acked, uint32_t sent_count)
{
    uint32_t missing = 0;
    uint32_t i;

    for (i = 0; i < sent_count; ++i) {
        if (!acked[i]) {
            missing++;
        }
    }

    return missing;
}

static int compare_u64(const void *a, const void *b)
{
    uint64_t va = *(const uint64_t *)a;
    uint64_t vb = *(const uint64_t *)b;

    if (va < vb) {
        return -1;
    }
    if (va > vb) {
        return 1;
    }
    return 0;
}

static uint32_t percentile_index(uint32_t count, uint32_t percentile)
{
    uint64_t rank;

    if (count == 0U) {
        return 0U;
    }

    rank = ((uint64_t)count * percentile + 99U) / 100U;
    if (rank == 0U) {
        rank = 1U;
    }

    return (uint32_t)(rank - 1U);
}

static int build_rtt_summary(const struct rpmsg_send_shared *shared,
                             struct rpmsg_rtt_summary *summary)
{
    uint64_t *sorted;
    uint32_t count = shared->rtt_count;

    memset(summary, 0, sizeof(*summary));
    if (count == 0U) {
        return 0;
    }

    sorted = malloc((size_t)count * sizeof(sorted[0]));
    if (sorted == NULL) {
        return -1;
    }

    memcpy(sorted, shared->rtt_ns, (size_t)count * sizeof(sorted[0]));
    qsort(sorted, count, sizeof(sorted[0]), compare_u64);

    summary->min_ns = shared->min_rtt_ns;
    summary->max_ns = shared->max_rtt_ns;
    summary->avg_ns = shared->total_rtt_ns / count;
    summary->p50_ns = sorted[percentile_index(count, 50U)];
    summary->p90_ns = sorted[percentile_index(count, 90U)];
    summary->p99_ns = sorted[percentile_index(count, 99U)];

    free(sorted);
    return 0;
}

int main(int argc, char **argv)
{
    struct rpmsg_send_config cfg;
    struct rpmsg_send_shared shared;
    pthread_t send_tid;
    pthread_t recv_tid;
    int ctrl_fd = -1;
    int data_fd = -1;
    int parse_ret;
    int ret = 1;
    uint64_t start_ns;
    double elapsed_s;
    uint32_t missing;
    uint32_t sent_count;
    struct rpmsg_rtt_summary rtt;
    double send_throughput_kbps;

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

    memset(&shared, 0, sizeof(shared));
    shared.data_fd = data_fd;
    shared.cfg = &cfg;
    shared.frame_len = RPMSG_SEND_HEADER_SIZE + cfg.payload_size;
    shared.acked = calloc(cfg.packet_count, sizeof(shared.acked[0]));
    shared.send_ns = calloc(cfg.packet_count, sizeof(shared.send_ns[0]));
    shared.rtt_ns = calloc(cfg.packet_count, sizeof(shared.rtt_ns[0]));
    if (shared.acked == NULL || shared.send_ns == NULL || shared.rtt_ns == NULL) {
        fprintf(stderr, "alloc statistics table failed\n");
        goto out_free;
    }
    if (pthread_mutex_init(&shared.lock, NULL) != 0) {
        fprintf(stderr, "pthread_mutex_init failed\n");
        goto out_free;
    }
    if (pthread_cond_init(&shared.cond, NULL) != 0) {
        fprintf(stderr, "pthread_cond_init failed\n");
        goto out_mutex;
    }

    start_ns = now_ns();
    if (pthread_create(&recv_tid, NULL, recv_thread_entry, &shared) != 0) {
        fprintf(stderr, "create recv thread failed\n");
        goto out_cond;
    }
    if (pthread_create(&send_tid, NULL, send_thread_entry, &shared) != 0) {
        fprintf(stderr, "create send thread failed\n");
        stop_requested = 1;
        pthread_join(recv_tid, NULL);
        goto out_cond;
    }

    pthread_join(send_tid, NULL);
    pthread_join(recv_tid, NULL);

    elapsed_s = (double)(now_ns() - start_ns) / 1000000000.0;
    sent_count = shared.sent_count;
    missing = count_missing_acks(shared.acked, sent_count);
    send_throughput_kbps = elapsed_s > 0.0 ?
        ((double)shared.frame_len * (double)sent_count) / elapsed_s / 1024.0 : 0.0;

    printf("\nSummary: sent=%u acked=%u match=%s elapsed=%.3f s\n",
           sent_count,
           shared.ack_count,
           (sent_count == shared.ack_count && missing == 0U &&
            shared.bad_ack_count == 0U && shared.duplicate_ack_count == 0U) ? "yes" : "no",
           elapsed_s);
    printf("Details: missing=%u bad_ack=%u duplicate_ack=%u short_write=%u sender_error=%d receiver_error=%d\n",
           missing,
           shared.bad_ack_count,
           shared.duplicate_ack_count,
           shared.short_write_count,
           shared.sender_error,
           shared.receiver_error);
    printf("Timeout ACKs: %u\n", missing);
    printf("Send throughput: %.2f KB/s (frame=%zu bytes, payload=%u bytes)\n",
           send_throughput_kbps, shared.frame_len, cfg.payload_size);
    if (build_rtt_summary(&shared, &rtt) == 0 && shared.rtt_count > 0U) {
        printf("ACK RTT: count=%u min=%.2f us avg=%.2f us max=%.2f us p50=%.2f us p90=%.2f us p99=%.2f us\n",
               shared.rtt_count,
               (double)rtt.min_ns / 1000.0,
               (double)rtt.avg_ns / 1000.0,
               (double)rtt.max_ns / 1000.0,
               (double)rtt.p50_ns / 1000.0,
               (double)rtt.p90_ns / 1000.0,
               (double)rtt.p99_ns / 1000.0);
    } else {
        printf("ACK RTT: no valid samples\n");
    }

    ret = (sent_count == cfg.packet_count && sent_count == shared.ack_count &&
           missing == 0U && shared.bad_ack_count == 0U &&
           shared.duplicate_ack_count == 0U && shared.short_write_count == 0U &&
           shared.sender_error == 0 && shared.receiver_error == 0) ? 0 : 1;

out_cond:
    pthread_cond_destroy(&shared.cond);
out_mutex:
    pthread_mutex_destroy(&shared.lock);
out_free:
    free(shared.rtt_ns);
    free(shared.send_ns);
    free(shared.acked);
    rpmsg_close(ctrl_fd, data_fd);
    return ret;
}

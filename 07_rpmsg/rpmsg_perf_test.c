/*
 * RPMsg communication performance test for Spacemit K3 RCPU side.
 *
 * Linux sends framed packets to service "rpmsg:perf_test". RCPU echoes each
 * packet back immediately and periodically prints throughput/latency summary.
 *
 * Packet header is intentionally fixed-width and little-endian on K3 Linux/RCPU:
 *   magic      : 0x52504631 ("RPF1")
 *   seq        : packet sequence number
 *   send_ns    : Linux CLOCK_MONOTONIC timestamp in ns, used by Linux side
 *                to calculate round-trip latency after echo.
 *   payload_len: valid payload bytes after this header
 */

#include <openamp/remoteproc.h>
#include <openamp/rpmsg.h>
#include <openamp/rpmsg_virtio.h>
#include <openamp/virtio.h>
#include <rtdef.h>
#include <rtthread.h>
#include <string.h>

#define RPMSG_PERF_SERVICE_NAME       "rpmsg:perf_test"
#define RPMSG_PERF_ADDR_SRC           1010U
#define RPMSG_PERF_ADDR_DST           1011U
#define RPMSG_PERF_MAGIC              0x52504631U
#define RPMSG_PERF_HEADER_SIZE        20U
#define RPMSG_PERF_MAX_FRAME_SIZE     496U
#define RPMSG_PERF_PRINT_INTERVAL_MS  1000U
#define RPMSG_PERF_THREAD_STACK_SIZE  4096U

extern struct rpmsg_device *rpdev;

struct rpmsg_perf_frame {
    rt_uint32_t magic;
    rt_uint32_t seq;
    rt_uint64_t send_ns;
    rt_uint32_t payload_len;
    rt_uint8_t payload[RPMSG_PERF_MAX_FRAME_SIZE - RPMSG_PERF_HEADER_SIZE];
};

struct rpmsg_perf_ctx {
    char *service_name;
    struct rpmsg_endpoint endp;
    rt_bool_t endpoint_ready;
    volatile rt_uint32_t rx_packets;
    volatile rt_uint32_t tx_packets;
    volatile rt_uint32_t rx_bytes;
    volatile rt_uint32_t tx_bytes;
    volatile rt_uint32_t bad_packets;
    volatile rt_uint32_t send_errors;
};

static struct rpmsg_perf_ctx rpmsg_perf;

static int rpmsg_perf_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
                                  size_t len, uint32_t src, void *priv)
{
    struct rpmsg_perf_frame *frame = (struct rpmsg_perf_frame *)data;
    int ret;

    (void)src;
    (void)priv;

    if (len < RPMSG_PERF_HEADER_SIZE || len > RPMSG_PERF_MAX_FRAME_SIZE ||
        frame->magic != RPMSG_PERF_MAGIC ||
        frame->payload_len + RPMSG_PERF_HEADER_SIZE > len) {
        rpmsg_perf.bad_packets++;
        return 0;
    }

    rpmsg_perf.rx_packets++;
    rpmsg_perf.rx_bytes += (rt_uint32_t)len;

    ret = rpmsg_send(ept, data, len);
    if (ret < 0) {
        rpmsg_perf.send_errors++;
    } else {
        rpmsg_perf.tx_packets++;
        rpmsg_perf.tx_bytes += (rt_uint32_t)len;
    }

    return 0;
}

static void rpmsg_perf_service_unbind(struct rpmsg_endpoint *ept)
{
    (void)ept;
    rpmsg_perf.endpoint_ready = RT_FALSE;
    rt_kprintf("[RPMSG_PERF] Service unbound\n");
}

static void rpmsg_perf_print_thread_entry(void *parameter)
{
    rt_uint32_t last_rx_packets = 0U;
    rt_uint32_t last_tx_packets = 0U;
    rt_uint32_t last_rx_bytes = 0U;
    rt_uint32_t last_tx_bytes = 0U;

    (void)parameter;

    while (1) {
        rt_thread_mdelay(RPMSG_PERF_PRINT_INTERVAL_MS);

        if (!rpmsg_perf.endpoint_ready) {
            continue;
        }

        rt_uint32_t rx_packets = rpmsg_perf.rx_packets;
        rt_uint32_t tx_packets = rpmsg_perf.tx_packets;
        rt_uint32_t rx_bytes = rpmsg_perf.rx_bytes;
        rt_uint32_t tx_bytes = rpmsg_perf.tx_bytes;
        rt_uint32_t delta_rx_packets = rx_packets - last_rx_packets;
        rt_uint32_t delta_tx_packets = tx_packets - last_tx_packets;
        rt_uint32_t delta_rx_bytes = rx_bytes - last_rx_bytes;
        rt_uint32_t delta_tx_bytes = tx_bytes - last_tx_bytes;

        last_rx_packets = rx_packets;
        last_tx_packets = tx_packets;
        last_rx_bytes = rx_bytes;
        last_tx_bytes = tx_bytes;

        rt_kprintf("[RPMSG_PERF] rx=%u pkt/s %u KB/s, tx=%u pkt/s %u KB/s, bad=%u, err=%u\n",
                   delta_rx_packets,
                   delta_rx_bytes / 1024U,
                   delta_tx_packets,
                   delta_tx_bytes / 1024U,
                   rpmsg_perf.bad_packets,
                   rpmsg_perf.send_errors);
    }
}

static void rpmsg_perf_init_thread_entry(void *parameter)
{
    int ret;

    (void)parameter;

    while (rpdev == RT_NULL) {
        rt_thread_delay(10);
    }

    rt_kprintf("[RPMSG_PERF] rpdev ready, creating endpoint...\n");

    rpmsg_perf.service_name = RPMSG_PERF_SERVICE_NAME;
    ret = rpmsg_create_ept(&rpmsg_perf.endp, rpdev, rpmsg_perf.service_name,
                           RPMSG_PERF_ADDR_SRC, RPMSG_PERF_ADDR_DST,
                           rpmsg_perf_endpoint_cb, rpmsg_perf_service_unbind);
    if (ret) {
        rt_kprintf("[RPMSG_PERF] Create endpoint failed, ret=%d\n", ret);
        return;
    }

    rpmsg_perf.endpoint_ready = RT_TRUE;
    rt_kprintf("[RPMSG_PERF] Endpoint created: %s (src=%u, dst=%u), max_frame=%u, tx_payload_limit=%d, rx_payload_limit=%d\n",
               rpmsg_perf.service_name,
               RPMSG_PERF_ADDR_SRC,
               RPMSG_PERF_ADDR_DST,
               RPMSG_PERF_MAX_FRAME_SIZE,
               rpmsg_virtio_get_tx_buffer_size(rpdev),
               rpmsg_virtio_get_rx_buffer_size(rpdev));
}

int rpmsg_perf_start(void)
{
    rt_thread_t init_tid;
    rt_thread_t print_tid;

    if (rpmsg_perf.endpoint_ready) {
        rt_kprintf("[RPMSG_PERF] Already started\n");
        return 0;
    }

    rt_memset(&rpmsg_perf, 0, sizeof(rpmsg_perf));

    init_tid = rt_thread_create("rpmsg_pi", rpmsg_perf_init_thread_entry,
                                RT_NULL, RPMSG_PERF_THREAD_STACK_SIZE,
                                RT_THREAD_PRIORITY_MAX / 3, 20);
    if (init_tid == RT_NULL) {
        rt_kprintf("[RPMSG_PERF] Failed to create init thread\n");
        return -RT_EINVAL;
    }
    rt_thread_startup(init_tid);

    print_tid = rt_thread_create("rpmsg_ps", rpmsg_perf_print_thread_entry,
                                 RT_NULL, RPMSG_PERF_THREAD_STACK_SIZE,
                                 RT_THREAD_PRIORITY_MAX / 3 + 1, 20);
    if (print_tid == RT_NULL) {
        rt_kprintf("[RPMSG_PERF] Failed to create stat thread\n");
        return -RT_EINVAL;
    }
    rt_thread_startup(print_tid);

    rt_kprintf("[RPMSG_PERF] Service starting...\n");
    return 0;
}

static int cmd_rpmsg_perf(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    return rpmsg_perf_start();
}

#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(cmd_rpmsg_perf, rpmsg_perf, RPMsg communication performance echo test);

/*
 * RPMsg send/ack test for Spacemit K3 RCPU side.
 *
 * Linux big core sends DATA frames to service "rpmsg:send_test". The RCPU
 * small core validates every frame and replies with an ACK frame carrying the
 * same sequence number and payload text "received".
 */

#include <openamp/remoteproc.h>
#include <openamp/rpmsg.h>
#include <openamp/rpmsg_virtio.h>
#include <openamp/virtio.h>
#include <rtdef.h>
#include <rtthread.h>
#include <string.h>

#define RPMSG_SEND_SERVICE_NAME       "rpmsg:send_test"
#define RPMSG_SEND_ADDR_SRC           1012U
#define RPMSG_SEND_ADDR_DST           1013U
#define RPMSG_SEND_MAGIC              0x52505331U
#define RPMSG_SEND_TYPE_DATA          1U
#define RPMSG_SEND_TYPE_ACK           2U
#define RPMSG_SEND_HEADER_SIZE        16U
#define RPMSG_SEND_RPMSG_BUFFER_SIZE  512U
#define RPMSG_SEND_RPMSG_HEADER_SIZE  16U
#define RPMSG_SEND_MAX_FRAME_SIZE     (RPMSG_SEND_RPMSG_BUFFER_SIZE - RPMSG_SEND_RPMSG_HEADER_SIZE)
#define RPMSG_SEND_ACK_TEXT           "received"
#define RPMSG_SEND_PRINT_INTERVAL_MS  1000U
#define RPMSG_SEND_THREAD_STACK_SIZE  4096U

extern struct rpmsg_device *rpdev;

struct rpmsg_send_frame {
    rt_uint32_t magic;
    rt_uint32_t type;
    rt_uint32_t seq;
    rt_uint32_t payload_len;
    rt_uint8_t payload[RPMSG_SEND_MAX_FRAME_SIZE - RPMSG_SEND_HEADER_SIZE];
};

struct rpmsg_send_ctx {
    const char *service_name;
    struct rpmsg_endpoint endp;
    rt_bool_t service_started;
    rt_bool_t endpoint_ready;
    volatile rt_uint32_t rx_packets;
    volatile rt_uint32_t tx_acks;
    volatile rt_uint32_t bad_packets;
    volatile rt_uint32_t send_errors;
};

static struct rpmsg_send_ctx rpmsg_send_test;

static int rpmsg_send_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
                                  size_t len, uint32_t src, void *priv)
{
    const struct rpmsg_send_frame *rx = (const struct rpmsg_send_frame *)data;
    struct rpmsg_send_frame ack;
    size_t ack_payload_len = strlen(RPMSG_SEND_ACK_TEXT);
    size_t ack_len = RPMSG_SEND_HEADER_SIZE + ack_payload_len;
    int ret;

    (void)src;
    (void)priv;

    if (len < RPMSG_SEND_HEADER_SIZE || len > RPMSG_SEND_MAX_FRAME_SIZE ||
        rx->magic != RPMSG_SEND_MAGIC || rx->type != RPMSG_SEND_TYPE_DATA ||
        rx->payload_len + RPMSG_SEND_HEADER_SIZE > len) {
        rpmsg_send_test.bad_packets++;
        return 0;
    }

    rpmsg_send_test.rx_packets++;

    rt_memset(&ack, 0, sizeof(ack));
    ack.magic = RPMSG_SEND_MAGIC;
    ack.type = RPMSG_SEND_TYPE_ACK;
    ack.seq = rx->seq;
    ack.payload_len = (rt_uint32_t)ack_payload_len;
    rt_memcpy(ack.payload, RPMSG_SEND_ACK_TEXT, ack_payload_len);

    ret = rpmsg_send(ept, &ack, ack_len);
    if (ret < 0) {
        rpmsg_send_test.send_errors++;
    } else {
        rpmsg_send_test.tx_acks++;
    }

    return 0;
}

static void rpmsg_send_service_unbind(struct rpmsg_endpoint *ept)
{
    (void)ept;
    rpmsg_send_test.endpoint_ready = RT_FALSE;
    rt_kprintf("[RPMSG_SEND] Service unbound\n");
}

static void rpmsg_send_print_thread_entry(void *parameter)
{
    rt_uint32_t last_rx = 0U;
    rt_uint32_t last_ack = 0U;

    (void)parameter;

    while (1) {
        rt_thread_mdelay(RPMSG_SEND_PRINT_INTERVAL_MS);

        if (!rpmsg_send_test.endpoint_ready) {
            continue;
        }

        rt_uint32_t rx = rpmsg_send_test.rx_packets;
        rt_uint32_t ack = rpmsg_send_test.tx_acks;

        rt_kprintf("[RPMSG_SEND] rx=%u(+%u), ack=%u(+%u), bad=%u, err=%u\n",
                   rx, rx - last_rx, ack, ack - last_ack,
                   rpmsg_send_test.bad_packets,
                   rpmsg_send_test.send_errors);

        last_rx = rx;
        last_ack = ack;
    }
}

static void rpmsg_send_init_thread_entry(void *parameter)
{
    int ret;

    (void)parameter;

    while (rpdev == RT_NULL) {
        rt_thread_delay(10);
    }

    rt_kprintf("[RPMSG_SEND] rpdev ready\n");

    rpmsg_send_test.service_name = RPMSG_SEND_SERVICE_NAME;
    while (1) {
        if (!rpmsg_send_test.endpoint_ready) {
            rt_kprintf("[RPMSG_SEND] creating endpoint...\n");
            ret = rpmsg_create_ept(&rpmsg_send_test.endp, rpdev,
                                   rpmsg_send_test.service_name,
                                   RPMSG_SEND_ADDR_SRC, RPMSG_SEND_ADDR_DST,
                                   rpmsg_send_endpoint_cb,
                                   rpmsg_send_service_unbind);
            if (ret) {
                rt_kprintf("[RPMSG_SEND] Create endpoint failed, ret=%d\n", ret);
                rt_thread_mdelay(1000);
                continue;
            }

            rpmsg_send_test.endpoint_ready = RT_TRUE;
            rt_kprintf("[RPMSG_SEND] Endpoint created: %s (src=%u, dst=%u), max_frame=%u\n",
                       rpmsg_send_test.service_name,
                       RPMSG_SEND_ADDR_SRC,
                       RPMSG_SEND_ADDR_DST,
                       RPMSG_SEND_MAX_FRAME_SIZE);
        }

        rt_thread_mdelay(100);
    }
}

int rpmsg_send_test_start(rt_bool_t start_print_thread)
{
    rt_thread_t init_tid;
    rt_thread_t print_tid;

    if (rpmsg_send_test.service_started) {
        rt_kprintf("[RPMSG_SEND] Already started\n");
        return 0;
    }

    rt_memset(&rpmsg_send_test, 0, sizeof(rpmsg_send_test));
    rpmsg_send_test.service_started = RT_TRUE;

    init_tid = rt_thread_create("rpmsg_si", rpmsg_send_init_thread_entry,
                                RT_NULL, RPMSG_SEND_THREAD_STACK_SIZE,
                                RT_THREAD_PRIORITY_MAX / 3, 20);
    if (init_tid == RT_NULL) {
        rt_kprintf("[RPMSG_SEND] Failed to create init thread\n");
        rpmsg_send_test.service_started = RT_FALSE;
        return -RT_EINVAL;
    }
    rt_thread_startup(init_tid);

    if (start_print_thread) {
        print_tid = rt_thread_create("rpmsg_ss", rpmsg_send_print_thread_entry,
                                     RT_NULL, RPMSG_SEND_THREAD_STACK_SIZE,
                                     RT_THREAD_PRIORITY_MAX / 3 + 1, 20);
        if (print_tid == RT_NULL) {
            rt_kprintf("[RPMSG_SEND] Failed to create stat thread\n");
            rpmsg_send_test.service_started = RT_FALSE;
            return -RT_EINVAL;
        }
        rt_thread_startup(print_tid);
    } else {
        rt_kprintf("[RPMSG_SEND] Stat print thread disabled\n");
    }

    rt_kprintf("[RPMSG_SEND] Service starting...\n");
    return 0;
}

static void cmd_rpmsg_send_test_usage(void)
{
    rt_kprintf("Usage: rpmsg_send_test [--print|-q]\n");
    rt_kprintf("       rpmsg_send_test --help\n");
}

static int cmd_rpmsg_send_test(int argc, char *argv[])
{
    rt_bool_t start_print_thread = RT_FALSE;
    int i;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--print") || !strcmp(argv[i], "-q")) {
            start_print_thread = RT_TRUE;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            cmd_rpmsg_send_test_usage();
            return 0;
        } else {
            rt_kprintf("[RPMSG_SEND] Unknown option: %s\n", argv[i]);
            cmd_rpmsg_send_test_usage();
            return -RT_EINVAL;
        }
    }

    return rpmsg_send_test_start(start_print_thread);
}

#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(cmd_rpmsg_send_test, rpmsg_send_test, RPMsg big-core send and small-core ack test);

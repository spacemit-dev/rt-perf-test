/*
 * RPMsg round-trip throughput/RTT test for Spacemit K3 RCPU side.
 *
 * Linux sends DATA frames to service "rpmsg:perf_test". The RCPU side validates
 * each frame in the RPMsg callback and immediately echoes the full frame back.
 */

#include <openamp/remoteproc.h>
#include <openamp/rpmsg.h>
#include <openamp/rpmsg_virtio.h>
#include <openamp/virtio.h>
#include <rtdef.h>
#include <rtthread.h>
#include <string.h>

#define RPMSG_PERF_SERVICE_NAME       "rpmsg:perf_test"
#define RPMSG_PERF_ADDR_SRC           1002U
#define RPMSG_PERF_ADDR_DST           1003U
#define RPMSG_PERF_MAGIC              0x52504631U
#define RPMSG_PERF_HEADER_SIZE        20U
#define RPMSG_PERF_RPMSG_BUFFER_SIZE  512U
#define RPMSG_PERF_RPMSG_HEADER_SIZE  16U
#define RPMSG_PERF_MAX_FRAME_SIZE     (RPMSG_PERF_RPMSG_BUFFER_SIZE - RPMSG_PERF_RPMSG_HEADER_SIZE)
#define RPMSG_PERF_THREAD_STACK_SIZE  4096U
#define RPMSG_PERF_THREAD_PRIORITY    (RT_THREAD_PRIORITY_MAX / 3)
#define RPMSG_PERF_THREAD_TIMESLICE   20U
#define RPMSG_PERF_PRINT_INTERVAL_MS  1000U

extern struct rpmsg_device *rpdev;

struct rpmsg_perf_frame {
	rt_uint32_t magic;
	rt_uint32_t seq;
	rt_uint64_t send_ns;
	rt_uint32_t payload_len;
	rt_uint8_t payload[RPMSG_PERF_MAX_FRAME_SIZE - RPMSG_PERF_HEADER_SIZE];
};

struct rpmsg_perf_ctx {
	const char *service_name;
	struct rpmsg_endpoint endp;
	rt_bool_t service_started;
	rt_bool_t endpoint_ready;
	volatile rt_uint32_t rx_packets;
	volatile rt_uint32_t tx_packets;
	volatile rt_uint32_t rx_bytes;
	volatile rt_uint32_t tx_bytes;
	volatile rt_uint32_t bad_packets;
	volatile rt_uint32_t send_errors;
	rt_bool_t stat_enabled;
};

static struct rpmsg_perf_ctx rpmsg_perf;

static rt_bool_t rpmsg_perf_frame_valid(const void *data, rt_size_t len)
{
	const struct rpmsg_perf_frame *frame = (const struct rpmsg_perf_frame *)data;

	if (len < RPMSG_PERF_HEADER_SIZE || len > RPMSG_PERF_MAX_FRAME_SIZE) {
		return RT_FALSE;
	}
	if (frame->magic != RPMSG_PERF_MAGIC) {
		return RT_FALSE;
	}
	if (frame->payload_len + RPMSG_PERF_HEADER_SIZE != len) {
		return RT_FALSE;
	}

	return RT_TRUE;
}

static void rpmsg_perf_print_delta(rt_tick_t *last_tick,
								   rt_uint32_t *last_rx_packets,
								   rt_uint32_t *last_tx_packets,
								   rt_uint32_t *last_rx_bytes,
								   rt_uint32_t *last_tx_bytes)
{
	rt_tick_t now = rt_tick_get();
	rt_tick_t interval = rt_tick_from_millisecond(RPMSG_PERF_PRINT_INTERVAL_MS);
	rt_uint32_t rx_packets;
	rt_uint32_t tx_packets;
	rt_uint32_t rx_bytes;
	rt_uint32_t tx_bytes;

	if ((rt_tick_t)(now - *last_tick) < interval) {
		return;
	}

	rx_packets = rpmsg_perf.rx_packets;
	tx_packets = rpmsg_perf.tx_packets;
	rx_bytes = rpmsg_perf.rx_bytes;
	tx_bytes = rpmsg_perf.tx_bytes;

	rt_kprintf("[RPMSG_PERF] rx=%u pkt/s %u KB/s, tx=%u pkt/s %u KB/s, bad=%u, err=%u\n",
			   rx_packets - *last_rx_packets,
			   (rx_bytes - *last_rx_bytes) / 1024U,
			   tx_packets - *last_tx_packets,
			   (tx_bytes - *last_tx_bytes) / 1024U,
			   rpmsg_perf.bad_packets,
			   rpmsg_perf.send_errors);

	*last_tick = now;
	*last_rx_packets = rx_packets;
	*last_tx_packets = tx_packets;
	*last_rx_bytes = rx_bytes;
	*last_tx_bytes = tx_bytes;
}

static int rpmsg_perf_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
								  size_t len, uint32_t src, void *priv)
{
	(void)src;
	(void)priv;

	if (!rpmsg_perf_frame_valid(data, (rt_size_t)len)) {
		rpmsg_perf.bad_packets++;
		return 0;
	}

	rpmsg_perf.rx_packets++;
	rpmsg_perf.rx_bytes += (rt_uint32_t)len;

	if (rpmsg_send(ept, data, len) >= 0) {
		rpmsg_perf.tx_packets++;
		rpmsg_perf.tx_bytes += (rt_uint32_t)len;
	} else {
		rpmsg_perf.send_errors++;
	}

	return 0;
}

static void rpmsg_perf_service_unbind(struct rpmsg_endpoint *ept)
{
	(void)ept;
	rpmsg_perf.endpoint_ready = RT_FALSE;
	rt_kprintf("[RPMSG_PERF] Service unbound\n");
}

static void rpmsg_perf_stat_thread_entry(void *parameter)
{
	rt_tick_t last_tick = rt_tick_get();
	rt_uint32_t last_rx_packets = 0U;
	rt_uint32_t last_tx_packets = 0U;
	rt_uint32_t last_rx_bytes = 0U;
	rt_uint32_t last_tx_bytes = 0U;

	(void)parameter;

	while (1) {
		rt_thread_mdelay(100U);
		rpmsg_perf_print_delta(&last_tick, &last_rx_packets, &last_tx_packets,
							   &last_rx_bytes, &last_tx_bytes);
	}
}

static rt_err_t rpmsg_perf_create_worker(const char *name,
										 void (*entry)(void *parameter))
{
	rt_thread_t tid;

	tid = rt_thread_create(name, entry, RT_NULL,
						   RPMSG_PERF_THREAD_STACK_SIZE,
						   RPMSG_PERF_THREAD_PRIORITY,
						   RPMSG_PERF_THREAD_TIMESLICE);
	if (tid == RT_NULL) {
		rt_kprintf("[RPMSG_PERF] Failed to create %s thread\n", name);
		return -RT_EINVAL;
	}

	rt_thread_startup(tid);
	return RT_EOK;
}

static void rpmsg_perf_print_usage(void)
{
	rt_kprintf("Usage: rpmsg_perf [stat]\n");
	rt_kprintf("  stat: 1/on/stat enables stat print thread (default)\n");
	rt_kprintf("        0/off/nostat disables stat print thread\n");
}

static rt_bool_t rpmsg_perf_parse_stat_arg(const char *arg, rt_bool_t *stat_enabled)
{
	if (strcmp(arg, "1") == 0 || strcmp(arg, "on") == 0 || strcmp(arg, "stat") == 0) {
		*stat_enabled = RT_TRUE;
		return RT_TRUE;
	}

	if (strcmp(arg, "0") == 0 || strcmp(arg, "off") == 0 ||
		strcmp(arg, "nostat") == 0 || strcmp(arg, "no-stat") == 0) {
		*stat_enabled = RT_FALSE;
		return RT_TRUE;
	}

	return RT_FALSE;
}

int rpmsg_perf_start(rt_bool_t stat_enabled)
{
	int ret;

	if (rpmsg_perf.service_started) {
		rt_kprintf("[RPMSG_PERF] Already started, stat thread %s\n",
				   rpmsg_perf.stat_enabled ? "enabled" : "disabled");
		return 0;
	}

	rt_memset(&rpmsg_perf, 0, sizeof(rpmsg_perf));
	rpmsg_perf.service_name = RPMSG_PERF_SERVICE_NAME;
	rpmsg_perf.stat_enabled = stat_enabled;

	while (rpdev == RT_NULL) {
		rt_thread_delay(10);
	}

	ret = rpmsg_create_ept(&rpmsg_perf.endp, rpdev, rpmsg_perf.service_name,
						   RPMSG_PERF_ADDR_SRC, RPMSG_PERF_ADDR_DST,
						   rpmsg_perf_endpoint_cb, rpmsg_perf_service_unbind);
	if (ret) {
		rt_kprintf("[RPMSG_PERF] Create endpoint failed, ret=%d\n", ret);
		return ret;
	}

	rpmsg_perf.endpoint_ready = RT_TRUE;
	rpmsg_perf.service_started = RT_TRUE;

	if (rpmsg_perf.stat_enabled &&
		rpmsg_perf_create_worker("rpmsg_pst", rpmsg_perf_stat_thread_entry) != RT_EOK) {
		rpmsg_perf.service_started = RT_FALSE;
		return -RT_EINVAL;
	}

	rt_kprintf("[RPMSG_PERF] Endpoint created: %s (src=%u, dst=%u), max_frame=%u, tx_limit=%d, rx_limit=%d\n",
			   rpmsg_perf.service_name,
			   RPMSG_PERF_ADDR_SRC,
			   RPMSG_PERF_ADDR_DST,
			   RPMSG_PERF_MAX_FRAME_SIZE,
			   rpmsg_virtio_get_tx_buffer_size(rpdev),
			   rpmsg_virtio_get_rx_buffer_size(rpdev));
	rt_kprintf("[RPMSG_PERF] Callback echo enabled, stat thread %s\n",
			   rpmsg_perf.stat_enabled ? "started" : "disabled");
	return 0;
}

static int cmd_rpmsg_perf(int argc, char *argv[])
{
	rt_bool_t stat_enabled = RT_FALSE;

	if (argc > 2) {
		rpmsg_perf_print_usage();
		return -RT_EINVAL;
	}

	if (argc == 2) {
		if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
			rpmsg_perf_print_usage();
			return 0;
		}

		if (!rpmsg_perf_parse_stat_arg(argv[1], &stat_enabled)) {
			rpmsg_perf_print_usage();
			return -RT_EINVAL;
		}
	}

	return rpmsg_perf_start(stat_enabled);
}

#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(cmd_rpmsg_perf, rpmsg_perf, RPMsg callback echo performance test);

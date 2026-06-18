#include <rtthread.h>

int main(void)
{
    rt_kprintf("########## RT Performance Test2 ##########\n");
    rt_kprintf("Use 'ekf_perf' to start the EKF background loop.\n");
    rt_kprintf("Use 'foc_perf' to start the FOC background loop.\n");
    rt_kprintf("Use 'signal_perf' to start the FIR/IIR + FFT background loop.\n");
    rt_kprintf("Use 'ahrs_perf' to start the quaternion AHRS background loop.\n");
    rt_kprintf("Use 'mem_perf' to start the memory/cache background loop.\n");
    rt_kprintf("Use 'rtlat_perf' to start the RT latency background test.\n");
    rt_kprintf("Use 'rpmsg_perf [stat]' to start the RPMsg echo performance service.\n");
    rt_kprintf("  rpmsg_perf stat: 1/on/stat enables stat prints, 0/off/nostat disables them.\n");
    rt_kprintf("Use 'rpmsg_send_test' to start the RPMsg send/ack service.\n");
    rt_kprintf("Use 'mpc_perf' to start the four-wheel MPC closed-loop background loop.\n");
    rt_kprintf("Use 'model_perf' to start the model inference background loop.\n");
    rt_kprintf("Use 'sched_perf' to start the scheduler stress / multi-task interference test.\n");
    rt_kprintf("Fixed 1000 Hz test loops.\n");
    return 0;
}
Import('RTT_ROOT')
Import('rtconfig')
from building import *

cwd = GetCurrentDir()

if rtconfig.BOARD == 'os0_rcpu':
	src = [
		'main.c',
	]
	CPPPATH = [
		cwd,
	]
elif rtconfig.BOARD == 'os1_rcpu':
	src = [
		'rt-perf-test/01_ekf/ekf_perf_test.c',
		'rt-perf-test/02_foc/foc_perf_test.c',
		'rt-perf-test/03_signal/signal_perf_test.c',
		'rt-perf-test/04_AHRS/ahrs_perf_test.c',
  		'rt-perf-test/05_memory/mem_perf_test.c',
		'rt-perf-test/06_rtlat/rtlat_perf_test.c',
		'rt-perf-test/07_rpmsg/rpmsg_perf_test.c',
		'rt-perf-test/08_mpc/mpc_perf_test.c',
		'rt-perf-test/09_model_infer/simulate_model.c',
		'rt-perf-test/10_sched_stress/sched_stress_perf_test.c',
		'rt-perf-test/main.c',
	]
	CPPPATH = [
		cwd
	]
else:
	src = Glob('*.c')
	CPPPATH = [
		cwd,
	]

CCFLAGS = ' -c -ffunction-sections'

group   = DefineGroup('Applications', src, depend = [''], CPPPATH = CPPPATH, CCFLAGS=CCFLAGS)

Return('group')

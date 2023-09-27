#pragma once


#ifdef USE_SEGGER_RTT
#include "../rtt/SEGGER_RTT.h"
#define dbg_printf(...) SEGGER_RTT_printf(0, __VA_ARGS__)
#else
#define dbg_printf(...) printf(__VA_ARGS__)
#endif

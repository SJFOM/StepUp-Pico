#ifndef _PICO_STDLIB_H
#define _PICO_STDLIB_H

#include "../types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    static inline void setup_default_uart(void);
    static inline void set_sys_clock_48mhz(void);

    static inline void set_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2);

    static inline bool check_sys_clock_khz(uint32_t freq_khz,
                             uint *vco_freq_out,
                             uint *post_div1_out,
                             uint *post_div2_out)
    {
        return true;
    }

    static inline bool set_sys_clock_khz(uint32_t freq_khz, bool required)
    {
        return true;
    }

#ifdef __cplusplus
}
#endif
#endif

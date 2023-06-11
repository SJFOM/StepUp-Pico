#ifndef _PICO_STDLIB_H
#define _PICO_STDLIB_H

#ifdef __cplusplus
extern "C" {
#endif

void setup_default_uart(void);
void set_sys_clock_48mhz(void);

void set_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2);

bool check_sys_clock_khz(uint32_t freq_khz, uint *vco_freq_out, uint *post_div1_out, uint *post_div2_out){ return true;}

bool set_sys_clock_khz(uint32_t freq_khz, bool required) {
    return true;
}

#ifdef __cplusplus
}
#endif
#endif

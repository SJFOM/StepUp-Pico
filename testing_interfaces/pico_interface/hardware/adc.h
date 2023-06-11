#ifndef _HARDWARE_ADC_H_
#define _HARDWARE_ADC_H_

#include <stdint.h>
#include "../types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void adc_init(void);

    void adc_gpio_init(uint gpio);

    void adc_select_input(uint input);

    static inline uint adc_get_selected_input(void)
    {
        return 0;
    }

    void adc_set_round_robin(uint input_mask);

    void adc_set_temp_sensor_enabled(bool enable);

    static inline uint16_t adc_read(void)
    {
        return 0;
    }

    void adc_run(bool run);

    void adc_set_clkdiv(float clkdiv);

    void adc_fifo_setup(bool en,
                        bool dreq_en,
                        uint16_t dreq_thresh,
                        bool err_in_fifo,
                        bool byte_shift);

    static inline bool adc_fifo_is_empty(void)
    {
        return true;
    }

    static inline uint8_t adc_fifo_get_level(void)
    {
        return 0;
    }

    static inline uint16_t adc_fifo_get(void)
    {
        return 0;
    }

    static inline uint16_t adc_fifo_get_blocking(void)
    {
        return 0;
    }

    static inline void adc_fifo_drain(void);

    static inline void adc_irq_set_enabled(bool enabled);

#ifdef __cplusplus
}
#endif

#endif

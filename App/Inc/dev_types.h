#ifndef DEVTYPES_H
#define DEVTYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
    /* Main settings structure with sequence embedded */
    typedef struct
    {
        uint32_t sequence;
        uint8_t fireplace_type;
        uint8_t heaters_count;
        uint8_t disp_touch_snd;
        uint8_t sys_snd;
        uint8_t disp_brightness;
        uint8_t pin_lock_sys;
        uint8_t pin_lock_remote;
        uint16_t pin_code;
        uint8_t serial_num[7];
        uint8_t burn_level;
        uint8_t burn_lvl_pwm_duty[5];
        uint8_t mp3_volume;
        uint8_t mp3_def_track;
        uint8_t motor_sens_max_open;
        uint8_t motor_sens_close;
        uint8_t motor_sens_open;

        uint8_t ir_delay_burn;
        uint8_t ir_delay_preburn;
        uint32_t ir_time_def;
        uint32_t ir_time_cool;
        uint32_t ir_time_preburn;
        uint32_t ir_time_flame_detected;

        uint8_t pre_heat_duty;
        uint8_t plug_fault_time;

        float kf_fuel_fueling;
        float kf_fuel_burning;
        float kf_fuel_default;

        float kf_temperature;

        float kf_cap_sense;

        uint8_t cap_fault_time;

        float k_froad;
        float max_fuel_mm;
        float min_fuel_mm;

        // define уровней для ацп
        uint8_t t_lvl_heater;
        uint8_t t_lvl_5;
        uint8_t t_lvl_4;
        uint8_t t_lvl_3;
        uint8_t t_lvl_2;
        uint8_t t_lvl_1;
        uint8_t t_lvl_burn_3;
        uint8_t t_lvl_burn_2;
        uint8_t t_lvl_heater_start;
        uint8_t t_startup;

    } device_settings_t;

#ifdef __cplusplus
}
#endif

#endif // DEVTYPES_H
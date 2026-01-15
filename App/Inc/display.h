#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t addr;
    uint16_t value;
} dwin_write_msg_t;

typedef struct {
    uint16_t address;
    uint16_t data;
} dwin_read_msg_t;

extern QueueHandle_t dwin_write_queue;
extern QueueHandle_t dwin_read_queue;
extern TaskHandle_t dwin_task_handle;

void dwin_init();
BaseType_t dwin_send(uint16_t addr, uint16_t value);
BaseType_t dwin_receive(dwin_read_msg_t *msg);
void dwin_set_page(uint16_t page);


// Экраны дисплея
enum DisplayScreens {
    FUELING_SCREEN = 150,
    FULL_FUEL_SCREEN = 184,
    HEAT_SCREEN = 149,
    BURN_SCREEN = 145,
    BURN_5_SCREEN = 174,
    COOLING_SCREEN = 148,
    COOLING_FUEL_SCREEN = 173,
    STANDBY_SCREEN = 144,
    ENTER_PIN_SCREEN = 147,
    ENTER_NEW_PIN_SCREEN = 146,
    PIN_SETTING_SCREEN = 168,
    PIN_CHANGE_SCREEN = 169,
    WRONG_PIN_SCREEN = 170,
    PIN_UNLOCK_SCREEN = 171,
    LOW_FUEL_WAR_SCREEN = 163,
    OPEN_FUEL_WAR_SCREEN = 164,
    LEAK2_WAR_SCREEN = 165,
    WARNING_SCREEN = 166,
    LOCK_STANDBY_SCREEN = 185,
    LOCK_HEAT_SCREEN = 187,
    LOCK_BURN_SCREEN = 186,
    LOCK_WARNING_SCREEN = 188,
    LOCK_BURN_5_SCREEN = 179,
    ENTER_SETT_PIN_SCREEN = 189,
    FAST_PIN_CHANGE_SCREEN = 190,
    SLEEP_MENU_SCREEN = 191,
    WIFI_RESET_SCREEN = 192,
    OPEN_ELCAMINO_SCREEN = 152,
    HE_ERR_SCREEN = 153,
    PLUG_ERR_SCREEN = 154,
    IR_ERR_SCREEN = 155,
    TSENS_ERR_SCREEN = 156,
    FUEL_ERR_SCREEN = 157,
    LEAK1_ERR_SCREEN = 159,
    LEAK2_ERR_SCREEN = 160,
    LEAK1_PISS_SCREEN = 158,
    HI_T_WAR_SCREEN = 161,
    LOW_T_WAR_SCREEN = 162,
    PREFUEL_SCREEN = 180,
    END_FUEL_SCREEN = 183
};


// Команды для камина
enum DisplayCommands {
    HEAT_CMD = 0x10,
    PREFUEL_CMD = 0x12,
    START_FUEL_CMD = 0x13,
    STOP_FUEL_CMD = 0x14,
    END_FUEL_CMD = 0x15,
    PRECOOL_CMD = 0x20,
    COOL_CMD = 0x22,
    CANCEL_COOL_CMD = 0x23,
    LOCK_ELCAMINO_CMD = 0x24,
    UNLOCK_ELCAMINO_CMD = 0x25,
    WAIT_CMD = 0x00,
    SHOW_WAR_CMD = 0x26,
    CLEAR_PIN_CMD = 0x28,
    BACK_FROM_MENU_CMD = 0x30,
    BACK_FROM_PIN_ENTER_CMD = 0x36,
    SLEEP_COUNT_PLUS_CMD = 0x35,
    SLEEP_COUNT_MINUS_CMD = 0x34,
    RATATA_CMD = 0x37,
    MIR_CMD = 0x38,
    GOTO_SLEEP_MENU_CMD = 0x39,
    GOTO_WIFI_MENU_CMD = 0x40,
    RESET_WIFI_CMD = 0x41,
    ON = 0x02,
    OFF = 0x01
};

// Адреса для экрана
enum DisplayAddresses {
    FLAME_LVL_ADDR = 0x5006,
    FUEL_ADDR = 0x5008,
    DISP_SOUND_ADDR = 0x5012,
    SYS_SOUND_ADDR = 0x5010,
    PIN_LOCK_ADDR = 0x5000,
    CMD_ADDR = 0x5020,
    HEAT_COOL_ADDR = 0x5026,
    BRIGHT_ADDR = 0x0082,
    PIN_USE_433_ADDR = 0x5014,
    PIN_USE_ADDR = 0x5016,
    COUNT_PRECOOL_ADDR = 0x5034,
    COUNT_SEC_ICON_ADDR = 0x5036,
    MP3_PLAY_ADDR = 0x5044,
    PIN_ENTER_ADDR = 0x5046,
    HEAT_COOL_TIME_ADDR_H = 0x5048,
    HEAT_COOL_TIME_ADDR_L = 0x5078,
    MP3_VOLUME_ADDR = 0x5062,
    HOUR_FUEL_ADDR = 0x5030,
    MIN_FUEL_ADDR = 0x5032,
    DROVA_MP3_ADDR = 0x5052,
    FOREST_MP3_ADDR = 0x5054,
    RAIN_MP3_ADDR = 0x5056,
    SLEEP_ON_ADDR = 0x5058,
    SLEEP_TIME_ADDR = 0x5060
};

#ifdef __cplusplus
}
#endif

#endif // BUZZER_H
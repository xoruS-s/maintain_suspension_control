#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <mcp2515.h>
#include <GyverEncoder.h>
#include <EEPROM.h>
#include <GyverTM1637.h>
#include <avr/wdt.h>
#include <Wire.h>

// [ Датчики ]
#define SENSOR1 A2
#define SENSOR2 A3
#define SENSOR3 A5
#define SENSOR4 A4

// [ Энкодер ]
#define CLK 48
#define DT 47
#define SW 46

// [ Меню: кнопки управления ]
#define BTNUP 26
#define BTNDOWN 27
#define BTNOK 28
#define BTNCANCEL 29

// [ Кнопка сброса системы ] - [ dev ]
#define BTNRST 50

LiquidCrystal_I2C lcd(0x27, 16, 2);
LiquidCrystal_I2C lcd2(0x21, 20, 4);
Encoder enc(CLK, DT, SW);

unsigned long timing;

struct {
    float front_left;
    float front_right;
    float back_left;
    float back_right;
} sensors_data;
struct {
    int selected_mode;
    byte selected_hysteresis_percent[4];
} overall_options;
struct {
    byte hysteresis_percent[4];
    float sensors[4];
} eeprom_mode_1;
struct {
    byte hysteresis_percent[4];
    float sensors[4];
} eeprom_mode_2;
struct {
    byte hysteresis_percent[4];
    float sensors[4];
} eeprom_mode_3;
struct {
    byte hysteresis_percent[4];
    float sensors[4];
} eeprom_mode_4;
struct {
    byte hysteresis_percent[4];
    float sensors[4];
} eeprom_mode_5;

// [ Кастомные символы ]
uint8_t symbols[8][8] = {
    { 15, 9, 9, 9, 9, 9, 25, 0 },         // л
    { 31, 17, 17, 17, 17, 17, 17, 0 },    // п
    { 0, 0, 21, 14, 4, 14, 21, 0 },       // ж
    { 0, 0, 9, 9, 15, 1, 1, 0 },          // ч
    { 31, 16, 16, 16, 16, 16, 16, 0 },    // г
    { 17, 17, 19, 21, 25, 17, 17, 0 },    // и
    { 0, 0, 14, 10, 10, 31, 17, 0 },      // д
    { 0, 0, 28, 18, 28, 18, 28, 0 },      // в
};

// [ Флаги ]
bool btn_flags[] = { false, false, false, false };  // кнопки управления
bool btn_rst_flag = false;                          // кнопка сброса
bool enc_flag = false;                              // энкодер
bool blink_flag = false;                            // индикация изменяемого параметра

// [ Кнопки управления: долгое нажатие ]
int hold_time = 500;
int hold_time_sum = 0;

// [ 7-сегм. дисплей ]
int display_pins[] = { 3, 4, 5, 6, 8, 9, 10, 11 };
const char *modes[] = { "1", "2", "3", "4", "5", "E", "C", "P" };
int mode_index = 0;
int mode_index_menu = 0;
bool count_modes_start[] = { false, false, false, false, false };

// [ maintaining_suspension... ]
int current_wheel = 0;          // Текущее колесо
bool config_flag = false;       // флаг настроек

// [ Реле: пины ]
int relay[7] = { 34, 39, 37, 38, 40, 41, 36 };

// [ Датчики высоты ]
float sensors[4];
float const_sensors[4];

// [ Гистерезис ]
int hysteresis_percent[] = { 5, 5, 5, 5 };      // Процент гистерезиса
int max_hysteresis_perc = 50;                   // Максимальный процент гистерезиса
float hysteresis[8];                            // (1-4: нижний порог) - (5-8: верхний порог)

// [ Меню ]
const char *menu_names[] = {
    "    \2ep-\1eB    ",       // ПЛ
    "    \2ep-\2paB   ",       // ПП
    "    3a\7-\1eB      ",     // ЗЛ
    "    3a\7-\2paB     ",     // ЗП
    "     \2\1-\2\2     ",     // ПЛ-ПП
    "     3\1-3\2     ",       // ЗЛ-ЗП
    "  coxp. pe\3um  ",        // сохранить режим
    "     cbpoc      ",        // сброс настроек
};
const char *management_names[] = {
    "Y\2PAB\1EH\6E     ",
    "\5\6CTEPE3\6C     "
};
int menu_level = 1;                                             // Уровень
int menu_selected = 1;                                          // Текущий пункт меню
int management_selected = 1;                                    // Текущий пункт под-меню
int menu_lenght = sizeof(menu_names) / sizeof(menu_names[0]);   // Кол-во пунктов меню


// [ Основной блок ]
void setup () {
    Serial.begin(115200);
    Wire.begin();
    wdt_disable();

    enc.setType(TYPE2);

    pinMode(BTNUP, INPUT_PULLUP);
    pinMode(BTNDOWN, INPUT_PULLUP);
    pinMode(BTNOK, INPUT_PULLUP);
    pinMode(BTNCANCEL, INPUT_PULLUP);
    pinMode(BTNRST, INPUT_PULLUP);

    // [ 7-сегм: инициализация ]
    for (int i = 0; i < 8; i++) {
        pinMode(display_pins[i], OUTPUT);
        digitalWrite(display_pins[i], HIGH);
    }

    // [ Инициализация реле ]
    for (int i = 0; i < 7; i++) {
        pinMode(relay[i], OUTPUT);
        digitalWrite(relay[i], HIGH); // Выключение всех реле
    }

    // [ Чтение из памяти ]
    if (check_data()) {
        mode_index = EEPROM.get(0, overall_options).selected_mode - 1;
        enc_flag = true;
    } else {
        const_sensors[0] = map(analogRead(SENSOR1),0,1023,25,200);
        const_sensors[1] = map(analogRead(SENSOR2),0,1023,25,200);
        const_sensors[2] = map(analogRead(SENSOR3),0,1023,25,200);
        const_sensors[3] = map(analogRead(SENSOR4),0,1023,25,200);

        for (int i = 0; i < 4; i++) {
            eeprom_mode_1.hysteresis_percent[i] = 5;
            eeprom_mode_1.sensors[i] = 25;
            eeprom_mode_2.hysteresis_percent[i] = 5;
            eeprom_mode_2.sensors[i] = 25;
            eeprom_mode_3.hysteresis_percent[i] = 5;
            eeprom_mode_3.sensors[i] = 25;
            eeprom_mode_4.hysteresis_percent[i] = 5;
            eeprom_mode_4.sensors[i] = 25;
            eeprom_mode_5.hysteresis_percent[i] = 5;
            eeprom_mode_5.sensors[i] = 25;
        }
        overall_options.selected_mode = 8;
        enc_flag = true;

        EEPROM.put(0, overall_options);
        EEPROM.put(24, eeprom_mode_1);
        EEPROM.put(45, eeprom_mode_2);
        EEPROM.put(66, eeprom_mode_3);
        EEPROM.put(87, eeprom_mode_4);
        EEPROM.put(108, eeprom_mode_5);

        Serial.println("+++++++++++++++++++++++++");
    }

    // [ Дисплей: инициализация ]
    lcd.init();
    lcd2.init();
    lcd2.backlight();

    // [ Создание кастомных символов ]
    lcd.createChar(1, symbols[0]); // л
    lcd.createChar(2, symbols[1]); // п
    lcd.createChar(3, symbols[2]); // ж
    lcd.createChar(4, symbols[3]); // ч
    lcd.createChar(5, symbols[4]); // г
    lcd.createChar(6, symbols[5]); // и
    lcd.createChar(7, symbols[6]); // д
    lcd.createChar(8, symbols[7]); // в
}
void loop () {
    enc.tick();

    encoder_control();
    view_disp(mode_index);

    // if (!enc_flag) Serial.println(mode_index);

    if (millis() - timing > 100) {
        timing = millis();

        data_sensors();
        hysteresis_search();
        maintaining_suspension();

        if (enc_flag) {
            if (mode_index == 6) {
                lcd.backlight();

                byte error, address, nDevices;
                for (address = 1; address < 127; address++ ) {
                    Wire.beginTransmission(address);
                    error = Wire.endTransmission();
                    if (error == 0) nDevices++;
                }
                if (nDevices > 0) {
                    print_to_lcd();
                    control_buttons();
                }
            } else if (mode_index == 0) {
                if (!count_modes_start[0]) {
                    for (int i = 0; i< 4; i++) {
                        const_sensors[i] = EEPROM.get(24, eeprom_mode_1).sensors[i];
                        hysteresis_percent[i] = EEPROM.get(24, eeprom_mode_1).hysteresis_percent[i];
                    }
                    count_modes_start[0] = true;

                    count_modes_start[1] = false;
                    count_modes_start[2] = false;
                    count_modes_start[3] = false;
                    count_modes_start[4] = false;
                }
            } else if (mode_index == 1) {
                if (!count_modes_start[1]) {
                    for (int i = 0; i< 4; i++) {
                        const_sensors[i] = EEPROM.get(45, eeprom_mode_2).sensors[i];
                        hysteresis_percent[i] = EEPROM.get(45, eeprom_mode_2).hysteresis_percent[i];
                    }
                    count_modes_start[1] = true;

                    count_modes_start[0] = false;
                    count_modes_start[2] = false;
                    count_modes_start[3] = false;
                    count_modes_start[4] = false;
                }
            } else if (mode_index == 2) {
                if (!count_modes_start[2]) {
                    for (int i = 0; i< 4; i++) {
                        const_sensors[i] = EEPROM.get(66, eeprom_mode_3).sensors[i];
                        hysteresis_percent[i] = EEPROM.get(66, eeprom_mode_3).hysteresis_percent[i];
                    }
                    count_modes_start[2] = true;

                    count_modes_start[0] = false;
                    count_modes_start[1] = false;
                    count_modes_start[3] = false;
                    count_modes_start[4] = false;
                }
            } else if (mode_index == 3) {
                if (!count_modes_start[3]) {
                    for (int i = 0; i< 4; i++) {
                        const_sensors[i] = EEPROM.get(87, eeprom_mode_4).sensors[i];
                        hysteresis_percent[i] = EEPROM.get(87, eeprom_mode_4).hysteresis_percent[i];
                    }
                    count_modes_start[3] = true;

                    count_modes_start[0] = false;
                    count_modes_start[1] = false;
                    count_modes_start[2] = false;
                    count_modes_start[4] = false;
                }
            } else if (mode_index == 4) {
                if (!count_modes_start[4]) {
                    for (int i = 0; i< 4; i++) {
                        const_sensors[i] = EEPROM.get(108, eeprom_mode_5).sensors[i];
                        hysteresis_percent[i] = EEPROM.get(108, eeprom_mode_5).hysteresis_percent[i];
                    }
                    count_modes_start[4] = true;

                    count_modes_start[0] = false;
                    count_modes_start[1] = false;
                    count_modes_start[2] = false;
                    count_modes_start[3] = false;
                }
            } else if (mode_index == 5) {
                //
            } else if (mode_index == 6) {
                //
            } else if (mode_index == 7) {
                //
            }
        } else {
            lcd.clear();
            lcd.noBacklight();
        }
    }


    // if (mode_index == 5) {
    //     byte error, address;
    //     int nDevices = 0;
    //     for (address = 1; address < 127; address++ ) {
    //         Wire.beginTransmission(address);
    //         error = Wire.endTransmission();
    //
    //         if (error == 0) {
    //             nDevices++;
    //         }
    //     }
    //
    //     if (nDevices > 0) {
    //         print_to_lcd();
    //         control_buttons();
    //     }
    // }
}


void encoder_control() {
    // [ Управление энкодером ]
    if (!enc_flag) {
        if (enc.isRight()) {
            if (mode_index < 7) {
                mode_index++;
            } else {
                mode_index = 0;
            }
        }
        if (enc.isLeft()) {
            if (mode_index > 0) {
                mode_index--;
            } else {
                mode_index = 7;
            }
        }
    }

    if (enc.isClick()) {
        enc_flag = !enc_flag;

        if (enc_flag) {
            overall_options.selected_mode = mode_index + 1;
            if (mode_index >= 0 && mode_index <= 4) EEPROM.put(0, overall_options);
        }
    }
}
void control_buttons () {
    // [ Управление кнопками меню ]
    if (!digitalRead(BTNOK) && !btn_flags[0]) {             // OK
        btn_flags[0] = true;

        switch (menu_level) {
            case 1: menu_level++; lcd.clear(); break;
            case 2: menu_level++; lcd.clear(); break;
            case 3:
                menu_level--;
                lcd.clear();

                if (menu_selected == 7) {
                    switch (mode_index_menu) {
                        case 0:
                            for (int i = 0; i < 4; i++) {
                                eeprom_mode_1.hysteresis_percent[i] = hysteresis_percent[i];
                                eeprom_mode_1.sensors[i] = const_sensors[i];
                            }

                            EEPROM.put(24, eeprom_mode_1);

                            overall_options.selected_mode = 1;
                            EEPROM.put(0, overall_options);
                            break;
                        case 1:
                            for (int i = 0; i < 4; i++) {
                                eeprom_mode_2.hysteresis_percent[i] = hysteresis_percent[i];
                                eeprom_mode_2.sensors[i] = const_sensors[i];
                            }

                            EEPROM.put(45, eeprom_mode_2);

                            overall_options.selected_mode = 2;
                            EEPROM.put(0, overall_options);
                            break;
                        case 2:
                            for (int i = 0; i < 4; i++) {
                                eeprom_mode_3.hysteresis_percent[i] = hysteresis_percent[i];
                                eeprom_mode_3.sensors[i] = const_sensors[i];
                            }

                            EEPROM.put(66, eeprom_mode_3);

                            overall_options.selected_mode = 3;
                            EEPROM.put(0, overall_options);
                            break;
                        case 3:
                            for (int i = 0; i < 4; i++) {
                                eeprom_mode_4.hysteresis_percent[i] = hysteresis_percent[i];
                                eeprom_mode_4.sensors[i] = const_sensors[i];
                            }

                            EEPROM.put(87, eeprom_mode_4);

                            overall_options.selected_mode = 4;
                            EEPROM.put(0, overall_options);
                            break;
                        case 4:
                            for (int i = 0; i < 4; i++) {
                                eeprom_mode_5.hysteresis_percent[i] = hysteresis_percent[i];
                                eeprom_mode_5.sensors[i] = const_sensors[i];
                            }

                            EEPROM.put(108, eeprom_mode_5);

                            overall_options.selected_mode = 5;
                            EEPROM.put(0, overall_options);
                            break;
                    }
                    wdt_enable(WDTO_15MS);
                } else if (menu_selected == 8) {
                    //
                } else {
                    switch (management_selected) {//TODO
                        case 1: EEPROM.put(7, sensors_data); break;
                        case 2: EEPROM.put(0, overall_options); break;
                        default: break;
                    }
                }
                break;
            default: break;
        }
    } else if (!digitalRead(BTNCANCEL) && !btn_flags[1]) {  // CANCEL
        btn_flags[1] = true;

        switch (menu_level) {
            case 1: menu_selected = 1; lcd.clear(); break;
            case 2: menu_level--; lcd.clear(); break;
            case 3:
                menu_level--;
                lcd.clear();

                switch (management_selected) {
                    case 1: //TODO
                        // switch (menu_selected) {
                        //     case 1: const_sensors[0] = EEPROM.get(7, sensors_data).front_left; break;
                        //     case 2: const_sensors[1] = EEPROM.get(7, sensors_data).front_right; break;
                        //     case 3: const_sensors[2] = EEPROM.get(7, sensors_data).back_left; break;
                        //     case 4: const_sensors[3] = EEPROM.get(7, sensors_data).back_right; break;
                        //     case 5:
                        //         const_sensors[0] = EEPROM.get(7, sensors_data).front_left; break;
                        //         const_sensors[1] = EEPROM.get(7, sensors_data).front_right; break;
                        //         break;
                        //     case 6:
                        //         const_sensors[2] = EEPROM.get(7, sensors_data).back_left; break;
                        //         const_sensors[3] = EEPROM.get(7, sensors_data).back_right; break;
                        //         break;
                        //     default: break;
                        // }
                        break;
                    case 2:
                        // if (menu_selected >= 1 && menu_selected <= 4) {
                        //     hysteresis_percent[menu_selected - 1] = EEPROM.get(0, overall_options).selected_hysteresis_percent[menu_selected - 1];
                        // } else if (menu_selected == 5) {
                        //     hysteresis_percent[0] = EEPROM.get(0, overall_options).selected_hysteresis_percent[0];
                        //     hysteresis_percent[1] = EEPROM.get(0, overall_options).selected_hysteresis_percent[1];
                        // } else if (menu_selected == 6) {
                        //     hysteresis_percent[2] = EEPROM.get(0, overall_options).selected_hysteresis_percent[2];
                        //     hysteresis_percent[3] = EEPROM.get(0, overall_options).selected_hysteresis_percent[3];
                        // }
                        break;
                    default: break;
                }
                break;
            default: break;
        }
    } else if (!digitalRead(BTNUP) && !btn_flags[2]) {      // UP
        btn_flags[2] = true;

        switch (menu_level) {
            case 1:
                if (menu_selected > 1) {
                    menu_selected--;
                } else {
                    menu_selected = menu_lenght;
                }
                break;
            case 2:
                if (menu_selected == 7) {
                    if (mode_index_menu > 0) mode_index_menu--;
                    else mode_index_menu = 4;
                } else {
                    if (management_selected > 1) {
                        management_selected--;
                    } else {
                        management_selected = 2;
                    }
                }
                break;
            case 3:
                if (menu_selected >=1 && menu_selected <= 4) {
                    switch (management_selected) {
                        case 1:
                            if (const_sensors[menu_selected - 1] < 199) const_sensors[menu_selected - 1]++;
                            else const_sensors[menu_selected - 1] = 25;
                            break;
                        case 2:
                            if (hysteresis_percent[menu_selected - 1] < max_hysteresis_perc) {
                                hysteresis_percent[menu_selected - 1]++;
                            }
                            break;
                        default: break;
                    }
                } else if (menu_selected == 5) {
                    switch (management_selected) {
                        case 1:
                            if (const_sensors[0] < 199 && const_sensors[1] < 199) {
                                const_sensors[0]++;
                                const_sensors[1]++;
                            } else {
                                const_sensors[0] = 25;
                                const_sensors[1] = 25;
                            }
                            break;
                        case 2:
                            if (hysteresis_percent[0] < max_hysteresis_perc && hysteresis_percent[1] < max_hysteresis_perc) {
                                hysteresis_percent[0]++;
                                hysteresis_percent[1]++;
                            }
                            break;
                        default: break;
                    }
                } else if (menu_selected == 6) {
                    switch (management_selected) {
                        case 1:
                            if (const_sensors[2] < 199 && const_sensors[3] < 199) {
                                const_sensors[2]++;
                                const_sensors[3]++;
                            } else {
                                const_sensors[2] = 25;
                                const_sensors[3] = 25;
                            }
                            break;
                        case 2:
                            if (hysteresis_percent[2] < max_hysteresis_perc && hysteresis_percent[3] < max_hysteresis_perc) {
                                hysteresis_percent[2]++;
                                hysteresis_percent[3]++;
                            }
                            break;
                        default: break;
                    }
                }
                break;
            default: break;
        }
    } else if (!digitalRead(BTNDOWN) && !btn_flags[3]) {    // DOWN
        btn_flags[3] = true;

        switch (menu_level) {
            case 1:
                if (menu_selected < menu_lenght) {
                    menu_selected++;
                } else {
                    menu_selected = 1;
                }
                break;
            case 2:
                if (menu_selected == 7) {
                    if (mode_index_menu < 4) mode_index_menu++;
                    else mode_index_menu = 0;
                } else {
                    if (management_selected < 2) management_selected++;
                    else management_selected = 1;
                }
                break;
            case 3:
                if (menu_selected >= 1 && menu_selected <= 4) {
                    switch (management_selected) {
                        case 1:
                            if (const_sensors[menu_selected - 1] > 25) const_sensors[menu_selected - 1]--;
                            else const_sensors[menu_selected - 1] = 199;
                            break;
                        case 2:
                            if (hysteresis_percent[menu_selected - 1] > 0) {
                                hysteresis_percent[menu_selected - 1]--;
                            }
                            break;
                        default: break;
                    }
                } else if (menu_selected == 5) {
                    switch (management_selected) {
                        case 1:
                            if (const_sensors[0] > 25 && const_sensors[1] > 25) {
                                const_sensors[0]--;
                                const_sensors[1]--;
                            } else {
                                const_sensors[0] = 199;
                                const_sensors[1] = 199;
                            }
                            break;
                        case 2:
                            if (hysteresis_percent[0] > 0 && hysteresis_percent[1] > 0) {
                                hysteresis_percent[0]--;
                                hysteresis_percent[1]--;
                            }
                            break;
                        default: break;
                    }
                } else if (menu_selected == 6) {
                    switch (management_selected) {
                        case 1:
                            if (const_sensors[2] > 25 && const_sensors[3] > 25) {
                                const_sensors[2]--;
                                const_sensors[3]--;
                            } else {
                                const_sensors[2] = 199;
                                const_sensors[3] = 199;
                            }
                            break;
                        case 2:
                            if (hysteresis_percent[2] > 0 && hysteresis_percent[3] > 0) {
                                hysteresis_percent[2]--;
                                hysteresis_percent[3]--;
                            }
                            break;
                        default: break;
                    }
                }
                break;
            default: break;
        }
    }

    if (digitalRead(BTNOK) && btn_flags[0]) {
        btn_flags[0] = false;
    } else if (digitalRead(BTNCANCEL) && btn_flags[1]) {
        btn_flags[1] = false;
    } else if (digitalRead(BTNUP) && btn_flags[2]) {
        btn_flags[2] = false;
    } else if (digitalRead(BTNDOWN) && btn_flags[3]) {
        btn_flags[3] = false;
    }

    if (!digitalRead(BTNRST) && !btn_rst_flag) {
        btn_rst_flag = true;

        for (int i = 0; i < 4; i++) {
            overall_options.selected_hysteresis_percent[i] = 0;
            eeprom_mode_1.hysteresis_percent[i] = 0;
            eeprom_mode_1.sensors[i] = 0;
            eeprom_mode_2.hysteresis_percent[i] = 0;
            eeprom_mode_2.sensors[i] = 0;
            eeprom_mode_3.hysteresis_percent[i] = 0;
            eeprom_mode_3.sensors[i] = 0;
            eeprom_mode_4.hysteresis_percent[i] = 0;
            eeprom_mode_4.sensors[i] = 0;
        }

        EEPROM.put(0, overall_options);
        EEPROM.put(24, eeprom_mode_1);
        EEPROM.put(45, eeprom_mode_2);
        EEPROM.put(66, eeprom_mode_3);
        EEPROM.put(87, eeprom_mode_4);
        EEPROM.put(108, eeprom_mode_5);

        wdt_enable(WDTO_15MS);
    } else if (digitalRead(BTNRST) && btn_rst_flag) {
        btn_rst_flag = false;
    }
}
void print_to_lcd () {
    // [ Вывод на экран ]
    switch (menu_level) {
        case 1: show_menu(); break;
        case 2: show_management(); break;
        case 3:
            if (menu_selected == 7) {
                save_mode_settings();
            } else {
                switch (management_selected) {
                    case 1: edit_management_value(); break;
                    case 2: edit_gysteresis(); break;
                    default: break;
                }
            }
            break;
        default: break;
    }
}

void show_menu () {
    lcd.setCursor(0, 0);
    lcd.print("  -py\4. pe\3um-  ");
    lcd.setCursor(0, 1);
    lcd.print(menu_names[menu_selected - 1]);
}
void show_management () {
    lcd.setCursor(0, 0);
    lcd.print(menu_names[menu_selected - 1]);
    lcd.setCursor(0, 1);
    if (menu_selected == 7) {
        lcd.print("pe\3um: ");
        lcd.print(modes[mode_index_menu]);
    } else {
        lcd.print(management_names[management_selected - 1]);
    }
}

void edit_management_value () {
    if (menu_selected != 0) {
        lcd.setCursor(0, 0);
        lcd.print(management_names[management_selected - 1]);

        lcd.setCursor(0, 1);
        switch (menu_selected) {
            case 1:
                lcd.print("\2-\1");
                sensors_data.front_left = const_sensors[0];
                break;
            case 2:
                lcd.print("\2-\2");
                sensors_data.front_right = const_sensors[1];
                break;
            case 3:
                lcd.print("3-\1");
                sensors_data.back_left = const_sensors[2];
                break;
            case 4:
                lcd.print("3-\2");
                sensors_data.back_right = const_sensors[3];
                break;
            case 5:
                lcd.print("\2\1-\2\2");
                break;
            case 6:
                lcd.print("3\1-3\2");
                break;
            default: break;
        }
        lcd.print(":");

        if (menu_selected >= 1 && menu_selected <= 4) {
            lcd.setCursor(6, 1);
            if (int(const_sensors[menu_selected - 1]) < 100) {
                lcd.print(int(const_sensors[menu_selected - 1]));
                lcd.setCursor(8, 1);
                lcd.print(" ");
            } else {
                lcd.print(int(const_sensors[menu_selected - 1]));
            }

        } else if (menu_selected == 5) {
            float overall_front = (const_sensors[0] + const_sensors[1]) / 2;
            lcd.print(int(overall_front));
        } else if (menu_selected == 6) {
            float overall_back = (const_sensors[2] + const_sensors[3]) / 2;
            lcd.print(int(overall_back));
        }
        // lcd.print("<");

        // Текущие показатели
        lcd.setCursor(13, 1);

        if (menu_selected >= 1 && menu_selected <= 4) {
            lcd.print(sensors[menu_selected - 1]);
        } else if (menu_selected == 5) {
            float overall_front = (sensors[0] + sensors[1]) / 2;
            lcd.print(overall_front);
        } else if (menu_selected == 6) {
            float overall_back = (sensors[2] + sensors[3]) / 2;
            lcd.print(overall_back);
        }

    }
}
void edit_gysteresis () {
    if (menu_selected != 0) {
        lcd.setCursor(0, 0);
        lcd.print(management_names[management_selected - 1]);

        lcd.setCursor(0, 1);
        switch (menu_selected) {
            case 1: lcd.print("\2-\1"); break;
            case 2: lcd.print("\2-\2"); break;
            case 3: lcd.print("3-\1"); break;
            case 4: lcd.print("3-\2"); break;
            case 5: lcd.print("\2\1-\2\2"); break;
            case 6: lcd.print("3\1-3\2"); break;
            default: break;
        }
        lcd.print(":");
        lcd.setCursor(6, 1);

        if (menu_selected >= 1 && menu_selected <= 4) {
            overall_options.selected_hysteresis_percent[menu_selected - 1] = hysteresis_percent[menu_selected - 1];

            if (hysteresis_percent[menu_selected - 1] < 10) {
                lcd.setCursor(8, 1);
                lcd.print(" ");
            }
            lcd.setCursor(6, 1);
            lcd.print(hysteresis_percent[menu_selected - 1]);
        } else if (menu_selected == 5) {
            overall_options.selected_hysteresis_percent[0] = hysteresis_percent[0];
            overall_options.selected_hysteresis_percent[1] = hysteresis_percent[1];

            if (hysteresis_percent[menu_selected - 1] < 10) {
                lcd.setCursor(8, 1);
                lcd.print(" ");
            }
            lcd.setCursor(6, 1);
            lcd.print((hysteresis_percent[0] + hysteresis_percent[1]) / 2);
        } else if (menu_selected == 6) {
            overall_options.selected_hysteresis_percent[2] = hysteresis_percent[2];
            overall_options.selected_hysteresis_percent[3] = hysteresis_percent[3];

            if (hysteresis_percent[menu_selected - 1] < 10) {
                lcd.setCursor(8, 1);
                lcd.print(" ");
            }
            lcd.setCursor(6, 1);
            lcd.print((hysteresis_percent[2] + hysteresis_percent[3]) / 2);
        }


        lcd.print("%");
        // lcd.print("<");
    }
}
void save_mode_settings () {
    lcd.setCursor(0, 0);
    lcd.print(menu_names[menu_selected - 1]);
    lcd.setCursor(0, 1);
    lcd.print("pe\3um: ");
    lcd.print(modes[mode_index_menu]);
    lcd.print(" -coxp?");


}

void data_sensors () {
    // [ Данные с датчиков: текущее ]
    sensors[0] = map(analogRead(SENSOR1),0,1023,25,200);
    sensors[1] = map(analogRead(SENSOR2),0,1023,25,200);
    sensors[2] = map(analogRead(SENSOR3),0,1023,25,200);
    sensors[3] = map(analogRead(SENSOR4),0,1023,25,200);
}
void hysteresis_search () {
    // [ Гистерезис: определение ]
    for (int i = 0; i < 4; i++) {
        hysteresis[i] = const_sensors[i] - ((const_sensors[i] / 100) * hysteresis_percent[i]);
        hysteresis[i + 4] = const_sensors[i] + ((const_sensors[i] / 100) * hysteresis_percent[i]);
    }
}
void maintaining_suspension () {
    // [ Поддержание уровня пневмоподвески ]
    if (enc_flag) {
        if (mode_index == 6) {
            if (sensors[current_wheel] < const_sensors[current_wheel] - ((const_sensors[current_wheel] / 100) * 1.5)) {
            // [ON]
            digitalWrite(relay[4], LOW);              // [ВКЛ] Компрессор
            digitalWrite(relay[5], LOW);              // [ВКЛ] Клапан подъема
            digitalWrite(relay[current_wheel], LOW);  // [ВКЛ] Клапан текущее колесо

            // [OFF]
            digitalWrite(relay[6], HIGH);             // [ВЫКЛ] Клапан спуска
            } else if (sensors[current_wheel] > const_sensors[current_wheel] + ((const_sensors[current_wheel] / 100) * 1.5)) {
                    // [ON]
                    digitalWrite(relay[6], LOW);              // [ВКЛ] Клапан спуска
                    digitalWrite(relay[current_wheel], LOW);  // [ВКЛ] Клапан текущее колесо

                    // [OFF]
                    digitalWrite(relay[4], HIGH);             // [ВЫКЛ] Компрессор
                    digitalWrite(relay[5], HIGH);             // [ВЫКЛ] Клапан подъема
            } else if (sensors[current_wheel] > const_sensors[current_wheel] - ((const_sensors[current_wheel] / 100) * 1.5) && sensors[current_wheel] < const_sensors[current_wheel] + ((const_sensors[current_wheel] / 100) * 1.5)) {
                digitalWrite(relay[4], HIGH);             // [ВЫКЛ] Компрессор
                digitalWrite(relay[5], HIGH);             // [ВЫКЛ] Клапан подъема
                digitalWrite(relay[6], HIGH);             // [ВЫКЛ] Клапан спуска
                digitalWrite(relay[current_wheel], HIGH); // [ВЫКЛ] Клапан текущее колесо

                // [ Проверка каждого колеса ]
                if (current_wheel < 3) {
                        current_wheel++;
                } else if (current_wheel == 3) {
                        current_wheel = 0;
                    }
            }
        } else if (mode_index == 5) {
            for (int i = 0; i < 7; i++) digitalWrite(relay[i], HIGH);
        } else if (mode_index == 7) {
            //
        } else {
            if (sensors[current_wheel] < hysteresis[current_wheel]) {
                // [ON]
                digitalWrite(relay[4], LOW);              // [ВКЛ] Компрессор
                digitalWrite(relay[5], LOW);              // [ВКЛ] Клапан подъема
                digitalWrite(relay[current_wheel], LOW);  // [ВКЛ] Клапан текущее колесо

                // [OFF]
                digitalWrite(relay[6], HIGH);             // [ВЫКЛ] Клапан спуска
            } else if (sensors[current_wheel] > hysteresis[current_wheel + 4]) {
                // [ON]
                digitalWrite(relay[6], LOW);              // [ВКЛ] Клапан спуска
                digitalWrite(relay[current_wheel], LOW);  // [ВКЛ] Клапан текущее колесо

                // [OFF]
                digitalWrite(relay[4], HIGH);             // [ВЫКЛ] Компрессор
                digitalWrite(relay[5], HIGH);             // [ВЫКЛ] Клапан подъема
            } else if (sensors[current_wheel] > hysteresis[current_wheel] && sensors[current_wheel] < hysteresis[current_wheel + 4]) {
                digitalWrite(relay[4], HIGH);             // [ВЫКЛ] Компрессор
                digitalWrite(relay[5], HIGH);             // [ВЫКЛ] Клапан подъема
                digitalWrite(relay[6], HIGH);             // [ВЫКЛ] Клапан спуска
                digitalWrite(relay[current_wheel], HIGH); // [ВЫКЛ] Клапан текущее колесо

                // [ Проверка каждого колеса ]
                if (current_wheel < 3) {
                    current_wheel++;
                } else if (current_wheel == 3) {
                    current_wheel = 0;
                }
            }
        }
    } else {
        if (sensors[current_wheel] < hysteresis[current_wheel]) {
            // [ON]
            digitalWrite(relay[4], LOW);              // [ВКЛ] Компрессор
            digitalWrite(relay[5], LOW);              // [ВКЛ] Клапан подъема
            digitalWrite(relay[current_wheel], LOW);  // [ВКЛ] Клапан текущее колесо

            // [OFF]
            digitalWrite(relay[6], HIGH);             // [ВЫКЛ] Клапан спуска
        } else if (sensors[current_wheel] > hysteresis[current_wheel + 4]) {
                // [ON]
                digitalWrite(relay[6], LOW);              // [ВКЛ] Клапан спуска
                digitalWrite(relay[current_wheel], LOW);  // [ВКЛ] Клапан текущее колесо

                // [OFF]
                digitalWrite(relay[4], HIGH);             // [ВЫКЛ] Компрессор
                digitalWrite(relay[5], HIGH);             // [ВЫКЛ] Клапан подъема
        } else if (sensors[current_wheel] > hysteresis[current_wheel] && sensors[current_wheel] < hysteresis[current_wheel + 4]) {
            digitalWrite(relay[4], HIGH);             // [ВЫКЛ] Компрессор
            digitalWrite(relay[5], HIGH);             // [ВЫКЛ] Клапан подъема
            digitalWrite(relay[6], HIGH);             // [ВЫКЛ] Клапан спуска
            digitalWrite(relay[current_wheel], HIGH); // [ВЫКЛ] Клапан текущее колесо

        // [ Проверка каждого колеса ]
        if (current_wheel < 3) {
            current_wheel++;
        } else if (current_wheel == 3) {
            current_wheel = 0;
        }
    }
    }
}

// [ Дополнительные функции ]
bool check_data () {
    int data[] = {
        EEPROM.get(0, overall_options).selected_hysteresis_percent[0],
        EEPROM.get(0, overall_options).selected_hysteresis_percent[1],
        EEPROM.get(0, overall_options).selected_hysteresis_percent[2],
        EEPROM.get(0, overall_options).selected_hysteresis_percent[3]
    };

    Serial.print(data[0]);

    // if (data[0] != 0 && data[1] != 0 && data[2] != 0 && data[3] != 0) return true;
    if (1 > 0) return true;
    else return false;
}
void view_disp (int mode) {
    // 3  - средний
    // 4  - верхний-левый
    // 5  - верхний
    // 6  - верхний-правый
    // 8  - нижний
    // 9  - нижний-левый
    // 10 - нижний-правый
    // 11 - точка

    switch (mode) {
        case 0:
            digitalWrite(6, LOW);
            digitalWrite(10, LOW);

            digitalWrite(3, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(5, HIGH);
            digitalWrite(8, HIGH);
            digitalWrite(9, HIGH);
            break;
        case 1:
            digitalWrite(5, LOW);
            digitalWrite(6, LOW);
            digitalWrite(3, LOW);
            digitalWrite(9, LOW);
            digitalWrite(8, LOW);

            digitalWrite(4, HIGH);
            digitalWrite(10, HIGH);
            break;
        case 2:
            digitalWrite(5, LOW);
            digitalWrite(6, LOW);
            digitalWrite(3, LOW);
            digitalWrite(10, LOW);
            digitalWrite(8, LOW);

            digitalWrite(4, HIGH);
            digitalWrite(9, HIGH);
            break;
        case 3:
            digitalWrite(4, LOW);
            digitalWrite(6, LOW);
            digitalWrite(3, LOW);
            digitalWrite(10, LOW);

            digitalWrite(9, HIGH);
            digitalWrite(8, HIGH);
            digitalWrite(5, HIGH);
            break;
        case 4:
            digitalWrite(5, LOW);
            digitalWrite(4, LOW);
            digitalWrite(3, LOW);
            digitalWrite(10, LOW);
            digitalWrite(8, LOW);

            digitalWrite(6, HIGH);
            digitalWrite(9, HIGH);
            break;
        case 5:
            digitalWrite(5, LOW);
            digitalWrite(4, LOW);
            digitalWrite(3, LOW);
            digitalWrite(9, LOW);
            digitalWrite(8, LOW);

            digitalWrite(6, HIGH);
            digitalWrite(10, HIGH);
            break;
        case 6:
            digitalWrite(5, LOW);
            digitalWrite(4, LOW);
            digitalWrite(9, LOW);
            digitalWrite(8, LOW);

            digitalWrite(3, HIGH);
            digitalWrite(6, HIGH);
            digitalWrite(10, HIGH);
            break;
        case 7:
            digitalWrite(5, LOW);
            digitalWrite(4, LOW);
            digitalWrite(3, LOW);
            digitalWrite(9, LOW);
            digitalWrite(6, LOW);

            digitalWrite(8, HIGH);
            digitalWrite(10, HIGH);
            break;
        default: break;
    }

    if (enc_flag) {
        digitalWrite(11, LOW);
    } else {
        digitalWrite(11, HIGH);
    }
}



// [ DEV ]
void dev_serial(int v) {
    switch (v) {
        case 1:
            Serial.print("  lvl: ");
            Serial.print(menu_level);
            Serial.print("  menu_sel: ");
            Serial.print(menu_selected);
            Serial.print("  manag_sel: ");
            Serial.print(management_selected);
            Serial.println();
            break;
        case 2:
            Serial.print("  tl: ");
            Serial.print(const_sensors[0]);
            Serial.print("  tr: ");
            Serial.print(const_sensors[1]);
            Serial.print("  bl: ");
            Serial.print(const_sensors[2]);
            Serial.print("  br: ");
            Serial.print(const_sensors[3]);
            Serial.println();
            break;
        case 3:
            Serial.print("current: ");
            Serial.print(sensors[0]);
            Serial.print(" ");
            Serial.print(sensors[1]);
            Serial.print(" ");
            Serial.print(sensors[2]);
            Serial.print(" ");
            Serial.print(sensors[3]);
            Serial.print("  ");
            Serial.print("const: ");
            Serial.print(const_sensors[0]);
            Serial.print(" ");
            Serial.print(const_sensors[1]);
            Serial.print(" ");
            Serial.print(const_sensors[2]);
            Serial.print(" ");
            Serial.print(const_sensors[3]);
            Serial.print("  ");
            Serial.print("hyst-lower: ");
            Serial.print(hysteresis[0]);
            Serial.print(" ");
            Serial.print(hysteresis[1]);
            Serial.print(" ");
            Serial.print(hysteresis[2]);
            Serial.print(" ");
            Serial.print(hysteresis[3]);
            Serial.print("  ");
            Serial.print("hyst-apper: ");
            Serial.print(hysteresis[4]);
            Serial.print(" ");
            Serial.print(hysteresis[5]);
            Serial.print(" ");
            Serial.print(hysteresis[6]);
            Serial.print(" ");
            Serial.print(hysteresis[7]);
            Serial.print("  ");
            Serial.print("hyst_perc: ");
            Serial.print(hysteresis_percent[menu_selected]);
            Serial.println();
            break;
        case 4:
            Serial.print("hyst_perc: ");
            Serial.print(EEPROM.get(0, overall_options).selected_hysteresis_percent[menu_selected]);
            Serial.println();
            break;
        case 5:
            Serial.print("overall_options: ");
            Serial.print("mode: ");
            Serial.print(EEPROM.get(0, overall_options).selected_mode);
            Serial.print(" hysteresis: ");
            Serial.print(EEPROM.get(0, overall_options).selected_hysteresis_percent[menu_selected]);
            Serial.print("  sensors_data: ");
            Serial.print(EEPROM.get(7, sensors_data).front_left);
            Serial.print(" ");
            Serial.print(EEPROM.get(7, sensors_data).front_right);
            Serial.print(" ");
            Serial.print(EEPROM.get(7, sensors_data).back_left);
            Serial.print(" ");
            Serial.print(EEPROM.get(7, sensors_data).back_right);
            Serial.println();
            break;
        case 6:
        Serial.print("MODE: ");
            Serial.print(mode_index + 1);
            Serial.print("  ");
            for (int i = 0; i < 4; i++) {
                Serial.print(const_sensors[i]);
                Serial.print(" ");
            }
            Serial.print("    ");
            for (int i = 0; i < 4; i++) {
                Serial.print(hysteresis_percent[i]);
                Serial.print(" ");
            }
            Serial.println();
            // Serial.print("   ");
            // for (int i = 0; i < 4; i++) {
            //     Serial.print(EEPROM.get(24, eeprom_mode_1).hysteresis_percent[i]);
            //     Serial.print(" ");
            // }
            // Serial.println();
        default: break;
    }
}
void view_data_lcd_dev() {


    lcd2.setCursor(0, 0);
    lcd2.print("mode:");
    lcd2.print(mode_index + 1);
    lcd2.setCursor(0, 1);
    lcd2.print(int(const_sensors[0]));
    lcd2.setCursor(4, 1);
    lcd2.print(int(const_sensors[1]));
    lcd2.setCursor(8, 1);
    lcd2.print(int(const_sensors[2]));
    lcd2.setCursor(12, 1);
    lcd2.print(int(const_sensors[3]));

    lcd2.setCursor(0, 2);
    lcd2.print(int(sensors[0]));
    lcd2.setCursor(4, 2);
    lcd2.print(int(sensors[1]));
    lcd2.setCursor(8, 2);
    lcd2.print(int(sensors[2]));
    lcd2.setCursor(12, 2);
    lcd2.print(int(sensors[3]));

    lcd2.setCursor(0, 3);
    lcd2.print(int(hysteresis_percent[0]));
    lcd2.setCursor(4, 3);
    lcd2.print(int(hysteresis_percent[1]));
    lcd2.setCursor(8, 3);
    lcd2.print(int(hysteresis_percent[2]));
    lcd2.setCursor(12, 3);
    lcd2.print(int(hysteresis_percent[3]));
}




























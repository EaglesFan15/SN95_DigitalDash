#define CONFIG_LVGL_PORT_AVOID_TEARING_MODE 0
#include <Arduino.h>
#include <ESP_Panel_Library.h> 
#include <lvgl.h>
#include <ui.h>                     
#include "lvgl_v8_port.h"
#include "driver/twai.h"
#include <TinyGPSPlus.h>
#include <Preferences.h> 

// --- HOLLEY CHANNEL INDEXES (Priority/Serial Agnostic) ---
#define INDEX_RPM      0x0E005000 
#define INDEX_CL_STAT  0x0E039000
#define INDEX_LEARN    0x0E03D000
#define INDEX_MAT      0x0E079000 
#define INDEX_CTS      0x0E07D000 
#define INDEX_VOLTAGE  0x0E085000 
#define INDEX_OIL_P    0x0E089000
#define INDEX_FUEL_P   0x0E08D000 
#define INDEX_FAN_1    0x0E0B1000

// --- PIN DEFINITIONS ---
#define CAN_TX_IO      15   
#define CAN_RX_IO      16   
#define GPS_RX_PIN      8   
#define GPS_TX_PIN      9   
#define DEBUG_INTERVAL 2000 

TinyGPSPlus gps;
HardwareSerial SerialGPS(1); 
esp_panel::board::Board board_obj; 
Preferences prefs;

// Global Sensor Variables
float v_volt = 0, v_fuel = 0, v_mat = 0, v_cts = 0, v_oil = 0;
int v_rpm = 0;
bool v_fan = false, v_cloop = false, v_learn = false;

// Odometer & Trip Variables
double total_odo = 146473.0; 
double total_trip = 0.0;
double last_lat = 0, last_lng = 0;
bool first_fix = true;

// Timing/Diagnostic Variables
bool lvgl_ready = false;
uint32_t can_msg_count = 0;
unsigned long last_debug_ms = 0;
unsigned long last_ui_draw = 0; 
unsigned long last_save_ms = 0;
unsigned long v_can_last_ms = 0;

static void trip_reset_handler(lv_event_t * e) {
    if(lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) {
        total_trip = 0;
        prefs.putDouble("trip", total_trip);
        Serial.println(">>> TRIP METER RESET <<<");
    }
}

// Decode Float from Holley bytes 0-3 (big-endian IEEE 754)
float decodeHolleyFloat(const uint8_t* b) {
    union { uint32_t raw; float f; } dat;
    dat.raw = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
    return (isnan(dat.f) || isinf(dat.f)) ? 0.0f : dat.f;
}

void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_IO, (gpio_num_t)CAN_RX_IO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    
    // REVERTED TO ACCEPT ALL: Ensures the hardware doesn't block any varying Holley IDs
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); 

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("CAN install failed");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("CAN start failed");
    } else {
        Serial.println("CAN Started - Accepting All IDs (Software Filtering)");
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    prefs.begin("car_data", false);
    total_odo = prefs.getDouble("odo", 146473.0);
    total_trip = prefs.getDouble("trip", 0.0);

    auto lcd = board_obj.getLCD();
    if (lcd) lcd->configFrameBufferNumber(1); 
    board_obj.init();
    board_obj.begin();

    lvgl_ready = lvgl_port_init(board_obj.getLCD(), board_obj.getTouch());
    if (lvgl_ready) {
        lvgl_port_lock(200);
        ui_init(); 
        if(ui_TripPanel) {
            lv_obj_add_flag(ui_TripPanel, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(ui_TripPanel, trip_reset_handler, LV_EVENT_ALL, NULL);
        }
        lvgl_port_unlock();
    }
    SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    initCAN();
}

void loop() {
    // 1. GPS Processing
    while (SerialGPS.available() > 0) gps.encode(SerialGPS.read());
    if (gps.location.isValid() && gps.speed.mph() > 1.5) {
        if (first_fix) { 
            last_lat = gps.location.lat(); 
            last_lng = gps.location.lng(); 
            first_fix = false; 
        } else {
            double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), last_lat, last_lng) * 0.000621371;
            if (dist > 0.001) { 
                total_odo += dist; 
                total_trip += dist; 
                last_lat = gps.location.lat(); 
                last_lng = gps.location.lng(); 
            }
        }
    }

    // 2. CAN Processing
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        v_can_last_ms = millis();
        can_msg_count++;
        
        // SOFTWARE MASK: Ignore first nibble (Priority) and last 11 bits (Serial)
        uint32_t masked_id = msg.identifier & 0x0FFFF800;
        float value = decodeHolleyFloat(msg.data);

        switch (masked_id) {
            case INDEX_RPM:      v_rpm = (int)value;     break;
            case INDEX_VOLTAGE:  v_volt = value;         break;
            case INDEX_FUEL_P:   v_fuel = value;         break;
            case INDEX_OIL_P:    v_oil = value;          break;
            case INDEX_CTS:      v_cts = value;          break;
            case INDEX_MAT:      v_mat = value;          break;
            case INDEX_FAN_1:    v_fan = (value > 0.5f); break;
            case INDEX_CL_STAT:  v_cloop = (value > 0.5f);break;
            case INDEX_LEARN:    v_learn = (value > 0.5f);break;
        }
    }

    // 3. Serial Debug Summary
    if (millis() - last_debug_ms > DEBUG_INTERVAL) {
        if (millis() - v_can_last_ms > 2000) {
            Serial.println("WARNING: CAN BUS OFFLINE");
        } else {
            Serial.printf("RPM: %d | VOLT: %.1f | FUEL: %.1f | CTS: %.1f | MSGS: %u\n", 
                          v_rpm, v_volt, v_fuel, v_cts, can_msg_count);
        }
        last_debug_ms = millis();
    }

    // 4. Update UI
    if (lvgl_ready && (millis() - last_ui_draw > 50)) {
        if (lvgl_port_lock(20)) {
            if (ui_RPMLabel) lv_label_set_text_fmt(ui_RPMLabel, "%d", (v_rpm < 50) ? 0 : v_rpm);
            if (ui_MAT) lv_label_set_text_fmt(ui_MAT, "%d", (int)v_mat);
            if (ui_FuelPressure) lv_label_set_text_fmt(ui_FuelPressure, "%d", (int)v_fuel);
            if (ui_OdometerLabel) lv_label_set_text_fmt(ui_OdometerLabel, "%d", (int)total_odo);

            if (ui_Voltage) {
                char buf[8];
                snprintf(buf, sizeof(buf), "%.1f", (double)v_volt);
                lv_label_set_text(ui_Voltage, buf);
            }
            
            if (ui_TripLabel) {
                int t_whole = (int)total_trip;
                int t_dec = (int)((total_trip - (double)t_whole) * 10);
                lv_label_set_text_fmt(ui_TripLabel, "%d.%d", t_whole, abs(t_dec));
            }

            if (ui_MPH) {
                if (!gps.location.isValid()) lv_label_set_text(ui_MPH, "0"); 
                else lv_label_set_text_fmt(ui_MPH, "%d", (int)gps.speed.mph());
            }

            if (ui_FanStatus) {
                lv_label_set_text(ui_FanStatus, v_fan ? "ON" : "OFF");
                lv_obj_set_style_text_color(ui_FanStatus, v_fan ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_RED), 0);
            }

            if (ui_CLSDLoopSwitch) {
                if (v_cloop) lv_obj_add_state(ui_CLSDLoopSwitch, LV_STATE_CHECKED);
                else lv_obj_clear_state(ui_CLSDLoopSwitch, LV_STATE_CHECKED);
            }

            if (ui_LearnSwitch) {
                if (v_learn) lv_obj_add_state(ui_LearnSwitch, LV_STATE_CHECKED);
                else lv_obj_clear_state(ui_LearnSwitch, LV_STATE_CHECKED);
            }

            lvgl_port_unlock();
        }
        last_ui_draw = millis();
    }

    // 5. Save Data every 30s
    if (millis() - last_save_ms > 30000) {
        prefs.putDouble("odo", total_odo);
        prefs.putDouble("trip", total_trip);
        last_save_ms = millis();
    }
    yield();
}

#define CONFIG_LVGL_PORT_AVOID_TEARING_MODE 0
#include <Arduino.h>
#include <ESP_Panel_Library.h> 
#include <lvgl.h>
#include <ui.h>                     
#include "lvgl_v8_port.h"
#include "driver/twai.h"
#include <TinyGPSPlus.h>
#include <Preferences.h> 

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
float v_volt = 0, v_fuel = 0, v_mat = 0, v_cts = 0;
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

// Decode Float from Holley 8-byte payload (Bytes 0-3)
float decodeHolleyFloat(byte* b) {
    union { float f; uint32_t raw; } dat;
    dat.raw = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
    return (isnan(dat.f) || isinf(dat.f)) ? 0.0 : dat.f;
}

// Status is in the second 4-byte block of the payload
bool decodeHolleyStatus(byte* b) {
    return b[7] > 0;
}

void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_IO, (gpio_num_t)CAN_RX_IO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbit/sec rate
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); 
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
}

void setup() {
    Serial.begin(115200);
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
    // 1. Process GPS
    while (SerialGPS.available() > 0) gps.encode(SerialGPS.read());
    if (gps.location.isValid() && gps.speed.mph() > 1.5) {
        if (first_fix) { last_lat = gps.location.lat(); last_lng = gps.location.lng(); first_fix = false; }
        else {
            double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), last_lat, last_lng) * 0.000621371;
            if (dist > 0.001) { total_odo += dist; total_trip += dist; last_lat = gps.location.lat(); last_lng = gps.location.lng(); }
        }
    }

    // 2. Process CAN & Serial Sniffing
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        v_can_last_ms = millis();
        can_msg_count++;
        
        // Target Serial is bits 24:14
        uint32_t channelIndex = (msg.identifier >> 14) & 0x7FF;

        // Diagnostic Print
        if (millis() - last_debug_ms > DEBUG_INTERVAL) {
            Serial.printf("RAW DATA [Idx %d]: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                          channelIndex, msg.data[0], msg.data[1], msg.data[2], msg.data[3], 
                          msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
        }

        switch (channelIndex) {
            case 1:  v_rpm = (int)decodeHolleyFloat(&msg.data[0]); break;   // Engine RPM
            case 2:  v_volt = decodeHolleyFloat(&msg.data[0]); break;      // Battery Voltage
            case 3:  v_mat = decodeHolleyFloat(&msg.data[0]); break;       // Air Temp
            case 4:  v_cts = decodeHolleyFloat(&msg.data[0]); break;       // Coolant Temp
            case 12: v_fuel = decodeHolleyFloat(&msg.data[0]); break;      // Fuel Pressure
            
            case 29: v_cloop = decodeHolleyStatus(&msg.data[0]); break; 
            case 30: v_learn = decodeHolleyStatus(&msg.data[0]); break; 
            case 44: v_fan = decodeHolleyStatus(&msg.data[0]); break;   
        }
    }

    // 3. Serial Debug Summary
    if (millis() - last_debug_ms > DEBUG_INTERVAL) {
        if (millis() - v_can_last_ms > 2000) {
            Serial.println("WARNING: CAN BUS OFFLINE (No data from ECU)");
        } else {
            Serial.printf("LIVE -> RPM: %d | VOLT: %.1f | FUEL: %.1f | TOTAL CAN: %u\n", v_rpm, v_volt, v_fuel, can_msg_count);
        }
        last_debug_ms = millis();
    }

    // 4. Update UI
    if (lvgl_ready && (millis() - last_ui_draw > 50)) {
        if (lvgl_port_lock(20)) {
            // RPM
            if (ui_RPMLabel) lv_label_set_text_fmt(ui_RPMLabel, "%d", (v_rpm < 50) ? 0 : v_rpm);
            
            // Sensors (Using %d.%d to avoid "F" error)
            if (ui_Voltage) {
                int whole = (int)v_volt;
                int dec = (int)((v_volt - (float)whole) * 10); 
                lv_label_set_text_fmt(ui_Voltage, "%d.%d", whole, abs(dec));
            }
            if (ui_MAT) lv_label_set_text_fmt(ui_MAT, "%d", (int)v_mat);
            if (ui_FuelPressure) lv_label_set_text_fmt(ui_FuelPressure, "%d", (int)v_fuel);
            
            // Odometer & Trip
            if (ui_OdometerLabel) lv_label_set_text_fmt(ui_OdometerLabel, "%d", (int)total_odo);
            if (ui_TripLabel) {
                int t_whole = (int)total_trip;
                int t_dec = (int)((total_trip - (double)t_whole) * 10);
                lv_label_set_text_fmt(ui_TripLabel, "%d.%d", t_whole, abs(t_dec));
            }

            // GPS MPH
            if (ui_MPH) {
                if (!gps.location.isValid() || gps.charsProcessed() < 10) lv_label_set_text(ui_MPH, "NS"); 
                else lv_label_set_text_fmt(ui_MPH, "%d", (int)gps.speed.mph());
            }

            // Switches
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

    // 5. Save Data
    if (millis() - last_save_ms > 30000) {
        prefs.putDouble("odo", total_odo);
        prefs.putDouble("trip", total_trip);
        last_save_ms = millis();
    }
    yield();
}
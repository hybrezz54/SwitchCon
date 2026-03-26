//
//  BlueCubeMod Firmware
//
//
//  Created by Nathan Reeves 2019
//
//  Expanded by nullstalgia 2020-2022
//  Expanded by hybrezz54 2026
//
// Main clock 80MHz

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rmt_reg.h"

// GPIO22: external LED output (active high).
// Wire: GPIO22 → 330 Ω resistor → LED anode → LED cathode → GND.
#define LED_GPIO 22
#define PIN_SEL (1ULL << LED_GPIO)

#define PRO_CON 0x03
#define JOYCON_L 0x01
#define JOYCON_R 0x02

#define CONTROLLER_TYPE JOYCON_L

// From least to most significant bits:
// (Right)  Y, X, B, A, SR, SL, R, ZR
static uint8_t but1_send = 0;
// (Shared)  -, +, Rs, Ls, H, Cap, --, Charging Grip
static uint8_t but2_send = 0;
// (Left)  D, U, R, L, SR, SL, L, ZL
static uint8_t but3_send = 0;
static uint8_t lx_send = 128;
static uint8_t ly_send = 128;
static uint8_t cx_send = 128;
static uint8_t cy_send = 128;

SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t spiSemaphore;
bool connected = false;
int paired = 0;
TaskHandle_t SendingHandle = NULL;
TaskHandle_t BlinkHandle = NULL;
// Timer has +1 added to it every send cycle
// Apparently, it can be used to detect packet loss/excess latency
uint8_t timer = 0;

// Switch button report example //         batlvl       Buttons Lstick Rstick
// static uint8_t report30[] = {0x30, 0x00, 0x90,   0x00, 0x00, 0x00,   0x00,
// 0x00, 0x00,   0x00, 0x00, 0x00};
// 80
static uint8_t report30[48] = {[0] = 0x00, [1] = 0x8E, [11] = 0x80};

static uint8_t dummy[11] = {0x00, 0x8E, 0x00, 0x00, 0x00,
                            0x00, 0x08, 0x80, 0x00, 0x08, 0x80};

/**
 * Builds and sends one HID input report to the Switch over Bluetooth.
 *
 * Copies the current button/stick globals into the report30 buffer, then
 * transmits it as a standard 0x30 input report. When paired/connected, reports
 * are sent at ~15 ms intervals (~66 Hz). While not yet paired, a minimal
 * neutral "dummy" report is sent at 100 ms intervals to keep the connection
 * alive.
 *
 * Button globals (set these to control input):
 *   but1_send  -- right buttons: Y(0) X(1) B(2) A(3) SR(4) SL(5) R(6) ZR(7)
 *   but2_send  -- shared buttons: -(0) +(1) Rs(2) Ls(3) Home(4) Cap(5)
 *   but3_send  -- left buttons:   D(0) U(1) R(2) L(3) SR(4) SL(5) L(6) ZL(7)
 *   lx_send / ly_send  -- left stick X/Y  (0-255, center = 128)
 *   cx_send / cy_send  -- right stick X/Y (0-255, center = 128)
 *
 * Must only be called from send_task. If modifying button globals from any
 * other task, take xSemaphore before writing and release it after.
 */
void send_buttons() {
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  report30[0] = timer;
  dummy[0] = timer;
  // buttons
  report30[2] = but1_send;
  report30[3] = but2_send;
  report30[4] = but3_send;
  // encode left stick
  // report30[6] = 0x00;
  // report30[7] = 0x08;
  // report30[8] = 0x80;
  report30[5] = (lx_send << 4) & 0xF0;
  report30[6] = (lx_send & 0xF0) >> 4;
  report30[7] = ly_send;
  // encode right stick
  report30[8] = (cx_send << 4) & 0xF0;
  report30[9] = (cx_send & 0xF0) >> 4;
  report30[10] = cy_send;
  xSemaphoreGive(xSemaphore);
  timer += 1;
  if (timer == 255) timer = 0;

  // Flash LED while any button is held; restore it when all released.
  static uint8_t prev_any_button = 0;
  uint8_t any_button = but1_send | but2_send | but3_send;
  if (any_button != prev_any_button) {
    gpio_set_level(LED_GPIO, any_button ? 0 : 1);
    prev_any_button = any_button;
  }

  if (paired || connected) {
    // If the BT stack is busy (a subcommand reply from intr_data_cb is in
    // flight), skip this frame rather than colliding with it.
    if (esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30,
                                      sizeof(report30), report30) != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(15));
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(15));
  } else {
    if (esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30,
                                      sizeof(dummy), dummy) != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(15));
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/// Switch Replies

// Reply for REQUEST_DEVICE_INFO
static uint8_t reply02[] = {
    //0x21, 
0x00, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,

    0x00, 0x00, 0x00, 0x82, 0x02, 0x04, 0x00,
    CONTROLLER_TYPE,  // Controller type byte.
    // 01 - Left Joycon
    // 02 - Right Joycon
    // 03 - Pro Controller
    0x02, 0xD4, 0xF0, 0x57, 0x6E, 0xF0, 0xD7, 0x01,
#if CONTROLLER_TYPE == PRO_CON
    0x02
#else
    0x01
#endif
    ,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Reply for SET_SHIPMENT_STATE
static uint8_t reply08[] = {
    //0x21, 
0x01, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x0,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Reply for SET_INPUT_REPORT_MODE
static uint8_t reply03[] = {
    //0x21, 
0x04, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Trigger buttons elapsed time
static uint8_t reply04[] = {
    //0x21, 
0x0A, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x83, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2c, 0x01, 0x2c, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Serial number and controller type (although, our code doesn't read (and as
// such, report) the controller type from here.)
static uint8_t spi_reply_address_0[] = {//0x21, 
0x02, 0x8E,
                                        0x00, 0x00, 0x00,
                                        0x00, 0x08, 0x80,
                                        0x00, 0x00, 0x00,
                                        0x00, 0x90, 0x10,
                                        0x00, 0x60, 0x00,
                                        0x00, 0x10, 0xff,
                                        0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff,
                                        0x00, 0x00, CONTROLLER_TYPE,
                                        0xA0, 0x00, 0x00,
                                        0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00};
// The CONTROLLER_TYPE is technically unused, but it makes me feel better.

// SPI Flash colors
static uint8_t spi_reply_address_0x50[] = {
    //0x21, 
0x03, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x90, 0x10, 0x50, 0x60, 0x00, 0x00, 0x0D,  // Start of colors
    0x23, 0x23, 0x23,                                      // Body color
    0xff, 0xff, 0xff,                                      // Buttons color
#if CONTROLLER_TYPE == PRO_CON
    0x95, 0x15, 0x15,  // Left Grip color (Pro Con)
    0x15, 0x15, 0x95,  // Right Grip color (Pro Con)
#else
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
#endif
    0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t spi_reply_address_0x80[] = {
    //0x21, 
0x0B, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x80, 0x60, 0x00, 0x00, 0x18,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t spi_reply_address_0x98[] = {
    //0x21, 
0x0C, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x98, 0x60, 0x00, 0x00, 0x12,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// User analog stick calib
static uint8_t spi_reply_address_0x10[] = {
    //0x21, 
0x0D, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x10, 0x80, 0x00, 0x00, 0x18,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t spi_reply_address_0x3d[] = {
    //0x21, 
0x0E, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x3D, 0x60, 0x00, 0x00, 0x19,
    0x00, 0x07, 0x70, 0x00, 0x08, 0x80, 0x00, 0x07, 0x70, 0x00,
    0x08, 0x80, 0x00, 0x07, 0x70, 0x00, 0x07, 0x70, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00};

static uint8_t spi_reply_address_0x20[] = {
    //0x21, 
0x10, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x20, 0x60, 0x00, 0x00, 0x18,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};

// Reply for changing the status of the IMU IMU (6-Axis sensor)
static uint8_t reply4001[] = {
    //0x21, 
0x15, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t reply4801[] = {
    //0x21, 
0x1A, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Reply for SubCommand.SET_PLAYER_LIGHTS
static uint8_t reply3001[] = {
    //0x21, 
0x1C, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#if CONTROLLER_TYPE == JOYCON_L
// If I had to guess, the Pro Controller equivalent for SET_NFC_IR_MCU_STATE
// Joycontrol calls it SET_NFC_IR_MCU_CONFIG, so maybe its setting the IR sensor
// to OFF?
static uint8_t reply3333[] = {
    //0x21, 
0x03, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01,
    0x18, 0x80, 0x80, 0x80, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//
#elif CONTROLLER_TYPE == JOYCON_R
static uint8_t reply3333[] = {
    //0x21, 
0x31, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x08, 0x80, 0x00, 0xa0, 0x21, 0x01, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7b, 0x00};
#else
static uint8_t reply3333[] = {
    //0x21, 
0x31, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x08, 0x80, 0x00, 0xa0, 0x21, 0x01, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7b, 0x00};
#endif

// Reply for SubCommand.SET_NFC_IR_MCU_STATE
static uint8_t reply3401[] = {
    //0x21, 
0x12, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x80, 0x00, 0x80, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t hid_descriptor[] = {
0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x06, 0x01, 0xff, 0x85, 0x21, 0x09, 0x21, 0x75, 0x08, 0x95, 0x30, 0x81, 0x02, 0x85, 0x30, 0x09, 0x30, 0x75, 0x08, 0x95, 0x30, 0x81, 0x02, 0x85, 0x31, 0x09, 0x31, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02, 0x85, 0x32, 0x09, 0x32, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02, 0x85, 0x33, 0x09, 0x33, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02, 0x85, 0x3f, 0x05, 0x09, 0x19, 0x01, 0x29, 0x10, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x10, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x05, 0x09, 0x75, 0x04, 0x95, 0x01, 0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x33, 0x09, 0x34, 0x16, 0x00, 0x00, 0x27, 0xff, 0xff, 0x00, 0x00, 0x75, 0x10, 0x95, 0x04, 0x81, 0x02, 0x06, 0x01, 0xff, 0x85, 0x01, 0x09, 0x01, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02, 0x85, 0x10, 0x09, 0x10, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02, 0x85, 0x11, 0x09, 0x11, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02, 0x85, 0x12, 0x09, 0x12, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02, 0xc0
};
int hid_descriptor_len = sizeof(hid_descriptor);

/**
 * FreeRTOS task: drives the HID input report loop for the duration of a
 * connection.
 *
 * Spawned by connection_cb when the Switch connects, deleted and re-spawned on
 * each reconnect. Runs pinned to core 0 at priority 2.
 *
 * Startup sequence:
 *   1. Presses SL+SR (Joy-Con) or L+R (Pro Con) for ~500 ms so the Switch's
 *      "Change Grip/Order" screen accepts this controller.
 *   2. Waits ~2 s with neutral reports while the Switch completes its
 *      subcommand handshake (handled in intr_data_cb).
 *   3. Enters the main input loop — replace the TEST block below with real
 *      input logic (GPIO, SPI, etc.) that updates the button/stick globals.
 */
void send_task(void* pvParameters) {
  const char* TAG = "send_task";
  ESP_LOGI(TAG, "Sending hid reports on core %d\n", xPortGetCoreID());

  // Press SL+SR (Joy-Con) or L+R (Pro Con) for ~500 ms so the Switch's
  // "Change Grip/Order" screen recognises and accepts this controller.
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
#if CONTROLLER_TYPE == JOYCON_L
  report30[4] |= (1 << 5) | (1 << 4);  // SL (bit5) + SR (bit4) on left byte
#elif CONTROLLER_TYPE == JOYCON_R
  report30[2] |= (1 << 5) | (1 << 4);  // SL (bit5) + SR (bit4) on right byte
#else  // PRO_CON
  report30[2] |= (1 << 6);  // R shoulder
  report30[4] |= (1 << 6);  // L shoulder
#endif
  xSemaphoreGive(xSemaphore);

  for (int i = 0; i < 33; i++) {  // ~500 ms at 15 ms/frame
    if (esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30,
                                      sizeof(report30), report30) != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(15));
      i--;  // retry this frame
      continue;
    }
    vTaskDelay(pdMS_TO_TICKS(15));
  }

  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  report30[2] &= ~((1 << 6) | (1 << 5) | (1 << 4));
  report30[4] &= ~((1 << 6) | (1 << 5) | (1 << 4));
  xSemaphoreGive(xSemaphore);

  // Wait for intr_data_cb to complete the subcommand handshake (paired=1).
  //
  // On a fresh pair the Switch sends SET_PLAYER_LIGHTS (0x30) which sets
  // paired=1. On a reconnect the Switch skips the handshake — time out after
  // 3 s and proceed anyway.
  //
  // We must keep sending neutral reports during this window so the Switch
  // doesn't time out the connection. Reports are sent every 50 ms (instead of
  // the normal 15 ms) to reduce contention with intr_data_cb reply sends.
  {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    while (!paired && xTaskGetTickCount() < deadline) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30,
                                    sizeof(dummy), dummy);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!paired) {
      paired = 1;  // reconnect — Switch skipped handshake, safe to proceed
    }
  }

  // TEST: repeatedly press A (physical Down on Joy-Con L sideways = bit0 of but3_send).
  // Replace this loop with real input logic once confirmed working.
  while (1) {
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    but3_send |= (1 << 0);   // press A
    xSemaphoreGive(xSemaphore);
    for (int i = 0; i < 20; i++) { send_buttons(); }  // hold ~300 ms

    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    but3_send &= ~(1 << 0);  // release A
    xSemaphoreGive(xSemaphore);
    for (int i = 0; i < 313; i++) { send_buttons(); }  // gap ~4700 ms → ~5 s cycle
  }
}

/**
 * FreeRTOS task: blinks the LED while waiting for a connection.
 * Deleted by connection_cb when the Switch connects.
 */
void startBlink() {
  while (1) {
    gpio_set_level(LED_GPIO, 0);
    vTaskDelay(150);
    gpio_set_level(LED_GPIO, 1);
    vTaskDelay(150);
    gpio_set_level(LED_GPIO, 0);
    vTaskDelay(150);
    gpio_set_level(LED_GPIO, 1);
    vTaskDelay(1000);
  }
  vTaskDelete(NULL);
}
/**
 * HID connection state callback.
 * Called from hidd_event_cb on OPEN_EVT and CLOSE_EVT.
 *
 * On CONNECTED: stops the blink task, sets connected=true, and spawns
 *   send_task to begin sending HID reports.
 * On DISCONNECTED: restarts the blink task, clears paired/connected, and
 *   makes the device connectable+discoverable again.
 */
void connection_cb(esp_bd_addr_t bd_addr, esp_hidd_connection_state_t state) {
  const char* TAG = "connection_cb";

  switch (state) {
    case ESP_HIDD_CONN_STATE_CONNECTED:
      ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", bd_addr[0],
               bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
      ESP_LOGI(TAG, "setting bluetooth non connectable");
      esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

      // clear blinking LED - solid
      vTaskDelete(BlinkHandle);
      BlinkHandle = NULL;
      gpio_set_level(LED_GPIO, 1);
      // start solid
      xSemaphoreTake(xSemaphore, portMAX_DELAY);
      connected = true;
      xSemaphoreGive(xSemaphore);
      // restart send_task
      if (SendingHandle != NULL) {
        vTaskDelete(SendingHandle);
        SendingHandle = NULL;
      }
      xTaskCreatePinnedToCore(send_task, "send_task", 4096, NULL, 2,
                              &SendingHandle, 0);
      break;
    case ESP_HIDD_CONN_STATE_CONNECTING:
      ESP_LOGI(TAG, "connecting");
      break;
    case ESP_HIDD_CONN_STATE_DISCONNECTED:
      xTaskCreate(startBlink, "blink_task", 1024, NULL, 1, &BlinkHandle);
      // start blink
      ESP_LOGI(TAG, "disconnected from %02x:%02x:%02x:%02x:%02x:%02x",
               bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4],
               bd_addr[5]);
      ESP_LOGI(TAG, "making self discoverable");
      paired = 0;
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
      xSemaphoreTake(xSemaphore, portMAX_DELAY);
      connected = false;
      xSemaphoreGive(xSemaphore);
      break;
    case ESP_HIDD_CONN_STATE_DISCONNECTING:
      ESP_LOGI(TAG, "disconnecting");
      break;
    default:
      ESP_LOGI(TAG, "unknown connection status");
      break;
  }
}

// callback for discovering
void get_device_cb() { ESP_LOGI("hi", "found a device"); }

// callback for when hid host requests a report
void get_report_cb(uint8_t type, uint8_t id, uint16_t buffer_size) {
  const char* TAG = "get_report_cb";
  ESP_LOGI(TAG, "got a get_report request from host");
}

// callback for when hid host sends a report
void set_report_cb(uint8_t type, uint8_t id, uint16_t len, uint8_t* p_data) {
  const char* TAG = "set_report_cb";
  ESP_LOGI(TAG, "got a report from host");
}

// callback for when hid host requests a protocol change
void set_protocol_cb(uint8_t protocol) {
  const char* TAG = "set_protocol_cb";
  ESP_LOGI(TAG, "got a set_protocol request from host");
}

/**
 * HID interrupt data callback — handles the Switch's subcommand handshake.
 * Called from hidd_event_cb on INTR_DATA_EVT.
 *
 * The Switch drives pairing by sending a fixed sequence of subcommands
 * (p_data[9] = subcommand ID). Each one is answered with a pre-built reply
 * buffer. When SET_PLAYER_LIGHTS (0x30) or the NFC/IR config (0x21,0x21) is
 * received, paired is set to 1 and send_task begins sending real input reports.
 *
 * Short packets (len < 10) are keepalive/rumble frames with no subcommand and
 * are ignored.
 */
void intr_data_cb(uint8_t report_id, uint16_t len, uint8_t* p_data) {
  const char* TAG = "intr_data_cb";
  // switch pairing sequence
  // Length seems to be quite important here, as the length of [49]
  // Is used for many replies, as well.
  //デバッグ用の受信データ表示 ( GT: Displaying received data for debugging )
  esp_log_buffer_hex(TAG, p_data, len);

  // All subcommand packets are at least 10 bytes (9 header + 1 subcommand ID).
  // Short packets (e.g. simple HID reports after pairing) have no subcommand.
  if (len < 10) return;

  {
    if (p_data[9] == 2) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply02), reply02);
      //これもそう ( GT: This is also the case )
      ESP_LOGI(TAG, "reply02");
    }
    if (p_data[9] == 8) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply08), reply08);
      ESP_LOGI(TAG, "reply08");
    }
    if (p_data[9] == 16 && p_data[10] == 0 && p_data[11] == 96) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0),
                                 spi_reply_address_0);
      ESP_LOGI(TAG, "replyspi0");
    }
    if (p_data[9] == 16 && p_data[10] == 80 && p_data[11] == 96) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x50),
                                 spi_reply_address_0x50);
      ESP_LOGI(TAG, "replyspi50");
    }
    if (p_data[9] == 3) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply03), reply03);
      ESP_LOGI(TAG, "reply03");
    }
    if (p_data[9] == 4) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply04), reply04);
      ESP_LOGI(TAG, "reply04");
    }
    if (p_data[9] == 16 && p_data[10] == 128 && p_data[11] == 96) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x80),
                                 spi_reply_address_0x80);
      ESP_LOGI(TAG, "replyspi80");
    }
    if (p_data[9] == 16 && p_data[10] == 152 && p_data[11] == 96) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x98),
                                 spi_reply_address_0x98);
      ESP_LOGI(TAG, "replyspi98");
    }
    if (p_data[9] == 16 && p_data[10] == 16 && p_data[11] == 128) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x10),
                                 spi_reply_address_0x10);
      ESP_LOGI(TAG, "replyspi10");
    }
    if (p_data[9] == 16 && p_data[10] == 61 && p_data[11] == 96) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x3d),
                                 spi_reply_address_0x3d);
      ESP_LOGI(TAG, "reply3d");
    }
    if (p_data[9] == 16 && p_data[10] == 32 && p_data[11] == 96) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x20),
                                 spi_reply_address_0x20);
      ESP_LOGI(TAG, "replyspi20");
    }
    if (p_data[9] == 64 /*&& p_data[11] == 1*/) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply4001), reply4001);
      ESP_LOGI(TAG, "reply4001");
    }
    if (p_data[9] == 72 /* && p_data[11] == 1*/) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply4801), reply4801);
      ESP_LOGI(TAG, "reply4801");
    }
    if (p_data[9] == 34 /*&& p_data[11] == 1*/) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply3401), reply3401);
      ESP_LOGI(TAG, "reply3401");
    }
    if (p_data[9] == 48 /*&& p_data[11] == 1*/) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply3001), reply3001);
      ESP_LOGI(TAG, "reply3001");
      if (CONTROLLER_TYPE == JOYCON_L) {
        paired = 1;
      }
    }

    if (p_data[9] == 33 && p_data[10] == 33) {
      esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply3333), reply3333);
      ESP_LOGI(TAG, "reply3333");
      paired = 1;
    }

  //ここらへんも必要に応じて ( GT: Here and there as needed )
  //  ESP_LOGI(
  //      TAG,
  //      "got an interrupt report from host, subcommand: %d  %d  %d Length: %d",
  //      p_data[10], p_data[11], p_data[12], len);
  //} else {
    // ESP_LOGI("heap size:", "%d", xPortGetFreeHeapSize());
    // ESP_LOGI(TAG, "pairing packet size != 49, subcommand: %d  %d  %d  Length:
    // %d", p_data[10], p_data[11], p_data[12], len);
  //}
  }  // end len >= 10 block
}

// callback for when hid host does a virtual cable unplug
void vc_unplug_cb(void) {
  const char* TAG = "vc_unplug_cb";
  ESP_LOGI(TAG, "host did a virtual cable unplug");
}

/**
 * Generates or restores a Nintendo-range Bluetooth MAC address (D4:F0:57:XX:XX:XX)
 * and sets it as the base MAC for this device.
 *
 * On first boot the random suffix bytes are generated and persisted to NVS so
 * the Switch always sees the same address on reconnect. To force a new address,
 * uncomment the reset block inside this function.
 */
esp_err_t set_bt_address() {
  // store a random mac address in flash
  nvs_handle my_handle;
  esp_err_t err;
  uint8_t bt_addr[8];

  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) return err;

  size_t addr_size = 0;
  err = nvs_get_blob(my_handle, "mac_addr", NULL, &addr_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

  if (addr_size > 0) {
    err = nvs_get_blob(my_handle, "mac_addr", bt_addr, &addr_size);
  } else {
    //前3桁はNintendo OUI Rangeの物にする ( The first 3 segments should be in Nintendo OUI Range )
    //具体的にはD4:F0:57:XX:XX:XXになる ( For example: D4:F0:57:XX:XX:XX )
    bt_addr[0] = 0xD4;
    bt_addr[1] = 0xF0;
    bt_addr[2] = 0x57;
    for (int i = 3; i < 8; i++) bt_addr[i] = esp_random() % 255;
    addr_size = sizeof(bt_addr);
    err = nvs_set_blob(my_handle, "mac_addr", bt_addr, addr_size);
  }

  //もしアドレスを再設定したいときはコメントアウト外す ( GT: If you want to reset the address, uncomment it )
  //bt_addr[0] = 0xD4;
  //bt_addr[1] = 0xF0;
  //bt_addr[2] = 0x57;
  //for (int i = 3; i < 8; i++) bt_addr[i] = esp_random() % 255;
  //addr_size = sizeof(bt_addr);
  //err = nvs_set_blob(my_handle, "mac_addr", bt_addr, addr_size);

  err = nvs_commit(my_handle);
  nvs_close(my_handle);
  esp_base_mac_addr_set(bt_addr);
  return ESP_OK;
}

void print_bt_address() {
  const char* TAG = "bt_address";
  const uint8_t* bd_addr;

  bd_addr = esp_bt_dev_get_address();
  ESP_LOGI(TAG, "my bluetooth address is %02X:%02X:%02X:%02X:%02X:%02X",
           bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4],
           bd_addr[5]);
}

static esp_hidd_app_param_t app_param;
static esp_hidd_qos_param_t both_qos;

/**
 * Unified HID device event callback registered with esp_bt_hid_device_register_callback.
 *
 * Dispatches BT HID stack events to the appropriate handler:
 *   INIT_EVT          -- BT HID stack ready; registers the HID app profile.
 *   REGISTER_APP_EVT  -- HID profile registered; makes device discoverable.
 *   OPEN_EVT          -- Switch opened a HID connection.
 *   CLOSE_EVT         -- HID connection closed.
 *   INTR_DATA_EVT     -- Switch sent a subcommand or input report.
 *   SEND_REPORT_EVT   -- Acknowledgement after each send_report call (ignored).
 */
static void hidd_event_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
  const char* TAG = "hidd_event_cb";
  switch (event) {
    case ESP_HIDD_INIT_EVT:
      if (param->init.status == ESP_HIDD_SUCCESS) {
        ESP_LOGI(TAG, "INIT_EVT: init ok, registering app");
        esp_bt_hid_device_register_app(&app_param, &both_qos, &both_qos);
      } else {
        ESP_LOGE(TAG, "INIT_EVT: init failed, status=%d", param->init.status);
      }
      break;
    case ESP_HIDD_REGISTER_APP_EVT:
      if (param->register_app.status == ESP_HIDD_SUCCESS) {
        ESP_LOGI(TAG, "REGISTER_APP_EVT: ok, going connectable+discoverable");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        xTaskCreate(startBlink, "blink_task", 1024, NULL, 2, &BlinkHandle);
      } else {
        ESP_LOGE(TAG, "REGISTER_APP_EVT: failed, status=%d", param->register_app.status);
      }
      break;
    case ESP_HIDD_OPEN_EVT:
      ESP_LOGI(TAG, "OPEN_EVT: conn_status=%d", param->open.conn_status);
      connection_cb(param->open.bd_addr, param->open.conn_status);
      break;
    case ESP_HIDD_CLOSE_EVT: {
      ESP_LOGI(TAG, "CLOSE_EVT: conn_status=%d", param->close.conn_status);
      static esp_bd_addr_t zero_addr = {0};
      connection_cb(zero_addr, param->close.conn_status);
      break;
    }
    case ESP_HIDD_GET_REPORT_EVT:
      get_report_cb(param->get_report.report_type, param->get_report.report_id, param->get_report.buffer_size);
      break;
    case ESP_HIDD_SET_REPORT_EVT:
      set_report_cb(param->set_report.report_type, param->set_report.report_id, param->set_report.len, param->set_report.data);
      break;
    case ESP_HIDD_SET_PROTOCOL_EVT:
      set_protocol_cb(param->set_protocol.protocol_mode);
      break;
    case ESP_HIDD_INTR_DATA_EVT:
      intr_data_cb(param->intr_data.report_id, param->intr_data.len, param->intr_data.data);
      break;
    case ESP_HIDD_SEND_REPORT_EVT:
      // fires after every esp_bt_hid_device_send_report — not an error, ignore
      break;
    case ESP_HIDD_VC_UNPLUG_EVT:
      vc_unplug_cb();
      break;
    default:
      ESP_LOGI(TAG, "unhandled event: %d", event);
      break;
  }
}

#define SPP_TAG "tag"
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event,
                          esp_bt_gap_cb_param_t* param) {
  switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
      ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
      esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
      break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
      ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
      break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
      ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
      ESP_LOGI(SPP_TAG, "%d", param->rmt_srvcs.num_uuids);
      break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
      ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
      break;
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
      if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
        ESP_LOGI(SPP_TAG, "authentication success: %s",
                 param->auth_cmpl.device_name);
        esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
      } else {
        ESP_LOGE(SPP_TAG, "authentication failed, status:%d",
                 param->auth_cmpl.stat);
      }
      break;
    }

    default:
      break;
  }
}
void app_main() {
  // SNES Contoller reading init
  // spi_init();
  // xTaskCreatePinnedToCore(get_buttons, "gbuttons", 2048, NULL, 1,
  //                         &ButtonsHandle, 1);
  const char* TAG = "app_main";
  ESP_LOGI(TAG, "checkpoint 1: app_main entered");

  if (CONTROLLER_TYPE != PRO_CON) {
    //report30[2] += (0x3 << 1);
    //dummy[2] += (0x3 << 1);
  }

  esp_err_t ret;
  static esp_bt_cod_t class;

  xSemaphore = xSemaphoreCreateMutex();

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = PIN_SEL;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);
  ESP_LOGI(TAG, "checkpoint 3: gpio configured");

  // Startup flash: 3 quick blinks to confirm external LED is wired correctly.
  for (int i = 0; i < 3; i++) {
    gpio_set_level(LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  //一応名前とプロバイダーを純正と一緒にする ( For now, set these the same as a genuine product )
  app_param.name = "Wireless Gamepad";
  app_param.description = "Gamepad";
  app_param.provider = "Nintendo";
  // app_param.subclass = 0x002508;
  app_param.subclass = 0x8;
  app_param.desc_list = hid_descriptor;
  app_param.desc_list_len = hid_descriptor_len;
  memset(&both_qos, 0, sizeof(esp_hidd_qos_param_t));

  ESP_LOGI(TAG, "checkpoint 4: hid params set");

  class.minor = 2;
  class.major = 5;
  class.service = 1;

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_LOGI(TAG, "checkpoint 5: nvs flash init ok");

  ESP_ERROR_CHECK(set_bt_address());
  ESP_LOGI(TAG, "checkpoint 6: bt address set");

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_mem_release(ESP_BT_MODE_BLE);
  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
    ESP_LOGE(TAG, "initialize controller failed: %s\n", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "checkpoint 7: bt controller init ok");

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
    ESP_LOGE(TAG, "enable controller failed: %s\n", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "checkpoint 8: bt controller enabled");

  if ((ret = esp_bluedroid_init()) != ESP_OK) {
    ESP_LOGE(TAG, "initialize bluedroid failed: %s\n", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "checkpoint 9: bluedroid init ok");

  if ((ret = esp_bluedroid_enable()) != ESP_OK) {
    ESP_LOGE(TAG, "enable bluedroid failed: %s\n", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "checkpoint 10: bluedroid enabled");

  // Required for HID device: no display/input, so use "Just Works" pairing
  esp_bt_sp_param_t sp_param = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
  esp_bt_gap_set_security_param(sp_param, &iocap, sizeof(esp_bt_io_cap_t));

  esp_bt_gap_register_callback(esp_bt_gap_cb);
  ESP_LOGI(TAG, "checkpoint 11: gap callback registered");

  esp_bt_hid_device_register_callback(hidd_event_cb);
  ESP_LOGI(TAG, "checkpoint 12: hid callback registered");

  esp_bt_hid_device_init();
  ESP_LOGI(TAG, "checkpoint 13: hid device init called (register_app called from INIT_EVT)");

  if (CONTROLLER_TYPE == JOYCON_L)
    esp_bt_dev_set_device_name("Joy-Con (L)");
  else if (CONTROLLER_TYPE == JOYCON_R)
    esp_bt_dev_set_device_name("Joy-Con (R)");
  else
    esp_bt_dev_set_device_name("Pro Controller");
  ESP_LOGI(TAG, "checkpoint 14: device name set");

  esp_bt_gap_set_cod(class, ESP_BT_SET_COD_ALL);
  ESP_LOGI(TAG, "checkpoint 15: device class set");

  ESP_LOGI(TAG, "checkpoint 16: app_main done, waiting for HID REGISTER_APP_EVT to go discoverable");

  // Keep main_task alive; all work happens in BT callbacks and spawned tasks
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}
//

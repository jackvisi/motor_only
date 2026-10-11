#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_err.h"
#include "mqtt_client.h"
#include "driver/twai.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_crt_bundle.h"

#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <inttypes.h>
#include "gvi_motor.h"   
#include "batteries.h"

//  lvrd
#include "ad7190.h"
#include "components/drivers/cooling.h"
//adc defines 
#define AD7190_CLK_PIN 35
#define AD7190_CS_PIN 36
#define AD7190_MISO_PIN 37
#define AD7190_MOSI_PIN 38

// #define AD7190_CLK_PIN 4
// #define AD7190_CS_PIN 5
// #define AD7190_MISO_PIN 6
// #define AD7190_MOSI_PIN 7

#define AD7190_RDY_GPIO   AD7190_MISO_PIN
#define AD7190_ACQ_MODE_ISR_SPI      1
#define AD7190_ACQ_MODE_TASK_NOTIFY  2

#define AD7190_ACQ_MODE AD7190_ACQ_MODE_TASK_NOTIFY

#include "esp_random.h"
#include <limits.h>   // for UINT32_MAX
#include <math.h>

#include "cangps.h" //gps frame handlers 

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

//#include "batteries_gps_motor_2.h"  // FF DTU Boat CAN Library
//#include "nma2000_motor.h"
//#include "gvi_motor.h"

//library for dht11 sensor, found on GitHub:https://github.com/abdellah2288/esp32-dht11/tree/main 
//#include "esp32-dht11.h"

// wifi configuration
//#define WIFI_SSID "DTU_FF_Boat_Team"
//#define WIFI_PASS "DTUBoatFF2425#"

//#define WIFI_SSID "iPhone di Giacomo"
//#define WIFI_PASS "labarca123"

#define RX_TOPIC_MAX   256
#define RX_PAYLOAD_MAX 256

//static esp_mqtt_client_handle_t mqtt_client;
//static volatile bool mqtt_conn = false;

// Pins for CAN Telemetry PCB:
#define CAN_TX GPIO_NUM_47
#define CAN_RX GPIO_NUM_48

// motor macros
#define GVI_NODE_ID          6
#define PDO_TX_PERIOD_MS     10
#define HEARTBEAT_TIMEOUT_MS 60000
#define ACCEL_RAMP           5
#define DECEL_RAMP            5
#define ACCEL_RAMP_R          2
#define DECEL_RAMP_R          2
#define MAX_SETPOINT_REVERSE      1500

#define ENABLE_BTN_PIN       GPIO_NUM_11
#define REVERSE_BTN_PIN       GPIO_NUM_10
#define BTN_POLL_MS          30
#define VOLT_MIN     900

// Bits CommandAll
#define CMD_SWITCH_ON   (1U << 0)
#define CMD_ENABLE      (1U << 3)
#define CMD_CONTACTOR   (1U << 10) 
#define REVERSE_INFO    (1U << 4)//00010000000 / 000010000
#define ID_REVERSE   (0x112u)

// Bit StatusAll
#define ST_READY         (1U << 0)
#define ST_SWITCHED_ON   (1U << 1)
#define ST_ENABLED       (1U << 2)
#define ST_TRIPPED       (1U << 3)
#define ST_ERROR_ACTIVE  (1U << 6)
#define ST_WARNING_ACTIVE (1U << 7)

#define MOTOR_DEFAULT_RPM 50   // target speed in cruise control

//motor state struct:
typedef struct {
    // commands: written by buttons task and sent by pdo task
    int16_t  target_rpm;
    uint16_t command_word;
    bool     enabled;

    // feedback GVI (written by can_rx_task, read by mqtt_task)
    uint16_t status_all;
    int16_t  rotor_speed_rpm;
    uint16_t rms_motor_current;
    int16_t  filtered_voltage;
    int16_t  act_torque;
    int16_t  dc_bus_current;
    int16_t  motor_temp;
    int16_t  inverter_temp;
    uint8_t  actual_control_mode;
    uint8_t  actual_limitation_type;
    uint8_t  nmt_state;
    int64_t  last_update_us;
    int64_t  last_heartbeat_us;
    bool reverse;
    bool     cruise_active;      
    int16_t  cruise_rpm;   
    int16_t  cruise_setpoint;  
    int16_t  setpoint;   

    // EMCY
    uint16_t last_emcy_code;
    uint16_t last_emcy_event_id;
    uint8_t  last_emcy_reaction;
    bool     emcy_pending;
} motor_state_t;

static motor_state_t g_motor = { .target_rpm = 0, .command_word = 0, .enabled = false, .reverse = false, .cruise_active = false, .filtered_voltage = 0 };  // first initialization
static SemaphoreHandle_t g_state_mutex;        //mutex

// -----------------------------------   ADC/lvr --------------------------
static const char *TAG = "ad7190_app";
static TaskHandle_t s_ad7190_task_handle = NULL;
static volatile uint32_t s_last_sample = 0;
static volatile bool s_sample_ready = false;


static void IRAM_ATTR ad7190_rdy_isr_handler(void *arg)
{
#if AD7190_ACQ_MODE == AD7190_ACQ_MODE_ISR_SPI

    uint32_t sample = 0;

    gpio_intr_disable(AD7190_RDY_GPIO);

    if (ad7190_conversion_read(&sample) == ESP_OK) {
        s_last_sample = sample;
        s_sample_ready = true;
    }

    gpio_intr_enable(AD7190_RDY_GPIO);

#elif AD7190_ACQ_MODE == AD7190_ACQ_MODE_TASK_NOTIFY

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (s_ad7190_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_ad7190_task_handle, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }

#endif
}

// 100 setpoints 45rpm resolution
// setpoint = sample * max_setpoint/max_sample 

volatile int16_t max_setpoint = 4500;  //max rpm
int16_t setpoint = 0;

static esp_err_t ad7190_rdy_gpio_init(void)
{
    esp_err_t err;

    // Important:
    // AD7190_RDY_GPIO is the same physical pin as SPI MISO/DOUT.
    // Do NOT call gpio_config() here, because it may overwrite the SPI pin config.
    // Just ensure input is enabled and configure the interrupt.
    err = gpio_input_enable(AD7190_RDY_GPIO);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_set_pull_mode(AD7190_RDY_GPIO, GPIO_FLOATING);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_set_intr_type(AD7190_RDY_GPIO, GPIO_INTR_NEGEDGE);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = gpio_isr_handler_add(AD7190_RDY_GPIO, ad7190_rdy_isr_handler, NULL);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

#define LEVER_MAX_SAMPLE 15170000
#define LEVER_MIN_SAMPLE 1748500
#define LEVER_TOLERANCE_PCT 10

typedef enum {
        SAMPLES_OK = 0,
        CHANNEL_MISMATCH,      // channels differ, false if same
        SAMPLE1_OUT_OF_RANGE,  // sample1 is out of expected range
        SAMPLE2_OUT_OF_RANGE,  // sample2 is out of expected range
        SAMPLES_DIFFER         // samples differ by more than tolerance
} lever_sample_error_t;

lever_sample_error_t lever_redundancy_check(uint32_t sample1, uint32_t sample2)
{
    /**
     *  Check lever redundancy: The two samples should be within a certain tolerance. If they differ too much, it may indicate a sensor fault.
     *  For example, if the samples differ by more than 10% of the full
     * returns the error code indicating the status of the samples.
     * */

    // Check channel:
    if ((sample1 & 0xFF0000) == (sample2 & 0xFF0000)) {
        ESP_LOGW(TAG, "Lever redundancy check failed: same channel");
        return CHANNEL_MISMATCH; // Same channels
    }

    // Check if any are out of range
    if (sample1 < LEVER_MIN_SAMPLE || sample1 > LEVER_MAX_SAMPLE) {
        ESP_LOGW(TAG, "Lever redundancy check failed: sample1 out of range");
        return SAMPLE1_OUT_OF_RANGE; // Sample out of range
    }

    // Check if they differ by more than the tolerance
    if (sample2 < LEVER_MIN_SAMPLE || sample2 > LEVER_MAX_SAMPLE) {
        ESP_LOGW(TAG, "Lever redundancy check failed: sample2 out of range");
        return SAMPLE2_OUT_OF_RANGE; // Sample out of range
    }

    if (sample1 > sample2) {
        if (sample1 - sample2 > (LEVER_TOLERANCE_PCT / 100.0f) * LEVER_MAX_SAMPLE) {
            ESP_LOGW(TAG, "Lever redundancy check failed: samples differ by more than tolerance");
            return SAMPLES_DIFFER; // Samples differ by more than tolerance
        }
    } else {
        if (sample2 - sample1 > (LEVER_TOLERANCE_PCT / 100.0f) * LEVER_MAX_SAMPLE) {
            ESP_LOGW(TAG, "Lever redundancy check failed: samples differ by more than tolerance");
            return SAMPLES_DIFFER; // Samples differ by more than tolerance
        }
    }
    return SAMPLES_OK; // Samples are OK
}

int16_t lever_sample_to_setpoint(uint32_t sample)
{
    /**
     *  Convert a raw lever sample to a setpoint value. This function assumes the sample is valid and within range.
     *  The conversion is linear, mapping the full sample range to the setpoint range.
     * */

    float percent = ((sample & 0xFFFFFF) - LEVER_MIN_SAMPLE) / (float)(LEVER_MAX_SAMPLE - LEVER_MIN_SAMPLE);
    int16_t setpoint1 = percent * max_setpoint;
    if (setpoint1 < 25){
        setpoint1 = 0;
    }

    return setpoint1;
}
static void ad7190_task_lever(void *pvParameters)
{
    ESP_LOGI(TAG, "AD7190 task started");
    esp_err_t err;

    
    ad7190_mode_cfg_t mode_cfg = {
        .mode = AD7190_MODE_CONT,
        .dat_sta = false,
        .clock = AD7190_CLK_INT,
        .sinc3 = false,
        .enpar = false,
        .single_cycle = false,
        .rej60 = false,
        .fs = 350,
    };

    // LEVER CONFIG
    ad7190_config_cfg_t lev_config_cfg = {
        .chop = false,
        .refsel = false, 
        .channel_mask = AD7190_CH_AIN4_AINCOM,
        .burn = false,
        .refdet = false,
        .buffer = false,
        .unipolar = true,
        .gain = AD7190_PGA_GAIN_1,
    };

    ad7190_gpocon_cfg_t gpocon_cfg = {
        .bpdsw = false,
        .gp32en = false,
        .gp10en = false,
        .p3dat = false,
        .p2dat = false,
        .p1dat = false,
        .p0dat = false,
    };

    // INITIALIZE AD7190
    err = ad7190_init(
        AD7190_CLK_PIN,
        AD7190_CS_PIN,
        AD7190_MISO_PIN,
        AD7190_MOSI_PIN,
        &mode_cfg,
        &lev_config_cfg,
        &gpocon_cfg
    );
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AD7190 init failed: %s", esp_err_to_name(err));
        return;
    }

    err = ad7190_rdy_gpio_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RDY GPIO init failed: %s", esp_err_to_name(err));
        return;
    }


    // START CONVERSION
    ad7190_continuous_read_start();

    while (1) {

#if AD7190_ACQ_MODE == AD7190_ACQ_MODE_ISR_SPI

        if (s_sample_ready) {
            uint32_t sample = s_last_sample;
            s_sample_ready = false;

            ESP_LOGI(TAG, "Sample: %" PRIu32, sample);
        }

        vTaskDelay(pdMS_TO_TICKS(1));

#elif AD7190_ACQ_MODE == AD7190_ACQ_MODE_TASK_NOTIFY

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t sample = 0;

        gpio_intr_disable(AD7190_RDY_GPIO);
        err = ad7190_conversion_read(&sample);
        gpio_intr_enable(AD7190_RDY_GPIO);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read AD7190 sample: %s", esp_err_to_name(err));
            continue;
        }
        
        // ESP_LOGI(TAG, "Sample: %x", sample);
        ESP_LOGI(TAG, "Sample: %" PRIu32, sample);

        int16_t setpoint = lever_sample_to_setpoint(sample);
        ESP_LOGI(TAG, "Setpoint: %d", setpoint);
        // dentro ad7190_task_lever, dopo aver calcolato 'setpoint' dalla leva (sempre ≥0):

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        bool    cruise = g_motor.cruise_active;
        int16_t crpm   = g_motor.cruise_rpm;
        bool    reverse = g_motor.reverse;
        int16_t cruise_setpoint = g_motor.cruise_setpoint;
        xSemaphoreGive(g_state_mutex);

        // override: se in cruise e la leva supera una soglia, esci dal cruise
        #define LEVER_OVERRIDE_RPM 20   // more than 50 rpm of 

        if (cruise &&  (setpoint > cruise_setpoint + LEVER_OVERRIDE_RPM || setpoint < cruise_setpoint - LEVER_OVERRIDE_RPM)) {
            // la leva è stata mossa: abbandona il cruise
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_motor.cruise_active = false;
            xSemaphoreGive(g_state_mutex);
            cruise = false;
            ESP_LOGI(TAG, "Cruise overridden by lever");
        }

        // decidi il setpoint finale
        /*
        int16_t final_rpm;
        if (cruise) {
            final_rpm = crpm;                          // cruise comanda
        } else {
            final_rpm = reverse ? -setpoint : setpoint; // leva normale (col segno)
        }
        */
        int16_t final_rpm;
        if (cruise) {
            final_rpm = crpm;
        } else if (reverse) {
            final_rpm = -(int16_t)((int32_t)setpoint * MAX_SETPOINT_REVERSE / max_setpoint);
        } else {
            final_rpm = setpoint;
        }
        ESP_LOGI(TAG, "Final RPM: %d", final_rpm);

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_motor.target_rpm = final_rpm;
        g_motor.setpoint = setpoint;
        xSemaphoreGive(g_state_mutex);
        // updating the target rpm to send the motor 
        //int16_t setpoint = lever_sample_to_setpoint(sample);

        //xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        //g_motor.target_rpm = g_motor.reverse ? -setpoint : setpoint;
        //xSemaphoreGive(g_state_mutex);
#endif
    }
}
typedef struct {
    uint8_t  cruise_on;     // 1 = attiva cruise, 0 = disattiva
    int16_t  cruise_rpm;    // rpm fissi (signed)
} __attribute__((packed)) cruise_cmd_t;

void can_receiver_task(void *pvParameters){
    while (1){
        twai_message_t rxMessage;
        while (twai_receive(&rxMessage, pdMS_TO_TICKS(10)) == ESP_OK) {
            //ESP_LOGI("CAN", "Received CAN frame - ID: 0x%03X, Length: %d", rxMessage.identifier, rxMessage.data_length_code);
            //char buf[64];
            //int pos = 0;
            //for (int i = 0; i < rxMessage.data_length_code && pos < (int)sizeof(buf)-3; ++i) {
            //    pos += snprintf(&buf[pos], sizeof(buf)-pos, "%02X ", rxMessage.data[i]);
            //}

            switch (rxMessage.identifier) {
                case GVI_MOTOR_GVI_STATUS_TX_PDO1_FRAME_ID: {  // 0x186
                    struct gvi_motor_gvi_status_tx_pdo1_t m;
                    if (gvi_motor_gvi_status_tx_pdo1_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0) {
                        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                        g_motor.status_all        = m.status_all;
                        g_motor.rotor_speed_rpm   = m.rotor_speed_in_rpm;
                        g_motor.rms_motor_current = m.rms_motor_current;
                        g_motor.filtered_voltage  = m.signal_f_iltered_voltage;
                        g_motor.last_update_us    = esp_timer_get_time();
                        xSemaphoreGive(g_state_mutex);
                    }
                    break;
                }
                case GVI_MOTOR_DIAGNOSTIC1_TX_PDO2_FRAME_ID: {  // 0x286
                    struct gvi_motor_diagnostic1_tx_pdo2_t m;
                    if (gvi_motor_diagnostic1_tx_pdo2_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0) {
                        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                        g_motor.act_torque     = m.signal_act_torque;
                        g_motor.dc_bus_current = m.dc_bus_current;
                        xSemaphoreGive(g_state_mutex);
                    }
                    break;
                }
                case GVI_MOTOR_DIAGNOSTIC2_TX_PDO3_FRAME_ID: {  // 0x386
                    struct gvi_motor_diagnostic2_tx_pdo3_t m;
                    if (gvi_motor_diagnostic2_tx_pdo3_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0) {
                        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                        g_motor.actual_limitation_type = m.actual_limitation_type;
                        xSemaphoreGive(g_state_mutex);
                    }
                    break;
                }
                case GVI_MOTOR_DIAGNOSTIC3_TX_PDO4_FRAME_ID: {  // 0x486
                    struct gvi_motor_diagnostic3_tx_pdo4_t m;
                    if (gvi_motor_diagnostic3_tx_pdo4_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0) {
                        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                        g_motor.motor_temp          = m.motor_temp;
                        g_motor.inverter_temp       = m.inverter_temp;
                        g_motor.actual_control_mode = m.actual_control_mode;
                        xSemaphoreGive(g_state_mutex);
                    }
                    break;
                }
                case GVI_MOTOR_HEARTBEAT_FRAME_ID: {  // 0x706
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_motor.nmt_state         = rxMessage.data[0];
                    g_motor.last_heartbeat_us = esp_timer_get_time();
                    xSemaphoreGive(g_state_mutex);
                    ESP_LOGI("HB", "HB[%02X]",rxMessage.data);

                    break;
                }
                case GVI_MOTOR_EMCY_FRAME_ID: {  // 0x086
                    struct gvi_motor_emcy_t m;
                    if (gvi_motor_emcy_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0) {
                        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                        g_motor.last_emcy_code     = m.emercy_error_code;
                        g_motor.last_emcy_event_id = m.event_id;
                        g_motor.last_emcy_reaction = m.system_reaction;
                        g_motor.emcy_pending = true;
                        xSemaphoreGive(g_state_mutex);
                        //ESP_LOGW("GVI", "EMCY code=0x%04X eventID=0x%04X", m.emercy_error_code, m.event_id);
                    }
                    break;
                }
                /*case 0x300: {
                    if (rxMessage.data_length_code >= sizeof(cruise_cmd_t)) {
                        cruise_cmd_t c;
                        memcpy(&c, rxMessage.data, sizeof(c));
                        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                        g_motor.cruise_active     = c.cruise_on;
                        g_motor.cruise_rpm        = c.cruise_rpm;
                        g_motor.cruise_setpoint = g_motor.setpoint;
                        xSemaphoreGive(g_state_mutex);
                    }
                    break;
                }*/
                default:
                    //ESP_LOGI("CAN","Unhandled CAN ID: 0x%03X\n", rxMessage.identifier);
                    break;
            }

        }
    }
}
static esp_err_t send_pdo(uint32_t id, const uint8_t *data, uint8_t dlc) {
    twai_message_t frame = { .identifier = id, .extd = 0, .data_length_code = dlc };
    memcpy(frame.data, data, dlc);
    return twai_transmit(&frame, pdMS_TO_TICKS(5));
}

void pdo_tx_task(void *arg) {
    uint8_t buf[8];
    TickType_t last = xTaskGetTickCount();
    while (1) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        uint16_t cmd = g_motor.command_word;
        int16_t  rpm = g_motor.target_rpm;
        bool     en  = g_motor.enabled; 
        bool rev = g_motor.reverse;
        int16_t  volt = g_motor.filtered_voltage;
        xSemaphoreGive(g_state_mutex);
        if (en) {
            if (rev){
                struct gvi_motor_commands1_rx_pdo1_t c1 = {
                    .commands_all = cmd, .command_speed = rpm,
                    .command_acceleration_change = ACCEL_RAMP_R,
                    .command_deceleration_change = DECEL_RAMP_R,
                };
                memset(buf, 0, 8); gvi_motor_commands1_rx_pdo1_pack(buf, &c1, 8);
                send_pdo(GVI_MOTOR_COMMANDS1_RX_PDO1_FRAME_ID, buf, GVI_MOTOR_COMMANDS1_RX_PDO1_LENGTH);

                struct gvi_motor_commands1_rx_pdo1_t reverse = {
                    .commands_all = REVERSE_INFO, .command_speed = 0,
                    .command_acceleration_change = 0,
                    .command_deceleration_change = 0,
                };
                memset(buf, 0, 8); gvi_motor_commands1_rx_pdo1_pack(buf, &reverse, 8);
                send_pdo(ID_REVERSE, buf, GVI_MOTOR_COMMANDS1_RX_PDO1_LENGTH);

            } else{
                struct gvi_motor_commands1_rx_pdo1_t c1 = {
                    .commands_all = cmd, .command_speed = rpm,
                    .command_acceleration_change = ACCEL_RAMP,
                    .command_deceleration_change = DECEL_RAMP,
                };
                memset(buf, 0, 8); gvi_motor_commands1_rx_pdo1_pack(buf, &c1, 8);
                send_pdo(GVI_MOTOR_COMMANDS1_RX_PDO1_FRAME_ID, buf, GVI_MOTOR_COMMANDS1_RX_PDO1_LENGTH);

                struct gvi_motor_commands1_rx_pdo1_t reverse = {
                    .commands_all = 0, .command_speed = 0,
                    .command_acceleration_change = 0,
                    .command_deceleration_change = 0,
                };
                memset(buf, 0, 8); gvi_motor_commands1_rx_pdo1_pack(buf, &reverse, 8);
                send_pdo(ID_REVERSE, buf, GVI_MOTOR_COMMANDS1_RX_PDO1_LENGTH);
            }
            
            struct gvi_motor_commands2_rx_pdo2_t c2 = { .requested_control_mode = 0 };
            memset(buf, 0, 8); gvi_motor_commands2_rx_pdo2_pack(buf, &c2, 8);
            send_pdo(GVI_MOTOR_COMMANDS2_RX_PDO2_FRAME_ID, buf, GVI_MOTOR_COMMANDS2_RX_PDO2_LENGTH);

            int16_t volt_safe = volt;
            if (volt_safe < VOLT_MIN) volt_safe = VOLT_MIN;   // evita zero e negativi
            double volt_real = (double)volt_safe * 0.1;
            double current_limit = 24000.0 / (double)volt_real;

            if (current_limit > 400) {current_limit = 400.0;}
            //ESP_LOGI("PWRLIM", "Current Limit: %.3f",current_limit);
            struct gvi_motor_limits_rx_pdo3_t lim = {
                .acc_torque_limit = gvi_motor_limits_rx_pdo3_acc_torque_limit_encode(70), .brake_torque_limit = gvi_motor_limits_rx_pdo3_brake_torque_limit_encode(70),
                .pos_dc_current_limit = gvi_motor_limits_rx_pdo3_pos_dc_current_limit_encode(current_limit), .neg_dc_current_limit = gvi_motor_limits_rx_pdo3_neg_dc_current_limit_encode(current_limit),
            };
            //ESP_LOGI("PWRLIM", "Current Limit after encode: %d",gvi_motor_limits_rx_pdo3_pos_dc_current_limit_encode(current_limit));
            memset(buf, 0, 8); gvi_motor_limits_rx_pdo3_pack(buf, &lim, 8);
            send_pdo(GVI_MOTOR_LIMITS_RX_PDO3_FRAME_ID, buf, GVI_MOTOR_LIMITS_RX_PDO3_LENGTH);


        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PDO_TX_PERIOD_MS));
    }
}

//--------------- buttons task -------------------
//functions for sending the NMT
static esp_err_t send_nmt(uint8_t command) {
    struct gvi_motor_nmt_t nmt = { .nmt_command = command, .node_id = GVI_NODE_ID };
    uint8_t buf[GVI_MOTOR_NMT_LENGTH];
    if (gvi_motor_nmt_pack(buf, &nmt, sizeof(buf)) < 0) return ESP_FAIL;
    twai_message_t f = { .identifier = GVI_MOTOR_NMT_FRAME_ID, .extd = 0, .data_length_code = 2 };
    memcpy(f.data, buf, 2);
    return twai_transmit(&f, pdMS_TO_TICKS(100));
}
#define send_nmt_start()  send_nmt(GVI_MOTOR_NMT_NMT_COMMAND_GO_TO__OPERATIONAL_CHOICE)

#define send_nmt_preop()  send_nmt(GVI_MOTOR_NMT_NMT_COMMAND_GO_TO__PRE_OPERATIONAL_CHOICE)

// task
void enable_button_task(void *arg) {
    bool en_prev  = true;   // pull-up: idle = HIGH
    bool rev_prev = true;
    while (1) {
        bool en_now  = gpio_get_level(ENABLE_BTN_PIN);
        bool rev_now = gpio_get_level(REVERSE_BTN_PIN);

        // ---------- ENABLE button ----------
        if (en_prev == true && en_now == false) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            bool new_state = !g_motor.enabled;
            xSemaphoreGive(g_state_mutex);

            if (new_state) {
                ESP_LOGI("GVI", "Waiting for lever to zero before enabling...");
                while (1) {
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    int16_t lever = g_motor.target_rpm;
                    xSemaphoreGive(g_state_mutex);
                    if (lever == 0) break;
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                send_nmt_start();
                vTaskDelay(pdMS_TO_TICKS(200));
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                //g_motor.command_word = CMD_SWITCH_ON | CMD_ENABLE | CMD_CONTACTOR;
                g_motor.command_word = CMD_SWITCH_ON;
                g_motor.enabled      = true;
                xSemaphoreGive(g_state_mutex);
                vTaskDelay(pdMS_TO_TICKS(200));
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_motor.command_word = CMD_SWITCH_ON | CMD_ENABLE | CMD_CONTACTOR;
                //g_motor.command_word = CMD_SWITCH_ON;
                g_motor.enabled      = true;
                xSemaphoreGive(g_state_mutex);
                ESP_LOGI("GVI", "Motor ENABLED");
            } else {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_motor.target_rpm   = 0;
                g_motor.command_word = 0;
                xSemaphoreGive(g_state_mutex);
                vTaskDelay(pdMS_TO_TICKS(50));
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_motor.enabled = false;
                xSemaphoreGive(g_state_mutex);
                send_nmt_preop();
                ESP_LOGI("GVI", "Motor DISABLED");
            }
        }

        // ---------- REVERSE button ----------
        if (rev_prev == true && rev_now == false) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            int16_t lever = g_motor.target_rpm;     // current value
            if (lever == 0) {
                g_motor.reverse = !g_motor.reverse;  // reverse
                ESP_LOGI("GVI", "Direction: %s", g_motor.reverse ? "REVERSE" : "FORWARD");
            } else {
                ESP_LOGW("GVI", "Lever not zero, reverse ignored");
            }
            xSemaphoreGive(g_state_mutex);
            // no waiting time, if the lever is not zero --> return
        }

        en_prev  = en_now;
        rev_prev = rev_now;
        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
    }
}



void app_main(void)
{

    g_state_mutex = xSemaphoreCreateMutex();
    //b_state_mutex = xSemaphoreCreateMutex();

    //wifi_init_sta();
    //mqtt_app_start();
    //cangps_init();

    // TWAI: USA I PIN REALI del tuo cablaggio, code grandi per i PDO a 10ms
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    g.tx_queue_len = 20;
    g.rx_queue_len = 30;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());

    // GPIO pulsante enable
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << ENABLE_BTN_PIN) | (1ULL << REVERSE_BTN_PIN),
        .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);

    // wait for HB before starting everything
    /*if (wait_for_heartbeat(HEARTBEAT_TIMEOUT_MS)) {
        ESP_LOGIa("GVI", "GVI sul bus, in attesa del pulsante per abilitare");
        // NESSUN send_nmt_start() qui.
        // Il GVI resta in pre-operational. command_word = 0, enabled = false.
    } else {
        ESP_LOGW("GVI", "No heartbeat: telemetria attiva, motore non disponibile");
    }*/


    // Core 1: only CAN
    xTaskCreatePinnedToCore(pdo_tx_task,        "pdo_tx",    4096, NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(can_receiver_task,  "can_rx",    4096, NULL, 6, NULL, 1);
    xTaskCreate(ad7190_task_lever,"ad7190_task",4096,NULL,10,&s_ad7190_task_handle);

    // Core 0: mqtt, buttons
    xTaskCreatePinnedToCore(enable_button_task, "enable_btn",4096, NULL, 4, NULL, 0);

}

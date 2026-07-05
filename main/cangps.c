#include "cangps.h"

#include <math.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/twai.h"

#include "batteries_gps_motor_2.h"

#define TAG "CANGPS"

#define STALE_POSITION_MS 3000
#define STALE_IMU_MS      500

typedef struct {
    bool fix_valid;
    bool pos_valid;
    int fix_type;
    int sats;

    double lat_deg;
    double lon_deg;
    double alt_m;

    bool speed_valid;
    double speed_ms;
    double speed_accuracy_ms;

    bool attitude_valid;
    double roll_deg;
    double roll_accuracy_deg;
    double pitch_deg;
    double pitch_accuracy_deg;
    double heading_deg;
    double heading_accuracy_deg;

    bool imu_valid;
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;

    //uint64_t last_status_ms;
    uint64_t last_position_ms;
    //uint64_t last_altitude_ms;
    //uint64_t last_attitude_ms;
    //uint64_t last_speed_ms;
    uint64_t last_imu_ms;

    //uint32_t can_timeout_count;
    uint32_t can_rx_count;
} CangpsState;

static CangpsState state = {0};
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

/* Holds up to 200 IMU samples (~2 seconds at 100 Hz) before overflowing. */
#define IMU_QUEUE_LEN 400
static QueueHandle_t imu_queue;

static inline uint64_t boot_ms_now(void)
{
    return esp_timer_get_time() / 1000ULL;
}



bool cangps_get_latest(CangpsData *out)
{
    if (out == NULL) return false;

    CangpsState s;
    uint64_t now_ms = boot_ms_now();

    portENTER_CRITICAL(&state_mux);
    s = state;
    portEXIT_CRITICAL(&state_mux);

    bool position_stale =
        (s.last_position_ms == 0) ||
        ((now_ms - s.last_position_ms) > STALE_POSITION_MS);

    bool imu_stale =
        (s.last_imu_ms == 0) ||
        ((now_ms - s.last_imu_ms) > STALE_IMU_MS);

    bool fix_valid = s.fix_type > 0;

    bool pos_valid =
        fix_valid &&
        isfinite(s.lat_deg) &&
        isfinite(s.lon_deg) &&
        (fabs(s.lat_deg) > 1e-9 || fabs(s.lon_deg) > 1e-9);

    memset(out, 0, sizeof(*out));

    out->fix_valid = fix_valid;
    out->pos_valid = pos_valid;
    out->fix_type = s.fix_type;
    out->sats = s.sats;

    out->lat_deg = s.lat_deg;
    out->lon_deg = s.lon_deg;
    out->alt_m = s.alt_m;

    out->speed_valid = s.speed_valid;
    out->speed_ms = s.speed_ms;
    out->speed_kmh = s.speed_ms * 3.6;
    out->speed_accuracy_ms = s.speed_accuracy_ms;

    out->attitude_valid = s.attitude_valid;
    out->roll_deg = s.roll_deg;
    out->pitch_deg = s.pitch_deg;
    out->heading_deg = s.heading_deg;

    out->imu_valid = s.imu_valid;
    out->accel_x = s.accel_x;
    out->accel_y = s.accel_y;
    out->accel_z = s.accel_z;
    out->gyro_x = s.gyro_x;
    out->gyro_y = s.gyro_y;
    out->gyro_z = s.gyro_z;

    out->position_stale = position_stale;
    out->imu_stale = imu_stale;
    out->data_fresh = !position_stale && !imu_stale;

    out->can_rx_count = s.can_rx_count;
    //out->can_timeout_count = s.can_timeout_count;

    out->timestamp_ms = now_ms;

    return true;
}

bool cangps_handle_frame(const twai_message_t *rx)
{
    if (rx == NULL) return false;

    uint64_t now_ms = boot_ms_now();
    bool handled = true;

    switch (rx->identifier) {

        case BATTERIES_GPS_MOTOR_2_GNSS_STATUS_FRAME_ID: {
            struct batteries_gps_motor_2_gnss_status_t m;
            batteries_gps_motor_2_gnss_status_unpack(&m, rx->data, rx->data_length_code);

            portENTER_CRITICAL(&state_mux);
            state.fix_type = m.fix_type;
            state.sats = m.satellites;
            state.fix_valid = m.fix_type > 0;
            //state.last_status_ms = now_ms;
            state.can_rx_count++;
            portEXIT_CRITICAL(&state_mux);
            break;
        }

        case BATTERIES_GPS_MOTOR_2_GNSS_POSITION_FRAME_ID: {
            struct batteries_gps_motor_2_gnss_position_t m;
            batteries_gps_motor_2_gnss_position_unpack(&m, rx->data, rx->data_length_code);

            double lat = batteries_gps_motor_2_gnss_position_latitude_decode(m.latitude);
            double lon = batteries_gps_motor_2_gnss_position_longitude_decode(m.longitude);

            portENTER_CRITICAL(&state_mux);
            state.lat_deg = lat;
            state.lon_deg = lon;
            state.pos_valid = isfinite(lat) && isfinite(lon);
            state.last_position_ms = now_ms;
            state.can_rx_count++;
            portEXIT_CRITICAL(&state_mux);
            break;
        }

        case BATTERIES_GPS_MOTOR_2_GNSS_ALTITUDE_FRAME_ID: {
            struct batteries_gps_motor_2_gnss_altitude_t m;
            batteries_gps_motor_2_gnss_altitude_unpack(&m, rx->data, rx->data_length_code);

            portENTER_CRITICAL(&state_mux);
            if (m.altitude_valid) {
                state.alt_m =
                    batteries_gps_motor_2_gnss_altitude_altitude_decode(m.altitude);
            }
            //state.last_altitude_ms = now_ms;
            state.can_rx_count++;
            portEXIT_CRITICAL(&state_mux);
            break;
        }

        case BATTERIES_GPS_MOTOR_2_GNSS_SPEED_FRAME_ID: {
            struct batteries_gps_motor_2_gnss_speed_t m;
            batteries_gps_motor_2_gnss_speed_unpack(&m, rx->data, rx->data_length_code);

            portENTER_CRITICAL(&state_mux);
            state.speed_valid = m.speed_valid > 0;
            state.speed_ms =
                batteries_gps_motor_2_gnss_speed_speed_decode(m.speed);
            state.speed_accuracy_ms =
                batteries_gps_motor_2_gnss_speed_speed_accuracy_decode(m.speed_accuracy);
            //state.last_speed_ms = now_ms;
            state.can_rx_count++;
            portEXIT_CRITICAL(&state_mux);
            break;
        }

        case BATTERIES_GPS_MOTOR_2_GNSS_ATTITUDE_FRAME_ID: {
            struct batteries_gps_motor_2_gnss_attitude_t m;
            batteries_gps_motor_2_gnss_attitude_unpack(&m, rx->data, rx->data_length_code);

            portENTER_CRITICAL(&state_mux);
            state.attitude_valid = m.attitude_valid > 0;
            state.roll_deg =
                batteries_gps_motor_2_gnss_attitude_roll_decode(m.roll);
            state.roll_accuracy_deg =
                batteries_gps_motor_2_gnss_attitude_roll_accuracy_decode(m.roll_accuracy);
            state.pitch_deg =
                batteries_gps_motor_2_gnss_attitude_pitch_decode(m.pitch);
            state.pitch_accuracy_deg =
                batteries_gps_motor_2_gnss_attitude_pitch_accuracy_decode(m.pitch_accuracy);
            state.heading_deg =
                batteries_gps_motor_2_gnss_attitude_heading_decode(m.heading);
            state.heading_accuracy_deg =
                batteries_gps_motor_2_gnss_attitude_heading_accuracy_decode(m.heading_accuracy);
            //state.last_attitude_ms = now_ms;
            state.can_rx_count++;
            portEXIT_CRITICAL(&state_mux);
            break;
        }

        case BATTERIES_GPS_MOTOR_2_GNSS_IMU_FRAME_ID: {
            struct batteries_gps_motor_2_gnss_imu_t m;
            batteries_gps_motor_2_gnss_imu_unpack(&m, rx->data, rx->data_length_code);

            double ax = batteries_gps_motor_2_gnss_imu_acceleration_x_decode(m.acceleration_x);
            double ay = batteries_gps_motor_2_gnss_imu_acceleration_y_decode(m.acceleration_y);
            double az = batteries_gps_motor_2_gnss_imu_acceleration_z_decode(m.acceleration_z);
            double gx = batteries_gps_motor_2_gnss_imu_angular_rate_x_decode(m.angular_rate_x);
            double gy = batteries_gps_motor_2_gnss_imu_angular_rate_y_decode(m.angular_rate_y);
            double gz = batteries_gps_motor_2_gnss_imu_angular_rate_z_decode(m.angular_rate_z);

            portENTER_CRITICAL(&state_mux);
            state.imu_valid = m.imu_valid > 0;
            state.accel_x = ax;
            state.accel_y = ay;
            state.accel_z = az;
            state.gyro_x = gx;
            state.gyro_y = gy;
            state.gyro_z = gz;
            state.last_imu_ms = now_ms;
            state.can_rx_count++;
            portEXIT_CRITICAL(&state_mux);

            if (m.imu_valid) {
                CangpsImuSample sample = {
                    .timestamp_ms = now_ms,
                    .accel_x = ax, .accel_y = ay, .accel_z = az,
                    .gyro_x  = gx, .gyro_y  = gy, .gyro_z  = gz,
                };
                xQueueSend(imu_queue, &sample, 0);
            }
            break;
        }

        default:
            handled = false;
            break;
    }

    return handled;
}

void cangps_init(void)
{
    imu_queue = xQueueCreate(IMU_QUEUE_LEN, sizeof(CangpsImuSample));
    if (imu_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU queue");
        return;
    }
    ESP_LOGI(TAG, "CANGPS module initialized");
}

size_t cangps_drain_imu(CangpsImuSample *buf, size_t max_samples)
{
    size_t count = 0;
    while (count < max_samples && xQueueReceive(imu_queue, &buf[count], 0) == pdTRUE) {
        count++;
    }
    return count;
}
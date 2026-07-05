#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/twai.h"

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
    double speed_kmh;
    double speed_accuracy_ms;

    bool attitude_valid;
    double roll_deg;
    double pitch_deg;
    double heading_deg;

    bool imu_valid;
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;

    bool position_stale;
    bool imu_stale;
    bool data_fresh;

    uint32_t can_rx_count;
    uint32_t can_timeout_count;

    uint64_t timestamp_ms;
} CangpsData;

/* Single IMU sample with timestamp, for lossless buffering. */
typedef struct {
    uint64_t timestamp_ms;
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
} CangpsImuSample;

/* Returns a snapshot of the latest GPS + IMU state. */
bool cangps_get_latest(CangpsData *out);

/*
 * Drains buffered IMU samples into caller-supplied array.
 * Returns the number of samples written (0..max_samples).
 * Call at whatever rate suits the MQTT publish cadence.
 */
size_t cangps_drain_imu(CangpsImuSample *buf, size_t max_samples);

/* Initializes the cangps module (creates the IMU queue and the spinlock state).
 * Does NOT touch the TWAI driver — that's the caller's responsibility.
 * Call this once at startup, before any can_handle_frame call.
 */
void cangps_init(void);

/* Feed a CAN frame into the GPS handler.
 * Returns true if the frame was a GPS frame (and was handled), false otherwise.
 * Safe to call from your CAN RX task.
 */
bool cangps_handle_frame(const twai_message_t *rx);
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* =========================================================================================================
    Data Types
*/

/** @brief Device monitored by the cooling system.
 *
 * Each cooled device stores its current temperature, the temperature threshold
 * above which cooling should start, and the temperature at which maximum
 * cooling is required.
 *
 * Temperatures are expressed in milli-degrees Celsius.
 */
typedef struct {
    char name[16];                  // Device being cooled
    int32_t temperature_mc;         // Current temperature in milliCelsius
    int32_t threshold_temp_mc;      // Cooling activation threshold
    int32_t max_allowed_temp_mc;    // Temperature at which max cooling is required
} cooled_dev_t;

/** @brief Group of devices cooled by the same actuator.
 *
 * A cooling system stores pointers to the devices monitored by one actuator.
 * The system does not own the device objects themselves; it only stores their
 * addresses.
 */
typedef struct {
    cooled_dev_t **devices;          // Devices cooled by a given actuator
    size_t num_devices;             // Number of cooled devices
    size_t max_devices;             // Maximum number of devices that can be cooled
} cool_sys_t;

/** @brief PWM-controlled cooling actuator.
 *
 * A cooling actuator controls one PWM output and computes its duty cycle from
 * the temperatures of the devices assigned through a cooling system.
 *
 * The actuator applies hysteresis to avoid fast on/off switching around the
 * threshold temperature. When active, the duty cycle is computed from the
 * hottest associated device and limited by the configured minimum duty cycle.
 */
typedef struct {
    char name[16];                  // Actuator name
    int gpio;                       // GPIO used for PWM output or enable control
    uint8_t pwm_channel;            // PWM channel (INTERNALLY SET)
    bool initialized;               // Initialization flag
    bool active;                    // Whether the actuator is currently active

    cool_sys_t *cool_sys;           // Devices cooled by this actuator

    uint32_t hysteresis_mc;         // Hysteresis in milliCelsius
    uint32_t slope_pct_per_degC;    // Duty increase [%] per milliCelsius above threshold

    uint8_t duty_cycle;             // Current duty cycle (0-100%)
    uint8_t min_duty_cycle;         // Minimum duty cycle to apply when above threshold (0-100%)

} cooling_actuator_t;




/* =========================================================================================================
    PWM Global Initialization
*/

/** @brief Initializes the global PWM timer used by all cooling actuators.
 *
 * This function configures the ESP-IDF LEDC timer used by the cooling module.
 * It must be called before initializing any PWM-based cooling actuator.
 *
 * The current driver configuration uses a fixed PWM frequency and resolution
 * defined internally by the module.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP-IDF LEDC error code if timer configuration fails
 */
esp_err_t cooling_pwm_global_init(void);


/* =========================================================================================================
    Cooling System Management
*/

/** @brief Initializes a cooling system with capacity for a fixed number of devices.
 *
 * This function allocates the internal array used to store pointers to cooled
 * devices. The devices themselves are not allocated or owned by the cooling
 * system.
 *
 * @param max_devices Maximum number of cooled devices that can be added.
 *
 * @return Initialized cooling system structure. If initialization fails, the
 * returned structure has devices set to NULL and must not be used until
 * successfully initialized.
 */
cool_sys_t cool_sys_init(size_t max_devices);


/** @brief Destroys a cooling system and releases its internal storage.
 *
 * This function frees the internal device-pointer array and resets the system
 * fields. It does not free the cooled devices themselves.
 *
 * @param sys Pointer to the cooling system to destroy.
 */
void cool_sys_destroy(cool_sys_t *sys);


/** @brief Adds a cooled device to a cooling system.
 *
 * The cooling system stores only a pointer to the provided device. Therefore,
 * the device object must remain valid for as long as the cooling system uses it.
 *
 * @param sys Pointer to an initialized cooling system.
 * @param dev Pointer to the cooled device to add.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if sys or dev is NULL
 *      - ESP_ERR_INVALID_STATE if the cooling system is not initialized
 *      - ESP_ERR_NO_MEM if the system has reached its maximum number of devices
 */
esp_err_t cool_sys_add_device(cool_sys_t *sys, cooled_dev_t *dev);


/* =========================================================================================================
    Cooling Actuator Management
*/

/** @brief Initializes a PWM-controlled cooling actuator.
 *
 * This function creates a cooling actuator structure, assigns it a free LEDC
 * PWM channel, configures the corresponding GPIO as a PWM output, and stores
 * the actuator control parameters.
 *
 * cooling_pwm_global_init() must be called before this function.
 *
 * @param name Actuator name. The string is copied into the actuator structure.
 * @param gpio GPIO used for the PWM output.
 * @param hysteresis_mc Hysteresis band in milliCelsius.
 * @param slope_pct_per_degC Duty-cycle increase in percent per degree Celsius
 * above the device threshold.
 * @param min_duty_cycle Minimum duty cycle applied when cooling is active,
 * from 0 to 100 percent.
 *
 * @return Initialized actuator structure. If initialization fails, the returned
 * actuator has initialized set to false.
 */
cooling_actuator_t cooling_actuator_init(const char *name,
                                        int gpio,
                                        uint32_t hysteresis_mc,
                                        uint32_t slope_pct_per_degC,
                                        uint8_t min_duty_cycle);


/** @brief Assigns a cooling system to an actuator.
 *
 * The actuator uses the assigned cooling system to compute its active state and
 * PWM duty cycle. The cooling system must remain valid while assigned.
 *
 * @param actuator Pointer to an initialized cooling actuator.
 * @param sys Pointer to an initialized cooling system.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if actuator or sys is NULL
 *      - ESP_ERR_INVALID_STATE if actuator or sys is not initialized
 */
esp_err_t cooling_actuator_set_system(cooling_actuator_t *actuator, cool_sys_t *sys);


/** @brief Destroys a cooling actuator and releases its PWM channel.
 *
 * This function stops the PWM output if possible, releases the internally
 * assigned PWM channel, and resets the actuator fields.
 *
 * @param actuator Pointer to the actuator to destroy.
 */
void cooling_actuator_destroy(cooling_actuator_t *actuator);


/* =========================================================================================================
    Cooling Control
*/

/** @brief Updates the actuator state and PWM duty cycle.
 *
 * This function evaluates the temperatures of all devices assigned to the
 * actuator's cooling system, applies hysteresis, computes the required duty
 * cycle, and updates the LEDC PWM output.
 *
 * The actuator must be initialized and assigned to a valid cooling system before
 * calling this function.
 *
 * @param actuator Pointer to an initialized cooling actuator.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if actuator is NULL
 *      - ESP_ERR_INVALID_STATE if the actuator is not initialized or has no
 *        cooling system assigned
 *      - ESP-IDF LEDC error code if PWM update fails
 */
esp_err_t cooling_actuator_update(cooling_actuator_t *actuator);




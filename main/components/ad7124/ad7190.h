#pragma once

#include "driver/spi_master.h"
#include "ad7190_driver.h"

// ============================
// CHANNEL SELECTION (CONFIG REG)
// ============================

// Differential inputs
#define AD7190_CH_AIN1_AIN2        (1u << 0)   // CH0
#define AD7190_CH_AIN3_AIN4        (1u << 1)   // CH1

// Internal
#define AD7190_CH_TEMP_SENSOR      (1u << 2)   // CH2

// Shorted / internal test
#define AD7190_CH_AIN2_AIN2        (1u << 3)   // CH3

// Pseudo-differential vs AINCOM
#define AD7190_CH_AIN1_AINCOM      (1u << 4)   // CH4
#define AD7190_CH_AIN2_AINCOM      (1u << 5)   // CH5
#define AD7190_CH_AIN3_AINCOM      (1u << 6)   // CH6
#define AD7190_CH_AIN4_AINCOM      (1u << 7)   // CH7


typedef struct {
    ad7190_mode_cfg_t mode;
    ad7190_config_cfg_t config;
    ad7190_gpocon_cfg_t gpocon;
    ad7190_offset_cfg_t offset;
    ad7190_fullscale_cfg_t fullscale;

    bool mode_valid;
    bool config_valid;
    bool gpocon_valid;
    bool offset_valid;
    bool fullscale_valid;
} ad7190_user_state_t;

/* =========================================================================================================
    INIT
*/
esp_err_t ad7190_init(int clk_pin,
                    int cs_pin,
                    int miso_pin,
                    int mosi_pin,
                    const ad7190_mode_cfg_t *mode_cfg,
                    const ad7190_config_cfg_t *config_cfg,
                    const ad7190_gpocon_cfg_t *gpocon_cfg);



/* =========================================================================================================
    Configuration Apply Functions
*/

/** @brief Applies an AD7190 mode-register configuration.
 *
 * This function validates the provided mode configuration, encodes it into the
 * AD7190 mode-register format, and writes it to the device.
 *
 * On success, the driver updates its internal state according to the new mode
 * configuration. In particular, the DAT_STA tracking flag is updated, and the
 * continuous-read state is cleared because writing the mode register interrupts
 * the previous data-read flow.
 *
 * A copy of the applied configuration is stored internally for later reference.
 *
 * @param cfg Pointer to the mode-register configuration to apply.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if cfg is NULL or contains invalid fields
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF error codes returned by the register write operation
 */
esp_err_t ad7190_mode_apply(const ad7190_mode_cfg_t *cfg);

/** @brief Applies an AD7190 configuration-register setup.
 *
 * This function validates the provided configuration-register structure,
 * encodes it into the AD7190 configuration-register format, and writes it to
 * the device.
 *
 * On success, the continuous-read state is cleared because changing the input
 * channel, gain, polarity, buffer, reference, or related analog settings
 * invalidates the previous data-read flow.
 *
 * A copy of the applied configuration is stored internally for later reference.
 *
 * @param cfg Pointer to the configuration-register setup to apply.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if cfg is NULL or contains invalid fields
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF error codes returned by the register write operation
 */
esp_err_t ad7190_config_apply(const ad7190_config_cfg_t *cfg);

/** @brief Applies an AD7190 GPOCON-register configuration.
 *
 * This function validates the provided GPIO-control configuration, encodes it
 * into the AD7190 GPOCON-register format, and writes it to the device.
 *
 * The GPOCON register controls the bridge power-down switch and the optional
 * digital output pins P3, P2, P1, and P0.
 *
 * A copy of the applied configuration is stored internally for later reference.
 *
 * @param cfg Pointer to the GPOCON-register configuration to apply.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if cfg is NULL or contains invalid fields
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF error codes returned by the register write operation
 */
esp_err_t ad7190_gpocon_apply(const ad7190_gpocon_cfg_t *cfg);

/** @brief Applies an AD7190 offset calibration coefficient.
 *
 * This function validates the provided 24-bit offset calibration value, encodes
 * it into the AD7190 offset-register format, and writes it to the device.
 *
 * The offset register stores the ADC offset calibration coefficient.
 *
 * A copy of the applied calibration value is stored internally for later
 * reference.
 *
 * @param cfg Pointer to the offset-register configuration to apply.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if cfg is NULL or the value exceeds 24 bits
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF error codes returned by the register write operation
 */
esp_err_t ad7190_offset_apply(const ad7190_offset_cfg_t *cfg);

/** @brief Applies an AD7190 full-scale calibration coefficient.
 *
 * This function validates the provided 24-bit full-scale calibration value,
 * encodes it into the AD7190 full-scale-register format, and writes it to the
 * device.
 *
 * The full-scale register stores the ADC gain calibration coefficient.
 *
 * A copy of the applied calibration value is stored internally for later
 * reference.
 *
 * @param cfg Pointer to the full-scale-register configuration to apply.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if cfg is NULL or the value exceeds 24 bits
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF error codes returned by the register write operation
 */
esp_err_t ad7190_fullscale_apply(const ad7190_fullscale_cfg_t *cfg);



/* =========================================================================================================
    Convenience Configuration Setters
*/
/** @brief Updates the AD7190 PGA gain while preserving the current configuration.
 *
 * This function modifies only the gain field of the currently cached
 * configuration-register setup and reapplies the configuration register.
 *
 * A valid configuration must have been previously applied using
 * ad7190_config_apply() or a higher-level initialization function.
 *
 * @param gain AD7190 PGA gain setting.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if no valid configuration state is available
 *      - ESP_ERR_INVALID_ARG if the gain value is invalid
 *      - Other ESP-IDF error codes returned by the underlying register write
 */
esp_err_t ad7190_set_gain(uint8_t gain);

/** @brief Selects exactly one AD7190 input channel.
 *
 * This function updates the channel-selection field of the currently cached
 * configuration-register setup and reapplies the configuration register.
 *
 * The provided channel mask must contain exactly one active channel bit. For
 * multi-channel configurations, use ad7190_config_apply() directly.
 *
 * A valid configuration must have been previously applied using
 * ad7190_config_apply() or a higher-level initialization function.
 *
 * @param channel_mask Channel mask containing exactly one enabled channel bit.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if no valid configuration state is available
 *      - ESP_ERR_INVALID_ARG if the channel mask is zero, contains invalid bits,
 *        or enables more than one channel
 *      - Other ESP-IDF error codes returned by the underlying register write
 */
esp_err_t ad7190_set_single_channel(uint16_t channel_mask);

/** @brief Updates the AD7190 filter-select value.
 *
 * This function modifies only the FS field of the currently cached mode-register
 * configuration and reapplies the mode register.
 *
 * The FS value controls the output data rate and digital filter settling
 * behavior according to the AD7190 mode-register configuration.
 *
 * A valid mode configuration must have been previously applied using
 * ad7190_mode_apply() or a higher-level initialization function.
 *
 * @param fs Filter-select value. Valid range is 1 to 1023.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if no valid mode state is available
 *      - ESP_ERR_INVALID_ARG if fs is outside the valid range
 *      - Other ESP-IDF error codes returned by the underlying register write
 */
esp_err_t ad7190_set_filter(uint16_t fs);

/** @brief Enables or disables status-byte append mode.
 *
 * This function updates the DAT_STA bit in the currently cached mode-register
 * configuration and reapplies the mode register.
 *
 * When enabled, the AD7190 appends one status byte to each data-register read.
 * In this driver, data reads then return a 32-bit value where bits [31:8]
 * contain the 24-bit conversion result and bits [7:0] contain the status byte.
 *
 * A valid mode configuration must have been previously applied using
 * ad7190_mode_apply() or a higher-level initialization function.
 *
 * @param enable True to append the status byte to conversion data, false to
 * disable status append mode.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if no valid mode state is available
 *      - ESP_ERR_INVALID_ARG if the resulting mode configuration is invalid
 *      - Other ESP-IDF error codes returned by the underlying register write
 */
esp_err_t ad7190_enable_status_append(bool enable);


/* =========================================================================================================
    Status and Single Conversion
*/

/** @brief Reads the AD7190 status register.
 *
 * This function reads the AD7190 status register and returns its 8-bit value.
 * The status register contains the RDY bit, error flags, no-reference flag,
 * parity flag, and the active channel identification bits.
 *
 * @param status Pointer where the status-register value will be stored.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if status is NULL
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF error codes returned by the underlying register read
 */
esp_err_t ad7190_status_read(uint8_t *status);

/** @brief Checks whether a new AD7190 conversion result is ready.
 *
 * This function reads the AD7190 status register and evaluates the RDY bit.
 * In the AD7190, RDY is active-low, so the device is considered ready when
 * the RDY bit is cleared.
 *
 * @param ready Pointer where the readiness state will be stored.
 *              True means a new conversion result is available.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if ready is NULL
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF error codes returned by the status-register read
 */
esp_err_t ad7190_is_ready(bool *ready);


/** @brief Starts a single AD7190 conversion.
 *
 * This function updates the cached mode-register configuration by selecting
 * single-conversion mode and then reapplies the mode register.
 *
 * A valid mode configuration must have been previously applied using
 * ad7190_mode_apply() or a higher-level initialization function.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if no valid mode state is available
 *      - ESP_ERR_INVALID_ARG if the resulting mode configuration is invalid
 *      - Other ESP-IDF error codes returned by the underlying register write
 */
esp_err_t ad7190_start_single_conversion(void);

/** @brief Reads one AD7190 conversion result without status byte.
 *
 * This function reads one conversion result using the current driver data-read
 * mode. It must be used only when DAT_STA is disabled.
 *
 * If DAT_STA is enabled, use ad7190_conversion_read_ex() instead, because the
 * AD7190 appends an additional status byte to each conversion result.
 *
 * @param data Pointer where the 24-bit conversion result will be stored.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if data is NULL
 *      - ESP_ERR_INVALID_STATE if DAT_STA is enabled or the SPI device is not initialized
 *      - Other ESP-IDF error codes returned by the underlying data read
 */
esp_err_t ad7190_conversion_read(uint32_t *data);


/** @brief Reads one AD7190 conversion result together with the status byte.
 *
 * This function reads one conversion frame when DAT_STA is enabled. The raw
 * frame returned by the low-level driver is split into a 24-bit conversion
 * result and an 8-bit status byte.
 *
 * DAT_STA must be enabled before calling this function.
 *
 * @param data Pointer where the 24-bit conversion result will be stored.
 * @param status Pointer where the appended status byte will be stored.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if data or status is NULL
 *      - ESP_ERR_INVALID_STATE if DAT_STA is not enabled or the SPI device is not initialized
 *      - Other ESP-IDF error codes returned by the underlying data read
 */
esp_err_t ad7190_conversion_read_ex(uint32_t *data, uint8_t *status);


/* =========================================================================================================
    Continuous Conversion Mode
*/

/** @brief Starts AD7190 continuous-read mode.
 *
 * This function enables the AD7190 continuous-read mode through the low-level
 * driver. In this mode, conversion data can be read directly from DOUT/RDY
 * without sending a new communication-register command before each sample.
 *
 * Use ad7190_conversion_read() when DAT_STA is disabled, or
 * ad7190_conversion_read_ex() when DAT_STA is enabled, to read samples after
 * continuous-read mode has been started.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF SPI error codes returned by the low-level driver
 */
esp_err_t ad7190_continuous_start(void);

/** @brief Stops AD7190 continuous-read mode and returns the last data frame.
 *
 * This function disables continuous-read mode and retrieves the data frame
 * shifted out during the stop transaction.
 *
 * If DAT_STA is disabled, last_data receives the 24-bit conversion result.
 * If DAT_STA is enabled, last_data receives the packed 32-bit frame where
 * bits [31:8] contain the 24-bit conversion result and bits [7:0] contain the
 * status byte.
 *
 * @param last_data Pointer where the last received data frame will be stored.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if last_data is NULL
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF SPI error codes returned by the low-level driver
 */
esp_err_t ad7190_continuous_stop(uint32_t *last_data);
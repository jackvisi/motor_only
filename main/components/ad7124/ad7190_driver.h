#pragma once


#include "ad7190_regs.h"
#include "driver/spi_master.h"


/* =========================================================================================================
    Structs
*/
typedef struct {
    bool disable_cont_read_pending;     // flag disable continuous read
    bool cs_keep_active;
} ad7190_flags_t;

typedef struct {
    spi_device_handle_t spi;
    bool continuous_read_enabled;   // Current state
    bool dat_sta_enabled;           // Current dat_sta_enabled
    ad7190_flags_t flags;
} ad7190_driver_t;

typedef struct {
    uint8_t mode;        // 0–7 (continuous, single, idle, etc.)
    bool dat_sta;        // Append status to data register
    uint8_t clock;       // 0–3
    bool sinc3;
    bool enpar;
    bool single_cycle;
    bool rej60;
    uint16_t fs;         // 10 bits
} ad7190_mode_cfg_t;

typedef struct {
    bool chop;
    bool refsel;
    uint16_t channel_mask;   // bits 0–7 → mucho mejor que “channel x”
    bool burn;
    bool refdet;
    bool buffer;
    bool unipolar;
    uint8_t gain;            // 0–7
} ad7190_config_cfg_t;

typedef struct {
    bool bpdsw;
    bool gp32en;
    bool gp10en;
    bool p3dat;
    bool p2dat;
    bool p1dat;
    bool p0dat;
} ad7190_gpocon_cfg_t;

typedef struct {
    uint32_t value;   // 24 bits
} ad7190_offset_cfg_t;

typedef struct {
    uint32_t value;   // 24 bits
} ad7190_fullscale_cfg_t;

/* =========================================================================================================
    Init functions
*/
/** @brief Initializes the SPI interface used to communicate with the AD7190.
 *
 * This function configures the selected SPI bus pins, initializes the SPI bus,
 * and attaches the AD7190 as an SPI device. The resulting SPI device handle is
 * stored internally by the driver and used by the remaining AD7190 functions.
 *
 * In manual chip-select mode, the CS pin is configured as a GPIO output and the
 * SPI driver does not control CS automatically. In automatic chip-select mode,
 * the CS pin is handled directly by the ESP-IDF SPI driver.
 *
 * @param clk_pin  GPIO number used as SPI clock/SCLK.
 * @param cs_pin   GPIO number used as chip select.
 * @param miso_pin GPIO number used as MISO/DOUT/RDY.
 * @param mosi_pin GPIO number used as MOSI/DIN.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if one of the GPIO numbers is invalid
 *      - ESP_ERR_INVALID_STATE if the SPI bus or device is already initialized
 *      - Other ESP-IDF error codes returned by spi_bus_initialize() or
 *        spi_bus_add_device()
 */
esp_err_t ad7190_spi_init(int clk_pin, int cs_pin, int miso_pin, int mosi_pin);


/** @brief Resets the AD7190 by sending at least 40 consecutive logic 1 bits.
 *
 * The AD7190 can be reset by writing a sequence of 40 consecutive HIGH bits
 * on DIN while the SPI interface is active. This function sends five bytes
 * with value 0xFF over SPI, resets the internal driver state flags, and waits
 * at least 500 us before allowing further register access.
 *
 * After this function returns successfully, the driver assumes that continuous
 * read mode is disabled and that DAT_STA is disabled, matching the AD7190
 * default communication state after reset.
 *
 * @note This function requires the AD7190 SPI device handle to be already
 * initialized.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP-IDF SPI error code if the reset transaction fails
 */
esp_err_t ad7190_reset(void);

/* =========================================================================================================
    Register Access
*/ 
/** @brief Reads a register from the AD7190.
 *
 * This function sends a normal AD7190 register-read command through the
 * communication register and then reads the number of bytes associated with
 * the selected register.
 *
 * The communication register is sent using the SPI command phase. During the
 * read phase, zeroes are transmitted on DIN to keep the line low and avoid an
 * accidental serial-interface reset caused by 40 consecutive logic 1 bits.
 *
 * If the data register is read while DAT_STA is enabled, one additional status
 * byte is read together with the conversion data.
 *
 * @param regCode AD7190 register code to read.
 * @param data_out Pointer where the register value will be stored.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if data_out is NULL or regCode is invalid
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - ESP_ERR_INVALID_SIZE if the selected register size is unsupported
 *      - Other ESP-IDF SPI error codes returned by spi_device_polling_transmit()
 */
esp_err_t ad7190_read_register(uint8_t regCode, uint32_t *data_out);

/** @brief Writes a value to an AD7190 register.
 *
 * This function sends a normal AD7190 register-write command through the
 * communication register and then writes the number of bytes associated with
 * the selected register.
 *
 * The communication register is sent using the SPI command phase. Therefore,
 * the SPI device must have been configured with command_bits = 8 during
 * initialization.
 *
 * Register values are transmitted MSB first, as required by the AD7190.
 *
 * @param regCode AD7190 register code to write.
 * @param value Value to write to the selected register. Only the least
 * significant bits corresponding to the register size are used.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if regCode is invalid
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - ESP_ERR_INVALID_SIZE if the selected register size is unsupported
 *      - Other ESP-IDF SPI error codes returned by spi_device_polling_transmit()
 */
esp_err_t ad7190_write_register(uint8_t regCode, uint32_t value);

/* =========================================================================================================
    Data Acquisition
*/
/** @brief Reads one conversion result from the AD7190 data register.
 *
 * This function is the recommended public data-read entry point. It selects
 * the appropriate read method depending on the current driver state:
 *
 * - If continuous-read mode is disabled, the data register is read using a
 *   normal register-read transaction.
 * - If continuous-read mode is enabled, one data frame is clocked directly
 *   from DOUT/RDY.
 * - If a continuous-read disable operation is pending, continuous-read mode is
 *   disabled while reading the current data frame.
 *
 * If DAT_STA is enabled, the returned 32-bit value contains the 24-bit
 * conversion result in bits [31:8] and the status byte in bits [7:0].
 * Otherwise, the returned value contains only the 24-bit conversion result.
 *
 * @param data Pointer where the conversion result will be stored.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if data is NULL
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF SPI error codes returned by the underlying transaction
 */
esp_err_t ad7190_read_data(uint32_t *data);

/* =========================================================================================================
 *  Continuous-Read Mode
 *
 *  Low-level continuous-read helpers used by the higher-level AD7190 API.
 *  Application code should normally use the functions exposed in ad7190.h
 *  instead of calling these directly.
 */

/** @brief Enables AD7190 continuous-read mode for the data register.
 *
 * This function sends the AD7190 continuous-read command for the data register.
 * After this command, the AD7190 continuously outputs conversion data whenever
 * DOUT/RDY goes low, without requiring a new communication-register command
 * before each data read.
 *
 * If DAT_STA is enabled in the mode register, each continuous-read frame is
 * expected to contain 4 bytes: 24 bits of conversion data followed by 8 bits
 * of status. Otherwise, each frame contains 3 bytes of conversion data.
 *
 * While continuous-read mode is enabled, normal register accesses should not be
 * performed directly. Continuous-read mode must first be disabled according to
 * the AD7190 communication protocol.
 *
 * @note This function assumes that the SPI interface has already been
 * initialized.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - Other ESP-IDF SPI error codes returned by spi_device_polling_transmit()
 */
esp_err_t ad7190_start_contread(void);

/** @brief Low-level function that disables AD7190 continuous-read mode.
 *
 * This function sends the AD7190 command required to exit continuous-read mode
 * and reads the data frame shifted out during that transaction.
 *
 * This is a low-level driver function. Application code should normally use
 * ad7190_continuous_stop() instead.
 *
 * @param data_out Pointer where the last data frame will be stored.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if data_out is NULL
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - ESP-IDF SPI error code if the transaction fails
 */
esp_err_t ad7190_stop_contread(uint32_t *data_out);

/** @brief Low-level function that reads one frame in continuous-read mode.
 *
 * This function clocks out one AD7190 data frame directly from DOUT/RDY without
 * sending a communication-register command. It is intended to be used only when
 * continuous-read mode is already active.
 *
 * This is a low-level driver function. Application code should normally use
 * ad7190_conversion_read() or ad7190_conversion_read_ex() instead.
 *
 * @param data Pointer where the received data frame will be stored.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if data is NULL
 *      - ESP_ERR_INVALID_STATE if the SPI device is not initialized
 *      - ESP-IDF SPI error code if the transaction fails
 */
esp_err_t ad7190_data_contread(uint32_t *data);



/* =========================================================================================================
    Register Encoders
*/

/** @brief Encodes an AD7190 mode-register configuration structure.
 *
 * This function converts a user-level mode configuration structure into the
 * 24-bit value expected by the AD7190 mode register.
 *
 * The encoded value includes the operating mode, DAT_STA option, clock source,
 * filter configuration, parity option, single-cycle settling option, and
 * 50/60 Hz rejection settings.
 *
 * @pre cfg must not be NULL.
 *
 * @param cfg Pointer to the mode-register configuration structure.
 *
 * @return Encoded 24-bit AD7190 mode-register value.
 */
uint32_t ad7190_mode_encode(const ad7190_mode_cfg_t *cfg);

/** @brief Encodes an AD7190 configuration-register structure.
 *
 * This function converts a user-level configuration structure into the 24-bit
 * value expected by the AD7190 configuration register.
 *
 * The encoded value includes the chop setting, reference selection, enabled
 * input channels, burnout current setting, reference-detect option, input
 * buffer setting, polarity mode, and PGA gain.
 *
 * @pre cfg must not be NULL.
 *
 * @param cfg Pointer to the configuration-register structure.
 *
 * @return Encoded 24-bit AD7190 configuration-register value.
 */
uint32_t ad7190_config_encode(const ad7190_config_cfg_t *cfg);

/** @brief Encodes an AD7190 GPOCON-register configuration structure.
 *
 * This function converts a user-level GPIO-control configuration structure into
 * the value expected by the AD7190 GPOCON register.
 *
 * The encoded value configures the bridge power-down switch, enables or
 * disables the P3/P2 and P1/P0 digital output pairs, and sets the output value
 * of pins P3, P2, P1, and P0.
 *
 * Bit GP7 is reserved and is kept cleared by the encoding macros.
 *
 * @pre cfg must not be NULL.
 *
 * @param cfg Pointer to the GPOCON-register configuration structure.
 *
 * @return Encoded AD7190 GPOCON-register value.
 */
uint32_t ad7190_gpocon_encode(const ad7190_gpocon_cfg_t *cfg);

/** @brief Encodes an AD7190 offset-register calibration value.
 *
 * This function converts a user-level offset calibration structure into the
 * 24-bit value expected by the AD7190 offset register.
 *
 * The offset register stores the ADC offset calibration coefficient.
 *
 * @pre cfg must not be NULL.
 *
 * @param cfg Pointer to the offset-register configuration structure.
 *
 * @return Encoded 24-bit AD7190 offset-register value.
 */
uint32_t ad7190_offset_encode(const ad7190_offset_cfg_t *cfg);

/** @brief Encodes an AD7190 full-scale-register calibration value.
 *
 * This function converts a user-level full-scale calibration structure into the
 * 24-bit value expected by the AD7190 full-scale register.
 *
 * The full-scale register stores the ADC gain calibration coefficient.
 *
 * @pre cfg must not be NULL.
 *
 * @param cfg Pointer to the full-scale-register configuration structure.
 *
 * @return Encoded 24-bit AD7190 full-scale-register value.
 */
uint32_t ad7190_fullscale_encode(const ad7190_fullscale_cfg_t *cfg);


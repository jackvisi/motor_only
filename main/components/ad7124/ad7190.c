//============================
// INCLUDES begin
//============================

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "ad7190.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

//============================
// INCLUDES end
//============================

//============================
// DEFINES begin
//============================

#define ad7190_TAG "ad7190"

//============================
// DEFINES end
//============================

//============================
// PV begin
//============================
extern ad7190_driver_t ad7190;
static ad7190_user_state_t s_user = {0};
//============================
// PV end
//============================



//============================
// INIT Functions begin
//============================

/** @brief Clears the cached user-level AD7190 configuration state.
 *
 * This internal helper resets the stored configuration copies and validity
 * flags used by the convenience setter functions.
 */
static void ad7190_user_state_reset(void)
{
    memset(&s_user, 0, sizeof(s_user));
}

esp_err_t ad7190_init(int clk_pin,
                    int cs_pin,
                    int miso_pin,
                    int mosi_pin,
                    const ad7190_mode_cfg_t *mode_cfg,
                    const ad7190_config_cfg_t *config_cfg,
                    const ad7190_gpocon_cfg_t *gpocon_cfg)
{
    esp_err_t err;

    if ((mode_cfg == NULL) || (config_cfg == NULL)) {
        ESP_LOGE(ad7190_TAG, "Mode and config initial settings must not be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ad7190_user_state_reset();
    ESP_LOGI(ad7190_TAG, "User state reset to defaults");
    
    err = ad7190_spi_init(clk_pin, cs_pin, miso_pin, mosi_pin);
    if (err != ESP_OK) {
        ESP_LOGE(ad7190_TAG, "SPI init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(ad7190_TAG, "SPI initialized successfully");
    
    err = ad7190_reset();
    if (err != ESP_OK) {
        ESP_LOGE(ad7190_TAG, "ADC reset failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(ad7190_TAG, "ADC reset successful");

    uint32_t REG = 0;
    ad7190_read_register(AD7190_ID_REG, &REG);
    ESP_LOGW(ad7190_TAG, "AD7190 ID: 0x%02X", REG);

    err = ad7190_mode_apply(mode_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(ad7190_TAG, "Initial mode apply failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(ad7190_TAG, "Initial mode configuration applied successfully");

    REG = 0;
    ad7190_read_register(AD7190_MODE_REG, &REG);
    ESP_LOGW(ad7190_TAG, "AD7190 MODE REG after apply: 0x%06X", REG);

    err = ad7190_config_apply(config_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(ad7190_TAG, "Initial config apply failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(ad7190_TAG, "Initial config configuration applied successfully");

    REG = 0;
    ad7190_read_register(AD7190_CONFIG_REG, &REG);
    ESP_LOGW(ad7190_TAG, "AD7190 CONFIG REG after apply: 0x%06X", REG);

    if (gpocon_cfg != NULL) {
        err = ad7190_gpocon_apply(gpocon_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(ad7190_TAG, "Initial gpocon apply failed: %s", esp_err_to_name(err));
            return err;
        }
    }
    ESP_LOGI(ad7190_TAG, "Initial GPOCON configuration applied successfully");
    REG = 0;
    ad7190_read_register(AD7190_GPOCON_REG, &REG);
    ESP_LOGW(ad7190_TAG, "AD7190 GPOCON REG after apply: 0x%06X", REG);

    ESP_LOGI(ad7190_TAG, "Initialization successful");
    return err;
}

//============================
// INIT Functions end
//============================


//============================
// VALIDATE Functions begin
//============================

/** @brief Validates an AD7190 mode-register configuration.
 *
 * Checks that the mode, clock source and filter-select values fit within the
 * AD7190 mode-register bit fields. Also verifies that parity is not enabled
 * unless DAT_STA is enabled, since parity information is only available when
 * the status byte is appended to the data register.
 *
 * @param cfg Pointer to the mode-register configuration.
 *
 * @return
 *      - ESP_OK if the configuration is valid
 *      - ESP_ERR_INVALID_ARG if cfg is NULL or any field is out of range
 */
static esp_err_t ad7190_mode_validate(const ad7190_mode_cfg_t *cfg)
{
    if (cfg == NULL) {
        ESP_LOGE(ad7190_TAG, "Mode config pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->mode > 7u) {
        ESP_LOGE(ad7190_TAG, "Invalid mode value: %u", cfg->mode);
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->clock > 3u) {
        ESP_LOGE(ad7190_TAG, "Invalid clock value: %u", cfg->clock);
        return ESP_ERR_INVALID_ARG;
    }

    if ((cfg->fs == 0u) || (cfg->fs > 1023u)) {
        ESP_LOGE(ad7190_TAG, "Invalid FS value: %u", cfg->fs);
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->enpar && !cfg->dat_sta) {
        ESP_LOGE(ad7190_TAG, "ENPAR requires DAT_STA to be enabled");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}


/** @brief Validates an AD7190 configuration-register structure.
 *
 * Checks that the selected channel mask only uses valid AD7190 channel bits,
 * that at least one channel is enabled, and that the selected PGA gain is
 * within the supported range.
 *
 * @param cfg Pointer to the configuration-register structure.
 *
 * @return
 *      - ESP_OK if the configuration is valid
 *      - ESP_ERR_INVALID_ARG if cfg is NULL or any field is out of range
 */
static esp_err_t ad7190_config_validate(const ad7190_config_cfg_t *cfg)
{
    if (cfg == NULL) {
        ESP_LOGE(ad7190_TAG, "Configuration pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if ((cfg->channel_mask & ~0x00FFu) != 0u) {
        ESP_LOGE(ad7190_TAG, "Invalid channel mask: 0x%04X", cfg->channel_mask);
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->channel_mask == 0u) {
        ESP_LOGE(ad7190_TAG, "At least one channel must be enabled");
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->gain > AD7190_PGA_GAIN_128) {
        ESP_LOGE(ad7190_TAG, "Invalid gain value: %u", cfg->gain);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/** @brief Validates an AD7190 GPOCON-register configuration.
 *
 * Currently this function only checks that the configuration pointer is valid.
 * The individual GPOCON fields are expected to be boolean or already limited
 * by their corresponding configuration types.
 *
 * @param cfg Pointer to the GPOCON-register configuration.
 *
 * @return
 *      - ESP_OK if the configuration is valid
 *      - ESP_ERR_INVALID_ARG if cfg is NULL
 */
static esp_err_t ad7190_gpocon_validate(const ad7190_gpocon_cfg_t *cfg)
{
    if (cfg == NULL) {
        ESP_LOGE(ad7190_TAG, "GPOCON config pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/** @brief Validates an AD7190 offset-register calibration value.
 *
 * Checks that the offset calibration coefficient fits in the 24-bit AD7190
 * offset register.
 *
 * @param cfg Pointer to the offset-register configuration.
 *
 * @return
 *      - ESP_OK if the configuration is valid
 *      - ESP_ERR_INVALID_ARG if cfg is NULL or the value exceeds 24 bits
 */
static esp_err_t ad7190_offset_validate(const ad7190_offset_cfg_t *cfg)
{
    if (cfg == NULL) {
        ESP_LOGE(ad7190_TAG, "Offset config pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->value > 0xFFFFFFu) {
        ESP_LOGE(ad7190_TAG, "Invalid offset value: 0x%08" PRIX32, cfg->value);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/** @brief Validates an AD7190 full-scale-register calibration value.
 *
 * Checks that the full-scale calibration coefficient fits in the 24-bit AD7190
 * full-scale register.
 *
 * @param cfg Pointer to the full-scale-register configuration.
 *
 * @return
 *      - ESP_OK if the configuration is valid
 *      - ESP_ERR_INVALID_ARG if cfg is NULL or the value exceeds 24 bits
 */
static esp_err_t ad7190_fullscale_validate(const ad7190_fullscale_cfg_t *cfg)
{
    if (cfg == NULL) {
        ESP_LOGE(ad7190_TAG, "Fullscale config pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->value > 0xFFFFFFu) {
        ESP_LOGE(ad7190_TAG, "Invalid fullscale value: 0x%08" PRIX32, cfg->value);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

//============================
// VALIDATE Functions end
//============================

//============================
// Configuration Apply Functions begin
//============================

esp_err_t ad7190_mode_apply(const ad7190_mode_cfg_t *cfg)
{
    esp_err_t err = ad7190_mode_validate(cfg);
    if(err != ESP_OK){
        return err;
    }

    uint32_t reg = ad7190_mode_encode(cfg);
    err =  ad7190_write_register(AD7190_MODE_REG, reg);
    if (err == ESP_OK) {
        ad7190.dat_sta_enabled = cfg->dat_sta;
        ad7190.continuous_read_enabled = false;
        ad7190.flags.disable_cont_read_pending = false;

        s_user.mode = *cfg;
        s_user.mode_valid = true;
    } else {
        ESP_LOGE(ad7190_TAG, "Failed to apply mode configuration: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ad7190_config_apply(const ad7190_config_cfg_t *cfg)
{
    esp_err_t err = ad7190_config_validate(cfg);
    if(err != ESP_OK){
        return err;
    }

    uint32_t reg = ad7190_config_encode(cfg);
    err =  ad7190_write_register(AD7190_CONFIG_REG, reg);
     if (err == ESP_OK) {
        ad7190.continuous_read_enabled = false;
        ad7190.flags.disable_cont_read_pending = false;

        s_user.config = *cfg;
        s_user.config_valid = true;
    } else {
        ESP_LOGE(ad7190_TAG, "Failed to apply config configuration: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ad7190_gpocon_apply(const ad7190_gpocon_cfg_t *cfg)
{
    esp_err_t err = ad7190_gpocon_validate(cfg);
    if(err != ESP_OK){
        return err;
    }

    uint32_t reg = ad7190_gpocon_encode(cfg);
    err =  ad7190_write_register(AD7190_GPOCON_REG, reg);
    if (err == ESP_OK) {
        s_user.gpocon = *cfg;
        s_user.gpocon_valid = true;
    } else {
        ESP_LOGE(ad7190_TAG, "Failed to apply gpocon configuration: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t ad7190_offset_apply(const ad7190_offset_cfg_t *cfg)
{
    esp_err_t err = ad7190_offset_validate(cfg);
    if(err != ESP_OK){
        return err;
    }

    uint32_t reg = ad7190_offset_encode(cfg);
    err =  ad7190_write_register(AD7190_OFFSET_REG, reg);
    if (err == ESP_OK) {
        s_user.offset = *cfg;
        s_user.offset_valid = true;
    } else {
        ESP_LOGE(ad7190_TAG, "Failed to apply offset configuration: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ad7190_fullscale_apply(const ad7190_fullscale_cfg_t *cfg)
{
    esp_err_t err = ad7190_fullscale_validate(cfg);
    if(err != ESP_OK){
        return err;
    }

    uint32_t reg = ad7190_fullscale_encode(cfg);
    err =  ad7190_write_register(AD7190_FULL_SCALE_REG, reg);
    if (err == ESP_OK) {
        s_user.fullscale = *cfg;
        s_user.fullscale_valid = true;
    } else {
        ESP_LOGE(ad7190_TAG, "Failed to apply fullscale configuration: %s", esp_err_to_name(err));
    }
    return err;
}

//============================
// Configuration Apply Functions end
//============================

// ===========================
// Internal User-State Helpers begin
// ===========================

esp_err_t ad7190_set_gain(uint8_t gain)
{
    if (!s_user.config_valid) {
        ESP_LOGE(ad7190_TAG, "Config state not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ad7190_config_cfg_t cfg = s_user.config;
    cfg.gain = gain;

    return ad7190_config_apply(&cfg);
}

esp_err_t ad7190_set_single_channel(uint16_t channel_mask)
{
    if (!s_user.config_valid) {
        ESP_LOGE(ad7190_TAG, "Config state not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if ((channel_mask == 0u) || ((channel_mask & ~0x00FFu) != 0u)) {
        ESP_LOGE(ad7190_TAG, "Invalid channel mask: 0x%04X", channel_mask);
        return ESP_ERR_INVALID_ARG;
    }

    if ((channel_mask & (channel_mask - 1u)) != 0u) {
        ESP_LOGE(ad7190_TAG, "Single-channel selection requires exactly one active bit");
        return ESP_ERR_INVALID_ARG;
    }

    ad7190_config_cfg_t cfg = s_user.config;
    cfg.channel_mask = channel_mask;

    return ad7190_config_apply(&cfg);
}

esp_err_t ad7190_set_filter(uint16_t fs)
{
    if (!s_user.mode_valid) {
        ESP_LOGE(ad7190_TAG, "Mode state not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ad7190_mode_cfg_t cfg = s_user.mode;
    cfg.fs = fs;

    return ad7190_mode_apply(&cfg);
}

esp_err_t ad7190_enable_status_append(bool enable)
{
    if (!s_user.mode_valid) {
        ESP_LOGE(ad7190_TAG, "Mode state not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ad7190_mode_cfg_t cfg = s_user.mode;
    cfg.dat_sta = enable;

    return ad7190_mode_apply(&cfg);
}

// ===========================
// Internal User-State Helpers end
// ===========================


//============================
// Status and Conversion Functions begin
//============================

esp_err_t ad7190_status_read(uint8_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t reg = 0;
    esp_err_t err = ad7190_read_register(AD7190_STATUS_REG, &reg);
    if (err != ESP_OK) {
        ESP_LOGE(ad7190_TAG, "Failed to read status register: %s", esp_err_to_name(err));
        return err;
    }

    *status = (uint8_t)(reg & 0xFFu);
    return ESP_OK;
}

esp_err_t ad7190_is_ready(bool *ready)
{
    if (ready == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status = 0;
    esp_err_t err = ad7190_status_read(&status);
    if (err != ESP_OK) {
        return err;
    }

    *ready = ((status & (1 << AD7190_STATUS_RDY_POS)) == 0u);
    return ESP_OK;
}

esp_err_t ad7190_start_single_conversion(void)
{
    if (!s_user.mode_valid) {
        ESP_LOGE(ad7190_TAG, "Mode state not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ad7190_mode_cfg_t cfg = s_user.mode;
    cfg.mode = AD7190_MODE_SINGLE;

    return ad7190_mode_apply(&cfg);
}

esp_err_t ad7190_conversion_read(uint32_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ad7190.dat_sta_enabled) {
        ESP_LOGE(ad7190_TAG, "DAT_STA enabled; use ad7190_conversion_read_ex()");
        return ESP_ERR_INVALID_STATE;
    }

    return ad7190_read_data(data);
}

esp_err_t ad7190_conversion_read_ex(uint32_t *data, uint8_t *status)
{
    if ((data == NULL) || (status == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!ad7190.dat_sta_enabled) {
        ESP_LOGE(ad7190_TAG, "DAT_STA not enabled");
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t raw = 0;
    esp_err_t err = ad7190_read_data(&raw);
    if (err != ESP_OK) {
        ESP_LOGE(ad7190_TAG, "Failed to read conversion with status: %s", esp_err_to_name(err));
        return err;
    }

    *data = (raw >> 8) & 0x00FFFFFFu;
    *status = (uint8_t)(raw & 0xFFu);

    return ESP_OK;
}

//============================
// Status and Conversion Functions end
//============================

//============================
// Continuous Conversion Mode Functions begin
//============================

esp_err_t ad7190_continuous_start(void)
{
    esp_err_t err = ad7190_start_contread();
    if (err != ESP_OK) {
        ESP_LOGE(ad7190_TAG, "Failed to start continuous read: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ad7190_continuous_stop(uint32_t *last_data)
{
    if (last_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ad7190_stop_contread(last_data);
    if (err != ESP_OK) {
        ESP_LOGE(ad7190_TAG, "Failed to stop continuous read: %s", esp_err_to_name(err));
    }
    return err;
}

//============================
// Continuous Conversion Mode Functions end
//============================


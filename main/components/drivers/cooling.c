#include "cooling.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include "driver/ledc.h"
#include "hal/ledc_types.h"
#include "soc/soc_caps.h"

#define COOLING_TAG "COOLING_DRV"

#define COOLING_MAX_PWM_CHANNELS 8
#define COOLING_PWM_INVALID_CH   0xFF

#if SOC_LEDC_SUPPORT_HS_MODE
#define COOLING_LEDC_MODE LEDC_HIGH_SPEED_MODE
#else
#define COOLING_LEDC_MODE LEDC_LOW_SPEED_MODE
#endif

#define COOLING_LEDC_TIMER        LEDC_TIMER_0
#define COOLING_LEDC_FREQ_HZ      25000
#define COOLING_LEDC_RESOLUTION   LEDC_TIMER_8_BIT
#define LEDC_MAX_DUTY ((1 << COOLING_LEDC_RESOLUTION) - 1)

static bool s_pwm_channel_used[COOLING_MAX_PWM_CHANNELS] = {0};
static bool s_pwm_global_initialized = false;



//============================
// PWM Handle begin
//============================

esp_err_t cooling_pwm_global_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = COOLING_LEDC_MODE,
        .timer_num = COOLING_LEDC_TIMER,
        .freq_hz = COOLING_LEDC_FREQ_HZ,
        .duty_resolution = COOLING_LEDC_RESOLUTION,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(COOLING_TAG, "Failed to initialize global PWM timer");
        return err;
    }

    s_pwm_global_initialized = true;
    ESP_LOGI(COOLING_TAG, "Global PWM initialized successfully");
    return ESP_OK;
}
static esp_err_t cooling_actuator_pwm_init(cooling_actuator_t *act)
{
    if (!s_pwm_global_initialized) {
        ESP_LOGE(COOLING_TAG, "PWM global init not done");
        return ESP_ERR_INVALID_STATE;
    }

    ledc_channel_config_t ch = {
        .gpio_num   = act->gpio,
        .speed_mode = COOLING_LEDC_MODE,
        .channel    = (ledc_channel_t)act->pwm_channel,
        .timer_sel  = COOLING_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };

    return ledc_channel_config(&ch);
}

static esp_err_t cooling_actuator_pwm_stop(const cooling_actuator_t *act)
{
    if (act == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_pwm_global_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (act->pwm_channel == COOLING_PWM_INVALID_CH) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ledc_set_duty(
        COOLING_LEDC_MODE,
        (ledc_channel_t)act->pwm_channel,
        0
    );
    if (err != ESP_OK) {
        return err;
    }

    return ledc_update_duty(COOLING_LEDC_MODE, (ledc_channel_t)act->pwm_channel);
}

//============================
// PWM Handle end
//============================


//============================
// VALIDATE FUNCTIONS begin
//============================

static esp_err_t cooling_system_validate(const cool_sys_t *sys)
{
    if (sys == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid cooling system pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (sys->devices == NULL) {
        ESP_LOGE(COOLING_TAG, "Cooling system not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static esp_err_t cooling_actuator_validate(const cooling_actuator_t *actuator)
{
    if (actuator == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid actuator pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (!actuator->initialized) {
        ESP_LOGE(COOLING_TAG, "Actuator not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}


//============================
// VALIDATE FUNCTIONS end
//============================

//============================
// PWM Functions begin
//============================

static uint8_t cooling_allocate_pwm_channel(void)
{
    for (uint8_t i = 0; i < COOLING_MAX_PWM_CHANNELS; i++) {
        if (!s_pwm_channel_used[i]) {
            s_pwm_channel_used[i] = true;
            return i;
        }
    }

    return COOLING_PWM_INVALID_CH;
}

static void cooling_release_pwm_channel(uint8_t channel)
{
    if (channel < COOLING_MAX_PWM_CHANNELS) {
        s_pwm_channel_used[channel] = false;
    }
}

//============================
// PWM Functions end
//============================

//============================
// COOLING SYSTEM begin
//============================

cool_sys_t cool_sys_init(size_t max_devices)
{
    cool_sys_t sys = {0};

    if (max_devices == 0) {
        ESP_LOGE(COOLING_TAG, "max_devices cannot be 0");
        return sys;
    }

    sys.devices = malloc(max_devices * sizeof(*sys.devices));
    if (sys.devices == NULL) {
        ESP_LOGE(COOLING_TAG, "Memory allocation failed");
        return sys;
    }

    sys.num_devices = 0;
    sys.max_devices = max_devices;

    ESP_LOGI(COOLING_TAG, "System was initialized, but does not contain any device yet");
    return sys;
}

void cool_sys_destroy(cool_sys_t *sys)
{
    if (sys == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid cooling system pointer");
        return;
    }

    free(sys->devices);
    sys->devices = NULL;
    sys->num_devices = 0;
    sys->max_devices = 0;

    ESP_LOGI(COOLING_TAG, "System was deleted");
}

esp_err_t cool_sys_add_device(cool_sys_t *sys, cooled_dev_t *dev)
{
    esp_err_t err = cooling_system_validate(sys);
    if (err != ESP_OK) {
        return err;
    }

    if (dev == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid device pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (sys->num_devices >= sys->max_devices) {
        ESP_LOGE(COOLING_TAG, "Exceeded maximum amount of devices within system");
        return ESP_ERR_NO_MEM;
    }

    sys->devices[sys->num_devices] = dev;
    sys->num_devices++;

    ESP_LOGI(COOLING_TAG, "Device was successfully added");
    return ESP_OK;
}

//============================
// COOLING SYSTEM end
//============================

//============================
// ACTUATOR begin
//============================

void cooling_actuator_destroy(cooling_actuator_t *actuator)
{
    if (actuator == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid actuator pointer");
        return;
    }

    if (actuator->pwm_channel != COOLING_PWM_INVALID_CH) {
        if (s_pwm_global_initialized) {
            cooling_actuator_pwm_stop(actuator);
        }
        cooling_release_pwm_channel(actuator->pwm_channel);
    }

    actuator->pwm_channel = COOLING_PWM_INVALID_CH;
    actuator->name[0] = '\0';
    actuator->gpio = -1;
    actuator->cool_sys = NULL;
    actuator->duty_cycle = 0;
    actuator->hysteresis_mc = 0;
    actuator->slope_pct_per_degC = 0;
    actuator->min_duty_cycle = 0;
    actuator->initialized = false;
    actuator->active = false;

    ESP_LOGI(COOLING_TAG, "Actuator was destroyed");
}


cooling_actuator_t cooling_actuator_init(const char *name,
                                        int gpio,
                                        uint32_t hysteresis_mc,
                                        uint32_t slope_pct_per_degC,
                                        uint8_t min_duty_cycle)
{
    cooling_actuator_t act = {0};

    if (name == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid actuator name");
        return act;
    }
    if (min_duty_cycle > 100U) {
        ESP_LOGE(COOLING_TAG, "Invalid minimum duty cycle");
        return act;
    }

    strncpy(act.name, name, sizeof(act.name) - 1);
    act.name[sizeof(act.name) - 1] = '\0';

    act.gpio = gpio;
    act.pwm_channel = cooling_allocate_pwm_channel();
    if (act.pwm_channel == COOLING_PWM_INVALID_CH) {
        ESP_LOGE(COOLING_TAG, "No PWM channels available");
        return act;
    }

    act.hysteresis_mc = hysteresis_mc;
    act.slope_pct_per_degC = slope_pct_per_degC;
    act.min_duty_cycle = min_duty_cycle;

    act.duty_cycle = 0;
    act.cool_sys = NULL;
    if (cooling_actuator_pwm_init(&act) != ESP_OK) {
        cooling_release_pwm_channel(act.pwm_channel);
        act.pwm_channel = COOLING_PWM_INVALID_CH;
        ESP_LOGE(COOLING_TAG, "Failed to initialize PWM for actuator");
        return act;
    }

    act.active = false;
    act.initialized = true;

    ESP_LOGI(COOLING_TAG, "Actuator initialized successfully");
    return act;
}

esp_err_t cooling_actuator_set_system(cooling_actuator_t *actuator, cool_sys_t *sys)
{
    esp_err_t err = cooling_actuator_validate(actuator);
    if(err != ESP_OK){return err;}

    err = cooling_system_validate(sys);
    if(err != ESP_OK){return err;}

    actuator->cool_sys = sys;

    ESP_LOGI(COOLING_TAG, "Cooling system assigned to actuator");
    return ESP_OK;
}



//============================
// ACTUATOR end
//============================

//============================
// CONTROL LOGIC begin
//============================
static bool cooling_actuator_apply_hysteresis(cooling_actuator_t *actuator)
{
    bool any_above_threshold = false;
    bool all_below_hyst = true;

    for (size_t i = 0; i < actuator->cool_sys->num_devices; i++) {
        const cooled_dev_t *dev = actuator->cool_sys->devices[i];
        int32_t off_threshold_mc = dev->threshold_temp_mc - (int32_t)actuator->hysteresis_mc;

        if (dev->temperature_mc > dev->threshold_temp_mc) {
            any_above_threshold = true;
        }

        if (dev->temperature_mc >= off_threshold_mc) {
            all_below_hyst = false;
        }
    }

    if (!actuator->active && any_above_threshold) {
        actuator->active = true;
    }
    else if (actuator->active && all_below_hyst) {
        actuator->active = false;
    }

    return actuator->active;
}

static uint8_t cooling_device_compute_duty(const cooled_dev_t *dev, uint32_t slope_pct_per_degC)
{
    if (dev->temperature_mc <= dev->threshold_temp_mc) {
        return 0;
    }

    if (dev->temperature_mc >= dev->max_allowed_temp_mc) {
        return 100;
    }

    int32_t delta_mc = dev->temperature_mc - dev->threshold_temp_mc;

    uint32_t duty = (uint32_t)(((uint64_t)delta_mc * slope_pct_per_degC) / 1000U);

    if (duty > 100U) {
        duty = 100U;
    }

    return (uint8_t)duty;
}

static uint8_t cooling_actuator_compute_duty(const cooling_actuator_t *actuator)
{
    uint8_t max_duty = 0;

    for (size_t i = 0; i < actuator->cool_sys->num_devices; i++) {
        uint8_t duty = cooling_device_compute_duty(
            actuator->cool_sys->devices[i],
            actuator->slope_pct_per_degC
        );

        if (duty > max_duty) {
            max_duty = duty;
        }
    }

    return max_duty;
}

esp_err_t cooling_actuator_update(cooling_actuator_t *actuator)
{
    esp_err_t err = cooling_actuator_validate(actuator);
    if (err != ESP_OK) {
        return err;
    }

    if (actuator->cool_sys == NULL) {
        ESP_LOGE(COOLING_TAG, "Actuator has no cooling system assigned");
        return ESP_ERR_INVALID_STATE;
    }

    // Update active state based on hysteresis
    cooling_actuator_apply_hysteresis(actuator);

    // Compute new duty cycle based on device temperatures and hysteresis decision
    uint8_t new_duty = 0;

    if (actuator->active) {
        new_duty = cooling_actuator_compute_duty(actuator);

        if (new_duty > 0U && new_duty < actuator->min_duty_cycle) {
            new_duty = actuator->min_duty_cycle;
        }
    }

    uint32_t ledc_duty = (new_duty * LEDC_MAX_DUTY) / 100U;

    err = ledc_set_duty(
        COOLING_LEDC_MODE,
        (ledc_channel_t)actuator->pwm_channel,
        ledc_duty
    );
    if (err != ESP_OK) {
        ESP_LOGE(COOLING_TAG, "Failed to set PWM duty: %s", esp_err_to_name(err));
        return err;
    }

    err = ledc_update_duty(
        COOLING_LEDC_MODE,
        (ledc_channel_t)actuator->pwm_channel
    );
    if (err != ESP_OK) {
        ESP_LOGE(COOLING_TAG, "Failed to update PWM duty: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(COOLING_TAG, "Updated actuator '%s' duty cycle from %u to %u", actuator->name, actuator->duty_cycle, new_duty);

    actuator->duty_cycle = new_duty;

    return ESP_OK;
}

//============================
// CONTROL LOGIC end
//============================
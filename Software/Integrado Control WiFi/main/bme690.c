/**
 * @file bme690.c
 * @brief Bosch BME690 Environmental Sensor driver wrapper implementation (I2C Bus 1).
 * 
 * Uses Bosch SensorAPI (bme69x.c / bme69x.h) mapped to ESP-IDF v5 driver/i2c_master.
 */

#include "bme690.h"
#include "app_config.h"
#include "bme69x.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BME690";

static i2c_master_bus_handle_t s_bme_bus_handle = NULL;
static i2c_master_dev_handle_t s_bme_dev_handle = NULL;

static struct bme69x_dev        s_bme_dev;
static struct bme69x_conf       s_bme_conf;
static struct bme69x_heatr_conf s_bme_heatr_conf;
static bool                     s_bme_initialized = false;

/* --- Bosch SensorAPI Adapter Callbacks --- */

static BME69X_INTF_RET_TYPE bme690_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;
    if (!s_bme_dev_handle) return BME69X_E_NULL_PTR;

    esp_err_t err = i2c_master_transmit_receive(s_bme_dev_handle, &reg_addr, 1, reg_data, len, 1000);
    return (err == ESP_OK) ? BME69X_OK : BME69X_E_COM_FAIL;
}

static BME69X_INTF_RET_TYPE bme690_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;
    if (!s_bme_dev_handle) return BME69X_E_NULL_PTR;

    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, len);

    esp_err_t err = i2c_master_transmit(s_bme_dev_handle, buffer, sizeof(buffer), 1000);
    return (err == ESP_OK) ? BME69X_OK : BME69X_E_COM_FAIL;
}

static void bme690_bus_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    uint32_t ms = (period + 999) / 1000;
    if (ms == 0) ms = 1;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* --- Public Driver API --- */

esp_err_t bme690_sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing BME690 on dedicated I2C Port %d (SDA=GPIO%d, SCL=GPIO%d)...",
             BME690_I2C_PORT, BME690_PIN_SDA, BME690_PIN_SCL);

    // 1. Initialize dedicated I2C master bus 1
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = (i2c_port_num_t)BME690_I2C_PORT,
        .sda_io_num        = BME690_PIN_SDA,
        .scl_io_num        = BME690_PIN_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bme_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus %d: %s", BME690_I2C_PORT, esp_err_to_name(ret));
        return ret;
    }

    // 2. Add BME690 device to the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BME690_I2C_ADDR,
        .scl_speed_hz    = BME690_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(s_bme_bus_handle, &dev_cfg, &s_bme_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BME690 device: %s", esp_err_to_name(ret));
        return ret;
    }

    // 3. Populate Bosch sensor API structure
    static uint8_t dev_addr = BME690_I2C_ADDR;
    s_bme_dev.read     = bme690_bus_read;
    s_bme_dev.write    = bme690_bus_write;
    s_bme_dev.delay_us = bme690_bus_delay_us;
    s_bme_dev.intf     = BME69X_I2C_INTF;
    s_bme_dev.intf_ptr = &dev_addr;
    s_bme_dev.amb_temp = BME690_AMB_TEMP_INIT_C;

    int8_t rslt = bme69x_init(&s_bme_dev);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_init failed: %d (check address 0x%02X)", rslt, BME690_I2C_ADDR);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BME690 detected with Chip ID: 0x%02X", s_bme_dev.chip_id);

    // 4. Configure oversampling & IIR filter
    s_bme_conf.filter  = BME69X_FILTER_SIZE_3;
    s_bme_conf.odr     = BME69X_ODR_NONE;
    s_bme_conf.os_hum  = BME69X_OS_2X;
    s_bme_conf.os_pres = BME69X_OS_4X;
    s_bme_conf.os_temp = BME69X_OS_8X;
    rslt = bme69x_set_conf(&s_bme_conf, &s_bme_dev);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_set_conf failed: %d", rslt);
        return ESP_FAIL;
    }

    // 5. Configure gas heater profile
    s_bme_heatr_conf.enable     = BME69X_ENABLE;
    s_bme_heatr_conf.heatr_temp = BME690_GAS_HEATER_TEMP_C;
    s_bme_heatr_conf.heatr_dur  = BME690_GAS_HEATER_DUR_MS;
    rslt = bme69x_set_heatr_conf(BME69X_FORCED_MODE, &s_bme_heatr_conf, &s_bme_dev);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_set_heatr_conf failed: %d", rslt);
        return ESP_FAIL;
    }

    s_bme_initialized = true;
    ESP_LOGI(TAG, "BME690 successfully initialized in Forced Mode.");
    return ESP_OK;
}

esp_err_t bme690_sensor_read(bme690_sensor_data_t *data)
{
    if (!s_bme_initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }

    // Trigger measurement in Forced mode
    int8_t rslt = bme69x_set_op_mode(BME69X_FORCED_MODE, &s_bme_dev);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "Failed to set forced mode: %d", rslt);
        return ESP_FAIL;
    }

    // Calculate delay required for sensor measurement
    uint32_t meas_dur_us = bme69x_get_meas_dur(BME69X_FORCED_MODE, &s_bme_conf, &s_bme_dev) +
                           (BME690_GAS_HEATER_DUR_MS * 1000);
    s_bme_dev.delay_us(meas_dur_us, s_bme_dev.intf_ptr);

    struct bme69x_data raw_data;
    uint8_t n_fields = 0;
    rslt = bme69x_get_data(BME69X_FORCED_MODE, &raw_data, &n_fields, &s_bme_dev);
    if (rslt != BME69X_OK || n_fields == 0) {
        ESP_LOGW(TAG, "No data ready or get_data failed: %d", rslt);
        return ESP_FAIL;
    }

    data->temperature_c      = raw_data.temperature;
    data->humidity_pct       = raw_data.humidity;
    data->pressure_hpa       = raw_data.pressure / 100.0f; // Pa to hPa
    data->gas_resistance_ohm = (uint32_t)raw_data.gas_resistance;
    data->gas_valid          = (raw_data.status & BME69X_GASM_VALID_MSK) ? 1 : 0;
    data->heater_stable      = (raw_data.status & BME69X_HEAT_STAB_MSK)  ? 1 : 0;

    // Dynamically update ambient temperature for next reading compensation
    s_bme_dev.amb_temp = (int8_t)raw_data.temperature;

    return ESP_OK;
}

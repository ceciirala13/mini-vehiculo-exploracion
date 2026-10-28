/**
 * @file arducam_ov2640.c
 * @brief Native ESP-IDF Driver for ArduCAM Mini 2MP Plus (OV2640)
 *
 * ESP-IDF v6 port: SCCB over driver/i2c_master.h, configurable SPI host.
 */

#include "arducam_ov2640.h"
#include "ov2640_regs.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

#include "esp_rom_sys.h"

static const char *TAG = "arducam_ov2640";

static arducam_config_t s_config;
static spi_device_handle_t s_spi_dev = NULL;
static bool s_initialized = false;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_i2c_dev = NULL;

#define SCCB_TIMEOUT_MS 100

// Low level SPI transfer
static uint8_t spi_transfer_byte(uint8_t out_byte) {
    uint8_t rx_data = 0;
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .length = 8,
        .tx_data[0] = out_byte,
    };
    esp_err_t ret = spi_device_polling_transmit(s_spi_dev, &t);
    if (ret == ESP_OK) {
        rx_data = t.rx_data[0];
    }
    return rx_data;
}

static inline void cs_low(void) {
    gpio_set_level(s_config.pin_cs, 0);
}

static inline void cs_high(void) {
    gpio_set_level(s_config.pin_cs, 1);
}

uint8_t arducam_read_reg(uint8_t addr) {
    cs_low();
    spi_transfer_byte(addr & 0x7F); // Read bit 7 is 0
    uint8_t data = spi_transfer_byte(0x00);
    cs_high();
    return data;
}

void arducam_write_reg(uint8_t addr, uint8_t data) {
    cs_low();
    spi_transfer_byte(addr | 0x80); // Write bit 7 is 1
    spi_transfer_byte(data);
    cs_high();
}

uint8_t arducam_get_bit(uint8_t addr, uint8_t bit) {
    uint8_t temp = arducam_read_reg(addr);
    return (temp & bit);
}

void arducam_set_bit(uint8_t addr, uint8_t bit) {
    uint8_t temp = arducam_read_reg(addr);
    arducam_write_reg(addr, temp | bit);
}

void arducam_clear_bit(uint8_t addr, uint8_t bit) {
    uint8_t temp = arducam_read_reg(addr);
    arducam_write_reg(addr, temp & (~bit));
}

esp_err_t arducam_write_sensor_reg(uint8_t reg, uint8_t val) {
    uint8_t write_buf[2] = {reg, val};
    esp_err_t ret = i2c_master_transmit(s_i2c_dev, write_buf, 2, SCCB_TIMEOUT_MS);
    esp_rom_delay_us(1000); // 1ms delay matching Arduino Wire delay(1)
    return ret;
}

esp_err_t arducam_read_sensor_reg(uint8_t reg, uint8_t *val) {
    esp_err_t ret = i2c_master_transmit_receive(s_i2c_dev, &reg, 1, val, 1, SCCB_TIMEOUT_MS);
    esp_rom_delay_us(1000);
    return ret;
}

static esp_err_t write_sensor_regs(const struct sensor_reg *reglist) {
    const struct sensor_reg *curr = reglist;
    while (curr->reg != SENSOR_REG_TERM_8BIT || curr->val != SENSOR_VAL_TERM_8BIT) {
        if (curr->reg == 0xFF && curr->val == 0xFF) {
            break;
        }
        if (curr->reg == 0xFF && curr->val != 0xFF) {
            arducam_write_sensor_reg(0xFF, (uint8_t)curr->val);
            vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            esp_err_t ret = arducam_write_sensor_reg((uint8_t)curr->reg, (uint8_t)curr->val);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed writing reg 0x%02X = 0x%02X", curr->reg, curr->val);
                return ret;
            }
        }
        curr++;
    }
    return ESP_OK;
}

esp_err_t arducam_init(const arducam_config_t *config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(&s_config, config, sizeof(arducam_config_t));

    ESP_LOGI(TAG, "Initializing ArduCAM OV2640 (CS: %d, MOSI: %d, MISO: %d, SCK: %d, SDA: %d, SCL: %d)",
             s_config.pin_cs, s_config.pin_mosi, s_config.pin_miso, s_config.pin_sck,
             s_config.pin_sda, s_config.pin_scl);

    // 1. Initialize CS GPIO
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << s_config.pin_cs),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_conf);
    cs_high();

    // 2. Initialize SPI Bus
    spi_bus_config_t buscfg = {
        .miso_io_num = s_config.pin_miso,
        .mosi_io_num = s_config.pin_mosi,
        .sclk_io_num = s_config.pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8192,
    };
    esp_err_t ret = spi_bus_initialize(s_config.spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = (s_config.spi_freq_hz > 0) ? s_config.spi_freq_hz : 8000000,
        .mode = 0, // SPI mode 0
        .spics_io_num = -1, // Manual CS control
        .queue_size = 7,
    };
    ret = spi_bus_add_device(s_config.spi_host, &devcfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // 3. Initialize I2C (SCCB) bus with the new master driver
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = s_config.i2c_port,
        .sda_io_num = s_config.pin_sda,
        .scl_io_num = s_config.pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OV2640_I2C_ADDR,
        .scl_speed_hz = (s_config.i2c_freq_hz > 0) ? s_config.i2c_freq_hz : 100000,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // 4. Reset CPLD
    arducam_write_reg(0x07, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    arducam_write_reg(0x07, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 5. Test SPI bus
    ret = arducam_test_spi();
    if (ret != ESP_OK) {
        return ret;
    }

    // 6. Test I2C Sensor detection
    uint8_t vid = 0, pid = 0;
    ret = arducam_test_i2c(&vid, &pid);
    if (ret != ESP_OK) {
        return ret;
    }

    // 7. Sensor software reset and init register tables
    arducam_write_sensor_reg(0xFF, 0x01);
    arducam_write_sensor_reg(0x12, 0x80); // Reset OV2640
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Writing OV2640 JPEG initialization tables...");
    write_sensor_regs(OV2640_JPEG_INIT);
    write_sensor_regs(OV2640_YUV422);
    write_sensor_regs(OV2640_JPEG);
    arducam_write_sensor_reg(0xFF, 0x01);
    arducam_write_sensor_reg(0x15, 0x00);
    write_sensor_regs(OV2640_320x240_JPEG); // Default resolution 320x240 (QVGA)

    vTaskDelay(pdMS_TO_TICKS(500));
    arducam_clear_fifo_flag();
    s_initialized = true;
    ESP_LOGI(TAG, "ArduCAM OV2640 successfully initialized!");
    return ESP_OK;
}

esp_err_t arducam_test_spi(void) {
    arducam_write_reg(ARDUCHIP_TEST1, 0x55);
    uint8_t val = arducam_read_reg(ARDUCHIP_TEST1);
    if (val != 0x55) {
        ESP_LOGE(TAG, "SPI Bus Test Failed! Expected 0x55, got 0x%02X", val);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SPI Bus Test OK! (ARDUCHIP_TEST1 = 0x55)");
    return ESP_OK;
}

esp_err_t arducam_test_i2c(uint8_t *out_vid, uint8_t *out_pid) {
    arducam_write_sensor_reg(0xFF, 0x01);
    uint8_t vid = 0, pid = 0;
    arducam_read_sensor_reg(OV2640_CHIPID_HIGH, &vid);
    arducam_read_sensor_reg(OV2640_CHIPID_LOW, &pid);

    if (out_vid) *out_vid = vid;
    if (out_pid) *out_pid = pid;

    if (vid != 0x26 || (pid != 0x41 && pid != 0x42)) {
        ESP_LOGE(TAG, "OV2640 Detection Failed! VID: 0x%02X, PID: 0x%02X (Expected VID: 0x26, PID: 0x41/0x42)", vid, pid);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "OV2640 Detected! VID: 0x%02X, PID: 0x%02X", vid, pid);
    return ESP_OK;
}

esp_err_t arducam_set_resolution(arducam_resolution_t res) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    const struct sensor_reg *table = NULL;
    switch (res) {
        case OV2640_RES_160x120:   table = OV2640_160x120_JPEG; break;
        case OV2640_RES_176x144:   table = OV2640_176x144_JPEG; break;
        case OV2640_RES_320x240:   table = OV2640_320x240_JPEG; break;
        case OV2640_RES_352x288:   table = OV2640_352x288_JPEG; break;
        case OV2640_RES_640x480:   table = OV2640_640x480_JPEG; break;
        case OV2640_RES_800x600:   table = OV2640_800x600_JPEG; break;
        case OV2640_RES_1024x768:  table = OV2640_1024x768_JPEG; break;
        case OV2640_RES_1280x1024: table = OV2640_1280x1024_JPEG; break;
        case OV2640_RES_1600x1200: table = OV2640_1600x1200_JPEG; break;
        default: table = OV2640_320x240_JPEG; break;
    }
    write_sensor_regs(table);
    vTaskDelay(pdMS_TO_TICKS(200));
    arducam_clear_fifo_flag();
    return ESP_OK;
}

void arducam_flush_fifo(void) {
    arducam_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void arducam_clear_fifo_flag(void) {
    arducam_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void arducam_start_capture(void) {
    arducam_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

bool arducam_check_capture_done(void) {
    return (arducam_get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK) != 0);
}

uint32_t arducam_read_fifo_length(void) {
    uint32_t len1 = arducam_read_reg(FIFO_SIZE1);
    uint32_t len2 = arducam_read_reg(FIFO_SIZE2);
    uint32_t len3 = arducam_read_reg(FIFO_SIZE3) & 0x7F;
    uint32_t length = ((len3 << 16) | (len2 << 8) | len1) & 0x07FFFFF;
    return length;
}

esp_err_t arducam_capture_into(uint8_t *buf, size_t buf_size, size_t *out_len) {
    if (!buf || !out_len || buf_size < 16) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_len = 0;
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // 1. Flush FIFO & start capture
    arducam_flush_fifo();
    arducam_clear_fifo_flag();
    arducam_start_capture();

    // 2. Wait for capture done (timeout 2s)
    int timeout = 400;
    while (!arducam_check_capture_done() && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
        timeout--;
    }
    if (timeout <= 0) {
        ESP_LOGE(TAG, "Timeout waiting for capture done");
        arducam_clear_fifo_flag();
        return ESP_ERR_TIMEOUT;
    }

    // 3. Read FIFO length
    uint32_t fifo_len = arducam_read_fifo_length();
    if (fifo_len == 0 || fifo_len >= MAX_FIFO_SIZE) {
        ESP_LOGE(TAG, "Invalid FIFO length: %u bytes", (unsigned int)fifo_len);
        arducam_clear_fifo_flag();
        return ESP_ERR_INVALID_SIZE;
    }
    if (fifo_len > buf_size) {
        ESP_LOGW(TAG, "Frame (%u bytes) larger than buffer (%u bytes), lower the resolution",
                 (unsigned int)fifo_len, (unsigned int)buf_size);
        arducam_clear_fifo_flag();
        return ESP_ERR_INVALID_SIZE;
    }

    // 4. Read all bytes in burst mode
    cs_low();
    spi_transfer_byte(BURST_FIFO_READ);

    size_t remaining = fifo_len;
    uint8_t *ptr = buf;
    while (remaining > 0) {
        size_t chunk = (remaining > 4096) ? 4096 : remaining;
        spi_transaction_t t = {
            .length = chunk * 8,
            .rx_buffer = ptr,
            .tx_buffer = NULL,
        };
        esp_err_t ret = spi_device_polling_transmit(s_spi_dev, &t);
        if (ret != ESP_OK) {
            cs_high();
            arducam_clear_fifo_flag();
            ESP_LOGE(TAG, "SPI burst transmit failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ptr += chunk;
        remaining -= chunk;
    }
    cs_high();
    arducam_clear_fifo_flag();

    // 5. Locate JPEG SOI marker (0xFF 0xD8) and EOI marker (0xFF 0xD9)
    uint8_t *soi = NULL;
    for (uint32_t i = 0; i + 1 < fifo_len; i++) {
        if (buf[i] == 0xFF && buf[i + 1] == 0xD8) {
            soi = &buf[i];
            break;
        }
    }
    if (!soi) {
        ESP_LOGE(TAG, "JPEG SOI (0xFF 0xD8) not found in %u bytes", (unsigned int)fifo_len);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t *eoi = NULL;
    for (uint32_t i = fifo_len - 1; i > (uint32_t)(soi - buf); i--) {
        if (buf[i - 1] == 0xFF && buf[i] == 0xD9) {
            eoi = &buf[i];
            break;
        }
    }

    if (!eoi) {
        ESP_LOGW(TAG, "JPEG EOI (0xFF 0xD9) not found in %u bytes, frame dropped", (unsigned int)fifo_len);
        return ESP_ERR_INVALID_RESPONSE;
    }
    size_t jpeg_len = (size_t)(eoi - soi) + 1;

    // 6. Shift JPEG data to the beginning of the buffer if needed
    if (soi != buf) {
        memmove(buf, soi, jpeg_len);
    }

    *out_len = jpeg_len;
    ESP_LOGD(TAG, "Frame captured: %u bytes (FIFO %u)", (unsigned int)jpeg_len, (unsigned int)fifo_len);
    return ESP_OK;
}

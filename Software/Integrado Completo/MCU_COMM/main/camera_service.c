/**
 * @file camera_service.c
 * @brief Camera capture task and latest-frame double buffer.
 */

#include "camera_service.h"
#include "app_config.h"
#include "arducam_ov2640.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "CAM_SVC";

static bool              s_available = false;
static SemaphoreHandle_t s_frame_mutex = NULL;
static TaskHandle_t      s_task = NULL;

static uint8_t *s_front = NULL;          /* published frame (readers, under mutex) */
static uint8_t *s_back  = NULL;          /* capture target (capture task only)     */
static size_t   s_front_len = 0;
static volatile uint32_t s_frame_id = 0;

static volatile int  s_viewers = 0;
static volatile bool s_snapshot_req = false;
static volatile int  s_res_current = CAM_DEFAULT_RES;
static volatile int  s_res_pending = -1;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static float        s_fps = 0.0f;

static void cam_task(void *arg)
{
    (void)arg;
    int64_t last_frame_us = 0;

    while (1) {
        int pending = s_res_pending;
        if (pending >= 0) {
            s_res_pending = -1;
            if (arducam_set_resolution((arducam_resolution_t)pending) == ESP_OK) {
                s_res_current = pending;
                ESP_LOGI(TAG, "Resolution set to index %d", pending);
            }
        }

        if (s_viewers <= 0 && !s_snapshot_req) {
            s_fps = 0.0f;
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));   /* woken by viewer/snapshot */
            continue;
        }
        s_snapshot_req = false;

        size_t len = 0;
        esp_err_t ret = arducam_capture_into(s_back, CAM_FRAME_BUF_SIZE, &len);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* publish: swap buffers (waits while a reader is sending the front buffer) */
        xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
        uint8_t *tmp = s_front;
        s_front = s_back;
        s_back = tmp;
        s_front_len = len;
        s_frame_id++;
        xSemaphoreGive(s_frame_mutex);

        int64_t now = esp_timer_get_time();
        if (last_frame_us != 0) {
            float inst = 1e6f / (float)(now - last_frame_us);
            s_fps = (s_fps == 0.0f) ? inst : (0.8f * s_fps + 0.2f * inst);
        }
        last_frame_us = now;

        vTaskDelay(pdMS_TO_TICKS(CAM_MIN_FRAME_INTERVAL_MS));
    }
}

esp_err_t camera_service_init(void)
{
    s_frame_mutex = xSemaphoreCreateMutex();
    s_front = heap_caps_malloc(CAM_FRAME_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_back  = heap_caps_malloc(CAM_FRAME_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_frame_mutex || !s_front || !s_back) {
        ESP_LOGE(TAG, "Out of memory for frame buffers (2 x %d bytes)", CAM_FRAME_BUF_SIZE);
        return ESP_ERR_NO_MEM;
    }

    arducam_config_t cfg = {
        .pin_cs      = CAM_PIN_CS,
        .pin_mosi    = CAM_PIN_MOSI,
        .pin_miso    = CAM_PIN_MISO,
        .pin_sck     = CAM_PIN_SCK,
        .pin_sda     = CAM_PIN_SDA,
        .pin_scl     = CAM_PIN_SCL,
        .spi_freq_hz = CAM_SPI_FREQ_HZ,
        .i2c_freq_hz = CAM_I2C_FREQ_HZ,
        .spi_host    = CAM_SPI_HOST,
        .i2c_port    = CAM_I2C_PORT,
    };
    esp_err_t ret = arducam_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ArduCAM init failed (%s). Camera endpoints will return 503.", esp_err_to_name(ret));
        return ret;
    }
    if (CAM_DEFAULT_RES != OV2640_RES_320x240) {
        arducam_set_resolution((arducam_resolution_t)CAM_DEFAULT_RES);
    }
    s_available = true;

    BaseType_t ok = xTaskCreatePinnedToCore(cam_task, "cam", TASK_CAM_STACK, NULL,
                                            TASK_CAM_PRIO, &s_task, TASK_CAM_CORE);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

bool camera_service_available(void)
{
    return s_available;
}

static void wake(void)
{
    if (s_task) xTaskNotifyGive(s_task);
}

void camera_service_viewer_add(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_viewers++;
    taskEXIT_CRITICAL(&s_lock);
    wake();
}

void camera_service_viewer_remove(void)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_viewers > 0) s_viewers--;
    taskEXIT_CRITICAL(&s_lock);
}

void camera_service_request_snapshot(void)
{
    s_snapshot_req = true;
    wake();
}

esp_err_t camera_service_acquire(uint32_t after_id, uint32_t timeout_ms,
                                 const uint8_t **data, size_t *len, uint32_t *frame_id)
{
    if (!s_available) {
        return ESP_ERR_INVALID_STATE;
    }

    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (s_frame_id <= after_id) {
        if (esp_timer_get_time() > deadline) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *data = s_front;
    *len = s_front_len;
    *frame_id = s_frame_id;
    return ESP_OK;
}

void camera_service_release(void)
{
    xSemaphoreGive(s_frame_mutex);
}

esp_err_t camera_service_set_resolution(int res)
{
    if (res < 0 || res > CAM_MAX_RES) {
        return ESP_ERR_INVALID_ARG;
    }
    s_res_pending = res;
    wake();
    return ESP_OK;
}

int camera_service_get_resolution(void)
{
    return (s_res_pending >= 0) ? s_res_pending : s_res_current;
}

float camera_service_get_fps(void)
{
    return s_fps;
}

size_t camera_service_get_last_size(void)
{
    return s_front_len;
}

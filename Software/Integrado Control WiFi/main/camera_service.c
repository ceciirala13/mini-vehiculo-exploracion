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
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "CAM_SVC";

static bool              s_available = false;
static SemaphoreHandle_t s_frame_mutex = NULL;
static TaskHandle_t      s_task = NULL;

static size_t   s_buf_size = 0;          /* bytes per frame buffer */
static int      s_max_res = CAM_MAX_RES_PSRAM;
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
        esp_err_t ret = arducam_capture_into(s_back, s_buf_size, &len);
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

#if CAM_WIRING_TEST
/* =========================================================================
 * Wiring walk test (CAM_WIRING_TEST 1 in app_config.h)
 *
 * Steps through the six camera lines forever. Each line is pulled LOW for
 * CAM_WIRING_STEP_MS, then released (internal pull-up -> HIGH) for the same
 * time, while the other five sit as inputs with pull-ups. Measure each line
 * with a multimeter AT THE CAMERA'S OWN PIN (camera GND as reference):
 * it must follow the log (~0 V on LOW, ~3.3 V on HIGH). Open-drain only, so
 * nothing ever fights the camera if it drives a line.
 *
 * It also reports shorts: if another line reads LOW while only this one is
 * pulled low, the two are bridged somewhere.
 * ========================================================================= */
#define CAM_WIRING_STEP_MS   4000

typedef struct {
    gpio_num_t  pin;
    const char *name;
    const char *where;
} wiring_line_t;

static const wiring_line_t s_lines[] = {
    { CAM_PIN_CS,   "CS",   "header pin 1" },
    { CAM_PIN_MOSI, "MOSI", "header pin 2" },
    { CAM_PIN_MISO, "MISO", "header pin 3" },
    { CAM_PIN_SCK,  "SCK",  "header pin 4" },
    { CAM_PIN_SDA,  "SDA",  "header pin 7" },
    { CAM_PIN_SCL,  "SCL",  "header pin 8" },
};
#define N_LINES (sizeof(s_lines) / sizeof(s_lines[0]))

static void wiring_test_task(void *arg)
{
    (void)arg;
    for (size_t i = 0; i < N_LINES; i++) {
        gpio_reset_pin(s_lines[i].pin);
        gpio_set_direction(s_lines[i].pin, GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_set_pull_mode(s_lines[i].pin, GPIO_PULLUP_ONLY);
        gpio_set_level(s_lines[i].pin, 1);     /* released */
    }
    ESP_LOGW(TAG, "CAMERA WIRING TEST: camera disabled. Measure each line at the camera pin "
                  "(camera GND as reference). Set CAM_WIRING_TEST 0 when done.");

    while (1) {
        for (size_t i = 0; i < N_LINES; i++) {
            const wiring_line_t *l = &s_lines[i];

            gpio_set_level(l->pin, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            char shorts[64] = "";
            for (size_t j = 0; j < N_LINES; j++) {
                if (j != i && gpio_get_level(s_lines[j].pin) == 0) {
                    strlcat(shorts, " ", sizeof(shorts));
                    strlcat(shorts, s_lines[j].name, sizeof(shorts));
                }
            }
            ESP_LOGI(TAG, "[%s] GPIO%d (%s) -> LOW   expect ~0 V at the camera %s pin", l->name, l->pin, l->where, l->name);
            if (shorts[0]) {
                ESP_LOGE(TAG, "[%s] also LOW while only %s is pulled low:%s  -> SHORT / bridged lines?",
                         l->name, l->name, shorts);
            }
            vTaskDelay(pdMS_TO_TICKS(CAM_WIRING_STEP_MS));

            gpio_set_level(l->pin, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            int back = gpio_get_level(l->pin);
            ESP_LOGI(TAG, "[%s] GPIO%d (%s) -> HIGH  expect ~3.3 V at the camera %s pin%s", l->name, l->pin, l->where,
                     l->name, back ? "" : "   !! still reads LOW: something holds this line low");
            vTaskDelay(pdMS_TO_TICKS(CAM_WIRING_STEP_MS));
        }
        ESP_LOGI(TAG, "---- wiring test cycle done, repeating ----");
    }
}
#endif

esp_err_t camera_service_init(void)
{
#if CAM_WIRING_TEST
    xTaskCreatePinnedToCore(wiring_test_task, "cam_wiring", 4096, NULL, TASK_CAM_PRIO, NULL, TASK_CAM_CORE);
    return ESP_ERR_NOT_SUPPORTED;
#endif

    s_frame_mutex = xSemaphoreCreateMutex();

    /* Frame buffers go in PSRAM (the driver reads the FIFO through its own
     * internal bounce buffer). Without PSRAM, fall back to small internal
     * buffers and cap the resolution at VGA. */
    s_buf_size = CAM_FRAME_BUF_SIZE_PSRAM;
    s_max_res  = CAM_MAX_RES_PSRAM;
    s_front = heap_caps_malloc(s_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_back  = heap_caps_malloc(s_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_front || !s_back) {
        heap_caps_free(s_front);
        heap_caps_free(s_back);
        ESP_LOGW(TAG, "No PSRAM for frame buffers, using internal RAM (max VGA)");
        s_buf_size = CAM_FRAME_BUF_SIZE_INTERNAL;
        s_max_res  = CAM_MAX_RES_INTERNAL;
        s_front = heap_caps_malloc(s_buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        s_back  = heap_caps_malloc(s_buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!s_frame_mutex || !s_front || !s_back) {
        ESP_LOGE(TAG, "Out of memory for frame buffers (2 x %u bytes)", (unsigned)s_buf_size);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Frame buffers: 2 x %u KB in %s, max resolution index %d",
             (unsigned)(s_buf_size / 1024), (s_buf_size == CAM_FRAME_BUF_SIZE_PSRAM) ? "PSRAM" : "internal RAM",
             s_max_res);

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
    if (res < 0 || res > s_max_res) {
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

int camera_service_get_max_resolution(void)
{
    return s_max_res;
}

float camera_service_get_fps(void)
{
    return s_fps;
}

size_t camera_service_get_last_size(void)
{
    return s_front_len;
}

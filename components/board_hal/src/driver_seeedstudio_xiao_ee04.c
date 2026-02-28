#include <esp_timer.h>

#include "board_hal.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "epaper.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "soc/soc_caps.h"
#include "soc/usb_serial_jtag_reg.h"

static const char *TAG = "board_hal_ee04";

// Pin Definitions for XIAO EE04
// XIAO ESP32-S3 has precise battery voltage on IO1 (A0) via voltage divider (R11=100k, R10=100k =>
// factor 2)
#define VBAT_ADC_CHANNEL ADC_CHANNEL_0  // GPIO 1 is ADC1_CHANNEL_0
#define VBAT_VOLTAGE_DIVIDER 2.0f
#define VBAT_EN_GPIO GPIO_NUM_6  // GPIO 6 enables battery divider on XIAO EE-series boards

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_cali_enabled = false;

esp_err_t board_hal_init(void)
{
    ESP_LOGI(TAG, "Initializing XIAO EE04 Power HAL (BQ24070)");

    // Initialize SPI bus
    ESP_LOGI(TAG, "Initializing SPI bus...");
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BOARD_HAL_SPI_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = BOARD_HAL_SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 800 * 480 / 2 + 100,  // Sufficient for 7.3" EPD
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // Initialize E-Paper Display Port
    epaper_config_t ep_cfg = {
        .spi_host = SPI2_HOST,
        .pin_cs = BOARD_HAL_EPD_CS_PIN,
        .pin_dc = BOARD_HAL_EPD_DC_PIN,
        .pin_rst = BOARD_HAL_EPD_RST_PIN,
        .pin_busy = BOARD_HAL_EPD_BUSY_PIN,
        .pin_cs1 = -1,
        .pin_enable = BOARD_HAL_EPD_ENABLE_PIN,
    };
    epaper_init(&ep_cfg);

    // Initialize Battery Enable Pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << VBAT_EN_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(VBAT_EN_GPIO, 0);  // Disable by default

    // Initialize ADC for battery voltage
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,  // 11dB or 12dB for full range (up to ~3.3V)
    };
    ret = adc_oneshot_config_channel(adc_handle, VBAT_ADC_CHANNEL, &adc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize ADC calibration
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = VBAT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (cali_ret == ESP_OK) {
        adc_cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration initialization successful");
    } else {
        ESP_LOGW(TAG, "ADC calibration initialization failed: %s", esp_err_to_name(cali_ret));
    }

    return ESP_OK;
}

esp_err_t board_hal_prepare_for_sleep(void)
{
    ESP_LOGI(TAG, "Preparing EE04 for sleep");

    epaper_enter_deepsleep();

    // Disable ADC to save power
    if (adc_handle) {
        if (adc_cali_enabled) {
            adc_cali_delete_scheme_curve_fitting(adc_cali_handle);
            adc_cali_enabled = false;
        }
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
    }
    return ESP_OK;
}

bool board_hal_is_battery_connected(void)
{
    return false;
}

int board_hal_get_battery_voltage(void)
{
    if (!adc_handle)
        return -1;

    // Enable voltage divider
    gpio_set_level(VBAT_EN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(20));  // Give it more time to stabilize

    // Take multiple samples to average out noise
    const int num_samples = 16;
    int32_t total_mv = 0;
    int samples_count = 0;

    for (int i = 0; i < num_samples; i++) {
        int adc_raw;
        if (adc_oneshot_read(adc_handle, VBAT_ADC_CHANNEL, &adc_raw) == ESP_OK) {
            int voltage_mv;
            if (adc_cali_enabled) {
                adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv);
            } else {
                voltage_mv = adc_raw * 3300 / 4095;
            }
            total_mv += voltage_mv;
            samples_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Disable voltage divider to save power
    gpio_set_level(VBAT_EN_GPIO, 0);

    if (samples_count > 0) {
        int avg_mv = (int) (total_mv / samples_count);
        int final_vbat_mv = (int) (avg_mv * VBAT_VOLTAGE_DIVIDER);
        ESP_LOGD(TAG, "Battery voltage: %d mV (avg from %d samples)", final_vbat_mv, samples_count);
        return final_vbat_mv;
    }
    return -1;
}

int board_hal_get_battery_percent(void)
{
    int voltage = board_hal_get_battery_voltage();
    if (voltage < 0)
        return -1;

    // Simple linear approximation for LiPo
    // 4.2V = 100%, 3.3V = 0%
    if (voltage >= 4200)
        return 100;
    if (voltage <= 3300)
        return 0;

    return (voltage - 3300) * 100 / (4200 - 3300);
}

bool board_hal_is_charging(void)
{
    return false;
}

bool board_hal_is_usb_connected(void)
{
    // 1. Check if USB host is connected by monitoring USB Serial JTAG frame counter
    // The frame counter increments every 1ms when connected to a host receiving SOFs.
    static uint32_t last_frame_num = 0;
    uint32_t current_frame_num = REG_READ(USB_SERIAL_JTAG_FRAM_NUM_REG);

    if (current_frame_num != last_frame_num) {
        last_frame_num = current_frame_num;
        return true;
    }

    // 2. Secondary check: Battery voltage threshold
    // When USB is connected, voltage is usually > 4.1V.
    // When on battery, it's usually < 4.1V (except for the first few minutes after full charge).
    int voltage = board_hal_get_battery_voltage();
    bool usb_by_voltage = (voltage > 4120);

    if (usb_by_voltage) {
        ESP_LOGD(TAG, "USB likely connected (voltage: %d mV)", voltage);
    }

    return usb_by_voltage;
}

void board_hal_shutdown(void)
{
    ESP_LOGI(TAG, "Shutdown not supported on BQ24070, entering deep sleep instead");
    board_hal_prepare_for_sleep();
    esp_deep_sleep_start();
}

esp_err_t board_hal_rtc_init(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t board_hal_rtc_get_time(time_t *t)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t board_hal_rtc_set_time(time_t t)
{
    return ESP_ERR_NOT_SUPPORTED;
}

bool board_hal_rtc_is_available(void)
{
    return false;
}

esp_err_t board_hal_get_temperature(float *t)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t board_hal_get_humidity(float *h)
{
    return ESP_ERR_NOT_SUPPORTED;
}

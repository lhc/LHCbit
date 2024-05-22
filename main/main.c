/***********************************************************************************************************************
 *
 * @file    template.c
 * @brief   Standard Leandro C file template
 * @author  Leandro Pereira
 * @date    30/10/2023
 * @company
 *
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * INCLUDES
 **********************************************************************************************************************/

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_INPUT_IO_0) | (1ULL << GPIO_INPUT_IO_1))
#define delay_ms(ms)       vTaskDelay((ms) / portTICK_RATE_MS)

/***********************************************************************************************************************
 * LOCAL COSNTANTS
 **********************************************************************************************************************/
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000    // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      14

#define EXAMPLE_LED_NUMBERS    1
#define EXAMPLE_CHASE_SPEED_MS 10
#define GPIO_INPUT_IO_0        13
#define GPIO_INPUT_IO_1        12
#define ESP_INTR_FLAG_DEFAULT  0

static const char *TAG = "Main";

/***********************************************************************************************************************
 * LOCAL TYPES DECLARATIONS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL VARIABLES
 **********************************************************************************************************************/

static QueueHandle_t gpio_evt_queue = NULL;
static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

/***********************************************************************************************************************
 * LOCAL FUNCTIONS DECLARATION
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL FUNCTIONS
 **********************************************************************************************************************/
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b) {
    h %= 360;    // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i    = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void *arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

/***********************************************************************************************************************
 * PUBLIC FUNCTIONS
 **********************************************************************************************************************/

void app_main(void) {

    uint32_t red       = 0;
    uint32_t green     = 0;
    uint32_t blue      = 0;
    uint16_t hue       = 0;
    uint16_t start_rgb = 0;

    esp_chip_info_t chip_info;    // Struct to store chip information
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s%s, ", CONFIG_IDF_TARGET, chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    gpio_config_t io_conf = {};                             // zero-initialize the config structure
    io_conf.intr_type     = GPIO_INTR_POSEDGE;              // interrupt of rising edge
    io_conf.pin_bit_mask  = GPIO_INPUT_PIN_SEL;             // bit mask of the pins, use GPIO4/5 here
    io_conf.mode          = GPIO_MODE_INPUT;                // set as input mode
    io_conf.pull_up_en    = 1;                              // enable pull-up mode
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));    // create a queue to handle gpio event from isr
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);    // start gpio task
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);                              // install gpio isr service
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler,
                         (void *)GPIO_INPUT_IO_0);    // hook isr handler for specific gpio pin

    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler,
                         (void *)GPIO_INPUT_IO_1);    // hook isr handler for specific gpio pin
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);         // remove isr handler for gpio number.
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler,
                         (void *)GPIO_INPUT_IO_0);    // hook isr handler for specific gpio pin again

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan          = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,     // select source clock
        .gpio_num          = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64,                      // increase the block size can make the LED less flickering
        .resolution_hz     = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4,    // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder          = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,           // no transfer loop
    };

    while (1) {
        printf("Hello World from LHCbit\n");
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        for (int i = 0; i < 3; i++) {
            for (int j = i; j < EXAMPLE_LED_NUMBERS; j += 3) {
                // Build RGB pixels
                hue = j * 360 / EXAMPLE_LED_NUMBERS + start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                led_strip_pixels[j * 3 + 0] = green;
                led_strip_pixels[j * 3 + 1] = blue;
                led_strip_pixels[j * 3 + 2] = red;
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
}

/***********************************************************************************************************************
 * END OF FILE
 **********************************************************************************************************************/

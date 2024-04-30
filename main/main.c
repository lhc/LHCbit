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

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/gpio.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL COSNTANTS
 **********************************************************************************************************************/

#define GPIO_INPUT_IO_0       13
#define GPIO_INPUT_IO_1       12
#define GPIO_INPUT_PIN_SEL    ((1ULL << GPIO_INPUT_IO_0) | (1ULL << GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

/***********************************************************************************************************************
 * LOCAL TYPES DECLARATIONS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL VARIABLES
 **********************************************************************************************************************/

static QueueHandle_t gpio_evt_queue = NULL;

/***********************************************************************************************************************
 * LOCAL FUNCTIONS DECLARATION
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL FUNCTIONS
 **********************************************************************************************************************/

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

    gpio_config_t io_conf = {};                                // zero-initialize the config structure
    io_conf.intr_type     = GPIO_INTR_POSEDGE;                 // interrupt of rising edge
    io_conf.pin_bit_mask  = GPIO_INPUT_PIN_SEL;                // bit mask of the pins, use GPIO4/5 here
    io_conf.mode          = GPIO_MODE_INPUT;                   // set as input mode
    io_conf.pull_up_en    = 1;                                 // enable pull-up mode
    gpio_config(&io_conf);

    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);    // change gpio interrupt type for one pin
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));       // create a queue to handle gpio event from isr
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);    // start gpio task
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);                              // install gpio isr service
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler,
                         (void *)GPIO_INPUT_IO_0);    // hook isr handler for specific gpio pin

    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler,
                         (void *)GPIO_INPUT_IO_1);    // hook isr handler for specific gpio pin
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);         // remove isr handler for gpio number.
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler,
                         (void *)GPIO_INPUT_IO_0);    // hook isr handler for specific gpio pin again

    esp_chip_info_t chip_info;                        // Struct to store chip information
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

    while (1) {
        printf("Hello World from LHCbit\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/***********************************************************************************************************************
 * END OF FILE
 **********************************************************************************************************************/

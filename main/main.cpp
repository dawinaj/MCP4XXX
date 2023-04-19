
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

// #include "lcd.h"

static const char *TAG = "LCD_Test";

extern "C" void app_main(void)
{
	esp_log_level_set("*", ESP_LOG_VERBOSE);
	ESP_LOGI(TAG, "H E N L O B E N C");
	vTaskDelay(pdMS_TO_TICKS(1000));

	disp.clear();

	ESP_LOGI(TAG, "Written");
}

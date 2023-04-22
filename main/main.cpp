#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#include "MCP3XXX.h"

static const char *TAG = "MCP_Test";

void init_spi()
{
	spi_bus_config_t bus_cfg = {
		.mosi_io_num = GPIO_NUM_23,
		.miso_io_num = GPIO_NUM_19,
		.sclk_io_num = GPIO_NUM_18,
		.quadwp_io_num = GPIO_NUM_NC,
		.quadhd_io_num = GPIO_NUM_NC,
		.data4_io_num = GPIO_NUM_NC,
		.data5_io_num = GPIO_NUM_NC,
		.data6_io_num = GPIO_NUM_NC,
		.data7_io_num = GPIO_NUM_NC,
		.max_transfer_sz = 0,
		.flags = SPICOMMON_BUSFLAG_MASTER,
	};

	spi_bus_initialize(SPI3_HOST, &bus_cfg, 0);
}

extern "C" void app_main(void)
{
	esp_log_level_set("*", ESP_LOG_VERBOSE);
	ESP_LOGI(TAG, "H E N L O B E N C");
	vTaskDelay(pdMS_TO_TICKS(1000));

	init_spi();

	MCP3204 adc(SPI3_HOST, GPIO_NUM_5);

	float voltage = adc.get_float_volt(0);

	ESP_LOGI("MCP3XXX", "Voltage: %f V", voltage);
}

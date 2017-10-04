#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "sht31.h"

#include "sdkconfig.h"

static char tag[] = "neosarchizo";

rgbVal *pixels;

void app_main() {
	ESP_LOGD(tag, "sht31 initialize");
	sht31_init();

	while (1) {
		// TODO something
		if (sht31_readTempHum()) {
			float h = sht31_readHumidity();
			float t = sht31_readTemperature();
			ESP_LOGD(tag, "H, T : %.f, %.f", h, t);
		} else {
			ESP_LOGD(tag, "sht31_readTempHum : failed");
		}

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
} // task_hmc5883l

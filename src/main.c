#include <boards/pico2_w.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include "bus/i2c.h"
#include "battery/battery.h"
#include "drivers/wifi/wifi.h"
#include "mongoose.h"

#include "config.h"

#include "tasks/wifi_server_task.h"
#include "tasks/sensors_task.h"
#include "tasks/pid_task.h"
#include "tasks/shared_data.h"
// #include "tasks/.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>


int main() {
	stdio_init_all();
	const char *rtos_name;
    rtos_name = "FreeRTOS SMP";
	sleep_ms(500);

	printf("Calibrating Sensors....\n");
	sensors_init();

	initSemaphore();
	TaskHandle_t xHandle;
    xTaskCreate(wifi_server_task, "server_task", 4096, 0,configMAX_PRIORITIES - 3, &xHandle);
	vTaskCoreAffinitySet(xHandle, (2 << 0));

	sensors_calibrate();
  	xTaskCreate(sensors_task, "sensors_task", 2048, 0, configMAX_PRIORITIES - 2, &xHandle);
	vTaskCoreAffinitySet(xHandle, (1 << 0));
  	xTaskCreate(pid_task, "pid_task", 2048, 0, configMAX_PRIORITIES - 1, &xHandle);
	vTaskCoreAffinitySet(xHandle, (1 << 0));

    //xTaskCreate(battery_task, "bat_task", 2048, 0,configMAX_PRIORITIES - 5, NULL);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    while (true);

	return 0;
}

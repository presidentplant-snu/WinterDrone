#include <boards/pico2_w.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "bus/i2c.h"
#include "pico/binary_info.h"
#include "drivers/sensors/bmp280.h"
#include "drivers/wifi/wifi.h"
#include "mongoose.h"
#include "motor/motor.h"

#include "FreeRTOS.h"
#include "task.h"

void main_task(__unused void *params) {
	init_wifi("PicoTesting","1234567890",true);
	sleep_ms(1000);
	mongoose();
    // while(true) {
    //     // not much to do as LED is in another task, and we're using RAW (callback) lwIP API
	// 	printf("test\n");
    //     vTaskDelay(1000);
    // }
	deinit_wifi();
}

int main() {
	stdio_init_all();
	    const char *rtos_name;
    rtos_name = "FreeRTOS SMP";

    printf("Starting %s on core 1:\n", rtos_name);

    xTaskCreate(main_task, "main_task", 4096, 0,configMAX_PRIORITIES - 1, NULL);
	// xTaskCreate(mongoose, "mongoose", 2048, 0, configMAX_PRIORITIES - 1, NULL);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
    while (true);


	return 0;
}

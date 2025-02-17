#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "motor/motor.h"

int main() {
	stdio_init_all();
	uint8_t pins[4]= {10, 11, 12, 13};
	initMotors(pins);
    printf("Hello, world!\n");
	int speeds[4]={10, 11,0,0};
	
	while(1){
		*speeds=0;
		*(speeds+1)=0;
		setSpeeds(speeds);
		printf("Testing1");
		sleep_ms	(1000);
		*speeds=100;
		*(speeds+1)=100;
		setSpeeds(speeds);
		printf("Testing2");
		sleep_ms	(1000);
	}
	return 0;
}

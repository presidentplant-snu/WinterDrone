#include <stdio.h>
#include "pico/stdlib.h"
// #include "pico/time.h"
// #include "motor/motor.h"

int main() {
	stdio_init_all();

	uint8_t pins[4]= {10, 11, 12, 13};
	initMotors(pins);
    printf("Hello, world!\n");
	uint16_t speeds[4]={0, 0,0,0};
	uint16_t speed=0;
	while(1){
		scanf("%hu",&speed);
		speeds[0]=speed;
		speeds[1]=speed;
	
		setSpeeds(speeds);
		printf("Testing1\n");
		sleep_ms(1000);
	}
	return 0;
}

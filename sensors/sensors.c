#include "sensors.h"

int getMPU(int* accel, int* gyro) {
	int a[2] = {1,2};
	
	for(int i=0; i<3; i++){
		*(accel+i)=1;
		*(gyro+i)=1;
	}
	return 0;
}

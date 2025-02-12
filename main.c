#include <stdio.h>
#include "pico/stdlib.h"

#include "sensors.h"
#include "motor.h"

int main() {
    stdio_init_all();

    initI2C();
    printf("Hello, world!\n");

    return 0;
}

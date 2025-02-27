#include "battery.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"


/**
 * Initialize the ADC for battery monitoring
 */
void battery_init(void) {
    // Initialize ADC
    adc_init();
    
    // Configure GPIO pin for ADC
    adc_gpio_init(BATTERY_ADC_PIN);
    
    // Select ADC input
    adc_select_input(BATTERY_ADC_INPUT);
    
    printf("Battery monitoring initialized on GPIO pin %d\n", BATTERY_ADC_PIN);
}

/**
 * Read the battery voltage
 * @return Current battery voltage in volts
 */
float battery_read_voltage(void) {
    // Read ADC
    uint16_t raw_reading = adc_read();
    
    // Convert to voltage (taking into account voltage divider if used)
    float voltage = (float)raw_reading / ADC_RANGE * ADC_VREF * VOLTAGE_DIVIDER_RATIO;
    
    return voltage;
}

/**
 * Print battery voltage information
 */
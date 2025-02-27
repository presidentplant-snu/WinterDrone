#ifndef BATTERY_H
#define BATTERY_H

// Configuration
#define BATTERY_ADC_PIN 26       // ADC0 - GPIO26
#define BATTERY_ADC_INPUT 0      // Input channel for ADC0
#define ADC_VREF 3.3f            // Reference voltage
#define ADC_RANGE (1 << 12)      // 12-bit ADC
#define VOLTAGE_DIVIDER_RATIO 2.0f  // If using voltage divider, set ratio here

// Function prototypes
void battery_init(void);
float battery_read_voltage(void);

#endif // BATTERY_H
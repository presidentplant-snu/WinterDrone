#include "pico/stdlib.h"
#include "hardware/pwm.h"

uint8_t motorPins[4];  // Store motor GPIO pins
uint8_t pwmSlices[4];  // Store corresponding PWM slice numbers

void initMotors(uint8_t pins[4]) {
    for (int i = 0; i < 4; i++) {
        motorPins[i] = pins[i];  // Store pin number
        
        // Set the pin as PWM output
        gpio_set_function(motorPins[i], GPIO_FUNC_PWM);
        
        // Get PWM slice associated with the pin
        pwmSlices[i] = pwm_gpio_to_slice_num(motorPins[i]);
        
        // Set PWM frequency (e.g., 1 kHz)
        pwm_set_wrap(pwmSlices[i], 65535);
        
        // Enable PWM output
        pwm_set_enabled(pwmSlices[i], true);
    }
}

int setSpeeds(uint16_t speeds[4]) {
    for (int i = 0; i < 4; i++) {
        int speed = speeds[i];

        // Ensure speed is between 0-100% and scale to 16-bit PWM (0-65535)
        uint16_t duty = (speed * 65535) / 100;

        // Set PWM duty cycle
        pwm_set_gpio_level(10, duty);
    }
    return 0;
}

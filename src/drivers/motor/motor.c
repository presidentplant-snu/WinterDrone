#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "config.h"

uint8_t motorPins[4];  // Store motor GPIO pins
uint8_t pwmSlices[4];  // Store corresponding PWM slice numbers

void initMotors(const uint8_t pins[4]) {
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

int setSpeeds(const float speeds[4]) {
    for (int i = 0; i < 4; i++) {
        float speed = speeds[i];

		if(speed > 1.0f) speed = 1.0f;

        // Ensure speed is between 0-100% and scale to 16-bit PWM (0-65535)
        uint16_t duty = (uint16_t) (65535.0f* MAX_THRUST * (float)speed);

        // Set PWM duty cycle
        pwm_set_gpio_level(motorPins[i], duty);
    }
    return 0;
}

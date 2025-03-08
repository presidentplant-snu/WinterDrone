/**
 * pid.c - General PID Controller Implementation
 */

#include "pid.h"
#include <stdbool.h>
#include <stddef.h>

/**
 * Initialize a PID controller with default values
 */
bool PID_Init(PID_Controller *pid) {
    if (pid == NULL) {
        return false;
    }
    
    // Initialize gains
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->kf = 0.0f;
    
    // Initialize controller state
    pid->setpoint = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_time = 0.0f;
    pid->integral = 0.0f;
    
    // Initialize output limits
    pid->output_min = -1.0f;
    pid->output_max = 1.0f;
    
    // Initialize anti-windup
    pid->anti_windup = false;
    pid->windup_limit = 1.0f;
    
    // Mark as initialized
    pid->initialized = true;
    
    return true;
}

/**
 * Configure PID controller parameters
 */
bool PID_Config(PID_Controller *pid, float kp, float ki, float kd, float kf,
                float output_min, float output_max) {
    if (pid == NULL || !pid->initialized) {
        return false;
    }
    
    // Configure gains
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;
    
    // Configure output limits
    pid->output_min = output_min;
    pid->output_max = output_max;
    
    return true;
}

/**
 * Configure anti-windup for the PID controller
 */
bool PID_ConfigAntiWindup(PID_Controller *pid, bool enable, float limit) {
    if (pid == NULL || !pid->initialized) {
        return false;
    }
    
    pid->anti_windup = enable;
    
    if (limit > 0.0f) {
        pid->windup_limit = limit;
    }
    
    return true;
}

/**
 * Set the desired setpoint for the PID controller
 */
bool PID_SetSetpoint(PID_Controller *pid, float setpoint) {
    if (pid == NULL || !pid->initialized) {
        return false;
    }
    
    pid->setpoint = setpoint;
    return true;
}

/**
 * Reset the PID controller state (integral and previous error)
 */
bool PID_Reset(PID_Controller *pid) {
    if (pid == NULL || !pid->initialized) {
        return false;
    }
    
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    
    return true;
}

/**
 * Update the PID controller and calculate the new output
 */
float PID_Update(PID_Controller *pid, float measurement, float current_time, float feedforward) {
    if (pid == NULL || !pid->initialized) {
        return 0.0f;
    }
    
    // Calculate time delta
    float dt;
    if (pid->prev_time <= 0.0f) {
        // First call, use a small default dt
        dt = 0.01f;
    } else {
        dt = current_time - pid->prev_time;
        // Check for negative or zero dt
        if (dt <= 0.0f) {
            dt = 0.01f; // Use default if time went backwards or didn't change
        }
    }
    
    // Save current time for next iteration
    pid->prev_time = current_time;
    
    // Calculate error
    float error = pid->setpoint - measurement;
    
    // Calculate proportional term
    float p_term = pid->kp * error;
    
    // Calculate integral term
    pid->integral += error * dt;
    
    // Apply anti-windup if enabled
    if (pid->anti_windup) {
        if (pid->integral > pid->windup_limit) {
            pid->integral = pid->windup_limit;
        } else if (pid->integral < -pid->windup_limit) {
            pid->integral = -pid->windup_limit;
        }
    }
    
    float i_term = pid->ki * pid->integral;
    
    // Calculate derivative term (with error derivative)
    float derivative;
    if (dt > 0.0f) {
        derivative = (error - pid->prev_error) / dt;
    } else {
        derivative = 0.0f;
    }
    float d_term = pid->kd * derivative;
    
    // Calculate feedforward term
    float ff_term = pid->kf * feedforward;
    
    // Save current error for next iteration
    pid->prev_error = error;
    
    // Calculate output
    float output = p_term + i_term + d_term + ff_term;
    
    // Apply output limits
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    return output;
}

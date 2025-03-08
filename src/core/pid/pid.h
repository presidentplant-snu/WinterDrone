/**
 * pid.h - General PID Controller Module
 * 
 * This module implements a general-purpose PID controller that can be used
 * for various control applications such as roll and pitch control.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * PID controller structure
 */
typedef struct {
    // Controller gains
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float kf;           // Feedforward gain
    
    // Controller state
    float setpoint;     // Desired value
    float prev_error;   // Previous error for derivative calculation
    float prev_time;    // Time of previous update
    float integral;     // Accumulated error for integral calculation
    
    // Output limits
    float output_min;   // Minimum output value
    float output_max;   // Maximum output value

    // Anti-windup parameters
    bool anti_windup;   // Enable anti-windup
    float windup_limit; // Integral windup limit
    
    // Controller flags
    bool initialized;   // Whether the controller has been initialized
} PID_Controller;

/**
 * Initialize a PID controller with default values
 * 
 * @param pid Pointer to PID controller structure
 * @return true if initialization was successful, false otherwise
 */
bool PID_Init(PID_Controller *pid);

/**
 * Configure PID controller parameters
 * 
 * @param pid Pointer to PID controller structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param kf Feedforward gain
 * @param output_min Minimum output value
 * @param output_max Maximum output value
 * @return true if configuration was successful, false otherwise
 */
bool PID_Config(PID_Controller *pid, float kp, float ki, float kd, float kf,
                float output_min, float output_max);

/**
 * Configure anti-windup for the PID controller
 * 
 * @param pid Pointer to PID controller structure
 * @param enable Enable or disable anti-windup
 * @param limit Integral windup limit
 * @return true if configuration was successful, false otherwise
 */
bool PID_ConfigAntiWindup(PID_Controller *pid, bool enable, float limit);

/**
 * Set the desired setpoint for the PID controller
 * 
 * @param pid Pointer to PID controller structure
 * @param setpoint Desired setpoint value
 * @return true if setting was successful, false otherwise
 */
bool PID_SetSetpoint(PID_Controller *pid, float setpoint);

/**
 * Reset the PID controller state (integral and previous error)
 * 
 * @param pid Pointer to PID controller structure
 * @return true if reset was successful, false otherwise
 */
bool PID_Reset(PID_Controller *pid);

/**
 * Update the PID controller and calculate the new output
 * 
 * @param pid Pointer to PID controller structure
 * @param measurement Current measured value
 * @param current_time Current time in seconds
 * @param feedforward Feedforward input (optional, set to 0.0f if not used)
 * @return Calculated control output
 */
float PID_Update(PID_Controller *pid, float measurement, float current_time, float feedforward);


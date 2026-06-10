#ifndef __MOTOR_ENCODER_H__
#define __MOTOR_ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

// Read wheel encoder values (populated from serial packets)
void read_encoder_values(int *left_encoder_value, int *right_encoder_value);

// Send velocity commands to STM32 as PWM values via serial.
// left_wheel_command and right_wheel_command are in rad/s.
// Internally maps rad/s → PWM (0–999) using MAX_RAD_S constant.
void set_motor_speeds(double left_wheel_command, double right_wheel_command, int serial_fd_);

// Signal handler for safe shutdown
void handler(int signo);

// Global encoder state (updated from serial packets)
extern int left_wheel_pulse_count;
extern int right_wheel_pulse_count;
extern int left_wheel_direction;
extern int right_wheel_direction;

#ifdef __cplusplus
}
#endif

#endif  // __MOTOR_ENCODER_H__
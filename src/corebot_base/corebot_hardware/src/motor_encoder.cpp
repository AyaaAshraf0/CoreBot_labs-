// motor_encoder.cpp
//
// Encoder state and motor command interface for CoreBot.
// Architecture: STM32 handles encoder counting and PWM generation.
// RPi receives absolute encoder ticks via binary serial frames and
// sends PWM commands as "M<left>,<right>\n" ASCII strings.
//
// Velocity → PWM mapping:
//   PWM = clamp(rad/s / MAX_RAD_S, -1, 1) × PWM_MAX
//   MAX_RAD_S must be measured empirically at full PWM on your hardware.
//   STM32 MOTOR_PWM_PERIOD = 1000, ARR = 999 → PWM range 0–999.

#include "corebot_hardware/motor_encoder.hpp"

#include <math.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>

// ── Global encoder state ──────────────────────────────────────────────────
int left_wheel_pulse_count  = 0;
int right_wheel_pulse_count = 0;
int left_wheel_direction    = 1;
int right_wheel_direction   = 1;

// ── Encoder read (informational — real state lives in Wheel objects) ──────
void read_encoder_values(int *left_encoder_value, int *right_encoder_value)
{
    *left_encoder_value  = left_wheel_pulse_count;
    *right_encoder_value = right_wheel_pulse_count;
}

// ── Motor command ─────────────────────────────────────────────────────────
void set_motor_speeds(double left_cmd_rads, double right_cmd_rads, int serial_fd_)
{
    if (serial_fd_ < 0)
    {
        return;
    }

    // STM32 PWM range: 0–999 (MOTOR_PWM_PERIOD=1000, ARR=999, 1 kHz)
    // Sign is conveyed by the M protocol: positive → CW, negative → CCW.
    //
    // MAX_RAD_S: measure by running at full PWM (999) and reading
    // joint_states velocity at steady state. Update this constant once known.
    static constexpr double MAX_RAD_S = 10.0;   // ← TUNE after measurement
    static constexpr double PWM_MAX   = 999.0;  // ← matches STM32 ARR value

    double l = std::clamp(left_cmd_rads  / MAX_RAD_S, -1.0, 1.0) * PWM_MAX;
    double r = std::clamp(right_cmd_rads / MAX_RAD_S, -1.0, 1.0) * PWM_MAX;

    char buf[32];
    int len = snprintf(buf, sizeof(buf), "M%d,%d\n",
                       static_cast<int>(l),
                       static_cast<int>(r));
    ::write(serial_fd_, buf, len);
}

// ── Signal handler ────────────────────────────────────────────────────────
void handler(int signo)
{
    (void)signo;
    exit(0);
}

#ifndef _COREBOT_BASE__WHEEL_H_
#define _COREBOT_BASE__WHEEL_H_

#include <string>
#include <cmath>

class Wheel
{   
  public:
    std::string name = "";
    int encoder_ticks = 0;
    double command = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double rads_per_tick = 0.0;
    
    int16_t prev_encoder = 0;   // last read MCU value
    int32_t total_ticks = 0;    // cumulative ticks

    Wheel() = default;

    Wheel(const std::string &wheel_name, int ticks_per_rev);

    void setup(const std::string &wheel_name, int ticks_per_rev);

    double calculate_encoder_angle();

    int32_t compute_encoder_delta(int16_t current);
};

#endif
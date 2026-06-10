#include "corebot_hardware/wheel.hpp"

Wheel::Wheel(const std::string &wheel_name, int ticks_per_rev)
{
    setup(wheel_name, ticks_per_rev);
}

void Wheel::setup(const std::string &wheel_name, int ticks_per_rev)
{
    name = wheel_name;
    rads_per_tick = (2*M_PI)/ticks_per_rev;
}

double Wheel::calculate_encoder_angle()
{
    return total_ticks * rads_per_tick;
}

int32_t Wheel::compute_encoder_delta(int16_t current)
{
    int32_t delta = static_cast<int32_t>(current - prev_encoder);

    // Handle 16-bit wrap-around
    if (delta > 32767)  delta -= 65536;
    if (delta < -32768) delta += 65536;

    prev_encoder += delta; // update last read (or prev_encoder = current)
    total_ticks += delta;  // accumulate total
    return delta;
}
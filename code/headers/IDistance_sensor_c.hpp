#pragma once

#include <frame_types.hpp>

class IDistance_sensor_c
{
public:
    virtual void fill_distance_frame(frame_distance_s frame) = 0;
};

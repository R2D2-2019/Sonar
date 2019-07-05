#pragma once

#include <base_module.hpp>
#include <frame_types.hpp>

namespace r2d2::distance
{
    class IDistance_sensor_c
    {
    public:
        virtual void fill_distance_frame(frame_distance_s & frame) = 0;
    };
} // R2D2::Distance
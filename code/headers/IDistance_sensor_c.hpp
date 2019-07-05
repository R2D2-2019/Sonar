#pragma once

#include <frame_types.hpp>

namespace R2D2::Distance
{
    class IDistance_sensor_c
    {
    public:
        virtual void fill_distance_frame(frame_distance_s frame) = 0;
    };
} // R2D2::Distance

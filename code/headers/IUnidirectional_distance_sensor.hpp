#pragma once

#include <hwlib.hpp>

namespace r2d2::distance
{
            /**
         * \brief 
         * This is an interface for the sonar
         *
         * \details
         * This interface is used to get the distance from a single sonar or a sonar array.
         */
    class IUnidirectional_distance_sensor_c {
    public:
        virtual uint16_t  get_distance() = 0;
    };
} // R2D2::Distance

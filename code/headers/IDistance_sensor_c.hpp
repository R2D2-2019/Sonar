#pragma once

#include <base_module.hpp>
#include <frame_types.hpp>

namespace r2d2::distance
{
    /// \brief
    /// An interface for classes that represent a distance sensor.
    /// \detail
    /// This interface is used by module_c as the distance sensor.
    class IDistance_sensor_c
    {
    public:
        /// \brief
        /// Function that fills a frame with the measured distances.
        /// \param frame A reference to the frame that will be filled.
        virtual void fill_distance_frame(frame_distance_s & frame) = 0;
    };
} // R2D2::Distance
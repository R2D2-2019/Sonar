#pragma once

#include "IDistance_sensor_c.hpp"
#include "lidar.hpp"
#include "hwlib.hpp"

namespace r2d2::distance
{
    /// \brief
    /// A class that uses a lidar_c to measure distances and convert them to usefull data.
    /// \details
    /// The lidar_distance_sensor_c is a class that implements the IDistance_sensor_c interface
    /// and therefore has a fill_distance_frame(frame_distance_s frame) function.
    /// In this function the class will instruct the lidar to measure the distance in 360 degrees,
    /// the class will use the data from the lidar to fill a frame_distance_s frame.
    class lidar_distance_sensor_c : public IDistance_sensor_c {
    private:
        lidar_c& lidar;
        uint16_t smallest_value;
        uint16_t current_value;
        bool smallest_value_set_first_time;
        uint16_t pov;
    public:
        /// \brief
        /// The constructor for the lidar_distance_sensor_c class.
        /// \param _lidar A reference to the lidar that will be used.
        /// \param _pov The angle at which the lidar is placed.
        lidar_distance_sensor_c(lidar_c& _lidar, uint16_t _pov):
            lidar(_lidar), smallest_value(0),
            current_value(0), smallest_value_set_first_time(false),
            pov(_pov){}

        /// \brief
        /// In this function the class will instruct the lidar to measure the distance in 360 degrees,
        /// the class will use the data from the lidar to fill a frame_distance_s frame.
        void fill_distance_frame(frame_distance_s & frame) override;
    };
}

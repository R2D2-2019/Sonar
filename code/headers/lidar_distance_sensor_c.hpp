#pragma once

#include "IDistance_sensor_c.hpp"
#include "lidar.hpp"
#include "hwlib.hpp"

namespace r2d2::distance
{
    class lidar_distance_sensor_c : public IDistance_sensor_c {
    private:
        lidar_c& lidar;
        uint16_t smallest_value;
        uint16_t current_value;
        bool smallest_value_set_first_time;
        uint16_t pov;
    public:
        lidar_distance_sensor_c(lidar_c& _lidar, uint16_t _pov):
            lidar(_lidar), smallest_value(0),
            current_value(0), smallest_value_set_first_time(false),
            pov(_pov){}

        void fill_distance_frame(frame_distance_s & frame) override;
    };
}

#pragma once

#include <hwlib.hpp>
#include "IDistance_sensor_c.hpp"
#include "IUnidirectional_distance_sensor.hpp"

namespace r2d2::distance{
     /**
         * \brief 
         * this class builds a struct
         *
         * \details
         * This class builds a struct from sonar data that gets send on the can-bus.
         */
    class single_direction_struct_builder_c : public IDistance_sensor_c {
    private:
        IUnidirectional_distance_sensor_c & distance_sensor;
        uint16_t pov;
    public:
        single_direction_struct_builder_c(IUnidirectional_distance_sensor_c & distance_sensor, uint16_t pov = 0):
        distance_sensor(distance_sensor),
        pov(pov)
        {}
        void fill_distance_frame(frame_distance_s & frame) override;

    };
}

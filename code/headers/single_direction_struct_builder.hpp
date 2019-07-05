#include <hwlib.hpp>
#include "IUnidirectional_distance_sensor.hpp"

namespace r2d2::distance{
    class single_direction_struct_builder {
    private:
        IUnidirectional_distance_sensor_c & distance_sensor;
        uint16_t pov;
    public:
        single_direction_struct_builder(IUnidirectional_distance_sensor_c & distance_sensor, uint16_t pov = 0):
        distance_sensor(distance_sensor),
        pov(pov)
        {}
        void get_distance_frame(frame_distance_s & frame);

    };
}


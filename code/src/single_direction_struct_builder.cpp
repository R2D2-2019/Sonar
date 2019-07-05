#include "single_direction_struct_builder.hpp"

void r2d2::distance::single_direction_struct_builder_c::fill_distance_frame(frame_distance_s & frame) {
    frame.sensor_type = 1;
    frame.sensor_pov = pov;
    frame.count = 1;
    frame.angles[0] = 0;
    frame.values[0] = distance_sensor.get_distance();
}
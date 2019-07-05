#include "lidar_distance_sensor_c.hpp"

namespace r2d2::distance
{
    void lidar_distance_sensor_c::fill_distance_frame(frame_distance_s& frame){

        frame.sensor_type = 0; // 0 = lidar, 1 = sonar
        frame.sensor_pov = 0;
        frame.count = 45;

        // 16 packets for one 360 degree measurement
        for (int i = 0; i < 16; i++) {
            lidar.receive_packet();
        }
        
        // save a measurement every 8 degrees.
        for (int i = 0; i < 45; i++) {
            current_value = (lidar.measurements[i*16].distance_value / 4); // divide the distance value by four to get the distance in mm

            frame.angles[i] = i*8;
            frame.values[i] = current_value;
        }
    }
} // r2d2::distance

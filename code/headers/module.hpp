#pragma once

#include <base_module.hpp>
#include "IDistance_sensor_c.hpp"


namespace r2d2::distance {
    class module_c : public base_module_c {
    private:
        IDistance_sensor_c & distance_sensor;
        frame_distance_s distance_frame;
    public:
        module_c(base_comm_c &comm, IDistance_sensor_c & distance_sensor)
            : base_module_c(comm), distance_sensor(distance_sensor), distance_frame() {

            comm.listen_for_frames({r2d2::frame_type::DISTANCE});
        }

        void process() override {
            while (comm.has_data()) {
                auto frame = comm.get_data();
                if (!frame.request) {
                    continue;
                }
                distance_sensor.fill_distance_frame(distance_frame);
                comm.send(distance_frame); 
            }
        }
    };
} // namespace r2d2::distance
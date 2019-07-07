#pragma once

#include <base_module.hpp>
#include "IDistance_sensor_c.hpp"


namespace r2d2::distance {
    /// \brief
    /// The module class of the distance module.
    /// \details
    /// In the process function, this module checks if a request of type DISTANCE is received,
    /// if such a request is received the module will use its distance sensor to measure the distance
    /// in as many directions and send it over the canbus in a frame_distance_s frame.
    /// The distance sensor this module uses can be easily replaced with another distance sensor,
    /// as this class has a reference to an IDistance_sensor_c for its distance sensor, which is an interface.
    class module_c : public base_module_c {
    private:
        IDistance_sensor_c & distance_sensor;
        frame_distance_s distance_frame;
    public:
        /// \brief
        /// The constructor of the distance module_c class.
        /// \param comm The comm that will be used to communicate on the canbus.
        /// \param distance_sensor The distance sensor that will be used.
        module_c(base_comm_c &comm, IDistance_sensor_c & distance_sensor)
            : base_module_c(comm), distance_sensor(distance_sensor), distance_frame() {

            comm.listen_for_frames({r2d2::frame_type::DISTANCE});
        }

        /// \brief
        /// The process function of this module.
        /// \details
        /// In this function the module checks for a DISTANCE request on the canbus.
        /// If such a request has been received, the module will use the distance sensor
        /// to measure the distance and send it on the canbus in a frame_distance_s frame.
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
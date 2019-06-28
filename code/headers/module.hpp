#pragma once

#include <base_module.hpp>
#include <lidar.hpp>

// This is the main module class that measures the smallest distance and sends
// it on the canbus
namespace r2d2::distance {

    /// \brief The main class for the distance module.
    /// \detail
    /// This class uses a lidar to measure the distance between the lidar and
    /// the closest object. When a distance request has been received this class
    /// sends the smallest detected distance over the can-bus.
    class module_c : public base_module_c {
    private:
        lidar_c& lidar;

        uint16_t smallest_value;
        uint16_t current_value;
        bool smallest_value_set_first_time = false;

    public:
        /// \brief r2d2::distance::module_c constructor
        /// \detail Constructs a r2d2::distance::module_c object that will use
        /// the given comm and lidar.
        module_c(base_comm_c &comm, lidar_c &lidar)
            : base_module_c(comm), lidar(lidar) {

            comm.listen_for_frames({r2d2::frame_type::DISTANCE});
        }

        /// \brief This modules process function that allows the module to
        /// process (input)data. \detail In this function the module first
        /// checks for distance request frames. If a distance request frame has
        /// been received, the module will update its distance data with input
        /// from the lidar. It then searches for the smallest distance and sends
        /// this distance over the can-bus in a frame_distance_s packet.
        void process() override {
            while (comm.has_data()) {
                auto frame = comm.get_data();
                // Only handle requests

                if (!frame.request) {
                    continue;
                }

                // Frame for the smallest distance measured
                frame_distance_s smallest_distance;

                for (int count = 0; count < 21; count++) {
                    // 16 packets for one 360 degree measurement

                    for (int count = 0; count < 16; count++) {
                        lidar.receive_packet();
                    }
                    // Every 100 360 degree measurements we print the distance
                    // for every half degree.
                    if (count == 20) {
                        for (int i = 0; i < 720; i++) {

                            current_value =
                                (lidar.measurements[i].distance_value / 4);

                            // Set the smallest distance that is not 0 for the
                            // first time
                            if ((current_value != 0 &&
                                 smallest_value_set_first_time == false)) {
                                smallest_value = current_value;
                                smallest_value_set_first_time = true;
                            }

                            if ((current_value != 0 &&
                                 current_value < smallest_value)) {
                                smallest_value = current_value;
                            }
                        }
                    }
                    // The smallest measured value(besides 0) will go into the
                    // frame and will be send
                    smallest_distance.mm = smallest_value;
                    comm.send(smallest_distance);
                }
            }
        }
    };
} // namespace r2d2::distance
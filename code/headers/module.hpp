#pragma once

#include <base_module.hpp>
#include <lidar.hpp>
#include <hardware_usart.hpp>

// This is the main module class that measures 
namespace r2d2::distance {
    class module_c : public base_module_c {
        private:
            lidar_c lidar;

            uint16_t smallest_value;
            uint16_t current_value;
            bool smallest_value_set_first_time = false;

        public:

            module_c(base_comm_c &comm, lidar_c &lidar)
                : base_module_c(comm),
                  lidar(lidar) {

                comm.listen_for_frames({r2d2::frame_type::DISTANCE});
            }

            void process() override {
                while(comm.has_data()) {
                    auto frame = comm.get_data();
                    // Only handle requests
                    
                    if (!frame.request) {
                        continue;
                    }
                    
                    // Frame for the smallest distance measured
                    frame_distance_s smallest_distance;
                    
                    
                    for (int count = 0;; count++) {
                        // 16 packets for one 360 degree measurement
                        
                        for (int count = 0; count < 16; count++) {
                            lidar.receive_packet();
                        } 
                        // Every 100 360 degree measurements we print the distance for every
                        // half degree.
                        if (count == 10) {
                            for (int i = 0; i < 720; i++) {
								
								current_value = (lidar.measurements[i].distance_value / 4);
								
                                // Set the smallest distance that is not 0 for the first time
                                if ((current_value != 0 && smallest_value_set_first_time == false) {
                                    smallest_value = current_value;
                                    smallest_value_set_first_time = true;
                                }
                                hwlib::cout << "smallest value: " << smallest_value << '\n';
                                
                                hwlib::cout << "Angle: " << i << " measurement: "
                                            << current_value
                                            << '\n';
                                
                                
                                
                                if ((current_value != 0 && current_value < smallest_value ) {
                                    smallest_value = current_value;
                                }
                            }
                            hwlib::cout << '\n';
                            count = 0;
                        }
					// The smallest measured value(besides 0) will go into the frame
					// and will be send
                    smallest_distance.mm = smallest_value;
                    comm.send(smallest_distance);
                    }
                }
            }
    };
}
#pragma once

#include <cstdint>
#include <hwlib.hpp>


namespace r2d2 {
    namespace distance {
        /**
         * \brief This struct contains the header data of a packet
         *
         * The data in the struct includes all the data that is fixed for every
         * packet. Because every packet could have other frame_lenghts (more or
         * less measured distance values) depending on the rotation speed of the
         * lidar.
         */
        struct lidar_packet_header_s {
            uint16_t frame_length;    // Number of bytes in the packet excluding
                                      // the check_code.
            uint8_t protocol_version; // protocol version
                                      //  0x00 for Error packet,
                                      // 0x01 for Data packet.
            uint8_t frame_type;       // Fixed 0x61
            uint8_t command_word; // 0xAE == Error packet, 0xAD == Data packet.
            uint16_t data_length; // Number of bytes in the packet after
                                  // data_length excluding the check_code.
            uint8_t radar_speed;  // Turning speed (rotations/second) =
                                  // radar_speed / 20

            // Only needed for data packets not for error packets.
            // If the packet is an error packet, the values of the zero_offset
            // and the starting_angle are 0xFFFF.
            uint16_t zero_offset;
            uint16_t starting_angle;
        };
        /** 
        * \brief <<Operator for lidar_packet_header_s struct returns in hexadecimal 
        * data is pushed on the stream up to down        
        */
        hwlib::ostream &operator<<(hwlib::ostream &stream, const lidar_packet_header_s &lphs);
        
        
        
        /**
         * \brief This struct contains the date of one single distancec
         * measurement.
         */
        struct measurement_data_s {
            uint8_t signal; // probably the strength of the signal measured by
                            // the lidar, not sure.
            uint16_t distance_value; // distance in milimeter.
        };
        /** 
        * \brief <<Operator for measurement_data_s struct returns in decimal 
        * data is pushed on the stream up to down        
        */
        hwlib::ostream &operator<<(hwlib::ostream &stream, const measurement_data_s &mds);
        
    } // namespace distance
} // namespace r2d2
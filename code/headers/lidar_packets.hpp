#pragma once

#include <cstdint>

namespace r2d2 {
    namespace measuring_distance {
        /**
         * \brief This struct contains the header data of a packet
         *
         * The data in the struct includes all the data that is fixed for every packet.
         * Because every packet could have other frame_lenghts (more or less measured distance values) depending on the rotation speed of the lidar.
         */
        struct lidar_packet_header_s {
            uint16_t frame_length;      // Number of bytes in the packet excluding the check_code.
            uint8_t protocol_version;   // 0x00 == Error packet, 0x01 == Data packet.
            uint8_t frame_type;         // Fixed 0x61
            uint8_t command_word;       // 0xAE == Error packet, 0xAD == Data packet.
            uint16_t data_length;       // Number of bytes in the packet after data_length excluding the check_code.
            uint8_t radar_speed;        // Turning speed (rotations/second) = radar_speed / 20

            // Only needed for data packets not for error packets.
            // If the packet is an error packet, the values of the zero_offset and the starting_angle are 0xFFFF.
            uint16_t zero_offset;
            uint16_t starting_angle;
        };

        /**
         * \brief This struct contains the date of one single distancec measurement.
         */
        struct measurement_data_s {
            uint8_t signal;          // probably the strength of the signal measured by the lidar.
            uint16_t distance_value; // distance in milimeter.
        };
    }
}
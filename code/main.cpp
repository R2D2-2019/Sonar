#include "hardware_usart.hpp"
#include "hwlib.hpp"

namespace r2d2 {
    namespace measuring_distance {
        /**
         * \brief This struct contains the header data of a packet
         *
         * The data in the struct includes all the data that is fixed
         * in every packet.
         * Because every packet could have other frame_lenghts (more or less
         * measured distance values) depending on the rotation speed of the
         * lidar.
         */
        struct lidar_packet_header_s {
            uint16_t frame_length; // Number of bytes in the packet excluding
                                   // the check_code.
            uint8_t
                protocol_version; // 0x00 == Error packet, 0x01 == Data packet.
            uint8_t frame_type;   // Fixed 0x61
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
         * \brief This struct contains the date of one single distance
         * measurement.
         */
        struct measurement_data_s {
            uint8_t signal; // probably the strength of the signal measured by
                            // the lidar.
            uint16_t distance_value; // distance in milimeter.
        };

        /**
         * \brief This class makes it easy to read and extract data from the
         * lidar using uart.
         */
        class lidar_c {
        public:
            lidar_packet_header_s header; // This is the fixed amount of data
                                          // every packet starts with.
            measurement_data_s measurements[720]; // 720 = 360 * 2. So we sample
                                                  // data every 0.5 degree.

        private:
            r2d2::uart_ports_c usart_port;
            r2d2::hardware_usart_c uart;
            uint16_t checksum = 0x0000; // Checksum includes start byte (1) and
                                        // excludes the check_code bytes (2)

        protected:
            /**
             * \brief Receive 8 bits from uart.
             *
             * This function also adds the byte to the checksum for error
             * detection at the end of the frame.
             *
             * \return One byte of uart data.
             */
            uint8_t receive_uint8() {
                for (;;) {
                    if (uart.available() > 0) {
                        uint8_t byte = uart.receive();
                        checksum += byte;
                        return byte;
                    }
                }
            }

            /**
             * \brief Receive 16 bits from uart.
             *
             * This function uses \link
             * r2d2::measuring_distance::lidar_c::receive_uint8 receive_uint8()
             * \endlink
             *
             * \return Two bytes of uart data.
             */
            uint16_t receive_uint16() {
                return (receive_uint8() << 8) | receive_uint8();
            }

            /**
             * \brief This function waits for the given start_byte
             *
             * By looping as long until we received the given start_byte we wait
             * for the begin of the packet. This function uses \link
             * r2d2::measuring_distance::lidar_c::receive_uint8 receive_uint8()
             * \endlink
             */
            void wait_for_startbyte(uint8_t start_byte = 0xAA) {
                for (;;) {
                    if (receive_uint8() == start_byte) {
                        checksum = start_byte;
                        break;
                    }
                }
            }

            /**
             * \brief This function receives the packet header
             *
             * When the packet type equals an error type (0xAE) the zero_offset
             * & starting_angle are 0xFFFF. When the packet contains a normal
             * data packet, the zero_offset & starting_angle contain usable
             * data.
             *
             * The header variable (struct type) is declaired in the public
             * header section.
             *
             * \return A bool depending on successfully receiving an usable
             * packet from uart.
             */
            bool receive_packet_header() {
                header.frame_length = receive_uint16();
                header.protocol_version = receive_uint8();
                header.frame_type = receive_uint8();
                header.command_word = receive_uint8();
                header.data_length = receive_uint16();
                header.radar_speed = receive_uint8();

                if (header.frame_type != 0x61) {
                    // ERROR
                    return false;
                }

                if (header.protocol_version == 0x01 &&
                    header.command_word == 0xAD) {
                    header.zero_offset = receive_uint16();
                    header.starting_angle = receive_uint16();
                    return true;
                } else if (header.protocol_version == 0x00 &&
                           header.command_word == 0xAE) {
                    header.zero_offset = 0xFFFF;
                    header.starting_angle = 0xFFFF;
                    return true;
                } else {
                    // ERROR
                    return false;
                }
            }

            /**
             * \brief This function receives the measurements in one packet.
             *
             * In this function we save the measurements of one packet at the
             * right spot in the measurement array.
             */
            void receive_measurement_data() {
                uint8_t measurements_to_receive =
                    ((header.data_length - 5) / 3);

                for (unsigned int count = 0; count < measurements_to_receive;
                     count++) {
                    // measurement_angle = starting_angle + (((measurement_count
                    // - 1) * 22.5) / total_points_per_frame) This formula is
                    // extracted from the datasheet of the lidar communication
                    // protocol.
                    measurements[((header.starting_angle / 10) +
                                  (((count * (225)) /
                                    ((header.data_length - 5) / 3)))) /
                                 5] = {receive_uint8(), receive_uint16()};
                }
            }

        public:
            /**
             * \brief This is the constructor of the lidar class
             *
             * Because the constructor has default parameters it's not necessary
             * to give any parameters, only when you want different settings.
             *
             * \param uart_port Using UART 1 for RX1 (pin 19), lidar doesn't use
             * TX because we only receive data. \param baudrate We use baudrate
             * of 224400 because the operating baudrate of the lidar module has
             * to be 230400. But because the usart lib rounds wrong in the code
             * because of unsigned int rounding (22.75 becomes 22), to get the
             * best value (23) we have to put a wrong baudrate that rounds to
             * the good register value 23. 5241600 / 230400 = (int)22.75 = 22 ->
             * 0.75 away from actual lidar baudrate value 5241600 / 224400 =
             * (int)23.36 = 23  ->  0.25 away from actual lidar baudrate value
             * This means that 224400 is the best value to put in the
             * constructor.
             */
            lidar_c(r2d2::uart_ports_c uart_port = r2d2::uart_ports_c::uart1,
                    unsigned int baudrate = 224400)
                : measurements({}),
                  usart_port(uart_port),
                  uart(r2d2::hardware_usart_c(baudrate, usart_port)){};

            /**
             * \brief This fucntion reads a total of one packet of data from the
             * lidar.
             *
             * By first waiting on the startbyte we can capture the whole rest
             * of the packet after it. Than we receive the packet header and
             * body. If one of the operations fail, the receive packet returns
             * false. If all the operations including receiving and checking the
             * checksum succeed, the function returns true. <br> This function
             * uses \link r2d2::measuring_distance::lidar_c::wait_for_startbyte
             * wait_for_startbyte() \endlink, \link
             * r2d2::measuring_distance::lidar_c::receive_packet_header
             * receive_packet_header() \endlink, \link
             * r2d2::measuring_distance::lidar_c::receive_measurement_data
             * receive_measurement_data() \endlink and \link
             * r2d2::measuring_distance::lidar_c::receive_uint16
             * receive_uint16() \endlink to complete the tasks.
             *
             * \return true if operation completes without errors, false if an
             * error occured.
             */
            bool receive_packet() {
                wait_for_startbyte();
                if (!receive_packet_header() || header.command_word != 0xAD) {
                    return false;
                }

                receive_measurement_data();

                if (checksum == receive_uint16()) {
                    return true;
                } else {
                    return false;
                }
            }
        };
    } // namespace measuring_distance
} // namespace r2d2

int main(void) {
    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    // Wait 1 sec for starting the program to get good results.
    hwlib::wait_ms(1000);

    auto lidar = r2d2::measuring_distance::lidar_c();

    for (int count = 0;; count++) {
        // 16 packets for one 360 degree measurement
        for (int count = 0; count < 16; count++) {
            lidar.receive_packet();
        }

        // Every 100 360 degree measurements we print the distance for every half degree.
        if (count == 100) {
            for (int i = 0; i < 720; i++) {
                hwlib::cout << "Angle: " << i << " measurement: "
                            << (lidar.measurements[i].distance_value / 4)
                            << '\n';
            }
            hwlib::cout << '\n';
            count = 0;
        }
    }
}
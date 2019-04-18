#pragma once

#include <hardware_usart.hpp>
#include <lidar_packets.hpp>

namespace r2d2 {
    namespace measuring_distance {
        /**
         * \brief This class makes it easy to read and extract data from the lidar using uart.
         */
        class lidar_c {
        public:
            lidar_packet_header_s header;         // This is the fixed amount of data every packet starts with.
            measurement_data_s measurements[720]; // 720 = 360 * 2. So we sample data every 0.5 degree.

        private:
            r2d2::uart_ports_c usart_port;
            r2d2::hardware_usart_c uart;
            uint16_t checksum = 0x0000; // Checksum includes start byte (1) and excludes the check_code bytes (2)

        protected:
            /**
             * \brief Receive 8 bits from uart.
             *
             * This function also adds the byte to the checksum for error detection at the end of the frame.
             *
             * \return One byte of uart data.
             */
            uint8_t receive_uint8();

            /**
             * \brief Receive 16 bits from uart.
             *
             * This function uses \link r2d2::measuring_distance::lidar_c::receive_uint8 receive_uint8() \endlink
             *
             * \return Two bytes of uart data.
             */
            uint16_t receive_uint16();

            /**
             * \brief This function waits for the given start_byte
             *
             * This funcion works By looping as long until we received the given start_byte we wait
             * for the begin of the packet. This function uses 
             * \link r2d2::measuring_distance::lidar_c::receive_uint8 receive_uint8() \endlink
             */
            void wait_for_startbyte(uint8_t start_byte = 0xAA);

            /**
             * \brief This function receives the packet header
             *
             * When the packet type equals an error type (0xAE), the zero_offset & starting_angle are 0xFFFF.
             * When the packet contains a normal data packet, the zero_offset & starting_angle contain usable data.
             *
             * The header variable (struct type) is declaired in the public header section.
             *
             * \return A bool depending on successfully receiving an usable packet from uart.
             */
            bool receive_packet_header();

            /**
             * \brief This function receives the measurements in one packet.
             *
             * In this function we save the measurements of one packet at the right spot in the measurement array.
             */
            void receive_measurement_data();

        public:
            /**
             * \brief This is the constructor of the lidar class
             *
             * Because the constructor has default parameters it's not necessary to give any parameters, only when you want different settings.
             *
             * \param uart_port Using UART 1 for RX1 (pin 19), lidar doesn't use TX because we only receive data. 
             * 
             * \param baudrate We use baudrate of 224400 because the operating baudrate of the lidar module has to be 230400.
             * But because the usart lib rounds wrong in the code because of unsigned int rounding (22.75 becomes 22), to get the
             * best value (23) we have to put a wrong baudrate that rounds to the good register value 23.
             * 5241600 / 230400 = (int)22.75 = 22 -> 0.75 away from actual lidar baudrate value
             * 5241600 / 224400 = (int)23.36 = 23  ->  0.25 away from actual lidar baudrate value
             * This means that 224400 is the best value to put in the constructor.
             */
            lidar_c(r2d2::uart_ports_c uart_port = r2d2::uart_ports_c::uart1, unsigned int baudrate = 224400);

            /**
             * \brief This fucntion reads a total of one packet of data from the lidar.
             *
             * By first waiting on the startbyte we can capture the whole rest of the packet after it.
             * Than we receive the packet header and body.
             * If one of the operations fail, the receive packet returns false.
             * If all the operations including receiving and checking the checksum succeed, the function returns true. 
             * <br> This function uses
             * \link r2d2::measuring_distance::lidar_c::wait_for_startbyte wait_for_startbyte() \endlink,
             * \link r2d2::measuring_distance::lidar_c::receive_packet_header receive_packet_header() \endlink,
             * \link r2d2::measuring_distance::lidar_c::receive_measurement_data receive_measurement_data() \endlink
             * and \link r2d2::measuring_distance::lidar_c::receive_uint16 receive_uint16() \endlink
             * to complete the tasks.
             *
             * \return true if operation completes without errors, false if an error occured.
             */
            bool receive_packet();
        };
    } // namespace measuring_distance
} // namespace r2d2
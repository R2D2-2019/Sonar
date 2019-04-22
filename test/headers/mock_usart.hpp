#pragma once
#include <usart_connection.hpp>
#include <vector>

namespace r2d2 {

    class mock_usart_c : public usart_connection_c {
    private:
        std::vector<uint8_t> receive_buffer = {};
        size_t receive_counter = 0;

    public:
        /// @brief does not actualy enable anything
        void enable() override;

        /// @brief does not actualy disable anyting
        void disable() override;

        /// @brief sends a byte
        /// @param c byte to send..
        /// does not actualy send anything in this implementation
        bool send(const uint8_t c) override;

        /// @brief sends char c
        /// @param c char to send..
        /// does not actualy send anything in this implementation
        void putc(char c) override;

        /// @brief returns byte
        /// @return the next byte from the receive buffer
        uint8_t receive() override;

        /// @brief returns true if char is available
        /// @return true when a byte is available in the receive buffer
        bool char_available() override;

        /// @brief returns receive()
        /// @return the byte from
        /// \link r2d2::mock_usart_c::receive receive() \endlink
        char getc() override;

        /// @brief returns the number of bytes available in the receive buffer
        /// @return the number of bytes available in the receive buffer
        unsigned int available() override;

        /// @brief prepares a message to be received durring the test
        ///
        /// the message can be received using
        /// \link r2d2::mock_usart_c::receive receive() \endlink or
        /// \link r2d2::mock_usart_c::getc getc(). \endlink
        ///  This fuctions will behave the same as the hardware implementation.
        void prepare_message(const std::vector<uint8_t> &message);
    };
}; // namespace r2d2

#pragma once
#include <usart_connection.hpp>
#include <vector>

namespace r2d2 {

    class test_usart_c : public usart_connection_c {
    private:
        std::vector<uint8_t> receive_buffer = {};
        size_t receive_counter = 0;

    public:
        test_usart_c();

        /// @brief does not actualy disable anything
        void enable() override;

        /// @brief does not actualy disable anyting
        void disable() override;

        /// @brief sends byte
        /// @param c byte to send.. does not actualy send anything
        bool send(const uint8_t c) override;

        ///@brief sends char c
        ///@param c char to send
        void putc(char c) override;

        ///@brief returns byte
        ///@return uint8_t always 0xAA
        uint8_t receive() override;

        ///@brief returns true if char is available
        ///@return bool always true
        bool char_available() override;

        ///@brief returns receive()
        ///@return char uint8_t from recieve
        char getc() override;

        ///@brief returns 1
        ///@return unsigned int always 1
        unsigned int available() override;

        void prepare_message(const std::vector<uint8_t> &message);
    };
}; // namespace r2d2

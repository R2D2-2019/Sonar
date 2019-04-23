#include "ostream"
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <lidar.hpp>
#include <mock_usart.hpp>

TEST_CASE("mock_usart::receive(), mock_usart::prepare_message()") {
    std::vector<uint8_t> message = {11, 13, 23, 55, 66, 83, 42};

    r2d2::mock_usart_c usart;
    usart.prepare_message(message);

    for (const uint8_t &b : message) {
        REQUIRE(b == usart.receive());
    }
}

TEST_CASE("Lidar - receive valid error packet") {
    std::vector<uint8_t> message = {0xAA, 0x00, 0x09, 0x00, 0x61, 0xAE,
                                    0x00, 0x01, 0xC9, 0x02, 0x8C};

    r2d2::mock_usart_c usart;
    usart.prepare_message(message);

    r2d2::distance::lidar_c lidar(usart);
    REQUIRE(lidar.receive_packet());

    REQUIRE(lidar.header.frame_length == 0x0009);
    REQUIRE(lidar.header.protocol_version == 0x00);
    REQUIRE(lidar.header.frame_type == 0x61);
    REQUIRE(lidar.header.command_word == 0xAE);
    REQUIRE(lidar.header.data_length == 0x0001);
    REQUIRE(lidar.header.radar_speed == 0xC9);

    REQUIRE(lidar.header.zero_offset == 0xFFFF);
    REQUIRE(lidar.header.starting_angle == 0xFFFF);
}

TEST_CASE("Lidar - receive valid packet with invalid checksum") {
    std::vector<uint8_t> message = {0xAA, 0x00, 0x09, 0x00, 0x61, 0xAE,
                                    0x00, 0x01, 0xC9, 0x02, 0x8D};

    r2d2::mock_usart_c usart;
    usart.prepare_message(message);

    r2d2::distance::lidar_c lidar(usart);
    REQUIRE(!lidar.receive_packet());
}

TEST_CASE("Lidar - invalid frame type") {
    std::vector<uint8_t> message = {
        0xAA, 0x00, 0x09, 0x00, 0x42, // wrong value: 0x42 should be 0x61
        0xAE, 0x00, 0x01, 0xC9, 0x02, 0x8D};

    r2d2::mock_usart_c usart;
    usart.prepare_message(message);

    r2d2::distance::lidar_c lidar(usart);
    REQUIRE(!lidar.receive_packet());
}

TEST_CASE("Lidar - unknown command word") {
    std::vector<uint8_t> message = {0xAA, 0x00, 0x09, 0x00, 0x61,
                                    0x2E, // 0x2E isnt a valid command
                                    0x00, 0x01, 0xC9, 0x02, 0x8D};

    r2d2::mock_usart_c usart;
    usart.prepare_message(message);

    r2d2::distance::lidar_c lidar(usart);
    REQUIRE(!lidar.receive_packet());
}

TEST_CASE("Lidar - receive valid data packet") {
    std::vector<uint8_t> message = {
        0xAA, 0x00, 0x9A, 0x01, 0x61, 0xAD, 0x00, 0x92, 0x82, 0x00, 0x87, 0x69,
        0x78, 0x00, 0x00, 0x00, 0x46, 0x21, 0x3A, 0x54, 0x23, 0x78, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x91, 0x33, 0x60, 0x82, 0x32, 0xF7, 0x93, 0x32,
        0xEB, 0x6D, 0x32, 0xE0, 0x51, 0x21, 0x88, 0x00, 0x00, 0x00, 0x5D, 0x21,
        0x88, 0x66, 0x21, 0x8D, 0x68, 0x21, 0xBF, 0x41, 0x32, 0xD4, 0x86, 0x33,
        0x02, 0x4D, 0x32, 0xE0, 0x89, 0x51, 0x48, 0x8E, 0x51, 0x48, 0x92, 0x51,
        0x48, 0x8C, 0x51, 0x48, 0x63, 0x50, 0x19, 0x6D, 0x51, 0x48, 0x7C, 0x51,
        0x64, 0x92, 0x51, 0x64, 0x89, 0x51, 0x48, 0x90, 0x51, 0x64, 0x89, 0x51,
        0x48, 0x93, 0x51, 0x64, 0x4B, 0x53, 0x2D, 0x57, 0x59, 0xBA, 0x43, 0x2F,
        0x78, 0x41, 0x2E, 0xE4, 0x00, 0x00, 0x00, 0x54, 0x2E, 0xDE, 0x6B, 0x2E,
        0xE4, 0x6B, 0x2F, 0x50, 0x58, 0x2E, 0xE4, 0x7E, 0x2F, 0x64, 0x5D, 0x2F,
        0x78, 0x3F, 0x5A, 0x0B, 0x5A, 0x5B, 0xFD, 0x57, 0x5B, 0xD3, 0x5B, 0x5C,
        0x28, 0x59, 0x5C, 0x28, 0x59, 0x5B, 0xFD, 0x5E, 0x5E, 0x32, 0x35, 0xBC};

    uint16_t sum = 0;
    for (const uint8_t &b : message) {
        sum += b;
    }

    r2d2::mock_usart_c usart;
    usart.prepare_message(message);

    r2d2::distance::lidar_c lidar(usart);
    REQUIRE(lidar.receive_packet());

    // check the header
    REQUIRE(lidar.header.frame_length == 0x009A);
    REQUIRE(lidar.header.protocol_version == 0x01);
    REQUIRE(lidar.header.frame_type == 0x61);
    REQUIRE(lidar.header.command_word == 0xAD);
    REQUIRE(lidar.header.data_length == 0x0092);
    REQUIRE(lidar.header.radar_speed == 0x82);
    REQUIRE(lidar.header.zero_offset == 0x0087);
    REQUIRE(lidar.header.starting_angle == 0x6978);

    // check the distance measurements
    const unsigned int number_of_measurements =
        (lidar.header.data_length - 5) / 3;
    const float offset = 22.5 / number_of_measurements;
    const float start_angle = lidar.header.starting_angle * 0.01;

    for (unsigned int i = 0; i < number_of_measurements; i++) {
        unsigned int angle = (start_angle + (i * offset)) * 2;

        unsigned int next_angle = (start_angle + ((i + 1) * offset)) * 2;

        if (next_angle == angle) {
            // the value is overwritten by the next one, skip the checks
            continue;
        }

        uint8_t signal = message[(i * 3) + 13];
        uint16_t dist = message[(i * 3) + 14] << 8 | message[(i * 3) + 15];

        REQUIRE(lidar.measurements[angle].signal == signal);
        REQUIRE(lidar.measurements[angle].distance_value == dist);
    }
}
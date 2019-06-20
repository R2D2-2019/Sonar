#include <hwlib.hpp>
#include <iostream>
#include <string>
#include "HC_SR04.hpp"

int main()
{
    auto echo_pin = hwlib::target::pin_in(hwlib::target::pins::d7);
    auto trigger_pin = hwlib::target::pin_out(hwlib::target::pins::d8);
    HC_SR04 sensor(echo_pin, trigger_pin);
}
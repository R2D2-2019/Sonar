#pragma once
#include <hwlib.hpp>


class HC_SR04
{
 private:
    hwlib::target::pin_in & echo_pin;
    hwlib::target::pin_out & trigger_pin;
public:
    HC_SR04(hwlib::target::pin_in & echo_pin, hwlib::target::pin_out & trigger_pin):
        echo_pin(echo_pin),
        trigger_pin(trigger_pin)
        {}
        
    int16_t get_distance();
       
};

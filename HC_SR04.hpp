#ifndef HC_SR04
#define HC_SR04

#include <hwlib.hpp>
#include <iostream>
#include <string>

class HC_SR04
{
 private:
    int distance;
    float duration;
    hwlib::target::pin_in & echo_pin;
    hwlib::target::pin_out & trigger_pin;
public:
    HC_SR04(hwlib::target::pin_in & echo_pin, hwlib::target::pin_out & trigger_pin):
        echo_pin(echo_pin),
        trigger_pin(trigger_pin)
        {}
        
    int get_distance();        
};

#endif //HC_SR04
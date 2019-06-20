#include <hwlib.hpp>
#include "HC_SR04.hpp"

int main()
{
    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    hwlib::wait_ms(1000);
    hwlib::cout << "this works via arduino" << hwlib::endl;
    auto echo_pin = hwlib::target::pin_in(hwlib::target::pins::d7);
    auto trigger_pin = hwlib::target::pin_out(hwlib::target::pins::d8);
 
    HC_SR04 sensor(echo_pin, trigger_pin);

    while(1)
    {
        hwlib::wait_ms(500);
        hwlib::cout << sensor.get_distance()  << hwlib::endl;
    }
}
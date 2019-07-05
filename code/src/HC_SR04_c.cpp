#include <HC_SR04_c.hpp>

uint16_t r2d2::distance::HC_SR04_c::get_distance(){
    trigger_pin.write(0);
    hwlib::wait_us(2);
    trigger_pin.write(1);
    hwlib::wait_us(10);
    trigger_pin.write(0);

    while(!echo_pin.read()){
        hwlib::wait_us(5);
    }

    uint32_t ticks_start = hwlib::now_ticks();

    while(echo_pin.read()){
        hwlib::wait_us(5);
    }

    uint32_t tick_counter = hwlib::now_ticks() - ticks_start;
    tick_counter = tick_counter / (hwlib::ticks_per_us() * 2) * 0.35; // formula to change ticks into centimeters.
    // if(tick_counter > 399) {
    //     return -1;
    // }
    return tick_counter;
}

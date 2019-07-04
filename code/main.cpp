#include "module.hpp"
#include <comm.hpp>
#include <hardware_usart.hpp>
#include <lidar.hpp>

int main(void) {
    using namespace r2d2;

    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    // Wait for starting the program to get good results.
    hwlib::wait_ms(1000);
    r2d2::comm_c comm;
    // comm for the test module
    r2d2::comm_c comm2;

    // We use baudrate of 224400 because the operating baudrate of the lidar
    // module has to be 230400. But because the usart lib rounds wrong in the
    // code because of unsigned int rounding (22.75 becomes 22), to get the best
    // value (23) we have to put a wrong baudrate that rounds to the good
    // register value 23. 5241600 / 230400 = (int)22.75 = 22 -> 0.75 away from
    // actual lidar baudrate value 5241600 / 224400 = (int)23.36 = 23  ->  0.25
    // away from actual lidar baudrate value This means that 224400 is the best
    // value to put in the constructor.
    auto usart = r2d2::usart::hardware_usart_c<r2d2::usart::usart0>(224400);

    auto lidar = r2d2::distance::lidar_c(usart);

    distance::module_c module(comm, lidar);

    for (;;) {
        module.process();
    }
}
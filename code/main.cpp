#include "module.hpp"
#include <lidar.hpp>
#include <hardware_usart.hpp>
#include <comm.hpp>
#include "test_module.hpp"

int main(void) {
    using namespace r2d2;

    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    // Wait 1 sec for starting the program to get good results.
    hwlib::wait_ms(10);

    r2d2::comm_c comm;
    //r2d2::comm_c comm2;

    // We use baudrate of 224400 because the operating baudrate of the lidar
    // module has to be 230400. But because the usart lib rounds wrong in the
    // code because of unsigned int rounding (22.75 becomes 22), to get the best
    // value (23) we have to put a wrong baudrate that rounds to the good
    // register value 23. 5241600 / 230400 = (int)22.75 = 22 -> 0.75 away from
    // actual lidar baudrate value 5241600 / 224400 = (int)23.36 = 23  ->  0.25
    // away from actual lidar baudrate value This means that 224400 is the best
    // value to put in the constructor.

    auto usart = r2d2::usart::hardware_usart_c(224400, r2d2::usart::usart_ports::uart1);

    auto lidar = r2d2::distance::lidar_c(usart);

    distance::module_c module(comm, lidar);

    // Test module for sending a request
    //distance::test_module_c test_module(comm2);

    for(;;) {
        //test_module.process();
        module.process();
    }

}
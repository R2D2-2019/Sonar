#include "hwlib.hpp"
#include "hardware_usart.hpp"


int main(void) {
  // kill the watchdog
  WDT->WDT_MR = WDT_MR_WDDIS;

  auto usart_port = r2d2::uart_ports_c::uart1;
  auto uart = r2d2::hardware_usart_c(230400, usart_port);

  for(;;){
    if(uart.available() > 0) {
      hwlib::cout << "Received from real UART: " << (int)uart.receive() << hwlib::endl;
    }
  }
}
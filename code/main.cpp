#include "hwlib.hpp"
#include "hardware_usart.hpp"

uint8_t receive_byte(r2d2::hardware_usart_c & uart){
  for(;;){
    if(uart.available() > 0){
      return uart.receive();
    }
  }
}

void wait_for_startbyte(r2d2::hardware_usart_c & uart, uint8_t start_byte = 0xAA){
  for(;;){
    if(receive_byte(uart) == start_byte){
      return;
    }
  }
}



int main(void) {
  // kill the watchdog
  WDT->WDT_MR = WDT_MR_WDDIS;
  // Wait 1 sec for starting the program to get good results.
  hwlib::wait_ms(1000);

  // Using UART 1 for RX1 (pin 19), lidar doesn't use TX because we only receive data
  auto usart_port = r2d2::uart_ports_c::uart1;
  // We use baudrate of 224400 because the operating baudrate of the lidar module has to be 230400. But
  // because the usart lib rounds wrong in the code because of unsigned int rounding (22.75 becomes 22), to
  // get the best value (23) we have to put a wrong baudrate that rounds to the good register value 23.
  // 5241600 / 230400 = (int)22.75 = 22  ->  0.75 away from actual lidar baudrate value
  // 5241600 / 224400 = (int)23.36 = 23  ->  0.25 away from actual lidar baudrate value
  // This means that 224400 is the best value to put in the constructor
  auto uart = r2d2::hardware_usart_c(224400, usart_port);

  
  for(;;){
    // Need for checksum check at the end
    uint16_t checksum = 0x00AA; // Checksum includes start byte and is allways the same so default the 0xAA

    wait_for_startbyte(uart, 0xAA); 

    // The frame_length is the byte count of the frame excluded the first 3 bytes ( 1 start byte + 2 frame_length bytes).
    uint16_t frame_length = ((receive_byte(uart) << 8) | receive_byte(uart)); 
    checksum += frame_length;

    auto bytes_to_receive_count = frame_length;
    (void)bytes_to_receive_count;

    uint8_t protocol_version = receive_byte(uart);
    checksum += protocol_version;
    bytes_to_receive_count -= 1;

    uint8_t frame_type = receive_byte(uart);
    checksum += frame_type;
    bytes_to_receive_count -= 1;

    if(protocol_version != 0x01 || frame_type != 0x61){
      continue;
    }

    uint8_t command_word = receive_byte(uart);
    checksum += command_word;
    bytes_to_receive_count -= 1;
    if(command_word != 0xAD){ // Data
      continue;
    }

    uint16_t data_length = ((receive_byte(uart) << 8) | receive_byte(uart));
    checksum += data_length;
    bytes_to_receive_count -= 2;
    (void)data_length;

    uint8_t radar_speed = receive_byte(uart);
    checksum += radar_speed;
    bytes_to_receive_count -= 1;
    (void)radar_speed;

    uint16_t zero_offset = ((receive_byte(uart) << 8) | receive_byte(uart));
    checksum += zero_offset;
    bytes_to_receive_count -= 2;
    (void)zero_offset;

    uint16_t starting_angle = ((receive_byte(uart) << 8) | receive_byte(uart));
    checksum += starting_angle;
    bytes_to_receive_count -= 2;
    (void)starting_angle;

    while(bytes_to_receive_count >= 0){
      if(bytes_to_receive_count >= 3){
        uint8_t signal_0 = receive_byte(uart);
        checksum += signal_0;
        bytes_to_receive_count -= 1;
        (void)signal_0;

        uint16_t distance_value = ((receive_byte(uart) << 8) | receive_byte(uart));

        checksum += distance_value;
        bytes_to_receive_count -= 2;
        (void)distance_value;
      }else{
        uint16_t check_code = ((receive_byte(uart) << 8) | receive_byte(uart));
        if(check_code == checksum){
          hwlib::cout << "Toppie!!\n";
          break;
        }else{
          hwlib::cout << "Jammer..\n";
          hwlib::cout << "checksum: " << checksum << ", check_code: " << check_code << '\n';
          break;
        }
      }
    }
    break; // exit inf loop
  }
}
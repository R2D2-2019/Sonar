#include <lidar.hpp>

int main(void) {
    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    // Wait 1 sec for starting the program to get good results.
    hwlib::wait_ms(1000);

    auto lidar = r2d2::measuring_distance::lidar_c();

    for (int count = 0;; count++) {
        // 16 packets for one 360 degree measurement
        for (int count = 0; count < 16; count++) {
            lidar.receive_packet();
        }

        // Every 100 360 degree measurements we print the distance for every half degree.
        if (count == 100) {
            for (int i = 0; i < 720; i++) {
                hwlib::cout << "Angle: " << i << " measurement: "
                            << (lidar.measurements[i].distance_value / 4)
                            << '\n';
            }
            hwlib::cout << '\n';
            count = 0;
        }
    }
}
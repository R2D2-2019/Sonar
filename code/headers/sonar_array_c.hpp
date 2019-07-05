#include <hwlib.hpp>
#include "IUnidirectional_distance_sensor.hpp"
#include "HC-SR04_c.hpp"


namespace r2d2::distance {
    template<uint8_t N>
    class sonar_array_c: public IUnidirectional_distance_sensor_c {
    private:
        HC_SR04_c sonar[N];
    public:
    sonar_array_c():
        {}
        uint16_t get_distance() {
            uint32_t sum[N];
            for(unsigned int i = 0; i < N ; i++) {
                sum[i] = sonar[i].get_distance();
            }
            for(unsigned int i = 0; i < N; i++) {
                
            }
        }   
    }
}
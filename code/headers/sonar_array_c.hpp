#pragma once

#include <hwlib.hpp>
#include "IUnidirectional_distance_sensor.hpp"
#include "HC-SR04_c.hpp"


namespace r2d2::distance {
     /**
         * \brief 
         * This is Template sonar class
         *
         * \details
         * This templated sonar class is able to use a unkown amount of sonars in an array to get distances from.
         */
    template<uint8_t N>
    class sonar_array_c: public IUnidirectional_distance_sensor_c {
    private:
        HC_SR04_c sonar[N]; // an array of sonar modules.
    public:
    /// \brief
    /// The constructor of the sonar array.
    /// \param no parameters are given for the default constructor.
    // 
    sonar_array_c():
        {}
     /// \brief
    /// function that returns a distance.
    /// \details function that calls every sonar module and returns the smallest measured distance.
        uint16_t get_distance() {
            uint32_t sum[N];
            for(unsigned int i = 0; i < N ; i++) {
                sum[i] = sonar[i].get_distance();
            }
            uint16_t smallest_measurement = sum[0]
            for(unsigned int i = 0; i < N; i++) {
                if(i > 0 && i <= N) {
                    if(sum[i] < smallest_measurement) {
                        smallest_measurement = sum[i];
                    }
                }
            }
            return smallest_measurement;
        }   
    }
}
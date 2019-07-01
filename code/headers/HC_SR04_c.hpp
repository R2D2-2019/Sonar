#pragma once
#include <hwlib.hpp>

namespace R2D2::Distance
{
        /**
     *  \brief
     *  This class is for reading the data from the HC-SR04 ultrasone sensor
     * 
     *  \details
     *   This class return the distance measured with the HC-SR04 ultrasone sensor, accurate up to 400 centimeters.
     *   After 400 centimeters measurements are unuseable and should be disgarded.
     */
    class HC_SR04_c
    {
    private:
        hwlib::pin_in & echo_pin;       //Listens to a high value from returning ultrasone sounds
        hwlib::pin_out & trigger_pin;   //Triggers the sensor to send out a ultrasone frequency
    public:
        HC_SR04_c(hwlib::target::pin_in & echo_pin, hwlib::target::pin_out & trigger_pin)
        :   echo_pin(echo_pin), trigger_pin(trigger_pin)
            {}
        /**
        * return measured distance in centimeters.
        */
        int16_t get_distance();
    };
}


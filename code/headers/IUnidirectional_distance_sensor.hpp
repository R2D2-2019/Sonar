#include <hwlib.hpp>

class IUnidirectional_distance_sensor {
public:
    virtual uint16_t  get_distance() = 0;
}
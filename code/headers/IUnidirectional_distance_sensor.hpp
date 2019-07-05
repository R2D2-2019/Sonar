#include <hwlib.hpp>

namespace r2d2::distance
{
    class IUnidirectional_distance_sensor_c {
    public:
        virtual uint16_t  get_distance() = 0;
    };
} // R2D2::Distance

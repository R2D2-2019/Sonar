#include <lidar_packets.hpp>

namespace r2d2::distance {

    hwlib::ostream &operator<<(hwlib::ostream &stream,
                               const lidar_packet_header_s &lphs) {
        return stream << hwlib::hex << "packet header: " << lphs.frame_length
                      << "\t" << lphs.protocol_version << "\t"
                      << lphs.frame_type << "\t" << lphs.command_word << "\t"
                      << lphs.data_length << "\t" << lphs.radar_speed << "\t"
                      << lphs.zero_offset << "\t" << lphs.starting_angle
                      << "\n";
    }
    hwlib::ostream &operator<<(hwlib::ostream &stream,
                               const measurement_data_s &mds) {
        return stream << hwlib::dec << "\tsignal: " << mds.signal
                      << "\tmeasurement: " << (mds.distance_value / 4) << "\n";
    }

} // namespace r2d2::distance

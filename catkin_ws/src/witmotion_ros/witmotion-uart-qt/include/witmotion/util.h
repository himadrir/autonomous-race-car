/*!
    \file util.h
    \brief Helper and utility functions to decode and represent the data
    \author Andrey Vukolov andrey.vukolov@elettra.eu
*/

#ifndef WITMOTION_UTIL
#define WITMOTION_UTIL
#include "witmotion/types.h"

namespace witmotion
{

class witmotion_typed_packets
{
private:
    witmotion_datapacket array[32];
public:
    witmotion_datapacket& operator[](const witmotion_packet_id id);
};

class witmotion_typed_bytecounts
{
private:
    size_t array[32];
public:
    witmotion_typed_bytecounts();
    size_t& operator[](const witmotion_packet_id id);
};

/*!
 \brief Converts the frequency value in Hertz to subsequent Witmotion opcode.

 Special values for the `hertz` argument are:
 |Value|Description|
 |-----|-----------|
 |`0`  | Shuts down the measurements but does not turn the device into [dormant mode](\ref ridStandbyMode) |
 |`-1` | Orders the single-shot measurement, then shutdown |
 |`-2` | 1 measurement in 2 seconds |
 |`-10`| 1 measurement in 10 seconds |
 \param hertz - frequency in Hertz, or a special value as it is described above
 \return Witmotion opcode value as a byte, `0x06` (10 Hz) by default is the argument is inacceptable
 */
uint8_t witmotion_output_frequency(const int hertz);

uint8_t witmotion_baud_rate(const QSerialPort::BaudRate rate);

bool id_registered(const size_t id);

/* COMPONENT DECODERS */
float decode_acceleration(const int16_t* value);
float decode_angular_velocity(const int16_t* value);
float decode_angle(const int16_t* value);
float decode_temperature(const int16_t* value);
float decode_orientation(const int16_t* value);
void decode_gps_coord(const int32_t* value,
                      double& deg,
                      double& min);

/* PACKET DECODERS */
void decode_realtime_clock(const witmotion_datapacket& packet,
                           uint8_t& year,
                           uint8_t& month,
                           uint8_t& day,
                           uint8_t& hour,
                           uint8_t& minute,
                           uint8_t& second,
                           uint16_t& millisecond);
void decode_accelerations(const witmotion_datapacket& packet,
                          float& x,
                          float& y,
                          float& z,
                          float& t);
void decode_angular_velocities(const witmotion_datapacket& packet,
                               float& x,
                               float& y,
                               float& z,
                               float& t);
void decode_angles(const witmotion_datapacket& packet,
                   float& roll,
                   float& pitch,
                   float& yaw,
                   float& t);
void decode_magnetometer(const witmotion_datapacket& packet,
                         float& x,
                         float& y,
                         float& z,
                         float& t);
void decode_altimeter(const witmotion_datapacket& packet,
                      double& pressure,
                      double& height);
void decode_gps(const witmotion_datapacket& packet,
                double& longitude_deg,
                double& longitude_min,
                double& latitude_deg,
                double& latitude_min);
void decode_gps_ground_speed(const witmotion_datapacket& packet,
                             float& altitude,
                             float& angular_velocity,
                             double& ground_speed);
void decode_orientation(const witmotion_datapacket& packet,
                        float& x,
                        float& y,
                        float& z,
                        float& w);
void decode_gps_accuracy(const witmotion_datapacket& packet,
                         size_t& satellites,
                         float& local_accuracy,
                         float& horizontal_accuracy,
                         float& vertical_accuracy);

/* MISCELLANEOUS UTILITIES */
template<typename T> T variance(const std::vector<T>& array)
{
    T sum = std::accumulate(array.begin(), array.end(), 0.f);
    T mean = sum / static_cast<T>(array.size());
    T sq_dif = 0.f;
    for(auto i = array.begin(); i != array.end(); i++)
        sq_dif += std::pow((*i) - mean, 2);
    sq_dif /= (array.size() > 1) ? static_cast<T>(array.size() - 1) : 1.f;
    return std::sqrt(sq_dif);
}

}
#endif

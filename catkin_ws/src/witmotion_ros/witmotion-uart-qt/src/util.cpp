#include "witmotion/util.h"

#include <iostream>

namespace witmotion
{

bool id_registered(const size_t id)
{
    return (witmotion_registered_ids.find(id) != witmotion_registered_ids.end());
}

witmotion_datapacket& witmotion_typed_packets::operator[](const witmotion_packet_id id)
{
    size_t int_id = static_cast<size_t>(id) - 0x50;
    return array[int_id];
}

witmotion_typed_bytecounts::witmotion_typed_bytecounts()
{
    for(size_t i = 0; i < 32; i++)
        array[i] = 0;
}

size_t& witmotion_typed_bytecounts::operator[](const witmotion_packet_id id)
{
    size_t int_id = static_cast<size_t>(id) - 0x50;
    return array[int_id];
}

uint8_t witmotion_output_frequency(const int hertz)
{
    switch(hertz)
    {
    case -10: // 0.1
        return 0x01;
    case -2: // 0.5
        return 0x02;
    case 1:
        return 0x03;
    case 2:
        return 0x04;
    case 5:
        return 0x05;
    case 20:
        return 0x07;
    case 50:
        return 0x08;
    case 100:
        return 0x09;
    case 125:
        return 0x0A;
    case 200:
        return 0x0B;
    case 0: // Shutdown
        return 0x0D;
    case -1: // Single shot
        return 0x0C;
    case 10:
    default:
        return 0x06;
    }
}

uint8_t witmotion_baud_rate(const QSerialPort::BaudRate rate)
{
    switch(rate)
    {
    case QSerialPort::Baud1200:
    case QSerialPort::Baud2400:
        return 0x00;
    case QSerialPort::Baud4800:
        return 0x01;
    case QSerialPort::Baud19200:
        return 0x03;
    case QSerialPort::Baud38400:
        return 0x04;
    case QSerialPort::Baud57600:
        return 0x05;
    case QSerialPort::Baud115200:
        return 0x06;
    case QSerialPort::Baud9600:
    default:
        return 0x02;
    }
}

/* COMPONENT DECODERS */
float decode_acceleration(const int16_t* value)
{
    return static_cast<float>(*value) / 32768.f * 16.f * 9.81;
}

float decode_angular_velocity(const int16_t* value)
{
    return (static_cast<float>(*value) / 32768.f * 2000.f);
}

float decode_angle(const int16_t* value)
{
    return (static_cast<float>(*value) / 32768.f * 180.f);
}

float decode_temperature(const int16_t* value)
{
    return static_cast<float>(*value) / 100.f;
}

float decode_orientation(const int16_t *value)
{
    return static_cast<float>(*value) / 32768.f;
}

void decode_gps_coord(const int32_t *value,
                      double &deg,
                      double &min)
{
    deg = static_cast<double>(*value) / 100000000.f;
    min = static_cast<double>((*value) % 10000000) / 100000.f;
}

/* PACKET DECODERS */
void decode_accelerations(const witmotion_datapacket &packet,
                          float &x,
                          float &y,
                          float &z,
                          float &t)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidAcceleration)
        return;
    x = decode_acceleration(packet.datastore.raw_cells);
    y = decode_acceleration(packet.datastore.raw_cells + 1);
    z = decode_acceleration(packet.datastore.raw_cells + 2);
    t = decode_temperature(packet.datastore.raw_cells + 3);
}

void decode_angular_velocities(const witmotion_datapacket &packet,
                               float &x,
                               float &y,
                               float &z,
                               float &t)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidAngularVelocity)
        return;
    x = decode_angular_velocity(packet.datastore.raw_cells);
    y = decode_angular_velocity(packet.datastore.raw_cells + 1);
    z = decode_angular_velocity(packet.datastore.raw_cells + 2);
    t = decode_temperature(packet.datastore.raw_cells + 3);
}

void decode_angles(const witmotion_datapacket &packet,
                   float &roll,
                   float &pitch,
                   float &yaw,
                   float &t)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidAngles)
        return;
    roll = decode_angle(packet.datastore.raw_cells);
    pitch = decode_angle(packet.datastore.raw_cells + 1);
    yaw = decode_angle(packet.datastore.raw_cells + 2);
    t = decode_temperature(packet.datastore.raw_cells + 3);
}

void decode_magnetometer(const witmotion_datapacket &packet,
                         float &x,
                         float &y,
                         float &z,
                         float &t)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidMagnetometer)
        return;
    x = static_cast<float>(packet.datastore.raw_cells[0]);
    y = static_cast<float>(packet.datastore.raw_cells[1]);
    z = static_cast<float>(packet.datastore.raw_cells[2]);
    t = decode_temperature(packet.datastore.raw_cells + 3);
}

void decode_altimeter(const witmotion_datapacket &packet,
                      double &pressure,
                      double &height)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidAltimeter)
        return;
    pressure = static_cast<double>(packet.datastore.raw_large[0]);
    height = static_cast<double>(packet.datastore.raw_large[1]) / 100.f;
}

void decode_gps(const witmotion_datapacket &packet,
                double &longitude_deg,
                double &longitude_min,
                double &latitude_deg,
                double &latitude_min)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidGPSCoordinates)
        return;
    decode_gps_coord(packet.datastore.raw_large, longitude_deg, longitude_min);
    decode_gps_coord(packet.datastore.raw_large + 1, latitude_deg, latitude_min);
}

void decode_gps_ground_speed(const witmotion_datapacket &packet,
                             float &altitude,
                             float &angular_velocity,
                             double &ground_speed)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidGPSGroundSpeed)
        return;
    altitude = static_cast<float>(packet.datastore.raw_cells[0]) / 10.f;
    angular_velocity = static_cast<float>(packet.datastore.raw_cells[1]) / 10.f;
    ground_speed = static_cast<double>(packet.datastore.raw_large[1]) / 1000.f;
}

void decode_orientation(const witmotion_datapacket &packet,
                        float &x,
                        float &y,
                        float &z,
                        float &w)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidOrientation)
        return;
    x = decode_orientation(packet.datastore.raw_cells);
    y = decode_orientation(packet.datastore.raw_cells + 1);
    z = decode_orientation(packet.datastore.raw_cells + 2);
    w = decode_orientation(packet.datastore.raw_cells + 3);
}

void decode_gps_accuracy(const witmotion_datapacket &packet,
                         size_t &satellites,
                         float &local_accuracy,
                         float &horizontal_accuracy,
                         float &vertical_accuracy)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidGPSAccuracy)
        return;
    satellites = static_cast<size_t>(packet.datastore.raw_cells[0]);
    local_accuracy = decode_orientation(packet.datastore.raw_cells + 1);
    horizontal_accuracy = decode_orientation(packet.datastore.raw_cells + 2);
    vertical_accuracy = decode_orientation(packet.datastore.raw_cells + 3);
}

void decode_realtime_clock(const witmotion_datapacket &packet,
                           uint8_t &year,
                           uint8_t &month,
                           uint8_t &day,
                           uint8_t &hour,
                           uint8_t &minute,
                           uint8_t &second,
                           uint16_t &millisecond)
{
    if(static_cast<witmotion_packet_id>(packet.id_byte) != pidRTC)
        return;
    year = packet.datastore.raw[0];
    month = packet.datastore.raw[1];
    day = packet.datastore.raw[2];
    hour = packet.datastore.raw[3];
    minute = packet.datastore.raw[4];
    second = packet.datastore.raw[5];
    millisecond = static_cast<uint16_t>(packet.datastore.raw_cells[3]);
}

}

#include "witmotion/jy901-uart.h"

namespace witmotion
{
namespace jy901
{

using namespace Qt;

const std::set<witmotion_packet_id> QWitmotionJY901Sensor::registered_types =
{
    pidAcceleration,
    pidAngularVelocity,
    pidAngles,
    pidMagnetometer,
    pidOrientation,
    pidAltimeter,
    pidRTC,
    pidDataPortStatus
};

const std::set<witmotion_packet_id> *QWitmotionJY901Sensor::RegisteredPacketTypes()
{
    return &registered_types;
}

void QWitmotionJY901Sensor::SetMeasurements(const bool realtime_clock,
                                            const bool acceleration,
                                            const bool angular_velocity,
                                            const bool euler_angles,
                                            const bool magnetometer,
                                            const bool orientation,
                                            const bool port_status,
                                            const bool altimeter)
{
    uint8_t measurement_setting_low = 0x00;
    uint8_t measurement_setting_high = 0x00;
    realtime_clock ? measurement_setting_low |= 0x01 : measurement_setting_low &= ~(0x01);
    acceleration ? measurement_setting_low |= (0x01 << 1) : measurement_setting_low &= ~(0x01 << 1);
    angular_velocity ? measurement_setting_low |= (0x01 << 2) : measurement_setting_low &= ~(0x01 << 2);
    euler_angles ? measurement_setting_low |= (0x01 << 3) : measurement_setting_low &= ~(0x01 << 3);
    magnetometer ? measurement_setting_low |= (0x01 << 4) : measurement_setting_low &= ~(0x01 << 4);
    port_status ? measurement_setting_low |= (0x01 << 5) : measurement_setting_low &= ~(0x01 << 5);
    altimeter ? measurement_setting_low |= (0x01 << 6) : measurement_setting_low &= ~(0x01 << 6);
    orientation ? measurement_setting_high |= (0x01 << 1) : measurement_setting_high &= ~(0x01 << 1);
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridOutputValueSet;
    config_packet.setting.raw[0] = measurement_setting_low;
    config_packet.setting.raw[1] = measurement_setting_high;
    emit SendConfig(config_packet);
    sleep(1);
}

QWitmotionJY901Sensor::QWitmotionJY901Sensor(const QString device,
                                             const QSerialPort::BaudRate rate,
                                             const uint32_t polling_period):
    witmotion::wt901::QWitmotionWT901Sensor(device, rate, polling_period)
{

}

}
}

#include "witmotion/wt901-uart.h"

namespace witmotion
{
namespace wt901
{

using namespace Qt;

const std::set<witmotion_packet_id> QWitmotionWT901Sensor::registered_types =
{
    pidAcceleration,
    pidAngularVelocity,
    pidAngles,
    pidMagnetometer,
    pidOrientation,
    pidRTC,
    pidDataPortStatus
};

void QWitmotionWT901Sensor::UnlockConfiguration()
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridUnlockConfiguration;
    config_packet.setting.raw[0] = 0x88;
    config_packet.setting.raw[1] = 0xB5;
    ttyout << "Configuration ROM: lock removal started" << ENDL;
    emit SendConfig(config_packet);
    sleep(1);
}

const std::set<witmotion_packet_id> *QWitmotionWT901Sensor::RegisteredPacketTypes()
{
    return &registered_types;
}

void QWitmotionWT901Sensor::Start()
{
    ttyout << "Running reader thread" << ENDL;
    emit RunReader();
}

void QWitmotionWT901Sensor::Calibrate()
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridCalibrate;
    config_packet.setting.raw[0] = 0x01;
    config_packet.setting.raw[1] = 0x00;
    ttyout << "Entering spatial calibration, please hold the sensor in fixed position for 5 seconds" << ENDL;
    emit SendConfig(config_packet);
    sleep(5);
    config_packet.setting.raw[0] = 0x00;
    ttyout << "Exiting spatial calibration mode" << ENDL;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::CalibrateMagnetometer()
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridCalibrate;
    config_packet.setting.raw[0] = 0x07;
    config_packet.setting.raw[1] = 0x00;
    ttyout << "Entering magnetic calibration, please hold the sensor in fixed position for 5 seconds" << ENDL;
    emit SendConfig(config_packet);
    sleep(5);
    config_packet.setting.raw[0] = 0x00;
    ttyout << "Exiting magnetic calibration mode" << ENDL;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::SetBaudRate(const QSerialPort::BaudRate &rate)
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridPortBaudRate;
    port_rate = rate;
    config_packet.setting.raw[0] = witmotion_baud_rate(port_rate);
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::SetPollingRate(const int32_t hz)
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridOutputFrequency;
    config_packet.setting.raw[0] = witmotion_output_frequency(hz);
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::ConfirmConfiguration()
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridSaveSettings;
    config_packet.setting.raw[0] = 0x00;
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::SetOrientation(const bool vertical)
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridInstallationDirection;
    config_packet.setting.raw[0] = vertical ? 0x01 : 0x00;
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::ToggleDormant()
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridStandbyMode;
    config_packet.setting.raw[0] = 0x01;
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::SetGyroscopeAutoRecalibration(const bool recalibrate)
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridGyroscopeAutoCalibrate;
    config_packet.setting.raw[0] = recalibrate ? 0x00 : 0x01;
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::SetAxisTransition(const bool axis9)
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridTransitionAlgorithm;
    config_packet.setting.raw[0] = axis9 ? 0x00 : 0x01;
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::SetLED(const bool on)
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridLED;
    config_packet.setting.raw[0] = on ? 0x00 : 0x01;
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::SetMeasurements(const bool realtime_clock,
                                            const bool acceleration,
                                            const bool angular_velocity,
                                            const bool euler_angles,
                                            const bool magnetometer,
                                            const bool orientation,
                                            const bool port_status)
{
    uint8_t measurement_setting_low = 0x00;
    uint8_t measurement_setting_high = 0x00;
    realtime_clock ? measurement_setting_low |= 0x01 : measurement_setting_low &= ~(0x01);
    acceleration ? measurement_setting_low |= (0x01 << 1) : measurement_setting_low &= ~(0x01 << 1);
    angular_velocity ? measurement_setting_low |= (0x01 << 2) : measurement_setting_low &= ~(0x01 << 2);
    euler_angles ? measurement_setting_low |= (0x01 << 3) : measurement_setting_low &= ~(0x01 << 3);
    magnetometer ? measurement_setting_low |= (0x01 << 4) : measurement_setting_low &= ~(0x01 << 4);
    port_status ? measurement_setting_low |= (0x01 << 5) : measurement_setting_low &= ~(0x01 << 5);
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

void QWitmotionWT901Sensor::CalculateAccelerationBias(witmotion_config_packet &packet,
                                                      const float bias)
{
    /*
        - .setting.raw[0] is fine tuning part of bias from 0.0 to -0.25, 256 grades uint8_t
        - .setting.raw[1] is rough tuning part of bias, signed integer, 127 grades, int8_t
        - Calibration coefficient is 4, so the rough bias is <abs(val) - 0.25> * 4 reduced
          down to 127
    */
    float rough_bias = std::trunc(bias / 0.25);
    float fine_bias = (bias < 0) ? 0.25 - std::abs(std::fmod(bias, 0.25)) : std::abs(std::fmod(bias, 0.25));
    if(std::abs(rough_bias) > 127.f)
        rough_bias = (rough_bias < 0) ? -127.f : 127.f;
    fine_bias *= (255.0 / 0.25);
    int8_t int_rough_bias = static_cast<int8_t>(rough_bias);
    int8_t int_fine_bias = static_cast<int8_t>(fine_bias);
    ttyout << "Acceleration bias renderer delivered rough part "
           << int_rough_bias << " (" << rough_bias << "), fine part "
           << int_fine_bias << " (" << fine_bias << ")" << ENDL;
    packet.setting.raw[0] = int_fine_bias;
    packet.setting.raw[1] = int_rough_bias;
}

void QWitmotionWT901Sensor::SetAccelerationBias(float x, float y, float z)
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    ttyout << "Setting up X acceleration bias" << ENDL;
    config_packet.address_byte = ridAccelerationBiasX;
    CalculateAccelerationBias(config_packet, x);
    emit SendConfig(config_packet);
    sleep(1);
    ttyout << "Setting up Y acceleration bias" << ENDL;
    config_packet.address_byte = ridAccelerationBiasY;
    CalculateAccelerationBias(config_packet, y);
    emit SendConfig(config_packet);
    sleep(1);
    ttyout << "Setting up Z acceleration bias" << ENDL;
    config_packet.address_byte = ridAccelerationBiasZ;
    CalculateAccelerationBias(config_packet, z);
    emit SendConfig(config_packet);
    sleep(1);
}

void QWitmotionWT901Sensor::SetI2CAddress(const uint8_t address)
{
    if(address > 0x7F)
        emit ErrorOccurred("I2C address is hexadecimal int, 7 bits long. Dropping request");
    else
    {
        witmotion_config_packet config_packet;
        config_packet.header_byte = WITMOTION_CONFIG_HEADER;
        config_packet.key_byte = WITMOTION_CONFIG_KEY;
        ttyout << "Setting up I2C bus connection address" << ENDL;
        config_packet.address_byte = ridIICAddress;
        config_packet.setting.raw[0] = address;
        config_packet.setting.raw[1] = 0x00;
        emit SendConfig(config_packet);
        sleep(1);
    }
}

void QWitmotionWT901Sensor::SetRTC(const QDateTime datetime)
{
    if(!datetime.isValid())
        emit ErrorOccurred("Invalid date string specified. Dropping request");
    else
    {
        witmotion_config_packet config_packet;
        config_packet.header_byte = WITMOTION_CONFIG_HEADER;
        config_packet.key_byte = WITMOTION_CONFIG_KEY;
        ttyout << "Setting up RTC date/time origin" << ENDL;
        config_packet.address_byte = ridTimeMilliseconds;
        uint16_t msec = static_cast<uint16_t>(datetime.time().msec());
        std::copy(&msec, &msec + 1, config_packet.setting.raw);
        emit SendConfig(config_packet);
        usleep(100000);
        config_packet.address_byte = ridTimeMinuteSecond;
        config_packet.setting.raw[0] = static_cast<uint8_t>(datetime.time().minute());
        config_packet.setting.raw[1] = static_cast<uint8_t>(datetime.time().second());
        emit SendConfig(config_packet);
        sleep(1);
        config_packet.address_byte = ridTimeDayHour;
        config_packet.setting.raw[0] = static_cast<uint8_t>(datetime.date().day());
        config_packet.setting.raw[1] = static_cast<uint8_t>(datetime.time().hour());
        emit SendConfig(config_packet);
        sleep(1);
        config_packet.address_byte = ridTimeYearMonth;
        config_packet.setting.raw[0] = static_cast<int8_t>(datetime.date().year() - 2000);
        config_packet.setting.raw[1] = static_cast<uint8_t>(datetime.date().month());
        emit SendConfig(config_packet);
        sleep(1);
    }

}

QWitmotionWT901Sensor::QWitmotionWT901Sensor(const QString device,
                                             const QSerialPort::BaudRate rate,
                                             const uint32_t polling_period):
    QAbstractWitmotionSensorController(device, rate)
{
    ttyout << "Creating multithreaded interface for Witmotion WT901 IMU sensor connected to "
           << port_name
           << " at "
           << static_cast<int32_t>(port_rate)
           << " baud"
           << ENDL;
    reader->SetSensorPollInterval(polling_period);
}

}
}

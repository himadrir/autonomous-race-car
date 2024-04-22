#ifndef WITMOTION_WT901
#define WITMOTION_WT901

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDateTime>

#include <iostream>
#include <string>
#include <cmath>

#include <unistd.h>

#include "witmotion/types.h"
#include "witmotion/util.h"
#include "witmotion/serial.h"

namespace witmotion
{
namespace wt901
{

class QWitmotionWT901Sensor: public QAbstractWitmotionSensorController
{
    Q_OBJECT
private:
    static const std::set<witmotion_packet_id> registered_types;
protected:
    virtual void CalculateAccelerationBias(witmotion_config_packet& packet, const float bias);
public:
    virtual const std::set<witmotion_packet_id>* RegisteredPacketTypes();
    virtual void Start();
    virtual void UnlockConfiguration();
    virtual void Calibrate();
    virtual void CalibrateMagnetometer();
    virtual void SetBaudRate(const QSerialPort::BaudRate& rate);
    virtual void SetPollingRate(const int32_t hz);
    virtual void SetOrientation(const bool vertical = false);
    virtual void ToggleDormant();
    virtual void SetGyroscopeAutoRecalibration(const bool recalibrate = true);
    virtual void SetAxisTransition(const bool axis9 = true);
    virtual void SetLED(const bool on = true);
    virtual void SetMeasurements(const bool realtime_clock = false,
                                 const bool acceleration = true,
                                 const bool angular_velocity = true,
                                 const bool euler_angles = true,
                                 const bool magnetometer = true,
                                 const bool orientation = false,
                                 const bool port_status = false);
    virtual void SetAccelerationBias(float x,
                                     float y,
                                     float z);
    virtual void SetI2CAddress(const uint8_t address);
    virtual void SetRTC(const QDateTime datetime);
    virtual void ConfirmConfiguration();
    QWitmotionWT901Sensor(const QString device,
                          const QSerialPort::BaudRate rate,
                          const uint32_t polling_period = 50);
};

}
}

#endif

#ifndef WITMOTION_JY901
#define WITMOTION_JY901

#include <QSerialPort>
#include <QSerialPortInfo>

#include <iostream>
#include <string>
#include <cmath>

#include <unistd.h>

#include "witmotion/types.h"
#include "witmotion/util.h"
#include "witmotion/serial.h"
#include "witmotion/wt901-uart.h"

namespace witmotion
{
namespace jy901
{

class QWitmotionJY901Sensor: public witmotion::wt901::QWitmotionWT901Sensor
{   Q_OBJECT
private:
    static const std::set<witmotion_packet_id> registered_types;
public:
    virtual const std::set<witmotion_packet_id>* RegisteredPacketTypes();
    virtual void SetMeasurements(const bool realtime_clock = false,
                                 const bool acceleration = true,
                                 const bool angular_velocity = true,
                                 const bool euler_angles = true,
                                 const bool magnetometer = true,
                                 const bool orientation = false,
                                 const bool port_status = false,
                                 const bool altimeter = true);
    QWitmotionJY901Sensor(const QString device,
                          const QSerialPort::BaudRate rate,
                          const uint32_t polling_period = 50);
};

}
}
#endif

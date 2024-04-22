#ifndef WITMOTION_WT31N
#define WITMOTION_WT31N

#include <QSerialPort>
#include <QSerialPortInfo>

#include <iostream>
#include <string>

#include <unistd.h>

#include "witmotion/types.h"
#include "witmotion/util.h"
#include "witmotion/serial.h"

namespace witmotion
{
namespace wt31n
{

class QWitmotionWT31NSensor: public QAbstractWitmotionSensorController
{
    Q_OBJECT
private:
    static const std::set<witmotion_packet_id> registered_types;
public:
    virtual const std::set<witmotion_packet_id>* RegisteredPacketTypes();
    virtual void Start();
    virtual void Calibrate();
    virtual void SetBaudRate(const QSerialPort::BaudRate& rate);
    virtual void SetPollingRate(const uint32_t hz);
    QWitmotionWT31NSensor(const QString device,
                          const QSerialPort::BaudRate rate,
                          const uint32_t polling_period = 50);
};

}
}

#endif

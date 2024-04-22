#ifndef WITMOTION_SERIAL_H
#define WITMOTION_SERIAL_H

#include "witmotion/types.h"
#include "witmotion/util.h"

#include <QtCore>
#include <QSerialPort>

#include <list>

namespace witmotion
{

class QBaseSerialWitmotionSensorReader: public QAbstractWitmotionSensorReader
{
    Q_OBJECT
private:
    QString port_name;
    QSerialPort* witmotion_port;
    QSerialPort::BaudRate port_rate;
    qint64 last_avail;
    quint16 avail_rep_count;
    uint8_t raw_data[128];
    bool validate;
    bool user_defined_return_interval;
    uint32_t return_interval;
protected:
    QTextStream ttyout;
    QTimer* poll_timer;
    QMetaObject::Connection timer_connection;
    QMetaObject::Connection config_connection;
    enum read_state_t
    {
        rsUnknown,
        rsClear,
        rsRead
    }read_state;
    witmotion_typed_packets packets;
    witmotion_typed_bytecounts counts;
    witmotion_packet_id read_cell;

    volatile bool configuring;
    std::list<witmotion_config_packet> configuration;
    virtual void ReadData();
    virtual void Configure();
    virtual void SendConfig(const witmotion_config_packet& packet);
public:
    void SetBaudRate(const QSerialPort::BaudRate& rate);
    QBaseSerialWitmotionSensorReader(const QString device, const QSerialPort::BaudRate rate);
    virtual ~QBaseSerialWitmotionSensorReader();
    virtual void RunPoll();
    virtual void Suspend();
    void ValidatePackets(const bool value);
    void SetSensorPollInterval(const uint32_t ms);
};

class QAbstractWitmotionSensorController: public QObject
{
    Q_OBJECT
private:
    QThread reader_thread;
protected:
    QString port_name;
    QSerialPort::BaudRate port_rate;
    QBaseSerialWitmotionSensorReader* reader;
    QTextStream ttyout;
public:
    virtual const std::set<witmotion_packet_id>* RegisteredPacketTypes() = 0;
    QAbstractWitmotionSensorController(const QString tty_name, const QSerialPort::BaudRate rate);
    virtual void Start() = 0;
    virtual ~QAbstractWitmotionSensorController();
    virtual void Calibrate() = 0;
    virtual void SetBaudRate(const QSerialPort::BaudRate& rate) = 0;
    void SetValidation(const bool validate);
public slots:
    virtual void Packet(const witmotion_datapacket& packet);
    virtual void Error(const QString& description);
signals:
    void RunReader();
    void ErrorOccurred(const QString& description);
    void Acquired(const witmotion_datapacket& packet);
    void SendConfig(const witmotion_config_packet& packet);
};

}
#endif

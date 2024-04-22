#ifndef WITMOTION_MESSAGE_ENUMERATOR
#define WITMOTION_MESSAGE_ENUMERATOR

#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QFile>
#include <QIODevice>

#include <iostream>
#include <string>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "witmotion/serial.h"

namespace witmotion
{

class QGeneralSensorController: public QObject
{
    Q_OBJECT
private:
    size_t packets;
    QString port_name;
    QSerialPort::BaudRate port_rate;
    QThread reader_thread;
    QBaseSerialWitmotionSensorReader* reader;
    QTextStream ttyout;
    witmotion_typed_bytecounts counts;
    size_t unknown_ids;
    bool log_set;
    QString log_name;
    QStringList log;
    QFile* logfile;
    QSet<uint8_t> unknown;

    void BuildLog();
public:
    QGeneralSensorController(const QString port, const QSerialPort::BaudRate rate);
    virtual ~QGeneralSensorController();
    void Start();
    void SetLog(const QString name);
    void SetInterval(uint32_t ms);
public slots:
    void Packet(const witmotion_datapacket& packet);
    void Error(const QString& description);
signals:
    void RunReader();
};

}
#endif

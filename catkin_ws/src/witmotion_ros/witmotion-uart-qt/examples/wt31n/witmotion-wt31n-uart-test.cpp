#include <QtCore>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QTimerEvent>
#include <QCommandLineParser>
#include <QCommandLineOption>

#include <iostream>
#include <string>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

enum witmotion_packet_id
{
    widAccel = 0x51,
    widAngles = 0x53
};

struct witmotion_accel
{
    const unsigned char packet_header = 0x55;
    const witmotion_packet_id packet_id = widAccel;
    double a_x;
    double a_y;
    double a_z;
    double temp;
    unsigned char crc;
    bool valid;
};

struct witmotion_angle
{
    const unsigned char packet_header = 0x55;
    const witmotion_packet_id packet_id = widAngles;
    double roll;
    double pitch;
    double yaw;
    double temp;
    unsigned char crc;
    bool valid;
};

class ThreadedReader: public QObject
{   Q_OBJECT
private:
    QString port_device;
    QSerialPort* witmotion_port;
    QTimer* timer;
    bool highbaud;
    qint64 last_avail;
    quint16 avail_rep_count;
    unsigned char raw_data[128];
    enum read_state_t
    {
        rsClear,
        rsAccel,
        rsAngles,
        rsUnknown
    }read_state;
    witmotion_accel accel;
    witmotion_angle angles;
    unsigned char raw_accel[9];
    qint64 current_raw_accel_byte;
    unsigned char raw_angles[9];
    qint64 current_raw_angles_byte;

    void ProcessAccel()
    {
        accel.crc = raw_accel[8];
        unsigned char crc = accel.packet_header + accel.packet_id;
        for(qint64 i = 0; i < 8; i++)
            crc += raw_accel[i];
        accel.valid = (crc == accel.crc);
        signed short* num = reinterpret_cast<signed short*>(raw_accel);
        accel.a_x = static_cast<float>(*num) / 32768.f * 16.f * 9.8;
        num = reinterpret_cast<signed short*>(raw_accel + 2);
        accel.a_y = static_cast<float>(*num) / 32768.f * 16.f * 9.8;
        num = reinterpret_cast<signed short*>(raw_accel + 4);
        accel.a_z = static_cast<float>(*num) / 32768.f * 16.f * 9.8;
        num = reinterpret_cast<signed short*>(raw_accel + 6);
        accel.temp = static_cast<float>(*num) / 340.f;
        std::cout << "Acceleration [X, Y, Z]: " << accel.a_x << ", " << accel.a_y << ", " << accel.a_z << std::endl;
        emit AccelAcquired(accel);
    }

    void ProcessAngles()
    {
        angles.crc = raw_angles[8];
        unsigned char crc = angles.packet_header + angles.packet_id;
        for(qint64 i = 0; i < 8; i++)
            crc += raw_angles[i];
        angles.valid = (crc == angles.crc);
        signed short* num = reinterpret_cast<signed short*>(raw_angles);
        angles.roll = static_cast<float>(*num) / 32768.f * 180;
        num = reinterpret_cast<signed short*>(raw_angles + 2);
        angles.pitch = static_cast<float>(*num) / 32768.f * 180;
        num = reinterpret_cast<signed short*>(raw_angles + 4);
        angles.yaw = static_cast<float>(*num) / 32768.f * 180;
        num = reinterpret_cast<signed short*>(raw_angles + 6);
        angles.temp = static_cast<float>(*num) / 340.f;
        std::cout << "Angles over [X, Y, Z]: " << angles.roll << ", " << angles.pitch << ", " << angles.yaw << ", T = " << angles.temp << std::endl;
        emit AnglesAcquired(angles);
    }

private slots:
    void DataReader()
    {
        qint64 bytes_read;
        qint64 bytes_avail = witmotion_port->bytesAvailable();
        if(bytes_avail == last_avail)
        {
            if(++avail_rep_count > 3)
            {
                std::cout << "No data acquired during last 3 iterations, please check the baudrate!" << std::endl;
            }
        }
        else
        {
            last_avail = bytes_avail;
            avail_rep_count = 0;
        }
        if(bytes_avail > 0)
        {
            bytes_read = witmotion_port->read(reinterpret_cast<char*>(raw_data), 128);
            for(qint64 i = 0; i < bytes_read; i++)
            {
                unsigned char current_byte = raw_data[i];
                switch(read_state)
                {
                case rsClear:
                    read_state = (current_byte == 0x55) ? rsUnknown : rsClear;
                    break;
                case rsUnknown:
                    if(current_byte == 0x51)
                    {
                        current_raw_accel_byte = 0;
                        read_state = rsAccel;
                    }
                    else if(current_byte == 0x53)
                    {
                        current_raw_angles_byte = 0;
                        read_state = rsAngles;
                    }
                    else
                        read_state= rsClear;
                    break;
                case rsAccel:
                    raw_accel[current_raw_accel_byte++] = current_byte;
                    if(current_raw_accel_byte == 9)
                    {
                        ProcessAccel();
                        read_state = rsClear;
                    }
                    break;
                case rsAngles:
                    raw_angles[current_raw_angles_byte++] = current_byte;
                    if(current_raw_angles_byte == 9)
                    {
                        ProcessAngles();
                        read_state = rsClear;
                    }
                    break;
                }
            }
        }
    }
public:
    ThreadedReader(const QString device, const bool highspeed):
        port_device(device),
        witmotion_port(nullptr),
        timer(nullptr),
        highbaud(highspeed),
        last_avail(0),
        avail_rep_count(0),
        read_state(rsClear),
        current_raw_accel_byte(0),
        current_raw_angles_byte(0)
    {
    }
    virtual ~ThreadedReader()
    {
        delete timer;
        std::cout << "Closing port" << std::endl;
        witmotion_port->close();
        delete witmotion_port;
    }
signals:
    void AccelAcquired(const witmotion_accel& accel);
    void AnglesAcquired(const witmotion_angle& angles);
public slots:
    void RunTimer()
    {
        witmotion_port = new QSerialPort(port_device);
        if(highbaud)
            witmotion_port->setBaudRate(QSerialPort::Baud115200, QSerialPort::Direction::AllDirections);
        else
            witmotion_port->setBaudRate(QSerialPort::Baud9600, QSerialPort::Direction::AllDirections);
        std::cout << "Opening /dev/" << witmotion_port->portName().toStdString() << " at " << witmotion_port->baudRate() << " baud" << std::endl;
        witmotion_port->setStopBits(QSerialPort::OneStop);
        witmotion_port->setParity(QSerialPort::NoParity);
        witmotion_port->setFlowControl(QSerialPort::FlowControl::NoFlowControl);
        if(!witmotion_port->open(QIODevice::ReadWrite))
        {
            std::cout << "Error opening the port, please check permissions and your username in \'dialout\' group!" << std::endl;
            std::exit(1);
        }
        timer = new QTimer(this);
        timer->setTimerType(Qt::TimerType::PreciseTimer);
        timer->setInterval((witmotion_port->baudRate() == QSerialPort::Baud9600) ? 50 : 5);
        connect(timer, &QTimer::timeout, this, &ThreadedReader::DataReader);
        std::cout << "Instantiating timer at " << timer->interval() << " ms" << std::endl;
        timer->start();
    }
};

class ThreadedReaderController:  public QObject
{   Q_OBJECT
private:
    QThread UARTThread;
    ThreadedReader* reader;
public:
    ThreadedReaderController(QString device, bool highspeed):
        UARTThread(dynamic_cast<QObject*>(this))
    {
        reader = new ThreadedReader(device, highspeed);
        reader->moveToThread(&UARTThread);
        connect(&UARTThread, &QThread::finished, reader, &QObject::deleteLater);
        connect(this, &ThreadedReaderController::init_uart, reader, &ThreadedReader::RunTimer);
        UARTThread.start();
    }
    virtual ~ThreadedReaderController()
    {
        std::cout << "Stopping thread" << std::endl;
        UARTThread.quit();
        std::cout << "Waiting for signals" << std::endl;
        UARTThread.wait(10000);
        std::cout << "Releasing thread" << std::endl;
        //delete reader;
    }
    void run_thread()
    {
        std::cout << "Running reader thread" << std::endl;
        emit init_uart();
    }
signals:
    void init_uart();
};

void handle_shutdown(int s)
{
    std::cout << std::endl;
    QCoreApplication::exit(0);
}

int main(int argc, char** args)
{
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_shutdown;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    QCoreApplication app(argc, args);
    QCommandLineParser parser;
    parser.setApplicationDescription("WITMOTION WT31N UART TEST PROGRAM");
    parser.addHelpOption();
    QCommandLineOption BaudRateOption(QStringList() << "b" << "baudrate",
                                      "Baudrate to set up the port",
                                      "9600 or 115200",
                                      "9600");
    QCommandLineOption DeviceNameOption(QStringList() << "d" << "device",
                                        "Port serial device name, without \'/dev\'",
                                        "ttyUSB0",
                                        "ttyUSB0");
    parser.addOption(BaudRateOption);
    parser.addOption(DeviceNameOption);
    parser.process(app);

    ThreadedReaderController controller(parser.value(DeviceNameOption), (parser.value(BaudRateOption) == "115200"));
    controller.setParent(dynamic_cast<QObject*>(&app));
    controller.run_thread();

    return app.exec();
}

#include "witmotion-wt31n-uart-test.moc"

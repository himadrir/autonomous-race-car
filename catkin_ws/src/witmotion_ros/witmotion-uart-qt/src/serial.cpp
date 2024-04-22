#include "witmotion/serial.h"
#include <exception>
#include <unistd.h>

namespace witmotion
{

void QBaseSerialWitmotionSensorReader::ReadData()
{
    if(configuring)
        return;
    qint64 bytes_read;
    qint64 bytes_avail = witmotion_port->bytesAvailable();
    if(bytes_avail == last_avail)
    {
        if(++avail_rep_count > 3)
            emit Error("No data acquired during last 3 iterations, please check the baudrate!");
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
            uint8_t current_byte = raw_data[i];
            if(read_state == rsClear)
            {
                read_state = (current_byte == WITMOTION_HEADER_BYTE) ? rsUnknown : rsClear;
            }
            else if(read_state == rsUnknown)
            {
                if(id_registered(current_byte))
                {
                    read_cell = static_cast<witmotion_packet_id>(current_byte);
                    counts[read_cell] = 0;
                    packets[read_cell].header_byte = WITMOTION_HEADER_BYTE;
                    packets[read_cell].id_byte = read_cell;
                    read_state = rsRead;
                }
                else
                    read_state = rsClear;
            }
            else
            {
                if(counts[read_cell] == 8)
                {
                    packets[read_cell].crc = current_byte;
                    uint8_t current_crc = packets[read_cell].header_byte + packets[read_cell].id_byte;
                    for(uint8_t i = 0; i < 8; i++)
                        current_crc += packets[read_cell].datastore.raw[i];
                    if(!validate || (current_crc == packets[read_cell].crc))
                        emit Acquired(packets[read_cell]);
                    read_state = rsClear;
                }
                else
                    packets[read_cell].datastore.raw[counts[read_cell]++] = current_byte;
            }
        }
    }
}

void QBaseSerialWitmotionSensorReader::Configure()
{
    if(configuration.empty())
        return;
    configuring = true;
    ttyout << "Configuration task detected, " << configuration.size() << " commands in list, configuring sensor..." << ENDL;
    bool error = false;
    for(auto i = configuration.begin(); i != configuration.end(); i++)
    {
        witmotion_config_packet packet = (*i);
        static uint8_t serial_datapacket[5];
        serial_datapacket[0] = packet.header_byte;
        serial_datapacket[1] = packet.key_byte;
        serial_datapacket[2] = packet.address_byte;
        serial_datapacket[3] = packet.setting.raw[0];
        serial_datapacket[4] = packet.setting.raw[1];
        quint64 written;
        ttyout << "Sending configuration packet " << HEX << "0x" << packet.address_byte << DEC << ENDL;
        written = witmotion_port->write(reinterpret_cast<const char*>(serial_datapacket), 5);
        witmotion_port->waitForBytesWritten();
        if(written != 5)
        {
            error = true;
            break;
        }
        ttyout << "Configuration packet sent, flushing buffers..." << ENDL;
        witmotion_port->flush();
    }
    ttyout << "Configuration completed" << ENDL;
    configuration.clear();
    configuring = false;
    if(error)
        emit Error("Error occurred when reconfiguring sensor!");
}

void QBaseSerialWitmotionSensorReader::SendConfig(const witmotion_config_packet &packet)
{
    configuration.push_back(packet);
}

void QBaseSerialWitmotionSensorReader::SetBaudRate(const QSerialPort::BaudRate &rate)
{
    port_rate = rate;
}

QBaseSerialWitmotionSensorReader::QBaseSerialWitmotionSensorReader(const QString device, const QSerialPort::BaudRate rate):
    port_name(device),
    witmotion_port(nullptr),
    port_rate(rate),
    last_avail(0),
    avail_rep_count(0),
    validate(false),
    user_defined_return_interval(false),
    return_interval(50),
    ttyout(stdout),
    poll_timer(nullptr),
    read_state(rsClear),
    configuring(false)
{
    qRegisterMetaType<witmotion_datapacket>("witmotion_datapacket");
    qRegisterMetaType<witmotion_config_packet>("witmotion_config_packet");
}

QBaseSerialWitmotionSensorReader::~QBaseSerialWitmotionSensorReader()
{
    if(poll_timer != nullptr)
        delete poll_timer;
    ttyout << "Closing TTL connection" << ENDL;
    if(witmotion_port != nullptr)
    {
        witmotion_port->close();
        delete witmotion_port;
    }
}

void QBaseSerialWitmotionSensorReader::RunPoll()
{
    witmotion_port = new QSerialPort(port_name);
    witmotion_port->setBaudRate(port_rate, QSerialPort::Direction::AllDirections);
    witmotion_port->setStopBits(QSerialPort::OneStop);
    witmotion_port->setParity(QSerialPort::NoParity);
    witmotion_port->setFlowControl(QSerialPort::FlowControl::NoFlowControl);
    ttyout << "Opening device \"" << witmotion_port->portName() << "\" at " << witmotion_port->baudRate() << " baud" << ENDL;
    if(!witmotion_port->open(QIODevice::ReadWrite))
    {
        emit Error("Error opening the port!");
        return;
    }
    poll_timer = new QTimer(this);
    poll_timer->setTimerType(Qt::TimerType::PreciseTimer);
    if(!user_defined_return_interval)
        poll_timer->setInterval((port_rate == QSerialPort::Baud9600) ? 50 : 30);
    else
        poll_timer->setInterval(return_interval);
    timer_connection = connect(poll_timer, &QTimer::timeout, this, &QBaseSerialWitmotionSensorReader::ReadData);
    config_connection = connect(poll_timer, &QTimer::timeout, this, &QBaseSerialWitmotionSensorReader::Configure);
    ttyout << "Instantiating timer at " << poll_timer->interval() << " ms" << ENDL;
    poll_timer->start();
}

void QBaseSerialWitmotionSensorReader::Suspend()
{
    disconnect(timer_connection);
    disconnect(config_connection);
    if(poll_timer != nullptr)
        delete poll_timer;
    if(witmotion_port != nullptr)
    {
        witmotion_port->close();
        delete witmotion_port;
    }
    ttyout << "Suspending TTL connection, please emit RunPoll() again to proceed!" << ENDL;
    poll_timer = nullptr;
    witmotion_port = nullptr;
}

void QBaseSerialWitmotionSensorReader::ValidatePackets(const bool value)
{
    validate = value;
}

void QBaseSerialWitmotionSensorReader::SetSensorPollInterval(const uint32_t ms)
{
    user_defined_return_interval = true;
    return_interval = ms;
}

QAbstractWitmotionSensorController::QAbstractWitmotionSensorController(const QString tty_name, const QSerialPort::BaudRate rate):
    reader_thread(dynamic_cast<QObject*>(this)),
    port_name(tty_name),
    port_rate(rate),
    reader(nullptr),
    ttyout(stdout)
{
    reader = new QBaseSerialWitmotionSensorReader(port_name, port_rate);
    reader->moveToThread(&reader_thread);
    connect(&reader_thread, &QThread::finished, reader, &QObject::deleteLater);
    connect(this, &QAbstractWitmotionSensorController::RunReader, reader, &QAbstractWitmotionSensorReader::RunPoll);
    connect(reader, &QAbstractWitmotionSensorReader::Acquired, this, &QAbstractWitmotionSensorController::Packet);
    connect(reader, &QAbstractWitmotionSensorReader::Error, this, &QAbstractWitmotionSensorController::Error);
    connect(this, &QAbstractWitmotionSensorController::SendConfig, reader, &QAbstractWitmotionSensorReader::SendConfig);
    reader_thread.start();
}

QAbstractWitmotionSensorController::~QAbstractWitmotionSensorController()
{
    reader_thread.quit();
    reader_thread.wait(10000);
}

void QAbstractWitmotionSensorController::SetValidation(const bool validate)
{
    reader->ValidatePackets(validate);
}

void QAbstractWitmotionSensorController::Packet(const witmotion_datapacket &packet)
{
    static const std::set<witmotion_packet_id>* registered = RegisteredPacketTypes();
    if(registered->find(static_cast<witmotion_packet_id>(packet.id_byte)) == registered->end())
    {
        emit ErrorOccurred("Unregistered packet ID acquired. Please be sure that you use a proper driver class and namespace!");
        return;
    }
    emit Acquired(packet);
}

void QAbstractWitmotionSensorController::Error(const QString &description)
{
    ttyout << "Internal error occurred. Suspending the reader thread. Please check the sensor!" << ENDL;
    reader->Suspend();
    emit ErrorOccurred(description);
}

}

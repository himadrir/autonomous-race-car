#include <QtCore>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QCommandLineParser>
#include <QCommandLineOption>

#include <iostream>
#include <string>

#include <unistd.h>

int main(int argc, char** args)
{
    QString port_device = "ttyUSB0";
    bool open_highspeed = false;
    bool set_highspeed = false;
    bool reset_speed = false;
    bool calibrate = false;

    QCoreApplication app(argc, args);
    QCommandLineParser parser;
    parser.setApplicationDescription("WITMOTION WT31N Standalone Calibration Utility by twdragon");
    parser.addHelpOption();
    QCommandLineOption BaudRateOption(QStringList() << "r" << "connect-baudrate",
                                      "Baudrate for initial connection",
                                      "9600 or 115200",
                                      "9600");
    QCommandLineOption DeviceNameOption(QStringList() << "d" << "device",
                                        "Port serial device name, without \'/dev\'",
                                        "ttyUSB0",
                                        "ttyUSB0");
    QCommandLineOption SpeedResetOption(QStringList() << "b" << "set-baudrate",
                                        "Port baudrate to be set as default",
                                        "9600 or 115200",
                                        "9600");
    QCommandLineOption CalibrateOption(QStringList() << "c" << "calibrate",
                                        "Toggles calibration");
    parser.addOption(BaudRateOption);
    parser.addOption(DeviceNameOption);
    parser.addOption(SpeedResetOption);
    parser.addOption(CalibrateOption);
    parser.process(app);
    reset_speed = parser.isSet(SpeedResetOption);
    calibrate = parser.isSet(CalibrateOption);
    set_highspeed = (parser.value(SpeedResetOption) == "115200");
    open_highspeed = (parser.value(BaudRateOption) == "115200");

    std::cout << "Please make sure that your WITMOTION WT31N device is connected via USB-TTL converter to /dev/" << port_device.toStdString() << std::endl;
    QSerialPort witmotion_port(port_device);
    if(open_highspeed)
        witmotion_port.setBaudRate(QSerialPort::Baud115200, QSerialPort::Direction::AllDirections);
    else
        witmotion_port.setBaudRate(QSerialPort::Baud9600, QSerialPort::Direction::AllDirections);
    std::cout << "Opening /dev/" << witmotion_port.portName().toStdString() << " at " << witmotion_port.baudRate() << " baud" << std::endl;
    witmotion_port.setStopBits(QSerialPort::OneStop);
    witmotion_port.setParity(QSerialPort::NoParity);
    witmotion_port.setFlowControl(QSerialPort::FlowControl::NoFlowControl);
    if(!witmotion_port.open(QIODevice::ReadWrite))
    {
        std::cout << "Error opening the port, please check permissions and your username in \'dialout\' group!" << std::endl;
        std::exit(1);
    }
    sleep(2);
    quint64 written;
    if(reset_speed)
    {
        std::cout << "Baudrate reset started" << std::endl;
        static const unsigned char command_set_speed_high[5] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
        static const unsigned char command_set_speed_low[5] = {0xFF, 0xAA, 0x04, 0x02, 0x00};
        if(set_highspeed)
        {
            written = witmotion_port.write(reinterpret_cast<const char*>(command_set_speed_high), 5);
            std::cout << "Settings: port reset to 115200 baud" << std::endl;
        }
        else
        {
            written = witmotion_port.write(reinterpret_cast<const char*>(command_set_speed_low), 5);
            std::cout << "Settings: port reset to 9600 baud" << std::endl;
        }
        witmotion_port.waitForBytesWritten();
        if(written != 5)
        {
            std::cout << "Error during desired baudrate reset!" << std::endl;
            std::exit(1);
        }
        witmotion_port.flush();
        sleep(1);
        std::cout << "Reopening port..." << std::endl;
        witmotion_port.close();
        if(set_highspeed)
            witmotion_port.setBaudRate(QSerialPort::Baud115200, QSerialPort::Direction::AllDirections);
        else
            witmotion_port.setBaudRate(QSerialPort::Baud9600, QSerialPort::Direction::AllDirections);
        witmotion_port.setStopBits(QSerialPort::OneStop);
        witmotion_port.setParity(QSerialPort::NoParity);
        witmotion_port.setFlowControl(QSerialPort::FlowControl::NoFlowControl);
        if(!witmotion_port.open(QIODevice::ReadWrite))
        {
            std::cout << "Error opening the port, please check permissions and your username in \'dialout\' group!" << std::endl;
            std::exit(1);
        }
        std::cout << std::dec << std::endl << "Baudrate reset completed" << std::endl;
    }
    sleep(2);
    if(calibrate)
    {
        std::cout << "Calibration started" << std::endl;
        static const unsigned char command_calibration[5] = {0xFF, 0xAA, 0x01, 0x01, 0x00};
        written = witmotion_port.write(reinterpret_cast<const char*>(command_calibration), 5);
        if(written != 5)
        {
            std::cout << "Error during calibration!" << std::endl;
            std::exit(1);
        }
        witmotion_port.flush();
        std::cout << std::dec << "Calibration completed" << std::endl;
    }
    witmotion_port.close();
    return 0;
}

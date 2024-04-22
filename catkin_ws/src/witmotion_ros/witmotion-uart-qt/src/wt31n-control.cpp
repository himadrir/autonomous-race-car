#include "witmotion/wt31n-uart.h"

#include <QCommandLineParser>
#include <QCommandLineOption>

#include <iostream>
#include <iomanip>
#include <string>
#include <list>
#include <chrono>
#include <ctime>
#include <fstream>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

using namespace witmotion::wt31n;

double variance(const std::vector<float>& array)
{
    double sum = std::accumulate(array.begin(), array.end(), 0.f);
    double mean = sum / static_cast<double>(array.size());
    double sq_dif = 0.f;
    for(auto i = array.begin(); i != array.end(); i++)
        sq_dif += std::pow((*i) - mean, 2);
    sq_dif /= (array.size() > 1) ? static_cast<double>(array.size() - 1) : 1.f;
    return std::sqrt(sq_dif);
}

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
    app.setApplicationVersion(QString(witmotion::library_version().c_str()));
    QCommandLineParser parser;
    parser.setApplicationDescription("WITMOTION WT31N STANDALONE SENSOR CONTROLLER/MONITOR");
    parser.addHelpOption();
    QCommandLineOption BaudRateOption(QStringList() << "b" << "baudrate",
                                      "Baudrate to set up the port",
                                      "9600 or 115200",
                                      "9600");
    QCommandLineOption IntervalOption(QStringList() << "i" << "interval",
                                      "Port polling interval",
                                      "50 ms",
                                      "50");
    QCommandLineOption DeviceNameOption(QStringList() << "d" << "device",
                                        "Port serial device name, without \'/dev\'",
                                        "ttyUSB0",
                                        "ttyUSB0");
    QCommandLineOption ValidateOption("validate",
                                      "Accept only valid datapackets");
    QCommandLineOption CalibrateOption("calibrate",
                                       "Run spatial calibration");
    QCommandLineOption SetBaudRateOption(QStringList() << "set-baudrate",
                                         "Reset the connection baud rate and polling interval",
                                         "9600 or 115200",
                                         "9600");
    QCommandLineOption SetPollingRateOption(QStringList() << "set-frequency",
                                            "Set output polling frequency",
                                            "10 or 100 Hz",
                                            "10");
    QCommandLineOption CovarianceOption("covariance",
                                        "Measure spatial covariance");
    QCommandLineOption LogOption("log", "Log acquisition to sensor.log file");
    parser.addOption(BaudRateOption);
    parser.addOption(IntervalOption);
    parser.addOption(DeviceNameOption);
    parser.addOption(ValidateOption);
    parser.addOption(CalibrateOption);
    parser.addOption(CovarianceOption);
    parser.addOption(SetBaudRateOption);
    parser.addOption(SetPollingRateOption);
    parser.addOption(LogOption);
    parser.process(app);

    QSerialPort::BaudRate rate;
    QString device;

    if(parser.value(BaudRateOption) == "115200")
        rate = QSerialPort::Baud115200;
    else if(parser.value(BaudRateOption) == "9600")
        rate = QSerialPort::Baud9600;
    else
    {
        std::cout << "[WARNING] Unknown baud rate value \"" << parser.value(BaudRateOption).toStdString() << "\", falling back to 9600 baud" << std::endl;
        rate = QSerialPort::Baud9600;
    }
    device = !parser.isSet(DeviceNameOption) ? "ttyUSB0" : parser.value(DeviceNameOption);

    std::cout << "Opening device /dev/" << device.toStdString() << " at " << static_cast<int32_t>(rate) << " baud" << std::endl;

    // Creating the sensor handler
    uint32_t interval = parser.value(IntervalOption).toUInt();
    if(interval < 5)
    {
        std::cout << "Wrong port polling interval specified, falling back to 50 ms!" << std::endl;
        interval = 50;
    }
    QWitmotionWT31NSensor sensor(device, rate, interval);
    sensor.SetValidation(parser.isSet(ValidateOption));

    // Control tasks
    bool control_set_baud = parser.isSet(SetBaudRateOption);
    bool control_baud_9600 = parser.value(SetBaudRateOption) != "115200";
    bool control_calibration = parser.isSet(CalibrateOption);
    bool covariance = parser.isSet(CovarianceOption);
    bool control_set_freq = parser.isSet(SetPollingRateOption);
    uint32_t new_freq = parser.value(SetPollingRateOption).toUInt();

    // Setting up data capturing slots: mutable/immutable C++14 lambda functions
    bool first = true;
    std::vector<float> accels_x, accels_y, accels_z, rolls, pitches, times;

    QObject::connect(&sensor, &QWitmotionWT31NSensor::ErrorOccurred, [](const QString description)
    {
        std::cout << "ERROR: " << description.toStdString() << std::endl;
        QCoreApplication::exit(1);
    });

    QObject::connect(&sensor, &QWitmotionWT31NSensor::Acquired,
                     [&sensor,
                     &first,
                     &accels_x,
                     &accels_y,
                     &accels_z,
                     &rolls,
                     &pitches,
                     &times,
                     control_set_baud,
                     control_baud_9600,
                     control_calibration,
                     control_set_freq,
                     new_freq](const witmotion::witmotion_datapacket& packet)
    {
        float ax, ay, az, roll, pitch, yaw, t;
        static size_t packets = 1;
        static auto time_start = std::chrono::system_clock::now();
        if(first)
        {
            first = false;
            std::time_t timestamp_start = std::chrono::system_clock::to_time_t(time_start);
            std::cout << "Acquisition started at " << std::ctime(&timestamp_start) << std::endl << std::endl;
            std::cout << "Connection test performed sucessfully, checking for control tasks..." << std::endl;
            if(control_set_baud)
            {
                QSerialPort::BaudRate new_baud = control_baud_9600 ? QSerialPort::Baud9600 : QSerialPort::Baud115200;
                std::cout << "Resetting baud rate to " << new_baud << " baud" << std::endl;
                sensor.SetBaudRate(new_baud);
                std::cout << "Baud rate reset, please reconnect with proper port settings" << std::endl;
                QCoreApplication::exit(0);
            }
            if(control_set_freq)
            {
                std::cout << "Changing output frequency to " << new_freq << " Hz" << std::endl;
                sensor.SetPollingRate(new_freq);
                std::cout << "Sensor output frequency reset, please reconnect with proper port settings" << std::endl;
                QCoreApplication::exit(0);
            }
            if(control_calibration)
            {
                std::cout << "Entering CALIBRATION mode"
                          << std::endl
                          << "PLEASE KEEP THE SENSOR STATIC ON THE HORIZONTAL SURFACE!!!"
                          << std::endl
                          << "Calibration starts in "
                          << std::endl;
                for(size_t i = 5; i >= 1; i--)
                {
                    std::cout << i << std::endl;
                    sleep(1);
                }
                std::cout << std::endl << "Calibrating..." << std::endl;
                sensor.Calibrate();
                sleep(1);
                std::cout << "Calibration completed. Please reconnect now" << std::endl;
                QCoreApplication::exit(0);
            }
            std::cout.precision(5);
            std::cout << std::fixed;
        }
        /* NOTE: Temperature is not measured by WT31N */
        auto time_acquisition = std::chrono::system_clock::now();
        std::chrono::duration<float> elapsed_seconds = time_acquisition - time_start;
        switch (static_cast<witmotion::witmotion_packet_id>(packet.id_byte))
        {
        case witmotion::pidAcceleration:
            witmotion::decode_accelerations(packet, ax, ay, az, t);
            accels_x.push_back(ax);
            accels_y.push_back(ay);
            accels_z.push_back(az);
            std::cout << packets << "\t"
                      << "Accelerations [X|Y|Z]:\t[ "
                      << ax << " | "
                      << ay << " | "
                      << az << " ]"
                      << " in " << elapsed_seconds.count() << " s"
                      << std::endl;
            break;
        case witmotion::pidAngles:
            witmotion::decode_angles(packet, roll, pitch, yaw, t);
            rolls.push_back(roll);
            pitches.push_back(pitch);
            std::cout << packets << "\t"
                      << "Euler angles [R|P|Y]:\t[ "
                      << roll << " | "
                      << pitch << " | "
                      << yaw << " ]"
                      << " in " << elapsed_seconds.count() << " s"
                      << std::endl;
            break;
        default:
            break;
        }
        times.push_back(elapsed_seconds.count());

        packets++;
        time_start = time_acquisition;
    });

    // Start acquisition
    sensor.Start();
    std::cout << "Waiting for the first packet acquired..." << std::endl;
    int result = app.exec();

    std::cout << "Average sensor return rate "
              << std::accumulate(times.begin(), times.end(), 0.f) / times.size()
              << " s" << std::endl << std::endl;

    if(covariance)
    {
        std::cout << "Calculating noise covariance matrices..." << std::endl
                  << std::endl
                  << "Accelerations (total for " << accels_x.size() << " measurements): " << std::endl
                  << "[\t" << variance(accels_x) << "\t0.00000\t0.00000" << std::endl
                  << "\t0.00000\t" << variance(accels_y) << "\t0.00000" << std::endl
                  << "\t0.00000\t0.00000\t" << variance(accels_z) << "\t]" << std::endl
                  << std::endl
                  << "Angles (total for " << pitches.size() << " measurements): " << std::endl
                  << "[\t" << variance(rolls) << "\t0.00000\t0.00000" << std::endl
                  << "\t0.00000\t" << variance(pitches) << "\t0.00000" << std::endl
                  << "\t0.00000\t0.00000\t0.00000\t]" << std::endl
                  << std::endl;
    }

    if(parser.isSet(LogOption))
    {
        std::cout << "Writing log file to sensor.log" << std::endl;
        std::fstream logfile;
        logfile.open("sensor.log", std::ios::out|std::ios::trunc);
        logfile.precision(5);
        logfile << std::fixed;
        logfile << "WITMOTION WT31N STANDALONE SENSOR CONTROLLER/MONITOR" << std::endl << std::endl;
        auto time_start = std::chrono::system_clock::now();
        std::time_t timestamp_start = std::chrono::system_clock::to_time_t(time_start);
        logfile << "Device /dev/" << device.toStdString() << " opened at " << static_cast<int32_t>(rate) << " baud" << std::endl;
        if(accels_x.empty() || rolls.empty())
            logfile << "Raw data storage is empty, only control operations performed" << std::endl;
        else
        {
            size_t tc = 0;
            logfile << std::endl << "Measurements (only full packets logged):" << std::endl;
            for(size_t i = 0; i < std::min(accels_x.size(), rolls.size()); i++)
            {
                logfile << i + 1 << "\t"
                        << "Accelerations [X|Y|Z]:\t[ "
                        << accels_x[i] << " | "
                        << accels_y[i] << " | "
                        << accels_z[i] << " ]"
                        << " in " << times[tc++] << " s"
                        << std::endl;
                logfile << "\t"
                        << "Euler angles [R|P|Y]:\t[ "
                        << rolls[i] << " | "
                        << pitches[i] << " | "
                        << "0.00000 ]"
                        << " in " << times[tc++] << " s"
                        << std::endl;
            }
            logfile << std::endl
                    << "Acquired "
                    << std::min(accels_x.size(), rolls.size())
                    << " measurements, average reading time "
                    << std::accumulate(times.begin(), times.end(), 0.f) / times.size()
                    << " s"
                    << std::endl;
        }
        if(covariance)
        {
            logfile << std::endl
                    << "Noise covariance matrices:" << std::endl
                    << "Accelerations (total for " << accels_x.size() << " measurements): " << std::endl
                    << "[\t" << variance(accels_x) << "\t0.00000\t0.00000" << std::endl
                    << "\t0.00000\t" << variance(accels_y) << "\t0.00000" << std::endl
                    << "\t0.00000\t0.00000\t" << variance(accels_z) << "\t]" << std::endl
                    << std::endl
                    << "Angles (total for " << pitches.size() << " measurements): " << std::endl
                    << "[\t" << variance(rolls) << "\t0.00000\t0.00000" << std::endl
                    << "\t0.00000\t" << variance(pitches) << "\t0.00000" << std::endl
                    << "\t0.00000\t0.00000\t0.00000\t]" << std::endl
                    << std::endl;
        }
        logfile << "Acquisition performed at " << std::ctime(&timestamp_start) << std::endl;
        logfile.close();
    }

    return result;
}

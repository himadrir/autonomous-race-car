#include "witmotion/wt901-uart.h"

#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QString>
#include <QStringList>

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

using namespace witmotion::wt901;

void handle_shutdown(int s)
{
    std::cout << std::endl;
    QCoreApplication::exit(0);
}

int main(int argc, char** args)
{
    bool maintenance = false;

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_shutdown;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    QCoreApplication app(argc, args);
    app.setApplicationVersion(QString(witmotion::library_version().c_str()));
    QCommandLineParser parser;
    parser.setApplicationDescription("WITMOTION WT901 STANDALONE SENSOR CONTROLLER/MONITOR");
    parser.addHelpOption();
    QCommandLineOption BaudRateOption(QStringList() << "b" << "baudrate",
                                      "Baudrate to set up the port",
                                      "2400 to 115200",
                                      "9600");
    parser.addOption(BaudRateOption);
    QCommandLineOption IntervalOption(QStringList() << "i" << "interval",
                                      "Port polling interval",
                                      "50 ms",
                                      "50");
    parser.addOption(IntervalOption);
    QCommandLineOption DeviceNameOption(QStringList() << "d" << "device",
                                        "Port serial device name, without \'/dev\'",
                                        "ttyUSB0",
                                        "ttyUSB0");
    parser.addOption(DeviceNameOption);
    QCommandLineOption ValidateOption("validate",
                                      "Accept only valid datapackets");
    parser.addOption(ValidateOption);
    QCommandLineOption SetBaudRateOption(QStringList() << "set-baudrate",
                                         "Reset the connection baud rate",
                                         "2400 to 115200",
                                         "9600");
    parser.addOption(SetBaudRateOption);
    QCommandLineOption SetPollingRateOption(QStringList() << "set-frequency",
                                            "Set output polling frequency, Hz",
                                            "1 - 200 Hz",
                                            "10");
    parser.addOption(SetPollingRateOption);
    QCommandLineOption CovarianceOption("covariance",
                                        "Measure spatial covariance");
    parser.addOption(CovarianceOption);
    QCommandLineOption LogOption("log", "Log acquisition to sensor.log file");
    parser.addOption(LogOption);

    QCommandLineOption CalibrateOption("calibrate",
                                       "Run spatial calibration");
    parser.addOption(CalibrateOption);
    QCommandLineOption MagnetometerCalibrateOption("calibrate-magnetometer",
                                       "Run magnetic calibration");
    parser.addOption(MagnetometerCalibrateOption);
    QCommandLineOption BaseHorizontalOrientationOption("set-horizontal",
                                                       "Set HORIZONTAL as basic spatial orientation");
    parser.addOption(BaseHorizontalOrientationOption);
    QCommandLineOption BaseVerticalOrientationOption("set-vertical",
                                                     "Set VERTICAL as basic spatial orientation");
    parser.addOption(BaseVerticalOrientationOption);
    QCommandLineOption DormantOption("dormant",
                                     "Toggle dormancy mode, only for support test");
    parser.addOption(DormantOption);
    QCommandLineOption GyroscopeAutoRecalibrateOption("gyroscope-auto-recalibrate",
                                                      "Turn on or off gyroscope auto recalibration",
                                                      "on / off",
                                                      "on");
    parser.addOption(GyroscopeAutoRecalibrateOption);
    QCommandLineOption AxisTransitionOption("transition-axis",
                                            "Fusion axis number, 9 by default",
                                            "6 or 9",
                                            "9");
    parser.addOption(AxisTransitionOption);
    QCommandLineOption LEDOption("led",
                                 "Turn on or off sensor's internal LED indication",
                                 "on / off",
                                 "on");
    parser.addOption(LEDOption);
    QCommandLineOption DisableMeasurementOption("disable",
                                                "Disables measurements, comma-separated list. [off], default or: acceleration, velocity, angles, magnetometer, orientation, rtc, status",
                                                "velocity,rtc,...",
                                                "off");
    parser.addOption(DisableMeasurementOption);
    QCommandLineOption AccelerationBiasOption("set-acceleration-bias",
                                              "Set acceleration biases by axis [0:0:0]",
                                              "X:Y:Z",
                                              "0:0:0");
    parser.addOption(AccelerationBiasOption);
    QCommandLineOption I2CAddressOption("set-i2c-address",
                                        "Set I2C bus address of the module (HEXADECIMAL) [50]",
                                        "HEX",
                                        "50");
    parser.addOption(I2CAddressOption);
    QCommandLineOption RTCSetupOption("set-clock",
                                      "Set realtime clock to ISO 8601 datetime [yyyy-MM-dd HH:mm:ss.mss] or NOW",
                                      "DATE TIME",
                                      "NOW");
    parser.addOption(RTCSetupOption);

    parser.process(app);

    QSerialPort::BaudRate rate = static_cast<QSerialPort::BaudRate>(parser.value(BaudRateOption).toUInt());
    QString device = parser.value(DeviceNameOption);

    // Creating the sensor handler
    uint32_t interval = parser.value(IntervalOption).toUInt();
    if(interval < 5)
    {
        std::cout << "Wrong port polling interval specified, falling back to 50 ms!" << std::endl;
        interval = 50;
    }
    QWitmotionWT901Sensor sensor(device, rate, interval);
    sensor.SetValidation(parser.isSet(ValidateOption));

    // Setting up data capturing slots: mutable/immutable C++14 lambda functions
    QObject::connect(&sensor, &QWitmotionWT901Sensor::ErrorOccurred, [](const QString description)
    {
        std::cout << "ERROR: " << description.toStdString() << std::endl;
        QCoreApplication::exit(1);
    });

    std::vector<witmotion::witmotion_datapacket> acquired;

    std::vector<float> accels_x,
            accels_y,
            accels_z,
            vels_x,
            vels_y,
            vels_z,
            rolls,
            pitches,
            yaws,
            temps,
            times,
            mags_x,
            mags_y,
            mags_z,
            quat_x,
            quat_y,
            quat_z,
            quat_w;

    std::cout.precision(5);
    std::cout << std::fixed;

    QObject::connect(&sensor, &QWitmotionWT901Sensor::Acquired,
                     [maintenance,
                     &acquired,
                     &accels_x,
                     &accels_y,
                     &accels_z,
                     &vels_x,
                     &vels_y,
                     &vels_z,
                     &rolls,
                     &pitches,
                     &yaws,
                     &temps,
                     &times,
                     &mags_x,
                     &mags_y,
                     &mags_z,
                     &quat_x,
                     &quat_y,
                     &quat_z,
                     &quat_w](const witmotion::witmotion_datapacket& packet)
    {
        if(maintenance)
            return;

        float ax, ay, az, wx, wy, wz, roll, pitch, yaw, t, mx, my, mz, qx, qy, qz, qw;
        uint8_t year, month, day, hour, minute, second;
        uint16_t millisecond;
        QString uptime;
        static size_t packets = 1;
        static auto time_start = std::chrono::system_clock::now();
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
                      << az << " ], temp "
                      << t << " degrees,"
                      << " in " << elapsed_seconds.count() << " s"
                      << std::endl;
            break;
        case witmotion::pidAngularVelocity:
            witmotion::decode_angular_velocities(packet, wx, wy, wz, t);
            vels_x.push_back(wx);
            vels_y.push_back(wy);
            vels_z.push_back(wz);
            std::cout << packets << "\t"
                      << "Angular velocities [X|Y|Z]:\t[ "
                      << wx << " | "
                      << wy << " | "
                      << wz << " ], temp "
                      << t << " degrees,"
                      << " in " << elapsed_seconds.count() << " s"
                      << std::endl;
            break;
        case witmotion::pidAngles:
            witmotion::decode_angles(packet, roll, pitch, yaw, t);
            rolls.push_back(roll);
            pitches.push_back(pitch);
            yaws.push_back(yaw);
            std::cout << packets << "\t"
                      << "Euler angles [R|P|Y]:\t[ "
                      << roll << " | "
                      << pitch << " | "
                      << yaw << " ], temp "
                      << t << " degrees, "
                      << " in " << elapsed_seconds.count() << " s"
                      << std::endl;
            break;
        case witmotion::pidMagnetometer:
            witmotion::decode_magnetometer(packet, mx, my ,mz, t);
            mags_x.push_back(mx);
            mags_y.push_back(my);
            mags_z.push_back(mz);
            temps.push_back(t);
            std::cout << packets << "\t"
                      << "Magnetic field [X|Y|Z]:\t[ "
                      << mx << " | "
                      << my << " | "
                      << mz << " ], temp "
                      << t << " degrees, "
                      << " in " << elapsed_seconds.count() << " s"
                      << std::endl;
            break;
        case witmotion::pidRTC:
            witmotion::decode_realtime_clock(packet, year, month, day, hour, minute, second, millisecond);
            uptime = QString().setNum(year) + "-"
                    + QString().setNum(month) + "-"
                    + QString().setNum(day) + " "
                    + QString().setNum(hour) + ":"
                    + QString().setNum(minute) + ":"
                    + QString().setNum(second) + "."
                    + QString().setNum(millisecond);
            std::cout << packets << "\t"
                      << "Uptime / Timestamp: "
                      << uptime.toStdString()
                      << " in " << elapsed_seconds.count() << " s"
                      << std::endl;
            break;
        case witmotion::pidOrientation:
            witmotion::decode_orientation(packet, qx, qy, qz, qw);
            quat_x.push_back(qx);
            quat_y.push_back(qy);
            quat_z.push_back(qz);
            quat_w.push_back(qw);
            std::cout << packets << "\t"
                      << "Orientation quaternion [X|Y|Z|W]:\t[ "
                      << qx << " | "
                      << qy << " | "
                      << qz << " | "
                      << qw << " ]"
                      << " in " << elapsed_seconds.count() << " s"
                      << std::endl;
            break;
        case witmotion::pidDataPortStatus:
            std::cout << packets << "\t"
                      << "Data port status string: 0x"
                      << std::hex
                      << static_cast<uint32_t>(packet.datastore.raw[0]) << " 0x"
                      << static_cast<uint32_t>(packet.datastore.raw[1]) << " 0x"
                      << static_cast<uint32_t>(packet.datastore.raw[2]) << " 0x"
                      << static_cast<uint32_t>(packet.datastore.raw[3]) << " 0x"
                      << static_cast<uint32_t>(packet.datastore.raw[4]) << " 0x"
                      << static_cast<uint32_t>(packet.datastore.raw[5]) << " 0x"
                      << static_cast<uint32_t>(packet.datastore.raw[6]) << " 0x"
                      << static_cast<uint32_t>(packet.datastore.raw[7])
                      << std::dec
                      << " in " << elapsed_seconds.count() << " s"
                      << std::endl;
            break;
        default:
            break;
        }
        times.push_back(elapsed_seconds.count());

        packets++;
        acquired.push_back(packet);
        time_start = time_acquisition;
    });


    // Start acquisition
    sensor.Start();

    // Rendering control packets
    maintenance = true;
    sleep(1);
    if(parser.isSet(CalibrateOption))
    {
        std::cout << "Entering SPATIAL CALIBRATION mode"
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
        sensor.UnlockConfiguration();
        sensor.Calibrate();
        sleep(5);
        sensor.ConfirmConfiguration();
        std::cout << "Calibration completed. Please reconnect now" << std::endl;
        std::exit(0);
    }

    if(parser.isSet(MagnetometerCalibrateOption))
    {
        std::cout << "Entering MAGNETIC CALIBRATION mode"
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
        sensor.UnlockConfiguration();
        sensor.CalibrateMagnetometer();
        sleep(5);
        sensor.ConfirmConfiguration();
        std::cout << "Calibration completed. Please reconnect now" << std::endl;
        std::exit(0);
    }

    if(parser.isSet(SetBaudRateOption))
    {
        uint32_t new_rate = parser.value(SetBaudRateOption).toUInt();
        if(!((new_rate == 2400) ||
             (new_rate == 4800) ||
             (new_rate == 9600) ||
             (new_rate == 19200) ||
             (new_rate == 38400) ||
             (new_rate == 57600) ||
             (new_rate == 115200) ))
            std::cout << "ERROR: Wrong baudrate setting (use --help for detailed information). Ignoring baudrate reconfiguration request." << std::endl;
        else
        {
            std::cout << "Configuring baudrate for " << new_rate << " baud. NOTE: Please reconnect the sensor after this operation with the proper baudrate setting!" << std::endl;
            sensor.UnlockConfiguration();
            sensor.SetBaudRate(static_cast<QSerialPort::BaudRate>(new_rate));
            sensor.ConfirmConfiguration();
            std::cout << "Reconfiguration completed. Please reconnect now" << std::endl;
            std::exit(0);
        }
    }

    if(parser.isSet(SetPollingRateOption))
    {
        int32_t new_poll = parser.value(SetPollingRateOption).toInt();
        if(!((new_poll == -10) ||
             (new_poll == -2) ||
             (new_poll == -1) ||
             (new_poll == 0) ||
             (new_poll == 1) ||
             (new_poll == 2) ||
             (new_poll == 5) ||
             (new_poll == 10) ||
             (new_poll == 20) ||
             (new_poll == 50) ||
             (new_poll == 100) ||
             (new_poll == 125) ||
             (new_poll == 200) ))
            std::cout << "ERROR: Wrong output frequency setting (use --help for detailed information). Ignoring output frequency reconfiguration request." << std::endl;
        else
        {
            std::cout << "Configuring output frequency. NOTE: Please reconnect the sensor after this operation with the proper setting!" << std::endl;
            sensor.UnlockConfiguration();
            sensor.SetPollingRate(new_poll);
            sensor.ConfirmConfiguration();
            sleep(1);
            std::cout << "Reconfiguration completed. Please reconnect now" << std::endl;
            std::exit(0);
        }
    }

    if(parser.isSet(I2CAddressOption))
    {
        bool success;
        uint8_t address = parser.value(I2CAddressOption).toUShort(&success, 16);
        if((address > 0x7F) || !success)
        {
            std::cout << "ERROR: I2C address is 7 bits long, should be less than 7F hexadecimal. Falling back to 50" << std::endl;
            address = 0x50;
        }
        std::cout << "Configuring I2C bus address. NOTE: Please reconnect the sensor after this operation with the proper setting!" << std::endl;
        sensor.UnlockConfiguration();
        sensor.SetI2CAddress(address);
        sensor.ConfirmConfiguration();
        sleep(1);
        std::cout << "Reconfiguration completed. Please reconnect now" << std::endl;
        std::exit(0);
    }

    if(parser.isSet(BaseVerticalOrientationOption) ||
            parser.isSet(BaseHorizontalOrientationOption) ||
            parser.isSet(DormantOption) ||
            parser.isSet(GyroscopeAutoRecalibrateOption) ||
            parser.isSet(AxisTransitionOption) ||
            parser.isSet(LEDOption) ||
            parser.isSet(DisableMeasurementOption) ||
            parser.isSet(AccelerationBiasOption) ||
            parser.isSet(RTCSetupOption))
    {
        std::cout << "Non-blocking configuration, please wait..." << std::endl;
        sensor.UnlockConfiguration();
        if(parser.isSet(BaseVerticalOrientationOption) &&
                parser.isSet(BaseHorizontalOrientationOption))
            std::cout << "WARNING: You set both VERTICAL and HORIZONTAL base orientation settings. Assume that VERTICAL orientation is used to be set" << std::endl;
        if(parser.isSet(BaseVerticalOrientationOption) ||
                parser.isSet(BaseHorizontalOrientationOption))
            sensor.SetOrientation(parser.isSet(BaseVerticalOrientationOption));
        if(parser.isSet(DormantOption))
            sensor.ToggleDormant();
        if(parser.isSet(GyroscopeAutoRecalibrateOption))
        {
            if(parser.value(GyroscopeAutoRecalibrateOption).toUpper() == "ON")
                sensor.SetGyroscopeAutoRecalibration(true);
            else if(parser.value(GyroscopeAutoRecalibrateOption).toUpper() == "OFF")
                sensor.SetGyroscopeAutoRecalibration(false);
            else
            {
                std::cout << "WARNING: unknown value for option, assuming ON" << std::endl;
                sensor.SetGyroscopeAutoRecalibration(true);
            }
        }
        if(parser.isSet(AxisTransitionOption))
            sensor.SetAxisTransition(parser.value(AxisTransitionOption).toInt() == 6);
        if(parser.isSet(LEDOption))
            sensor.SetLED(!(parser.value(LEDOption).toUpper() == "OFF"));
        if(parser.isSet(DisableMeasurementOption))
        {
            QString arguments = parser.value(DisableMeasurementOption).toUpper();
            bool enable_accel = true;
            bool enable_velocity = true;
            bool enable_angles = true;
            bool enable_magnetometer = true;
            bool enable_rtc = true;
            bool enable_orientation = true;
            bool enable_port_status = true;
            bool enable_all = false;
            bool disable_all = false;
            if(arguments.trimmed().isEmpty())
            {
                std::cout << "WARNING: DISABLE setting is left empty, falling back to default" << std::endl;
                arguments = "DEFAULT";
                enable_all = true;
            }
            if(arguments.trimmed().contains("OFF"))
                enable_all = true;
            else if(arguments.trimmed().contains("DEFAULT"))
                disable_all = true;
            else
            {
                #if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
                QStringList arglist = arguments.trimmed().split(",", QString::SkipEmptyParts);
                #else
                QStringList arglist = arguments.trimmed().split(",", Qt::SkipEmptyParts);
                #endif
                if(arglist.size() > 0)
                {
                    for(auto i = arglist.begin(); i != arglist.end(); i++)
                    {
                        QString ref = (*i);
                        std::cout << "Attempting to disable:\t" << ref.toStdString() << std::endl;
                        if(ref.contains("ACCEL"))
                            enable_accel = false;
                        else if(ref.contains("VELOCIT"))
                            enable_velocity = false;
                        else if(ref.contains("EULER") || ref.contains("ANGLE"))
                            enable_angles = false;
                        else if(ref.contains("MAGNET"))
                            enable_magnetometer = false;
                        else if(ref.contains("RTC") || ref.contains("CLOCK") || ref.contains("TIME"))
                            enable_rtc = false;
                        else if(ref.contains("ORIENTATION") || ref.contains("QUATERNION"))
                            enable_orientation = false;
                        else if(ref.contains("PORT") || ref.contains("STATUS"))
                            enable_port_status = false;
                        else
                            std::cout << "WARNING: cannot interpret measurement name " << ref.toStdString() << ", dropping symbol" << std::endl;
                    }
                }
            }
            if(enable_all)
                sensor.SetMeasurements(true,
                                       true,
                                       true,
                                       true,
                                       true,
                                       true,
                                       true);
            else if(disable_all)
                sensor.SetMeasurements(false,
                                       false,
                                       false,
                                       false,
                                       false,
                                       false,
                                       false);
            else
                sensor.SetMeasurements(enable_rtc,
                                       enable_accel,
                                       enable_velocity,
                                       enable_angles,
                                       enable_magnetometer,
                                       enable_orientation,
                                       enable_port_status);
        }
        if(parser.isSet(AccelerationBiasOption))
        {
            #if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
                QStringList biases = parser.value(AccelerationBiasOption).split(":", QString::SkipEmptyParts, Qt::CaseInsensitive);
            #else
                QStringList biases = parser.value(AccelerationBiasOption).split(":", Qt::SkipEmptyParts, Qt::CaseInsensitive);
            #endif
            if(!biases.empty())
            {
                float bias_x = 0;
                float bias_y = 0;
                float bias_z = 0;
                bias_x = biases[0].toFloat();
                if(biases.size() > 1)
                    bias_y = biases[1].toFloat();
                if(biases.size() > 2)
                    bias_z = biases[2].toFloat();
                sensor.SetAccelerationBias(bias_x, bias_y, bias_z);
                sleep(3);
            }
            else
                std::cout << "ERROR: Cannot parse value list for acceleration biases. Please use <X:Y:Z> formulation" << std::endl;
        }
        if(parser.isSet(RTCSetupOption))
        {
            if(parser.value(RTCSetupOption).toUpper() == "NOW")
                sensor.SetRTC(QDateTime::currentDateTime());
            else
            {
                QDateTime datetime = QDateTime::fromString(parser.value(RTCSetupOption), Qt::ISODateWithMs);
                sensor.SetRTC(datetime);
                sleep(1);
            }
        }
        sensor.ConfirmConfiguration();
        std::cout << "Reconfiguration completed, proceeding to normal operation" << std::endl << std::endl;
    }

    maintenance = false;

    int result = app.exec();

    std::cout << "Average sensor return rate "
              << std::accumulate(times.begin(), times.end(), 0.f) / times.size()
              << " s" << std::endl << std::endl;

    if(parser.isSet(CovarianceOption))
    {
        std::cout << "Calculating noise covariance matrices..." << std::endl
                  << std::endl
                  << "Accelerations (total for " << accels_x.size() << " measurements): " << std::endl
                  << "[\t" << witmotion::variance(accels_x) << "\t0.00000\t0.00000" << std::endl
                  << "\t0.00000\t" << witmotion::variance(accels_y) << "\t0.00000" << std::endl
                  << "\t0.00000\t0.00000\t" << witmotion::variance(accels_z) << "\t]" << std::endl
                  << std::endl
                  << "Angular velocities (total for " << vels_x.size() << " measurements): " << std::endl
                  << "[\t" << witmotion::variance(vels_x) << "\t0.00000\t0.00000" << std::endl
                  << "\t0.00000\t" << witmotion::variance(vels_y) << "\t0.00000" << std::endl
                  << "\t0.00000\t0.00000\t" << witmotion::variance(vels_z) << "\t]" << std::endl
                  << std::endl
                  << "Angles (total for " << pitches.size() << " measurements): " << std::endl
                  << "[\t" << witmotion::variance(rolls) << "\t0.00000\t0.00000" << std::endl
                  << "\t0.00000\t" << witmotion::variance(pitches) << "\t0.00000" << std::endl
                  << "\t0.00000\t0.00000\t" << witmotion::variance(yaws) << "\t]" << std::endl
                  << std::endl
                  << "Temperature (total for " << temps.size() << " measurements): " << std::endl
                  << "[\t" << witmotion::variance(temps) << "\t]" << std::endl
                  << std::endl
                  << "Magnetometer (total for " << mags_x.size() << " measurements): " << std::endl
                  << "[\t" << witmotion::variance(mags_x) << "\t00.00000\t00.00000" << std::endl
                  << "\t00.00000\t" << witmotion::variance(mags_y) << "\t00.00000" << std::endl
                  << "\t00.00000\t00.00000\t" << witmotion::variance(mags_z) << "\t]" << std::endl
                  << std::endl;
    }

    if(parser.isSet(LogOption))
    {
        std::cout << "Writing log file to sensor.log" << std::endl;
        std::fstream logfile;
        logfile.open("sensor.log", std::ios::out|std::ios::trunc);
        logfile.precision(5);
        logfile << std::fixed;
        logfile << " -== WITMOTION WT901 STANDALONE SENSOR CONTROLLER/MONITOR ==-" << std::endl << std::endl;
        auto time_start = std::chrono::system_clock::now();
        std::time_t timestamp_start = std::chrono::system_clock::to_time_t(time_start);
        logfile << "Device /dev/" << device.toStdString() << " opened at " << static_cast<int32_t>(rate) << " baud" << std::endl;
        logfile << std::endl << "Acquired packets: " << std::endl;
        if(!acquired.empty())
        {
            uint32_t packets = 1;
            for(auto i = acquired.begin(); i != acquired.end(); i++)
            {
                witmotion::witmotion_datapacket packet = (*i);
                float ax, ay, az, wx, wy, wz, roll, pitch, yaw, t, mx, my, mz, qx, qy, qz, qw;
                uint8_t year, month, day, hour, minute, second;
                uint16_t millisecond;
                QString uptime;
                switch (static_cast<witmotion::witmotion_packet_id>(packet.id_byte))
                {
                case witmotion::pidAcceleration:
                    witmotion::decode_accelerations(packet, ax, ay, az, t);
                    logfile << packets << "\t"
                            << "Accelerations [X|Y|Z]:\t[ "
                            << ax << " | "
                            << ay << " | "
                            << az << " ], temp "
                            << t << " degrees"
                            << std::endl;
                    break;
                case witmotion::pidAngularVelocity:
                    witmotion::decode_angular_velocities(packet, wx, wy, wz, t);
                    logfile << packets << "\t"
                            << "Angular velocities [X|Y|Z]:\t[ "
                            << wx << " | "
                            << wy << " | "
                            << wz << " ], temp "
                            << t << " degrees"
                            << std::endl;
                    break;
                case witmotion::pidAngles:
                    witmotion::decode_angles(packet, roll, pitch, yaw, t);
                    logfile << packets << "\t"
                            << "Euler angles [R|P|Y]:\t[ "
                            << roll << " | "
                            << pitch << " | "
                            << yaw << " ], temp "
                            << t << " degrees"
                            << std::endl;
                    break;
                case witmotion::pidMagnetometer:
                    witmotion::decode_magnetometer(packet, mx, my ,mz, t);
                    logfile << packets << "\t"
                            << "Magnetic field [X|Y|Z]:\t[ "
                            << mx << " | "
                            << my << " | "
                            << mz << " ], temp "
                            << t << " degrees"
                            << std::endl;
                    break;
                case witmotion::pidRTC:
                    witmotion::decode_realtime_clock(packet, year, month, day, hour, minute, second, millisecond);
                    uptime = QString().setNum(year) + "-"
                            + QString().setNum(month) + "-"
                            + QString().setNum(day) + " "
                            + QString().setNum(hour) + ":"
                            + QString().setNum(minute) + ":"
                            + QString().setNum(second) + "."
                            + QString().setNum(millisecond);
                    logfile << packets << "\t"
                            << "Uptime / Timestamp: "
                            << uptime.toStdString()
                            << std::endl;
                    break;
                case witmotion::pidOrientation:
                    witmotion::decode_orientation(packet, qx, qy, qz, qw);
                    logfile << packets << "\t"
                            << "Orientation quaternion [X|Y|Z|W]:\t[ "
                            << qx << " | "
                            << qy << " | "
                            << qz << " | "
                            << qw << " ]"
                            << std::endl;
                    break;
                case witmotion::pidDataPortStatus:
                    logfile << packets << "\t"
                            << "Data port status string: 0x"
                            << std::hex
                            << static_cast<uint32_t>(packet.datastore.raw[0]) << " 0x"
                            << static_cast<uint32_t>(packet.datastore.raw[1]) << " 0x"
                            << static_cast<uint32_t>(packet.datastore.raw[2]) << " 0x"
                            << static_cast<uint32_t>(packet.datastore.raw[3]) << " 0x"
                            << static_cast<uint32_t>(packet.datastore.raw[4]) << " 0x"
                            << static_cast<uint32_t>(packet.datastore.raw[5]) << " 0x"
                            << static_cast<uint32_t>(packet.datastore.raw[6]) << " 0x"
                            << static_cast<uint32_t>(packet.datastore.raw[7])
                            << std::dec
                            << std::endl;
                    break;
                default:
                    break;
                }
                packets++;
            }
            logfile << std::endl;
        }

        if(parser.isSet(CovarianceOption))
        {
            logfile << "-= NOISE COVARIANCE MATRICES =-" << std::endl
                    << std::endl
                    << "Accelerations (total for " << accels_x.size() << " measurements): " << std::endl
                    << "[\t" << witmotion::variance(accels_x) << "\t0.00000\t0.00000" << std::endl
                    << "\t0.00000\t" << witmotion::variance(accels_y) << "\t0.00000" << std::endl
                    << "\t0.00000\t0.00000\t" << witmotion::variance(accels_z) << "\t]" << std::endl
                    << std::endl
                    << "Angular velocities (total for " << vels_x.size() << " measurements): " << std::endl
                    << "[\t" << witmotion::variance(vels_x) << "\t0.00000\t0.00000" << std::endl
                    << "\t0.00000\t" << witmotion::variance(vels_y) << "\t0.00000" << std::endl
                    << "\t0.00000\t0.00000\t" << witmotion::variance(vels_z) << "\t]" << std::endl
                    << std::endl
                    << "Angles (total for " << pitches.size() << " measurements): " << std::endl
                    << "[\t" << witmotion::variance(rolls) << "\t0.00000\t0.00000" << std::endl
                    << "\t0.00000\t" << witmotion::variance(pitches) << "\t0.00000" << std::endl
                    << "\t0.00000\t0.00000\t" << witmotion::variance(yaws) << "\t]" << std::endl
                    << std::endl
                    << "Temperature (total for " << temps.size() << " measurements): " << std::endl
                    << "[\t" << witmotion::variance(temps) << "\t]" << std::endl
                    << std::endl
                    << "Magnetometer (total for " << mags_x.size() << " measurements): " << std::endl
                    << "[\t" << witmotion::variance(mags_x) << "\t00.00000\t00.00000" << std::endl
                    << "\t00.00000\t" << witmotion::variance(mags_y) << "\t00.00000" << std::endl
                    << "\t00.00000\t00.00000\t" << witmotion::variance(mags_z) << "\t]" << std::endl
                      << std::endl;
        }

        logfile << "Acquisition performed at " << std::ctime(&timestamp_start) << std::endl;
        logfile.close();
    }

    return result;
}

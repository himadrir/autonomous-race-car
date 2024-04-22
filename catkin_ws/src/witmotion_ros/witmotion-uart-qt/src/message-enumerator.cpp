#include "witmotion/message-enumerator.h"

using namespace witmotion;

void QGeneralSensorController::BuildLog()
{
    log << QString()
        << "WITMOTION UART MESSAGE ENUMERATOR BY TWDRAGON"
        << QString()
        << "Acquired at " + QDateTime::currentDateTime().toString(Qt::SystemLocaleLongDate)
        << QString();
    log << "ID\tQty\tDescription" << QString();
    for(auto i = witmotion_packet_descriptions.begin(); i != witmotion_packet_descriptions.end(); i++)
    {
        QString logline;
        if(counts[static_cast<witmotion_packet_id>(i->first)] > 0)
        {
            logline = "0x"
                    + QString::number(i->first, 16) + "\t"
                    + QString::number(counts[static_cast<witmotion_packet_id>(i->first)])
                    + "\t"
                    + QString::fromStdString(i->second);
            log << logline;
        }
    }
    log << QString();
    QString unknown_print = "\tUnknown IDs: " + QString::number(unknown_ids) + " [ ";
    for(auto i = unknown.begin(); i != unknown.end(); i++)
        unknown_print = unknown_print + "0x" + QString::number(*i, 16);
    unknown_print += " ] ";
    log << unknown_print;
    log << "Total messages: " + QString::number(packets) << QString();
}

QGeneralSensorController::QGeneralSensorController(const QString port, const QSerialPort::BaudRate rate):
    packets(0),
    port_name(port),
    port_rate(rate),
    reader_thread(dynamic_cast<QObject*>(this)),
    reader(nullptr),
    ttyout(stdout),
    unknown_ids(0),
    log_set(false),
    logfile(nullptr)
{
    reader = new QBaseSerialWitmotionSensorReader(port_name, port_rate);
    reader->moveToThread(&reader_thread);
    connect(&reader_thread, &QThread::finished, reader, &QObject::deleteLater);
    connect(this, &QGeneralSensorController::RunReader, reader, &QAbstractWitmotionSensorReader::RunPoll);
    connect(reader, &QAbstractWitmotionSensorReader::Acquired, this, &QGeneralSensorController::Packet);
    connect(reader, &QAbstractWitmotionSensorReader::Error, this, &QGeneralSensorController::Error);
    reader_thread.start();
}

QGeneralSensorController::~QGeneralSensorController()
{
    reader_thread.quit();
    reader_thread.wait(10000);

    BuildLog();
    for(auto i = log.begin(); i != log.end(); i++)
        ttyout << *i << ENDL;

    if(logfile != nullptr)
    {
        QTextStream logstream(logfile);
        for(auto i = log.begin(); i != log.end(); i++)
            logstream << *i << ENDL;
        ttyout << "Log file written to " << logfile->fileName() << ENDL;
        logfile->close();
        delete logfile;
    }
}

void QGeneralSensorController::Start()
{
    emit RunReader();
}

void QGeneralSensorController::SetLog(const QString name)
{
    log_set = true;
    log_name = name;
    logfile = new QFile(name);
    if(!logfile->open(QFile::Truncate|QFile::WriteOnly|QFile::Text))
    {
        ttyout << "ERROR: cannot open logfile! Proceeding without a log" << ENDL;
        log_set = false;
    }
}

void QGeneralSensorController::SetInterval(uint32_t ms)
{
    reader->SetSensorPollInterval(ms);
}

void QGeneralSensorController::Packet(const witmotion_datapacket &packet)
{
    ++packets;
    if(id_registered(packet.id_byte))
        counts[static_cast<witmotion_packet_id>(packet.id_byte)]++;
    else
    {
        unknown_ids++;
        unknown.insert(packet.id_byte);
        ttyout << ", unknown ID 0x" << HEX << packet.id_byte;
    }
}

void QGeneralSensorController::Error(const QString &description)
{
    ttyout << "ERROR: " << description << ENDL;
    QCoreApplication::exit(1);
}

void handle_shutdown(int s)
{
    std::cout << std::endl;
    QCoreApplication::exit(0);
}

int main(int argc, char** args)
{
    std::cout << "Press Ctrl+C to stop enumeration and see the report" << std::endl;

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_shutdown;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    QCoreApplication app(argc, args);
    QCommandLineParser parser;
    parser.setApplicationDescription("WITMOTION UART MESSAGE ENUMERATOR BY TWDRAGON");
    parser.addHelpOption();
    QCommandLineOption BaudRateOption(QStringList() << "b" << "baudrate",
                                      "Baudrate to set up the port",
                                      "QSerialPort::BaudRate set of values",
                                      "9600");
    QCommandLineOption DeviceNameOption(QStringList() << "d" << "device",
                                        "Port serial device name, without \'/dev\'",
                                        "ttyUSB0",
                                        "ttyUSB0");
    QCommandLineOption FileNameOption(QStringList() << "f" << "log-file",
                                      "Log file name",
                                      "log",
                                      "log.txt");
    QCommandLineOption IntervalOption(QStringList() << "p" << "poll",
                                      "Sensor poll interval (ms)",
                                      "interval",
                                      "50");
    parser.addOption(BaudRateOption);
    parser.addOption(DeviceNameOption);
    parser.addOption(FileNameOption);
    parser.addOption(IntervalOption);
    parser.process(app);

    QGeneralSensorController controller(parser.value(DeviceNameOption),
                                        static_cast<QSerialPort::BaudRate>(parser.value(BaudRateOption).toInt()));
    controller.setParent(dynamic_cast<QObject*>(&app));
    if(parser.isSet(FileNameOption))
        controller.SetLog(parser.value(FileNameOption));
    if(parser.isSet(IntervalOption))
        controller.SetInterval(parser.value(IntervalOption).toUInt());
    controller.Start();

    return app.exec();
}


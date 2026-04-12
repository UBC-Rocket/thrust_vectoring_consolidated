#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQmlNetworkAccessManagerFactory>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QCommandLineParser>
#include <QTimer>
#include "SerialBridge.h"
#include "SensorDataModel.h"
#include "CommandSender.h"
#include "AlarmReceiver.h"

// Custom NAM that sets a User-Agent header on all requests (required by OSM tile servers)
class OsmNetworkAccessManager : public QNetworkAccessManager {
protected:
    QNetworkReply *createRequest(Operation op, const QNetworkRequest &request, QIODevice *outgoingData) override {
        QNetworkRequest req(request);
        req.setHeader(QNetworkRequest::UserAgentHeader, "UlyssesGroundControl/0.1");
        return QNetworkAccessManager::createRequest(op, req, outgoingData);
    }
};

class OsmNetworkAccessManagerFactory : public QQmlNetworkAccessManagerFactory {
public:
    QNetworkAccessManager *create(QObject *parent) override {
        return new OsmNetworkAccessManager();
    }
};

int main(int argc, char *argv[])
{
    // Qt GUI application (event loop owner)
    QGuiApplication app(argc, argv);

    // Backend objects live for the duration of main
    SerialBridge bridge;
    CommandSender   commandsender(&bridge);   // sends commands via bridge
    AlarmReceiver   alarmreceiver(&bridge);   // receives/decodes alarms via bridge
    SensorDataModel sensorData(&bridge);      // decodes all downlink packets (telemetry + status)

    // QML engine + expose C++ backends to QML by name
    QQmlApplicationEngine engine;
    engine.setNetworkAccessManagerFactory(new OsmNetworkAccessManagerFactory());
    engine.rootContext()->setContextProperty("bridge", &bridge);
    engine.rootContext()->setContextProperty("commandsender", &commandsender);
    engine.rootContext()->setContextProperty("alarmreceiver", &alarmreceiver);
    engine.rootContext()->setContextProperty("sensorData", &sensorData);

    // If QML fails to load, quit with error code
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    // Load the QML entry point from the compiled QML module
    using namespace Qt::StringLiterals;
    const QUrl url(u"qrc:/ulysses_ground_control/QMLFiles/Main.qml"_s);
    engine.load(url);

    // Safety check: no root objects means load failed
    if (engine.rootObjects().isEmpty())
        return -1;

    // Start the event loop
    return app.exec();
}

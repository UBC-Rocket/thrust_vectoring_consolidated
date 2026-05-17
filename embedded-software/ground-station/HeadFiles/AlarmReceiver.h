#ifndef ALARMRECEIVER_H
#define ALARMRECEIVER_H

#include <QObject>

/**
 * @brief AlarmReceiver
 * Re-emit point for alarm chips rendered by Panel_System_Alert. The classified
 * signals (rxError/rxWarning/rxSuccess) are driven by SensorDataModel's
 * SystemStatus-derived synthesis, wired in main.cpp.
 */
class AlarmReceiver : public QObject {
    Q_OBJECT
public:
    explicit AlarmReceiver(QObject* parent = nullptr);

signals:
    /// Emitted when an event is classified as an error.
    void rxError(const QString& line);

    /// Emitted when an event is classified as a warning.
    void rxWarning(const QString& line);

    /// Emitted when an event is classified as a success/OK.
    void rxSuccess(const QString& line);
};

#endif // ALARMRECEIVER_H

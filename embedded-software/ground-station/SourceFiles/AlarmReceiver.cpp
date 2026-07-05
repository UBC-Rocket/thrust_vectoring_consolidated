#include "AlarmReceiver.h"

// AlarmReceiver is now a pure re-emit point for the alarm chip stream. Earlier
// versions classified incoming text lines from SerialBridge, but the text-mode
// RX path was removed when D3 wired SystemStatus-derived events as the
// alarm source. main.cpp connects SensorDataModel::alarm{Error,Warning,Success}
// directly to this object's rx{Error,Warning,Success} signals (signal-to-signal),
// so the body here only exists to host the QObject.
AlarmReceiver::AlarmReceiver(QObject* parent)
    : QObject(parent)
{
}

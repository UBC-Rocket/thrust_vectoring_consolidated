#ifndef COMMANDSENDER_H
#define COMMANDSENDER_H

#include <QObject>
#include <cstdint>
#include <QVariant>


extern "C" {
    #include "rp/codec.h"
    #include "command.pb.h"
}

class SerialBridge;

class CommandSender : public QObject {
    Q_OBJECT

public:
    /// Construct a sender bound to a SerialBridge (non-owning).
    explicit CommandSender(SerialBridge* bridge, QObject* parent = nullptr);

    /// Send a FlightCommand via encoded protobuf packet
    Q_INVOKABLE bool sendFlightCommand(int which, int commandType);

    // Send PID values.
    // Legacy 10-element format: [attKpX, attKpY, attKpZ, attKdX, attKdY, attKdZ, zKp, zKi, zKd, zIntegralLimit].
    // Preferred 12-element format: [has_attitude_kp, kp.x, kp.y, kp.z, has_attitude_kd, kd.x, kd.y, kd.z, z_kp, z_ki, z_kd, z_integral_limit].
    Q_INVOKABLE bool sendPIDValues(int which, const QVariantList& PIDValues);

    // Send reference values.
    // Legacy 6-element format: [z_ref, vz_ref, q_ref.w, q_ref.x, q_ref.y, q_ref.z].
    // Preferred 7-element format: [z_ref, vz_ref, has_q_ref, q_ref.w, q_ref.x, q_ref.y, q_ref.z].
    Q_INVOKABLE bool sendReferenceValues(int which, const QVariantList& referenceValues);

    // Send config values as: [mass, T_min, T_max, theta_min, theta_max].
    Q_INVOKABLE bool sendConfigValues(int which, const QVariantList& configValues);

    // Send UWB probe layout. probes is a QVariantList of exactly 4 anchor objects
    // with {x: float, y: float}. Sends on the bridge's currently selected TX channel.
    Q_INVOKABLE bool sendProbeLayout(const QVariantList& probes);

    // Send target motor RPM setpoints for the upper and lower ESC-driven motors.
    Q_INVOKABLE bool sendMotorSpeed(int which, double rpmUpper, double rpmLower);

signals:
    // -----------------------
    // App-level signals
    // -----------------------

    /// Emitted after a command is successfully dispatched (echoes payload).
    void messageSent(const QString payload);

    /// Emitted when sending fails (invalid channel, closed port, no bridge, etc.).
    void errorOccurred(const QString error);

public slots:
    /// Set or swap the SerialBridge used for all subsequent sends (non-owning).
    void setBridge(SerialBridge* bridge) { m_bridge = bridge; }

private:
    /// Check that the channel index is valid (1 or 2).
    static inline bool validWhich(int which) {
        return which == 1 || which == 2;
    }

    SerialBridge* m_bridge = nullptr; ///< Serial transport used to send commands.
};

#endif // COMMANDSENDER_H

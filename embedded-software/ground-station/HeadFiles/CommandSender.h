#ifndef COMMANDSENDER_H
#define COMMANDSENDER_H

#include <QObject>
#include <QTimer>
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

    /// Send a single command string out via the given channel on the bridge.
    Q_INVOKABLE bool sendCode(int which, const QString& code);

    /// Send a FlightCommand via encoded protobuf packet
    Q_INVOKABLE bool sendFlightCommand(int which, int commandType);

    // -----------------------
    // Periodic test helpers
    // -----------------------

    /// Start sending a command at a fixed frequency (Hz) on the given channel.
    Q_INVOKABLE void startPeriodic(int which, const QString& code, int hz = 50);

    /// Stop periodic sending on the given channel.
    Q_INVOKABLE void stopPeriodic(int which);

    /// Return true if periodic sending is active on the given channel.
    Q_INVOKABLE bool isPeriodicRunning(int which) const;

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
    /// Per-channel periodic sending state (timer + payload + rate).
    struct PeriodicChan {
        QTimer timer;
        QString payload;
        int hz = 0;
    };

    /// Check that the channel index is valid (1 or 2).
    static inline bool validWhich(int which) {
        return which == 1 || which == 2;
    }

    /// Non-const accessor to channel state (1 or 2).
    PeriodicChan& chan(int which) {
        return (which == 1) ? m_ch1 : m_ch2;
    }

    /// Const accessor to channel state (1 or 2).
    const PeriodicChan& chan(int which) const {
        return (which == 1) ? m_ch1 : m_ch2;
    }

    SerialBridge* m_bridge = nullptr; ///< Serial transport used to send commands.
    PeriodicChan m_ch1;               ///< Periodic send state for channel 1.
    PeriodicChan m_ch2;               ///< Periodic send state for channel 2.
};

#endif // COMMANDSENDER_H

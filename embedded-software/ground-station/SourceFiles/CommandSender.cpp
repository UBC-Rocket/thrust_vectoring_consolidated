#include "CommandSender.h"
#include "SerialBridge.h"
#include <QTimer>
extern "C" {
    #include "rp/codec.h"
    #include "command.pb.h"
}


CommandSender::CommandSender(SerialBridge* bridge, QObject* parent)
    : m_bridge(bridge), QObject(parent)
{
    // Channel 1 periodic sender: fires at fixed rate, sends cached payload if bridge exists.
    m_ch1.timer.setTimerType(Qt::PreciseTimer);
    connect(&m_ch1.timer, &QTimer::timeout, this, [this]() {
        if (!m_bridge) { emit errorOccurred("No Bridge"); return; }
        if (!m_ch1.payload.isEmpty()) {
            if (m_bridge->sendText(1, m_ch1.payload))
                emit messageSent(m_ch1.payload);
            else
                emit errorOccurred("Periodic send failed (P1)");
        }
    });


    // Channel 2 periodic sender: same idea but for channel 2 payload.
    m_ch2.timer.setTimerType(Qt::PreciseTimer);
    connect(&m_ch2.timer, &QTimer::timeout, this, [this]() {
        if (!m_bridge) { emit errorOccurred("No Bridge"); return; }
        if (!m_ch2.payload.isEmpty()) {
            if (m_bridge->sendText(1, m_ch2.payload))
                emit messageSent(m_ch2.payload);
            else
                emit errorOccurred("Periodic send failed (P1)");
        }
    });
}


bool CommandSender::sendCode(int which, const QString& code) {
    if (!validWhich(which)) {
        emit errorOccurred("which must be 1 or 2");
        return false;
    }


    if (!m_bridge) {
        emit errorOccurred("No bridge");   // Cannot send without a transport.
        return false;
    }


    // Delegate the actual serial write to SerialBridge.
    bool ok = m_bridge->sendText(which, code);


    if (ok) {
        emit messageSent(code);            // Notify listeners what was sent.
    } else {
        emit errorOccurred("Failed to send code");
    }
    return ok;                             // Let caller know if it worked.
}


void CommandSender::startPeriodic(int which, const QString& code, int hz) {
    if (!validWhich(which)) {
        emit errorOccurred("which must be 1 or 2");
        return;
    }


    if (hz <= 0) {
        emit errorOccurred("Hz must be > 0");
        return;
    }


    // Store payload and frequency, then arm the channel timer.
    auto& c = chan(which);
    c.payload = code;
    c.hz = hz;
    c.timer.start(1000 / hz);             // Simple ms interval: 1000ms / Hz.
}


void CommandSender::stopPeriodic(int which) {
    if (!validWhich(which)) {
        emit errorOccurred("which must be 1 or 2");
        return;
    }


    // Just stop the underlying QTimer for that channel.
    chan(which).timer.stop();
}


bool CommandSender::isPeriodicRunning(int which) const {
    if (!validWhich(which))
        return false;


    return chan(which).timer.isActive();  // True if the periodic timer is currently running.
}


bool CommandSender::sendFlightCommand(int which, int commandType) {
    if (!validWhich(which)) {
        emit errorOccurred("which must be 1 or 2");
        return false;
    }
   
    if (!m_bridge) {
        emit errorOccurred("No bridge");
        return false;
    }

    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_state_cmd_tag;  // Set oneof
    cmd.payload.state_cmd.type = (tvr_StateCommand_Type)commandType;  // Set type

    uint8_t packet[300];
    rp_packet_encode_result_t result = rp_packet_encode( 
        packet,
        sizeof(packet),
        tvr_FlightCommand_fields,
        &cmd
    );

    if (result.status != RP_CODEC_OK) {
        emit errorOccurred("Failed to encode packet");
        return false;
    }

    // 3. Send binary packet
    QByteArray data(reinterpret_cast<const char*>(packet), result.written);

    if (!m_bridge->sendBinary(which, data)) {
        emit errorOccurred("Failed to send binary packet");
        return false;
    }

    emit messageSent(QString("FlightCommand %1").arg(commandType));
    return true;
   
}


bool CommandSender::sendPIDValues(int which, const QVariantList& PIDValues) {
    if (!validWhich(which)) {
        emit errorOccurred("which must be 1 or 2");
        return false;
    }
   
    if (!m_bridge) {
        emit errorOccurred("No bridge");
        return false;
    }

    const bool hasExplicitFlags = (PIDValues.size() >= 12);
    if (!hasExplicitFlags && PIDValues.size() < 10) {
        emit errorOccurred("PIDValues must contain 10 entries (legacy) or 12 entries (with has_* flags)");
        return false;
    }

    tvr_SetPidGains pid = tvr_SetPidGains_init_zero;
    if (hasExplicitFlags) {
        // New format:
        // [has_attitude_kp, kp.x, kp.y, kp.z, has_attitude_kd, kd.x, kd.y, kd.z, z_kp, z_ki, z_kd, z_integral_limit]
        pid.has_attitude_kp = PIDValues[0].toBool();
        pid.attitude_kp.x = static_cast<float>(PIDValues[1].toDouble());
        pid.attitude_kp.y = static_cast<float>(PIDValues[2].toDouble());
        pid.attitude_kp.z = static_cast<float>(PIDValues[3].toDouble());

        pid.has_attitude_kd = PIDValues[4].toBool();
        pid.attitude_kd.x = static_cast<float>(PIDValues[5].toDouble());
        pid.attitude_kd.y = static_cast<float>(PIDValues[6].toDouble());
        pid.attitude_kd.z = static_cast<float>(PIDValues[7].toDouble());

        pid.z_kp = static_cast<float>(PIDValues[8].toDouble());
        pid.z_ki = static_cast<float>(PIDValues[9].toDouble());
        pid.z_kd = static_cast<float>(PIDValues[10].toDouble());
        pid.z_integral_limit = static_cast<float>(PIDValues[11].toDouble());
    } else {
        // Legacy format:
        // [kp.x, kp.y, kp.z, kd.x, kd.y, kd.z, z_kp, z_ki, z_kd, z_integral_limit]
        pid.has_attitude_kp = true;
        pid.attitude_kp.x = static_cast<float>(PIDValues[0].toDouble());
        pid.attitude_kp.y = static_cast<float>(PIDValues[1].toDouble());
        pid.attitude_kp.z = static_cast<float>(PIDValues[2].toDouble());

        pid.has_attitude_kd = true;
        pid.attitude_kd.x = static_cast<float>(PIDValues[3].toDouble());
        pid.attitude_kd.y = static_cast<float>(PIDValues[4].toDouble());
        pid.attitude_kd.z = static_cast<float>(PIDValues[5].toDouble());

        pid.z_kp = static_cast<float>(PIDValues[6].toDouble());
        pid.z_ki = static_cast<float>(PIDValues[7].toDouble());
        pid.z_kd = static_cast<float>(PIDValues[8].toDouble());
        pid.z_integral_limit = static_cast<float>(PIDValues[9].toDouble());
    }

    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_pid_gains_tag;
    cmd.payload.set_pid_gains = pid;

    uint8_t packet[300];
    rp_packet_encode_result_t result = rp_packet_encode(
        packet,
        sizeof(packet),
        tvr_FlightCommand_fields,
        &cmd
    );


    if (result.status != RP_CODEC_OK) {
        emit errorOccurred("Failed to encode packet");
        return false;
    }


    QByteArray data(reinterpret_cast<const char*>(packet), result.written);
   
    if (!m_bridge->sendBinary(which, data)) {
        emit errorOccurred("Failed to send binary packet");
        return false;
    }
   
    emit messageSent("SetPidGains sent");
    return true;

}

bool CommandSender::sendReferenceValues(int which, const QVariantList& referenceValues) {

    if (!validWhich(which)) {
        emit errorOccurred("which must be 1 or 2");
        return false;
    }
   
    if (!m_bridge) {
        emit errorOccurred("No bridge");
        return false;
    }

    const bool hasExplicitFlag = (referenceValues.size() >= 7);
    if (!hasExplicitFlag && referenceValues.size() < 6) {
        emit errorOccurred("Reference values must contain 6 entries (legacy) or 7 entries (with has_q_ref)");
        return false;
    }

    tvr_SetReference reference = tvr_SetReference_init_zero;
    reference.z_ref = static_cast<float>(referenceValues[0].toDouble());
    reference.vz_ref = static_cast<float>(referenceValues[1].toDouble());
    if (hasExplicitFlag) {
        // New format:
        // [z_ref, vz_ref, has_q_ref, q_ref.w, q_ref.x, q_ref.y, q_ref.z]
        reference.has_q_ref = referenceValues[2].toBool();
        reference.q_ref.w = static_cast<float>(referenceValues[3].toDouble());
        reference.q_ref.x = static_cast<float>(referenceValues[4].toDouble());
        reference.q_ref.y = static_cast<float>(referenceValues[5].toDouble());
        reference.q_ref.z = static_cast<float>(referenceValues[6].toDouble());
    } else {
        // Legacy format:
        // [z_ref, vz_ref, q_ref.w, q_ref.x, q_ref.y, q_ref.z]
        reference.has_q_ref = true;
        reference.q_ref.w = static_cast<float>(referenceValues[2].toDouble());
        reference.q_ref.x = static_cast<float>(referenceValues[3].toDouble());
        reference.q_ref.y = static_cast<float>(referenceValues[4].toDouble());
        reference.q_ref.z = static_cast<float>(referenceValues[5].toDouble());
    }

    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_reference_tag;
    cmd.payload.set_reference = reference;

    uint8_t packet[300];
    rp_packet_encode_result_t result = rp_packet_encode(
        packet,
        sizeof(packet),
        tvr_FlightCommand_fields,
        &cmd
    );


    if (result.status != RP_CODEC_OK) {
        emit errorOccurred("Failed to encode packet");
        return false;
    }


    QByteArray data(reinterpret_cast<const char*>(packet), result.written);
   
    if (!m_bridge->sendBinary(which, data)) {
        emit errorOccurred("Failed to send binary packet");
        return false;
    }
   
    emit messageSent("SetReference sent");
    return true;


}

bool CommandSender::sendProbeLayout(const QVariantList& probes) {
    // TODO(ulysses): tvr_SetProbeLayout / tvr_Vec2 / tvr_FlightCommand_set_probe_layout_tag
    // are not defined in libs/rocket-protocol yet. Restore the probe-layout uplink path
    // once the .proto is updated and regenerated (tracked on the dynamic-probes feature).
    Q_UNUSED(probes);
    if (!m_bridge) {
        emit errorOccurred("No bridge");
        return false;
    }
    emit errorOccurred("SetProbeLayout: protocol schema not yet regenerated");
    return false;
}


bool CommandSender::sendConfigValues(int which, const QVariantList& configValues) {

    if (!validWhich(which)) {
        emit errorOccurred("which must be 1 or 2");
        return false;
    }
   
    if (!m_bridge) {
        emit errorOccurred("No bridge");
        return false;
    }

    if (configValues.size() < 5) {
        emit errorOccurred("Config values must contain 5 entries");
        return false;
    }

    tvr_SetConfig config = tvr_SetConfig_init_zero;
    config.mass = static_cast<float>(configValues[0].toDouble());
    config.T_min = static_cast<float>(configValues[1].toDouble());
    config.T_max = static_cast<float>(configValues[2].toDouble());
    config.theta_min = static_cast<float>(configValues[3].toDouble());
    config.theta_max = static_cast<float>(configValues[4].toDouble());

    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_config_tag;
    cmd.payload.set_config = config;

    uint8_t packet[300];
    rp_packet_encode_result_t result = rp_packet_encode(
        packet,
        sizeof(packet),
        tvr_FlightCommand_fields,
        &cmd
    );


    if (result.status != RP_CODEC_OK) {
        emit errorOccurred("Failed to encode packet");
        return false;
    }


    QByteArray data(reinterpret_cast<const char*>(packet), result.written);
   
    if (!m_bridge->sendBinary(which, data)) {
        emit errorOccurred("Failed to send binary packet");
        return false;
    }
   
    emit messageSent("SetConfig sent");
    return true;

}

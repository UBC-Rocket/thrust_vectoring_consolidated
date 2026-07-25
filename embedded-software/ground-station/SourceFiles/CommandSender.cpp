#include "CommandSender.h"
#include "SerialBridge.h"
#include <QTimer>
extern "C" {
    #include "rp/codec.h"
    #include "command.pb.h"
}


CommandSender::CommandSender(SerialBridge* bridge, QObject* parent)
    : QObject(parent), m_bridge(bridge)
{
    m_heartbeatTimer = new QTimer(this);
    m_heartbeatTimer->setInterval(250);   // 2 refreshes per 500 ms FC watchdog window
    connect(m_heartbeatTimer, &QTimer::timeout, this, &CommandSender::sendHeartbeat);
    m_heartbeatTimer->start();
}

void CommandSender::sendHeartbeat() {
    if (!m_bridge)
        return;

    // Prefer the operator's selected TX channel; fall back to whichever
    // port is actually open. No port open -> nothing to keep alive.
    int which = 0;
    if (m_bridge->isConnected(m_bridge->txTo()))
        which = m_bridge->txTo();
    else if (m_bridge->isConnected(1))
        which = 1;
    else if (m_bridge->isConnected(2))
        which = 2;
    if (which == 0)
        return;

    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_state_cmd_tag;
    cmd.payload.state_cmd.type = tvr_StateCommand_Type_CMD_NONE;

    uint8_t packet[64];
    rp_packet_encode_result_t result = rp_packet_encode(
        packet, sizeof(packet), tvr_FlightCommand_fields, &cmd);
    if (result.status != RP_CODEC_OK)
        return;

    QByteArray data(reinterpret_cast<const char*>(packet), result.written);
    m_bridge->sendBinary(which, data);
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
    if (!m_bridge) {
        emit errorOccurred("No bridge");
        return false;
    }

    // Layout is fixed at exactly 4 ground anchors forming a rectangle.
    // Caller (Panel_Probe_Map) hands us the 4 corner positions as
    // {x, y} maps in nav-frame meters.
    if (probes.size() != 4) {
        emit errorOccurred(
            QStringLiteral("SetProbeLayout expects 4 anchors, got %1").arg(probes.size()));
        return false;
    }

    tvr_SetProbeLayout layout = tvr_SetProbeLayout_init_zero;
    bool* hasArr[4]    = { &layout.has_anchor_0, &layout.has_anchor_1,
                           &layout.has_anchor_2, &layout.has_anchor_3 };
    tvr_Vec2* probeArr[4] = { &layout.anchor_0, &layout.anchor_1,
                              &layout.anchor_2, &layout.anchor_3 };

    for (int i = 0; i < 4; ++i) {
        const QVariantMap entry = probes[i].toMap();
        *hasArr[i] = true;
        probeArr[i]->x = static_cast<float>(entry.value("x").toDouble());
        probeArr[i]->y = static_cast<float>(entry.value("y").toDouble());
    }

    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_probe_layout_tag;
    cmd.payload.set_probe_layout = layout;

    uint8_t packet[300];
    rp_packet_encode_result_t result = rp_packet_encode(
        packet,
        sizeof(packet),
        tvr_FlightCommand_fields,
        &cmd
    );

    if (result.status != RP_CODEC_OK) {
        emit errorOccurred("Failed to encode probe layout packet");
        return false;
    }

    // Use the operator's currently selected TX channel (matches D4 — PID/Reference/
    // Config now also bind to bridge.txTo via Panel_PID_Controller.which).
    const int which = m_bridge->txTo();
    QByteArray data(reinterpret_cast<const char*>(packet), result.written);
    if (!m_bridge->sendBinary(which, data)) {
        emit errorOccurred("Failed to send probe layout packet");
        return false;
    }

    emit messageSent(QStringLiteral("SetProbeLayout sent (4 anchors)"));
    return true;
}


bool CommandSender::sendThrottle(int which, double throttle) {
    if (!validWhich(which)) {
        emit errorOccurred("which must be 1 or 2");
        return false;
    }

    if (!m_bridge) {
        emit errorOccurred("No bridge");
        return false;
    }

    /* Clamp here as well as on the FC — the QML field has no validator. */
    float v = static_cast<float>(throttle);
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;

    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_throttle_tag;
    cmd.payload.set_throttle.throttle = v;

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

    emit messageSent(QString("SetThrottle sent (%1%)")
                         .arg(static_cast<int>(v * 100.0f + 0.5f)));
    return true;
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

#include "SensorDataModel.h"
#include "SerialBridge.h"
extern "C" {
    #include "rp/codec.h"
    #include "downlink.pb.h"
}
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>
#include <QtMath>
#include <cmath>

namespace {
static constexpr bool kDownlinkDebug = false;

// Quaternion (w,x,y,z) to Euler angles (roll, pitch, yaw) in radians.
void quatToEulerRad(float w, float x, float y, float z,
                    float* roll_rad, float* pitch_rad, float* yaw_rad) {
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
        *pitch_rad = static_cast<float>(std::copysign(M_PI / 2, sinp));
    } else {
        *pitch_rad = static_cast<float>(std::asin(sinp));
    }
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    *yaw_rad = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    *roll_rad = static_cast<float>(std::atan2(sinr_cosp, cosr_cosp));
}

float radToDeg(float rad) {
    return static_cast<float>(rad * 180.0 / M_PI);
}

// CSV column order (kept in one place so the header and row writers agree).
const char* const kCsvHeader =
    "wall_ms,type,timestamp_ms,flight_state,"
    "pos_x,pos_y,pos_z,"
    "vel_x,vel_y,vel_z,"
    "att_w,att_x,att_y,att_z,"
    "gyro_x,gyro_y,gyro_z,"
    "thrust_cmd,gimbal_x,gimbal_y,"
    "uptime_ms,accel_ok,gyro_ok,baro1_ok,baro2_ok,"
    "gps_connected,radio_rx_count,radio_tx_count,cmd_rx_count";

QString fmt(double v) { return QString::number(v, 'g', 9); }
QString fmtBool(bool v) { return v ? QStringLiteral("1") : QStringLiteral("0"); }
} // namespace

SensorDataModel::SensorDataModel(SerialBridge* bridge, QObject* parent)
    : QObject(parent), m_bridge(bridge)
{
    if (!m_bridge)
        return;

    // Subscribe to binary COBS packets (rocket sends COBS+protobuf Downlink).
    QObject::connect(
        m_bridge,
        &SerialBridge::binaryPacketReceived,
        this,
        [this](int which, const QByteArray &packet) {
            if (!m_bridge) return;
            const bool p1 = m_bridge->isConnected(1);
            const bool p2 = m_bridge->isConnected(2);
            bool listen = (p1 && p2) ? (which == 1) : m_bridge->isConnected(which);
            if (listen)
                onBinaryPacketReceived(which, packet);
        });
}

SensorDataModel::~SensorDataModel()
{
    stopCsvRecording();
}

void SensorDataModel::clearRawPacketLog()
{
    m_rawPacketLog.clear();
    emit rawPacketLogChanged();
}

QString SensorDataModel::defaultCsvPath() const
{
    const QString dir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
                        + QStringLiteral("/ulysses_logs");
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_hhmmss"));
    return dir + QStringLiteral("/ulysses_") + stamp + QStringLiteral(".csv");
}

bool SensorDataModel::startCsvRecording(const QString& path)
{
    if (isRecording())
        stopCsvRecording();

    QString target = path.isEmpty() ? defaultCsvPath() : path;
    QFileInfo info(target);
    QDir dir = info.absoluteDir();
    if (!dir.exists() && !dir.mkpath(QStringLiteral("."))) {
        qWarning() << "CSV: cannot create directory" << dir.absolutePath();
        return false;
    }

    auto* file = new QFile(target);
    if (!file->open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        qWarning() << "CSV: cannot open" << target << file->errorString();
        delete file;
        return false;
    }

    m_csvFile   = file;
    m_csvStream = new QTextStream(m_csvFile);
    (*m_csvStream) << kCsvHeader << '\n';
    m_csvStream->flush();
    m_csvPath = info.absoluteFilePath();
    emit recordingStateChanged();
    return true;
}

void SensorDataModel::stopCsvRecording()
{
    if (m_csvStream) {
        m_csvStream->flush();
        delete m_csvStream;
        m_csvStream = nullptr;
    }
    if (m_csvFile) {
        m_csvFile->close();
        m_csvFile->deleteLater();
        m_csvFile = nullptr;
    }
    if (!m_csvPath.isEmpty()) {
        m_csvPath.clear();
        emit recordingStateChanged();
    }
}

void SensorDataModel::onBinaryPacketReceived(int which, const QByteArray& packet)
{
    if (packet.isEmpty())
        return;

    tvr_Downlink downlink = tvr_Downlink_init_default;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(packet.constData());
    size_t size = static_cast<size_t>(packet.size());

    rp_packet_decode_result_t result =
        rp_packet_decode(data, size, &tvr_Downlink_msg, &downlink);


    if (result.status != RP_CODEC_OK) {
        m_rawPacketLog += QStringLiteral("[decode error %1]\n").arg(result.status);
        emit rawPacketLogChanged();
        return;
    }

    // Log decoded fields as readable text
    if (downlink.which_payload == tvr_Downlink_telemetry_tag) {
        const tvr_TelemetryState* t = &downlink.payload.telemetry;
        QString line = QStringLiteral("TELEM t=%1 state=%2 thrust=%3 gx=%4 gy=%5")
            .arg(t->timestamp_ms)
            .arg(t->flight_state)
            .arg(static_cast<double>(t->thrust_cmd))
            .arg(static_cast<double>(t->gimbal_x))
            .arg(static_cast<double>(t->gimbal_y));
        if (t->has_position)
            line += QStringLiteral(" pos=%1,%2,%3")
                .arg(static_cast<double>(t->position.x))
                .arg(static_cast<double>(t->position.y))
                .arg(static_cast<double>(t->position.z));
        if (t->has_velocity)
            line += QStringLiteral(" vel=%1,%2,%3")
                .arg(static_cast<double>(t->velocity.x))
                .arg(static_cast<double>(t->velocity.y))
                .arg(static_cast<double>(t->velocity.z));
        if (t->has_attitude)
            line += QStringLiteral(" att=%1,%2,%3,%4")
                .arg(static_cast<double>(t->attitude.w))
                .arg(static_cast<double>(t->attitude.x))
                .arg(static_cast<double>(t->attitude.y))
                .arg(static_cast<double>(t->attitude.z));
        if (t->has_angular_rate)
            line += QStringLiteral(" gyro=%1,%2,%3")
                .arg(static_cast<double>(t->angular_rate.x))
                .arg(static_cast<double>(t->angular_rate.y))
                .arg(static_cast<double>(t->angular_rate.z));
        m_rawPacketLog += line + "\n";
    } else if (downlink.which_payload == tvr_Downlink_status_tag) {
        const tvr_SystemStatus* s = &downlink.payload.status;
        m_rawPacketLog += QStringLiteral(
            "STATUS t=%1 up=%2 state=%3 accel=%4 gyro=%5 b1=%6 b2=%7 gps=%8 rx=%9 tx=%10 cmd=%11\n")
            .arg(s->timestamp_ms)
            .arg(s->uptime_ms)
            .arg(s->flight_state)
            .arg(s->accel_ok)
            .arg(s->gyro_ok)
            .arg(s->baro1_ok)
            .arg(s->baro2_ok)
            .arg(s->gps_connected)
            .arg(s->radio_rx_count)
            .arg(s->radio_tx_count)
            .arg(s->cmd_rx_count);
    }
    emit rawPacketLogChanged();

    writeCsvRow(&downlink);
    applyDownlink(which, &downlink);
}

void SensorDataModel::updateKalman(double rawAngleX, double filteredAngleX,
                                   double rawAngleY, double filteredAngleY,
                                   double rawAngleZ, double filteredAngleZ)
{
    m_rawAngleX = rawAngleX;
    m_filteredAngleX = filteredAngleX;
    m_rawAngleY = rawAngleY;
    m_filteredAngleY = filteredAngleY;
    m_rawAngleZ = rawAngleZ;
    m_filteredAngleZ = filteredAngleZ;

    emit kalmanDataChanged();
}

void SensorDataModel::updatePosition(double altitude, double posX, double posY)
{
    m_altitude = altitude;
    m_posX     = posX;
    m_posY     = posY;

    emit baroDataChanged();
}

void SensorDataModel::updateEngine(double thrustCmd, double gimbalX, double gimbalY)
{
    m_thrustCmd = thrustCmd;
    m_gimbalX   = gimbalX;
    m_gimbalY   = gimbalY;

    emit engineDataChanged();
}

void SensorDataModel::updateTelemetry(double velocity)
{
    m_velocity = velocity;

    emit telemetryDataChanged();
}

void SensorDataModel::applyDownlink(int which, const void* downlinkStruct)
{
    Q_UNUSED(which);
    const tvr_Downlink* d = static_cast<const tvr_Downlink*>(downlinkStruct);

    if (d->which_payload == tvr_Downlink_telemetry_tag) {
        const tvr_TelemetryState* t = &d->payload.telemetry;

        if (kDownlinkDebug) {
            qDebug() << "TelemetryState: has_pos=" << t->has_position
                     << "has_vel=" << t->has_velocity
                     << "has_att=" << t->has_attitude
                     << "flight_state=" << t->flight_state;
        }

        // Velocity magnitude in m/s (matches altitude unit).
        double vel = m_velocity;
        if (t->has_velocity) {
            const tvr_Vec3* v = &t->velocity;
            double vx = static_cast<double>(v->x), vy = static_cast<double>(v->y), vz = static_cast<double>(v->z);
            vel = std::sqrt(vx * vx + vy * vy + vz * vz);
        }

        // Filtered Euler angles (deg) from attitude quaternion.
        double filtX = 0.0, filtY = 0.0, filtZ = 0.0;
        if (t->has_attitude) {
            float roll, pitch, yaw;
            quatToEulerRad(t->attitude.w, t->attitude.x, t->attitude.y, t->attitude.z,
                           &roll, &pitch, &yaw);
            filtX = radToDeg(roll);
            filtY = radToDeg(pitch);
            filtZ = radToDeg(yaw);
        }

        // Raw angular rates (rad/s) → deg/s for display alongside Euler angles.
        double rawX = 0.0, rawY = 0.0, rawZ = 0.0;
        if (t->has_angular_rate) {
            rawX = radToDeg(t->angular_rate.x);
            rawY = radToDeg(t->angular_rate.y);
            rawZ = radToDeg(t->angular_rate.z);
        }

        // Position [m]: altitude from z, horizontal from x/y.
        double alt = m_altitude, px = m_posX, py = m_posY;
        if (t->has_position) {
            alt = static_cast<double>(t->position.z);
            px  = static_cast<double>(t->position.x);
            py  = static_cast<double>(t->position.y);
        }

        // Flight state from TelemetryState (10Hz update).
        m_flightState = static_cast<int>(t->flight_state);

        updateKalman(rawX, filtX, rawY, filtY, rawZ, filtZ);
        updatePosition(alt, px, py);
        updateTelemetry(vel);
        updateEngine(
            static_cast<double>(t->thrust_cmd),
            static_cast<double>(t->gimbal_x),
            static_cast<double>(t->gimbal_y)
        );

        // Emit statusReceived so flightState binding updates from telemetry too.
        emit statusReceived();

    } else if (d->which_payload == tvr_Downlink_status_tag) {
        const tvr_SystemStatus* s = &d->payload.status;

        if (kDownlinkDebug) {
            qDebug() << "SystemStatus: flight_state=" << s->flight_state
                     << "accel=" << s->accel_ok << "gyro=" << s->gyro_ok
                     << "baro1=" << s->baro1_ok << "baro2=" << s->baro2_ok
                     << "gps=" << s->gps_connected
                     << "uptime=" << s->uptime_ms;
        }

        m_flightState  = static_cast<int>(s->flight_state);
        m_uptimeMs     = s->uptime_ms;
        m_accelOk      = s->accel_ok;
        m_gyroOk       = s->gyro_ok;
        m_baro1Ok      = s->baro1_ok;
        m_baro2Ok      = s->baro2_ok;
        m_gpsConnected = s->gps_connected;
        m_radioRxCount = s->radio_rx_count;
        m_radioTxCount = s->radio_tx_count;
        m_cmdRxCount   = s->cmd_rx_count;
        m_lastStatusMs = QDateTime::currentMSecsSinceEpoch();

        emit statusReceived();
    }
}

void SensorDataModel::writeCsvRow(const void* downlinkStruct)
{
    if (!m_csvStream)
        return;

    const tvr_Downlink* d = static_cast<const tvr_Downlink*>(downlinkStruct);
    const qint64 wallMs = QDateTime::currentMSecsSinceEpoch();

    if (d->which_payload == tvr_Downlink_telemetry_tag) {
        const tvr_TelemetryState* t = &d->payload.telemetry;
        (*m_csvStream) << wallMs << ",TELEM," << t->timestamp_ms << ',' << t->flight_state << ',';

        // position
        if (t->has_position) {
            (*m_csvStream) << fmt(t->position.x) << ',' << fmt(t->position.y) << ',' << fmt(t->position.z) << ',';
        } else {
            (*m_csvStream) << ",,,";
        }
        // velocity
        if (t->has_velocity) {
            (*m_csvStream) << fmt(t->velocity.x) << ',' << fmt(t->velocity.y) << ',' << fmt(t->velocity.z) << ',';
        } else {
            (*m_csvStream) << ",,,";
        }
        // attitude quaternion
        if (t->has_attitude) {
            (*m_csvStream) << fmt(t->attitude.w) << ',' << fmt(t->attitude.x) << ','
                           << fmt(t->attitude.y) << ',' << fmt(t->attitude.z) << ',';
        } else {
            (*m_csvStream) << ",,,,";
        }
        // gyro / angular rate
        if (t->has_angular_rate) {
            (*m_csvStream) << fmt(t->angular_rate.x) << ',' << fmt(t->angular_rate.y) << ','
                           << fmt(t->angular_rate.z) << ',';
        } else {
            (*m_csvStream) << ",,,";
        }
        // engine
        (*m_csvStream) << fmt(t->thrust_cmd) << ',' << fmt(t->gimbal_x) << ',' << fmt(t->gimbal_y) << ',';
        // status columns empty
        (*m_csvStream) << ",,,,,,,," << '\n';
    } else if (d->which_payload == tvr_Downlink_status_tag) {
        const tvr_SystemStatus* s = &d->payload.status;
        (*m_csvStream) << wallMs << ",STATUS," << s->timestamp_ms << ',' << s->flight_state << ',';
        // empty telemetry columns (pos, vel, att, gyro, engine)
        (*m_csvStream) << ",,,"     // pos
                       << ",,,"     // vel
                       << ",,,,"    // att
                       << ",,,"     // gyro
                       << ",,,";    // engine
        (*m_csvStream) << s->uptime_ms << ','
                       << fmtBool(s->accel_ok) << ','
                       << fmtBool(s->gyro_ok) << ','
                       << fmtBool(s->baro1_ok) << ','
                       << fmtBool(s->baro2_ok) << ','
                       << fmtBool(s->gps_connected) << ','
                       << s->radio_rx_count << ','
                       << s->radio_tx_count << ','
                       << s->cmd_rx_count
                       << '\n';
    }
    m_csvStream->flush();
}

import QtQuick
import QtQuick.Layouts
import "../Items"

BasePanel {
    id: panel_System_Health

    // Consider status fresh if we've received a SystemStatus in the last N seconds.
    readonly property int freshnessWindowMs: 3000

    // Wall-clock "now" driven by the Timer below. Only updates while linkUp so
    // the "last status" counter doesn't tick forever when no radio is connected.
    property real _nowMs: Date.now()

    // True when at least one radio port is open. Refreshed on every connect/
    // disconnect signal from the bridge.
    property bool linkUp: bridge.isConnected(1) || bridge.isConnected(2)

    Connections {
        target: bridge
        function onConnectedChanged(which, connected) {
            panel_System_Health.linkUp = bridge.isConnected(1) || bridge.isConnected(2)
        }
    }

    readonly property real statusAgeMs: linkUp
        ? Math.max(0, _nowMs - sensorData.lastStatusMs)
        : 0
    readonly property bool statusFresh: linkUp
                                        && sensorData.lastStatusMs > 0
                                        && statusAgeMs < freshnessWindowMs
    readonly property bool allSensorsOk: sensorData.accelOk && sensorData.gyroOk
                                         && sensorData.baro1Ok && sensorData.baro2Ok

    readonly property color overallColor:
        !linkUp        ? Theme.textTertiary
      : !statusFresh   ? Theme.warn
      : allSensorsOk   ? Theme.success
      :                  Theme.danger
    readonly property string overallLabel:
        !linkUp      ? "NO LINK"
      : !statusFresh ? (sensorData.lastStatusMs > 0 ? "STALE" : "NO DATA")
      : allSensorsOk ? "NOMINAL"
      :                "FAULT"

    function sensorOk(name) {
        switch (name) {
        case "ACCEL":  return sensorData.accelOk
        case "GYRO":   return sensorData.gyroOk
        case "BARO 1": return sensorData.baro1Ok
        case "BARO 2": return sensorData.baro2Ok
        default: return false
        }
    }

    function formatAge(ms) {
        if (!linkUp) return "disconnected"
        if (sensorData.lastStatusMs <= 0) return "—"
        const s = ms / 1000
        if (s < 60) return s.toFixed(1) + "s ago"
        return Math.floor(s / 60) + "m ago"
    }

    Timer {
        interval: 500
        running: panel_System_Health.linkUp
        repeat: true
        onTriggered: panel_System_Health._nowMs = Date.now()
    }

    BaseHeader {
        id: header
        headerText: "System Health"
    }

    // ── Overall badge ──────────────────────────────────────────────────────
    Rectangle {
        id: overallBadge
        anchors.verticalCenter: header.verticalCenter
        anchors.right: parent.right
        anchors.rightMargin: 15
        height: 26
        width: overallText.implicitWidth + 20
        radius: Theme.radiusControl
        color: panel_System_Health.overallColor

        Text {
            id: overallText
            anchors.centerIn: parent
            text: panel_System_Health.overallLabel
            color: Theme.textPrimary
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontCaption
            font.bold: true
        }
    }

    // ── Section 1: Sensor health badges ────────────────────────────────────

    Text {
        id: sensorLabel
        anchors {
            top: header.bottom
            left: parent.left
            leftMargin: 15
            topMargin: 6
        }
        text: "Sensor Status"
        font.family: Theme.fontFamily
        font.pixelSize: 15
        color: Theme.textSecondary
    }

    Row {
        id: sensorRow
        anchors {
            top: sensorLabel.bottom
            left: parent.left
            right: parent.right
            leftMargin: 15
            rightMargin: 15
            topMargin: 8
        }
        spacing: 8

        Repeater {
            id: sensorRepeater
            model: ["ACCEL", "GYRO", "BARO 1", "BARO 2"]

            delegate: Column {
                spacing: 4
                width: (sensorRow.width - sensorRow.spacing * 3) / 4

                Rectangle {
                    width: parent.width
                    height: 28
                    radius: Theme.radiusControl
                    color: !panel_System_Health.linkUp
                        ? Theme.surfaceElevated
                        : (panel_System_Health.sensorOk(modelData) ? Theme.success : Theme.danger)

                    Text {
                        anchors.centerIn: parent
                        text: !panel_System_Health.linkUp
                            ? "—"
                            : (panel_System_Health.sensorOk(modelData) ? "OK" : "FAIL")
                        font.family: Theme.fontFamily
                        font.pixelSize: 13
                        font.bold: true
                        color: Theme.textPrimary
                    }
                }

                Text {
                    width: parent.width
                    horizontalAlignment: Text.AlignHCenter
                    text: modelData
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontCaption
                    color: Theme.textTertiary
                }
            }
        }
    }

    // ── Section 2: System stats ─────────────────────────────────────────────

    Text {
        id: statsLabel
        anchors {
            top: sensorRow.bottom
            left: parent.left
            leftMargin: 15
            topMargin: 14
        }
        text: "System Stats"
        font.family: Theme.fontFamily
        font.pixelSize: 15
        color: Theme.textSecondary
    }

    DataBoxList {
        id: statsBoxes
        anchors {
            top: statsLabel.bottom
            left: parent.left
            right: parent.right
            leftMargin: 15
            topMargin: 8
        }

        size: 2
        boxHeight: 56
        dataNames: ["UPTIME (s)", "LAST STATUS (s)"]
        dataValues: panel_System_Health.linkUp
            ? [sensorData.uptimeMs / 1000, panel_System_Health.statusAgeMs / 1000]
            : [NaN, NaN]
    }

    Text {
        anchors {
            top: statsBoxes.bottom
            left: parent.left
            leftMargin: 15
            topMargin: 10
        }
        text: "Last status: " + panel_System_Health.formatAge(panel_System_Health.statusAgeMs)
        color: !panel_System_Health.linkUp
            ? Theme.textTertiary
            : (panel_System_Health.statusFresh ? Theme.textSecondary : Theme.warn)
        font.family: Theme.fontFamily
        font.pixelSize: Theme.fontCaption
    }
}

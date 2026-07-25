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

    // ── Overall badge — outline chip in the state color ────────────────────
    Rectangle {
        id: overallBadge
        anchors.verticalCenter: header.verticalCenter
        anchors.right: parent.right
        anchors.rightMargin: 15
        height: overallText.implicitHeight + 8
        width: overallText.implicitWidth + 20
        radius: 0
        color: "transparent"
        border.width: 1
        border.color: panel_System_Health.overallColor

        Text {
            id: overallText
            anchors.centerIn: parent
            text: panel_System_Health.overallLabel
            color: panel_System_Health.overallColor
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontCaption
            font.bold: true
            font.letterSpacing: 2
        }
    }

    // ── Sensor health tiles (ACCEL / GYRO) ─────────────────────────────────

    Row {
        id: sensorRow
        anchors {
            top: header.bottom
            left: parent.left
            right: parent.right
            leftMargin: 15
            rightMargin: 15
            topMargin: 2
        }
        spacing: 8

        Repeater {
            id: sensorRepeater
            model: ["ACCEL", "GYRO"]

            delegate: Rectangle {
                width: (sensorRow.width - sensorRow.spacing * (sensorRepeater.count - 1))
                       / sensorRepeater.count
                height: 46
                radius: 0
                color: !panel_System_Health.linkUp
                    ? Theme.surfaceElevated
                    : (panel_System_Health.sensorOk(modelData) ? Theme.successBg : Theme.dangerBg)
                border.width: 1
                border.color: !panel_System_Health.linkUp
                    ? Theme.divider
                    : (panel_System_Health.sensorOk(modelData) ? Theme.successBorder : Theme.danger)

                Column {
                    anchors.centerIn: parent
                    spacing: 2

                    Text {
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: !panel_System_Health.linkUp
                            ? "—"
                            : (panel_System_Health.sensorOk(modelData) ? "OK" : "FAIL")
                        font.family: Theme.fontFamily
                        font.pixelSize: 14
                        font.bold: true
                        color: !panel_System_Health.linkUp
                            ? Theme.textTertiary
                            : (panel_System_Health.sensorOk(modelData) ? Theme.success : Theme.danger)
                    }

                    Text {
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: modelData
                        font.family: Theme.fontFamily
                        font.pixelSize: 9
                        font.letterSpacing: 2
                        color: Theme.textTertiary
                    }
                }
            }
        }
    }

    // ── System stats tiles (UPTIME / LAST STATUS) ──────────────────────────

    DataBoxList {
        id: statsBoxes
        anchors {
            top: sensorRow.bottom
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
        text: "LAST STATUS: "
              + panel_System_Health.formatAge(panel_System_Health.statusAgeMs).toUpperCase()
        color: !panel_System_Health.linkUp
            ? Theme.textTertiary
            : (panel_System_Health.statusFresh ? Theme.textSecondary : Theme.warn)
        font.family: Theme.fontFamily
        font.pixelSize: 10
        font.letterSpacing: 1
    }
}

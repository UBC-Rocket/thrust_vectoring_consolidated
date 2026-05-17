import QtQuick
import "Items"

Rectangle {
    id: header

    property alias modeBar: modeBarInstance

    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    height: 64
    color: Theme.background

    function flightStateLabel(state) {
        switch (state) {
            case 0: return "IDLE"
            case 1: return "ESTOP"
            case 2: return "RISE"
            case 3: return "HOVER"
            case 4: return "LOWER"
            default: return "UNKNOWN"
        }
    }

    function flightStateColor(state) {
        switch (state) {
            case 1: return Theme.danger
            case 2: return Theme.success
            case 3: return Theme.success
            case 4: return Theme.warn
            default: return Theme.textTertiary
        }
    }

    // ── Title block (top-left) ──────────────────────────────────────────
    Column {
        id: titleBlock
        anchors.left: parent.left
        anchors.leftMargin: 20
        anchors.verticalCenter: parent.verticalCenter
        spacing: 0

        Text {
            text: "Rocket Ground Control"
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontH1
            font.bold: true
            color: Theme.accent
        }
        Text {
            text: "Ulysses"
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontCaption
            color: Theme.textSecondary
        }
    }

    // ── Mode tab bar (center) ───────────────────────────────────────────
    ModeBar {
        id: modeBarInstance
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        labels: ["Flight", "Tuning", "Map", "Diagnostics"]
    }

    // ── Flight-state badge (top-right) ──────────────────────────────────
    Rectangle {
        id: flightStateBadge
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        anchors.rightMargin: 20
        width: flightStateText.implicitWidth + 24
        height: 32
        radius: Theme.radiusPanel
        color: header.flightStateColor(sensorData.flightState)

        Text {
            id: flightStateText
            anchors.centerIn: parent
            text: header.flightStateLabel(sensorData.flightState)
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontH2
            font.bold: true
            color: Theme.textPrimary
        }
    }

    Rectangle {
        id: line
        anchors.bottom: parent.bottom
        color: Theme.divider
        width: parent.width
        height: 1
    }
}

import QtQuick
import QtQuick.Layouts
import "Items"

Rectangle {
    id: header

    property alias modeBar: modeBarInstance

    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    height: 78
    color: "transparent"

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
            case 0: return Theme.amber
            case 1: return Theme.alarm
            case 2: return Theme.success
            case 3: return Theme.success
            case 4: return Theme.amber
            default: return Theme.textTertiary
        }
    }

    // Alarm takeover wins over the firmware flight state on the badge.
    readonly property color badgeColor: UiState.alarmActive ? Theme.alarm
                                                            : flightStateColor(sensorData.flightState)
    readonly property string badgeLabel: UiState.alarmActive ? "ABORT"
                                                             : flightStateLabel(sensorData.flightState)

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 6
        anchors.rightMargin: 6
        anchors.topMargin: 16
        anchors.bottomMargin: 12
        spacing: 22

        // ── Logo chip (light plate, right edge cut 12px) ────────────────
        Item {
            Layout.preferredWidth: logoImage.width + 28 + 12
            Layout.preferredHeight: 46

            ClippedRect {
                anchors.fill: parent
                slantBR: 12
                fillColor: Theme.logoChip
            }

            Image {
                id: logoImage
                source: "../Resources/images/ubc-rocket-logo.png"
                height: 34
                fillMode: Image.PreserveAspectFit
                anchors.left: parent.left
                anchors.leftMargin: 14
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        // ── Title block ──────────────────────────────────────────────────
        ColumnLayout {
            spacing: 1

            RowLayout {
                spacing: 12
                Text {
                    text: "ROCKET GROUND CONTROL"
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontH1
                    font.bold: true
                    font.letterSpacing: 2
                    color: Theme.textPrimary
                }
                JpText {
                    Layout.alignment: Qt.AlignBaseline
                    text: "地上管制システム"
                    font.pixelSize: 15
                    color: Theme.accent
                }
            }
            Text {
                text: "ULYSSES · THRUST VECTOR PROGRAM"
                font.family: Theme.fontFamily
                font.pixelSize: 11
                font.letterSpacing: 4
                color: Theme.textSecondary
            }
        }

        Item { Layout.fillWidth: true }

        // ── Mode tab bar ─────────────────────────────────────────────────
        ModeBar {
            id: modeBarInstance
            Layout.alignment: Qt.AlignVCenter
            labels: ["Flight", "Tuning", "Map", "Diagnostics"]
        }

        // ── Pilot theme switch (normal mode only) ────────────────────────
        // In anime mode the right-strip PILOT SELECT chips take over.
        Item {
            id: pilotSeg
            Layout.alignment: Qt.AlignVCenter
            visible: !UiState.animeMode
            implicitWidth: segRow.implicitWidth + 6
            implicitHeight: segRow.implicitHeight + 6

            ClippedRect {
                anchors.fill: parent
                slantTL: 8
                slantBR: 8
                fillColor: "transparent"
                borderColor: Theme.border
                borderWidth: 1
            }

            Row {
                id: segRow
                anchors.centerIn: parent
                spacing: 6

                Repeater {
                    model: ["ASUKA", "REI", "MITSURI"]
                    delegate: Item {
                        id: seg
                        required property int index
                        required property string modelData
                        readonly property bool active: Theme.pilot === index
                        width: segLabel.implicitWidth + 20
                        height: segLabel.implicitHeight + 12

                        Rectangle {
                            anchors.fill: parent
                            color: seg.active ? Theme.accent : "transparent"
                            border.width: 1
                            border.color: seg.active ? Theme.accent : Theme.border
                        }
                        Text {
                            id: segLabel
                            anchors.centerIn: parent
                            text: seg.modelData
                            font.family: Theme.fontFamilySemiBold
                            font.pixelSize: 10
                            font.letterSpacing: 1
                            color: seg.active ? Theme.background : Theme.textSecondary
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: Theme.pilot = seg.index
                        }
                    }
                }
            }
        }

        // ── ANIME mode toggle ────────────────────────────────────────────
        ClippedButton {
            Layout.alignment: Qt.AlignVCenter
            text: UiState.animeMode ? "ANIME: ON" : "ANIME: OFF"
            cut: 8
            fillColor: "#14ffb400"
            hoverFillColor: "#33ffb400"
            borderColor: Theme.amber
            hoverBorderColor: Theme.amber
            textColor: Theme.amber
            hoverTextColor: Theme.amber
            fontSize: 11
            horizontalPadding: 14
            verticalPadding: 8
            onClicked: UiState.animeMode = !UiState.animeMode
        }

        // ── Sync rate + flight-state badge ───────────────────────────────
        ColumnLayout {
            Layout.alignment: Qt.AlignVCenter
            spacing: 3

            RowLayout {
                Layout.alignment: Qt.AlignRight
                spacing: 6
                Text {
                    text: "SYNC RATE"
                    font.family: Theme.fontFamily
                    font.pixelSize: 11
                    font.letterSpacing: 2
                    color: Theme.textSecondary
                }
                Text {
                    text: UiState.syncRate.toFixed(1) + "%"
                    font.family: Theme.fontFamily
                    font.pixelSize: 14
                    font.bold: true
                    color: Theme.success
                }
            }

            Item {
                id: flightStateBadge
                Layout.alignment: Qt.AlignRight
                Layout.preferredWidth: flightStateText.implicitWidth + 32
                Layout.preferredHeight: flightStateText.implicitHeight + 8

                ClippedRect {
                    anchors.fill: parent
                    slantTL: 8
                    slantBR: 8
                    fillColor: Qt.rgba(header.badgeColor.r, header.badgeColor.g,
                                       header.badgeColor.b, UiState.alarmActive ? 0.18 : 0.10)
                    borderColor: header.badgeColor
                    borderWidth: 1
                }

                Text {
                    id: flightStateText
                    anchors.centerIn: parent
                    text: header.badgeLabel
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    font.bold: true
                    font.letterSpacing: 3
                    color: header.badgeColor
                }
            }
        }
    }

    Rectangle {
        id: line
        anchors.bottom: parent.bottom
        color: Theme.accent
        width: parent.width
        height: 2
    }
}

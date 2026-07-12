import QtQuick
import QtQuick.Layouts
import QtQuick.Effects
import "Items"

// Right side strip (anime mode only): warning stripes, vertical pilot name,
// PILOT SELECT portrait chips, A.T. FIELD readout. No main character image.
Rectangle {
    id: strip

    width: 200

    readonly property var _labels: ["ASUKA", "REI", "MITSURI"]

    gradient: Gradient {
        GradientStop { position: 0.0; color: Theme.strip1 }
        GradientStop { position: 0.6; color: Theme.strip2 }
        GradientStop { position: 1.0; color: Theme.surfaceElevated }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: 2
        color: Theme.accent
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.leftMargin: 2
        spacing: 0

        WarningStripes {
            Layout.fillWidth: true
            Layout.preferredHeight: 14
            angleDeg: 45
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.topMargin: 14
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            Layout.bottomMargin: 6
            spacing: 4

            Text {
                Layout.alignment: Qt.AlignRight
                text: "緊急発進"
                font.family: Theme.japaneseFamilyHeavy
                font.pixelSize: 26
                font.letterSpacing: 2
                color: Theme.accent
            }
            Text {
                Layout.alignment: Qt.AlignRight
                text: "LAUNCH READY"
                font.family: Theme.fontFamily
                font.pixelSize: 10
                font.letterSpacing: 3
                color: Theme.textSecondary
            }
        }

        // ── PILOT SELECT — portrait chips (drives Theme.pilot) ────────────
        ColumnLayout {
            Layout.fillWidth: true
            Layout.topMargin: 10
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            Layout.bottomMargin: 4
            spacing: 6

            RowLayout {
                Layout.alignment: Qt.AlignRight
                spacing: 6
                Text {
                    text: "PILOT SELECT"
                    font.family: Theme.fontFamily
                    font.pixelSize: 9
                    font.letterSpacing: 2
                    color: Theme.textTertiary
                }
                JpText {
                    text: "操縦者"
                    color: Theme.textTertiary
                }
            }

            Row {
                Layout.alignment: Qt.AlignRight
                spacing: 6

                Repeater {
                    model: 3
                    delegate: Item {
                        id: chip
                        required property int index
                        readonly property bool active: Theme.pilot === index
                        width: 54
                        height: 74

                        Rectangle {
                            anchors.fill: parent
                            color: Theme.surfaceElevated
                            border.width: 2
                            border.color: chip.active ? Theme.accent : "#59808080"
                        }

                        Image {
                            id: thumb
                            anchors.fill: parent
                            anchors.margins: 2
                            source: "../Resources/images/" + Theme.pilots[chip.index].img
                            fillMode: Image.PreserveAspectCrop
                            verticalAlignment: Image.AlignTop
                            visible: status === Image.Ready
                            layer.enabled: !chip.active
                            layer.effect: MultiEffect {
                                saturation: -1.0
                                brightness: -0.55
                            }
                        }

                        // Bottom label bar.
                        Rectangle {
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.bottom: parent.bottom
                            anchors.margins: 2
                            height: chipLabel.implicitHeight + 4
                            color: "#99000000"
                            Text {
                                id: chipLabel
                                anchors.centerIn: parent
                                text: strip._labels[chip.index]
                                font.family: Theme.fontFamilySemiBold
                                font.pixelSize: 7
                                font.letterSpacing: 0.5
                                color: chip.active ? Theme.accent : "#9a9a9a"
                            }
                        }

                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: Theme.pilot = chip.index
                        }
                    }
                }
            }
        }

        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true

            // Inset decorative panel with faint -45° accent stripes.
            Rectangle {
                anchors.fill: parent
                anchors.margins: 14
                color: "transparent"
                border.width: 1
                border.color: Theme.divider

                WarningStripes {
                    anchors.fill: parent
                    anchors.margins: 1
                    colorA: Theme.accentTintFaint
                    colorB: "transparent"
                    stripeWidth: 18
                    angleDeg: -45
                }
            }

            // Active pilot's name, vertical-rl with amber glow.
            Text {
                id: verticalName
                anchors.top: parent.top
                anchors.topMargin: 12
                anchors.right: parent.right
                anchors.rightMargin: 14
                text: Theme.pilotName.split("").join("\n")
                font.family: Theme.japaneseFamily
                font.pixelSize: 20
                lineHeight: 1.4
                horizontalAlignment: Text.AlignHCenter
                color: Theme.amber
                visible: false
            }

            MultiEffect {
                source: verticalName
                anchors.fill: verticalName
                shadowEnabled: true
                shadowColor: "#80ffb400"
                shadowBlur: 0.7
                blurMax: 24
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: footerRow.implicitHeight + 20
            color: "transparent"

            Rectangle {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                height: 1
                color: Theme.border
            }

            RowLayout {
                id: footerRow
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 12
                Text {
                    text: "A.T. FIELD"
                    font.family: Theme.fontFamily
                    font.pixelSize: 10
                    font.letterSpacing: 2
                    color: Theme.textSecondary
                }
                Item { Layout.fillWidth: true }
                Text {
                    id: atFieldStatus
                    text: "NOMINAL"
                    font.family: Theme.fontFamily
                    font.pixelSize: 14
                    font.bold: true
                    color: Theme.amber
                }
            }

            Blink { target: atFieldStatus; period: 1600 }
        }

        WarningStripes {
            Layout.fillWidth: true
            Layout.preferredHeight: 14
            angleDeg: 45
        }
    }
}

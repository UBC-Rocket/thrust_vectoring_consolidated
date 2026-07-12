import QtQuick
import QtQuick.Layouts
import QtQuick.Effects
import "Items"

// EVA right side strip (anime mode only): decoration — warning stripes,
// vertical katakana, A.T. FIELD readout. No character image here.
Rectangle {
    id: strip

    width: 200

    gradient: Gradient {
        GradientStop { position: 0.0; color: "#1a0908" }
        GradientStop { position: 0.6; color: "#240b08" }
        GradientStop { position: 1.0; color: "#12100a" }
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

        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true

            // Inset decorative panel with faint -45° red stripes.
            Rectangle {
                anchors.fill: parent
                anchors.margins: 14
                color: "transparent"
                border.width: 1
                border.color: Theme.divider

                WarningStripes {
                    anchors.fill: parent
                    anchors.margins: 1
                    colorA: "#0fff3b2f"
                    colorB: "transparent"
                    stripeWidth: 18
                    angleDeg: -45
                }
            }

            // 式波・アスカ・ラングレー, vertical-rl with amber glow.
            Text {
                id: verticalName
                anchors.top: parent.top
                anchors.topMargin: 12
                anchors.right: parent.right
                anchors.rightMargin: 14
                text: "式波・アスカ・ラングレー".split("").join("\n")
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

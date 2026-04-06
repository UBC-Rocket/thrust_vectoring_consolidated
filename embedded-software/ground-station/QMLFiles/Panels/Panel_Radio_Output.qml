import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "../Items"

Pane {
    id: radioOutWin
    padding: 0
    font.family: Theme.fontFamily

    background: Rectangle {
        color: Theme.background
        border.color: Theme.border
        border.width: Theme.strokePanel
        radius: Theme.radiusPanel
    }

    palette {
        window:          Theme.background
        base:            Theme.surfaceInset
        alternateBase:   Theme.surfaceElevated
        text:            Theme.textPrimary
        windowText:      Theme.textPrimary
        button:          Theme.btnSecondaryBg
        buttonText:      Theme.btnSecondaryText
        highlight:       Theme.accent
        highlightedText: Theme.background
        placeholderText: Theme.textTertiary
        mid:             Theme.border
        dark:            Theme.border
        light:           Theme.borderLight
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: Theme.paddingMd
        spacing: Theme.paddingSm

        Label {
            text: "Radio Output — Raw Packets"
            color: Theme.accent
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontH1
            font.bold: true
            Layout.fillWidth: true
        }

        Flickable {
            id: outputFlick
            Layout.fillWidth: true
            Layout.fillHeight: true
            contentWidth: width
            contentHeight: outputText.paintedHeight
            clip: true

            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            TextEdit {
                id: outputText
                width: parent.width
                readOnly: true
                selectByMouse: true
                wrapMode: TextEdit.WrapAnywhere
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontBody
                color: Theme.textPrimary
                text: sensorData.rawPacketLog

                onTextChanged: {
                    outputFlick.contentY = Math.max(0, outputFlick.contentHeight - outputFlick.height)
                }
            }
        }

        Button {
            text: "Clear"
            Layout.alignment: Qt.AlignRight
            onClicked: sensorData.clearRawPacketLog()
        }
    }
}

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Dialogs
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

    FileDialog {
        id: saveDialog
        title: "Record telemetry to CSV"
        fileMode: FileDialog.SaveFile
        nameFilters: ["CSV files (*.csv)", "All files (*)"]
        currentFolder: "file://" + sensorData.defaultCsvPath.substring(0, sensorData.defaultCsvPath.lastIndexOf("/"))
        currentFile: "file://" + sensorData.defaultCsvPath
        onAccepted: {
            const path = selectedFile.toString().replace(/^file:\/\//, "")
            sensorData.startCsvRecording(path)
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: Theme.paddingMd
        spacing: Theme.paddingSm

        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.paddingMd

            Label {
                text: "Radio Output — Raw Packets"
                color: Theme.accent
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontH1
                font.bold: true
                Layout.fillWidth: true
                elide: Label.ElideRight
            }

            // Recording indicator dot — pulses while capturing.
            Rectangle {
                visible: sensorData.isRecording
                width: 10; height: 10; radius: 5
                color: Theme.danger
                Layout.alignment: Qt.AlignVCenter
                SequentialAnimation on opacity {
                    running: sensorData.isRecording
                    loops: Animation.Infinite
                    NumberAnimation { to: 0.25; duration: 600 }
                    NumberAnimation { to: 1.0;  duration: 600 }
                }
            }

            Label {
                visible: sensorData.isRecording
                text: "REC " + sensorData.currentCsvPath
                color: Theme.textSecondary
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontCaption
                elide: Label.ElideLeft
                Layout.maximumWidth: 320
            }

            Button {
                text: sensorData.isRecording ? "Stop" : "Record CSV…"
                onClicked: {
                    if (sensorData.isRecording)
                        sensorData.stopCsvRecording()
                    else
                        saveDialog.open()
                }
            }
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

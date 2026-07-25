import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Dialogs
import QtQuick.Layouts 1.15
import "../Items"

// Raw radio packet log + CSV recording — EVA-02 diagnostics skin.
Pane {
    id: radioOutWin
    padding: 0
    font.family: Theme.fontFamily

    // EVA panel: dark red surface, 1px border, 3px red top accent. Radius 0.
    background: Rectangle {
        color: Theme.surface
        border.color: Theme.border
        border.width: Theme.strokePanel
        radius: 0

        Rectangle {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            height: Theme.accentStroke
            color: Theme.accent
        }
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

    // Dim outline utility button (CLEAR / CHOOSE PATH…).
    component DimButton: ClippedButton {
        cut: 0
        bold: false
        fontSize: 10
        fontLetterSpacing: 2
        verticalPadding: 5
        horizontalPadding: 12
        fillColor: "transparent"
        hoverFillColor: "transparent"
        textColor: Theme.btnSecondaryText
        hoverTextColor: Theme.accentMuted
        borderColor: Theme.btnSecondaryBorder
        hoverBorderColor: Theme.accent
    }

    // Rich-text wrapper for the raw packet stream: HTML-escapes the log,
    // tints the TELEM keyword, and sets the console line-height. The feed
    // itself is unchanged — still the live sensorData.rawPacketLog binding.
    function fmtPackets(s) {
        let t = s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;")
        t = t.replace(/TELEM/g, "<span style=\"color:" + Theme.accentMuted + ";\">TELEM</span>")
        return "<div style=\"white-space:pre-wrap; line-height:170%;\">" + t + "</div>"
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

    // The log string grows without bound and the TextEdit re-lays-out the whole
    // thing on every packet, which drags the UI down in long sessions — wipe it
    // every 10 s. The manual Clear button still works between wipes, and CSV
    // recording is unaffected (it writes straight to disk).
    Timer {
        interval: 10000
        running: true
        repeat: true
        onTriggered: sensorData.clearRawPacketLog()
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: Theme.paddingMd
        anchors.topMargin: Theme.paddingMd + Theme.accentStroke
        spacing: 12

        // Banner shown when auto-CSV (D6) failed to open a file. Stays up until
        // the next successful start. Operator sees a red strip with the reason
        // instead of silently believing recording is on.
        Rectangle {
            Layout.fillWidth: true
            visible: sensorData.lastCsvError.length > 0 && !sensorData.isRecording
            color: Theme.dangerBg
            border.color: Theme.danger
            border.width: 1
            radius: 0
            implicitHeight: csvErrorRow.implicitHeight + 12

            RowLayout {
                id: csvErrorRow
                anchors.fill: parent
                anchors.leftMargin: 10
                anchors.rightMargin: 10
                spacing: 8

                Label {
                    text: "CSV recording failed: " + sensorData.lastCsvError
                    color: Theme.dangerText
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    Layout.fillWidth: true
                    wrapMode: Text.WordWrap
                }
                DimButton {
                    text: "CHOOSE PATH…"
                    onClicked: saveDialog.open()
                }
            }
        }

        // Header row: heading + JP label | REC dot · filename · STOP
        RowLayout {
            Layout.fillWidth: true
            spacing: 10

            Text {
                text: "RADIO OUTPUT — RAW PACKETS"
                color: Theme.accentMuted
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontH2
                font.bold: true
                font.letterSpacing: 2
                elide: Text.ElideRight
                Layout.maximumWidth: radioOutWin.availableWidth * 0.6
            }

            JpText {
                text: "受信記録"
                color: Theme.textTertiary
            }

            Item { Layout.fillWidth: true }

            // Recording indicator — 9px red square, stepped blink while capturing.
            Rectangle {
                id: recDot
                visible: sensorData.isRecording
                implicitWidth: 9
                implicitHeight: 9
                radius: 0
                color: Theme.accent
                Layout.alignment: Qt.AlignVCenter

                Blink { target: recDot; period: 1200 }
            }

            Label {
                visible: sensorData.isRecording
                text: "REC · " + sensorData.currentCsvPath
                color: Theme.textSecondary
                font.family: Theme.monoFamily
                font.pixelSize: 10
                font.letterSpacing: 1
                elide: Label.ElideLeft
                Layout.maximumWidth: 320
            }

            ClippedButton {
                cut: 0
                bold: false
                fontSize: 10
                fontLetterSpacing: 2
                verticalPadding: 5
                horizontalPadding: 14
                fillColor: sensorData.isRecording ? Theme.accentTint : "transparent"
                hoverFillColor: sensorData.isRecording ? Theme.accentTintHover : "transparent"
                borderColor: sensorData.isRecording ? Theme.accent : Theme.btnSecondaryBorder
                hoverBorderColor: Theme.accent
                textColor: sensorData.isRecording ? Theme.accentMuted : Theme.btnSecondaryText
                hoverTextColor: Theme.accentMuted
                text: sensorData.isRecording ? "STOP" : "RECORD CSV…"
                onClicked: {
                    if (sensorData.isRecording)
                        sensorData.stopCsvRecording()
                    else
                        saveDialog.open()
                }
            }
        }

        // Raw packet console
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 420
            radius: 0
            color: Theme.sceneBackground
            border.width: 1
            border.color: Theme.divider

            Flickable {
                id: outputFlick
                anchors.fill: parent
                anchors.margins: 12
                contentWidth: width
                contentHeight: outputText.paintedHeight
                clip: true

                ScrollBar.vertical: ThemedScrollBar { policy: ScrollBar.AsNeeded }

                TextEdit {
                    id: outputText
                    width: outputFlick.width
                    readOnly: true
                    selectByMouse: true
                    wrapMode: TextEdit.WrapAnywhere
                    textFormat: TextEdit.RichText
                    font.family: Theme.monoFamily
                    font.pixelSize: 12
                    color: "#d8b8a4"
                    text: radioOutWin.fmtPackets(sensorData.rawPacketLog)

                    onTextChanged: {
                        outputFlick.contentY = Math.max(0, outputFlick.contentHeight - outputFlick.height)
                    }
                }
            }
        }

        DimButton {
            text: "CLEAR"
            Layout.alignment: Qt.AlignRight
            onClicked: sensorData.clearRawPacketLog()
        }
    }
}

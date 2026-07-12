import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Controls.Basic as Basic   // skinnable controls
import "../Items"

BasePanel {
    id: panel

    // ---------- Header (left aligned) ----------
    RowLayout {
        id: headerRow
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            margins: 14
        }
        spacing: 8

        Text {
            id: header_System_Alert
            text: "SYSTEM ALERT"
            color: Theme.accentMuted
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontH2
            font.bold: true
            font.letterSpacing: 2
        }

        Item { Layout.fillWidth: true }

        // compact CLEAR button — dim outline utility style
        Basic.Button {
            id: clearBtn
            text: "CLEAR"
            hoverEnabled: true
            leftPadding: 12
            rightPadding: 12
            topPadding: 4
            bottomPadding: 4
            font.family: Theme.fontFamily
            font.pixelSize: 10
            font.letterSpacing: 2
            background: Rectangle {
                radius: 0
                color: clearBtn.down ? Theme.btnSecondaryPress : "transparent"
                border.width: Theme.strokeControl
                border.color: clearBtn.hovered ? Theme.accent : Theme.border
            }
            contentItem: Text {
                text: clearBtn.text
                color: clearBtn.hovered ? Theme.accentMuted : Theme.textSecondary
                font: clearBtn.font
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
            onClicked: alertModel.clear()
        }
    }

    // ---------- Log list container ----------
    Rectangle {
        id: inner
        anchors {
            top: headerRow.bottom
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            leftMargin: 14; rightMargin: 14; topMargin: 6; bottomMargin: 14
        }
        radius: 0
        color: "transparent"

        // --------- Model & View (only classified messages) ----------
        ListModel { id: alertModel }  // { ts: Date, level: "error|warning|success", text: string }

        ListView {
            id: list
            anchors.fill: parent
            clip: true
            spacing: 6
            model: alertModel
            boundsBehavior: Flickable.StopAtBounds

            ScrollBar.vertical: Basic.ScrollBar {
                id: control
                policy: ScrollBar.AsNeeded
                contentItem: Rectangle {
                    implicitWidth: 6
                    radius: 0
                    color: parent.pressed ? Theme.textSecondary
                         : parent.hovered ? Theme.textTertiary
                         :                  Theme.borderLight
                }
                background: Rectangle { color: "transparent" }
            }

            function trim() { const max = 400; if (alertModel.count > max) alertModel.remove(0, alertModel.count - max); }
            Component.onCompleted: positionViewAtEnd()

            // ===== Delegate: #12100a row, 3px severity left border =====
            delegate: Rectangle {
                id: card
                width: ListView.view.width
                radius: 0
                color: Theme.surfaceElevated

                // ---- severity colors ----
                property color stripe:     (level==="error")   ? Theme.danger
                                         : (level==="warning") ? Theme.warn
                                         :                       Theme.success
                property color chipText:   (level==="error")   ? Theme.dangerText
                                         : (level==="warning") ? Theme.warnText
                                         :                       Theme.successText
                property color chipBorder: (level==="error")   ? Theme.danger
                                         : (level==="warning") ? Theme.warn
                                         :                       Theme.successBorder
                property color timeText: Theme.textTertiary
                property color bodyText: Theme.textPrimary

                property int padX: 10
                property int padY: 8
                height: Math.max(36, body.implicitHeight + padY*2)

                // 3px severity left border
                Rectangle {
                    anchors {
                        left: parent.left
                        top: parent.top
                        bottom: parent.bottom
                    }
                    width: 3
                    color: card.stripe
                }

                // content: timestamp · severity chip · message
                RowLayout {
                    anchors {
                        fill: parent
                        leftMargin: padX + 3
                        rightMargin: padX
                        topMargin: padY
                        bottomMargin: padY
                    }
                    spacing: 10

                    Text {
                        text: Qt.formatTime(ts, "hh:mm:ss")
                        color: card.timeText
                        font.family: Theme.monoFamily
                        font.pixelSize: 11
                    }

                    Rectangle {
                        id: chip
                        radius: 0
                        Layout.preferredHeight: 18
                        color: "transparent"
                        border.width: 1
                        border.color: card.chipBorder
                        property string label: (level === "error")   ? "ERROR"
                                              : (level === "warning")? "WARN"
                                              : "OK"

                        Text {
                            id: chipText
                            anchors.centerIn: parent
                            text: parent.label
                            color: card.chipText
                            font.family: Theme.fontFamily
                            font.pixelSize: 10
                            font.bold: true
                            padding: 8
                        }

                        Layout.preferredWidth: Math.max(36, chipText.implicitWidth + 2 * chipText.padding)
                    }

                    // message body
                    Text {
                        id: body
                        Layout.fillWidth: true
                        text: model.text
                        color: card.bodyText
                        font.family: Theme.fontFamily
                        font.pixelSize: 13
                        wrapMode: Text.Wrap
                    }
                }
            }
        }
    }

    // ===== Helper to append & autoscroll =====
    function appendLine(level, line) {
        alertModel.append({ ts: new Date(), level: level, text: line })
        list.trim()
        list.positionViewAtEnd()
    }

    // ===== Listen to classified signals =====
    Connections {
        target: alarmreceiver
        function onRxError(line)   { appendLine("error",   line) }
        function onRxWarning(line) { appendLine("warning", line) }
        function onRxSuccess(line) { appendLine("success", line) }
    }
}

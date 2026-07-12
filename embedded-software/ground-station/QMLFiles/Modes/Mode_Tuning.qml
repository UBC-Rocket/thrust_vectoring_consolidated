import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Basic as Basic
import QtQuick.Layouts
import "../Items"
import "../Panels"

Item {
    id: tuningMode

    // ── Themed Save popup (no Qt default chrome) ──────────────────────
    Popup {
        id: savePopup
        modal: true
        focus: true
        padding: 0
        width: 360
        parent: tuningMode.Window.window ? tuningMode.Window.window.contentItem : tuningMode
        anchors.centerIn: parent
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent

        background: Rectangle {
            color: Theme.surfaceSolid
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

        contentItem: ColumnLayout {
            spacing: 12

            Text {
                Layout.fillWidth: true
                Layout.margins: 16
                Layout.bottomMargin: 0
                text: "SAVE PRESET"
                color: Theme.accentMuted
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontH2
                font.bold: true
                font.letterSpacing: 2
            }

            Rectangle {
                Layout.fillWidth: true
                implicitHeight: 1
                color: Theme.divider
            }

            ColumnLayout {
                Layout.fillWidth: true
                Layout.margins: 16
                Layout.topMargin: 4
                spacing: 6

                Text {
                    text: "NAME THIS PRESET"
                    color: Theme.textSecondary
                    font.family: Theme.fontFamily
                    font.pixelSize: 10
                    font.letterSpacing: 2
                }

                Basic.TextField {
                    id: saveNameField
                    Layout.fillWidth: true
                    Layout.preferredHeight: 36
                    placeholderText: "e.g. hover-v1"
                    placeholderTextColor: Theme.textTertiary
                    color: Theme.textPrimary
                    leftPadding: 10
                    rightPadding: 10
                    font.family: Theme.monoFamily
                    font.pixelSize: 14
                    selectByMouse: true
                    background: Rectangle {
                        radius: 0
                        color: Theme.surfaceElevated
                        border.width: Theme.strokeControl
                        border.color: saveNameField.activeFocus ? Theme.accent : Theme.divider
                    }
                    onAccepted: savePopup._commit()
                }
            }

            RowLayout {
                Layout.fillWidth: true
                Layout.margins: 16
                Layout.topMargin: 4
                spacing: 8

                Item { Layout.fillWidth: true }

                Basic.Button {
                    id: cancelBtn
                    text: "CANCEL"
                    padding: 10
                    hoverEnabled: true
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    font.letterSpacing: 2
                    background: Rectangle {
                        radius: 0
                        color: cancelBtn.down    ? Theme.btnSecondaryPress
                             : cancelBtn.hovered ? Theme.btnSecondaryHover
                             :                     Theme.btnSecondaryBg
                        border.width: Theme.strokeControl
                        border.color: cancelBtn.hovered ? Theme.accent : Theme.btnSecondaryBorder
                    }
                    contentItem: Text {
                        anchors.centerIn: parent
                        text: cancelBtn.text
                        color: cancelBtn.hovered ? Theme.accentMuted : Theme.btnSecondaryText
                        font: cancelBtn.font
                    }
                    onClicked: savePopup.close()
                }

                PrimaryButton {
                    text: "Save"
                    onClicked: savePopup._commit()
                }
            }
        }

        function _commit() {
            const name = saveNameField.text.trim()
            if (name.length === 0) return
            if (presetManager.save(name, pidPanel.currentValues())) {
                pidPanel.loadedPresetName = name
                pidPanel.dirty = false
            }
            savePopup.close()
        }

        onOpened: {
            saveNameField.text = pidPanel.loadedPresetName
            saveNameField.selectAll()
            saveNameField.forceActiveFocus()
        }
    }

    // ── Themed Delete confirm popup ───────────────────────────────────
    Popup {
        id: deletePopup
        modal: true
        focus: true
        padding: 0
        width: 360
        parent: tuningMode.Window.window ? tuningMode.Window.window.contentItem : tuningMode
        anchors.centerIn: parent
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent
        property string targetName: ""

        background: Rectangle {
            color: Theme.surfaceSolid
            border.color: Theme.border
            border.width: Theme.strokePanel
            radius: 0

            Rectangle {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                height: Theme.accentStroke
                color: Theme.danger
            }
        }

        contentItem: ColumnLayout {
            spacing: 12

            Text {
                Layout.fillWidth: true
                Layout.margins: 16
                Layout.bottomMargin: 0
                text: "DELETE PRESET"
                color: Theme.danger
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontH2
                font.bold: true
                font.letterSpacing: 2
            }

            Rectangle {
                Layout.fillWidth: true
                implicitHeight: 1
                color: Theme.divider
            }

            Text {
                Layout.fillWidth: true
                Layout.margins: 16
                Layout.topMargin: 4
                text: "Remove preset \"" + deletePopup.targetName + "\"? This cannot be undone."
                color: Theme.textPrimary
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontBody
                wrapMode: Text.WordWrap
            }

            RowLayout {
                Layout.fillWidth: true
                Layout.margins: 16
                Layout.topMargin: 4
                spacing: 8

                Item { Layout.fillWidth: true }

                Basic.Button {
                    id: keepBtn
                    text: "KEEP"
                    padding: 10
                    hoverEnabled: true
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    font.letterSpacing: 2
                    background: Rectangle {
                        radius: 0
                        color: keepBtn.down    ? Theme.btnSecondaryPress
                             : keepBtn.hovered ? Theme.btnSecondaryHover
                             :                   Theme.btnSecondaryBg
                        border.width: Theme.strokeControl
                        border.color: keepBtn.hovered ? Theme.accent : Theme.btnSecondaryBorder
                    }
                    contentItem: Text {
                        anchors.centerIn: parent
                        text: keepBtn.text
                        color: keepBtn.hovered ? Theme.accentMuted : Theme.btnSecondaryText
                        font: keepBtn.font
                    }
                    onClicked: deletePopup.close()
                }

                PrimaryButton {
                    text: "Delete"
                    onClicked: {
                        presetManager.remove(deletePopup.targetName)
                        if (pidPanel.loadedPresetName === deletePopup.targetName) {
                            pidPanel.loadedPresetName = ""
                        }
                        deletePopup.close()
                    }
                }
            }
        }
    }

    // ── 280px | 1fr grid, 14px gap ──────────────────────────────────────
    RowLayout {
        anchors.fill: parent
        spacing: Theme.gridSpacing

        // ── Presets panel (設定保存) — align-self: start ────────────────
        Rectangle {
            id: presetsPanel
            Layout.preferredWidth: 280
            Layout.minimumWidth: 280
            Layout.maximumWidth: 280
            Layout.alignment: Qt.AlignTop
            Layout.fillHeight: false
            Layout.preferredHeight: presetsColumn.implicitHeight + 2 * Theme.paddingMd
            color: Theme.surface
            border.color: Theme.border
            border.width: Theme.strokePanel
            radius: 0

            // 3px red top accent
            Rectangle {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                height: Theme.accentStroke
                color: Theme.accent
            }

            ColumnLayout {
                id: presetsColumn
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.margins: Theme.paddingMd
                spacing: 12

                // Heading row: PRESETS + 設定保存
                Item {
                    Layout.fillWidth: true
                    implicitHeight: presetsTitle.implicitHeight

                    Text {
                        id: presetsTitle
                        text: "PRESETS"
                        color: Theme.accentMuted
                        font.family: Theme.fontFamily
                        font.pixelSize: Theme.fontH2
                        font.bold: true
                        font.letterSpacing: 2
                    }

                    JpText {
                        anchors.left: presetsTitle.right
                        anchors.leftMargin: 10
                        anchors.baseline: presetsTitle.baseline
                        text: "設定保存"
                        color: Theme.textTertiary
                    }
                }

                Text {
                    Layout.fillWidth: true
                    text: "Save the current PID / Reference / Config values for later recall."
                    color: Theme.textSecondary
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontCaption
                    lineHeight: 1.5
                    wrapMode: Text.WordWrap
                }

                PrimaryButton {
                    text: "Save current as…"
                    Layout.fillWidth: true
                    font.pixelSize: 12
                    onClicked: savePopup.open()
                }

                // Preset list box (min-height 260)
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 260
                    Layout.minimumHeight: 260
                    color: Theme.surfaceElevated
                    border.color: Theme.divider
                    border.width: Theme.strokeControl
                    radius: 0

                    ListView {
                        id: presetList
                        anchors.fill: parent
                        anchors.margins: 6
                        clip: true
                        model: presetManager.presets
                        spacing: 2
                        ScrollBar.vertical: ThemedScrollBar { }
                        currentIndex: -1

                        delegate: Rectangle {
                            required property int index
                            required property string modelData
                            width: ListView.view.width
                            height: 32
                            radius: 0
                            color: presetList.currentIndex === index
                                ? Theme.accentSubtle
                                : (mouse.containsMouse ? Theme.surfaceSolid : "transparent")

                            // 3px red left border on the selected row
                            Rectangle {
                                anchors.top: parent.top
                                anchors.bottom: parent.bottom
                                anchors.left: parent.left
                                width: 3
                                color: Theme.accent
                                visible: presetList.currentIndex === index
                            }

                            Text {
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.leftMargin: 12
                                anchors.right: parent.right
                                anchors.rightMargin: 8
                                text: modelData
                                color: Theme.textPrimary
                                font.family: Theme.fontFamily
                                font.pixelSize: Theme.fontBody
                                elide: Text.ElideRight
                            }

                            MouseArea {
                                id: mouse
                                anchors.fill: parent
                                hoverEnabled: true
                                onClicked: presetList.currentIndex = index
                                onDoubleClicked: {
                                    presetList.currentIndex = index
                                    pidPanel.loadValues(presetManager.load(modelData))
                                    pidPanel.loadedPresetName = modelData
                                }
                            }
                        }
                    }
                }

                // LOAD / DELETE pair
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8

                    ClippedButton {
                        Layout.fillWidth: true
                        cut: 0
                        text: "LOAD"
                        bold: false
                        fontSize: 12
                        fillColor: Theme.surfaceElevated
                        hoverFillColor: Theme.surfaceElevated
                        borderColor: Theme.border
                        hoverBorderColor: Theme.accentMuted
                        textColor: Theme.textPrimary
                        hoverTextColor: Theme.textPrimary
                        enabled: presetList.currentIndex >= 0
                        onClicked: {
                            const name = presetManager.presets[presetList.currentIndex]
                            if (!name) return
                            pidPanel.loadValues(presetManager.load(name))
                            pidPanel.loadedPresetName = name
                        }
                    }
                    ClippedButton {
                        Layout.fillWidth: true
                        cut: 0
                        text: "DELETE"
                        bold: false
                        fontSize: 12
                        fillColor: Theme.surfaceElevated
                        hoverFillColor: Theme.surfaceElevated
                        borderColor: Theme.border
                        hoverBorderColor: Theme.accent
                        textColor: Theme.textPrimary
                        hoverTextColor: Theme.accent
                        enabled: presetList.currentIndex >= 0
                        onClicked: {
                            const name = presetManager.presets[presetList.currentIndex]
                            if (!name) return
                            deletePopup.targetName = name
                            deletePopup.open()
                        }
                    }
                }

                // ── SAVED TO block ─────────────────────────────────────
                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: 1
                    color: Theme.divider
                    Layout.topMargin: 4
                }
                Text {
                    text: "SAVED TO"
                    color: Theme.textTertiary
                    font.family: Theme.fontFamily
                    font.pixelSize: 9
                    font.letterSpacing: 2
                }
                Text {
                    Layout.fillWidth: true
                    text: presetManager.storagePath
                    color: Theme.textSecondary
                    font.family: Theme.monoFamily
                    font.pixelSize: 10
                    lineHeight: 1.5
                    wrapMode: Text.WrapAnywhere
                    ToolTip.visible: mouseArea.containsMouse
                    ToolTip.text: presetManager.storagePath
                    MouseArea {
                        id: mouseArea
                        anchors.fill: parent
                        hoverEnabled: true
                    }
                }
            }
        }

        // ── Controller Commands panel (制御指令) ────────────────────────
        Panel_PID_Controller {
            id: pidPanel
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }
}

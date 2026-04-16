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
            color: Theme.surface
            border.color: Theme.border
            border.width: Theme.strokePanel
            radius: Theme.radiusPanel
        }

        contentItem: ColumnLayout {
            spacing: 12

            Text {
                Layout.fillWidth: true
                Layout.margins: 16
                Layout.bottomMargin: 0
                text: "Save preset"
                color: Theme.accent
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontH2
                font.bold: true
            }

            Rectangle {
                Layout.fillWidth: true
                height: 1
                color: Theme.divider
            }

            ColumnLayout {
                Layout.fillWidth: true
                Layout.margins: 16
                Layout.topMargin: 4
                spacing: 6

                Text {
                    text: "Name this preset:"
                    color: Theme.textSecondary
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                }

                Basic.TextField {
                    id: saveNameField
                    Layout.fillWidth: true
                    Layout.preferredHeight: 36
                    placeholderText: "e.g. hover-v1"
                    color: Theme.textPrimary
                    leftPadding: 10
                    rightPadding: 10
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontBody
                    selectByMouse: true
                    background: Rectangle {
                        radius: Theme.radiusCard
                        color: Theme.background
                        border.width: Theme.strokeControl
                        border.color: saveNameField.activeFocus ? Theme.accent : Theme.border
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
                    text: "Cancel"
                    padding: 10
                    hoverEnabled: true
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    background: Rectangle {
                        radius: Theme.radiusControl
                        color: cancelBtn.down    ? Theme.btnSecondaryPress
                             : cancelBtn.hovered ? Theme.btnSecondaryHover
                             :                     Theme.btnSecondaryBg
                        border.width: Theme.strokeControl
                        border.color: Theme.btnSecondaryBorder
                    }
                    contentItem: Text {
                        anchors.centerIn: parent
                        text: cancelBtn.text
                        color: Theme.btnSecondaryText
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
            color: Theme.surface
            border.color: Theme.border
            border.width: Theme.strokePanel
            radius: Theme.radiusPanel
        }

        contentItem: ColumnLayout {
            spacing: 12

            Text {
                Layout.fillWidth: true
                Layout.margins: 16
                Layout.bottomMargin: 0
                text: "Delete preset"
                color: Theme.danger
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontH2
                font.bold: true
            }

            Rectangle {
                Layout.fillWidth: true
                height: 1
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
                    text: "Keep"
                    padding: 10
                    hoverEnabled: true
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    background: Rectangle {
                        radius: Theme.radiusControl
                        color: keepBtn.down    ? Theme.btnSecondaryPress
                             : keepBtn.hovered ? Theme.btnSecondaryHover
                             :                   Theme.btnSecondaryBg
                        border.width: Theme.strokeControl
                        border.color: Theme.btnSecondaryBorder
                    }
                    contentItem: Text {
                        anchors.centerIn: parent
                        text: keepBtn.text
                        color: Theme.btnSecondaryText
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

    RowLayout {
        anchors.fill: parent
        anchors.margins: 6
        spacing: 8

        // ── Presets sidebar ─────────────────────────────────────────────
        Rectangle {
            Layout.preferredWidth: 240
            Layout.fillHeight: true
            color: Theme.surface
            border.color: Theme.border
            border.width: Theme.strokePanel
            radius: Theme.radiusPanel

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 12
                spacing: 8

                Text {
                    text: "Presets"
                    color: Theme.accent
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontH1
                    font.bold: true
                }

                Text {
                    Layout.fillWidth: true
                    text: "Save the current PID / Reference / Config values for later recall."
                    color: Theme.textTertiary
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontCaption
                    wrapMode: Text.WordWrap
                }

                PrimaryButton {
                    text: "Save current as…"
                    Layout.fillWidth: true
                    onClicked: savePopup.open()
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: Theme.surfaceInset
                    border.color: Theme.border
                    border.width: Theme.strokeControl
                    radius: Theme.radiusCard

                    ListView {
                        id: presetList
                        anchors.fill: parent
                        anchors.margins: 4
                        clip: true
                        model: presetManager.presets
                        spacing: 2
                        ScrollBar.vertical: ThemedScrollBar { }
                        currentIndex: -1

                        delegate: Rectangle {
                            required property int index
                            required property string modelData
                            width: ListView.view.width
                            height: 30
                            radius: Theme.radiusControl
                            color: presetList.currentIndex === index
                                ? Theme.accentSubtle
                                : (mouse.containsMouse ? Theme.surfaceElevated : "transparent")

                            Text {
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.leftMargin: 8
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

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 6

                    PrimaryButton {
                        text: "Load"
                        Layout.fillWidth: true
                        enabled: presetList.currentIndex >= 0
                        onClicked: {
                            const name = presetManager.presets[presetList.currentIndex]
                            if (!name) return
                            pidPanel.loadValues(presetManager.load(name))
                            pidPanel.loadedPresetName = name
                        }
                    }
                    PrimaryButton {
                        text: "Delete"
                        Layout.fillWidth: true
                        enabled: presetList.currentIndex >= 0
                        onClicked: {
                            const name = presetManager.presets[presetList.currentIndex]
                            if (!name) return
                            deletePopup.targetName = name
                            deletePopup.open()
                        }
                    }
                }

                // ── Storage path caption ──────────────────────────────
                Rectangle {
                    Layout.fillWidth: true
                    height: 1
                    color: Theme.divider
                    Layout.topMargin: 4
                }
                Text {
                    text: "Saved to"
                    color: Theme.textTertiary
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontCaption
                }
                Text {
                    Layout.fillWidth: true
                    text: presetManager.storagePath
                    color: Theme.textSecondary
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontCaption
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

        // ── Enlarged PID panel ─────────────────────────────────────────
        Panel_PID_Controller {
            id: pidPanel
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }
}

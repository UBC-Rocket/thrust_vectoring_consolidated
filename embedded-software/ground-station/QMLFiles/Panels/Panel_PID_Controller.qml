import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Basic as Basic
import QtQuick.Layouts
import "../Items"

BasePanel {
    id: pidPanel

    signal commandTriggered(int which, var code)

    property int displayPrecision: 2

    // TODO: check if channel 1 is valid for PID Value sending
    property int which: 1

    // Attitude Controller gains (Vec3 each)
    property double attKpX: 0.0;  property double attKpY: 0.0;  property double attKpZ: 0.0
    property double attKdX: 0.0;  property double attKdY: 0.0;  property double attKdZ: 0.0

    // Z Position Controller gains (scalars)
    property double zKp: 0.0;     property double zKi: 0.0;     property double zKd: 0.0
    property double zIntegralLimit: 0.0

    property var editBuffer: ({})

    function formatGain(value) {
        return Number(value).toFixed(displayPrecision)
    }

    function sanitizedNumber(value, fallback) {
        const parsed = Number(value)
        return isNaN(parsed) ? fallback : parsed
    }

    function syncEditBuffer() {
        editBuffer = {
            attKpX: attKpX, attKpY: attKpY, attKpZ: attKpZ,
            attKdX: attKdX, attKdY: attKdY, attKdZ: attKdZ,
            zKp: zKp, zKi: zKi, zKd: zKd,
            zIntegralLimit: zIntegralLimit
        }
    }

    function applyEdits() {
        attKpX = sanitizedNumber(editBuffer.attKpX, attKpX)
        attKpY = sanitizedNumber(editBuffer.attKpY, attKpY)
        attKpZ = sanitizedNumber(editBuffer.attKpZ, attKpZ)
        attKdX = sanitizedNumber(editBuffer.attKdX, attKdX)
        attKdY = sanitizedNumber(editBuffer.attKdY, attKdY)
        attKdZ = sanitizedNumber(editBuffer.attKdZ, attKdZ)
        zKp = sanitizedNumber(editBuffer.zKp, zKp)
        zKi = sanitizedNumber(editBuffer.zKi, zKi)
        zKd = sanitizedNumber(editBuffer.zKd, zKd)
        zIntegralLimit = sanitizedNumber(editBuffer.zIntegralLimit, zIntegralLimit)

        var pidValues = [attKpX, attKpY, attKpZ, attKdX, attKdY, attKdZ, zKp, zKi, zKd, zIntegralLimit]
        pidPanel.commandTriggered(which, pidValues)

        editorPopup.close()
    }

    // --- Reusable gain chip component ---
    component GainChip: Rectangle {
        property string label: ""
        property string value: ""
        Layout.fillWidth: true
        implicitHeight: 48
        radius: Theme.radiusCard
        color: Theme.background
        border.width: Theme.strokeControl
        border.color: Theme.border

        Column {
            anchors.centerIn: parent
            spacing: 2
            Text { text: label; color: Theme.textTertiary; font.family: Theme.fontFamily; font.pixelSize: 12; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
            Text { text: value; color: Theme.textPrimary; font.family: Theme.monoFamily; font.pixelSize: Theme.fontH2; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
        }
    }

    // --- Reusable text field for edit popup ---
    component GainField: Basic.TextField {
        property string label: ""
        Layout.fillWidth: true
        placeholderText: label
        color: Theme.textPrimary
        font.family: Theme.monoFamily
        font.pixelSize: Theme.fontBody
        inputMethodHints: Qt.ImhFormattedNumbersOnly
        validator: DoubleValidator { decimals: 4 }
        background: Rectangle {
            radius: Theme.radiusCard
            color: Theme.background
            border.width: Theme.strokeControl
            border.color: Theme.border
        }
    }

    BaseHeader {
        id: header
        headerText: "PID Controller"
    }

    Basic.Button {
        id: editButton
        text: "Edit PID Values"
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 12
        padding: 10
        font.family: Theme.fontFamily
        font.pixelSize: Theme.fontBody
        hoverEnabled: true
        background: Rectangle {
            radius: Theme.radiusControl
            color: editButton.down    ? Theme.btnPrimaryPress
                 : editButton.hovered ? Theme.btnPrimaryHover
                 :                      Theme.btnPrimaryBg
            border.width: Theme.strokeControl
            border.color: Theme.btnPrimaryBorder
        }
        contentItem: Text {
            anchors.centerIn: parent
            text: editButton.text
            color: Theme.btnPrimaryText
            font: editButton.font
        }
        onClicked: {
            syncEditBuffer()
            editorPopup.open()
        }
    }

    ColumnLayout {
        anchors {
            top: header.bottom
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            topMargin: 12
            leftMargin: 12
            rightMargin: 12
            bottomMargin: 12
        }
        spacing: 10

        // ── Attitude Controller Section ──
        Text {
            text: "Attitude Controller"
            color: "#60A5FA"
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontH2
            font.bold: true
        }

        // Kp row
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 72
            radius: Theme.radiusCard
            color: Theme.surfaceInset
            border.width: Theme.strokeControl
            border.color: Theme.border

            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 10

                Rectangle { width: 6; radius: 3; color: "#60A5FA"; Layout.fillHeight: true }

                Column {
                    Layout.preferredWidth: 90
                    spacing: 2
                    Text { text: "Kp"; color: Theme.textPrimary; font.family: Theme.fontFamily; font.pixelSize: Theme.fontH2; font.bold: true }
                    Text { text: "Proportional"; color: Theme.textTertiary; font.family: Theme.fontFamily; font.pixelSize: 12 }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    GainChip { label: "X"; value: formatGain(attKpX) }
                    GainChip { label: "Y"; value: formatGain(attKpY) }
                    GainChip { label: "Z"; value: formatGain(attKpZ) }
                }
            }
        }

        // Kd row
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 72
            radius: Theme.radiusCard
            color: Theme.surfaceInset
            border.width: Theme.strokeControl
            border.color: Theme.border

            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 10

                Rectangle { width: 6; radius: 3; color: "#60A5FA"; Layout.fillHeight: true }

                Column {
                    Layout.preferredWidth: 90
                    spacing: 2
                    Text { text: "Kd"; color: Theme.textPrimary; font.family: Theme.fontFamily; font.pixelSize: Theme.fontH2; font.bold: true }
                    Text { text: "Derivative"; color: Theme.textTertiary; font.family: Theme.fontFamily; font.pixelSize: 12 }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    GainChip { label: "X"; value: formatGain(attKdX) }
                    GainChip { label: "Y"; value: formatGain(attKdY) }
                    GainChip { label: "Z"; value: formatGain(attKdZ) }
                }
            }
        }

        // ── Z Position Controller Section ──
        Text {
            text: "Z Position Controller"
            color: "#34D399"
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontH2
            font.bold: true
            Layout.topMargin: 6
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 120
            radius: Theme.radiusCard
            color: Theme.surfaceInset
            border.width: Theme.strokeControl
            border.color: Theme.border

            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 10

                Rectangle { width: 6; radius: 3; color: "#34D399"; Layout.fillHeight: true }

                Column {
                    Layout.preferredWidth: 90
                    spacing: 2
                    Text { text: "Gains"; color: Theme.textPrimary; font.family: Theme.fontFamily; font.pixelSize: Theme.fontH2; font.bold: true }
                    Text { text: "Z Position"; color: Theme.textTertiary; font.family: Theme.fontFamily; font.pixelSize: 12 }
                }

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 8

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        GainChip { label: "Kp"; value: formatGain(zKp) }
                        GainChip { label: "Ki"; value: formatGain(zKi) }
                        GainChip { label: "Kd"; value: formatGain(zKd) }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        GainChip { label: "I Limit"; value: formatGain(zIntegralLimit) }
                    }
                }
            }
        }

        Item { Layout.fillHeight: true }
    }

    // ── Edit Popup ──
    Popup {
        id: editorPopup
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape
        anchors.centerIn: Overlay.overlay
        width: 640
        padding: 0

        background: Rectangle {
            color: Theme.surfaceInset
            radius: 12
            border.width: Theme.strokeControl
            border.color: Theme.border
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 18
            spacing: 12

            Text { text: "Edit PID Values"; color: Theme.textPrimary; font.family: Theme.fontFamily; font.pixelSize: 20; font.bold: true }

            // ── Attitude Controller ──
            Text { text: "Attitude Controller"; color: "#60A5FA"; font.family: Theme.fontFamily; font.pixelSize: Theme.fontH2; font.bold: true }

            Text { text: "Proportional (Kp)"; color: Theme.textTertiary; font.family: Theme.fontFamily; font.pixelSize: 12 }
            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                GainField { label: "X"; text: editBuffer.attKpX !== undefined ? editBuffer.attKpX : ""; onTextChanged: editBuffer.attKpX = text }
                GainField { label: "Y"; text: editBuffer.attKpY !== undefined ? editBuffer.attKpY : ""; onTextChanged: editBuffer.attKpY = text }
                GainField { label: "Z"; text: editBuffer.attKpZ !== undefined ? editBuffer.attKpZ : ""; onTextChanged: editBuffer.attKpZ = text }
            }

            Text { text: "Derivative (Kd)"; color: Theme.textTertiary; font.family: Theme.fontFamily; font.pixelSize: 12 }
            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                GainField { label: "X"; text: editBuffer.attKdX !== undefined ? editBuffer.attKdX : ""; onTextChanged: editBuffer.attKdX = text }
                GainField { label: "Y"; text: editBuffer.attKdY !== undefined ? editBuffer.attKdY : ""; onTextChanged: editBuffer.attKdY = text }
                GainField { label: "Z"; text: editBuffer.attKdZ !== undefined ? editBuffer.attKdZ : ""; onTextChanged: editBuffer.attKdZ = text }
            }

            // ── Divider ──
            Rectangle { Layout.fillWidth: true; height: 1; color: Theme.border }

            // ── Z Position Controller ──
            Text { text: "Z Position Controller"; color: "#34D399"; font.family: Theme.fontFamily; font.pixelSize: Theme.fontH2; font.bold: true }

            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                GainField { label: "Kp"; text: editBuffer.zKp !== undefined ? editBuffer.zKp : ""; onTextChanged: editBuffer.zKp = text }
                GainField { label: "Ki"; text: editBuffer.zKi !== undefined ? editBuffer.zKi : ""; onTextChanged: editBuffer.zKi = text }
                GainField { label: "Kd"; text: editBuffer.zKd !== undefined ? editBuffer.zKd : ""; onTextChanged: editBuffer.zKd = text }
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                GainField { label: "Integral Limit"; text: editBuffer.zIntegralLimit !== undefined ? editBuffer.zIntegralLimit : ""; onTextChanged: editBuffer.zIntegralLimit = text }
            }

            // ── Buttons ──
            RowLayout {
                Layout.fillWidth: true
                spacing: 10

                Item { Layout.fillWidth: true }

                Basic.Button {
                    id: cancelButton
                    text: "Cancel"
                    padding: 10
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    background: Rectangle {
                        radius: Theme.radiusControl
                        color: Theme.btnSecondaryBg
                        border.width: Theme.strokeControl
                        border.color: Theme.btnSecondaryBorder
                    }
                    contentItem: Text {
                        anchors.centerIn: parent
                        text: cancelButton.text
                        color: Theme.btnSecondaryText
                        font: cancelButton.font
                    }
                    onClicked: editorPopup.close()
                }

                Basic.Button {
                    id: saveButton
                    text: "Save Changes"
                    padding: 10
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    background: Rectangle {
                        radius: Theme.radiusControl
                        color: saveButton.down    ? Theme.btnPrimaryPress
                             : saveButton.hovered ? Theme.btnPrimaryHover
                             :                      Theme.btnPrimaryBg
                        border.width: Theme.strokeControl
                        border.color: Theme.btnPrimaryBorder
                    }
                    contentItem: Text {
                        anchors.centerIn: parent
                        text: saveButton.text
                        color: Theme.btnPrimaryText
                        font: saveButton.font
                    }
                    onClicked: applyEdits()
                }
            }
        }
    }

    // --- Signal wiring: forward PID values to the C++ CommandSender ---
    Connections {
        target: pidPanel
        function onCommandTriggered(txWhich, code) {
            commandsender.sendPIDValues(txWhich, code)
        }
    }
}

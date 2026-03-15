import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Basic as Basic
import QtQuick.Layouts
import "../Items"

BasePanel {
    id: panel

    // TODO: check if channel 1 is valid for command sending.
    property int which: 1

    // SetPidGains
    property bool hasAttitudeKp: true
    property bool hasAttitudeKd: true
    property double attKpX: 0.0
    property double attKpY: 0.0
    property double attKpZ: 0.0
    property double attKdX: 0.0
    property double attKdY: 0.0
    property double attKdZ: 0.0
    property double zKp: 0.0
    property double zKi: 0.0
    property double zKd: 0.0
    property double zIntegralLimit: 0.0

    // SetReference
    property double zRef: 0.0
    property double vzRef: 0.0
    property bool hasQRef: true
    property double qRefW: 1.0
    property double qRefX: 0.0
    property double qRefY: 0.0
    property double qRefZ: 0.0

    // SetConfig
    property double mass: 0.0
    property double tMin: 0.0
    property double tMax: 0.0
    property double thetaMin: 0.0
    property double thetaMax: 0.0

    function sanitizedNumber(value, fallback) {
        const parsed = Number(value)
        return isNaN(parsed) ? fallback : parsed
    }

    component NumberField: ColumnLayout {
        property string label: ""
        property alias text: input.text
        Layout.fillWidth: true
        spacing: 4

        Text {
            text: parent.label
            color: Theme.textTertiary
            font.family: Theme.fontFamily
            font.pixelSize: 12
        }

        Basic.TextField {
            id: input
            Layout.fillWidth: true
            color: Theme.textPrimary
            font.family: Theme.monoFamily
            font.pixelSize: Theme.fontBody
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            validator: DoubleValidator { decimals: 6 }
            background: Rectangle {
                radius: Theme.radiusCard
                color: Theme.background
                border.width: Theme.strokeControl
                border.color: Theme.border
            }
        }
    }

    BaseHeader {
        id: header
        headerText: "Controller Commands"
    }

    Text {
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 12
        text: "TX channel: " + which
        color: Theme.textTertiary
        font.family: Theme.fontFamily
        font.pixelSize: 12
    }

    TabBar {
        id: tabs
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.margins: 12
        spacing: 8

        background: Rectangle {
            radius: Theme.radiusControl
            color: Theme.surfaceInset
            border.width: Theme.strokeControl
            border.color: Theme.border
        }

        TabButton {
            id: pidTab
            text: "PID"
            width: (tabs.width - (tabs.spacing * 2)) / 3
            hoverEnabled: true
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontBody
            background: Rectangle {
                radius: Theme.radiusControl
                color: pidTab.checked ? Theme.btnPrimaryBg
                     : pidTab.down ? Theme.btnSecondaryPress
                     : pidTab.hovered ? Theme.btnSecondaryHover
                     : Theme.btnSecondaryBg
                border.width: Theme.strokeControl
                border.color: pidTab.checked ? Theme.btnPrimaryBorder : Theme.btnSecondaryBorder
            }
            contentItem: Text {
                anchors.centerIn: parent
                text: pidTab.text
                color: pidTab.checked ? Theme.btnPrimaryText : Theme.btnSecondaryText
                font: pidTab.font
            }
        }
        TabButton {
            id: referenceTab
            text: "Reference"
            width: (tabs.width - (tabs.spacing * 2)) / 3
            hoverEnabled: true
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontBody
            background: Rectangle {
                radius: Theme.radiusControl
                color: referenceTab.checked ? Theme.btnPrimaryBg
                     : referenceTab.down ? Theme.btnSecondaryPress
                     : referenceTab.hovered ? Theme.btnSecondaryHover
                     : Theme.btnSecondaryBg
                border.width: Theme.strokeControl
                border.color: referenceTab.checked ? Theme.btnPrimaryBorder : Theme.btnSecondaryBorder
            }
            contentItem: Text {
                anchors.centerIn: parent
                text: referenceTab.text
                color: referenceTab.checked ? Theme.btnPrimaryText : Theme.btnSecondaryText
                font: referenceTab.font
            }
        }
        TabButton {
            id: configTab
            text: "Config"
            width: (tabs.width - (tabs.spacing * 2)) / 3
            hoverEnabled: true
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontBody
            background: Rectangle {
                radius: Theme.radiusControl
                color: configTab.checked ? Theme.btnPrimaryBg
                     : configTab.down ? Theme.btnSecondaryPress
                     : configTab.hovered ? Theme.btnSecondaryHover
                     : Theme.btnSecondaryBg
                border.width: Theme.strokeControl
                border.color: configTab.checked ? Theme.btnPrimaryBorder : Theme.btnSecondaryBorder
            }
            contentItem: Text {
                anchors.centerIn: parent
                text: configTab.text
                color: configTab.checked ? Theme.btnPrimaryText : Theme.btnSecondaryText
                font: configTab.font
            }
        }
    }

    StackLayout {
        anchors.top: tabs.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 12
        currentIndex: tabs.currentIndex

        // PID tab (tvr_SetPidGains)
        ScrollView {
            id: pidScroll
            clip: true
            contentWidth: availableWidth

            ColumnLayout {
                width: pidScroll.availableWidth
                spacing: 12

                CheckBox {
                    text: "Proportional"
                    checked: hasAttitudeKp
                    onToggled: hasAttitudeKp = checked
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "x"; text: String(attKpX); onTextChanged: attKpX = panel.sanitizedNumber(text, attKpX) }
                    NumberField { label: "y"; text: String(attKpY); onTextChanged: attKpY = panel.sanitizedNumber(text, attKpY) }
                    NumberField { label: "z"; text: String(attKpZ); onTextChanged: attKpZ = panel.sanitizedNumber(text, attKpZ) }
                }

                CheckBox {
                    text: "Derivative"
                    checked: hasAttitudeKd
                    onToggled: hasAttitudeKd = checked
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "x"; text: String(attKdX); onTextChanged: attKdX = panel.sanitizedNumber(text, attKdX) }
                    NumberField { label: "y"; text: String(attKdY); onTextChanged: attKdY = panel.sanitizedNumber(text, attKdY) }
                    NumberField { label: "z"; text: String(attKdZ); onTextChanged: attKdZ = panel.sanitizedNumber(text, attKdZ) }
                }

                Text {
                    text: "z"
                    color: Theme.textTertiary
                    font.family: Theme.fontFamily
                    font.pixelSize: 12
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "proportional"; text: String(zKp); onTextChanged: zKp = panel.sanitizedNumber(text, zKp) }
                    NumberField { label: "integral"; text: String(zKi); onTextChanged: zKi = panel.sanitizedNumber(text, zKi) }
                    NumberField { label: "derivative"; text: String(zKd); onTextChanged: zKd = panel.sanitizedNumber(text, zKd) }
                }

                NumberField {
                    label: "integral limit"
                    text: String(zIntegralLimit)
                    onTextChanged: zIntegralLimit = panel.sanitizedNumber(text, zIntegralLimit)
                }

                Basic.Button {
                    id: sendPidButton
                    text: "Send PID"
                    Layout.alignment: Qt.AlignRight
                    padding: 10
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    background: Rectangle {
                        radius: Theme.radiusControl
                        color: sendPidButton.down ? Theme.btnPrimaryPress
                             : sendPidButton.hovered ? Theme.btnPrimaryHover
                             : Theme.btnPrimaryBg
                        border.width: Theme.strokeControl
                        border.color: Theme.btnPrimaryBorder
                    }
                    contentItem: Text {
                        anchors.centerIn: parent
                        text: sendPidButton.text
                        color: Theme.btnPrimaryText
                        font: sendPidButton.font
                    }
                    onClicked: {
                        const values = [
                            hasAttitudeKp,
                            attKpX, attKpY, attKpZ,
                            hasAttitudeKd,
                            attKdX, attKdY, attKdZ,
                            zKp, zKi, zKd, zIntegralLimit
                        ]
                        commandsender.sendPIDValues(which, values)
                    }
                }
            }
        }

        // Reference tab (tvr_SetReference)
        ScrollView {
            id: referenceScroll
            clip: true
            contentWidth: availableWidth

            ColumnLayout {
                width: referenceScroll.availableWidth
                spacing: 12

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "z ref"; text: String(zRef); onTextChanged: zRef = panel.sanitizedNumber(text, zRef) }
                    NumberField { label: "vz ref"; text: String(vzRef); onTextChanged: vzRef = panel.sanitizedNumber(text, vzRef) }
                }

                CheckBox {
                    text: "has_q_ref"
                    checked: hasQRef
                    onToggled: hasQRef = checked
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "q ref w"; text: String(qRefW); onTextChanged: qRefW = panel.sanitizedNumber(text, qRefW) }
                    NumberField { label: "q ref x"; text: String(qRefX); onTextChanged: qRefX = panel.sanitizedNumber(text, qRefX) }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "q ref y"; text: String(qRefY); onTextChanged: qRefY = panel.sanitizedNumber(text, qRefY) }
                    NumberField { label: "q ref z"; text: String(qRefZ); onTextChanged: qRefZ = panel.sanitizedNumber(text, qRefZ) }
                }

                Basic.Button {
                    id: sendReferenceButton
                    text: "Send Reference"
                    Layout.alignment: Qt.AlignRight
                    padding: 10
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    background: Rectangle {
                        radius: Theme.radiusControl
                        color: sendReferenceButton.down ? Theme.btnPrimaryPress
                             : sendReferenceButton.hovered ? Theme.btnPrimaryHover
                             : Theme.btnPrimaryBg
                        border.width: Theme.strokeControl
                        border.color: Theme.btnPrimaryBorder
                    }
                    contentItem: Text {
                        anchors.centerIn: parent
                        text: sendReferenceButton.text
                        color: Theme.btnPrimaryText
                        font: sendReferenceButton.font
                    }
                    onClicked: {
                        const values = [zRef, vzRef, hasQRef, qRefW, qRefX, qRefY, qRefZ]
                        commandsender.sendReferenceValues(which, values)
                    }
                }
            }
        }

        // Config tab (tvr_SetConfig)
        ScrollView {
            id: configScroll
            clip: true
            contentWidth: availableWidth

            ColumnLayout {
                width: configScroll.availableWidth
                spacing: 12

                NumberField { label: "mass"; text: String(mass); onTextChanged: mass = panel.sanitizedNumber(text, mass) }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "T_min"; text: String(tMin); onTextChanged: tMin = panel.sanitizedNumber(text, tMin) }
                    NumberField { label: "T_max"; text: String(tMax); onTextChanged: tMax = panel.sanitizedNumber(text, tMax) }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "theta_min"; text: String(thetaMin); onTextChanged: thetaMin = panel.sanitizedNumber(text, thetaMin) }
                    NumberField { label: "theta_max"; text: String(thetaMax); onTextChanged: thetaMax = panel.sanitizedNumber(text, thetaMax) }
                }

                Basic.Button {
                    id: sendConfigButton
                    text: "Send Config"
                    Layout.alignment: Qt.AlignRight
                    padding: 10
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontBody
                    background: Rectangle {
                        radius: Theme.radiusControl
                        color: sendConfigButton.down ? Theme.btnPrimaryPress
                             : sendConfigButton.hovered ? Theme.btnPrimaryHover
                             : Theme.btnPrimaryBg
                        border.width: Theme.strokeControl
                        border.color: Theme.btnPrimaryBorder
                    }
                    contentItem: Text {
                        anchors.centerIn: parent
                        text: sendConfigButton.text
                        color: Theme.btnPrimaryText
                        font: sendConfigButton.font
                    }
                    onClicked: {
                        const values = [mass, tMin, tMax, thetaMin, thetaMax]
                        commandsender.sendConfigValues(which, values)
                    }
                }
            }
        }
    }
}

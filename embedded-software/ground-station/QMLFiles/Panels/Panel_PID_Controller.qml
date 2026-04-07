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
        Layout.minimumWidth: 0
        Layout.preferredWidth: 0
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
            Layout.minimumWidth: 0
            Layout.preferredWidth: 0
            leftPadding: 8
            rightPadding: 8
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

    component SectionLabel: Text {
        Layout.fillWidth: true
        Layout.topMargin: 4
        color: Theme.textTertiary
        font.family: Theme.fontFamily
        font.pixelSize: Theme.fontCaption
        font.bold: true
    }

    BaseHeader {
        id: header
        headerText: "Controller Commands"
    }

    // TX channel indicator — aligned with the header bar, not floating
    Text {
        anchors.verticalCenter: header.verticalCenter
        anchors.right: parent.right
        anchors.rightMargin: 15
        text: "TX channel: " + which
        color: Theme.textTertiary
        font.family: Theme.fontFamily
        font.pixelSize: Theme.fontCaption
    }

    Basic.TabBar {
        id: tabs
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.margins: 12
        spacing: 8

        background: Rectangle {
            color: "transparent"
            border.width: 0
        }

        ThemedTabButton {
            text: "PID"
            width: (tabs.width - (tabs.spacing * 2)) / 3
        }
        ThemedTabButton {
            text: "Reference"
            width: (tabs.width - (tabs.spacing * 2)) / 3
        }
        ThemedTabButton {
            text: "Config"
            width: (tabs.width - (tabs.spacing * 2)) / 3
        }
    }

    StackLayout {
        anchors.top: tabs.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 12
        currentIndex: tabs.currentIndex

        // ── PID tab (tvr_SetPidGains) ──
        ScrollView {
            id: pidScroll
            clip: true
            contentWidth: availableWidth
            ScrollBar.vertical: ThemedScrollBar { }
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

            ColumnLayout {
                width: pidScroll.availableWidth
                spacing: 10

                // Attitude proportional
                RowLayout {
                    Layout.fillWidth: true
                    SectionLabel { text: "ATTITUDE — PROPORTIONAL" }
                    ThemedCheckBox {
                        text: "enable"
                        checked: hasAttitudeKp
                        onToggled: hasAttitudeKp = checked
                    }
                }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    enabled: hasAttitudeKp
                    NumberField { label: "x"; text: String(attKpX); onTextChanged: attKpX = panel.sanitizedNumber(text, attKpX) }
                    NumberField { label: "y"; text: String(attKpY); onTextChanged: attKpY = panel.sanitizedNumber(text, attKpY) }
                    NumberField { label: "z"; text: String(attKpZ); onTextChanged: attKpZ = panel.sanitizedNumber(text, attKpZ) }
                }

                // Attitude derivative
                RowLayout {
                    Layout.fillWidth: true
                    SectionLabel { text: "ATTITUDE — DERIVATIVE" }
                    ThemedCheckBox {
                        text: "enable"
                        checked: hasAttitudeKd
                        onToggled: hasAttitudeKd = checked
                    }
                }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    enabled: hasAttitudeKd
                    NumberField { label: "x"; text: String(attKdX); onTextChanged: attKdX = panel.sanitizedNumber(text, attKdX) }
                    NumberField { label: "y"; text: String(attKdY); onTextChanged: attKdY = panel.sanitizedNumber(text, attKdY) }
                    NumberField { label: "z"; text: String(attKdZ); onTextChanged: attKdZ = panel.sanitizedNumber(text, attKdZ) }
                }

                // Z axis
                SectionLabel { text: "Z AXIS" }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "proportional"; text: String(zKp); onTextChanged: zKp = panel.sanitizedNumber(text, zKp) }
                    NumberField { label: "integral";     text: String(zKi); onTextChanged: zKi = panel.sanitizedNumber(text, zKi) }
                    NumberField { label: "derivative";   text: String(zKd); onTextChanged: zKd = panel.sanitizedNumber(text, zKd) }
                }
                NumberField {
                    label: "integral limit"
                    text: String(zIntegralLimit)
                    onTextChanged: zIntegralLimit = panel.sanitizedNumber(text, zIntegralLimit)
                }

                PrimaryButton {
                    text: "Send PID"
                    Layout.alignment: Qt.AlignRight
                    Layout.topMargin: 6
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

        // ── Reference tab (tvr_SetReference) ──
        ScrollView {
            id: referenceScroll
            clip: true
            contentWidth: availableWidth
            ScrollBar.vertical: ThemedScrollBar { }
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

            ColumnLayout {
                width: referenceScroll.availableWidth
                spacing: 10

                SectionLabel { text: "TRANSLATION" }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "z ref";  text: String(zRef);  onTextChanged: zRef  = panel.sanitizedNumber(text, zRef) }
                    NumberField { label: "vz ref"; text: String(vzRef); onTextChanged: vzRef = panel.sanitizedNumber(text, vzRef) }
                }

                RowLayout {
                    Layout.fillWidth: true
                    SectionLabel { text: "QUATERNION" }
                    ThemedCheckBox {
                        text: "enable"
                        checked: hasQRef
                        onToggled: hasQRef = checked
                    }
                }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    enabled: hasQRef
                    NumberField { label: "q ref w"; text: String(qRefW); onTextChanged: qRefW = panel.sanitizedNumber(text, qRefW) }
                    NumberField { label: "q ref x"; text: String(qRefX); onTextChanged: qRefX = panel.sanitizedNumber(text, qRefX) }
                }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    enabled: hasQRef
                    NumberField { label: "q ref y"; text: String(qRefY); onTextChanged: qRefY = panel.sanitizedNumber(text, qRefY) }
                    NumberField { label: "q ref z"; text: String(qRefZ); onTextChanged: qRefZ = panel.sanitizedNumber(text, qRefZ) }
                }

                PrimaryButton {
                    text: "Send Reference"
                    Layout.alignment: Qt.AlignRight
                    Layout.topMargin: 6
                    onClicked: {
                        const values = [zRef, vzRef, hasQRef, qRefW, qRefX, qRefY, qRefZ]
                        commandsender.sendReferenceValues(which, values)
                    }
                }
            }
        }

        // ── Config tab (tvr_SetConfig) ──
        ScrollView {
            id: configScroll
            clip: true
            contentWidth: availableWidth
            ScrollBar.vertical: ThemedScrollBar { }
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

            ColumnLayout {
                width: configScroll.availableWidth
                spacing: 10

                SectionLabel { text: "VEHICLE" }
                NumberField { label: "mass"; text: String(mass); onTextChanged: mass = panel.sanitizedNumber(text, mass) }

                SectionLabel { text: "THRUST LIMITS" }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "T_min"; text: String(tMin); onTextChanged: tMin = panel.sanitizedNumber(text, tMin) }
                    NumberField { label: "T_max"; text: String(tMax); onTextChanged: tMax = panel.sanitizedNumber(text, tMax) }
                }

                SectionLabel { text: "GIMBAL LIMITS" }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    NumberField { label: "theta_min"; text: String(thetaMin); onTextChanged: thetaMin = panel.sanitizedNumber(text, thetaMin) }
                    NumberField { label: "theta_max"; text: String(thetaMax); onTextChanged: thetaMax = panel.sanitizedNumber(text, thetaMax) }
                }

                PrimaryButton {
                    text: "Send Config"
                    Layout.alignment: Qt.AlignRight
                    Layout.topMargin: 6
                    onClicked: {
                        const values = [mass, tMin, tMax, thetaMin, thetaMax]
                        commandsender.sendConfigValues(which, values)
                    }
                }
            }
        }
    }
}

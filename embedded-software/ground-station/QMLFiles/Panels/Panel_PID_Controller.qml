import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Basic as Basic
import QtQuick.Layouts
import "../Items"

BasePanel {
    id: panel

    // TODO: check if channel 1 is valid for command sending.
    property int which: 1

    // Preset status — shown in the strip below the header.
    property string loadedPresetName: ""
    property bool   dirty: false

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

    // Snapshot every editable field for preset save.
    function currentValues() {
        return {
            hasAttitudeKp: hasAttitudeKp, hasAttitudeKd: hasAttitudeKd,
            attKpX: attKpX, attKpY: attKpY, attKpZ: attKpZ,
            attKdX: attKdX, attKdY: attKdY, attKdZ: attKdZ,
            zKp: zKp, zKi: zKi, zKd: zKd, zIntegralLimit: zIntegralLimit,
            zRef: zRef, vzRef: vzRef,
            hasQRef: hasQRef, qRefW: qRefW, qRefX: qRefX, qRefY: qRefY, qRefZ: qRefZ,
            mass: mass, tMin: tMin, tMax: tMax, thetaMin: thetaMin, thetaMax: thetaMax
        }
    }

    // Apply a preset map; missing keys are left untouched. Resets the dirty flag.
    function loadValues(v) {
        if (!v) return
        if (v.hasAttitudeKp !== undefined) hasAttitudeKp = v.hasAttitudeKp
        if (v.hasAttitudeKd !== undefined) hasAttitudeKd = v.hasAttitudeKd
        if (v.attKpX !== undefined) attKpX = v.attKpX
        if (v.attKpY !== undefined) attKpY = v.attKpY
        if (v.attKpZ !== undefined) attKpZ = v.attKpZ
        if (v.attKdX !== undefined) attKdX = v.attKdX
        if (v.attKdY !== undefined) attKdY = v.attKdY
        if (v.attKdZ !== undefined) attKdZ = v.attKdZ
        if (v.zKp !== undefined) zKp = v.zKp
        if (v.zKi !== undefined) zKi = v.zKi
        if (v.zKd !== undefined) zKd = v.zKd
        if (v.zIntegralLimit !== undefined) zIntegralLimit = v.zIntegralLimit
        if (v.zRef !== undefined) zRef = v.zRef
        if (v.vzRef !== undefined) vzRef = v.vzRef
        if (v.hasQRef !== undefined) hasQRef = v.hasQRef
        if (v.qRefW !== undefined) qRefW = v.qRefW
        if (v.qRefX !== undefined) qRefX = v.qRefX
        if (v.qRefY !== undefined) qRefY = v.qRefY
        if (v.qRefZ !== undefined) qRefZ = v.qRefZ
        if (v.mass !== undefined) mass = v.mass
        if (v.tMin !== undefined) tMin = v.tMin
        if (v.tMax !== undefined) tMax = v.tMax
        if (v.thetaMin !== undefined) thetaMin = v.thetaMin
        if (v.thetaMax !== undefined) thetaMax = v.thetaMax
        dirty = false
    }

    // Wrapper for NumberField onValueEdited so each write also flips dirty.
    function _edit(writer, v) {
        writer(v)
        dirty = true
    }

    component SectionHeader: ColumnLayout {
        property string title: ""
        Layout.fillWidth: true
        Layout.topMargin: 6
        spacing: 2
        Text {
            text: parent.title
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
    }

    component SubLabel: Text {
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

    Text {
        anchors.verticalCenter: header.verticalCenter
        anchors.right: parent.right
        anchors.rightMargin: 15
        text: "TX channel: " + panel.which
        color: Theme.textTertiary
        font.family: Theme.fontFamily
        font.pixelSize: Theme.fontCaption
    }

    // ── Preset status strip ─────────────────────────────────────────────
    RowLayout {
        id: presetStrip
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.leftMargin: 15
        anchors.rightMargin: 15
        anchors.topMargin: 4
        spacing: 8

        Text {
            text: "Preset:"
            color: Theme.textTertiary
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontCaption
        }
        Text {
            text: panel.loadedPresetName.length > 0 ? panel.loadedPresetName : "(unsaved)"
            color: panel.loadedPresetName.length > 0 ? Theme.accent : Theme.textTertiary
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontCaption
            font.bold: panel.loadedPresetName.length > 0
            Layout.fillWidth: true
            elide: Text.ElideRight
        }
        Text {
            visible: panel.dirty
            text: "• modified"
            color: Theme.warn
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontCaption
            font.italic: true
        }
    }

    // ── Stacked sections (no tabs) ──────────────────────────────────────
    ScrollView {
        id: sectionScroll
        anchors.top: presetStrip.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.leftMargin: 15
        anchors.rightMargin: 15
        anchors.topMargin: 8
        anchors.bottomMargin: 12
        clip: true
        contentWidth: availableWidth
        ScrollBar.vertical: ThemedScrollBar { }
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

        ColumnLayout {
            width: sectionScroll.availableWidth
            spacing: 14

            // ── PID GAINS ─────────────────────────────────────────────
            SectionHeader { title: "PID Gains" }

            RowLayout {
                Layout.fillWidth: true
                SubLabel { text: "ATTITUDE — PROPORTIONAL" }
                ThemedCheckBox {
                    text: "enable"
                    checked: panel.hasAttitudeKp
                    onToggled: { panel.hasAttitudeKp = checked; panel.dirty = true }
                }
            }
            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                NumberField {
                    label: "x"; value: panel.attKpX; fieldEnabled: panel.hasAttitudeKp
                    onValueEdited: (v) => panel._edit((x) => panel.attKpX = x, v)
                }
                NumberField {
                    label: "y"; value: panel.attKpY; fieldEnabled: panel.hasAttitudeKp
                    onValueEdited: (v) => panel._edit((x) => panel.attKpY = x, v)
                }
                NumberField {
                    label: "z"; value: panel.attKpZ; fieldEnabled: panel.hasAttitudeKp
                    onValueEdited: (v) => panel._edit((x) => panel.attKpZ = x, v)
                }
            }

            RowLayout {
                Layout.fillWidth: true
                SubLabel { text: "ATTITUDE — DERIVATIVE" }
                ThemedCheckBox {
                    text: "enable"
                    checked: panel.hasAttitudeKd
                    onToggled: { panel.hasAttitudeKd = checked; panel.dirty = true }
                }
            }
            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                NumberField {
                    label: "x"; value: panel.attKdX; fieldEnabled: panel.hasAttitudeKd
                    onValueEdited: (v) => panel._edit((x) => panel.attKdX = x, v)
                }
                NumberField {
                    label: "y"; value: panel.attKdY; fieldEnabled: panel.hasAttitudeKd
                    onValueEdited: (v) => panel._edit((x) => panel.attKdY = x, v)
                }
                NumberField {
                    label: "z"; value: panel.attKdZ; fieldEnabled: panel.hasAttitudeKd
                    onValueEdited: (v) => panel._edit((x) => panel.attKdZ = x, v)
                }
            }

            SubLabel { text: "Z AXIS" }
            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                NumberField {
                    label: "proportional"; value: panel.zKp
                    onValueEdited: (v) => panel._edit((x) => panel.zKp = x, v)
                }
                NumberField {
                    label: "integral"; value: panel.zKi
                    onValueEdited: (v) => panel._edit((x) => panel.zKi = x, v)
                }
                NumberField {
                    label: "derivative"; value: panel.zKd
                    onValueEdited: (v) => panel._edit((x) => panel.zKd = x, v)
                }
            }
            NumberField {
                label: "integral limit"; value: panel.zIntegralLimit
                onValueEdited: (v) => panel._edit((x) => panel.zIntegralLimit = x, v)
            }

            PrimaryButton {
                text: "Send PID"
                Layout.alignment: Qt.AlignRight
                Layout.topMargin: 4
                onClicked: {
                    const values = [
                        panel.hasAttitudeKp,
                        panel.attKpX, panel.attKpY, panel.attKpZ,
                        panel.hasAttitudeKd,
                        panel.attKdX, panel.attKdY, panel.attKdZ,
                        panel.zKp, panel.zKi, panel.zKd, panel.zIntegralLimit
                    ]
                    commandsender.sendPIDValues(panel.which, values)
                }
            }

            // ── REFERENCE ─────────────────────────────────────────────
            SectionHeader { title: "Reference" }

            SubLabel { text: "TRANSLATION" }
            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                NumberField {
                    label: "z ref"; value: panel.zRef
                    onValueEdited: (v) => panel._edit((x) => panel.zRef = x, v)
                }
                NumberField {
                    label: "vz ref"; value: panel.vzRef
                    onValueEdited: (v) => panel._edit((x) => panel.vzRef = x, v)
                }
            }

            RowLayout {
                Layout.fillWidth: true
                SubLabel { text: "QUATERNION" }
                ThemedCheckBox {
                    text: "enable"
                    checked: panel.hasQRef
                    onToggled: { panel.hasQRef = checked; panel.dirty = true }
                }
            }
            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                NumberField {
                    label: "q ref w"; value: panel.qRefW; fieldEnabled: panel.hasQRef
                    onValueEdited: (v) => panel._edit((x) => panel.qRefW = x, v)
                }
                NumberField {
                    label: "q ref x"; value: panel.qRefX; fieldEnabled: panel.hasQRef
                    onValueEdited: (v) => panel._edit((x) => panel.qRefX = x, v)
                }
            }
            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                NumberField {
                    label: "q ref y"; value: panel.qRefY; fieldEnabled: panel.hasQRef
                    onValueEdited: (v) => panel._edit((x) => panel.qRefY = x, v)
                }
                NumberField {
                    label: "q ref z"; value: panel.qRefZ; fieldEnabled: panel.hasQRef
                    onValueEdited: (v) => panel._edit((x) => panel.qRefZ = x, v)
                }
            }

            PrimaryButton {
                text: "Send Reference"
                Layout.alignment: Qt.AlignRight
                Layout.topMargin: 4
                onClicked: {
                    const values = [panel.zRef, panel.vzRef, panel.hasQRef,
                                    panel.qRefW, panel.qRefX, panel.qRefY, panel.qRefZ]
                    commandsender.sendReferenceValues(panel.which, values)
                }
            }

            // ── CONFIG ────────────────────────────────────────────────
            SectionHeader { title: "Config" }

            SubLabel { text: "VEHICLE" }
            NumberField {
                label: "mass"; value: panel.mass
                onValueEdited: (v) => panel._edit((x) => panel.mass = x, v)
            }

            SubLabel { text: "THRUST LIMITS" }
            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                NumberField {
                    label: "T_min"; value: panel.tMin
                    onValueEdited: (v) => panel._edit((x) => panel.tMin = x, v)
                }
                NumberField {
                    label: "T_max"; value: panel.tMax
                    onValueEdited: (v) => panel._edit((x) => panel.tMax = x, v)
                }
            }

            SubLabel { text: "GIMBAL LIMITS" }
            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                NumberField {
                    label: "theta_min"; value: panel.thetaMin
                    onValueEdited: (v) => panel._edit((x) => panel.thetaMin = x, v)
                }
                NumberField {
                    label: "theta_max"; value: panel.thetaMax
                    onValueEdited: (v) => panel._edit((x) => panel.thetaMax = x, v)
                }
            }

            PrimaryButton {
                text: "Send Config"
                Layout.alignment: Qt.AlignRight
                Layout.topMargin: 4
                Layout.bottomMargin: 6
                onClicked: {
                    const values = [panel.mass, panel.tMin, panel.tMax, panel.thetaMin, panel.thetaMax]
                    commandsender.sendConfigValues(panel.which, values)
                }
            }
        }
    }
}

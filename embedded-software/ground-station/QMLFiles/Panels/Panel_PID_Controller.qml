import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../Items"

BasePanel {
    id: panel

    // Mirrors the operator's TX channel choice in the Diagnostics panel.
    // Bridge changes propagate via txToChanged.
    property int which: bridge.txTo

    // Preset status — shown in the header caption.
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

    // SetThrottle — live actuation value, deliberately NOT part of presets
    // (a loaded preset must never silently change a running engine).
    property double throttlePct: 0.0

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

    // Small subhead label above field groups: 10px ls2 dim orange.
    component SubLabel: Text {
        Layout.fillWidth: true
        Layout.topMargin: 4
        color: Theme.textSecondary
        font.family: Theme.fontFamily
        font.pixelSize: 10
        font.letterSpacing: 2
    }

    BaseHeader {
        id: header
        headerText: "Controller Commands"
        jpText: "制御指令"
        anchors.topMargin: Theme.paddingLg
        anchors.leftMargin: Theme.paddingLg
    }

    // Header caption: TX channel + preset (dirty) state, right-aligned.
    Row {
        anchors.top: header.top
        anchors.topMargin: 2
        anchors.right: parent.right
        anchors.rightMargin: Theme.paddingLg
        spacing: 8

        Text {
            text: "TX CHANNEL: " + panel.which + " · PRESET: "
                  + (panel.loadedPresetName.length > 0 ? panel.loadedPresetName : "(UNSAVED)")
            color: Theme.textSecondary
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontCaption
            font.letterSpacing: 2
            font.capitalization: Font.AllUppercase
        }
        Text {
            visible: panel.dirty
            text: "● MODIFIED"
            color: Theme.amber
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontCaption
            font.letterSpacing: 2
        }
    }

    // ── Stacked sections (no tabs) ──────────────────────────────────────
    ScrollView {
        id: sectionScroll
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.leftMargin: Theme.paddingLg
        anchors.rightMargin: Theme.paddingLg
        anchors.topMargin: 4
        anchors.bottomMargin: Theme.paddingLg
        clip: true
        contentWidth: availableWidth
        ScrollBar.vertical: ThemedScrollBar { }
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

        ColumnLayout {
            width: sectionScroll.availableWidth
            spacing: 8

            // ── PID GAINS ─────────────────────────────────────────────
            SectionChip {
                Layout.fillWidth: true
                text: "PID GAINS"
            }

            RowLayout {
                Layout.fillWidth: true
                Layout.topMargin: 4
                SubLabel { text: "ATTITUDE — PROPORTIONAL" }
                ThemedCheckBox {
                    text: "ENABLE"
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
                    text: "ENABLE"
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
                text: "Send PID ▸"
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
            SectionChip {
                Layout.fillWidth: true
                Layout.topMargin: 8
                text: "REFERENCE"
            }

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
                    text: "ENABLE"
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
                text: "Send Reference ▸"
                Layout.alignment: Qt.AlignRight
                Layout.topMargin: 4
                onClicked: {
                    const values = [panel.zRef, panel.vzRef, panel.hasQRef,
                                    panel.qRefW, panel.qRefX, panel.qRefY, panel.qRefZ]
                    commandsender.sendReferenceValues(panel.which, values)
                }
            }

            // ── CONFIG ────────────────────────────────────────────────
            SectionChip {
                Layout.fillWidth: true
                Layout.topMargin: 8
                text: "CONFIG"
            }

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
                text: "Send Config ▸"
                Layout.alignment: Qt.AlignRight
                Layout.topMargin: 4
                Layout.bottomMargin: 6
                onClicked: {
                    const values = [panel.mass, panel.tMin, panel.tMax, panel.thetaMin, panel.thetaMax]
                    commandsender.sendConfigValues(panel.which, values)
                }
            }

            // ── THROTTLE ──────────────────────────────────────────────
            SectionChip {
                Layout.fillWidth: true
                Layout.topMargin: 8
                text: "THROTTLE"
            }

            SubLabel { text: "MANUAL THROTTLE (%)" }
            NumberField {
                label: "throttle %"
                value: panel.throttlePct
                // Direct write, not _edit: throttle is excluded from presets,
                // so it must not flip the preset MODIFIED indicator.
                onValueEdited: (v) => panel.throttlePct = v
            }

            PrimaryButton {
                text: "Send Throttle ▸"
                Layout.alignment: Qt.AlignRight
                Layout.topMargin: 4
                Layout.bottomMargin: 6
                // Clamp at send so the field always shows what was typed while
                // the wire never carries more than 100 % (FC clamps again).
                onClicked: commandsender.sendThrottle(
                    panel.which,
                    Math.max(0, Math.min(100, panel.throttlePct)) / 100)
            }
        }
    }
}

import QtQuick
import QtQuick.Layouts
import "../Items"

BasePanel {
    id: panel_Angles_And_Engine

    // ===== Spacing controls =====
    property real sectionSpacing: 18                     // spacing between sections (X->Y->Z->Engine->Motors)
    property real headerToFirstSectionSpacing: 6         // main title -> "Angles" section
    property real subheaderToDataSpacing: 12             // subheader -> data boxes
    property real rowPadding: 8                          // extra padding for implicitHeight

    // Radio TX channel used for outgoing commands. Mirrors the operator's choice
    // in the Diagnostics panel (same pattern as Panel_PID_Controller).
    property int txWhich: bridge.txTo

    // Editable target RPM setpoints; preserved across user typing via NumberField.
    property double targetRpmUpper: 0.0
    property double targetRpmLower: 0.0

    // Angular rates (deg/s) — raw gyro output
    property double raw_angle_x: sensorData.rawAngleX
    property double raw_angle_y: sensorData.rawAngleY
    property double raw_angle_z: sensorData.rawAngleZ

    // Euler angles (deg) — from attitude quaternion
    property double filtered_angle_x: sensorData.filteredAngleX
    property double filtered_angle_y: sensorData.filteredAngleY
    property double filtered_angle_z: sensorData.filteredAngleZ

    // Engine outputs from telemetry
    property double thrustCmd: sensorData.thrustCmd
    property double gimbalX:   sensorData.gimbalX
    property double gimbalY:   sensorData.gimbalY

    // Motor RPM from bdshot telemetry
    property double motorRpmUpper: sensorData.motorRpmUpper
    property double motorRpmLower: sensorData.motorRpmLower

    BaseHeader {
        id: header
        headerText: "Angles and Engine"
    }

    Rectangle {
        id: kalman_angles_x
        color: "transparent"

        implicitHeight: subheader_angles.implicitHeight
                        + panel_Angles_And_Engine.subheaderToDataSpacing
                        + dataBoxListX.height
        height: implicitHeight

        anchors {
            top: header.bottom
            left: parent.left
            right: parent.right
            topMargin: panel_Angles_And_Engine.headerToFirstSectionSpacing
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        Text {
            id: subheader_angles
            text: "Attitude (deg) / Angular Rate (°/s)"
            font.family: Theme.fontFamily
            font.pixelSize: 18
            color: Theme.textSecondary
            y: 0
        }

        DataBoxList {
            id: dataBoxListX
            anchors.top: subheader_angles.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            width: parent.width

            size: 2
            boxHeight: 56
            dataNames: ["ANG RATE X (°/s)", "ROLL (°)"]
            dataValues: [raw_angle_x, filtered_angle_x]
        }
    }

    Rectangle {
        id: kalman_angles_y
        color: "transparent"
        implicitHeight: dataBoxListY.height + panel_Angles_And_Engine.rowPadding
        height: implicitHeight

        anchors {
            top: kalman_angles_x.bottom
            left: parent.left
            right: parent.right
            topMargin: panel_Angles_And_Engine.sectionSpacing
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        DataBoxList {
            id: dataBoxListY
            width: parent.width

            size: 2
            boxHeight: 56
            dataNames: ["ANG RATE Y (°/s)", "PITCH (°)"]
            dataValues: [raw_angle_y, filtered_angle_y]
        }
    }

    Rectangle {
        id: kalman_angles_z
        color: "transparent"
        implicitHeight: dataBoxListZ.height + panel_Angles_And_Engine.rowPadding
        height: implicitHeight

        anchors {
            top: kalman_angles_y.bottom
            left: parent.left
            right: parent.right
            topMargin: panel_Angles_And_Engine.sectionSpacing
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        DataBoxList {
            id: dataBoxListZ
            width: parent.width

            size: 2
            boxHeight: 56
            dataNames: ["ANG RATE Z (°/s)", "YAW (°)"]
            dataValues: [raw_angle_z, filtered_angle_z]
        }
    }

    Rectangle {
        id: engine
        color: "transparent"

        implicitHeight: subheader_engine.implicitHeight
                        + panel_Angles_And_Engine.subheaderToDataSpacing
                        + dataBoxListEngine.height
        height: implicitHeight

        anchors {
            top: kalman_angles_z.bottom
            left: parent.left
            right: parent.right
            topMargin: panel_Angles_And_Engine.sectionSpacing
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        Text {
            id: subheader_engine
            text: "Engine Control"
            font.family: Theme.fontFamily
            font.pixelSize: 18
            color: Theme.textSecondary
            y: 0
        }

        DataBoxList {
            id: dataBoxListEngine
            anchors.top: subheader_engine.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            width: parent.width

            size: 3
            boxHeight: 56
            dataNames: ["THRUST", "GIMBAL X", "GIMBAL Y"]
            dataValues: [thrustCmd, gimbalX, gimbalY]
        }
    }

    Rectangle {
        id: motors
        color: "transparent"

        implicitHeight: subheader_motors.implicitHeight
                        + panel_Angles_And_Engine.subheaderToDataSpacing
                        + dataBoxListMotors.height
                        + panel_Angles_And_Engine.sectionSpacing
                        + motorTargetRow.height
        height: implicitHeight

        anchors {
            top: engine.bottom
            left: parent.left
            right: parent.right
            topMargin: panel_Angles_And_Engine.sectionSpacing
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        Text {
            id: subheader_motors
            text: "Motor RPM (bdshot feedback)"
            font.family: Theme.fontFamily
            font.pixelSize: 18
            color: Theme.textSecondary
            y: 0
        }

        DataBoxList {
            id: dataBoxListMotors
            anchors.top: subheader_motors.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            width: parent.width

            size: 2
            boxHeight: 56
            dataNames: ["RPM UPPER", "RPM LOWER"]
            dataValues: [motorRpmUpper, motorRpmLower]
        }

        ColumnLayout {
            id: motorTargetRow
            anchors.top: dataBoxListMotors.bottom
            anchors.topMargin: panel_Angles_And_Engine.sectionSpacing
            width: parent.width
            spacing: 4

            Text {
                text: "TARGET RPM"
                color: Theme.textTertiary
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontCaption
                font.bold: true
                Layout.fillWidth: true
                Layout.topMargin: 4
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 10

                NumberField {
                    label: "upper"
                    value: panel_Angles_And_Engine.targetRpmUpper
                    onValueEdited: (v) => panel_Angles_And_Engine.targetRpmUpper = v
                }
                NumberField {
                    label: "lower"
                    value: panel_Angles_And_Engine.targetRpmLower
                    onValueEdited: (v) => panel_Angles_And_Engine.targetRpmLower = v
                }
            }

            PrimaryButton {
                text: "Send Motor Speed"
                Layout.alignment: Qt.AlignRight
                Layout.topMargin: 4
                onClicked: commandsender.sendMotorSpeed(
                    panel_Angles_And_Engine.txWhich,
                    panel_Angles_And_Engine.targetRpmUpper,
                    panel_Angles_And_Engine.targetRpmLower)
            }
        }
    }
}

import QtQuick
import "../Items"

BasePanel {
    id: panel_Angles_And_Engine

    // ===== Spacing controls =====
    property real sectionSpacing: 8                       // spacing between sections (X->Y->Z->Engine)
    property real headerToFirstSectionSpacing: 2          // main title -> "Angles" section
    property real subheaderToDataSpacing: 10              // subheader -> data boxes
    property real rowPadding: 0                           // extra padding for implicitHeight

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

    // Per-motor RPM (bidirectional-DShot readback); invalid → NaN → "—"
    property double motorRpmLower: sensorData.motorRpmLower
    property double motorRpmUpper: sensorData.motorRpmUpper
    property bool   motorRpmValid: sensorData.motorRpmValid

    BaseHeader {
        id: header
        headerText: "Angles / Engine"
        jpText: "姿勢制御"
    }

    Rectangle {
        id: kalman_angles_x
        color: "transparent"

        implicitHeight: subheader_angles.implicitHeight
                        + panel_Angles_And_Engine.subheaderToDataSpacing
                        + boxRateX.height
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
            text: "ATTITUDE (°) / ANGULAR RATE (°/S)"
            font.family: Theme.fontFamily
            font.pixelSize: 10
            font.letterSpacing: 2
            color: Theme.textSecondary
            y: 0
        }

        DataBox {
            id: boxRateX
            anchors.top: subheader_angles.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            sections: 2; section_num: 1
            dataName: "ANG RATE X"
            dataValue: raw_angle_x
        }

        DataBox {
            anchors.top: subheader_angles.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            sections: 2; section_num: 2
            dataName: "ROLL"
            dataValue: filtered_angle_x
            valueColor: Theme.amber
        }
    }

    Rectangle {
        id: kalman_angles_y
        color: "transparent"
        implicitHeight: boxRateY.height + panel_Angles_And_Engine.rowPadding
        height: implicitHeight

        anchors {
            top: kalman_angles_x.bottom
            left: parent.left
            right: parent.right
            topMargin: panel_Angles_And_Engine.sectionSpacing
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        DataBox {
            id: boxRateY
            anchors.top: parent.top
            sections: 2; section_num: 1
            dataName: "ANG RATE Y"
            dataValue: raw_angle_y
        }

        DataBox {
            anchors.top: parent.top
            sections: 2; section_num: 2
            dataName: "PITCH"
            dataValue: filtered_angle_y
            valueColor: Theme.amber
        }
    }

    Rectangle {
        id: kalman_angles_z
        color: "transparent"
        implicitHeight: boxRateZ.height + panel_Angles_And_Engine.rowPadding
        height: implicitHeight

        anchors {
            top: kalman_angles_y.bottom
            left: parent.left
            right: parent.right
            topMargin: panel_Angles_And_Engine.sectionSpacing
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        DataBox {
            id: boxRateZ
            anchors.top: parent.top
            sections: 2; section_num: 1
            dataName: "ANG RATE Z"
            dataValue: raw_angle_z
        }

        DataBox {
            anchors.top: parent.top
            sections: 2; section_num: 2
            dataName: "YAW"
            dataValue: filtered_angle_z
            valueColor: Theme.amber
        }
    }

    Rectangle {
        id: engine
        color: "transparent"

        implicitHeight: subheader_engine.implicitHeight
                        + panel_Angles_And_Engine.subheaderToDataSpacing
                        + boxThrust.height
        height: implicitHeight

        anchors {
            top: kalman_angles_z.bottom
            left: parent.left
            right: parent.right
            topMargin: panel_Angles_And_Engine.sectionSpacing + 4
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        Text {
            id: subheader_engine
            text: "ENGINE CONTROL"
            font.family: Theme.fontFamily
            font.pixelSize: 10
            font.letterSpacing: 2
            color: Theme.textSecondary
            y: 0
        }

        JpText {
            anchors.right: parent.right
            anchors.baseline: subheader_engine.baseline
            text: "推力偏向"
            color: Theme.textTertiary
        }

        DataBox {
            id: boxThrust
            anchors.top: subheader_engine.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            sections: 3; section_num: 1
            dataName: "THRUST"
            dataValue: thrustCmd
            valueColor: Theme.success
            accentColor: Theme.success
        }

        DataBox {
            anchors.top: subheader_engine.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            sections: 3; section_num: 2
            dataName: "GIMBAL X"
            dataValue: gimbalX
        }

        DataBox {
            anchors.top: subheader_engine.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            sections: 3; section_num: 3
            dataName: "GIMBAL Y"
            dataValue: gimbalY
        }
    }

    Rectangle {
        id: motor_rpm
        color: "transparent"

        implicitHeight: subheader_rpm.implicitHeight
                        + panel_Angles_And_Engine.subheaderToDataSpacing
                        + boxRpmLower.height
        height: implicitHeight

        anchors {
            top: engine.bottom
            left: parent.left
            right: parent.right
            topMargin: panel_Angles_And_Engine.sectionSpacing + 4
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        Text {
            id: subheader_rpm
            text: "MOTOR RPM"
            font.family: Theme.fontFamily
            font.pixelSize: 10
            font.letterSpacing: 2
            color: Theme.textSecondary
            y: 0
        }

        JpText {
            anchors.right: parent.right
            anchors.baseline: subheader_rpm.baseline
            text: "回転数"
            color: Theme.textTertiary
        }

        DataBox {
            id: boxRpmLower
            anchors.top: subheader_rpm.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            sections: 2; section_num: 1
            dataName: "RPM LOWER"
            dataValue: motorRpmValid ? motorRpmLower : NaN
        }

        DataBox {
            anchors.top: subheader_rpm.bottom
            anchors.topMargin: panel_Angles_And_Engine.subheaderToDataSpacing
            sections: 2; section_num: 2
            dataName: "RPM UPPER"
            dataValue: motorRpmValid ? motorRpmUpper : NaN
        }
    }
}

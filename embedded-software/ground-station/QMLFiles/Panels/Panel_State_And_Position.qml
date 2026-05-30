import QtQuick
import "../Items"

BasePanel {
    id: panel_State_And_Position

    // Position
    property double posX: sensorData.posX
    property double posY: sensorData.posY
    property double altitude: sensorData.altitude

    // Telemetry — velocity magnitude in m/s
    property double velocity: sensorData.velocity

    BaseHeader {
        id: header
        headerText: "State & Position Data"
    }

    Rectangle {
        id: position_xy
        color: "transparent"
        height: (parent.height - header.height) / 2
        anchors {
            top: parent.top
            left: parent.left
            topMargin: header.height
            leftMargin: header.anchors.leftMargin
        }

        DataBoxList {
            anchors.top: position_xy.top
            width: panel_State_And_Position.width

            size: 2
            boxHeight: 56
            dataNames: ["POS X (m)", "POS Y (m)"]
            dataValues: [posX, posY]
        }
    }

    Rectangle {
        id: altitude_and_velocity
        color: "transparent"
        height: (parent.height - header.height) / 2
        anchors {
            top: position_xy.bottom
            left: parent.left
            leftMargin: header.anchors.leftMargin
        }

        DataBoxList {
            anchors.top: altitude_and_velocity.top
            width: panel_State_And_Position.width

            size: 2
            boxHeight: 56
            dataNames: ["ALTITUDE (m)", "VELOCITY (m/s)"]
            dataValues: [altitude, velocity]
        }
    }
}

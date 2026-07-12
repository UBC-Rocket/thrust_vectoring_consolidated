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
        headerText: "State / Position"
        jpText: "位置情報"
    }

    Rectangle {
        id: position_xy
        color: "transparent"
        height: 56
        anchors {
            top: header.bottom
            left: parent.left
            right: parent.right
            topMargin: 2
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        DataBox {
            anchors.top: parent.top
            sections: 2; section_num: 1
            dataName: "POS X (m)"
            dataValue: posX
        }

        DataBox {
            anchors.top: parent.top
            sections: 2; section_num: 2
            dataName: "POS Y (m)"
            dataValue: posY
        }
    }

    Rectangle {
        id: altitude_and_velocity
        color: "transparent"
        height: 56
        anchors {
            top: position_xy.bottom
            left: parent.left
            right: parent.right
            topMargin: 8
            leftMargin: header.anchors.leftMargin
            rightMargin: header.anchors.leftMargin
        }

        DataBox {
            anchors.top: parent.top
            sections: 2; section_num: 1
            dataName: "ALTITUDE (m)"
            dataValue: altitude
            valueColor: Theme.amber
        }

        DataBox {
            anchors.top: parent.top
            sections: 2; section_num: 2
            dataName: "VELOCITY (m/s)"
            dataValue: velocity
            valueColor: Theme.amber
        }
    }
}

import QtQuick
import QtQuick.Controls

ScrollBar {
    id: bar
    policy: ScrollBar.AsNeeded
    minimumSize: 0.1

    contentItem: Rectangle {
        implicitWidth: 6
        implicitHeight: 6
        radius: 0
        color: Theme.border
        opacity: bar.pressed ? 0.95 : (bar.hovered ? 0.8 : 0.55)
        Behavior on opacity { NumberAnimation { duration: Theme.transitionFast } }
    }

    background: Rectangle {
        color: "transparent"
        border.width: 0
    }
}

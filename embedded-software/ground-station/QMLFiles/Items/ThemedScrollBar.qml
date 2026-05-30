import QtQuick
import QtQuick.Controls

ScrollBar {
    id: bar
    policy: ScrollBar.AsNeeded
    minimumSize: 0.1

    contentItem: Rectangle {
        implicitWidth: 6
        implicitHeight: 6
        radius: 3
        color: Theme.borderLight
        opacity: bar.pressed ? 0.9 : (bar.hovered ? 0.7 : 0.45)
        Behavior on opacity { NumberAnimation { duration: Theme.transitionFast } }
    }

    background: Rectangle {
        color: "transparent"
        border.width: 0
    }
}

import QtQuick
import QtQuick.Controls

CheckBox {
    id: cb
    hoverEnabled: true
    padding: 4
    spacing: 8
    font.family: Theme.fontFamily
    font.pixelSize: Theme.fontBody

    indicator: Rectangle {
        implicitWidth: 16
        implicitHeight: 16
        x: cb.leftPadding
        y: parent.height / 2 - height / 2
        radius: 3
        color: cb.checked ? Theme.accentSubtle : Theme.surfaceInset
        border.width: Theme.strokeControl
        border.color: cb.checked ? Theme.accent
                    : cb.hovered ? Theme.borderLight
                    :              Theme.border
        Behavior on color { ColorAnimation { duration: Theme.transitionFast } }

        Rectangle {
            anchors.centerIn: parent
            width: 8
            height: 8
            radius: 2
            color: Theme.accent
            visible: cb.checked
        }
    }

    contentItem: Text {
        text: cb.text
        font: cb.font
        color: Theme.textPrimary
        verticalAlignment: Text.AlignVCenter
        leftPadding: cb.indicator.width + cb.spacing
    }
}

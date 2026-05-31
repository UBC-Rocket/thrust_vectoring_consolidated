import QtQuick
import QtQuick.Controls.Basic

TabButton {
    id: tab
    hoverEnabled: true
    font.family: Theme.fontFamily
    font.pixelSize: Theme.fontBody

    background: Rectangle {
        radius: Theme.radiusControl
        color: tab.checked ? Theme.btnPrimaryBg
             : tab.down    ? Theme.btnSecondaryPress
             : tab.hovered ? Theme.btnSecondaryHover
             :               Theme.btnSecondaryBg
        border.width: Theme.strokeControl
        border.color: tab.checked ? Theme.btnPrimaryBorder : Theme.btnSecondaryBorder
        Behavior on color { ColorAnimation { duration: Theme.transitionFast } }
    }

    contentItem: Text {
        anchors.centerIn: parent
        text: tab.text
        color: tab.checked ? Theme.btnPrimaryText : Theme.btnSecondaryText
        font: tab.font
    }
}

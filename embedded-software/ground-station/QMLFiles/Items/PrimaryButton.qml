import QtQuick
import QtQuick.Controls.Basic as Basic

Basic.Button {
    id: btn
    hoverEnabled: true
    padding: 10
    font.family: Theme.fontFamily
    font.pixelSize: Theme.fontBody

    background: Rectangle {
        radius: Theme.radiusControl
        color: !btn.enabled    ? Theme.btnSecondaryBg
             : btn.down        ? Theme.btnPrimaryPress
             : btn.hovered     ? Theme.btnPrimaryHover
             :                   Theme.btnPrimaryBg
        border.width: Theme.strokeControl
        border.color: btn.enabled ? Theme.btnPrimaryBorder : Theme.btnSecondaryBorder
        Behavior on color { ColorAnimation { duration: Theme.transitionFast } }
    }

    contentItem: Text {
        anchors.centerIn: parent
        text: btn.text
        color: btn.enabled ? Theme.btnPrimaryText : Theme.textTertiary
        font: btn.font
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }
}

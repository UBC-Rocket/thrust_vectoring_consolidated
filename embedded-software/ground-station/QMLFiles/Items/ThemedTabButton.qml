import QtQuick
import QtQuick.Controls.Basic

TabButton {
    id: tab
    hoverEnabled: true
    font.family: Theme.fontFamilySemiBold
    font.pixelSize: Theme.fontBody

    background: ClippedRect {
        implicitWidth: 100
        implicitHeight: 36
        slantTL: 10
        slantBR: 10
        fillColor: tab.down    ? Theme.btnSecondaryPress
                 : tab.hovered ? Theme.btnSecondaryHover
                 :               Theme.btnSecondaryBg
        borderColor: (tab.checked || tab.hovered) ? Theme.accent : Theme.btnSecondaryBorder
        borderWidth: 1
    }

    contentItem: Text {
        text: tab.text.toUpperCase()
        color: tab.checked ? Theme.textPrimary
             : tab.hovered ? Theme.accentMuted
             :               Theme.btnSecondaryText
        font: tab.font
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }
}

import QtQuick
import QtQuick.Controls.Basic as Basic

// Red-outline EVA action button (SEND / LOAD / CLEAR …) with angular cuts.
Basic.Button {
    id: btn
    hoverEnabled: true
    padding: 10
    leftPadding: 18
    rightPadding: 18
    font.family: Theme.fontFamilySemiBold
    font.pixelSize: Theme.fontBody
    font.letterSpacing: 2

    background: ClippedRect {
        implicitWidth: 100
        implicitHeight: 36
        slantTL: 10
        slantBR: 10
        fillColor: !btn.enabled ? Theme.btnSecondaryBg
                 : btn.down     ? Theme.btnPrimaryPress
                 : btn.hovered  ? Theme.btnPrimaryHover
                 :                Theme.btnPrimaryBg
        borderColor: btn.enabled ? Theme.btnPrimaryBorder : Theme.btnSecondaryBorder
        borderWidth: 1
    }

    contentItem: Text {
        text: btn.text.toUpperCase()
        color: btn.enabled ? Theme.btnPrimaryText : Theme.textTertiary
        font: btn.font
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }
}

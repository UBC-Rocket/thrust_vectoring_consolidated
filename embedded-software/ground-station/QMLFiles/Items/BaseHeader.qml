import QtQuick

Rectangle {
    id: header

    property string headerText
    // Japanese panel label rendered beside the heading (anime mode only).
    property string jpText: ""

    color: "transparent"

    height: 50
    anchors {
        top: parent.top
        left: parent.left
        topMargin: 15
        leftMargin: 15
    }

    Text {
        //Initializing Header for the panel
        id: header_Panel
        color: Theme.accentMuted
        text: headerText.toUpperCase()
        font.family: Theme.fontFamily
        font.pixelSize: Theme.fontH2
        font.bold: true
        font.letterSpacing: 2
    }

    JpText {
        anchors.left: header_Panel.right
        anchors.leftMargin: 10
        anchors.baseline: header_Panel.baseline
        text: header.jpText
        color: Theme.textTertiary
    }
}

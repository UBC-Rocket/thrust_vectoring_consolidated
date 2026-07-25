import QtQuick

// Amber section divider: chip text followed by a thin rule to the right edge.
Item {
    id: root

    property string text: ""

    implicitHeight: chip.implicitHeight

    Text {
        id: chip
        anchors.verticalCenter: parent.verticalCenter
        text: root.text
        color: Theme.amber
        font.family: Theme.fontFamily
        font.pixelSize: 12
        font.bold: true
        font.letterSpacing: 3
    }

    Rectangle {
        anchors.left: chip.right
        anchors.leftMargin: 10
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        height: 1
        color: Theme.divider
    }
}

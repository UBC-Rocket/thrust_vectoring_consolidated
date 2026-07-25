import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: base

    // 3px top accent strip; amber for command panels, red elsewhere.
    property color accentColor: Theme.accent

    //Visualization
    color: Theme.surface
    border.color: Theme.border
    border.width: Theme.strokePanel
    radius: 0

    //Sizing
    height: (parent.parent.height - 20)/2 - 10
    width: (parent.parent.width - 20)/4 - 5

    //Layout
    Layout.margins: 2
    Layout.fillWidth: true
    Layout.fillHeight: true
    Layout.minimumWidth: width
    Layout.minimumHeight: height

    Rectangle {
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: Theme.accentStroke
        color: base.accentColor
    }
}

import QtQuick

// EVA angular button: parallelogram-cut body, outline style, hover shift to
// red. Color roles are configurable so the same component covers the dim
// nav/secondary buttons, amber toggles, green LAUNCH, and red ABORT.
Item {
    id: root

    property string text: ""
    property string jpText: ""            // Japanese suffix, hidden in normal mode
    property real cut: 10                 // shorthand for slantTL + slantBR
    property real slantTL: cut
    property real slantTR: 0
    property real slantBL: 0
    property real slantBR: cut
    property color fillColor: Theme.btnSecondaryBg
    property color hoverFillColor: fillColor
    property color borderColor: Theme.btnSecondaryBorder
    property color hoverBorderColor: Theme.accent
    property color textColor: Theme.textPrimary
    property color hoverTextColor: Theme.accentMuted
    property real borderWidth: 1
    property int fontSize: 13
    property int fontLetterSpacing: 2
    property bool bold: true
    property real horizontalPadding: 20
    property real verticalPadding: 9

    readonly property bool hovered: mouse.containsMouse
    readonly property bool pressed: mouse.pressed

    signal clicked()

    implicitWidth: label.implicitWidth + horizontalPadding * 2
    implicitHeight: label.implicitHeight + verticalPadding * 2
    opacity: enabled ? 1.0 : 0.4

    ClippedRect {
        anchors.fill: parent
        slantTL: root.slantTL
        slantTR: root.slantTR
        slantBL: root.slantBL
        slantBR: root.slantBR
        fillColor: root.hovered ? root.hoverFillColor : root.fillColor
        borderColor: root.hovered ? root.hoverBorderColor : root.borderColor
        borderWidth: root.borderWidth
    }

    Text {
        id: label
        anchors.centerIn: parent
        text: UiState.animeMode && root.jpText.length > 0
              ? root.text + " · " + root.jpText
              : root.text
        color: root.hovered ? root.hoverTextColor : root.textColor
        font.family: root.bold ? Theme.fontFamilySemiBold : Theme.fontFamily
        font.pixelSize: root.fontSize
        font.letterSpacing: root.fontLetterSpacing
    }

    MouseArea {
        id: mouse
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: root.clicked()
    }
}

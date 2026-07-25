import QtQuick
import QtQuick.Controls.Basic as Basic
import QtQuick.Layouts

// Horizontal mode tab strip. The selected index drives whichever SwipeView / StackLayout
// the parent provides via `currentIndex`.
Row {
    id: bar

    property var labels: []
    property int currentIndex: 0
    signal activated(int index)

    spacing: 8

    Repeater {
        model: bar.labels

        delegate: Basic.Button {
            id: tab
            required property int index
            required property string modelData

            hoverEnabled: true
            padding: 0
            width: Math.max(88, tabText.implicitWidth + 40)
            height: 34
            font.family: Theme.fontFamilySemiBold
            font.pixelSize: Theme.fontBody

            readonly property bool selected: index === bar.currentIndex

            background: ClippedRect {
                slantTL: 10
                slantBR: 10
                fillColor: tab.down    ? Theme.btnSecondaryPress
                         : tab.hovered ? Theme.btnSecondaryHover
                         :               Theme.surfaceSolid
                borderColor: (tab.selected || tab.hovered) ? Theme.accent : Theme.border
                borderWidth: 1
            }

            contentItem: Item {
                anchors.fill: parent
                Text {
                    id: tabText
                    anchors.centerIn: parent
                    text: tab.modelData.toUpperCase()
                    color: tab.selected ? Theme.textPrimary
                         : tab.hovered  ? Theme.accentMuted
                         :                Theme.textSecondary
                    font.family: tab.font.family
                    font.pixelSize: tab.font.pixelSize
                    font.letterSpacing: 2
                }

                // Active-tab underline bar, inset 14px per side.
                Rectangle {
                    visible: tab.selected
                    anchors.bottom: parent.bottom
                    anchors.bottomMargin: 3
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: parent.width - 28
                    height: 3
                    color: Theme.accent
                }
            }

            onClicked: bar.activated(index)
        }
    }
}

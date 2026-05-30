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

    spacing: 4

    Repeater {
        model: bar.labels

        delegate: Basic.Button {
            id: tab
            required property int index
            required property string modelData

            hoverEnabled: true
            padding: 0
            width: Math.max(88, tabText.implicitWidth + 24)
            height: 32
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontBody

            readonly property bool selected: index === bar.currentIndex

            background: Rectangle {
                radius: Theme.radiusControl
                color: tab.selected ? Theme.accentSubtle
                     : tab.down     ? Theme.btnSecondaryPress
                     : tab.hovered  ? Theme.btnSecondaryHover
                     :                "transparent"
                border.width: tab.selected ? 1 : 0
                border.color: Theme.accent
                Behavior on color { ColorAnimation { duration: Theme.transitionFast } }
            }

            contentItem: Item {
                anchors.fill: parent
                Text {
                    id: tabText
                    anchors.centerIn: parent
                    text: tab.modelData
                    color: tab.selected ? Theme.accent : Theme.textSecondary
                    font.family: tab.font.family
                    font.pixelSize: tab.font.pixelSize
                    font.bold: tab.selected
                }

                Rectangle {
                    visible: tab.selected
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: parent.width - 16
                    height: 2
                    radius: 1
                    color: Theme.accent
                }
            }

            onClicked: bar.activated(index)
        }
    }
}

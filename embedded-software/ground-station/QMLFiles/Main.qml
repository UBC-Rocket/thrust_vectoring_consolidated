import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Basic as Basic
import QtQuick.Layouts
import QtQuick.Window
import "Items"
import "Panels"

ApplicationWindow {
    //Initializing the Window
    id: window
    width: 1400
    height: 900
    visible: true
    title: qsTr("Ulysses Ground Control")
    color: Theme.background

    //Import all the keyboard shortcuts
    Shortcuts {
        targetWindow: window        //Passing in arguments
    }

    Header {
        id: headerMain
    }

    Rectangle {
        //Initialize the actual content board
        id: contentBoard

        width: parent.width - 4
        height: parent.height - (headerMain.height) - 4
        color: Theme.background
        anchors {
            top: headerMain.bottom
            horizontalCenter: parent.horizontalCenter
        }


        // Pages: [main grid, UWB probe map, radio console, radio output]
        // Switched via the arrow row, not by swipe
        SwipeView {
            id: pageSwipe
            anchors.fill: parent
            anchors.bottomMargin: navRow.height + 8
            currentIndex: 0
            clip: true
            interactive: false

            Item {
                LayoutGrid { anchors.fill: parent }
            }

            Item {
                Panel_Probe_Map { anchors.fill: parent; anchors.margins: 2 }
            }

            Item {
                Panel_Radio_Console { anchors.fill: parent; anchors.margins: 2 }
            }

            Item {
                Panel_Radio_Output { anchors.fill: parent; anchors.margins: 2 }
            }
        }

        // Bottom-center navigation row: ‹  • •  ›
        RowLayout {
            id: navRow
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 6
            spacing: 10
            z: 100

            Basic.Button {
                id: prevBtn
                text: "‹"
                enabled: pageSwipe.currentIndex > 0
                hoverEnabled: true
                Layout.preferredWidth: 24
                Layout.preferredHeight: 22
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontBody
                background: Rectangle {
                    radius: Theme.radiusControl
                    color: !prevBtn.enabled ? Theme.surface
                         : prevBtn.down     ? Theme.btnSecondaryPress
                         : prevBtn.hovered  ? Theme.btnSecondaryHover
                         :                    Theme.btnSecondaryBg
                    border.width: Theme.strokeControl
                    border.color: Theme.btnSecondaryBorder
                }
                contentItem: Text {
                    anchors.centerIn: parent
                    text: prevBtn.text
                    color: prevBtn.enabled ? Theme.textSecondary : Theme.textTertiary
                    font.pixelSize: 16
                    font.family: Theme.fontFamily
                }
                onClicked: pageSwipe.decrementCurrentIndex()
            }

            PageIndicator {
                id: pageIndicator
                count: pageSwipe.count
                currentIndex: pageSwipe.currentIndex
                interactive: true
                onCurrentIndexChanged: pageSwipe.currentIndex = currentIndex

                delegate: Rectangle {
                    implicitWidth: 10
                    implicitHeight: 10
                    radius: 5
                    color: index === pageIndicator.currentIndex ? Theme.accent : Theme.borderLight
                    border.color: Theme.border
                    border.width: 1

                    MouseArea {
                        anchors.fill: parent
                        onClicked: pageSwipe.currentIndex = index
                    }
                }
            }

            Basic.Button {
                id: nextBtn
                text: "›"
                enabled: pageSwipe.currentIndex < pageSwipe.count - 1
                hoverEnabled: true
                Layout.preferredWidth: 24
                Layout.preferredHeight: 22
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontBody
                background: Rectangle {
                    radius: Theme.radiusControl
                    color: !nextBtn.enabled ? Theme.surface
                         : nextBtn.down     ? Theme.btnSecondaryPress
                         : nextBtn.hovered  ? Theme.btnSecondaryHover
                         :                    Theme.btnSecondaryBg
                    border.width: Theme.strokeControl
                    border.color: Theme.btnSecondaryBorder
                }
                contentItem: Text {
                    anchors.centerIn: parent
                    text: nextBtn.text
                    color: nextBtn.enabled ? Theme.textSecondary : Theme.textTertiary
                    font.pixelSize: 16
                    font.family: Theme.fontFamily
                }
                onClicked: pageSwipe.incrementCurrentIndex()
            }
        }
    }

}

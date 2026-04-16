import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import "Items"
import "Modes"

ApplicationWindow {
    id: window
    width: 1400
    height: 900
    visible: true
    title: qsTr("Ulysses Ground Control")
    color: Theme.background

    Shortcuts {
        targetWindow: window
    }

    Header {
        id: headerMain

        // Mode bar drives the mode container below.
        modeBar.currentIndex: modeStack.currentIndex
        modeBar.onActivated: (index) => modeStack.currentIndex = index
    }

    Rectangle {
        id: contentBoard
        width: parent.width - 4
        height: parent.height - (headerMain.height) - 4
        color: Theme.background
        anchors {
            top: headerMain.bottom
            horizontalCenter: parent.horizontalCenter
        }

        SwipeView {
            id: modeStack
            anchors.fill: parent
            currentIndex: 0
            clip: true
            interactive: false

            // Keep ordering aligned with Header.modeBar.labels
            Mode_Flight { }
            Mode_Tuning { }
            Mode_Map { }
            Mode_Diagnostics { }
        }
    }
}

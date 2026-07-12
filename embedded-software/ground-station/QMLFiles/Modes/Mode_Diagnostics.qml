import QtQuick
import QtQuick.Layouts
import "../Items"
import "../Panels"

Item {
    RowLayout {
        anchors.fill: parent
        anchors.margins: 6
        anchors.topMargin: Theme.gridSpacing
        spacing: Theme.gridSpacing

        Panel_Radio_Console {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 1
        }

        Panel_Radio_Output {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 1
        }
    }
}

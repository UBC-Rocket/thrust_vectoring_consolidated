// LayoutGrid.qml
import QtQuick
import QtQuick.Layouts
import "Items"
import "Panels"

GridLayout {
    id: grid
    columns: 4
    flow: GridLayout.LeftToRight
    rowSpacing: Theme.gridSpacing
    columnSpacing: Theme.gridSpacing
    anchors.fill: parent
    anchors.topMargin: 2

    // ───── Row 0 ──────────────────────────────────────────────────────────

    // Top-left: stack State+Position and System_Health, each at half cell height.
    Item {
        Layout.row: 0; Layout.column: 0
        Layout.fillWidth: true; Layout.fillHeight: true

        ColumnLayout {
            anchors.fill: parent
            spacing: Theme.gridSpacing

            Item {
                Layout.fillWidth: true; Layout.fillHeight: true
                Panel_State_And_Position { anchors.fill: parent }
            }

            Item {
                Layout.fillWidth: true; Layout.fillHeight: true
                Panel_System_Health { anchors.fill: parent }
            }
        }
    }

    Item {
        Layout.row: 0; Layout.column: 1; Layout.columnSpan: 2
        Layout.fillWidth: true; Layout.fillHeight: true
        Panel_Rocket_Visualization { anchors.fill: parent }
    }

    // Right column: Angles+Engine spans both rows (double height).
    Item {
        Layout.row: 0; Layout.column: 3; Layout.rowSpan: 2
        Layout.fillWidth: true; Layout.fillHeight: true
        Panel_Angles_And_Engine { anchors.fill: parent }
    }

    // ───── Row 1 ──────────────────────────────────────────────────────────

    Item {
        Layout.row: 1; Layout.column: 0; Layout.columnSpan: 2
        Layout.fillWidth: true; Layout.fillHeight: true
        Panel_Control { anchors.fill: parent }
    }

    Item {
        Layout.row: 1; Layout.column: 2
        Layout.fillWidth: true; Layout.fillHeight: true
        Panel_System_Alert { anchors.fill: parent }
    }
}

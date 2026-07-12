// LayoutGrid.qml — FLIGHT screen grid (EVA-02)
// 300px | 1fr | 300px columns, 14px gaps.
// Row 0: Angles/Engine | Rocket Visualization | (State/Position + System Health stacked)
// Row 1: Command Controls (spans cols 0-1)    | System Alert (col 2)
import QtQuick
import QtQuick.Layouts
import "Items"
import "Panels"

GridLayout {
    id: grid
    columns: 3
    flow: GridLayout.LeftToRight
    rowSpacing: Theme.gridSpacing
    columnSpacing: Theme.gridSpacing
    anchors.fill: parent
    anchors.topMargin: 2

    // ───── Row 0 ──────────────────────────────────────────────────────────

    Item {
        Layout.row: 0; Layout.column: 0
        Layout.preferredWidth: 300
        Layout.minimumWidth: 300
        Layout.maximumWidth: 300
        Layout.fillHeight: true
        Panel_Angles_And_Engine { anchors.fill: parent }
    }

    Item {
        Layout.row: 0; Layout.column: 1
        Layout.fillWidth: true; Layout.fillHeight: true
        Panel_Rocket_Visualization { anchors.fill: parent }
    }

    Item {
        Layout.row: 0; Layout.column: 2
        Layout.preferredWidth: 300
        Layout.minimumWidth: 300
        Layout.maximumWidth: 300
        Layout.fillHeight: true

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

    // ───── Row 1 ──────────────────────────────────────────────────────────

    Item {
        Layout.row: 1; Layout.column: 0; Layout.columnSpan: 2
        Layout.fillWidth: true
        Layout.fillHeight: false
        Layout.preferredHeight: 200
        Panel_Control { anchors.fill: parent }
    }

    Item {
        Layout.row: 1; Layout.column: 2
        Layout.preferredWidth: 300
        Layout.minimumWidth: 300
        Layout.maximumWidth: 300
        Layout.fillHeight: false
        Layout.preferredHeight: 200
        Panel_System_Alert { anchors.fill: parent }
    }
}

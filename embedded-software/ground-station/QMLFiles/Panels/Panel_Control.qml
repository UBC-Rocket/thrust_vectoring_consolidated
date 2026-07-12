import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../Items"

// COMMAND CONTROLS (指令入力) — amber-accent panel, four cut command buttons.
BasePanel {
    id: control_panel
    accentColor: Theme.amber
    property int txWhich: 1
    signal commandTriggered(int which, string code) // Public signal: emitted when a command card is clicked; parent can handle it.

    // --- Header area: amber title + kana label (spec exception: amber, not red) ---
    Item {
        id: header
        height: headerLabel.implicitHeight
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            topMargin: 15
            leftMargin: 15
            rightMargin: 15
        }

        Text {
            id: headerLabel
            text: "COMMAND CONTROLS"
            color: Theme.amber
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontH2
            font.bold: true
            font.letterSpacing: 2
        }

        JpText {
            anchors.left: headerLabel.right
            anchors.leftMargin: 10
            anchors.baseline: headerLabel.baseline
            text: "指令入力"
            color: Theme.textTertiary
        }
    }

    // --- Card grid: four equal command buttons in one row, 12px gaps ---
    GridLayout {
        id: cards
        anchors {
            top: header.bottom; topMargin: 12
            left: parent.left; right: parent.right; bottom: parent.bottom
            leftMargin: 15; rightMargin: 15; bottomMargin: 15
        }
        columns: 4
        rowSpacing: 12
        columnSpacing: 12

        // --- Reusable command card component: displays a title and the code it sends ---
        CommandCard {
            id: cmdCard
        }

        // --- Four grid items below instantiate the reusable card with different title/code pairs ---

        // ARM -> sends "1" (amber border/label)
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Loader {
                anchors.fill: parent
                sourceComponent: cmdCard
                onLoaded: {
                    item.title = "Arm"; item.cmd = 1 // Initialize instance properties.
                    item.jpSuffix = "兵装"
                    item.borderColor = Theme.amber
                    item.hoverFillColor = "#24ffb400"    // rgba(255,180,0,0.14)
                }
            }
        }

        // LAUNCH -> sends "2" (green border, green label)
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Loader {
                anchors.fill: parent
                sourceComponent: cmdCard
                onLoaded: {
                    item.title = "Launch"; item.cmd = 2
                    item.jpSuffix = "発進"
                    item.borderColor = Theme.success
                    item.labelColor = Theme.success
                    item.hoverFillColor = "#1f74e05a"    // rgba(116,224,90,0.12)
                }
            }
        }

        // ABORT -> sends "3" (2px red border, red label, hazard-stripe fill)
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Loader {
                anchors.fill: parent
                sourceComponent: cmdCard
                onLoaded: {
                    item.title = "Abort"; item.cmd = 3
                    item.jpSuffix = "中止"
                    item.striped = true
                    item.borderWidth = 2
                    item.borderColor = Theme.accent
                    item.labelColor = Theme.accent
                    item.subLabelColor = Theme.textSecondary
                    item.hoverFillColor = "#4dff3b2f"    // rgba(255,59,47,0.3)
                }
            }
        }

        // LAND -> sends "4" (dim border; hover brightens the border)
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Loader {
                anchors.fill: parent
                sourceComponent: cmdCard
                onLoaded: {
                    item.title = "Land"; item.cmd = 4
                    item.jpSuffix = "着陸"
                    item.borderColor = Theme.border
                    item.hoverBorderColor = Theme.accentMuted
                }
            }
        }
    }

    // --- Signal wiring: when a card is clicked, forward the code to the C++ CommandSender object ---
    Connections {
        target: control_panel
        function onCommandTriggered(txWhich, code) {
            commandsender.sendFlightCommand(txWhich, code) // Delegate to Q_INVOKABLE; panel stays transport-agnostic.
            if (Number(code) === 3)
                UiState.alarmActive = true // ABORT also raises the emergency overlay (ACKNOWLEDGE clears it).
        }
    }
}

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

    // ── z0: graph-grid backdrop ─────────────────────────────────────────
    GridBackdrop {
        anchors.fill: parent
        z: 0
    }

    // ── z5: EVA side strips (anime mode only) ───────────────────────────
    SideStripLeft {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        visible: UiState.animeMode
        z: 5
    }

    SideStripRight {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        visible: UiState.animeMode
        z: 5
    }

    // ── z10: main content column ────────────────────────────────────────
    Item {
        id: mainColumn
        anchors.fill: parent
        anchors.leftMargin: UiState.animeMode ? 224 : 14
        anchors.rightMargin: UiState.animeMode ? 224 : 14
        z: 10

        Header {
            id: headerMain

            // Mode bar drives the mode container below.
            modeBar.currentIndex: modeStack.currentIndex
            modeBar.onActivated: (index) => modeStack.currentIndex = index
        }

        Rectangle {
            id: contentBoard
            color: "transparent"
            anchors {
                top: headerMain.bottom
                topMargin: Theme.gridSpacing
                left: parent.left
                right: parent.right
                bottom: footerStrip.top
                bottomMargin: 10
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

        // ── Footer strip ────────────────────────────────────────────────
        RowLayout {
            id: footerStrip
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 10
            spacing: 14

            WarningStripes {
                Layout.fillWidth: true
                Layout.preferredHeight: 10
                colorA: Theme.amber
                colorB: "transparent"
                angleDeg: -45
                opacity: 0.5
            }

            Text {
                text: "UBC ROCKET · GROUND STATION v2.0"
                      + (UiState.animeMode ? " · 特別非常事態宣言対応" : "")
                font.family: Theme.fontFamily
                font.pixelSize: 10
                font.letterSpacing: 3
                color: Theme.textTertiary
            }

            WarningStripes {
                Layout.fillWidth: true
                Layout.preferredHeight: 10
                colorA: Theme.amber
                colorB: "transparent"
                angleDeg: -45
                opacity: 0.5
            }
        }
    }

    // ── z60: scanlines (under the takeover overlays) ────────────────────
    Scanlines {
        anchors.fill: parent
        z: 60
    }

    // ── z80: emergency abort takeover ───────────────────────────────────
    EmergencyOverlay {
        z: 80
    }

    // ── z100: boot sequence ─────────────────────────────────────────────
    BootOverlay {
        z: 100
    }
}

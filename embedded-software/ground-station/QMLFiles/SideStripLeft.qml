import QtQuick
import QtQuick.Layouts
import QtQuick.Effects
import "Items"

// Left side strip (anime mode only): active pilot's unit header + figure +
// pilot sync. All character content and colors follow Theme.pilot.
Rectangle {
    id: strip

    width: 200

    gradient: Gradient {
        GradientStop { position: 0.0; color: Theme.strip1 }
        GradientStop { position: 0.6; color: Theme.strip2 }
        GradientStop { position: 1.0; color: Theme.surfaceElevated }
    }

    // 2px inner border on the content side.
    Rectangle {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: 2
        color: Theme.accent
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.rightMargin: 2
        spacing: 0

        WarningStripes {
            Layout.fillWidth: true
            Layout.preferredHeight: 14
            angleDeg: -45
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.margins: 0
            Layout.topMargin: 14
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            Layout.bottomMargin: 8
            spacing: 4

            Text {
                Layout.fillWidth: true
                text: Theme.pilotUnitKanji
                font.family: Theme.japaneseFamilyHeavy
                font.pixelSize: 26
                font.letterSpacing: 2
                color: Theme.accent
            }
            Text {
                // Wrap long unit labels (e.g. "DEMON SLAYER · LOVE HASHIRA")
                // so they stay inside the strip instead of overflowing.
                Layout.fillWidth: true
                text: Theme.pilotUnitLabel
                font.family: Theme.fontFamily
                font.pixelSize: 10
                font.letterSpacing: 3
                lineHeight: 1.3
                wrapMode: Text.WordWrap
                color: Theme.textSecondary
            }
        }

        // Character area: bottom-anchored figure with accent glow + top fade.
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true

            // Accent glow: shadow-only duplicate rendered BEHIND the visible
            // image, so the figure still shows if effects can't render
            // (e.g. software scenegraph fallback).
            MultiEffect {
                source: pilotFigure
                anchors.fill: pilotFigure
                visible: pilotFigure.status === Image.Ready
                shadowEnabled: true
                shadowColor: Theme.accentGlow
                shadowBlur: 1.0
                blurMax: 36
            }

            Image {
                id: pilotFigure
                source: "../Resources/images/" + Theme.pilotImage
                // Cap to the strip's inner width so a wide pose keeps a
                // margin off the accent border (never wider than handoff spec).
                width: Math.min(Theme.pilotImageWidth, parent.width - 16)
                fillMode: Image.PreserveAspectFit
                // Only show once the new source resolves — avoids a
                // broken-image flash when switching pilots.
                visible: status === Image.Ready
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottom: parent.bottom
                anchors.bottomMargin: -10
            }

            // Dark fade from the top, gone by 35% height.
            Rectangle {
                anchors.fill: parent
                gradient: Gradient {
                    GradientStop { position: 0.0;  color: Qt.rgba(Theme.strip1.r, Theme.strip1.g, Theme.strip1.b, 0.55) }
                    GradientStop { position: 0.35; color: Qt.rgba(Theme.strip1.r, Theme.strip1.g, Theme.strip1.b, 0.0) }
                    GradientStop { position: 1.0;  color: Qt.rgba(Theme.strip1.r, Theme.strip1.g, Theme.strip1.b, 0.0) }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: footerRow.implicitHeight + 20
            color: "transparent"

            Rectangle {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                height: 1
                color: Theme.border
            }

            RowLayout {
                id: footerRow
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 12
                Text {
                    text: "PILOT SYNC"
                    font.family: Theme.fontFamily
                    font.pixelSize: 10
                    font.letterSpacing: 2
                    color: Theme.textSecondary
                }
                Item { Layout.fillWidth: true }
                Text {
                    text: UiState.syncRate.toFixed(1) + "%"
                    font.family: Theme.fontFamily
                    font.pixelSize: 18
                    font.bold: true
                    color: Theme.success
                }
            }
        }

        WarningStripes {
            Layout.fillWidth: true
            Layout.preferredHeight: 14
            angleDeg: -45
        }
    }
}

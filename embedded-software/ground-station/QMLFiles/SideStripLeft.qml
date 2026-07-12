import QtQuick
import QtQuick.Layouts
import QtQuick.Effects
import "Items"

// EVA left side strip (anime mode only): 弐号機 header + Asuka figure + pilot sync.
Rectangle {
    id: strip

    width: 200

    gradient: Gradient {
        GradientStop { position: 0.0; color: "#1a0908" }
        GradientStop { position: 0.6; color: "#240b08" }
        GradientStop { position: 1.0; color: "#12100a" }
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
                text: "弐号機"
                font.family: Theme.japaneseFamilyHeavy
                font.pixelSize: 26
                font.letterSpacing: 2
                color: Theme.accent
            }
            Text {
                text: "EVA UNIT-02 · SOURYU"
                font.family: Theme.fontFamily
                font.pixelSize: 10
                font.letterSpacing: 3
                color: Theme.textSecondary
            }
        }

        // Character area: bottom-anchored figure with red glow + top fade.
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true

            // Red glow: shadow-only duplicate rendered BEHIND the visible
            // image, so the figure still shows if effects can't render
            // (e.g. software scenegraph fallback).
            MultiEffect {
                source: asuka
                anchors.fill: asuka
                shadowEnabled: true
                shadowColor: "#73ff3b2f"
                shadowBlur: 1.0
                blurMax: 36
            }

            Image {
                id: asuka
                source: "../Resources/images/asuka.png"
                width: 195
                fillMode: Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottom: parent.bottom
                anchors.bottomMargin: -10
            }

            // Dark fade from the top, gone by 35% height.
            Rectangle {
                anchors.fill: parent
                gradient: Gradient {
                    GradientStop { position: 0.0;  color: "#8c1a0908" }
                    GradientStop { position: 0.35; color: "#001a0908" }
                    GradientStop { position: 1.0;  color: "#001a0908" }
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

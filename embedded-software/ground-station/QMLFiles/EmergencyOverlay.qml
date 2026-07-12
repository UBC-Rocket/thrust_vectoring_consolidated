import QtQuick
import QtQuick.Effects
import "Items"

// Emergency takeover (z80): shown while UiState.alarmActive. The ABORT
// command itself is sent by the command panel — this overlay is pure UI.
Item {
    id: root

    anchors.fill: parent
    visible: UiState.alarmActive

    // Dimmed red backdrop.
    Rectangle {
        anchors.fill: parent
        color: "#b83c0402"
    }

    // Pulsing red radial vignette.
    Canvas {
        id: vignette
        anchors.fill: parent
        opacity: 0.55
        onWidthChanged: requestPaint()
        onHeightChanged: requestPaint()
        onPaint: {
            const ctx = getContext("2d")
            ctx.reset()
            const g = ctx.createRadialGradient(width / 2, height / 2, Math.min(width, height) * 0.3,
                                               width / 2, height / 2, Math.max(width, height) * 0.72)
            g.addColorStop(0, "rgba(255,20,10,0)")
            g.addColorStop(1, "rgba(255,20,10,0.5)")
            ctx.fillStyle = g
            ctx.fillRect(0, 0, width, height)
        }

        SequentialAnimation on opacity {
            running: root.visible
            loops: Animation.Infinite
            NumberAnimation { to: 0.95; duration: 450; easing.type: Easing.InOutQuad }
            NumberAnimation { to: 0.55; duration: 450; easing.type: Easing.InOutQuad }
        }
    }

    // Swallow all interaction beneath the overlay.
    MouseArea { anchors.fill: parent; hoverEnabled: true }

    Column {
        anchors.verticalCenter: parent.verticalCenter
        width: parent.width
        spacing: 0

        WarningStripes {
            id: topBand
            width: parent.width
            height: 34
            colorA: Theme.alarm
            colorB: "#120404"
            stripeWidth: 28
            angleDeg: -45
            NumberAnimation on offset {
                running: root.visible
                loops: Animation.Infinite
                from: 0; to: 56
                duration: 800
            }
        }

        Rectangle {
            width: parent.width
            height: centerCol.implicitHeight + 68
            color: "#120404"

            Rectangle { anchors.top: parent.top; width: parent.width; height: 3; color: Theme.alarm }
            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 3; color: Theme.alarm }

            Column {
                id: centerCol
                anchors.centerIn: parent
                width: parent.width - 40
                spacing: 0

                Item {
                    width: parent.width
                    height: UiState.animeMode ? 96 : 0
                    visible: UiState.animeMode

                    Text {
                        id: jpHeadline
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: "緊急事態"
                        font.family: Theme.japaneseFamilyHeavy
                        font.pixelSize: 96
                        font.letterSpacing: 12
                        lineHeight: 1.0
                        color: Theme.alarm
                        visible: false
                    }
                    MultiEffect {
                        id: jpHeadlineFx
                        source: jpHeadline
                        anchors.fill: jpHeadline
                        shadowEnabled: true
                        shadowColor: "#ccff2413"
                        shadowBlur: 1.0
                        blurMax: 64
                    }
                    Blink { target: jpHeadlineFx; period: 1000 }
                }

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    topPadding: 14
                    text: "EMERGENCY — ABORT SENT"
                    font.family: Theme.fontFamily
                    font.pixelSize: 30
                    font.bold: true
                    font.letterSpacing: 18
                    color: Theme.textPrimary
                }

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    topPadding: 12
                    text: "CMD \"3\" TRANSMITTED · ENGINE SAFING · RECOVERY SEQUENCE ACTIVE"
                    font.family: Theme.monoFamily
                    font.pixelSize: 13
                    font.letterSpacing: 2
                    color: "#ff8a70"
                }

                Item { width: 1; height: 26 }

                ClippedButton {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: "ACKNOWLEDGE / RESET"
                    cut: 14
                    borderWidth: 2
                    fillColor: "transparent"
                    hoverFillColor: "#26ffe9dc"
                    borderColor: Theme.textPrimary
                    hoverBorderColor: Theme.textPrimary
                    textColor: Theme.textPrimary
                    hoverTextColor: Theme.textPrimary
                    fontSize: 14
                    fontLetterSpacing: 4
                    horizontalPadding: 44
                    verticalPadding: 12
                    onClicked: UiState.alarmActive = false
                }
            }
        }

        WarningStripes {
            id: bottomBand
            width: parent.width
            height: 34
            colorA: Theme.alarm
            colorB: "#120404"
            stripeWidth: 28
            angleDeg: 45
            NumberAnimation on offset {
                running: root.visible
                loops: Animation.Infinite
                from: 0; to: 56
                duration: 800
            }
        }
    }
}

import QtQuick
import QtQuick.Effects
import "Items"

// Boot sequence overlay (z100): 3.4s staged status lines + progress bar,
// fading out at the end. Skippable via the persisted skipBoot setting.
Rectangle {
    id: root

    // 0..1 fill of the progress bar, driven by the timeline below.
    property real progress: 0

    anchors.fill: parent
    color: Theme.bootBg
    visible: !UiState.booted
    enabled: visible

    // Swallow clicks while booting.
    MouseArea { anchors.fill: parent }

    Component.onCompleted: {
        if (UiState.skipBoot)
            UiState.booted = true
    }

    Column {
        anchors.centerIn: parent
        width: 520
        spacing: 10

        Item {
            width: parent.width
            height: UiState.animeMode ? jpTitle.implicitHeight : 0
            visible: UiState.animeMode

            Text {
                id: jpTitle
                text: "起動シーケンス"
                font.family: Theme.japaneseFamilyHeavy
                font.pixelSize: 58
                font.letterSpacing: 6
                color: Theme.accent
                visible: false
            }
            MultiEffect {
                source: jpTitle
                anchors.fill: jpTitle
                shadowEnabled: true
                shadowColor: Theme.accentGlowStrong
                shadowBlur: 1.0
                blurMax: 60
            }
        }

        Text {
            text: "ULYSSES GROUND STATION — BOOT SEQUENCE"
            font.family: Theme.monoFamily
            font.pixelSize: 12
            font.letterSpacing: 5
            color: Theme.textSecondary
            bottomPadding: 14
        }

        Text {
            id: bootLine1
            text: "> MAGI LINK ............ OK"
            font.family: Theme.monoFamily
            font.pixelSize: 12
            color: Theme.success
            opacity: 0
        }
        Text {
            id: bootLine2
            text: "> RFD900X RADIO ........ CONNECTED @57600"
            font.family: Theme.monoFamily
            font.pixelSize: 12
            color: Theme.success
            opacity: 0
        }
        Text {
            id: bootLine3
            text: "> IMU / GYRO / ACCEL ... NOMINAL"
            font.family: Theme.monoFamily
            font.pixelSize: 12
            color: Theme.success
            opacity: 0
        }
        Text {
            id: bootLine4
            text: "> PILOT SYNC RATE ...... 98.7%"
            font.family: Theme.monoFamily
            font.pixelSize: 12
            color: Theme.amber
            opacity: 0
        }

        // Progress bar: red→amber fill with two hesitations (~62%, ~71%).
        Rectangle {
            width: parent.width
            height: 14
            color: "transparent"
            border.width: 1
            border.color: Theme.border

            Rectangle {
                anchors.left: parent.left
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                anchors.margins: 2
                width: (parent.width - 4) * root.progress
                gradient: Gradient {
                    orientation: Gradient.Horizontal
                    GradientStop { position: 0.0; color: Theme.accent }
                    GradientStop { position: 1.0; color: Theme.amber }
                }
            }
        }
    }

    // Timeline (from the reference keyframes): lines appear stepped at
    // ~0.41 / 0.92 / 1.46 / 2.04s; bar 0→62% @1.65s, →71% @2.25s, →100% @3s;
    // fade out 2.99–3.4s; teardown 3.6s.
    SequentialAnimation {
        running: root.visible

        ParallelAnimation {
            SequentialAnimation {
                PauseAnimation { duration: 410 }
                PropertyAction { target: bootLine1; property: "opacity"; value: 1 }
                PauseAnimation { duration: 510 }
                PropertyAction { target: bootLine2; property: "opacity"; value: 1 }
                PauseAnimation { duration: 540 }
                PropertyAction { target: bootLine3; property: "opacity"; value: 1 }
                PauseAnimation { duration: 580 }
                PropertyAction { target: bootLine4; property: "opacity"; value: 1 }
            }
            SequentialAnimation {
                NumberAnimation { target: root; property: "progress"; from: 0; to: 0.62; duration: 1650; easing.type: Easing.OutQuad }
                NumberAnimation { target: root; property: "progress"; to: 0.71; duration: 600 }
                NumberAnimation { target: root; property: "progress"; to: 1.0; duration: 750; easing.type: Easing.OutQuad }
            }
        }

        PauseAnimation { duration: 340 }
        NumberAnimation { target: root; property: "opacity"; to: 0; duration: 410; easing.type: Easing.InOutQuad }
        PauseAnimation { duration: 200 }
        ScriptAction { script: UiState.booted = true }
    }
}

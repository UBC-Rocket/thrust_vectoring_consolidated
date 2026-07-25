import QtQuick
import QtQuick.Controls

// EVA command button: parallelogram-cut body (14px), 20px main label over a
// 10px SENDS "n" sub-label with an anime-only kana suffix. Accent colors are
// configurable per command (ARM amber / LAUNCH green / ABORT red-striped /
// LAND dim). Root stays a Component: Panel_Control instantiates it through
// Loaders and sets title/cmd (plus the color roles) in onLoaded.
Component {
    id: cmdCard

    Item {
        id: card
        anchors.fill: parent

        property string title: ""       // User-facing name of the command (e.g., "Arm").
        property string cmd: ""         // Actual code to emit (1..4, coerced to string).
        property string jpSuffix: ""    // Kana sub-label suffix — hidden in normal mode.

        // EVA color roles
        property real cut: 14
        property real borderWidth: 1
        property color borderColor: Theme.border
        property color hoverBorderColor: borderColor
        property color labelColor: Theme.textPrimary
        property color subLabelColor: Theme.textTertiary
        property color fillColor: Theme.surfaceElevated
        property color hoverFillColor: fillColor
        property bool striped: false    // ABORT: diagonal red hazard stripes

        readonly property bool hovered: ma.containsMouse // Centralized hover state for styling.
        Behavior on scale { NumberAnimation { duration: 90 } } // Brief press/release animation.

        // --- Cut body fill ---
        ClippedRect {
            anchors.fill: parent
            slantTL: card.cut
            slantBR: card.cut
            fillColor: card.hovered ? card.hoverFillColor : card.fillColor
        }

        // --- ABORT hazard stripes, clipped to the cut shape (hover shows plain tint) ---
        Canvas {
            anchors.fill: parent
            visible: card.striped && !card.hovered
            onWidthChanged: requestPaint()
            onHeightChanged: requestPaint()
            onVisibleChanged: if (visible) requestPaint()

            onPaint: {
                const ctx = getContext("2d")
                ctx.reset()
                ctx.beginPath()
                ctx.moveTo(card.cut, 0)
                ctx.lineTo(width, 0)
                ctx.lineTo(width - card.cut, height)
                ctx.lineTo(0, height)
                ctx.closePath()
                ctx.clip()

                const diag = Math.sqrt(width * width + height * height)
                ctx.translate(width / 2, height / 2)
                ctx.rotate(-45 * Math.PI / 180)
                ctx.fillStyle = Theme.abortStripeTint
                const stripe = 12
                for (let x = -diag; x < diag; x += stripe * 2)
                    ctx.fillRect(x, -diag, stripe, diag * 2)
            }
        }

        // --- Cut outline on top of fill/stripes ---
        ClippedRect {
            anchors.fill: parent
            slantTL: card.cut
            slantBR: card.cut
            borderColor: card.hovered ? card.hoverBorderColor : card.borderColor
            borderWidth: card.borderWidth
        }

        // --- Text stack: main label + SENDS "n" hint (safety-relevant copy) ---
        Column {
            id: labelCol
            anchors.centerIn: parent
            spacing: 4

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: card.title.toUpperCase()
                color: card.labelColor
                font.family: Theme.fontFamily
                font.pixelSize: 20
                font.bold: true
                font.letterSpacing: 3
            }

            Row {
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 0

                Text {
                    text: 'SENDS "' + card.cmd + '"' // Secondary hint: shows the code that will be sent.
                    color: card.subLabelColor
                    font.family: Theme.fontFamily
                    font.pixelSize: 10
                    font.letterSpacing: 2
                }

                JpText {
                    text: card.jpSuffix.length > 0 ? " · " + card.jpSuffix : ""
                    font.pixelSize: 10
                    color: card.subLabelColor
                }
            }
        }

        // --- Mouse handler: emits the panel's signal with this card's code when clicked ---
        MouseArea {
            id: ma
            anchors.fill: parent
            hoverEnabled: true
            cursorShape: Qt.PointingHandCursor
            onPressed: parent.scale = 0.985 // Tactile press effect (shrinks slightly).
            onReleased: parent.scale = 1.0  // Reset scale on release.
            onClicked: control_panel.commandTriggered(txWhich, cmd) // Raise event upward with payload.
        }
    }
}

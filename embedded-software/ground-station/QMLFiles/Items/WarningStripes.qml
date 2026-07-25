import QtQuick

// Diagonal hazard stripes (amber/dark by default). Angle in degrees;
// use -45 for the mirrored direction. `offset` lets overlays scroll
// the pattern for the abort-band animation.
Canvas {
    id: canvas

    property color colorA: Theme.amber
    property color colorB: Theme.stripeDark
    property real stripeWidth: 14
    property real angleDeg: 45
    property real offset: 0

    onColorAChanged: requestPaint()
    onColorBChanged: requestPaint()
    onStripeWidthChanged: requestPaint()
    onAngleDegChanged: requestPaint()
    onOffsetChanged: requestPaint()
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()

    onPaint: {
        const ctx = getContext("2d")
        ctx.reset()
        ctx.fillStyle = colorB
        ctx.fillRect(0, 0, width, height)

        const diag = Math.sqrt(width * width + height * height)
        ctx.save()
        ctx.translate(width / 2, height / 2)
        ctx.rotate(angleDeg * Math.PI / 180)
        ctx.fillStyle = colorA
        const period = stripeWidth * 2
        const start = -diag - (offset % period)
        for (let x = start; x < diag; x += period)
            ctx.fillRect(x, -diag, stripeWidth, diag * 2)
        ctx.restore()
    }
}

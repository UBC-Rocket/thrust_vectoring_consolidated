import QtQuick

// CRT scanline overlay: 1px black line every 3px, drawn once per resize.
// Non-interactive; place at the top of the z-order.
Canvas {
    id: canvas

    opacity: 0.14
    enabled: false

    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()

    onPaint: {
        const ctx = getContext("2d")
        ctx.reset()
        ctx.fillStyle = Qt.rgba(0, 0, 0, 0.7)
        for (let y = 0; y < height; y += 3)
            ctx.fillRect(0, y, width, 1)
    }
}

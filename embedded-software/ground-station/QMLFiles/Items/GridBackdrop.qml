import QtQuick

// Graph-grid backdrop: square cells of faint lines over an optional fill.
// Defaults match the page backdrop (32px, #ff5a3c at 5%); viewports reuse it
// with their own cell size / opacity.
Canvas {
    id: canvas

    property real cellSize: 32
    property color lineColor: Theme.gridLine
    property real lineOpacity: 0.05
    property color fillColor: "transparent"

    onCellSizeChanged: requestPaint()
    onLineColorChanged: requestPaint()
    onLineOpacityChanged: requestPaint()
    onFillColorChanged: requestPaint()
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()

    onPaint: {
        const ctx = getContext("2d")
        ctx.reset()
        if (fillColor.a > 0) {
            ctx.fillStyle = fillColor
            ctx.fillRect(0, 0, width, height)
        }
        ctx.strokeStyle = Qt.rgba(lineColor.r, lineColor.g, lineColor.b, lineOpacity)
        ctx.lineWidth = 1
        ctx.beginPath()
        for (let x = 0.5; x < width; x += cellSize) {
            ctx.moveTo(x, 0)
            ctx.lineTo(x, height)
        }
        for (let y = 0.5; y < height; y += cellSize) {
            ctx.moveTo(0, y)
            ctx.lineTo(width, y)
        }
        ctx.stroke()
    }
}

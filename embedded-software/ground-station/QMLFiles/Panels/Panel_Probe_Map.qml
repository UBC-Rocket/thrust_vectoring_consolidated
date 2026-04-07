import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Basic as Basic
import QtQuick.Layouts
import "../Items"

Rectangle {
    id: panel

    color: Theme.surface
    border.color: Theme.border
    border.width: Theme.strokePanel
    radius: Theme.radiusPanel

    // ── State ──────────────────────────────────────────────────────────────
    property real rectWidth:  0     // meters (full width)
    property real rectHeight: 0     // meters (full height)

    property real pxPerMeter: 40    // zoom level
    property real panX: 0           // pan offset, pixels
    property real panY: 0

    property bool dragging: false
    property real dragStartXm: 0
    property real dragStartYm: 0
    property real dragCurXm: 0
    property real dragCurYm: 0

    readonly property var poles: (rectWidth > 0 && rectHeight > 0)
        ? [
            { id: "P0", x: 0,             y: 0             },
            { id: "P1", x: -rectWidth/2,  y: -rectHeight/2 },
            { id: "P2", x:  rectWidth/2,  y: -rectHeight/2 },
            { id: "P3", x:  rectWidth/2,  y:  rectHeight/2 },
            { id: "P4", x: -rectWidth/2,  y:  rectHeight/2 }
          ]
        : []

    // ── Coordinate transforms ──────────────────────────────────────────────
    function mxToPx(xm) { return mapView.width  / 2 + panX + xm * pxPerMeter }
    function myToPy(ym) { return mapView.height / 2 + panY - ym * pxPerMeter }
    function pxToMx(px) { return (px - mapView.width  / 2 - panX) /  pxPerMeter }
    function pyToMy(py) { return (py - mapView.height / 2 - panY) / -pxPerMeter }

    function resetView() {
        pxPerMeter = 40
        panX = 0
        panY = 0
        gridLayer.requestPaint()
        geometryLayer.requestPaint()
    }

    function zoomAt(cx, cy, factor) {
        const sxm = pxToMx(cx)
        const sym = pyToMy(cy)
        pxPerMeter = Math.max(2, Math.min(2000, pxPerMeter * factor))
        panX += cx - mxToPx(sxm)
        panY += cy - myToPy(sym)
        gridLayer.requestPaint()
        geometryLayer.requestPaint()
    }

    // ── Header ─────────────────────────────────────────────────────────────
    BaseHeader {
        id: header
        headerText: "UWB Probe Layout"
    }

    // ── Top toolbar ────────────────────────────────────────────────────────
    RowLayout {
        id: toolbar
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.topMargin: 18
        anchors.rightMargin: 18
        spacing: 12

        Text {
            text: "W: " + panel.rectWidth.toFixed(2) + " m"
            color: Theme.textSecondary
            font.family: Theme.monoFamily
            font.pixelSize: Theme.fontBody
        }
        Text {
            text: "H: " + panel.rectHeight.toFixed(2) + " m"
            color: Theme.textSecondary
            font.family: Theme.monoFamily
            font.pixelSize: Theme.fontBody
        }
        Text {
            text: poles.length + " poles"
            color: Theme.textTertiary
            font.family: Theme.monoFamily
            font.pixelSize: Theme.fontCaption
        }

        Basic.Button {
            id: clearBtn
            text: "Clear"
            enabled: panel.poles.length > 0
            hoverEnabled: true
            padding: 8
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontBody
            background: Rectangle {
                radius: Theme.radiusControl
                color: !clearBtn.enabled ? Theme.surfaceInset
                     : clearBtn.down     ? Theme.btnSecondaryPress
                     : clearBtn.hovered  ? Theme.btnSecondaryHover
                     :                     Theme.btnSecondaryBg
                border.width: Theme.strokeControl
                border.color: Theme.btnSecondaryBorder
            }
            contentItem: Text {
                anchors.centerIn: parent
                text: clearBtn.text
                color: clearBtn.enabled ? Theme.btnSecondaryText : Theme.textTertiary
                font: clearBtn.font
            }
            onClicked: {
                panel.rectWidth = 0
                panel.rectHeight = 0
                geometryLayer.requestPaint()
            }
        }

        PrimaryButton {
            id: sendBtn
            text: "Send"
            padding: 8
            enabled: panel.poles.length > 0
            onClicked: {
                console.log("probe layout:", JSON.stringify(panel.poles))
                if (typeof commandsender !== "undefined" && commandsender
                    && typeof commandsender.sendProbeLayout === "function") {
                    commandsender.sendProbeLayout(panel.poles)
                }
            }
        }
    }

    // ── Map area ───────────────────────────────────────────────────────────
    Item {
        id: mapView
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.top: header.bottom
        anchors.topMargin: 12
        anchors.leftMargin: 12
        anchors.rightMargin: 12
        anchors.bottomMargin: 12
        clip: true

        Rectangle {
            anchors.fill: parent
            color: Theme.surfaceInset
            border.color: Theme.border
            border.width: Theme.strokeControl
            radius: Theme.radiusCard
        }

        // ── Grid + axes layer ──
        Canvas {
            id: gridLayer
            anchors.fill: parent
            renderStrategy: Canvas.Cooperative

            onWidthChanged:  requestPaint()
            onHeightChanged: requestPaint()

            function niceStep(targetPx) {
                const candidates = [0.05, 0.1, 0.2, 0.25, 0.5, 1, 2, 2.5, 5, 10, 20, 25, 50, 100, 200, 500]
                for (let i = 0; i < candidates.length; ++i) {
                    if (candidates[i] * panel.pxPerMeter >= targetPx) return candidates[i]
                }
                return candidates[candidates.length - 1]
            }

            onPaint: {
                const ctx = getContext("2d")
                ctx.reset()
                const w = width, h = height

                const stepM = niceStep(60)
                const stepPx = stepM * panel.pxPerMeter

                // Visible meter range
                const xmMin = panel.pxToMx(0)
                const xmMax = panel.pxToMx(w)
                const ymMin = panel.pyToMy(h)
                const ymMax = panel.pyToMy(0)

                // Minor grid
                ctx.strokeStyle = Theme.divider
                ctx.lineWidth = 1
                ctx.beginPath()
                let xStart = Math.ceil(xmMin / stepM) * stepM
                for (let xm = xStart; xm <= xmMax; xm += stepM) {
                    const px = Math.round(panel.mxToPx(xm)) + 0.5
                    ctx.moveTo(px, 0)
                    ctx.lineTo(px, h)
                }
                let yStart = Math.ceil(ymMin / stepM) * stepM
                for (let ym = yStart; ym <= ymMax; ym += stepM) {
                    const py = Math.round(panel.myToPy(ym)) + 0.5
                    ctx.moveTo(0, py)
                    ctx.lineTo(w, py)
                }
                ctx.stroke()

                // Axes
                ctx.strokeStyle = Theme.borderLight
                ctx.lineWidth = 1.5
                ctx.beginPath()
                const ax = Math.round(panel.mxToPx(0)) + 0.5
                const ay = Math.round(panel.myToPy(0)) + 0.5
                if (ax >= 0 && ax <= w) { ctx.moveTo(ax, 0); ctx.lineTo(ax, h) }
                if (ay >= 0 && ay <= h) { ctx.moveTo(0, ay); ctx.lineTo(w, ay) }
                ctx.stroke()

                // Tick labels (along bottom + left)
                ctx.fillStyle = Theme.textTertiary
                ctx.font = "10px " + Theme.monoFamily
                ctx.textBaseline = "bottom"
                ctx.textAlign = "left"
                for (let xm = xStart; xm <= xmMax; xm += stepM) {
                    if (Math.abs(xm) < stepM / 2) continue
                    const px = panel.mxToPx(xm)
                    ctx.fillText(xm.toFixed(stepM < 1 ? 2 : (stepM < 10 ? 1 : 0)) + "m", px + 3, h - 3)
                }
                ctx.textBaseline = "top"
                for (let ym = yStart; ym <= ymMax; ym += stepM) {
                    if (Math.abs(ym) < stepM / 2) continue
                    const py = panel.myToPy(ym)
                    ctx.fillText(ym.toFixed(stepM < 1 ? 2 : (stepM < 10 ? 1 : 0)) + "m", 3, py + 2)
                }
            }
        }

        // Repaint grid on view changes
        Connections {
            target: panel
            function onPxPerMeterChanged() { gridLayer.requestPaint() }
            function onPanXChanged()       { gridLayer.requestPaint() }
            function onPanYChanged()       { gridLayer.requestPaint() }
        }

        // ── Geometry layer (rectangle, dashed lines, length labels) ──
        Canvas {
            id: geometryLayer
            anchors.fill: parent
            renderStrategy: Canvas.Cooperative

            onWidthChanged:  requestPaint()
            onHeightChanged: requestPaint()

            function drawDashedLine(ctx, x1, y1, x2, y2) {
                ctx.beginPath()
                ctx.moveTo(x1, y1)
                ctx.lineTo(x2, y2)
                ctx.stroke()
            }

            function drawLengthLabel(ctx, x1, y1, x2, y2, lengthM) {
                const mx = (x1 + x2) / 2
                const my = (y1 + y2) / 2
                const label = lengthM.toFixed(2) + " m"
                ctx.font = "11px " + Theme.monoFamily
                const tw = ctx.measureText(label).width
                const padX = 5, padY = 2
                ctx.fillStyle = Theme.surface
                ctx.strokeStyle = Theme.border
                ctx.lineWidth = 1
                ctx.setLineDash([])
                const bx = mx - tw/2 - padX
                const by = my - 8 - padY
                const bw = tw + padX * 2
                const bh = 16 + padY * 2 - 4
                ctx.beginPath()
                ctx.rect(bx, by, bw, bh)
                ctx.fill()
                ctx.stroke()
                ctx.fillStyle = Theme.textPrimary
                ctx.textAlign = "center"
                ctx.textBaseline = "middle"
                ctx.fillText(label, mx, by + bh/2)
                ctx.setLineDash([6, 4])
            }

            onPaint: {
                const ctx = getContext("2d")
                ctx.reset()

                // Live preview while dragging
                if (panel.dragging) {
                    const w = 2 * Math.abs(panel.dragCurXm - panel.dragStartXm)
                    const h = 2 * Math.abs(panel.dragCurYm - panel.dragStartYm)
                    if (w > 0 && h > 0) {
                        const x1 = panel.mxToPx(-w/2)
                        const x2 = panel.mxToPx( w/2)
                        const y1 = panel.myToPy( h/2)
                        const y2 = panel.myToPy(-h/2)
                        ctx.strokeStyle = Theme.accent
                        ctx.lineWidth = 1
                        ctx.setLineDash([4, 4])
                        ctx.strokeRect(x1, y1, x2 - x1, y2 - y1)
                        ctx.setLineDash([])
                    }
                }

                if (panel.poles.length !== 5) return

                const p = panel.poles
                function pt(i) { return Qt.point(panel.mxToPx(p[i].x), panel.myToPy(p[i].y)) }
                const c  = pt(0)
                const c1 = pt(1)
                const c2 = pt(2)
                const c3 = pt(3)
                const c4 = pt(4)

                // Solid rectangle outline
                ctx.strokeStyle = Theme.accentMuted
                ctx.lineWidth = 1.5
                ctx.setLineDash([])
                ctx.beginPath()
                ctx.moveTo(c1.x, c1.y)
                ctx.lineTo(c2.x, c2.y)
                ctx.lineTo(c3.x, c3.y)
                ctx.lineTo(c4.x, c4.y)
                ctx.closePath()
                ctx.stroke()

                // Dashed lines: 4 perimeter + 4 spokes
                ctx.strokeStyle = Theme.accent
                ctx.lineWidth = 1.2
                ctx.setLineDash([6, 4])

                const w = panel.rectWidth
                const h = panel.rectHeight
                const diag = Math.sqrt((w/2)*(w/2) + (h/2)*(h/2))

                // Perimeter
                drawDashedLine(ctx, c1.x, c1.y, c2.x, c2.y)
                drawDashedLine(ctx, c2.x, c2.y, c3.x, c3.y)
                drawDashedLine(ctx, c3.x, c3.y, c4.x, c4.y)
                drawDashedLine(ctx, c4.x, c4.y, c1.x, c1.y)
                // Spokes
                drawDashedLine(ctx, c.x, c.y, c1.x, c1.y)
                drawDashedLine(ctx, c.x, c.y, c2.x, c2.y)
                drawDashedLine(ctx, c.x, c.y, c3.x, c3.y)
                drawDashedLine(ctx, c.x, c.y, c4.x, c4.y)

                // Length labels
                drawLengthLabel(ctx, c1.x, c1.y, c2.x, c2.y, w)        // bottom edge (in screen-y, but we labeled top in scene-y. doesn't matter — w is correct)
                drawLengthLabel(ctx, c3.x, c3.y, c4.x, c4.y, w)
                drawLengthLabel(ctx, c2.x, c2.y, c3.x, c3.y, h)
                drawLengthLabel(ctx, c4.x, c4.y, c1.x, c1.y, h)
                drawLengthLabel(ctx, c.x, c.y, c1.x, c1.y, diag)
                drawLengthLabel(ctx, c.x, c.y, c2.x, c2.y, diag)
                drawLengthLabel(ctx, c.x, c.y, c3.x, c3.y, diag)
                drawLengthLabel(ctx, c.x, c.y, c4.x, c4.y, diag)
            }
        }

        Connections {
            target: panel
            function onPxPerMeterChanged() { geometryLayer.requestPaint() }
            function onPanXChanged()       { geometryLayer.requestPaint() }
            function onPanYChanged()       { geometryLayer.requestPaint() }
            function onRectWidthChanged()  { geometryLayer.requestPaint() }
            function onRectHeightChanged() { geometryLayer.requestPaint() }
        }

        // ── Pole markers ──
        Repeater {
            model: panel.poles
            delegate: Item {
                required property var modelData
                x: panel.mxToPx(modelData.x) - 6
                y: panel.myToPy(modelData.y) - 6
                width: 12
                height: 12
                visible: panel.poles.length === 5
                z: 5

                Rectangle {
                    anchors.centerIn: parent
                    width: 12
                    height: 12
                    radius: 6
                    color: modelData.id === "P0" ? Theme.accent : Theme.successText
                    border.color: Theme.background
                    border.width: 2
                }
                Text {
                    anchors.left: parent.right
                    anchors.bottom: parent.top
                    anchors.leftMargin: 2
                    text: modelData.id + " (" + modelData.x.toFixed(2) + ", " + modelData.y.toFixed(2) + ")"
                    color: Theme.textSecondary
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontCaption
                }
            }
        }

        // ── Interaction (drag rectangle, pan, wheel zoom) ──
        MouseArea {
            id: interaction
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
            hoverEnabled: false

            property bool panning: false
            property real lastX: 0
            property real lastY: 0

            onPressed: (mouse) => {
                if (mouse.button === Qt.LeftButton) {
                    panel.dragging = true
                    panel.dragStartXm = panel.pxToMx(mouse.x)
                    panel.dragStartYm = panel.pyToMy(mouse.y)
                    panel.dragCurXm = panel.dragStartXm
                    panel.dragCurYm = panel.dragStartYm
                    geometryLayer.requestPaint()
                } else {
                    panning = true
                    lastX = mouse.x
                    lastY = mouse.y
                }
            }

            onPositionChanged: (mouse) => {
                if (panel.dragging) {
                    panel.dragCurXm = panel.pxToMx(mouse.x)
                    panel.dragCurYm = panel.pyToMy(mouse.y)
                    geometryLayer.requestPaint()
                } else if (panning) {
                    panel.panX += mouse.x - lastX
                    panel.panY += mouse.y - lastY
                    lastX = mouse.x
                    lastY = mouse.y
                }
            }

            onReleased: (mouse) => {
                if (panel.dragging && mouse.button === Qt.LeftButton) {
                    const w = 2 * Math.abs(panel.dragCurXm - panel.dragStartXm)
                    const h = 2 * Math.abs(panel.dragCurYm - panel.dragStartYm)
                    if (w > 0.01 && h > 0.01) {
                        panel.rectWidth = w
                        panel.rectHeight = h
                    }
                    panel.dragging = false
                    geometryLayer.requestPaint()
                }
                panning = false
            }

            onWheel: (wheel) => {
                const factor = wheel.angleDelta.y > 0 ? 1.1 : 1/1.1
                panel.zoomAt(wheel.x, wheel.y, factor)
            }
        }

        // ── Zoom controls (bottom-right) ──
        ColumnLayout {
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            anchors.margins: 12
            spacing: 6

            Basic.Button {
                id: zoomInBtn
                text: "+"
                Layout.preferredWidth: 32
                Layout.preferredHeight: 28
                hoverEnabled: true
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontBody
                background: Rectangle {
                    radius: Theme.radiusControl
                    color: zoomInBtn.down    ? Theme.btnSecondaryPress
                         : zoomInBtn.hovered ? Theme.btnSecondaryHover
                         :                     Theme.btnSecondaryBg
                    border.width: Theme.strokeControl
                    border.color: Theme.btnSecondaryBorder
                }
                contentItem: Text {
                    anchors.centerIn: parent
                    text: zoomInBtn.text
                    color: Theme.btnSecondaryText
                    font: zoomInBtn.font
                }
                onClicked: panel.zoomAt(mapView.width/2, mapView.height/2, 1.2)
            }
            Basic.Button {
                id: zoomOutBtn
                text: "−"
                Layout.preferredWidth: 32
                Layout.preferredHeight: 28
                hoverEnabled: true
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontBody
                background: Rectangle {
                    radius: Theme.radiusControl
                    color: zoomOutBtn.down    ? Theme.btnSecondaryPress
                         : zoomOutBtn.hovered ? Theme.btnSecondaryHover
                         :                      Theme.btnSecondaryBg
                    border.width: Theme.strokeControl
                    border.color: Theme.btnSecondaryBorder
                }
                contentItem: Text {
                    anchors.centerIn: parent
                    text: zoomOutBtn.text
                    color: Theme.btnSecondaryText
                    font: zoomOutBtn.font
                }
                onClicked: panel.zoomAt(mapView.width/2, mapView.height/2, 1/1.2)
            }
            Basic.Button {
                id: resetBtn
                text: "⟲"
                Layout.preferredWidth: 32
                Layout.preferredHeight: 28
                hoverEnabled: true
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontBody
                background: Rectangle {
                    radius: Theme.radiusControl
                    color: resetBtn.down    ? Theme.btnSecondaryPress
                         : resetBtn.hovered ? Theme.btnSecondaryHover
                         :                    Theme.btnSecondaryBg
                    border.width: Theme.strokeControl
                    border.color: Theme.btnSecondaryBorder
                }
                contentItem: Text {
                    anchors.centerIn: parent
                    text: resetBtn.text
                    color: Theme.btnSecondaryText
                    font: resetBtn.font
                }
                onClicked: panel.resetView()
            }
        }

        // ── Legend (bottom-left) ──
        Rectangle {
            anchors.left: parent.left
            anchors.bottom: parent.bottom
            anchors.margins: 12
            color: Theme.surface
            border.color: Theme.border
            border.width: Theme.strokeControl
            radius: Theme.radiusCard
            width: legendCol.implicitWidth + 16
            height: legendCol.implicitHeight + 12

            ColumnLayout {
                id: legendCol
                anchors.centerIn: parent
                spacing: 4

                Text {
                    text: "LEGEND"
                    color: Theme.textTertiary
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontCaption
                    font.bold: true
                }
                RowLayout {
                    spacing: 6
                    Rectangle { width: 10; height: 10; radius: 5; color: Theme.accent }
                    Text {
                        text: "Center pole (0,0)"
                        color: Theme.textSecondary
                        font.family: Theme.fontFamily
                        font.pixelSize: Theme.fontCaption
                    }
                }
                RowLayout {
                    spacing: 6
                    Rectangle { width: 10; height: 10; radius: 5; color: Theme.successText }
                    Text {
                        text: "Corner pole"
                        color: Theme.textSecondary
                        font.family: Theme.fontFamily
                        font.pixelSize: Theme.fontCaption
                    }
                }
                Text {
                    text: "Scale: " + panel.pxPerMeter.toFixed(0) + " px/m"
                    color: Theme.textTertiary
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontCaption
                }
                Text {
                    text: "Drag: rectangle  •  Right-drag: pan  •  Wheel: zoom"
                    color: Theme.textTertiary
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontCaption
                }
            }
        }
    }
}

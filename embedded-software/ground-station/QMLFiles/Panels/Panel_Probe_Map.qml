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

    // ── OSM map state ──────────────────────────────────────────────────────
    property bool   mapEnabled:  false
    property real   refLat:      0.0
    property real   refLon:      0.0
    property bool   refValid:    false  // true once user commits valid lat/lon
    property real   mapOpacity:  0.85   // tile opacity (keep dark theme feel)
    property var    visibleTiles: []

    // ── Poles (mutable) ───────────────────────────────────────────────────
    // Each pole: { id: "Pn", x: float, y: float, isBase: bool }
    // P0 = center, P1-P4 = corners (isBase: true), P5+ = user-placed extras
    property var poles: []

    function recomputeBasePoles() {
        if (rectWidth <= 0 || rectHeight <= 0) {
            poles = []
            return
        }
        const base = [
            { id: "P0", x: 0,            y: 0,             isBase: true },
            { id: "P1", x: -rectWidth/2, y: -rectHeight/2, isBase: true },
            { id: "P2", x:  rectWidth/2, y: -rectHeight/2, isBase: true },
            { id: "P3", x:  rectWidth/2, y:  rectHeight/2, isBase: true },
            { id: "P4", x: -rectWidth/2, y:  rectHeight/2, isBase: true }
        ]
        const extras = poles.length > 5 ? poles.slice(5) : []
        poles = base.concat(extras)
    }

    function movePole(index, newXm, newYm) {
        const arr = poles.slice()
        arr[index] = { id: arr[index].id, x: newXm, y: newYm, isBase: arr[index].isBase }
        poles = arr
    }

    function addPole(xm, ym) {
        if (poles.length >= 15) return
        const arr = poles.slice()
        arr.push({ id: "P" + arr.length, x: xm, y: ym, isBase: false })
        poles = arr
    }

    function removePole(index) {
        if (index < 5) return  // cannot remove base poles
        const arr = poles.slice()
        arr.splice(index, 1)
        // re-label extras
        for (let i = 5; i < arr.length; i++)
            arr[i] = { id: "P" + i, x: arr[i].x, y: arr[i].y, isBase: false }
        poles = arr
    }

    onRectWidthChanged:  recomputeBasePoles()
    onRectHeightChanged: recomputeBasePoles()

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

    // ── OSM tile helpers ───────────────────────────────────────────────────
    function osmZoomLevel() {
        const earthCircum = 2 * Math.PI * 6378137
        const metersPerTile = 256.0 / pxPerMeter
        const z = Math.log2(earthCircum / metersPerTile)
        return Math.max(0, Math.min(19, Math.round(z)))
    }

    function computeVisibleTiles() {
        if (!mapEnabled || !refValid) return []
        if (mapView.width <= 0 || mapView.height <= 0) return []

        const z = osmZoomLevel()
        const n = Math.pow(2, z)
        const earthCircum = 2.0 * Math.PI * 6378137.0

        // Reference point in tile float coordinates
        const latRad = refLat * Math.PI / 180.0
        const refTileX = (refLon + 180.0) / 360.0 * n
        const refTileY = (1.0 - Math.log(Math.tan(latRad) + 1.0 / Math.cos(latRad)) / Math.PI) / 2.0 * n

        // Probe origin (0,0) pixel position on canvas
        const originPx = mapView.width  / 2.0 + panX
        const originPy = mapView.height / 2.0 + panY

        // Tile size in canvas pixels
        const tileMeters = earthCircum / n
        const tilePx = tileMeters * pxPerMeter

        // Where the reference tile's top-left sits in canvas pixels
        const refTileFracX = refTileX - Math.floor(refTileX)
        const refTileFracY = refTileY - Math.floor(refTileY)
        const refTileOriginPx = originPx - refTileFracX * tilePx
        const refTileOriginPy = originPy - refTileFracY * tilePx

        const refTileIntX = Math.floor(refTileX)
        const refTileIntY = Math.floor(refTileY)

        // Tile range to cover viewport (with margin)
        const startDx = -Math.ceil((refTileOriginPx) / tilePx) - 1
        const startDy = -Math.ceil((refTileOriginPy) / tilePx) - 1
        const endDx = Math.ceil((mapView.width  - refTileOriginPx) / tilePx) + 1
        const endDy = Math.ceil((mapView.height - refTileOriginPy) / tilePx) + 1

        const tiles = []
        for (let dy = startDy; dy <= endDy; dy++) {
            for (let dx = startDx; dx <= endDx; dx++) {
                const tx = ((refTileIntX + dx) % n + n) % n  // wrap X
                const ty = refTileIntY + dy
                if (ty < 0 || ty >= n) continue               // clamp poles

                const px = refTileOriginPx + dx * tilePx
                const py = refTileOriginPy + dy * tilePx

                // Cull off-screen tiles
                if (px + tilePx < 0 || py + tilePx < 0) continue
                if (px > mapView.width || py > mapView.height) continue

                tiles.push({ z: z, x: tx, y: ty, px: px, py: py, size: tilePx })
            }
        }
        return tiles
    }

    function refreshTiles() {
        visibleTiles = computeVisibleTiles()
    }

    // ── Header ─────────────────────────────────────────────────────────────
    BaseHeader {
        id: header
        headerText: "UWB Probe Layout"
    }

    // ── Top toolbar (probe controls) ──────────────────────────────────────
    RowLayout {
        id: toolbar
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.topMargin: 18
        anchors.rightMargin: 18
        spacing: 12
        z: 30

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
            text: panel.poles.length + " poles"
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
                panel.poles = []
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

        onWidthChanged:  panel.refreshTiles()
        onHeightChanged: panel.refreshTiles()

        // ── Background (hidden when map tiles are active) ──
        Rectangle {
            id: mapBackground
            anchors.fill: parent
            color: Theme.surfaceInset
            border.color: Theme.border
            border.width: Theme.strokeControl
            radius: Theme.radiusCard
            visible: !panel.mapEnabled || !panel.refValid
        }

        // ── Satellite tile layer (Esri World Imagery) ──
        Repeater {
            id: tileLayer
            model: panel.visibleTiles
            z: 1

            delegate: Image {
                required property var modelData

                x: modelData.px
                y: modelData.py
                width:  modelData.size
                height: modelData.size

                source: panel.mapEnabled && panel.refValid
                        ? "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/" + modelData.z + "/" + modelData.y + "/" + modelData.x
                        : ""

                smooth: true
                fillMode: Image.Stretch
                opacity: status === Image.Ready ? panel.mapOpacity : 0
                Behavior on opacity { NumberAnimation { duration: 200 } }

                // Loading placeholder
                Rectangle {
                    anchors.fill: parent
                    color: "#0D1520"
                    border.color: Theme.border
                    border.width: 0.5
                    visible: parent.status !== Image.Ready && panel.mapEnabled && panel.refValid
                }
            }
        }

        // ── Map controls bar (top-center, inside map area) ──
        Rectangle {
            id: mapControlBar
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            anchors.topMargin: 8
            width: mapBarRow.implicitWidth + 16
            height: mapBarRow.implicitHeight + 8
            color: Theme.surface
            opacity: 0.92
            border.color: Theme.border
            border.width: Theme.strokeControl
            radius: Theme.radiusCard
            z: 25

            RowLayout {
                id: mapBarRow
                anchors.centerIn: parent
                spacing: 8

                ThemedCheckBox {
                    id: mapToggle
                    text: "Map"
                    checked: panel.mapEnabled
                    onCheckedChanged: panel.mapEnabled = checked
                }

                Text {
                    text: "Lat"
                    color: Theme.textTertiary
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontCaption
                }
                Basic.TextField {
                    id: latField
                    implicitWidth: 110
                    text: panel.refValid ? panel.refLat.toFixed(6) : ""
                    placeholderText: "49.000000"
                    color: Theme.textPrimary
                    placeholderTextColor: Theme.textTertiary
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontBody
                    inputMethodHints: Qt.ImhFormattedNumbersOnly
                    validator: DoubleValidator { bottom: -90; top: 90; decimals: 8 }
                    padding: 4
                    background: Rectangle {
                        radius: Theme.radiusControl
                        color: Theme.surfaceInset
                        border.width: Theme.strokeControl
                        border.color: latField.activeFocus ? Theme.accent : Theme.border
                    }
                    onEditingFinished: {
                        if (acceptableInput && lonField.acceptableInput) {
                            panel.refLat   = parseFloat(text)
                            panel.refLon   = parseFloat(lonField.text)
                            panel.refValid = true
                            panel.refreshTiles()
                        }
                    }
                }

                Text {
                    text: "Lon"
                    color: Theme.textTertiary
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontCaption
                }
                Basic.TextField {
                    id: lonField
                    implicitWidth: 120
                    text: panel.refValid ? panel.refLon.toFixed(6) : ""
                    placeholderText: "-123.000000"
                    color: Theme.textPrimary
                    placeholderTextColor: Theme.textTertiary
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontBody
                    inputMethodHints: Qt.ImhFormattedNumbersOnly
                    validator: DoubleValidator { bottom: -180; top: 180; decimals: 8 }
                    padding: 4
                    background: Rectangle {
                        radius: Theme.radiusControl
                        color: Theme.surfaceInset
                        border.width: Theme.strokeControl
                        border.color: lonField.activeFocus ? Theme.accent : Theme.border
                    }
                    onEditingFinished: {
                        if (acceptableInput && latField.acceptableInput) {
                            panel.refLat   = parseFloat(latField.text)
                            panel.refLon   = parseFloat(text)
                            panel.refValid = true
                            panel.refreshTiles()
                        }
                    }
                }
            }
        }

        // ── Grid + axes layer ──
        Canvas {
            id: gridLayer
            anchors.fill: parent
            renderStrategy: Canvas.Cooperative
            z: 2

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

                // Minor grid (darker when satellite map is active)
                const mapOn = panel.mapEnabled && panel.refValid
                ctx.strokeStyle = mapOn ? "rgba(0,0,0,0.35)" : Theme.divider
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
                ctx.strokeStyle = mapOn ? "rgba(0,0,0,0.5)" : Theme.borderLight
                ctx.lineWidth = 1.5
                ctx.beginPath()
                const ax = Math.round(panel.mxToPx(0)) + 0.5
                const ay = Math.round(panel.myToPy(0)) + 0.5
                if (ax >= 0 && ax <= w) { ctx.moveTo(ax, 0); ctx.lineTo(ax, h) }
                if (ay >= 0 && ay <= h) { ctx.moveTo(0, ay); ctx.lineTo(w, ay) }
                ctx.stroke()

                // Tick labels (along bottom + left)
                ctx.fillStyle = mapOn ? "rgba(255,255,255,0.5)" : Theme.textTertiary
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
            function onPxPerMeterChanged() { gridLayer.requestPaint(); panel.refreshTiles() }
            function onPanXChanged()       { gridLayer.requestPaint(); panel.refreshTiles() }
            function onPanYChanged()       { gridLayer.requestPaint(); panel.refreshTiles() }
        }

        // ── Geometry layer (rectangle, dashed lines, length labels) ──
        Canvas {
            id: geometryLayer
            anchors.fill: parent
            renderStrategy: Canvas.Cooperative
            z: 3

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

                if (panel.poles.length < 5) return

                const p = panel.poles
                function ptOf(pole) { return Qt.point(panel.mxToPx(pole.x), panel.myToPy(pole.y)) }

                const c  = ptOf(p[0])  // center
                const c1 = ptOf(p[1])  // P1 corner
                const c2 = ptOf(p[2])  // P2 corner
                const c3 = ptOf(p[3])  // P3 corner
                const c4 = ptOf(p[4])  // P4 corner

                // Solid rectangle outline (P1-P4)
                const mapOn = panel.mapEnabled && panel.refValid
                ctx.strokeStyle = mapOn ? "rgba(20,60,90,0.8)" : Theme.accentMuted
                ctx.lineWidth = 1.5
                ctx.setLineDash([])
                ctx.beginPath()
                ctx.moveTo(c1.x, c1.y)
                ctx.lineTo(c2.x, c2.y)
                ctx.lineTo(c3.x, c3.y)
                ctx.lineTo(c4.x, c4.y)
                ctx.closePath()
                ctx.stroke()

                // Dashed lines
                ctx.strokeStyle = mapOn ? "rgba(30,80,120,0.7)" : Theme.accent
                ctx.lineWidth = 1.2
                ctx.setLineDash([6, 4])

                const w = panel.rectWidth
                const h = panel.rectHeight

                // Perimeter edges
                drawDashedLine(ctx, c1.x, c1.y, c2.x, c2.y)
                drawDashedLine(ctx, c2.x, c2.y, c3.x, c3.y)
                drawDashedLine(ctx, c3.x, c3.y, c4.x, c4.y)
                drawDashedLine(ctx, c4.x, c4.y, c1.x, c1.y)

                // Spokes from center to ALL poles
                for (let i = 1; i < p.length; ++i) {
                    const pi = ptOf(p[i])
                    drawDashedLine(ctx, c.x, c.y, pi.x, pi.y)
                }

                // Perimeter length labels
                drawLengthLabel(ctx, c1.x, c1.y, c2.x, c2.y, w)
                drawLengthLabel(ctx, c3.x, c3.y, c4.x, c4.y, w)
                drawLengthLabel(ctx, c2.x, c2.y, c3.x, c3.y, h)
                drawLengthLabel(ctx, c4.x, c4.y, c1.x, c1.y, h)

                // Spoke distance labels
                for (let i = 1; i < p.length; ++i) {
                    const pi = ptOf(p[i])
                    const dist = Math.sqrt(p[i].x * p[i].x + p[i].y * p[i].y)
                    drawLengthLabel(ctx, c.x, c.y, pi.x, pi.y, dist)
                }
            }
        }

        Connections {
            target: panel
            function onPxPerMeterChanged() { geometryLayer.requestPaint() }
            function onPanXChanged()       { geometryLayer.requestPaint() }
            function onPanYChanged()       { geometryLayer.requestPaint() }
            function onRectWidthChanged()  { geometryLayer.requestPaint() }
            function onRectHeightChanged() { geometryLayer.requestPaint() }
            function onPolesChanged()      { geometryLayer.requestPaint() }
        }

        // Refresh tiles and repaint overlays on map state changes
        Connections {
            target: panel
            function onMapEnabledChanged() { panel.refreshTiles(); gridLayer.requestPaint(); geometryLayer.requestPaint() }
            function onRefValidChanged()   { panel.refreshTiles(); gridLayer.requestPaint(); geometryLayer.requestPaint() }
        }

        // ── Pole markers (draggable) ──
        Repeater {
            id: poleRepeater
            model: panel.poles
            delegate: Item {
                id: markerItem
                required property var modelData
                required property int index

                x: panel.mxToPx(modelData.x) - hitSize/2
                y: panel.myToPy(modelData.y) - hitSize/2

                property int hitSize: 20
                width: hitSize
                height: hitSize
                visible: panel.poles.length >= 5
                z: 10

                Rectangle {
                    anchors.centerIn: parent
                    width: 12
                    height: 12
                    radius: 6
                    property bool mapOn: panel.mapEnabled && panel.refValid
                    color: modelData.id === "P0" ? (mapOn ? "#1a5a80" : Theme.accent)
                         : modelData.isBase      ? (mapOn ? "#2a6648" : Theme.successText)
                         :                         (mapOn ? "#8a6a20" : Theme.warnText)
                    border.color: mapOn ? "#111" : Theme.background
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

                MouseArea {
                    id: markerDragArea
                    anchors.fill: parent
                    anchors.margins: -4
                    hoverEnabled: true
                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                    cursorShape: containsMouse ? Qt.PointingHandCursor : Qt.ArrowCursor

                    property bool isDragging: false
                    property real offsetXm: 0
                    property real offsetYm: 0

                    onPressed: (mouse) => {
                        if (mouse.button === Qt.RightButton) {
                            if (!markerItem.modelData.isBase) {
                                panel.removePole(markerItem.index)
                            }
                            mouse.accepted = true
                            return
                        }
                        // Start drag
                        isDragging = true
                        const clickXm = panel.pxToMx(mouse.x + markerItem.x + markerItem.hitSize/2)
                        const clickYm = panel.pyToMy(mouse.y + markerItem.y + markerItem.hitSize/2)
                        offsetXm = markerItem.modelData.x - clickXm
                        offsetYm = markerItem.modelData.y - clickYm
                        mouse.accepted = true
                    }

                    onPositionChanged: (mouse) => {
                        if (!isDragging) return
                        const newXm = panel.pxToMx(mouse.x + markerItem.x + markerItem.hitSize/2) + offsetXm
                        const newYm = panel.pyToMy(mouse.y + markerItem.y + markerItem.hitSize/2) + offsetYm
                        panel.movePole(markerItem.index, newXm, newYm)
                    }

                    onReleased: (mouse) => {
                        isDragging = false
                        mouse.accepted = true
                    }
                }
            }
        }

        // ── Interaction (drag rectangle, pan, wheel zoom, click-to-add) ──
        MouseArea {
            id: interaction
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
            hoverEnabled: false
            z: 0

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
                    const dxm = Math.abs(panel.dragCurXm - panel.dragStartXm)
                    const dym = Math.abs(panel.dragCurYm - panel.dragStartYm)
                    const isClick = dxm < 0.05 && dym < 0.05

                    if (isClick && panel.poles.length >= 5 && panel.poles.length < 15) {
                        // Click-to-add extra probe
                        panel.addPole(panel.dragStartXm, panel.dragStartYm)
                    } else if (!isClick) {
                        const w = 2 * dxm
                        const h = 2 * dym
                        if (w > 0.01 && h > 0.01) {
                            panel.rectWidth = w
                            panel.rectHeight = h
                        }
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

        // ── OSM Attribution ──
        Rectangle {
            id: osmAttribution
            anchors.right: parent.right
            anchors.bottom: zoomControls.top
            anchors.rightMargin: 12
            anchors.bottomMargin: 4
            width: osmText.implicitWidth + 6
            height: osmText.implicitHeight + 4
            color: Theme.surface
            opacity: 0.8
            radius: 3
            visible: panel.mapEnabled && panel.refValid
            z: 20

            Text {
                id: osmText
                anchors.centerIn: parent
                text: "\u00A9 Esri World Imagery"
                color: Theme.textTertiary
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontCaption
            }
        }

        // ── Zoom controls (bottom-right) ──
        ColumnLayout {
            id: zoomControls
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            anchors.margins: 12
            spacing: 6
            z: 20

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
            z: 20

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
                RowLayout {
                    spacing: 6
                    Rectangle { width: 10; height: 10; radius: 5; color: Theme.warnText }
                    Text {
                        text: "Extra probe (draggable)"
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
                    text: "Drag: rectangle  •  Click: add probe  •  Right-click: delete"
                    color: Theme.textTertiary
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontCaption
                }
                Text {
                    text: "Right-drag: pan  •  Wheel: zoom  •  Max 15 probes"
                    color: Theme.textTertiary
                    font.family: Theme.fontFamily
                    font.pixelSize: Theme.fontCaption
                }
            }
        }
    }
}

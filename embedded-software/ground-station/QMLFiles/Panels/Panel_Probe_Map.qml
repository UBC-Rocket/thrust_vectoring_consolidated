import QtQuick
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

    // ── Anchors ───────────────────────────────────────────────────────────
    // Exactly 4 ground UWB anchors forming a rectangle.
    // No center pole — the rocket's takeoff origin (0,0) is implied, not anchored.
    // poles[0..3] map directly to SetProbeLayout.anchor_0..anchor_3 on the wire.
    property var poles: []

    // Corner currently being dragged. While a drag is in progress we deliberately
    // do NOT rewrite `poles` — that would reset the pole Repeater's model and
    // destroy the MouseArea mid-drag. Instead we draw the live position from these
    // props and commit it back into `poles` once, on release (see commitPole()).
    property int  activePoleIndex: -1
    property real activePoleXm: 0
    property real activePoleYm: 0

    function recomputeBasePoles() {
        if (rectWidth <= 0 || rectHeight <= 0) {
            poles = []
            return
        }
        poles = [
            { id: "A0", x: -rectWidth/2, y: -rectHeight/2, isBase: true },
            { id: "A1", x:  rectWidth/2, y: -rectHeight/2, isBase: true },
            { id: "A2", x:  rectWidth/2, y:  rectHeight/2, isBase: true },
            { id: "A3", x: -rectWidth/2, y:  rectHeight/2, isBase: true }
        ]
    }

    onRectWidthChanged:  recomputeBasePoles()
    onRectHeightChanged: recomputeBasePoles()

    // Live update for the corner being dragged. Records the position and repaints
    // the geometry overlay, but leaves `poles` untouched until commitPole().
    function movePole(index, xm, ym) {
        if (index < 0 || index >= poles.length) return
        activePoleIndex = index
        activePoleXm = xm
        activePoleYm = ym
        geometryLayer.requestPaint()
    }

    // Write the dragged corner back into `poles` (single reassign -> one repaint +
    // Repeater refresh). Flips isBase=false: the corner is no longer a pristine
    // rectangle vertex, it's been nudged to match the real-world anchor position.
    function commitPole() {
        const i = activePoleIndex
        if (i < 0 || i >= poles.length) { activePoleIndex = -1; return }
        var arr = poles.slice()
        var pole = {}
        for (var k in arr[i]) pole[k] = arr[i][k]
        pole.x = activePoleXm
        pole.y = activePoleYm
        pole.isBase = false
        arr[i] = pole
        poles = arr
        activePoleIndex = -1
    }

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

    // ── EVA panel top accent ───────────────────────────────────────────────
    Rectangle {
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: Theme.accentStroke
        color: Theme.accent
    }

    // ── Header ─────────────────────────────────────────────────────────────
    BaseHeader {
        id: header
        headerText: "UWB Probe Layout"
        jpText: "探査配置"
    }

    // ── Top toolbar (probe controls) ──────────────────────────────────────
    RowLayout {
        id: toolbar
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.topMargin: 18
        anchors.rightMargin: 18
        spacing: 14
        z: 30

        Text {
            text: "W: " + panel.rectWidth.toFixed(2) + " m"
            color: Theme.textSecondary
            font.family: Theme.monoFamily
            font.pixelSize: 12
        }
        Text {
            text: "H: " + panel.rectHeight.toFixed(2) + " m"
            color: Theme.textSecondary
            font.family: Theme.monoFamily
            font.pixelSize: 12
        }
        Text {
            text: panel.poles.length + "/4 ANCHORS"
            color: Theme.amber
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontCaption
            font.letterSpacing: 2
        }

        ClippedButton {
            id: clearBtn
            text: "CLEAR"
            cut: 0
            bold: false
            fontSize: 10
            fontLetterSpacing: 2
            horizontalPadding: 14
            verticalPadding: 5
            fillColor: "transparent"
            hoverFillColor: "transparent"
            borderColor: Theme.border
            hoverBorderColor: Theme.accent
            textColor: Theme.textSecondary
            hoverTextColor: Theme.accentMuted
            enabled: panel.poles.length > 0
            onClicked: {
                panel.rectWidth = 0
                panel.rectHeight = 0
                panel.poles = []
                geometryLayer.requestPaint()
            }
        }

        ClippedButton {
            id: sendBtn
            text: "SEND ▸"
            cut: 8
            bold: true
            fontSize: 10
            fontLetterSpacing: 2
            horizontalPadding: 18
            verticalPadding: 5
            fillColor: Theme.accentTint
            hoverFillColor: Theme.accentTintHover
            borderColor: Theme.accent
            hoverBorderColor: Theme.accent
            textColor: Theme.accentMuted
            hoverTextColor: Theme.accentMuted
            enabled: panel.poles.length === 4
            onClicked: {
                console.log("probe layout:", JSON.stringify(panel.poles))
                if (typeof commandsender !== "undefined" && commandsender
                    && typeof commandsender.sendProbeLayout === "function") {
                    commandsender.sendProbeLayout(panel.poles)
                }
            }
        }
    }

    // ── Controls bar (satellite toggle + lat/lon + hints) ─────────────────
    Rectangle {
        id: controlsBar
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.leftMargin: 12
        anchors.rightMargin: 12
        height: mapBarRow.implicitHeight + 14
        color: Theme.surfaceElevated
        border.color: Theme.divider
        border.width: Theme.strokeControl
        radius: 0

        RowLayout {
            id: mapBarRow
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.verticalCenter: parent.verticalCenter
            anchors.leftMargin: 12
            anchors.rightMargin: 12
            spacing: 12

            ThemedCheckBox {
                id: mapToggle
                text: "SATELLITE LAYER"
                font.pixelSize: Theme.fontCaption
                font.letterSpacing: 2
                checked: panel.mapEnabled
                onCheckedChanged: panel.mapEnabled = checked
                contentItem: Text {
                    text: mapToggle.text
                    font: mapToggle.font
                    color: Theme.amber
                    verticalAlignment: Text.AlignVCenter
                    leftPadding: mapToggle.indicator.width + mapToggle.spacing
                }
            }

            Rectangle {
                Layout.preferredWidth: 1
                Layout.preferredHeight: 18
                color: Theme.divider
            }

            Text {
                text: "LAT"
                color: Theme.accentMuted
                font.family: Theme.fontFamily
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
                    radius: 0
                    color: Theme.sceneBackground
                    border.width: Theme.strokeControl
                    border.color: latField.activeFocus ? Theme.accent : Theme.divider
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
                text: "LON"
                color: Theme.accentMuted
                font.family: Theme.fontFamily
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
                    radius: 0
                    color: Theme.sceneBackground
                    border.width: Theme.strokeControl
                    border.color: lonField.activeFocus ? Theme.accent : Theme.divider
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

            Item { Layout.fillWidth: true }

            Text {
                text: "SCALE " + panel.pxPerMeter.toFixed(0) + " px/m · DRAG: RECTANGLE · RIGHT-DRAG: PAN · WHEEL: ZOOM"
                color: Theme.textTertiary
                font.family: Theme.monoFamily
                font.pixelSize: 10
                elide: Text.ElideRight
                Layout.maximumWidth: implicitWidth
            }
        }
    }

    // ── Map area ───────────────────────────────────────────────────────────
    Item {
        id: mapView
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.top: controlsBar.bottom
        anchors.topMargin: 12
        anchors.leftMargin: 12
        anchors.rightMargin: 12
        anchors.bottomMargin: 12
        clip: true

        onWidthChanged:  panel.refreshTiles()
        onHeightChanged: panel.refreshTiles()

        // ── Deep viewport background (tiles fade in above it) ──
        Rectangle {
            id: mapBackground
            anchors.fill: parent
            color: Theme.sceneBackground
            border.color: Theme.divider
            border.width: Theme.strokeControl
            radius: 0
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
                    color: Theme.sceneBackground
                    border.color: Theme.divider
                    border.width: 0.5
                    visible: parent.status !== Image.Ready && panel.mapEnabled && panel.refValid
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

                // Minor grid: red meter grid @10% (darker when satellite map is active)
                const mapOn = panel.mapEnabled && panel.refValid
                ctx.strokeStyle = mapOn ? "rgba(0,0,0,0.35)" : "rgba(255,90,60,0.10)"
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

                // Axes (amber crosshair through the takeoff origin)
                ctx.strokeStyle = mapOn ? "rgba(0,0,0,0.5)" : "rgba(255,180,0,0.4)"
                ctx.lineWidth = 1.5
                ctx.beginPath()
                const ax = Math.round(panel.mxToPx(0)) + 0.5
                const ay = Math.round(panel.myToPy(0)) + 0.5
                if (ax >= 0 && ax <= w) { ctx.moveTo(ax, 0); ctx.lineTo(ax, h) }
                if (ay >= 0 && ay <= h) { ctx.moveTo(0, ay); ctx.lineTo(w, ay) }
                ctx.stroke()

                // Tick labels (along bottom + left)
                ctx.fillStyle = mapOn ? "rgba(255,255,255,0.5)" : Theme.textTertiary
                ctx.font = "10px \"" + Theme.monoFamily + "\""
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
                ctx.font = "11px \"" + Theme.monoFamily + "\""
                const tw = ctx.measureText(label).width
                const padX = 5, padY = 2
                ctx.fillStyle = Theme.surfaceElevated
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

                if (panel.poles.length < 4) return

                const p = panel.poles
                // Live position of corner i: while it's being dragged, read from
                // the active* props (poles isn't rewritten until release).
                function poleXm(i) { return panel.activePoleIndex === i ? panel.activePoleXm : p[i].x }
                function poleYm(i) { return panel.activePoleIndex === i ? panel.activePoleYm : p[i].y }
                function ptOf(i)   { return Qt.point(panel.mxToPx(poleXm(i)), panel.myToPy(poleYm(i))) }
                function edgeLenM(i, j) { return Math.hypot(poleXm(i) - poleXm(j), poleYm(i) - poleYm(j)) }

                const c0 = ptOf(0)  // anchor 0 (bottom-left)
                const c1 = ptOf(1)  // anchor 1 (bottom-right)
                const c2 = ptOf(2)  // anchor 2 (top-right)
                const c3 = ptOf(3)  // anchor 3 (top-left)

                // Solid rectangle outline (4 anchors)
                const mapOn = panel.mapEnabled && panel.refValid
                ctx.strokeStyle = mapOn ? "rgba(20,60,90,0.8)" : Theme.accentMuted
                ctx.lineWidth = 1.5
                ctx.setLineDash([])
                ctx.beginPath()
                ctx.moveTo(c0.x, c0.y)
                ctx.lineTo(c1.x, c1.y)
                ctx.lineTo(c2.x, c2.y)
                ctx.lineTo(c3.x, c3.y)
                ctx.closePath()
                ctx.stroke()

                // Dashed perimeter
                ctx.strokeStyle = mapOn ? "rgba(30,80,120,0.7)" : Theme.accent
                ctx.lineWidth = 1.2
                ctx.setLineDash([6, 4])

                drawDashedLine(ctx, c0.x, c0.y, c1.x, c1.y)
                drawDashedLine(ctx, c1.x, c1.y, c2.x, c2.y)
                drawDashedLine(ctx, c2.x, c2.y, c3.x, c3.y)
                drawDashedLine(ctx, c3.x, c3.y, c0.x, c0.y)

                // Per-edge length labels, computed from the actual corner
                // positions so they stay correct after corners are dragged
                // off the pristine rectangle.
                drawLengthLabel(ctx, c0.x, c0.y, c1.x, c1.y, edgeLenM(0, 1))
                drawLengthLabel(ctx, c2.x, c2.y, c3.x, c3.y, edgeLenM(2, 3))
                drawLengthLabel(ctx, c1.x, c1.y, c2.x, c2.y, edgeLenM(1, 2))
                drawLengthLabel(ctx, c3.x, c3.y, c0.x, c0.y, edgeLenM(3, 0))

                // Origin crosshair (no anchor here, just the takeoff reference)
                const origin = Qt.point(panel.mxToPx(0), panel.myToPy(0))
                ctx.setLineDash([])
                ctx.strokeStyle = mapOn ? "rgba(255,200,80,0.9)" : Theme.warnText
                ctx.lineWidth = 1
                const arm = 8
                ctx.beginPath()
                ctx.moveTo(origin.x - arm, origin.y); ctx.lineTo(origin.x + arm, origin.y)
                ctx.moveTo(origin.x, origin.y - arm); ctx.lineTo(origin.x, origin.y + arm)
                ctx.stroke()

                // Live rocket UWB tag positions from downlink (Vec2 each)
                if (sensorData.uwbTag0Valid || sensorData.uwbTag1Valid) {
                    ctx.fillStyle = mapOn ? "rgba(255,80,80,0.95)" : Theme.danger
                    ctx.strokeStyle = mapOn ? "#111" : Theme.background
                    ctx.lineWidth = 1.5
                    if (sensorData.uwbTag0Valid) {
                        const t0 = Qt.point(panel.mxToPx(sensorData.uwbTag0X),
                                            panel.myToPy(sensorData.uwbTag0Y))
                        ctx.beginPath()
                        ctx.arc(t0.x, t0.y, 5, 0, 2 * Math.PI)
                        ctx.fill(); ctx.stroke()
                    }
                    if (sensorData.uwbTag1Valid) {
                        const t1 = Qt.point(panel.mxToPx(sensorData.uwbTag1X),
                                            panel.myToPy(sensorData.uwbTag1Y))
                        ctx.beginPath()
                        ctx.arc(t1.x, t1.y, 5, 0, 2 * Math.PI)
                        ctx.fill(); ctx.stroke()
                    }
                    if (sensorData.uwbTag0Valid && sensorData.uwbTag1Valid) {
                        ctx.strokeStyle = mapOn ? "rgba(255,80,80,0.7)" : Theme.danger
                        ctx.setLineDash([3, 3])
                        ctx.beginPath()
                        ctx.moveTo(panel.mxToPx(sensorData.uwbTag0X),
                                   panel.myToPy(sensorData.uwbTag0Y))
                        ctx.lineTo(panel.mxToPx(sensorData.uwbTag1X),
                                   panel.myToPy(sensorData.uwbTag1Y))
                        ctx.stroke()
                        ctx.setLineDash([])
                    }
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

        // Live rocket-side UWB tag positions (downlink) drive the red overlay.
        Connections {
            target: sensorData
            function onUwbDataChanged() { geometryLayer.requestPaint() }
        }

        // Refresh tiles and repaint overlays on map state changes
        Connections {
            target: panel
            function onMapEnabledChanged() { panel.refreshTiles(); gridLayer.requestPaint(); geometryLayer.requestPaint() }
            function onRefValidChanged()   { panel.refreshTiles(); gridLayer.requestPaint(); geometryLayer.requestPaint() }
        }

        // ── Origin decorations (dashed range circle + amber diamond) ──
        Canvas {
            id: originCircle
            width: 160
            height: 160
            x: panel.mxToPx(0) - width / 2
            y: panel.myToPy(0) - height / 2
            z: 4
            onPaint: {
                const ctx = getContext("2d")
                ctx.reset()
                ctx.strokeStyle = "rgba(255,59,47,0.5)"
                ctx.lineWidth = 1
                ctx.setLineDash([5, 4])
                ctx.beginPath()
                ctx.arc(width / 2, height / 2, width / 2 - 1, 0, 2 * Math.PI)
                ctx.stroke()
            }
        }

        Rectangle {
            // soft amber halo behind the origin diamond
            width: 22
            height: 22
            rotation: 45
            color: "#33ffb400"
            x: panel.mxToPx(0) - width / 2
            y: panel.myToPy(0) - height / 2
            z: 4
        }
        Rectangle {
            id: originDiamond
            width: 14
            height: 14
            rotation: 45
            color: Theme.amber
            x: panel.mxToPx(0) - width / 2
            y: panel.myToPy(0) - height / 2
            z: 4
        }

        // ── Grid caption (top-left) ──
        Text {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.topMargin: 10
            anchors.leftMargin: 12
            z: 20
            text: "GRID " + gridLayer.niceStep(60) + " m · +" + panel.pxToMx(mapView.width).toFixed(1) + " m"
            color: Theme.success
            font.family: Theme.monoFamily
            font.pixelSize: 10
        }

        // ── Pole markers (draggable) ──
        Repeater {
            id: poleRepeater
            model: panel.poles
            delegate: Item {
                id: markerItem
                required property var modelData
                required property int index

                // Follow the live drag position for the corner being moved.
                readonly property bool isActive: panel.activePoleIndex === index
                readonly property real poleXm: isActive ? panel.activePoleXm : modelData.x
                readonly property real poleYm: isActive ? panel.activePoleYm : modelData.y

                x: panel.mxToPx(poleXm) - hitSize/2
                y: panel.myToPy(poleYm) - hitSize/2

                property int hitSize: 20
                width: hitSize
                height: hitSize
                visible: panel.poles.length === 4
                z: 10

                Rectangle {
                    anchors.centerIn: parent
                    width: 12
                    height: 12
                    radius: 6
                    property bool mapOn: panel.mapEnabled && panel.refValid
                    color: mapOn ? "#2a6648" : Theme.successText
                    border.color: mapOn ? "#111" : Theme.background
                    border.width: 2
                }

                Text {
                    anchors.left: parent.right
                    anchors.bottom: parent.top
                    anchors.leftMargin: 2
                    text: markerItem.modelData.id + " (" + markerItem.poleXm.toFixed(2) + ", " + markerItem.poleYm.toFixed(2) + ")"
                    color: Theme.textSecondary
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontCaption
                }

                MouseArea {
                    id: markerDragArea
                    anchors.fill: parent
                    anchors.margins: -4
                    hoverEnabled: true
                    acceptedButtons: Qt.LeftButton
                    cursorShape: isDragging ? Qt.ClosedHandCursor
                               : containsMouse ? Qt.OpenHandCursor : Qt.ArrowCursor

                    property bool isDragging: false
                    property real offsetXm: 0
                    property real offsetYm: 0

                    // Map the press into mapView coords (stable even as the marker
                    // moves under the cursor) and remember the grab offset so the
                    // corner doesn't jump to the cursor on the first move.
                    onPressed: (mouse) => {
                        isDragging = true
                        const pt = markerDragArea.mapToItem(mapView, mouse.x, mouse.y)
                        offsetXm = markerItem.modelData.x - panel.pxToMx(pt.x)
                        offsetYm = markerItem.modelData.y - panel.pyToMy(pt.y)
                        panel.movePole(markerItem.index,
                                       markerItem.modelData.x, markerItem.modelData.y)
                        mouse.accepted = true
                    }

                    onPositionChanged: (mouse) => {
                        if (!isDragging) return
                        const pt = markerDragArea.mapToItem(mapView, mouse.x, mouse.y)
                        panel.movePole(markerItem.index,
                                       panel.pxToMx(pt.x) + offsetXm,
                                       panel.pyToMy(pt.y) + offsetYm)
                    }

                    onReleased: (mouse) => {
                        if (isDragging) { isDragging = false; panel.commitPole() }
                        mouse.accepted = true
                    }

                    // Mouse grab lost (e.g. window focus change): commit what we have.
                    onCanceled: {
                        if (isDragging) { isDragging = false; panel.commitPole() }
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

        // ── OSM Attribution ──
        Rectangle {
            id: osmAttribution
            anchors.right: parent.right
            anchors.bottom: zoomControls.top
            anchors.rightMargin: 12
            anchors.bottomMargin: 4
            width: osmText.implicitWidth + 6
            height: osmText.implicitHeight + 4
            color: Theme.surfaceElevated
            opacity: 0.85
            radius: 0
            visible: panel.mapEnabled && panel.refValid
            z: 20

            Text {
                id: osmText
                anchors.centerIn: parent
                text: "© Esri World Imagery"
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
                Layout.preferredWidth: 34
                Layout.preferredHeight: 34
                hoverEnabled: true
                font.family: Theme.fontFamily
                font.pixelSize: 16
                background: Rectangle {
                    radius: 0
                    color: zoomInBtn.down    ? Theme.btnSecondaryPress
                         : zoomInBtn.hovered ? Theme.btnSecondaryHover
                         :                     Theme.surfaceElevated
                    border.width: Theme.strokeControl
                    border.color: zoomInBtn.hovered ? Theme.accent : Theme.border
                }
                contentItem: Text {
                    anchors.centerIn: parent
                    text: zoomInBtn.text
                    color: zoomInBtn.hovered ? Theme.accentMuted : Theme.textPrimary
                    font: zoomInBtn.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: panel.zoomAt(mapView.width/2, mapView.height/2, 1.2)
            }
            Basic.Button {
                id: zoomOutBtn
                text: "−"
                Layout.preferredWidth: 34
                Layout.preferredHeight: 34
                hoverEnabled: true
                font.family: Theme.fontFamily
                font.pixelSize: 16
                background: Rectangle {
                    radius: 0
                    color: zoomOutBtn.down    ? Theme.btnSecondaryPress
                         : zoomOutBtn.hovered ? Theme.btnSecondaryHover
                         :                      Theme.surfaceElevated
                    border.width: Theme.strokeControl
                    border.color: zoomOutBtn.hovered ? Theme.accent : Theme.border
                }
                contentItem: Text {
                    anchors.centerIn: parent
                    text: zoomOutBtn.text
                    color: zoomOutBtn.hovered ? Theme.accentMuted : Theme.textPrimary
                    font: zoomOutBtn.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: panel.zoomAt(mapView.width/2, mapView.height/2, 1/1.2)
            }
            Basic.Button {
                id: resetBtn
                text: "⟲"
                Layout.preferredWidth: 34
                Layout.preferredHeight: 34
                hoverEnabled: true
                font.family: Theme.fontFamily
                font.pixelSize: 14
                background: Rectangle {
                    radius: 0
                    color: resetBtn.down    ? Theme.btnSecondaryPress
                         : resetBtn.hovered ? Theme.btnSecondaryHover
                         :                    Theme.surfaceElevated
                    border.width: Theme.strokeControl
                    border.color: resetBtn.hovered ? Theme.accent : Theme.border
                }
                contentItem: Text {
                    anchors.centerIn: parent
                    text: resetBtn.text
                    color: resetBtn.hovered ? Theme.accentMuted : Theme.textPrimary
                    font: resetBtn.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: panel.resetView()
            }
        }

        // ── Axis range caption (bottom-center) ──
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 10
            z: 20
            text: {
                const a = panel.pxToMx(0)
                const b = panel.pxToMx(mapView.width)
                const fmt = function(v) { return (v >= 0 ? "+" : "") + v.toFixed(1) }
                return fmt(a) + " m ◄────────► " + fmt(b) + " m"
            }
            color: Theme.textTertiary
            font.family: Theme.monoFamily
            font.pixelSize: 10
        }

        // ── Legend (bottom-left) ──
        Rectangle {
            anchors.left: parent.left
            anchors.bottom: parent.bottom
            anchors.margins: 12
            color: "#e612100a"           // rgba(18,16,10,0.9)
            border.color: Theme.divider
            border.width: Theme.strokeControl
            radius: 0
            width: legendCol.implicitWidth + 24
            height: legendCol.implicitHeight + 20
            z: 20

            ColumnLayout {
                id: legendCol
                anchors.centerIn: parent
                spacing: 5

                Text {
                    text: "LEGEND"
                    color: Theme.amber
                    font.family: Theme.fontFamily
                    font.pixelSize: 9
                    font.letterSpacing: 2
                    font.bold: true
                }
                RowLayout {
                    spacing: 8
                    Rectangle { implicitWidth: 8; implicitHeight: 8; radius: 4; color: Theme.success }
                    Text {
                        text: "GROUND ANCHOR (" + panel.poles.length + " OF 4)"
                        color: Theme.textPrimary
                        font.family: Theme.fontFamily
                        font.pixelSize: Theme.fontCaption
                    }
                }
                RowLayout {
                    spacing: 8
                    Rectangle { implicitWidth: 8; implicitHeight: 8; radius: 4; color: Theme.amber }
                    Text {
                        text: "TAKEOFF ORIGIN (0,0)"
                        color: Theme.textPrimary
                        font.family: Theme.fontFamily
                        font.pixelSize: Theme.fontCaption
                    }
                }
                RowLayout {
                    spacing: 8
                    Rectangle {
                        id: tagDot
                        implicitWidth: 8; implicitHeight: 8; radius: 4
                        color: Theme.accent
                        Blink { target: tagDot }
                    }
                    Text {
                        text: "ROCKET UWB TAG (LIVE)"
                        color: Theme.textPrimary
                        font.family: Theme.fontFamily
                        font.pixelSize: Theme.fontCaption
                    }
                }
            }
        }
    }
}

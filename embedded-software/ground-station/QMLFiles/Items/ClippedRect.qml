import QtQuick
import QtQuick.Shapes

// EVA angular panel shape. Mirrors the HTML clip-path polygons:
//   polygon(slantTL 0, 100%-slantTR 0, 100%-slantBR 100%, slantBL 100%)
// Each slant is an x-inset at that corner; equal TL+BR insets give the
// standard parallelogram cut used on buttons and badges.
//
// Root is an Item (not the Shape itself): QQuickShape recomputes its
// implicit size from the path bounds on resize, which creates an
// implicit-size binding loop when used directly as a Control background.
Item {
    id: root

    property real slantTL: 0
    property real slantTR: 0
    property real slantBL: 0
    property real slantBR: 0
    property color fillColor: "transparent"
    property color borderColor: "transparent"
    property real borderWidth: 0

    Shape {
        anchors.fill: parent
        preferredRendererType: Shape.CurveRenderer

        ShapePath {
            strokeWidth: root.borderWidth > 0 ? root.borderWidth : -1
            strokeColor: root.borderWidth > 0 ? root.borderColor : "transparent"
            fillColor: root.fillColor
            joinStyle: ShapePath.MiterJoin

            startX: root.slantTL; startY: 0
            PathLine { x: root.width - root.slantTR; y: 0 }
            PathLine { x: root.width - root.slantBR; y: root.height }
            PathLine { x: root.slantBL;              y: root.height }
            PathLine { x: root.slantTL;              y: 0 }
        }
    }
}

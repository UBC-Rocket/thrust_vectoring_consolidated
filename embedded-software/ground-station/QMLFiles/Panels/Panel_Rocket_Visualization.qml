import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers
import "../Items"
import ".."

BasePanel {
    id: panel_Rocket_Visualization

    BaseHeader {
        id: header
        headerText: "Rocket Visualization"
        jpText: "機体表示"
    }

    Item {
        id: visualization
        focus: true

        anchors{
                   top: header.bottom
                   left: parent.left
                   right: parent.right
                   bottom: parent.bottom
                   leftMargin: 14
                   rightMargin: 14
                   bottomMargin: 14
                }



        //Variables & Constants received from sensorData
           property real angle_x: sensorData.filteredAngleX
           property real angle_y: sensorData.filteredAngleY
           property real angle_z: sensorData.filteredAngleZ
           property real length: 200        // shaft length
           property real thickness: 0.15       // shaft thickness

        // Cosmetic T+ mission clock for the HUD (UI-local; no backend source).
        property int missionSeconds: 0

        function fmtAngle(v) {
            return Number.isFinite(v) ? v.toFixed(2) : "—"
        }

        function fmtClock(s) {
            const h = Math.floor(s / 3600)
            const m = Math.floor((s % 3600) / 60)
            const sec = s % 60
            const pad = (n) => (n < 10 ? "0" : "") + n
            return pad(h) + ":" + pad(m) + ":" + pad(sec)
        }

        Timer {
            interval: 1000
            repeat: true
            running: true
            onTriggered: visualization.missionSeconds += 1
        }

        // Arrow-key camera orbit. Rotates cam.position around the origin (where the
        // rocket frame sits) in spherical coordinates. Composes with WasdController:
        // each keypress re-derives spherical coords from the current position, so any
        // WASD translation in between is carried forward.
        readonly property real azimStep: 3 * Math.PI / 180
        readonly property real elevStep: 3 * Math.PI / 180
        readonly property real elevClamp: Math.PI / 2 - 0.03

        function orbit(dAzim, dElev) {
            const px = cam.position.x, py = cam.position.y, pz = cam.position.z
            const r = Math.sqrt(px*px + py*py + pz*pz)
            if (r < 1e-6) return
            let azim = Math.atan2(px, pz)
            let elev = Math.asin(py / r)
            azim += dAzim
            elev = Math.max(-elevClamp, Math.min(elevClamp, elev + dElev))
            const cosE = Math.cos(elev)
            cam.position = Qt.vector3d(r * cosE * Math.sin(azim),
                                       r * Math.sin(elev),
                                       r * cosE * Math.cos(azim))
        }

        Keys.onPressed: (event) => {
            switch (event.key) {
                case Qt.Key_Left:  orbit(-azimStep, 0);        event.accepted = true; break
                case Qt.Key_Right: orbit( azimStep, 0);        event.accepted = true; break
                case Qt.Key_Up:    orbit(0,          elevStep);  event.accepted = true; break
                case Qt.Key_Down:  orbit(0,         -elevStep); event.accepted = true; break
            }
        }

        // ── Viewport dressing (behind the 3D view) ─────────────────────────
        // Deep viewport fill + 40px red grid + amber crosshair + range circles.

        Rectangle {
            id: viewportBg
            anchors.fill: parent
            color: Theme.sceneBackground
            border.color: Theme.divider
            border.width: 1
        }

        GridBackdrop {
            anchors.fill: viewportBg
            anchors.margins: 1
            cellSize: 40
            lineColor: Theme.gridLine
            lineOpacity: 0.12
        }

        // Center crosshair — amber @35%
        Rectangle {
            x: Math.round(parent.width / 2)
            y: 1
            width: 1
            height: parent.height - 2
            color: Theme.amber
            opacity: 0.35
        }
        Rectangle {
            x: 1
            y: Math.round(parent.height / 2)
            width: parent.width - 2
            height: 1
            color: Theme.amber
            opacity: 0.35
        }

        // Solid 120px + dashed 220px range circles, centered
        Canvas {
            id: rangeCircles
            anchors.fill: parent
            onWidthChanged: requestPaint()
            onHeightChanged: requestPaint()

            function strokeCircle(ctx, r, color, dashed) {
                ctx.strokeStyle = color
                ctx.lineWidth = 1
                const cx = width / 2, cy = height / 2
                if (!dashed) {
                    ctx.beginPath()
                    ctx.arc(cx, cy, r, 0, 2 * Math.PI)
                    ctx.stroke()
                    return
                }
                // Qt's Canvas has no setLineDash — draw arc segments manually.
                const dash = 5 / r
                for (let a = 0; a < 2 * Math.PI; a += dash * 2) {
                    ctx.beginPath()
                    ctx.arc(cx, cy, r, a, Math.min(a + dash, 2 * Math.PI))
                    ctx.stroke()
                }
            }

            onPaint: {
                const ctx = getContext("2d")
                ctx.reset()
                strokeCircle(ctx, 60,  Qt.rgba(1, 59/255, 47/255, 0.6), false)
                strokeCircle(ctx, 110, Qt.rgba(1, 59/255, 47/255, 0.3), true)
            }
        }

        // Click the 3D view to claim keyboard focus (arrows + WASD).
        // accepted=false lets the click fall through to WasdController.
        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton
            onPressed: (mouse) => { visualization.forceActiveFocus(); mouse.accepted = false }
        }

        // //FAKE DATA
        //    Timer {
        //        interval: 16
        //        repeat: true
        //        running: true
        //        triggeredOnStart: true
        //        property real t: 0
        //        onTriggered: {
        //            t += interval/1000
        //            visualization.angle_x = 25 * Math.sin(2*Math.PI*0.27 * t)          // generating random fake X angle
        //            visualization.angle_y = 35 * Math.sin(2*Math.PI*0.19 * t + 1.1)    // fake Y angle
        //        }
        //    }




           //3D render of rocket's angle
           View3D{
               anchors.fill: parent

               PerspectiveCamera{
                   id: cam
                   position: Qt.vector3d(4500,2000,4500)
                   lookAtNode: rocket_frame
               }

               environment: SceneEnvironment{
                   // Transparent so the EVA grid/crosshair dressing shows through;
                   // the viewport fill itself is Theme.sceneBackground (viewportBg).
                   backgroundMode: SceneEnvironment.Transparent
                   clearColor: Theme.sceneBackground
               }

               DirectionalLight{}



               //The actual rocket framh
               Node {
                   id: rocket_frame

                   eulerRotation: Qt.vector3d(visualization.angle_x, visualization.angle_z, visualization.angle_y)
                   pivot: Qt.vector3d(0, 0, 0)

                   FullDroneAssembly{
                                      id: droneModel

                                  //     //Qt has y-axis pointing up by default. So the y-axis rotation is actually the "z-axis"
                                      // eulerRotation: Qt.vector3d(visualization.angle_x, visualization.angle_z, visualization.angle_y)
                                      // eulerRotation: Qt.vector3d(0,0,0)
                                      // pivot: Qt.vector3d(0,0,0)
                                      scale: Qt.vector3d(3,3,3)
                                      // position: Qt.vector3d(0, 0, 0)
                                      position: Qt.vector3d(-360, -1040, -1400)


                                      // property real d: 10          // diameter
                                      // property real h_body: 30    // cylinder height
                                      // property real h_nose: h_body/3 //nose height

                                      // // Fin dimensions: offset, length, thickness
                                      // property real finLength: d * 1.7
                                      // property real finThickness: d * 0.05
                                      // property real finOffsetY: -h_body * 24


                                      // //body
                                      // Model{
                                      //         source: "#Cylinder"
                                      //         scale: Qt.vector3d(rocket_frame.d, rocket_frame.h_body, rocket_frame.d)
                                      //         materials: DefaultMaterial { diffuseColor: "#d9d9d9" }
                                      // }

                                      // //top cone
                                      // Model {
                                      //     source: "#Cone"
                                      //     position: Qt.vector3d(0,1500,0)
                                      //     scale: Qt.vector3d(rocket_frame.d, rocket_frame.h_nose, rocket_frame.d)
                                      //     materials: DefaultMaterial { diffuseColor: "#d9d9d9" }
                                      // }

                                      //Helper lines to visualize tilt
                                      Model{
                                          source: "#Cylinder"
                                          scale: Qt.vector3d(0.03, visualization.length, visualization.thickness)
                                          materials: DefaultMaterial{
                                              diffuseColor: "green"
                                              lighting: PrincipledMaterial.NoLighting
                                          }
                                          position: Qt.vector3d(360/3, 1040/3, 1400/3)
                                      }

                                      Model{
                                          source: "#Cylinder"
                                          eulerRotation: Qt.vector3d(0,0,90)
                                          scale: Qt.vector3d(0.03, visualization.length, visualization.thickness)
                                          materials: DefaultMaterial{
                                              diffuseColor: "red"
                                              lighting: PrincipledMaterial.NoLighting
                                          }
                                          position: Qt.vector3d(360/3, 1040/3, 1400/3)
                                      }

                                      Model{
                                          source: "#Cylinder"
                                          eulerRotation: Qt.vector3d(90,0,0)
                                          scale: Qt.vector3d(0.03, visualization.length, visualization.thickness)
                                          materials: DefaultMaterial{
                                              diffuseColor: "blue"
                                              lighting: PrincipledMaterial.NoLighting
                                          }
                                          position: Qt.vector3d(360/3, 1040/3, 1400/3)
                                      }
                                  }
               }

               AxisHelper{
                   enableAxisLines: true
                   enableXYGrid: true
                   enableXZGrid: false
                   enableYZGrid: true
                   gridOpacity: 0.2
                   scale: Qt.vector3d(10,10,10)
               }

               //Press W,A,S,D to move view of camera
               WasdController{
                   controlledObject: cam
                   leftSpeed: 20
                   rightSpeed: 20
                   forwardSpeed: 10
                   backSpeed: 10
                   upSpeed: 0
                   downSpeed: 0
                   mouseEnabled: false
               }
           }

        // ── HUD overlays (above the 3D view; non-interactive) ──────────────

        // Top-left: camera / grid caption (static cosmetic copy)
        Text {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.topMargin: 10
            anchors.leftMargin: 12
            text: "CAM 01 · ORBIT\nGRID 1.0 m"
            font.family: Theme.monoFamily
            font.pixelSize: 10
            lineHeight: 1.4
            color: Theme.success
        }

        // Bottom-right: live roll/pitch + cosmetic T+ clock
        Text {
            anchors.bottom: parent.bottom
            anchors.right: parent.right
            anchors.bottomMargin: 10
            anchors.rightMargin: 12
            horizontalAlignment: Text.AlignRight
            text: "ROLL " + visualization.fmtAngle(visualization.angle_x)
                  + "° PITCH " + visualization.fmtAngle(visualization.angle_y) + "°"
                  + "\nT+ " + visualization.fmtClock(visualization.missionSeconds)
            font.family: Theme.monoFamily
            font.pixelSize: 10
            lineHeight: 1.4
            color: Theme.accentMuted
        }

        // Top-right: blinking red status square
        Rectangle {
            id: blinkSquare
            anchors.top: parent.top
            anchors.right: parent.right
            anchors.topMargin: 10
            anchors.rightMargin: 12
            width: 10
            height: 10
            color: Theme.accent
        }
        Blink { target: blinkSquare; period: Theme.blinkPeriod }
    }
}

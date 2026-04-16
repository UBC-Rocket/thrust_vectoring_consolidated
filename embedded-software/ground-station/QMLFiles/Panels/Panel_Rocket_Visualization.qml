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
        id:header
        headerText: "Rocket Visualization"
    }

    Item {
        id: visualization
        focus: true

        anchors{
                   top: header.bottom
                   left: parent.left
                   right: parent.right
                   bottom: parent.bottom
                   leftMargin: 10
                   rightMargin: 10
                   bottomMargin: 10
                }



        //Variables & Constants received from sensorData
           property real angle_x: sensorData.filteredAngleX
           property real angle_y: sensorData.filteredAngleY
           property real angle_z: sensorData.filteredAngleZ
           property real length: 200        // shaft length
           property real thickness: 0.15       // shaft thickness

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
                   backgroundMode: SceneEnvironment.Color
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
    }
}

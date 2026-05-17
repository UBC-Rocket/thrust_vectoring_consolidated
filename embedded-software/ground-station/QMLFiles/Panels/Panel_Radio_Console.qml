import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "../Items"

// Embedded RFD900x single/dual port terminal
Pane {
    id: radioWin
    padding: 0
    font.family: Theme.fontFamily

    background: Rectangle {
        color: Theme.background
        border.color: Theme.border
        border.width: Theme.strokePanel
        radius: Theme.radiusPanel
    }

    palette {
        window:          Theme.background
        base:            Theme.surfaceInset
        alternateBase:   Theme.surfaceElevated
        text:            Theme.textPrimary
        windowText:      Theme.textPrimary
        button:          Theme.btnSecondaryBg
        buttonText:      Theme.btnSecondaryText
        highlight:       Theme.accent
        highlightedText: Theme.background
        placeholderText: Theme.textTertiary
        mid:             Theme.border
        dark:            Theme.border
        light:           Theme.borderLight
    }

    // ---------- Modes / State ----------

    property bool singleMode: true

    property int rxWhich: 1
    property int txWhich: 2

    property bool   p1Connected: bridge.isConnected(1)
    property bool   p2Connected: bridge.isConnected(2)
    property string rxLogP1: ""
    property string rxLogP2: ""

    property bool singleConnected: bridge.isConnected(rxWhich)
    property bool txConnected: bridge.isConnected(txWhich)
    property bool rxConnected: bridge.isConnected(rxWhich)

    property var baudList: [57600, 115200]

    // ---------- Toolbar title ----------

    function titleText() {
        if (singleMode) {
            const n = bridge.portName(rxWhich) || "—"
            const b = bridge.baudRate(rxWhich) || "—"
            return "RFD900x — Single-Port Mode  P" + rxWhich + ": " + n + "@" + b
        } else {
            const tn = bridge.portName(txWhich) || "—"
            const rn = bridge.portName(rxWhich) || "—"
            const tb = bridge.baudRate(txWhich) || "—"
            const rb = bridge.baudRate(rxWhich) || "—"
            return "Dual RFD900x Chat —  TX(P" + txWhich + "): " + tn + "@" + tb +
                   "  |  RX(P" + rxWhich + "): " + rn + "@" + rb
        }
    }
    property string toolbarTitle: titleText()

    // ---------- Error popup ----------

    Popup {
        id: errorPopup
        x: 16; y: 16
        modal: false; focus: false
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        contentItem: Frame {
            padding: 10
            Label { id: errorLabel; wrapMode: Text.Wrap; text: "" }
        }
    }

    // ---------- Signals from backend ----------

    Connections {
        target: bridge

        function onErrorMessage(msg) {
            errorLabel.text = msg
            errorPopup.open()
        }

        function onPortsChanged() {
            singlePortSel.model = bridge.ports
            radioWin.toolbarTitle = radioWin.titleText()
        }

        function onBaudChanged(which) {
            if (which === rxWhich || which === txWhich)
                radioWin.toolbarTitle = radioWin.titleText()
        }

        function onPortNameChanged(which) {
            if (which === rxWhich || which === txWhich)
                radioWin.toolbarTitle = radioWin.titleText()
        }

        function onTextReceivedFrom(which, line) {
            if (which === 1)
                rxLogP1 += line + "\n"
            else if (which === 2)
                rxLogP2 += line + "\n"
        }

        function onConnectedChanged(which, connected) {
            if (which === 1) {
                p1Connected = connected
                if (rxWhich === 1)
                    singleConnected = connected
            } else if (which === 2) {
                p2Connected = connected
                if (rxWhich === 2)
                    singleConnected = connected
            }
        }

        function onTxToChanged() { radioWin.toolbarTitle = radioWin.titleText() }
        function onRxFromChanged() { radioWin.toolbarTitle = radioWin.titleText() }
    }

    // Shared RX text area alias (logical only)
    property alias rxTextArea: rxText
    TextArea {
        id: rxText
        visible: false
    }

    // ---------- Layout ----------

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        ToolBar {
            Layout.fillWidth: true

            RowLayout {
                anchors.fill: parent
                spacing: 12

                Label {
                    text: radioWin.toolbarTitle
                    font.bold: true
                    Layout.alignment: Qt.AlignVCenter
                    padding: 8
                    elide: Label.ElideRight
                    Layout.fillWidth: true
                }

                // Mode switcher: Single vs Dual
                RowLayout {
                    spacing: 6
                    Label { text: "Mode:"; Layout.alignment: Qt.AlignVCenter }
                    ComboBox {
                        id: modeBox
                        model: ["Single Port", "Dual Port"]
                        currentIndex: singleMode ? 0 : 1
                        onActivated: {
                            if (bridge.isConnected(1)) bridge.disconnectPort(1)
                            if (bridge.isConnected(2)) bridge.disconnectPort(2)

                            p1Connected = false
                            p2Connected = false
                            singleConnected = false

                            if (typeof timerP1 !== "undefined") timerP1.stop()
                            if (typeof timerP2 !== "undefined") timerP2.stop()

                            singleMode = (currentIndex === 0)

                            if (singleMode) {
                                rxWhich = 1
                                txWhich = 1
                            } else {
                                rxWhich = 1
                                txWhich = 2
                            }

                            radioWin.toolbarTitle = radioWin.titleText()
                        }
                    }
                }

                Button { text: "Refresh Ports"; onClicked: bridge.refreshPorts() }
            }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.margins: 12
            currentIndex: singleMode ? 0 : 1

            // ===== Page 0: Single-Port Mode =====
            Item {
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 10

                    Frame {
                        Layout.fillWidth: true
                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8

                            Label { text: "Single-Port (RX + TX)"; font.bold: true; padding: 4 }

                            RowLayout {
                                spacing: 8

                                Label { text: "Use physical:"; Layout.alignment: Qt.AlignVCenter }
                                ComboBox {
                                    id: singleWhichSel
                                    model: [1, 2]
                                    Layout.preferredWidth: 90
                                    Component.onCompleted: currentIndex = (rxWhich === 2 ? 1 : 0)
                                    onActivated: {
                                        const w = Number(currentText)
                                        rxWhich = w
                                        txWhich = w
                                        if (bridge.isConnected(w)) {
                                            bridge.setRxFrom(w)
                                            bridge.setTxTo(w)
                                        }
                                        radioWin.toolbarTitle = radioWin.titleText()
                                    }
                                }

                                ComboBox {
                                    id: singlePortSel
                                    model: bridge.ports
                                    Layout.preferredWidth: 220
                                    Component.onCompleted: {
                                        const name = bridge.portName(rxWhich)
                                        const i = model.indexOf(name)
                                        currentIndex = (i >= 0 ? i : -1)
                                    }
                                }

                                Button { text: "Refresh"; onClicked: bridge.refreshPorts() }

                                ComboBox {
                                    id: singleBaudSel
                                    model: baudList
                                    Layout.preferredWidth: 120
                                    Component.onCompleted: {
                                        const i = baudList.indexOf(bridge.baudRate(rxWhich) || 57600)
                                        currentIndex = (i >= 0 ? i : 0)
                                    }
                                }

                                Button {
                                  id: singleConnBtn
                                  text: singleConnected ? "Disconnect" : "Connect"
                                  onClicked: {
                                    if (singleConnected) {
                                      bridge.disconnectPort(rxWhich)
                                      singleConnected = false
                                    } else {
                                      if (singlePortSel.currentIndex >= 0) {
                                        if (bridge.connectPort(rxWhich, singlePortSel.currentText, Number(singleBaudSel.currentText))) {
                                          bridge.setRxFrom(rxWhich)
                                          bridge.setTxTo(rxWhich)
                                          singleConnected = true
                                        }
                                      } else {
                                        errorLabel.text = "Select a COM port first."
                                        errorPopup.open()
                                      }
                                    }
                                    radioWin.toolbarTitle = radioWin.titleText()
                                  }
                                }

                                Label {
                                  text: singleConnected ? "Connected" : "—"
                                  color: singleConnected ? Theme.success : Theme.textTertiary
                                }
                            }
                        }
                    }

                    Frame {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8

                            GroupBox {
                                title: "Type & Send (press Enter)"
                                Layout.fillWidth: true
                                ColumnLayout {
                                    anchors.fill: parent
                                    spacing: 8

                                    TextField {
                                        id: singleInput
                                        placeholderText: "Type text and press Enter…"
                                        Layout.fillWidth: true
                                        enabled: singleConnected
                                        focus: true
                                        onAccepted: {
                                            if (text.length && bridge.isConnected(rxWhich)) {
                                                bridge.sendText(rxWhich, text)
                                                text = ""
                                            }
                                        }
                                    }
                                }
                            }

                            GroupBox {
                                title: "Received"
                                Layout.fillWidth: true
                                Layout.fillHeight: true

                                ColumnLayout {
                                    anchors.fill: parent
                                    spacing: 4

                                    Flickable {
                                        id: flickSingle
                                        Layout.fillWidth: true
                                        Layout.fillHeight: true
                                        clip: true

                                        contentWidth: singleRxText.paintedWidth
                                        contentHeight: singleRxText.paintedHeight

                                        ScrollBar.vertical: ScrollBar {
                                            policy: ScrollBar.AsNeeded
                                        }

                                        TextEdit {
                                            id: singleRxText
                                            readOnly: true
                                            wrapMode: TextEdit.Wrap
                                            width: flickSingle.width
                                            font.family: Theme.monoFamily
                                            color: Theme.textPrimary
                                            text: (rxWhich === 1 ? rxLogP1 : rxLogP2)

                                            onTextChanged: {
                                                flickSingle.contentY = Math.max(0, flickSingle.contentHeight - flickSingle.height)
                                            }
                                        }
                                    }

                                    RowLayout {
                                        Layout.alignment: Qt.AlignRight
                                        Button {
                                            text: "Clear RX"
                                            onClicked: {
                                                if (rxWhich === 1)
                                                    rxLogP1 = ""
                                                else
                                                    rxLogP2 = ""
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // ===== Page 1: Dual-Port Mode =====
            Item {
                id: dualModePage

                property string rxLogP1: ""
                property string rxLogP2: ""
                property bool   p1Connected: bridge.isConnected(1)
                property bool   p2Connected: bridge.isConnected(2)

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 12

                    // ------------------- PORT 1 COLUMN -------------------
                    Frame {
                        Layout.fillWidth: true
                        Layout.fillHeight: true

                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8

                            Label { text: "Port 1"; font.bold: true; padding: 4 }

                            RowLayout {
                                spacing: 8

                                ComboBox {
                                    id: portSel1
                                    model: bridge.ports
                                    Layout.preferredWidth: 160
                                    Component.onCompleted: {
                                        const name = bridge.portName(1)
                                        const i = model.indexOf(name)
                                        currentIndex = (i >= 0 ? i : -1)
                                    }
                                }

                                ComboBox {
                                    id: baudSel1
                                    model: [57600, 115200]
                                    Layout.preferredWidth: 80
                                    Component.onCompleted: {
                                        const i = baudSel1.model.indexOf(bridge.baudRate(1) || 57600)
                                        currentIndex = (i >= 0 ? i : 0)
                                    }
                                }

                                Button {
                                    id: connBtn1
                                    text: p1Connected ? "Disconnect" : "Connect"
                                    Layout.preferredWidth: Math.max(implicitWidth, 100)
                                    onClicked: {
                                        if (p1Connected) {
                                            bridge.disconnectPort(1)
                                            p1Connected = false
                                            timerP1.stop()
                                        } else {
                                            if (portSel1.currentIndex >= 0) {
                                                if (bridge.connectPort(1,
                                                                       portSel1.currentText,
                                                                       Number(baudSel1.currentText)))
                                                    p1Connected = true
                                            } else {
                                                errorLabel.text = "Pick a COM port for P1."
                                                errorPopup.open()
                                            }
                                        }
                                    }
                                }

                                Button {
                                    text: "Refresh"
                                    Layout.preferredWidth: Math.max(implicitWidth, 80)
                                    onClicked: bridge.refreshPorts()
                                }

                                Label {
                                    text: p1Connected ? "Connected" : "—"
                                    color: p1Connected ? Theme.success : Theme.textTertiary
                                    Layout.alignment: Qt.AlignVCenter
                                }
                            }

                            GroupBox {
                                title: "Send (P1)"
                                Layout.fillWidth: true
                                ColumnLayout {
                                    anchors.fill: parent
                                    spacing: 8

                                    TextField {
                                        id: sendOnce1
                                        placeholderText: "Type and press Enter to send from P1…"
                                        Layout.fillWidth: true
                                        enabled: p1Connected
                                        onAccepted: {
                                            if (text.length && p1Connected) {
                                                bridge.sendText(1, text)
                                                text = ""
                                            }
                                        }
                                    }

                                    RowLayout {
                                        spacing: 10

                                        TextField {
                                            id: periodicMsg1
                                            placeholderText: "Periodic message (P1)…"
                                            Layout.preferredWidth: 230
                                        }

                                        RowLayout {
                                            spacing: 6
                                            SpinBox {
                                                id: hz1
                                                from: 1; to: 200; value: 50
                                                editable: true
                                            }
                                            Label { text: "Hz"; Layout.alignment: Qt.AlignVCenter }
                                        }

                                        Button {
                                            id: startP1
                                            text: "Start"
                                            enabled: p1Connected
                                            onClicked: {
                                                timerP1.interval = Math.max(1, Math.floor(1000 / hz1.value))
                                                timerP1.start()
                                            }
                                        }
                                        Button {
                                            text: "Stop"
                                            onClicked: timerP1.stop()
                                        }
                                    }

                                    Timer {
                                        id: timerP1
                                        repeat: true
                                        running: false
                                        interval: 20
                                        onTriggered: {
                                            if (p1Connected && periodicMsg1.text.length) {
                                                bridge.sendText(1, periodicMsg1.text)
                                            }
                                        }
                                    }
                                }
                            }

                            GroupBox {
                                title: "Received (P1)"
                                Layout.fillWidth: true
                                Layout.fillHeight: true

                                ColumnLayout {
                                    anchors.fill: parent
                                    spacing: 4

                                    Flickable {
                                        id: flickP1
                                        Layout.fillWidth: true
                                        Layout.fillHeight: true
                                        clip: true

                                        contentWidth: rxTextP1.paintedWidth
                                        contentHeight: rxTextP1.paintedHeight

                                        ScrollBar.vertical: ScrollBar {
                                            policy: ScrollBar.AsNeeded
                                        }

                                        TextEdit {
                                            id: rxTextP1
                                            readOnly: true
                                            wrapMode: TextEdit.Wrap
                                            width: flickP1.width
                                            font.family: Theme.monoFamily
                                            color: Theme.textPrimary
                                            text: rxLogP1

                                            onTextChanged: {
                                                flickP1.contentY = Math.max(0, flickP1.contentHeight - flickP1.height)
                                            }
                                        }
                                    }

                                    RowLayout {
                                        Layout.alignment: Qt.AlignRight
                                        Button {
                                            text: "Clear"
                                            onClicked: rxLogP1 = ""
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // ------------------- PORT 2 COLUMN -------------------
                    Frame {
                        Layout.fillWidth: true
                        Layout.fillHeight: true

                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8

                            Label { text: "Port 2"; font.bold: true; padding: 4 }

                            RowLayout {
                                spacing: 8

                                ComboBox {
                                    id: portSel2
                                    model: bridge.ports
                                    Layout.preferredWidth: 160
                                    Component.onCompleted: {
                                        const name = bridge.portName(2)
                                        const i = model.indexOf(name)
                                        currentIndex = (i >= 0 ? i : -1)
                                    }
                                }

                                ComboBox {
                                    id: baudSel2
                                    model: [57600, 115200]
                                    Layout.preferredWidth: 80
                                    Component.onCompleted: {
                                        const i = baudSel2.model.indexOf(bridge.baudRate(2) || 57600)
                                        currentIndex = (i >= 0 ? i : 0)
                                    }
                                }

                                Button {
                                    id: connBtn2
                                    text: p2Connected ? "Disconnect" : "Connect"
                                    Layout.preferredWidth: Math.max(implicitWidth, 100)
                                    onClicked: {
                                        if (p2Connected) {
                                            bridge.disconnectPort(2)
                                            p2Connected = false
                                            timerP2.stop()
                                        } else {
                                            if (portSel2.currentIndex >= 0) {
                                                if (bridge.connectPort(2,
                                                                       portSel2.currentText,
                                                                       Number(baudSel2.currentText)))
                                                    p2Connected = true
                                            } else {
                                                errorLabel.text = "Pick a COM port for P2."
                                                errorPopup.open()
                                            }
                                        }
                                    }
                                }

                                Button {
                                    text: "Refresh"
                                    Layout.preferredWidth: Math.max(implicitWidth, 80)
                                    onClicked: bridge.refreshPorts()
                                }

                                Label {
                                    text: p2Connected ? "Connected" : "—"
                                    color: p2Connected ? Theme.success : Theme.textTertiary
                                    Layout.alignment: Qt.AlignVCenter
                                }
                            }

                            GroupBox {
                                title: "Send (P2)"
                                Layout.fillWidth: true
                                ColumnLayout {
                                    anchors.fill: parent
                                    spacing: 8

                                    TextField {
                                        id: sendOnce2
                                        placeholderText: "Type and press Enter to send from P2…"
                                        Layout.fillWidth: true
                                        enabled: p2Connected
                                        onAccepted: {
                                            if (text.length && p2Connected) {
                                                bridge.sendText(2, text)
                                                text = ""
                                            }
                                        }
                                    }

                                    RowLayout {
                                        spacing: 10

                                        TextField {
                                            id: periodicMsg2
                                            placeholderText: "Periodic message (P2)…"
                                            Layout.preferredWidth: 230
                                        }

                                        RowLayout {
                                            spacing: 6
                                            SpinBox {
                                                id: hz2
                                                from: 1; to: 200; value: 50
                                                editable: true
                                            }
                                            Label { text: "Hz"; Layout.alignment: Qt.AlignVCenter }
                                        }

                                        Button {
                                            id: startP2
                                            text: "Start"
                                            enabled: p2Connected
                                            onClicked: {
                                                timerP2.interval = Math.max(1, Math.floor(1000 / hz2.value))
                                                timerP2.start()
                                            }
                                        }
                                        Button {
                                            text: "Stop"
                                            onClicked: timerP2.stop()
                                        }
                                    }

                                    Timer {
                                        id: timerP2
                                        repeat: true
                                        running: false
                                        interval: 20
                                        onTriggered: {
                                            if (p2Connected && periodicMsg2.text.length) {
                                                bridge.sendText(2, periodicMsg2.text)
                                            }
                                        }
                                    }
                                }
                            }

                            GroupBox {
                                title: "Received (P2)"
                                Layout.fillWidth: true
                                Layout.fillHeight: true

                                ColumnLayout {
                                    anchors.fill: parent
                                    spacing: 4

                                    Flickable {
                                        id: flickP2
                                        Layout.fillWidth: true
                                        Layout.fillHeight: true
                                        clip: true

                                        contentWidth: rxTextP2.paintedWidth
                                        contentHeight: rxTextP2.paintedHeight

                                        ScrollBar.vertical: ScrollBar {
                                            policy: ScrollBar.AsNeeded
                                        }

                                        TextEdit {
                                            id: rxTextP2
                                            readOnly: true
                                            wrapMode: TextEdit.Wrap
                                            width: flickP2.width
                                            font.family: Theme.monoFamily
                                            color: Theme.textPrimary
                                            text: rxLogP2

                                            onTextChanged: {
                                                flickP2.contentY = Math.max(0, flickP2.contentHeight - flickP2.height)
                                            }
                                        }
                                    }

                                    RowLayout {
                                        Layout.alignment: Qt.AlignRight
                                        Button {
                                            text: "Clear"
                                            onClicked: rxLogP2 = ""
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

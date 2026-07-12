import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "../Items"

// Embedded RFD900x single/dual port terminal — EVA-02 diagnostics skin.
Pane {
    id: radioWin
    padding: 0
    font.family: Theme.fontFamily

    // EVA panel: dark red surface, 1px border, 3px red top accent. Radius 0.
    background: Rectangle {
        color: Theme.surface
        border.color: Theme.border
        border.width: Theme.strokePanel
        radius: 0

        Rectangle {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            height: Theme.accentStroke
            color: Theme.accent
        }
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

    // ---------- Reusable EVA-styled controls (file-local) ----------

    // Dim outline utility button (REFRESH PORTS / CLEAR RX / DISCONNECT).
    component DimButton: ClippedButton {
        cut: 0
        bold: false
        fontSize: 10
        fontLetterSpacing: 2
        verticalPadding: 5
        horizontalPadding: 12
        fillColor: "transparent"
        hoverFillColor: "transparent"
        textColor: Theme.btnSecondaryText
        hoverTextColor: Theme.accentMuted
        borderColor: Theme.btnSecondaryBorder
        hoverBorderColor: Theme.accent
    }

    // Flat monospace combo on the deep console fill.
    component EvaCombo: ComboBox {
        id: ec
        hoverEnabled: true
        font.family: Theme.monoFamily
        font.pixelSize: 12

        background: Rectangle {
            implicitWidth: 90
            implicitHeight: 30
            radius: 0
            color: Theme.sceneBackground
            border.width: 1
            border.color: ec.activeFocus || ec.hovered ? Theme.accent : Theme.divider
        }

        contentItem: Text {
            leftPadding: 8
            rightPadding: ec.indicator.width + 6
            text: ec.displayText
            font: ec.font
            color: Theme.textPrimary
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
        }
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

    // ---------- Port status line ----------

    function titleText() {
        if (singleMode) {
            const n = bridge.portName(rxWhich) || "—"
            const b = bridge.baudRate(rxWhich) || "—"
            return "P" + rxWhich + ": " + n + " @ " + b
        } else {
            const tn = bridge.portName(txWhich) || "—"
            const rn = bridge.portName(rxWhich) || "—"
            const tb = bridge.baudRate(txWhich) || "—"
            const rb = bridge.baudRate(rxWhich) || "—"
            return "TX(P" + txWhich + "): " + tn + " @ " + tb +
                   "  |  RX(P" + rxWhich + "): " + rn + " @ " + rb
        }
    }
    property string toolbarTitle: titleText()

    // ---------- Error popup ----------

    Popup {
        id: errorPopup
        x: 16; y: 16
        modal: false; focus: false
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        padding: 0
        background: Rectangle {
            radius: 0
            color: Theme.surfaceSolid
            border.width: 1
            border.color: Theme.accent
        }
        contentItem: Label {
            id: errorLabel
            padding: 12
            wrapMode: Text.Wrap
            color: Theme.textPrimary
            font.family: Theme.fontFamily
            font.pixelSize: Theme.fontBody
            text: ""
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
        anchors.margins: Theme.paddingMd
        anchors.topMargin: Theme.paddingMd + Theme.accentStroke
        spacing: 12

        // Header row: heading + JP label | mode switch | REFRESH PORTS
        RowLayout {
            Layout.fillWidth: true
            spacing: 12

            Text {
                text: singleMode ? "RFD900X — SINGLE-PORT" : "RFD900X — DUAL-PORT"
                color: Theme.accentMuted
                font.family: Theme.fontFamily
                font.pixelSize: Theme.fontH2
                font.bold: true
                font.letterSpacing: 2
            }

            JpText {
                text: "無線通信"
                color: Theme.textTertiary
            }

            Item { Layout.fillWidth: true }

            Text {
                text: "MODE"
                color: Theme.textTertiary
                font.family: Theme.fontFamily
                font.pixelSize: 10
                font.letterSpacing: 2
            }

            EvaCombo {
                id: modeBox
                model: ["Single Port", "Dual Port"]
                Layout.preferredWidth: 120
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

            DimButton {
                text: "REFRESH PORTS"
                onClicked: bridge.refreshPorts()
            }
        }

        // Live port status line — 'P1: cu.usbserial-BG013W95 @ 57600'
        Text {
            Layout.fillWidth: true
            text: radioWin.toolbarTitle
            color: Theme.textSecondary
            font.family: Theme.monoFamily
            font.pixelSize: 11
            elide: Text.ElideRight
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: singleMode ? 0 : 1

            // ===== Page 0: Single-Port Mode =====
            Item {
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 12

                    // Connection bar: RX + TX | which | port | baud | CONNECTED chip | (dis)connect
                    Rectangle {
                        Layout.fillWidth: true
                        radius: 0
                        color: Theme.surfaceElevated
                        border.width: 1
                        border.color: Theme.divider
                        implicitHeight: connRow.implicitHeight + 20

                        RowLayout {
                            id: connRow
                            anchors.fill: parent
                            anchors.leftMargin: 12
                            anchors.rightMargin: 12
                            spacing: 10

                            Text {
                                text: "RX + TX"
                                color: Theme.textTertiary
                                font.family: Theme.fontFamily
                                font.pixelSize: 10
                                font.letterSpacing: 2
                            }

                            EvaCombo {
                                id: singleWhichSel
                                model: [1, 2]
                                Layout.preferredWidth: 64
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

                            EvaCombo {
                                id: singlePortSel
                                model: bridge.ports
                                Layout.fillWidth: true
                                Layout.minimumWidth: 110
                                Layout.maximumWidth: 200
                                Component.onCompleted: {
                                    const name = bridge.portName(rxWhich)
                                    const i = model.indexOf(name)
                                    currentIndex = (i >= 0 ? i : -1)
                                }
                            }

                            EvaCombo {
                                id: singleBaudSel
                                model: baudList
                                Layout.preferredWidth: 100
                                Component.onCompleted: {
                                    const i = baudList.indexOf(bridge.baudRate(rxWhich) || 57600)
                                    currentIndex = (i >= 0 ? i : 0)
                                }
                            }

                            Item { Layout.fillWidth: true }

                            // Green CONNECTED chip (live link state)
                            Rectangle {
                                visible: singleConnected
                                radius: 0
                                color: Theme.successBg
                                border.width: 1
                                border.color: Theme.successBorder
                                implicitWidth: connChipText.implicitWidth + 20
                                implicitHeight: connChipText.implicitHeight + 6

                                Text {
                                    id: connChipText
                                    anchors.centerIn: parent
                                    text: "CONNECTED"
                                    color: Theme.success
                                    font.family: Theme.fontFamilySemiBold
                                    font.pixelSize: 11
                                    font.letterSpacing: 2
                                }
                            }

                            ClippedButton {
                                id: singleConnBtn
                                cut: 0
                                bold: false
                                fontSize: 10
                                fontLetterSpacing: 1
                                verticalPadding: 4
                                horizontalPadding: 10
                                fillColor: singleConnected ? "transparent" : "#24ff3b2f"
                                hoverFillColor: singleConnected ? "transparent" : "#4dff3b2f"
                                borderColor: singleConnected ? Theme.btnSecondaryBorder : Theme.accent
                                hoverBorderColor: Theme.accent
                                textColor: singleConnected ? Theme.btnSecondaryText : Theme.accentMuted
                                hoverTextColor: Theme.accent
                                text: singleConnected ? "DISCONNECT" : "CONNECT"
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
                        }
                    }

                    // TX line
                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 4

                        Text {
                            text: "TYPE & SEND (PRESS ENTER)"
                            color: Theme.textSecondary
                            font.family: Theme.fontFamily
                            font.pixelSize: 10
                            font.letterSpacing: 2
                        }

                        TextField {
                            id: singleInput
                            placeholderText: "type command and press enter…"
                            Layout.fillWidth: true
                            enabled: singleConnected
                            focus: true
                            color: Theme.textPrimary
                            placeholderTextColor: Theme.textTertiary
                            font.family: Theme.monoFamily
                            font.pixelSize: 14
                            leftPadding: 12
                            rightPadding: 12
                            topPadding: 10
                            bottomPadding: 10
                            background: Rectangle {
                                radius: 0
                                color: Theme.surfaceElevated
                                border.width: 1
                                border.color: singleInput.activeFocus ? Theme.accent : Theme.divider
                            }
                            onAccepted: {
                                if (text.length && bridge.isConnected(rxWhich)) {
                                    bridge.sendText(rxWhich, text)
                                    text = ""
                                }
                            }
                        }
                    }

                    // RX console
                    Text {
                        text: "RECEIVED"
                        color: Theme.textSecondary
                        font.family: Theme.fontFamily
                        font.pixelSize: 10
                        font.letterSpacing: 2
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumHeight: 320
                        radius: 0
                        color: Theme.sceneBackground
                        border.width: 1
                        border.color: Theme.divider

                        Flickable {
                            id: flickSingle
                            anchors.fill: parent
                            anchors.margins: 10
                            clip: true

                            contentWidth: singleRxText.paintedWidth
                            contentHeight: singleRxText.paintedHeight

                            ScrollBar.vertical: ThemedScrollBar { }

                            TextEdit {
                                id: singleRxText
                                readOnly: true
                                wrapMode: TextEdit.Wrap
                                width: flickSingle.width
                                font.family: Theme.monoFamily
                                font.pixelSize: 12
                                color: Theme.success
                                text: (rxWhich === 1 ? rxLogP1 : rxLogP2)

                                onTextChanged: {
                                    flickSingle.contentY = Math.max(0, flickSingle.contentHeight - flickSingle.height)
                                }
                            }
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true

                        Item { Layout.fillWidth: true }

                        DimButton {
                            text: "CLEAR RX"
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

            // ===== Page 1: Dual-Port Mode =====
            Item {
                id: dualModePage

                property string rxLogP1: ""
                property string rxLogP2: ""
                property bool   p1Connected: bridge.isConnected(1)
                property bool   p2Connected: bridge.isConnected(2)

                RowLayout {
                    anchors.fill: parent
                    spacing: 12

                    // ------------------- PORT 1 COLUMN -------------------
                    Frame {
                        Layout.fillWidth: true
                        Layout.fillHeight: true

                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8

                            Label { text: "Port 1"; font.bold: true; padding: 4 }

                            Flow {
                                Layout.fillWidth: true
                                spacing: 8

                                EvaCombo {
                                    id: portSel1
                                    model: bridge.ports
                                    width: 150
                                    Component.onCompleted: {
                                        const name = bridge.portName(1)
                                        const i = model.indexOf(name)
                                        currentIndex = (i >= 0 ? i : -1)
                                    }
                                }

                                EvaCombo {
                                    id: baudSel1
                                    model: [57600, 115200]
                                    width: 90
                                    Component.onCompleted: {
                                        const i = baudSel1.model.indexOf(bridge.baudRate(1) || 57600)
                                        currentIndex = (i >= 0 ? i : 0)
                                    }
                                }

                                Button {
                                    id: connBtn1
                                    text: p1Connected ? "Disconnect" : "Connect"
                                    width: Math.max(implicitWidth, 100)
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

                                Label {
                                    text: p1Connected ? "Connected" : "—"
                                    color: p1Connected ? Theme.success : Theme.textTertiary
                                    topPadding: 6
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

                                        ScrollBar.vertical: ThemedScrollBar {
                                            policy: ScrollBar.AsNeeded
                                        }

                                        TextEdit {
                                            id: rxTextP1
                                            readOnly: true
                                            wrapMode: TextEdit.Wrap
                                            width: flickP1.width
                                            font.family: Theme.monoFamily
                                            color: Theme.success
                                            text: rxLogP1

                                            onTextChanged: {
                                                flickP1.contentY = Math.max(0, flickP1.contentHeight - flickP1.height)
                                            }
                                        }
                                    }

                                    RowLayout {
                                        Layout.alignment: Qt.AlignRight
                                        DimButton {
                                            text: "CLEAR"
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

                            Flow {
                                Layout.fillWidth: true
                                spacing: 8

                                EvaCombo {
                                    id: portSel2
                                    model: bridge.ports
                                    width: 150
                                    Component.onCompleted: {
                                        const name = bridge.portName(2)
                                        const i = model.indexOf(name)
                                        currentIndex = (i >= 0 ? i : -1)
                                    }
                                }

                                EvaCombo {
                                    id: baudSel2
                                    model: [57600, 115200]
                                    width: 90
                                    Component.onCompleted: {
                                        const i = baudSel2.model.indexOf(bridge.baudRate(2) || 57600)
                                        currentIndex = (i >= 0 ? i : 0)
                                    }
                                }

                                Button {
                                    id: connBtn2
                                    text: p2Connected ? "Disconnect" : "Connect"
                                    width: Math.max(implicitWidth, 100)
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

                                Label {
                                    text: p2Connected ? "Connected" : "—"
                                    color: p2Connected ? Theme.success : Theme.textTertiary
                                    topPadding: 6
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

                                        ScrollBar.vertical: ThemedScrollBar {
                                            policy: ScrollBar.AsNeeded
                                        }

                                        TextEdit {
                                            id: rxTextP2
                                            readOnly: true
                                            wrapMode: TextEdit.Wrap
                                            width: flickP2.width
                                            font.family: Theme.monoFamily
                                            color: Theme.success
                                            text: rxLogP2

                                            onTextChanged: {
                                                flickP2.contentY = Math.max(0, flickP2.contentHeight - flickP2.height)
                                            }
                                        }
                                    }

                                    RowLayout {
                                        Layout.alignment: Qt.AlignRight
                                        DimButton {
                                            text: "CLEAR"
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

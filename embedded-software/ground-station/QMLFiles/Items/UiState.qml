pragma Singleton
import QtQuick
import QtCore

// Pure-UI application state for the EVA theme. No backend coupling —
// telemetry, commands, and serial state stay on the C++ context properties.
QtObject {
    id: uiState

    // Anime mode: full EVA dressing (side strips + Japanese accents).
    // Normal mode keeps the palette but hides all Japanese text and strips.
    property bool animeMode: true

    // Set by the ABORT command button, cleared by ACKNOWLEDGE / RESET.
    property bool alarmActive: false

    // Boot overlay teardown (flips true when the boot sequence finishes).
    property bool booted: false

    // Skip the boot sequence entirely (persisted config).
    property bool skipBoot: false

    // Cosmetic pilot-sync readout shown in the header / side strip.
    readonly property real syncRate: 98.7

    readonly property Settings _settings: Settings {
        category: "ui"
        property alias animeMode: uiState.animeMode
        property alias skipBoot: uiState.skipBoot
    }
}

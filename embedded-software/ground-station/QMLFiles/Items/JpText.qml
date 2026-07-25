import QtQuick

// Japanese accent label (Shippori Mincho B1). Automatically hidden when
// anime mode is off — every katakana/kanji label in the app goes through this.
Text {
    visible: UiState.animeMode && text.length > 0
    font.family: Theme.japaneseFamily
    font.pixelSize: 12
    color: Theme.accent
}

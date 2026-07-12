pragma Singleton
import QtQuick
import QtCore

// Anime-cockpit design tokens. Three pilot themes (Asuka / Rei / Mitsuri)
// re-skin the ENTIRE palette at runtime by switching `pilot`; every color
// property below is derived from the active palette, so components that bind
// to Theme.* re-skin automatically. Amber warning stripes, the green
// OK/health color, and the alarm-red abort overlay are intentionally NOT
// themed (shared across all pilots). Property names are unchanged from the
// single-theme version so existing panels need no edits.
QtObject {
    id: theme

    // ── Bundled fonts ──
    readonly property FontLoader _chakra:        FontLoader { source: "../../Resources/fonts/ChakraPetch-Regular.ttf" }
    readonly property FontLoader _chakraMedium:  FontLoader { source: "../../Resources/fonts/ChakraPetch-Medium.ttf" }
    readonly property FontLoader _chakraSemi:    FontLoader { source: "../../Resources/fonts/ChakraPetch-SemiBold.ttf" }
    readonly property FontLoader _chakraBold:    FontLoader { source: "../../Resources/fonts/ChakraPetch-Bold.ttf" }
    readonly property FontLoader _mincho:        FontLoader { source: "../../Resources/fonts/ShipporiMinchoB1-Bold.ttf" }
    readonly property FontLoader _minchoHeavy:   FontLoader { source: "../../Resources/fonts/ShipporiMinchoB1-ExtraBold.ttf" }
    readonly property FontLoader _mono:          FontLoader { source: "../../Resources/fonts/JetBrainsMono-Regular.ttf" }
    readonly property FontLoader _monoBold:      FontLoader { source: "../../Resources/fonts/JetBrainsMono-Bold.ttf" }

    // ── Pilot / character theme selection (persisted) ──
    // 0 = Asuka (EVA-02), 1 = Rei (EVA-00), 2 = Mitsuri (Love Hashira).
    property int pilot: 0

    // Per-pilot token palettes + character metadata. `panel` alpha 0.92 -> 0xEB.
    readonly property var _palettes: [
        {   // ASUKA — EVA-02 (red/amber/black)
            cRed: "#ff3b2f", cRed2: "#ff6a3d", bgPage: "#0c0505", bgTile: "#12100a",
            bgDeep: "#0d0705", bgBtn: "#1c0b09", bgBoot: "#060302", stripeDark: "#171008",
            bd1: "#5a1c12", bd2: "#3d1710", tx1: "#ffe9dc", tx2: "#c98a6e", tx3: "#8a4a38",
            tx4: "#d8b8a4", strip1: "#1a0908", strip2: "#240b08", panel: "#eb1c0b09",
            img: "asuka.png", imgWidth: 195, unitKanji: "弐号機",
            unitLabel: "EVA UNIT-02 · SOURYU", pilotName: "式波・アスカ・ラングレー"
        },
        {   // REI — EVA-00 (blue)
            cRed: "#4a86ff", cRed2: "#7dabff", bgPage: "#04060c", bgTile: "#0a0e18",
            bgDeep: "#05070d", bgBtn: "#0b1220", bgBoot: "#020308", stripeDark: "#0e1218",
            bd1: "#1e3a66", bd2: "#152a4a", tx1: "#e8f1ff", tx2: "#8fa8cc", tx3: "#4a6a94",
            tx4: "#b9c9e2", strip1: "#080d18", strip2: "#0a1428", panel: "#eb090e1a",
            img: "rei-cut.png", imgWidth: 200, unitKanji: "零号機",
            unitLabel: "EVA UNIT-00 · AYANAMI", pilotName: "綾波・レイ"
        },
        {   // MITSURI — Demon Slayer, Love Hashira (pink accent, green secondary)
            cRed: "#ff5fa2", cRed2: "#93d94a", bgPage: "#120810", bgTile: "#1c0f18",
            bgDeep: "#0f0a10", bgBtn: "#231320", bgBoot: "#0a050a", stripeDark: "#1a0f16",
            bd1: "#5c2a48", bd2: "#3a1c30", tx1: "#ffe3ef", tx2: "#d99cc0", tx3: "#8a5c76",
            tx4: "#e6c4d8", strip1: "#1e0f1a", strip2: "#2a1526", panel: "#eb1c0f18",
            img: "mitsuri.png", imgWidth: 176, unitKanji: "恋柱",
            unitLabel: "DEMON SLAYER · LOVE HASHIRA", pilotName: "甘露寺・蜜璃"
        }
    ]
    readonly property var _p: _palettes[pilot]

    // Character metadata for the active pilot (used by the side strips).
    readonly property var pilots: _palettes
    readonly property string pilotImage:  _p.img          // filename under Resources/images/
    readonly property int    pilotImageWidth: _p.imgWidth
    readonly property string pilotUnitKanji:  _p.unitKanji
    readonly property string pilotUnitLabel:  _p.unitLabel
    readonly property string pilotName:       _p.pilotName

    // Accent tints derived from the active accent (re-skin with the palette).
    function _tint(a) { return Qt.rgba(accent.r, accent.g, accent.b, a) }

    // ── Surfaces ──
    readonly property color background:      _p.bgPage
    readonly property color surface:         _p.panel        // panel fill ~92% alpha
    readonly property color surfaceSolid:    _p.bgBtn
    readonly property color surfaceElevated: _p.bgTile        // inner tile / input fill
    readonly property color surfaceInset:    _p.bgTile
    readonly property color sceneBackground: _p.bgDeep         // deep viewport fill
    readonly property color bootBg:          _p.bgBoot

    // ── Borders ──
    readonly property color border:      _p.bd1               // panel border
    readonly property color borderLight: accent               // hover border
    readonly property color divider:     _p.bd2               // inner border / rules

    // ── Text ──
    readonly property color textPrimary:   _p.tx1
    readonly property color textSecondary: _p.tx2
    readonly property color textTertiary:  _p.tx3             // dim / tile labels
    readonly property color logBody:       _p.tx4             // console log body text

    // ── Accent ──
    readonly property color accent:       _p.cRed             // primary accent
    readonly property color accentMuted:  _p.cRed2            // hover / link / accent-2
    readonly property color accentSubtle: _tint(0.12)         // selected accent tint
    readonly property color accentTint:      _tint(0.14)      // action-button idle fill
    readonly property color accentTintHover: _tint(0.30)      // action-button hover fill
    readonly property color accentTintFaint: _tint(0.06)      // faint inset stripes
    readonly property color accentGlow:       _tint(0.45)     // figure / label glow
    readonly property color accentGlowStrong: _tint(0.60)     // boot-title glow

    // ── Status (SHARED — never themed) ──
    readonly property color success:     "#74e05a"
    readonly property color successText: "#74e05a"
    readonly property color successBg:   "#1a74e05a"          // rgba(116,224,90,0.1)
    readonly property color warn:        "#ffb400"
    readonly property color warnText:    "#ffb400"
    readonly property color warnBg:      "#1affb400"          // rgba(255,180,0,0.1)
    readonly property color danger:      "#ff2413"
    readonly property color dangerText:  "#ff2413"
    readonly property color dangerBg:    "#2eff2413"          // rgba(255,36,19,0.18)

    // ── Extras ──
    readonly property color alarm:        "#ff2413"           // abort overlay (shared)
    readonly property color amber:        "#ffb400"           // warning stripes (shared)
    readonly property color gridLine:     accent              // backdrop / viewport grid
    readonly property color logoChip:     "#f4ede0"           // shared light logo plate
    readonly property color stripeDark:   _p.stripeDark       // warning stripe dark band
    readonly property color successBorder:"#2c5a22"
    readonly property color abortStripeTint: _tint(0.16)      // ABORT button stripe fill

    // ── Side-strip gradient stops ──
    readonly property color strip1: _p.strip1
    readonly property color strip2: _p.strip2

    // ── Typography ──
    readonly property string fontFamily:         _chakra.name
    readonly property string fontFamilyMedium:   _chakraMedium.name
    readonly property string fontFamilySemiBold: _chakraSemi.name
    readonly property string monoFamily:         _mono.name
    readonly property string japaneseFamily:     _mincho.name
    readonly property string japaneseFamilyHeavy:_minchoHeavy.name

    readonly property int fontH1:          24    // page title 24/700 ls2
    readonly property int fontH2:          15    // panel headings 15/700 ls2
    readonly property int fontBody:        13
    readonly property int fontCaption:     11
    readonly property int fontMetricValue: 20
    readonly property int fontMetricLabel: 9     // tile labels 9 ls2

    // ── Metrics ── (hard corners everywhere; angular cuts instead of radius)
    readonly property int radiusPanel:   0
    readonly property int radiusControl: 0
    readonly property int radiusCard:    0
    readonly property int strokePanel:   1
    readonly property int strokeControl: 1
    readonly property int accentStroke:  3     // panel top accent border
    readonly property int paddingSm:     6
    readonly property int paddingMd:    14     // panel padding
    readonly property int paddingLg:    18
    readonly property int paddingXl:    24
    readonly property int gridSpacing:  14     // gap between panels

    // ── Buttons: Primary (accent outline action) ──
    readonly property color btnPrimaryBg:     _p.bgBtn
    readonly property color btnPrimaryHover:  _tint(0.16)
    readonly property color btnPrimaryPress:  _tint(0.23)
    readonly property color btnPrimaryBorder: accent
    readonly property color btnPrimaryText:   accentMuted

    // ── Buttons: Secondary (dim outline) ──
    readonly property color btnSecondaryBg:     _p.bgBtn
    readonly property color btnSecondaryHover:  _tint(0.08)
    readonly property color btnSecondaryPress:  _tint(0.14)
    readonly property color btnSecondaryBorder: _p.bd1
    readonly property color btnSecondaryText:   _p.tx2

    // ── Transitions / blink cadence ──
    readonly property int transitionFast:   100
    readonly property int transitionNormal: 160
    readonly property int blinkPeriod:     1200   // stepped, no easing

    readonly property Settings _settings: Settings {
        category: "ui"
        property alias pilot: theme.pilot
    }
}

pragma Singleton
import QtQuick

// EVA-02 design tokens (red-black-amber). Property names are kept from the
// previous theme so existing panels restyle without code changes; EVA-specific
// tokens are added below.
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

    // ── Surfaces ──
    readonly property color background:      "#0c0505"
    readonly property color surface:         "#ea1c0b09"   // panel fill rgba(28,11,9,0.92)
    readonly property color surfaceSolid:    "#1c0b09"
    readonly property color surfaceElevated: "#12100a"     // inner tile / input fill
    readonly property color surfaceInset:    "#12100a"
    readonly property color sceneBackground: "#0d0705"     // deep viewport fill

    // ── Borders ──
    readonly property color border:      "#5a1c12"         // panel border
    readonly property color borderLight: "#ff3b2f"         // hover border
    readonly property color divider:     "#3d1710"         // inner border / rules

    // ── Text ──
    readonly property color textPrimary:   "#ffe9dc"
    readonly property color textSecondary: "#c98a6e"
    readonly property color textTertiary:  "#8a4a38"       // dim / tile labels

    // ── Accent ──
    readonly property color accent:       "#ff3b2f"        // primary red
    readonly property color accentMuted:  "#ff6a3d"        // hover / link red-orange
    readonly property color accentSubtle: "#1fff3b2f"      // selected red tint

    // ── Status ──
    readonly property color success:     "#74e05a"
    readonly property color successText: "#74e05a"
    readonly property color successBg:   "#1a74e05a"       // rgba(116,224,90,0.1)
    readonly property color warn:        "#ffb400"
    readonly property color warnText:    "#ffb400"
    readonly property color warnBg:      "#1affb400"       // rgba(255,180,0,0.1)
    readonly property color danger:      "#ff2413"
    readonly property color dangerText:  "#ff2413"
    readonly property color dangerBg:    "#2eff2413"       // rgba(255,36,19,0.18)

    // ── EVA extras ──
    readonly property color alarm:        "#ff2413"
    readonly property color amber:        "#ffb400"
    readonly property color gridLine:     "#ff5a3c"        // backdrop grid (draw at 5%)
    readonly property color logoChip:     "#f4ede0"
    readonly property color stripeDark:   "#171008"        // warning stripe dark band
    readonly property color successBorder:"#2c5a22"
    readonly property color abortStripeTint: "#29ff3b2f"   // rgba(255,59,47,0.16)

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

    // ── Buttons: Primary (red outline action) ──
    readonly property color btnPrimaryBg:     "#1c0b09"
    readonly property color btnPrimaryHover:  "#2aff3b2f"
    readonly property color btnPrimaryPress:  "#3aff3b2f"
    readonly property color btnPrimaryBorder: "#ff3b2f"
    readonly property color btnPrimaryText:   "#ff6a3d"

    // ── Buttons: Secondary (dim outline) ──
    readonly property color btnSecondaryBg:     "#1c0b09"
    readonly property color btnSecondaryHover:  "#241008"
    readonly property color btnSecondaryPress:  "#2b130a"
    readonly property color btnSecondaryBorder: "#5a1c12"
    readonly property color btnSecondaryText:   "#c98a6e"

    // ── Transitions / blink cadence ──
    readonly property int transitionFast:   100
    readonly property int transitionNormal: 160
    readonly property int blinkPeriod:     1200   // stepped, no easing
}

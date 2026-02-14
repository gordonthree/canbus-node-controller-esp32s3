#ifndef COLOR_PALETTE_H_
#define COLOR_PALETTE_H_

#include <FastLED.h>

/** * @brief Global index-based palette.
 * @note Transmit the index (0-31) over CAN to save bandwidth.
 */
// const CRGB SystemPalette[32] = {
//     CRGB::Black,        CRGB::White,        CRGB::Red,          CRGB::Green,
//     CRGB::Blue,         CRGB::Yellow,       CRGB::Cyan,         CRGB::Magenta,
//     CRGB::Orange,       CRGB::Purple,       CRGB::Lime,         CRGB::DarkGreen,
//     CRGB::DeepPink,     CRGB::HotPink,      CRGB::DarkOrange,   CRGB::LightSkyBlue,
//     CRGB::DodgerBlue,   CRGB::DarkViolet,   CRGB::DarkRed,      CRGB::FireBrick,
//     CRGB::Aqua,         CRGB::Aquamarine,   CRGB::Teal,         CRGB::Gold,
//     CRGB::Salmon,       CRGB::Lavender,     CRGB::Maroon,       CRGB::Olive,
//     CRGB::SteelBlue,    CRGB::Chocolate,    CRGB::OrangeRed,    CRGB::RoyalBlue
// };

// const CRGB SystemPalette[32] = {
//     CRGB::Black,         CRGB::Red4,         CRGB::Green4,       CRGB::Blue4,        CRGB::Yellow4,      CRGB::Cyan4,        CRGB::Magenta4,
//     CRGB::Gray25,        CRGB::Red3,         CRGB::Green3,       CRGB::Blue3,        CRGB::Yellow3,      CRGB::Cyan3,        CRGB::Magenta3,
//     CRGB::Gray50,        CRGB::Red2,         CRGB::Green2,       CRGB::Blue2,        CRGB::Yellow2,      CRGB::Cyan2,        CRGB::Magenta2,
//     CRGB::White,         CRGB::Red,          CRGB::Green,        CRGB::Blue,         CRGB::Yellow,       CRGB::Cyan,         CRGB::Magenta,
// };

/** * @brief 32-color palette organized for an 8x4 grid (8 columns x 4 rows).
 * @details Columns: Grayscale, Red, Green, Blue, Yellow, Cyan, Magenta, Orange.
 * Rows: Darkest (Top) to Lightest (Bottom).
 */
const CRGB SystemPalette[32] = {
    /* Row 1: Darkest */
    CRGB(40,40,40),   CRGB::Red4,    CRGB::Green4,  CRGB::Blue4,   CRGB::Yellow4, CRGB::Cyan4,   CRGB::Magenta4, CRGB(128,64,0),
    /* Row 2: Medium-Dark */
    CRGB::Gray25,     CRGB::Red3,    CRGB::Green3,  CRGB::Blue3,   CRGB::Yellow3, CRGB::Cyan3,   CRGB::Magenta3, CRGB::DarkOrange,
    /* Row 3: Medium-Light */
    CRGB::Gray50,     CRGB::Red2,    CRGB::Green2,  CRGB::Blue2,   CRGB::Yellow2, CRGB::Cyan2,   CRGB::Magenta2, CRGB::Orange,
    /* Row 4: Brightest */
    CRGB::White,      CRGB::Red,     CRGB::Green1,   CRGB::Blue,    CRGB::Yellow,  CRGB::Cyan,    CRGB::Magenta,  CRGB::Gold
};
#endif /* END COLOR_PALETTE_H_ */
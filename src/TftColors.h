#pragma once
#include <Arduino.h>

// Couleurs RGB565 communes aux pilotes d'écran (Ili9225 / Ili9341).
static const uint16_t C_BLACK  = 0x0000, C_WHITE = 0xFFFF, C_RED = 0xF800;
static const uint16_t C_GREEN  = 0x07E0, C_BLUE  = 0x001F, C_YELLOW = 0xFFE0;
static const uint16_t C_CYAN   = 0x07FF, C_GREY  = 0x8410, C_DKGREY = 0x39E7;
static const uint16_t C_ORANGE = 0xFD20;

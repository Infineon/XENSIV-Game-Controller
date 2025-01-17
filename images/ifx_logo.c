/*********************************************************************
*            (c) 1998 - 2023 SEGGER Microcontroller GmbH             *
*        Solutions for real time microcontroller applications        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
* C-file generated by                                                *
*                                                                    *
*        Bitmap Converter for emWin (Demo version) V6.36.            *
*        Compiled Dec 18 2023, 11:48:17                              *
*                                                                    *
*        (c) 1998 - 2023 SEGGER Microcontroller GmbH                 *
*                                                                    *
*        May not be used in a product                                *
*                                                                    *
**********************************************************************
*                                                                    *
* Source file: ifx_logo                                              *
* Dimensions:  128 * 55                                              *
* NumColors:   2                                                     *
* NumBytes:    920                                                   *
*                                                                    *
**********************************************************************
*/

#include <stdlib.h>

#include "GUI.h"

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif


/*********************************************************************
*
*       Palette
*
*  Description
*    The following are the entries of the palette table.
*    The entries are stored as a 32-bit values of which 24 bits are
*    actually used according to the following bit mask: 0xBBGGRR
*
*    The lower   8 bits represent the Red   component.
*    The middle  8 bits represent the Green component.
*    The highest 8 bits represent the Blue  component.
*/
static GUI_CONST_STORAGE GUI_COLOR _Colorsifx_logo[] = {
#if (GUI_USE_ARGB == 0)
  0x000000, 0xFFFFFF
#else
  0xFF000000, 0xFFFFFFFF
#endif

};

static GUI_CONST_STORAGE GUI_LOGPALETTE _Palifx_logo = {
  2,  // Number of entries
  0,  // No transparency
  (const LCD_COLOR *)&_Colorsifx_logo[0]
};

static GUI_CONST_STORAGE unsigned char _acifx_logo[] = {
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ________, _____XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ________, ____XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX______, ______XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, ________, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX_____, ______XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXX___XX, X_______, ___XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XX_____X, X______X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, X_______, XX___XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, X_______, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXX_, X_______, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXX___, X_______, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXX_____, XX_____X, XXXXXXXX, XXXXXXXX, XXXX____, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XX______, _XXX_XXX, XXXXXXXX, XXXXXXXX, XXXX____, __XX____, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, ________, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXX_____, __XX____, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXX_, _______X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX____X, XXXX____, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXX__, ______XX, XXX___XX, XXXXXXXX, XXXXXXXX, XXX___XX, XXXX____, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXX___, _____XXX, XXX___XX, XXXXXXXX, XXXXXXXX, XXX___XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXX____, ___XXXXX, XXX___XX, XXXXXXXX, XXXXXXXX, XXX___XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXX_____, ___XXXXX, XXX___XX, XXXX__XX, X__XXXXX, XXX____X, XXXXXXX_, XXXXXX_X, XXX__XXX, XXXXXXX_, ___XXXXX, XXXXXX__, _XXXXXXX, XX_XXXX_, _XXXXXXX, XXXXXXXX,
  XXX_____, __XXXXXX, XXX___XX, XX____X_, _____XXX, XX______, _XXX____, XXXX____, X______X, XXXXXX__, _____XXX, XXXX____, ___XXXXX, ____X___, ___XXXXX, XXXXXXXX,
  XX______, _XXXXXXX, XXX___XX, XXX_____, ______XX, XX______, _XXX____, XXXX____, _______X, XXXXX___, _____XXX, XXX_____, ____XXXX, ________, ___XXXXX, XXXXXXXX,
  XX______, XXXXXXXX, XXX___XX, XXX_____, ______XX, XX______, _XXX____, XXXX____, ________, XXXX____, XX____XX, XX_____X, ____XXXX, X_______, ____XXXX, XXXXXXXX,
  X_______, XXXXXXXX, XXX___XX, XXX____X, XX____XX, XXX___XX, XXXX____, XXXX____, _XXX____, XXXX___X, XXX___XX, XX____XX, X____XXX, X____XXX, ____XXXX, XXXXXXXX,
  X_______, XXXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXX____X, XXX___XX, XX___XXX, X____XXX, X___XXXX, ____XXXX, XXXXXXXX,
  _______X, XXXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXX____X, XXX____X, X____XXX, XX___XXX, X___XXXX, ____XXXX, XXXXXXXX,
  _______X, XXXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXX____X, XXX____X, X____XXX, XX____XX, X___XXXX, ____XXXX, XXXXXXXX,
  _______X, XXXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXX_____, _______X, X____XXX, XX____XX, X___XXXX, ____XXXX, XXXXXXXX,
  _______X, XXXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXX_____, _______X, X____XXX, XX____XX, X___XXXX, ____XXXX, XXXXXXXX,
  _______X, XXXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXX_____, ______XX, X____XXX, XX____XX, X___XXXX, ____XXXX, XXXXXXXX,
  ________, XXXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXX____X, XXXXXXXX, X____XXX, XX____XX, X___XXXX, ____XXXX, XXXXXXXX,
  ________, XXXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXX____X, XXXXXXXX, X____XXX, XX___XXX, X___XXXX, ____XXXX, XXXXXXXX,
  X_______, XXXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXX____X, XXXXXXXX, XX___XXX, X____XXX, X___XXXX, ____XXXX, XXXXXXXX,
  X_______, _XXXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXXX___X, XXXXXXXX, XX____XX, X____XXX, X___XXXX, ____XXXX, XXXXXXXX,
  X_______, __XXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXXX____, _XX___XX, XX____XX, _____XXX, X___XXXX, ____XXXX, XXXXXXXX,
  XX______, __XXXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXXXX___, ______XX, XXX_____, ____XXXX, X___XXXX, ____XXXX, XXXXXXXX,
  XXX_____, ___XXXXX, XXX___XX, XXX___XX, XX____XX, XXX___XX, XXXX____, XXXXX___, XXXX____, XXXXX___, ______XX, XXXX____, ___XXXXX, X___XXXX, ____XXXX, XXXXXXXX,
  XXX_____, ____XXXX, XXX___XX, XXX___XX, XXX__XXX, XXX___XX, XXXXX__X, XXXXX___, XXXX___X, XXXXXXX_, ____XXXX, XXXXX___, __XXXXXX, X___XXXX, X__XXXXX, XXXXXXXX,
  XXXX____, _____XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXX___, ______XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXX__, ________, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXX_, ________, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX_XXXX,
  XXXXXXXX, X_______, ___XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X_XXXXXX,
  XXXXXXXX, XX______, ____XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, _XXXXXXX,
  XXXXXXXX, XXXX____, _______X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX__X, XXXXXXXX,
  XXXXXXXX, XXXXX___, ________, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX___XXX, XXXXXXXX,
  XXXXXXXX, XXXXXXX_, ________, ___XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, ___XXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, X_______, ______XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX____, _XXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXX____, ________, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ______XX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXX__, ________, ______XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX____, ____XXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, X_______, ________, ___XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, ________, _XXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX___, ________, ________, ___XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, ________, _____XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X_______, ________, ________, ________, ________, ________, ________, ________, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX___, ________, ________, ________, ________, ________, ________, _____XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX_____, ________, ________, ________, ________, _______X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX___, ________, ________, ______XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX
};

GUI_CONST_STORAGE GUI_BITMAP bmifx_logo = {
  128, // xSize
  55, // ySize
  16, // BytesPerLine
  1, // BitsPerPixel
  (const unsigned char *)_acifx_logo,  // Pointer to picture data (indices)
  &_Palifx_logo,  // Pointer to palette
  NULL
};

/*************************** End of file ****************************/

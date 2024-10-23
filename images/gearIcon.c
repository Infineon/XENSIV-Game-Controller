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
* Source file: gear                                                  *
* Dimensions:  22 * 26                                               *
* NumColors:   2                                                     *
* NumBytes:    118                                                   *
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
static GUI_CONST_STORAGE GUI_COLOR _Colorsgear[] = {
#if (GUI_USE_ARGB == 0)
  0x000000, 0xFFFFFF
#else
  0xFF000000, 0xFFFFFFFF
#endif

};

static GUI_CONST_STORAGE GUI_LOGPALETTE _Palgear = {
  2,  // Number of entries
  0,  // No transparency
  (const LCD_COLOR *)&_Colorsgear[0]
};

static GUI_CONST_STORAGE unsigned char _acgear[] = {
  _____X__, __X_____, ________,
  ___XXXX_, _XXXX___, ________,
  ___X__XX, X___X___, ________,
  ___X____, ___X____, ________,
  ___X____, ___XX___, ________,
  __XX__XX, X___X___, ________,
  XXX__XX_, _X___XX_, ________,
  X____X__, _X______, ________,
  XXX__XX_, _X______, ________,
  __XX__XX, X__XXXX_, ________,
  ___X____, ___X___X, ________,
  ___X____, ___X___X, ___X____,
  ___X__XX, XXX_____, XXXXX___,
  ______X_, ________, ____X___,
  _____XX_, ____XXX_, ____XX__,
  ______XX, ___XX_XX, ___XX___,
  _______X, __XX___X, ___X____,
  _______X, __X____X, X__X____,
  _______X, ___X___X, ___X____,
  _____XX_, ___XXXXX, ____X___,
  _____XX_, _____X__, ____XX__,
  ______X_, XX______, _XX_X___,
  ______XX, XXXX___X, XXXXX___,
  ________, ___X___X, ________,
  ________, ___X__XX, ________,
  ________, ___XXXX_, ________
};

GUI_CONST_STORAGE GUI_BITMAP bmgear = {
  22, // xSize
  26, // ySize
  3, // BytesPerLine
  1, // BitsPerPixel
  (const unsigned char *)_acgear,  // Pointer to picture data (indices)
  &_Palgear,  // Pointer to palette
  NULL
};

/*************************** End of file ****************************/
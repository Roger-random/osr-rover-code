/*
 * Draw on a RGB LED matrix panel using Adafruit's RGBmatrixPanel class.
 * The data are sets of fixed bitmaps, each bitmap is an index into a color
 * palette.
 */
#ifndef PaletteBitmap_h
#define PaletteBitmap_h

#include "RGBmatrixPanel.h"
class PaletteBitmap
{
  public:
    PaletteBitmap(bool debug);
    void draw_bitmap(int index, RGBmatrixPanel* panel);
};
#endif

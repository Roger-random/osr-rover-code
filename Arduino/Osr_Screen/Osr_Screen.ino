// testcolors demo for Adafruit RGBmatrixPanel library.
// Renders 512 colors on our 16x32 RGB LED matrix:
// http://www.adafruit.com/products/420
// Library supports 4096 colors, but there aren't that many pixels!  :)

// Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon
// for Adafruit Industries.
// BSD license, all text above must be included in any redistribution.

#include <RGBmatrixPanel.h>
//#include "screen_test.h"
#include "OsrScreen.h"

Screen screen(1);
int c = 0;
int* data = 0;

unsigned long last_update;
unsigned long hold_duration = 500;
int face = 0;

void setup() 
{
  Serial.begin(9600);
  screen.init_display();
  last_update = millis();
}

void loop() 
{
/*
  data = screen.get_data();
  if (data){
    screen.update_screen(data);
  }
  data = 0;
*/
  if (millis() > last_update + hold_duration) {
    screen.clear_face();
    screen.display_face(face++);
    last_update = millis();

    if (face > 4) {
      face = 0;
    }
  }
}

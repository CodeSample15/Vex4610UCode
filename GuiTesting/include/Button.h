/*
  Luke Crimi
  Button.h
  10/24/2021
*/

#include "vex.h"

class Button 
{
  public:
  //constructors
  Button();
  Button(int x, int y, int width, int height, std::string text, color buttonColor);
  
  void draw();
  bool isPressed();
  bool showing;

  private:
  int _x;
  int _y;
  int _width;
  int _height;
  std::string _text;
  color _buttonColor;
};
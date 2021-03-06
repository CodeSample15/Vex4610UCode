/*
  Luke Crimi
  Button.cpp
  10/24/2021
*/

#include "Button.h"
#include "vex.h"
#include <string>
#include <cstring>

using namespace vex;

Button::Button() {
  Button::_x = 0;
  Button::_y = 0;
  Button::_width = 0;
  Button::_height = 0;
  Button::_text = "";
  Button::showing = false;
}

Button::Button(int x, int y, int width, int height, std::string text, color buttonColor) {
  Button::_x = x;
  Button::_y = y;
  Button::_width = width;
  Button::_height = height;
  Button::_text = text;
  Button::_buttonColor = buttonColor;
  Button::showing = true;
}

void Button::draw() {
  if(Button::showing) {
    int screenWidth = 480;
    int screenHeight = 272;

    //set the text location to the center of the button
    int middleX = Button::_x + (Button::_width / 2);
    int middleY = Button::_y + (Button::_height / 2);
    double percentageX = middleX / screenWidth;
    double percentageY = middleY / screenHeight;

    //size of screen (rows and columns): 45 x 12
    middleX = (int)(1 * percentageX) - (int)(Button::_text.length()/2);
    middleY = (int)(1 * percentageY);

    //draw the button
    Brain.Screen.setFillColor(Button::_buttonColor);
    Brain.Screen.drawRectangle(Button::_x, Button::_y, Button::_width, Button::_height);

    //draw the text on the button
    Brain.Screen.setCursor(middleY, middleX);

    //converting the string to a character array
    char *letters = new char[1024];

    for(int i=0; i<Button::_text.length(); i++) {
      letters[i] = Button::_text[i];
    }

    //printing the string
    Brain.Screen.print(letters);

    delete[] letters;
  }
}

bool Button::isPressed() {
  bool xInRange = Brain.Screen.xPosition() >= Button::_x && Brain.Screen.xPosition() <= Button::_x + Button::_width;
  bool yInRange = Brain.Screen.yPosition() >= Button::_y && Brain.Screen.yPosition() <= Button::_y + Button::_height;

  return xInRange && yInRange;
}
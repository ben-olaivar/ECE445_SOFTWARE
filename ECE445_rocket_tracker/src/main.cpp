#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "SparkFun_Ublox_Arduino_Library.h" // http://librarymanager/All#SparkFun_u-blox_GNSS



// constants won't change. They're used here to set pin numbers:
const int down_button_pin = 2;  // the number of the pushbutton pin

// variables will change:
int down_button_state = 0;  // variable for reading the pushbutton status

int menu_change = 0;  // do we want to change what is being displayed
int menu_state = 0;   // the current menu being displayed

void setup() {
  Serial.begin(9600);   // open the serial port at 9600 bps:
  while(!Serial);       // wait for serial to start

  Serial.println("");
  Serial.println("--------------MENU--------------");
  Serial.println("> Compass <");
  Serial.println("Frequency Change");

  // initialize the pushbutton pin as an input:
  pinMode(down_button_pin, INPUT);

}

// menu_type corresponds to the menu options being selected
// 0 = default (select 'compass')
// 1 = select frequency change
void print_menu(int menu_type) {
  Serial.println("");
  Serial.println("--------------MENU--------------");

  if (menu_type == 0 || menu_type == 1) {
    Serial.println("> Compass <");
    Serial.println("Frequency Change");
  } else if (menu_type == 2) {
    Serial.println("Compass");
    Serial.println("> Frequency Change <");
  }
}


// the loop function runs over and over again forever
void loop() {
  // Serial.println(menu_state);
  // read the state of the pushbutton value:
  down_button_state = digitalRead(down_button_pin);

  // check if the pushbutton is pressed. If it is, the down_button_state is HIGH:
  if (down_button_state == LOW) {
    // don't change menu state
    // menu_state = 0;
    // menu_change = 0;


    if (menu_change) {
      print_menu(0);
    }

  } else {
    // frequency change
    if (menu_state != 2) {
      menu_change = 1;
      menu_state = 2;
    }

    if (menu_change) {
      print_menu(2);
      menu_change = 0;
    }
    
  }
}



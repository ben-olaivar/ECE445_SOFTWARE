#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "SparkFun_Ublox_Arduino_Library.h" // http://librarymanager/All#SparkFun_u-blox_GNSS



// constants won't change. They're used here to set pin numbers:
//pin mappings defined in pin.h
const int down_button_pin   = pushbutton_down;  // the port mapping of the "DOWN" pushbutton pin
const int up_button_pin     = pushbutton_up;    // the port mapping of the "UP"   pushbutton pin
const int one_button_pin    = pushbutton_1;     // the port mapping of the "1"    pushbutton pin
const int two_button_pin    = pushbutton_2;     // the port mapping of the "2"    pushbutton pin
const int three_button_pin  = pushbutton_3;     // the port mapping of the "3"    pushbutton pin
const int four_button_pin   = pushbutton_4;     // the port mapping of the "4"    pushbutton pin
const int five_button_pin   = pushbutton_5;     // the port mapping of the "5"    pushbutton pin
const int six_button_pin    = pushbutton_6;     // the port mapping of the "6"    pushbutton pin
const int seven_button_pin  = pushbutton_7;     // the port mapping of the "7"    pushbutton pin
const int eight_button_pin  = pushbutton_8;     // the port mapping of the "8"    pushbutton pin
const int nine_button_pin   = pushbutton_9;     // the port mapping of the "9"    pushbutton pin
const int zero_button_pin   = pushbutton_0;     // the port mapping of the "0"    pushbutton pin
const int dot_button_pin    = pushbutton_dot;   // the port mapping of the "DOT"  pushbutton pin
const int enter_button_pin  = pushbutton_enter; // the port mapping of the "ENTER"pushbutton pin

// variables will change:
int down_button_state   = 0;    // variable for reading the "DOWN"  pushbutton status
int up_button_state     = 0;    // variable for reading the "UP"    pushbutton status
int one_button_state    = 0;    // variable for reading the "ONE"   pushbutton status
int two_button_state    = 0;    // variable for reading the "TWO"   pushbutton status
int three_button_state  = 0;    // variable for reading the "THREE" pushbutton status
int four_button_state   = 0;    // variable for reading the "FOUR"  pushbutton status
int five_button_state   = 0;    // variable for reading the "FIVE"  pushbutton status
int six_button_state    = 0;    // variable for reading the "SIX"   pushbutton status
int seven_button_state  = 0;    // variable for reading the "SEVEN" pushbutton status
int eight_button_state  = 0;    // variable for reading the "EIGHT" pushbutton status
int nine_button_state   = 0;    // variable for reading the "NINE"  pushbutton status
int zero_button_state   = 0;    // variable for reading the "ZERO"  pushbutton status
int dot_button_state    = 0;    // variable for reading the "DOT"   pushbutton status
int enter_button_state  = 0;    // variable for reading the "ENTER" pushbutton status

int should_menu_change = 0;     // do we want to change what is being displayed
int menu_type = 0;             // the current menu being displayed

int main_m_row_num = 0;        // keep track of the main menu row number
int set_freq_row_num = 1;      // keep track of the set frequency row number
float t_frequency = 0.0;            // the current frequency 

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

// set frequency
void set_frequency(float t_freq) {
    t_frequency = t_freq;
}
  
// menu_type corresponds to the menu options being selected
// 0 = default (select 'compass')
// 1 = select frequency change
void print_menu(int m_type) {
  Serial.println("");
  Serial.println("--------------MENU--------------");

  switch (m_type) {
    case 0: // This is main menu selected Compass
        Serial.println("> Compass <");
        Serial.println("Frequency Change");
        break;
    case 1: // Main menu selected Frequency Change
        Serial.println("Compass");
        Serial.println("> Frequency Change <");
        break;  
    case 2: // This is sub menu of selected Compass
        Serial.println("Relevant Information:");
        Serial.println("> Back  <"); // this to mimic enter button
        break;
    case 3: // This is sub menu of selected Frequency Change
        Serial.println(t_frequency);
        Serial.println("> Set Frequency: <");
        Serial.println(" Back ");
        break;  
    case 4: // This is sub menu of selected Frequency Change
        Serial.println(t_frequency);
        Serial.println("Set Frequency:");
        Serial.println("> Back <");
        break;  
    case 5: // This is sub menu of Set Frequency Change
        Serial.println("Set Frequency:");
        break;
    default:
        break;
  }
}

void executeAction(){
    switch (menu_type) {
        case 0: // user pressed Enter on Compass
            // set the menu type to sub menu of selected Compass
            menu_type = 2;
            print_menu(menu_type);
            break;
        case 1: // User Pressed Enter on Frequency Change
            // set the menu type to sub menu of selected Frequency Change
            menu_type = 3;
            print_menu(menu_type);
            break;  
        case 2: // This is sub menu of selected Compass
            menu_type = 0; // go back to main menu
            print_menu(menu_type);
            break;
        case 3: // This is sub menu of selected Frequency Change
            menu_type = 5; // set frequency
            print_menu(menu_type);
            break;  
        case 4: // User pressed the back button from set frequency
            menu_type = 1; // go back to main menu
            print_menu(menu_type);
            break;
        case 5: // user entered the frequency & pressed enter to set the frequency
            set_frequency(t_frequency);
            menu_type = 4; // go back to set frequency
            print_menu(menu_type);
            break;
        default:
            break;
    }
}

// the loop function runs over and over again forever
void loop() {
  // Serial.println(menu_state);
  // read the state of the pushbutton value:
  down_button_state = digitalRead(down_button_pin);
  up_button_state   = digitalRead(up_button_pin);
  enter_button_state = digitalRead(enter_button_pin);


  // check if the pushbutton is pressed. If it is, the down_button_state is HIGH:
    if (down_button_state != LOW) {
        down_button_state = LOW;
        if ((menu_type == 0 || menu_type == 1) && main_m_row_num == 0){ // if we are at the main menu and the first row
            main_m_row_num += 1;
            menu_type = 1; // main menu has two menu 0 & 1 - 1 to highlight the Frequency Change 
            print_menu(menu_type);
        } else if ((menu_type == 3 || menu_type == 4) && set_freq_row_num == 1) {
            set_freq_row_num += 1;
            menu_type = 4; //menu type 3 & 4 to set frequency - 4 is to highlight the back button
            print_menu(menu_type);
        }
    } else if (up_button_state != LOW) {
        up_button_state = LOW;
        if ((menu_type == 0 || menu_type == 1) && main_m_row_num == 1){ // if we are at the main menu and the second row
            main_m_row_num -= 1;
            menu_type = 0; // main menu has two menu 0 & 1 - 0 to highlight Compass
            print_menu(menu_type);
        } else if ((menu_type == 3 || menu_type == 4) && set_freq_row_num == 2) {
            set_freq_row_num -= 1;
            menu_type = 3; //menu type 3 & 4 to set frequency - 3 is to highlight set frequency
            print_menu(menu_type);
        }
    } else if (enter_button_state != LOW) {
        enter_button_state = LOW;
        executeAction();

    } 
}

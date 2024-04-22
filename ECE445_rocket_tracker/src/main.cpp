#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
// #include "pins.h"
// #include <SparkFun_Ublox_Arduino_Library.h> // http://librarymanager/All#SparkFun_u-blox_GNSS
#include "gps.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include <RH_RF95.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define COMPASS_LENGTH 11


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// GPS setup
SFE_UBLOX_GPS myGPS;
float curr_longitude  = 0;
float curr_latitude   = 0;
long lastTime   = 0;

// Menu setup
int menu_type = -1;
const int menu_button_pin   = 2;  // pushbutton_enter; // the port mapping of the "ENTER"pushbutton pin
const int enter_button_pin  = 3;  // pushbutton_enter; // the port mapping of the "ENTER"pushbutton pin
const int down_button_pin   = 4;  // pushbutton_down;  // the port mapping of the "DOWN" pushbutton pin
const int up_button_pin     = 5;  // pushbutton_up;    // the port mapping of the "UP"   pushbutton pin

int menu_button_state   = 0;      // variable for reading the "ENTER" pushbutton status
int enter_button_state  = 0;      // variable for reading the "ENTER" pushbutton status
int down_button_state   = 0;      // variable for reading the "DOWN"  pushbutton status
int up_button_state     = 0;      // variable for reading the "UP"    pushbutton status



//!------------------------BEGIN COMPASS DISPLAY------------------------
/* angle_to_x_pos():
* Given some input number, update the distance data the user will see:
*
*                 "Distance: <distance_data>m"
*
* Params: 
*     - long distance_data: the distance in meters we'd like to show on the screen
*/
void display_distance_data(long distance_data) {
  int text_length = 15;
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(35, 0);             // Start at top-left corner
  display.cp437(true);                  // Use full 256 char 'Code Page 437' font

  char distance_text[text_length] = "Distance: ";
  String distance = "Distance: " + String(distance_data) + "m";
  distance.toCharArray(distance_text, text_length);

  for (int letter = 0; letter < text_length; letter++) {
    display.write(distance_text[letter]);
  }
}

/* angle_to_x_pos():
* Given some input angle, return the x position COMPASS_LENGTH from the zero point.
* DOES NOT ACCOUNT FOR CENTER OF COMPASS...ONLY THE CHANGE IN X FROM CENTER
*
* Params: 
*     - float angle: the angle from 0 degrees going counter clockwise
*/
int angle_to_x_pos(float angle_radians) {
  // float radians = angle * PI / 180;
  return int(COMPASS_LENGTH * cos(angle_radians));
}

/* angle_to_y_pos():
* Given some input angle, return the y position COMPASS_LENGTH from the zero point.
* DOES NOT ACCOUNT FOR CENTER OF COMPASS...ONLY THE CHANGE IN Y FROM CENTER
*
* Params: 
*     - float angle: the angle from 0 degrees going counter clockwise
*/
int angle_to_y_pos(float angle_radians) {
  // float radians = angle * PI / 180;
  return int(COMPASS_LENGTH * sin(angle_radians));
}

/* drawCompass():
* Handles updating the angle the compass is pointing
*
* Params: 
*     - float angle: the angle from 0 degrees going counter clockwise
* 
* Dependencies: 
*     - angle_to_x_pos()
*     - angle_to_y_pos()
*/
void drawCompass(float curr_latitude, float curr_longitude, float target_latitude, float target_longitude) {
  int compass_x_center = 12;
  int compass_y_center = SCREEN_HEIGHT / 2;

  float y_1 = curr_latitude;
  float x_1 = curr_longitude;

  float y_2 = target_latitude;
  float x_2 = target_longitude;

  float angle_radians = atan((abs(y_2) - abs(y_1)) / (abs(x_1) - abs(x_2)));
  // Serial.print("Angle in Radians: ");
  // Serial.println(angle_radians);
  // Serial.println("-----------------------------------");
  angle_radians = abs(angle_radians);
  // Serial.print("Angle in Degrees: ");
  // Serial.println(angle_radians * 180 / PI);

  // Serial.println(angle_radians);
  
  // 1st quadrant
  if (x_1 < x_2 && y_1 < y_2) {
    // Serial.println("1st quadrant");
    angle_radians = angle_radians;
  }
  // 2nd quadrant
  else if (x_1 > x_2 && y_1 < y_2) {
    // Serial.println("2nd quadrant");
    angle_radians = PI - angle_radians;
  }
  // 3rd quadrant
  else if (x_1 > x_2 && y_1 > y_2) {
    // Serial.println("3rd quadrant");
    angle_radians = PI + angle_radians;
  }
  // 4th quadrant
  else if (x_1 < x_2 && y_1 > y_2) {
    // Serial.println("4th quadrant");
    angle_radians = 2 * PI - angle_radians;
  }

  // Serial.print(" Angle after quadrant: ");
  // Serial.println(angle_radians * 180 / PI);

  // angle_radians = PI / 2;

  int compass_tip_x = compass_x_center + angle_to_x_pos(angle_radians);
  int compass_tip_y = compass_y_center - angle_to_y_pos(angle_radians);

  display.fillCircle(compass_x_center, compass_y_center, 2, SSD1306_INVERSE); // Draw the abse of the 

  display.drawLine(compass_x_center, compass_y_center, compass_tip_x, compass_tip_y, SSD1306_WHITE);

}

/* display_data():
* The do-all function for updating the display given GPS/heading data. Handles
* calculations for the arrow + distances
*
* Params: 
*     - long curr_longitude: the longitude of the user (handheld tracker)
*     - long curr_latitude:  the latitude of the user (handheld tracker)
*     - long heading: the angle the user is facing (in degrees) from North
* 
* Dependencies: 
*     - draw_arrow()
*     - dist_between_points()
*     - display_distance_data()
*/
void display_data(float curr_latitude, float curr_longitude) {
  display.clearDisplay();

  curr_latitude  = curr_latitude  / 10000000.0; // getting from the form xxxxxxxxx to xx.xxxxxxx
  curr_longitude = curr_longitude / 10000000.0;
  // 40.11450311099257, -8822732977554023
  float target_latitude  = 401145031 / 10000000.0;   //TODO: Remove these
  float target_longitude = -882273297 / 10000000.0;  //TODO: Remove these

  // First put arrow into screen memory (Left side of screen)
  drawCompass(curr_latitude, curr_longitude, target_latitude, target_longitude);
  
  // Second, put distance data into screen memory (right side of screen)
  int distance = TinyGPSPlus::distanceBetween(curr_latitude, curr_longitude, target_latitude, target_longitude);
  
  display_distance_data(distance);

  // Upload to display
  display.display();

}



void compass() {

  //TODO: while(menu button not pressed)
  while (true) {
    if (millis() - lastTime > 1000) {       // grab GPS data every 1 second
      curr_latitude = myGPS.getLatitude();
      curr_longitude = myGPS.getLongitude();
      display_data(curr_latitude, curr_longitude);  // update display given our new gps coords
      // lastTime = millis();
    }

  }

}
//!------------------------END COMPASS DISPLAY------------------------



//!------------------------BEGIN MENU DISPLAY------------------------

void display_menu(int m_type) {

  if (m_type == menu_type) {  // don't waste time displaying the same menu
    return;
  }

  display.clearDisplay();       // clear buffer memory

  int text_length = 80;
  char menu_text[text_length];// = "-------MENU-------\n> Compass <\nFrequency Change";
  String row_1 = "";
  String row_2 = "";
  String row_3 = "";
  String display_string = "";

  switch (m_type) {
    case 0: // This is main menu selected Compass
      row_1 = "-------MENU-------\n";
      row_2 = "> Compass <\n";
      row_3 = "Frequency Change";

      break;

    case 1: // Main menu selected Frequency Change
      row_1 = "-------MENU-------\n";
      row_2 = "Compass\n";
      row_3 = "> Frequency Change <";


      break;

    case 2: // This is sub menu of selected Frequency Change
      row_1 = "---Freq Change---\n";
      row_2 = "> Set Frequency: <\n";
      row_3 = "Back";
        // t_frequency;

      break;

    case 3: // This is sub menu of selected Frequency Change
      row_1 = "---Freq Change---\n";
      row_2 = "Set Frequency:\n";
      row_3 = "> Back <";
        // t_frequency;

      break;

    case 4: // This is sub menu of Set Frequency Change
      compass();

      break;

    default:
      break;
  }

  // Serial.println("Display String: " + display_string);

  display_string = row_1 + row_2 + row_3;

  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.cp437(true);                  // Use full 256 char 'Code Page 437' font
  display_string.toCharArray(menu_text, display_string.length() + 1); // convert string to char array for .write() func
  display.write(menu_text, display_string.length());  // put string into buffer memory

  display.display();  // update display from buffer

}


//!------------------------END MENU DISPLAY------------------------



//!------------------------BEGIN RADIO DISPLAY------------------------
#define RF95_FREQ 434.0
#define RFM96_RST 4

RH_RF95 radio;


//!------------------------END RADIO DISPLAY------------------------





void setup() {
  // Serial.begin(9600);
  // while(!Serial){}
  Wire.begin();         // join i2c bus (address optional for master)

  //!-------------------MENU SETUP-------------------
  pinMode(menu_button_pin,  INPUT_PULLUP);
  pinMode(enter_button_pin, INPUT_PULLUP);
  pinMode(down_button_pin,  INPUT_PULLUP);
  pinMode(up_button_pin,    INPUT_PULLUP);



  //!-------------------DISPLAY SETUP-------------------
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // Serial.println(F("SSD1306 allocation failed"));
    while(true){}; //proceed, loop forever
  }
  display.clearDisplay(); // Clear the buffer


  //!-------------------GPS SETUP-------------------
  // Connect to the Ublox module using Wire port
  if (myGPS.begin() == false) {
    // Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1) {}
  }

  //!-------------------RADIO SETUP-------------------
  if (!radio.init()) {
    // return ErrorCode::RADIO_INIT_FAILED;
    // Serial.println("Radio init failed");
    while (1) {}
  }
  if (!radio.setFrequency(RF95_FREQ)) {
    // return ErrorCode::RADIO_SET_FREQUENCY_FAILED;
    // Serial.println("Radio init failed");
    while (1) {}
  }

  /*
  * The default transmitter power is 13dBm, using PA_BOOST.
  * If you are using RFM95/96/97/98 modules which uses the PA_BOOST
  * transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
  */
  radio.setTxPower(6, false);

  // start with the top (base) menu (menu 0)
  display_menu(0);
  menu_type = 0;
}




void loop() {
  menu_button_state   = !digitalRead(menu_button_pin);
  enter_button_state  = !digitalRead(enter_button_pin);
  up_button_state     = !digitalRead(up_button_pin);
  down_button_state   = !digitalRead(down_button_pin);

  // 0 = base menu (compass selected)
  // 1 = base menu (frequency change selected)
  // 2 = frequency change menu (set frequency selected)
  // 3 = frequency change menu (back selected)


  //!-----------------------MENU-----------------------
  if (menu_button_state == 1) {  // Display default menu
    // Serial.println("Menu button pressed");
    display_menu(0);
    menu_type = 0;
  }


  //!-----------------------ENTER-----------------------
  else if (enter_button_state == 1) {
    // Serial.println("Enter button pressed");

    if (menu_type == 0) {       // display compass
      // display compass
      // display.clearDisplay();
      display_menu(4);
      // compass();
    }
    else if (menu_type == 1) {  // enter frequency change sub-menu 
      display_menu(2);
      menu_type = 2;
    }
    else if (menu_type == 2) {  // change frequency function
      // user-input frequency
      // press enter again
      // change frequency
    }
    else if (menu_type == 3) {  // 'back' go back to main menu
      display_menu(0);
      delay(300);
      menu_type = 0;
    }

  }



  //!-----------------------UP-----------------------
  else if (up_button_state == 1) {
    // Serial.println("UP button pressed");
    
    if (menu_type == 1) {  // enter frequency change sub-menu 
      display_menu(0);
      menu_type = 0;
    }
    else if (menu_type == 3) {  // 'back' go back to main menu
      display_menu(2);
      menu_type = 2;
    }
  }




  //!-----------------------DOWN-----------------------
  else if (down_button_state == 1) {
    // Serial.println("Down button pressed");
    // Serial.println(menu_type);
    if (menu_type == 0) {  // enter frequency change sub-menu 
      display_menu(1);
      menu_type = 1;
    }
    else if (menu_type == 2) {  // 'back' go back to main menu
      display_menu(3);
      menu_type = 3;
    }
  }

  
}




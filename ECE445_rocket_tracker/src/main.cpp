#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "pins.h"
// #include <SparkFun_Ublox_Arduino_Library.h> // http://librarymanager/All#SparkFun_u-blox_GNSS
#include "gps.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <TinyGPSPlus.h>



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

void display_distance_data(long distance_data) {
  int text_length = 15;
  // display.clearDisplay();
  // Serial.println(distance_data);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(35, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  char distance_text[text_length] = "Distance: ";
  String distance = "Distance: " + String(distance_data) + "m";
  distance.toCharArray(distance_text, text_length);

  for (int letter = 0; letter < text_length; letter++) {
    display.write(distance_text[letter]);
  }


  // display.display();
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
  float target_latitude  = 401066500 / 10000000.0;   //TODO: Remove these
  float target_longitude = -882271760 / 10000000.0;  //TODO: Remove these

  // First put arrow into screen memory (Left side of screen)
  drawCompass(curr_latitude, curr_longitude, target_latitude, target_longitude);
  
  // Second, put distance data into screen memory (right side of screen)
  int distance = TinyGPSPlus::distanceBetween(curr_latitude, curr_longitude, target_latitude, target_longitude);
  
  display_distance_data(distance);

  // Upload to display
  display.display();
  // Serial.println("end of display data");

}

void setup() {
  // Serial.begin(9600);
  // while(!Serial){}
  // Serial.println("------------------------------setup------------------------------");
  
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();         // join i2c bus (address optional for master)

  //-------------------DISPLAY SETUP-------------------
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // Serial.println(F("SSD1306 allocation failed"));
    while(true){}; //proceed, loop forever
  }
  display.clearDisplay(); // Clear the buffer
  // display.display();

  // -------------------GPS SETUP-------------------
  // Connect to the Ublox module using Wire port
  if (myGPS.begin() == false) {
    // Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);                       // wait for a second
    }
  }

  

}





void loop() {

  // Serial.println("Start of loop");

  // take GPS data once every second
  if (millis() - lastTime > 1000) {
    // SFE_UBLOX_GPS myGPS;
    // myGPS.begin();
    // Serial.println("inside gps update");
    // lastTime = millis(); //Update the timer
    
    // 4010716164997774, -8822859195374717
    // curr_latitude = 401071616;
    // curr_longitude = -882285919;

    curr_latitude = myGPS.getLatitude();
    // Serial.print(F("Lat: "));
    // Serial.println(curr_latitude);

    curr_longitude = myGPS.getLongitude();
    // Serial.print(F("Long: "));
    // Serial.println(curr_longitude);

    // curr_heading = myGPS.getHeading();
    // Serial.print(F("Heading: "));
    // Serial.print(curr_heading);
    // Serial.println(" ");

    // Serial.println("----------------");
  }

  // Serial.println(curr_longitude===
  

// 40109742224700966, -8824001914554178
  // curr_latitude = 401097422;
  // curr_longitude = -882400191;

  // curr_heading = 0;
  display_data(curr_latitude, curr_longitude);

  // Serial.println("end loop");

  // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  // delay(1000);                       // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  // delay(1000);                       // wait for a second

}




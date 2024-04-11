#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "pins.h"
#include "SparkFun_Ublox_Arduino_Library.h" // http://librarymanager/All#SparkFun_u-blox_GNSS
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define COMPASS_LENGTH 11

// GPS setup
SFE_UBLOX_GPS myGPS;
long lastTime = 0;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


float dist_between_points(long long1, long lat1, long long2, long lat2) {
  long1 = abs(long1);
  lat1  = abs(lat1);
  long2 = abs(long2);
  lat2  = abs(lat2);

  return sqrt(sq(long1 - long2) + sq(lat1 - lat2));
}

void display_distance_data(long distance_data) {
  int text_length = 15;
  // display.clearDisplay();
  // Serial.println(distance_data);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(50, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  char distance_text[text_length] = "Distance: ";
  String distance = "Distance: " + String(distance_data);
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
int angle_to_x_pos(float angle) {
  float radians = angle * PI / 180;
  return int(COMPASS_LENGTH * cos(radians));
}

/* angle_to_y_pos():
* Given some input angle, return the y position COMPASS_LENGTH from the zero point.
* DOES NOT ACCOUNT FOR CENTER OF COMPASS...ONLY THE CHANGE IN Y FROM CENTER
*
* Params: 
*     - float angle: the angle from 0 degrees going counter clockwise
*/
int angle_to_y_pos(float angle) {
  float radians = angle * PI / 180;
  return int(COMPASS_LENGTH * sin(radians));
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
void drawCompass(float angle) {
  int compass_x_center = 12;
  int compass_y_center = SCREEN_HEIGHT / 2;

  int compass_tip_x = compass_x_center - angle_to_x_pos(angle);
  int compass_tip_y = compass_y_center - angle_to_y_pos(angle);

  // Serial.println(compass_tip_x);
  // Serial.println(compass_tip_y);

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
void display_data(long curr_longitude, long curr_latitude, long heading) {
  display.clearDisplay();

  // First put arrow into screen memory (Left side of screen)
  drawCompass(heading);

  long target_longitude = 0;
  long target_latitude = 0;
  float distance = dist_between_points(curr_longitude, curr_latitude, target_longitude, target_latitude);
  // Second, put distance data into screen memory (right side of screen)
  display_distance_data(distance);

  // Upload to display
  display.display();

}

void setup() {
  Serial.begin(9600);

  // pinMode(OLED_RESET, OUTPUT);

  // digitalWrite(OLED_RESET, HIGH);
  // delay(10);
  // digitalWrite(OLED_RESET, LOW);
  // delay(10);
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

   //!-------------------GPS STUFF-------------------
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  //!-------------------END GPS STUFF-------------------

  // Clear the buffer
  display.clearDisplay();

}

void loop() {
  //!draw arrow
  // testdrawArrow();
  while (true) {
    for (int angle = 0; angle < 360; angle++) {
      display_data(215, -80, angle);
    }
  }
}





// void setup() {

//   //!-------------------GPS STUFF-------------------
//   if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
//   {
//     Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
//     while (1);
//   }
//   //!-------------------END GPS STUFF-------------------

//   pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
// }

// // the loop function runs over and over again forever
// void loop() {
//   // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//   // Serial.print("on\n");
//   // delay(100);                       // wait for a second
//   // digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//   // Serial.print("off\n");
//   // delay(100);                       // wait for a second
  
//   //!-------------------GPS STUFF-------------------
//   if (millis() - lastTime > 1000)
//   {
//     lastTime = millis(); //Update the timer
    
//     long latitude = myGPS.getLatitude();
//     Serial.print(F("Lat: "));
//     Serial.print(latitude);

//     long longitude = myGPS.getLongitude();
//     Serial.print(F(" Long: "));
//     Serial.print(longitude);
//     Serial.print(F(" (degrees * 10^-7)"));

//     long altitude = myGPS.getAltitude();
//     Serial.print(F(" Alt: "));
//     Serial.print(altitude);
//     Serial.print(F(" (mm)"));

//     long altitudeMSL = myGPS.getAltitudeMSL();
//     Serial.print(F(" AltMSL: "));
//     Serial.print(altitudeMSL);
//     Serial.print(F(" (mm)"));

//     Serial.println();
//   }
//   //!-------------------END GPS STUFF-------------------

  
// }

#include <Arduino.h>

int compass[32][64];
int Rows = 32;
int Cols = 64;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Serial success");

  for (int col = 0; col < Cols; col++){
    compass[0][col] = 1;
    compass[31][col] = 1;
  }
  for (int row = 0; row < Rows; row++) {
    compass[row][0] = 1;
    compass[row][63] = 1;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(compass[0][0]);
  // delay(20000);
}
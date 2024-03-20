// Authors: Ben Olaivar, 
#ifndef pins_H_INCLUDED
#define pins_H_INCLUDED

// SPI
#define MOSI    15
#define MISO    16
#define SCK     17

// I2C
#define I2C_SDA 27
#define I2C_SCL 28

// Do not touch
#define XTAL1   7       // Crystal Oscillator pins (DO NOT TOUCH)
#define XTAL2   8       // Crystal Oscillator pins (DO NOT TOUCH)
#define RESET   29      // Reset Button
#define RX      30      // Programming pins (DO NOT TOUCH)
#define TX      31      // Programming pins (DO NOT TOUCH)

#define transceiver_CS 19
#define display_RST 22

//-------------Pushbuttons for UI-------------
// Numbers
#define pushbutton_0    3
#define pushbutton_1    23
#define pushbutton_2    32
#define pushbutton_3    6
#define pushbutton_4    24
#define pushbutton_5    1
#define pushbutton_6    9
#define pushbutton_7    25
#define pushbutton_8    2
#define pushbutton_9    10

// Menu UI
#define pushbutton_menu     12
#define pushbutton_up       13
#define pushbutton_down     14
#define pushbutton_enter    11
#define pushbutton_dot      26

#endif //pins.h included
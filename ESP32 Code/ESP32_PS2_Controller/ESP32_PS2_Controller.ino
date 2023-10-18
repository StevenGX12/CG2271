#include <PS2X_lib.h>  //for v1.6

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/

//  ESP32 pin
// https://github.com/espressif/arduino-esp32/blob/master/docs/esp32_pinmap.png

#define PS2_DAT        19  //MISO  19
#define PS2_CMD        23  //MOSI  23
#define PS2_SEL         5  //SS     5
#define PS2_CLK        18  //SLK   18

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   false
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you connect the controller,
//or call config_gamepad(pins) again after connecting the controller.

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;

void setup(){

 // 115200
  Serial.begin(115200);

  //added delay to give wireless ps2 module some time to startup, before configuring it
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  while (error != 0) {
    delay(1000);// 1 second wait
    //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print("#try config ");
    Serial.println(tryNum);
    tryNum ++;
  }

  Serial.println(ps2x.Analog(1), HEX);
}

void loop() {

    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");
      // Serial.print(ps2x.Analog(PSS_LY)); //Left stick, Y axis. Other options: LX, RY, RX
      // Serial.print(",");
      // Serial.print(ps2x.Analog(PSS_LX), DEC);
      // Serial.print(",");
      // Serial.print(ps2x.Analog(PSS_RY), DEC);
      // Serial.print(",");
      // Serial.println(ps2x.Analog(PSS_RX), DEC);
      //print y for both sticks
      int leftY = ps2x.Analog(PSS_LY);
      int rightY = ps2x.Analog(PSS_RY);
      Serial.print(leftY);
      Serial.print(",");
      Serial.println(rightY);

      //convert to a binary format for the motor controller where 128 is the centred position for the joystick
      //and 0 is full forward and 255 is full reverse
      //but convert to MSB for the sign bit
      //and the rest of the bits are the magnitude
      //so 128 is 10000000
      //and 0 is 00000000
      //and 255 is 11111111

      //more concise way to do the above use bitwise operators
      int leftYBinary = 0;
      int rightYBinary = 0;
      leftYBinary = (leftY & 0x7F) | 0x80;
      rightYBinary = (rightY & 0x7F) | 0x80;

      Serial.print("Binary:");
      Serial.print(leftYBinary, BIN);
      Serial.print(",");
      Serial.println(rightYBinary, BIN);
    
    }

  
  delay(10e0);
}

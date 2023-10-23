#include <PS2X_lib.h> //for v1.6
#include <HardwareSerial.h>

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/

//  ESP32 pin
// https://github.com/espressif/arduino-esp32/blob/master/docs/esp32_pinmap.png

#define PS2_DAT 19 // MISO  19
#define PS2_CMD 23 // MOSI  23
#define PS2_SEL 5  // SS     5
#define PS2_CLK 18 // SLK   18

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures false
#define rumble false

/**
 * UART between ESP32 and KL25
 */
#define RXD0 3
#define TXD0 1

/**
 * Define common bit masks
 *
 * 
 */
#define MASK_BITS_2_TO_7 0b11111100

/**
 * Debug mode
 * 
 */
// #define DEBUG 

// Create a UART object
HardwareSerial SerialKL25(0); // UART0 on the ESP32

PS2X ps2x; // create PS2 Controller Class

// right now, the library does NOT support hot pluggable controllers, meaning
// you must always either restart your Arduino after you connect the controller,
// or call config_gamepad(pins) again after connecting the controller.

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;

void setup()
{
    Serial.begin(9600);
    SerialKL25.begin(9600, SERIAL_8N1, RXD0, TXD0); // Baud rate should match the KL25 configuration

    // added delay to give wireless ps2 module some time to startup, before configuring it
    // CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

    while (error != 0)
    {
        delay(1000); // 1 second wait
        // setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
        error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
        Serial.print("#try config ");
        Serial.println(tryNum);
        tryNum++;
    }

    Serial.println(ps2x.Analog(1), HEX);
}

void printBinary(byte val){
  for (int i = 7; i>=0; i--){
    Serial.print(bitRead(val,i));
  }
}

uint8_t processJoystickValue(int joystickValue) {
    uint8_t dataPacket = 0;
    if (joystickValue == 128) {
        dataPacket |= 0b00;
    } else if (joystickValue >= 0 && joystickValue <= 127) {
        dataPacket |= 0b01;
        dataPacket |= (uint8_t)(((128.0 - joystickValue) / 128.0) * 63) << 2;
    } else if (joystickValue >= 129 && joystickValue <= 255) {
        dataPacket |= 0b10;
        dataPacket |= (uint8_t)(((joystickValue - 128) / 127.0) * 63) << 2;
    }
    return dataPacket;
}


void loop()
{
    ps2x.read_gamepad(false, vibrate); // read controller and set large motor to spin at 'vibrate' speed
    
    // Create a buffer to hold the payload
    uint8_t payload[4];
    
    // Initialize data packets for left and right drive trains
    uint8_t leftDataPacket = 0;
    uint8_t rightDataPacket = 0;

    // Create a start byte and an end byte
    uint8_t startByte = 0b00000011; // Start byte with LSB '11'
    uint8_t endByte = 0b01111111;   // End byte with LSB '11'

     // Tank Drive
    int leftJoystickY = ps2x.Analog(PSS_LY);
    int rightJoystickY = ps2x.Analog(PSS_RY);

    #ifdef DEBUG 
    Serial.print("Stick Values:");
    Serial.print(leftJoystickY);
    Serial.print(",");
    Serial.println(rightJoystickY);
    #endif

    uint8_t leftDataPacket = processJoystickValue(leftJoystickY);
    uint8_t rightDataPacket = processJoystickValue(rightJoystickY);

    // Copy the bytes into the payload
    payload[0] = startByte; 
    payload[1] = leftDataPacket;
    payload[2] = rightDataPacket;
    payload[3] = endByte;

    SerialKL25.write(payload, sizeof(payload));
   
    #ifdef DEBUG
    Serial.print("Payload: ");
    for (int i = 0; i < sizeof(payload); i++)
    {
        printBinary(payload[i]);
        Serial.print(" ");
    }
    Serial.println();
    #endif

    //Add a delay or adjust timing based on your application requirements
    delay(10);
}

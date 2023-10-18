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
    SerialKL25.begin(9600); // Baud rate should match the KL25 configuration

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

void loop()
{
    ps2x.read_gamepad(false, vibrate); // read controller and set large motor to spin at 'vibrate' speed
    Serial.print("Stick Values:");

    int leftJoystickY = ps2x.Analog(PSS_LY);
    int rightJoystickY = ps2x.Analog(PSS_RY);
    Serial.print(leftJoystickY);
    Serial.print(",");
    Serial.println(rightJoystickY);

    // convert to a binary format for the motor controller where 128 is the centred position for the joystick
    // and 0 is full forward and 255 is full reverse
    // but convert to MSB for the sign bit
    // and the rest of the bits are the magnitude
    // so 128 is 10000000
    // and 0 is 00000000
    // and 255 is 11111111

    // Create a buffer to hold the payload
    uint8_t payload[4]; // Adjust the size as needed
    // Initialize data packets for left and right drive trains
    uint8_t leftDataPacket = 0;
    uint8_t rightDataPacket = 0;
    // Create a start byte and an end byte
    uint8_t startByte = 0b00000011; // Start byte with LSB '11'
    uint8_t endByte = 0b01111111;   // End byte with LSB '11'

    // Process the left joystick Y value
    if (leftJoystickY == 128)
    {
        leftDataPacket |= 0b00 << 0; // Set bits 0 and 1 to '00'
    }
    else if (leftJoystickY >= 0 && leftJoystickY <= 127)
    {
        leftDataPacket |= 0b01 << 0; // Set bits 0 and 1 to '01'
    }
    else if (leftJoystickY >= 129 && leftJoystickY <= 255)
    {
        leftDataPacket |= 0b10 << 0; // Set bits 0 and 1 to '10'
    }

    uint8_t leftScaledValue = 0;
    if (leftJoystickY >= 0 && leftJoystickY <= 127)
    {
        leftScaledValue = (uint8_t)(((128.0 - leftJoystickY) / 128.0) * 63);
    }
    else if (leftJoystickY >= 129 && leftJoystickY <= 255)
    {
        leftScaledValue = (uint8_t)(((leftJoystickY - 128) / 127.0) * 63);
    }
    leftDataPacket |= leftScaledValue << 2; // 6-bit scaled value is stored in bits 2 to 7 of the leftDataPacket byte.

    // Process the right joystick Y value (similar to left joystick)
    if (rightJoystickY == 128)
    {
        rightDataPacket |= 0b00 << 0;
    }
    else if (rightJoystickY >= 0 && rightJoystickY <= 127)
    {
        rightDataPacket |= 0b01 << 0;
    }
    else if (rightJoystickY >= 129 && rightJoystickY <= 255)
    {
        rightDataPacket |= 0b10 << 0;
    }

    uint8_t rightScaledValue = 0;
    if (rightJoystickY >= 0 && rightJoystickY <= 127)
    {
        rightScaledValue = (uint8_t)(((128.0 - rightJoystickY) / 128.0) * 63);
    }
    else if (rightJoystickY >= 129 && rightJoystickY <= 255)
    {
        rightScaledValue = (uint8_t)(((rightJoystickY - 128) / 127.0) * 63);
    }
    rightDataPacket |= rightScaledValue << 2; // 6-bit scaled value is stored in bits 2 to 7 of the rightDataPacket byte.

    // Copy the bytes into the payload
    payload[0] = startByte;
    payload[1] = leftDataPacket;
    payload[2] = rightDataPacket;
    payload[3] = endByte;

    // Send the payload over UART to KL25
    // for (int i = 0; i < sizeof(payload); i++) {
    //     SerialKL25.write(payload[i]);
    // }
    SerialKL25.write(payload, sizeof(payload));
   
    // Print the payload (for testing)
    Serial.print("Payload: ");
    for (int i = 0; i < sizeof(payload); i++)
    {
        // Serial.print(payload[i], BIN);
        printBinary(payload[i]);
        Serial.print(" ");
    }
    Serial.println();

    // Add a delay or adjust timing based on your application requirements
    delay(100);
}

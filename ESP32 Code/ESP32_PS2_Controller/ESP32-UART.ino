// #include <Arduino.h>

// #define RXD0 3
// #define TXD0 1

// // Create a UART object
// HardwareSerial SerialKL25(0); // UART0 on the ESP32

// void setup() {
//   Serial.begin(115200);
//   SerialKL25.begin(9600); // Baud rate should match the KL25 configuration
// }

// void loop() {
// // -------- Joystick control -----------
//   // Simulated joystick Y values for left and right drive trains (replace with actual inputs)
//   int leftJoystickY = analogRead(A0); // Replace with the actual pin reading your left joystick Y value
//   int rightJoystickY = analogRead(A1); // Replace with the actual pin reading your right joystick Y value

//   // Create a buffer to hold the payload
//   uint8_t payload[5]; // Adjust the size as needed  
//   // Initialize data packets for left and right drive trains
//   uint8_t leftDataPacket = 0;
//   uint8_t rightDataPacket = 0;
//   // Create a start byte and an end byte
//   uint8_t startByte = 0b00000011; // Start byte with LSB '11'
//   uint8_t endByte = 0b00000011;   // End byte with LSB '11'

//   // Process the left joystick Y value
//   if (leftJoystickY == 128) {
//     leftDataPacket |= 0b00 << 0; // Set bits 0 and 1 to '00'
//   } else if (leftJoystickY >= 0 && leftJoystickY <= 127) {
//     leftDataPacket |= 0b01 << 0; // Set bits 0 and 1 to '01'
//   } else if (leftJoystickY >= 129 && leftJoystickY <= 255) {
//     leftDataPacket |= 0b10 << 0; // Set bits 0 and 1 to '10'
//   }

//   uint8_t leftScaledValue = 0;
//   if (leftJoystickY >= 0 && leftJoystickY <= 127) {
//     leftScaledValue = (uint8_t)(((128.0 - leftJoystickY) / 128.0) * 63);
//   } else if (leftJoystickY >= 129 && leftJoystickY <= 255) {
//     leftScaledValue = (uint8_t)(((leftJoystickY - 128) / 127.0) * 63);
//   }
//   leftDataPacket |= leftScaledValue << 2; // 6-bit scaled value is stored in bits 2 to 7 of the leftDataPacket byte.

//   // Process the right joystick Y value (similar to left joystick)
//   if (rightJoystickY == 128) {
//     rightDataPacket |= 0b00 << 0;
//   } else if (rightJoystickY >= 0 && rightJoystickY <= 127) {
//     rightDataPacket |= 0b01 << 0;
//   } else if (rightJoystickY >= 129 && rightJoystickY <= 255) {
//     rightDataPacket |= 0b10 << 0;
//   }

//   uint8_t rightScaledValue = 0;
//   if (rightJoystickY >= 0 && rightJoystickY <= 127) {
//     rightScaledValue = (uint8_t)(((128.0 - rightJoystickY) / 128.0) * 63);
//   } else if (rightJoystickY >= 129 && rightJoystickY <= 255) {
//     rightScaledValue = (uint8_t)(((rightJoystickY - 128) / 127.0) * 63);
//   }
//   rightDataPacket |= rightScaledValue << 2; // 6-bit scaled value is stored in bits 2 to 7 of the rightDataPacket byte.

//   // Copy the bytes into the payload
//   payload[0] = startByte;
//   payload[1] = leftDataPacket;
//   payload[2] = rightDataPacket;
//   payload[3] = endByte;
//   payload[4] = 0; // Add more bytes as needed

//   // Send the payload over UART to KL25
//   SerialKL25.write(payload, sizeof(payload));

//   // Print the payload (for testing)
//   Serial.print("Payload: ");
//   for (int i = 0; i < sizeof(payload); i++) {
//     Serial.print(payload[i], BIN);
//     Serial.print(" ");
//   }
//   Serial.println();

//   // Add a delay or adjust timing based on your application requirements
//   delay(100);

// }
#include <WiFi.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <BluetoothSerial.h>
#include <HardwareSerial.h>


BluetoothSerial SerialBT;
byte receivedByte;
HardwareSerial SerialPort(2); // use UART2


void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);
    if (i < 5){
      Serial.print(":");
    }
  }
}

void setup() {
	// Allow allocation of all timers
	// Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, 16, 17); 
  delay(500);
  SerialBT.begin("59z19"); //Bluetooth device name
  delay(500);
  // printDeviceAddress();
  // Serial.println("<A>");
}


void loop() {
  if (SerialBT.available()) {
    receivedByte = (byte)SerialBT.read();
    SerialPort.write(receivedByte);
  }
  // byte a = 255;
  // byte b = 254;
  // SerialPort.write(a);
  // delay(500);
  // SerialPort.write(b);
  // delay(500);
}

/*
#include <Arduino.h>
int incomingByte = 0; // for incoming serial data

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
  }
}
*/
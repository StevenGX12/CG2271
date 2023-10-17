// #include <Arduino.h>
// #include <Servo.h>

// int SERVO_1_PIN = 11;   // Servo 1
// int SERVO_2_PIN = 2;    // Servo 2
// int SERVO_3_PIN = 4;    // Servo 3
// int SERVO_4_PIN = 3;    // Servo 4

// // // Winch connections
// int winch_en = 10;
// int winch_in1 = 14;
// int winch_in2 = 15;

// // Servo Declarations
// Servo servo_1;
// Servo servo_2;
// Servo servo_3;

// #define BAUD 9600

// void setup() {
//   servo_1.attach(SERVO_1_PIN);
//   servo_2.attach(SERVO_2_PIN);
//   servo_3.attach(SERVO_3_PIN);
//   pinMode(winch_en, OUTPUT);
//   pinMode(winch_in1, OUTPUT);
//   pinMode(winch_in2, OUTPUT);
// }

// void loop() {
//   while (1){
//     // servo_1.write(90);
//     // servo_2.write(180);
//     // servo_3.write(60);

//     analogWrite(winch_en, 145);
//     digitalWrite(winch_in1, HIGH);
//     digitalWrite(winch_in2, LOW);
//     delay(5);
//   } 
// }

#include <Arduino.h>
#include <Servo.h>

// Incoming UART Byte Handler
byte incoming_byte = 0;
byte packet_count = 0;
unsigned long rx_flag_millis = 0;
unsigned long packet_millis = 0;

// Main Loop Timer
unsigned long t_loop = 0;
#define DELAY 10 //time for main loop to run


// STATE FLAG
int two_period_state_flag = 0;
int three_period_state_flag = 0;


// PIN DECLARATIONS
int ENA = 5;  // Enable A --> Right Speed
int ENB = 6;  // Enable B --> Left Speed
int IN1 = 8;  // Motor Interface 1 
int IN2 = 7;  // Motor Interface 2 
int IN3 = 12; // Motor Interface 3 
int IN4 = 13; // Motor Interface 4 
int ledpin1 = A6;   // System Startup Indicator 1
int ledpin2 = A7;   // System Startup Indicator 2
int SERVO_1_PIN = 11;   // Servo 1
int SERVO_2_PIN = 2;    // Servo 2
int SERVO_3_PIN = 4;    // Servo 3
int SERVO_4_PIN = 3;    // Servo 4


// Winch connections
int winch_en = 10;
int winch_in1 = 14;
int winch_in2 = 15;

  // servo1.attach(11);                      //定义舵机1控制口
  // servo2.attach(2);                       //定义舵机2控制口
  // servo3.attach(4);                       //定义舵机3控制口
  // servo4.attach(3);                       //定义舵机4控制口
  // servo5.attach(A8);                      //定义舵机5控制口
  // servo6.attach(A9);                      //定义舵机6控制口
  // servo7.attach(9);                       //定义舵机7控制口
  // servo8.attach(10);                      //定义舵机8控制口

byte left_speed = 65;
byte right_speed = 65;   // 0 - 255

int buffer[7];                       
int rec_flag;



// Servo Declarations
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;


byte servo_1_mode = 3;      // Map speeds of -3, -2, ... +2, +3 to Mode Value of 0, 1, ... 5, 6. Mode 7 represents custom speed.
byte servo_1_val = 90;
// volatile byte servo_1_mode = 3;      // Map speeds of -3, -2, ... +2, +3 to Mode Value of 0, 1, ... 5, 6. Mode 7 represents custom speed.
// volatile byte servo_1_val = 90;
#define servo_1_low_bound 0
#define servo_1_high_bound 180

byte servo_2_mode = 3;      
byte servo_2_val = 110;
#define servo_2_low_bound 40
#define servo_2_high_bound 180

byte servo_3_mode = 3;      
byte servo_3_val = 0;
#define servo_3_low_bound 0
#define servo_3_high_bound 145

byte servo_4_mode = 3;      
byte servo_4_val = 0;
#define servo_4_low_bound 0
#define servo_4_high_bound 180


#define BASE_FWD {digitalWrite(IN1, LOW); digitalWrite(IN2,HIGH);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);}
#define BASE_REV {digitalWrite(IN1, HIGH); digitalWrite(IN2,LOW);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);}
#define BASE_RIGHT {digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);}      
#define BASE_LEFT {digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);}     
#define BASE_STOP {digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);} 

#define BAUD 9600

void safe_mode(){
  servo_1_mode = 3;
  servo_2_mode = 3;
  servo_3_mode = 3;
  servo_4_mode = 3;
  BASE_STOP;
  analogWrite(ENB, 0);
  analogWrite(ENA, 0); 
  analogWrite(winch_en, 0);
}

void Communication_Decode()
{
  // Base Control
  char signed_left_speed = buffer[0] - 126;    // Accept speeds of -126 to 126. Zero-shift the inputs
  char signed_right_speed = buffer[1] - 126;
  
  // Assign Speed
  left_speed = abs(signed_left_speed) * 2.02381;
  right_speed = abs(signed_right_speed) * 2.02381;
  
  // Assign Direction based on Signed Speed
  if (signed_left_speed==0 && signed_right_speed==0){
    BASE_STOP;
  }else if (signed_left_speed > 0 && signed_right_speed > 0){
    BASE_FWD;
  }else if (signed_left_speed < 0 && signed_right_speed < 0){
    BASE_REV;
  }else if ((signed_left_speed > 0 && signed_right_speed <= 0)||(signed_left_speed == 0 && signed_right_speed < 0)){
    BASE_RIGHT;
  }else if ((signed_left_speed <= 0 && signed_right_speed > 0)||(signed_left_speed < 0 && signed_right_speed == 0)){
    BASE_LEFT;
  }

  analogWrite(ENB, left_speed);
  analogWrite(ENA, right_speed);  

  // Winch Control
  char signed_winch_speed = buffer[5] - 126;
  byte winch_speed = abs(signed_winch_speed) * 2.02381;
  if (signed_winch_speed > 0) {
    digitalWrite(winch_in1, HIGH);
  	digitalWrite(winch_in2, LOW);
  } else if (signed_winch_speed < 0){
    digitalWrite(winch_in1, LOW);
  	digitalWrite(winch_in2, HIGH);
  } else {
    digitalWrite(winch_in1, LOW);
  	digitalWrite(winch_in2, LOW);
  }
  analogWrite(winch_en, winch_speed);




  // Servo 1 Control
  if (buffer[2] >= 7){
    // Set according to Integer Range
    servo_1_mode = 7;               // Flag 7: Set Custom Angle
    servo_1_val = buffer[2] - 10;   // Transmit values of 10 - 190 (map to servo angle of 0 - 180)
  }else{
    servo_1_mode = buffer[2];   // Modes 0, 1, 2, 3, 4, 5, 6, to correspond to speeds -3, -2, -1, 0, 1, 2, 3
  }

  // Servo 2 Control
  if (buffer[3] >= 7){
    // Set according to Integer Range
    servo_2_mode = 7;               // Flag 4: Set Custom Angle
    servo_2_val = buffer[3] - 10;   // Transmit values of 10 - 190 (map to servo angle of 0 - 180)
  }else{
    servo_2_mode = buffer[3];   // TCP Transmit 0, 1, 2, 3, 4, 5, 6, to correspond to speeds -3, -2, -1, 0, 1, 2, 3
  }

  // Servo 3 Control
  if (buffer[4] >= 7){
    // Set according to Integer Range
    servo_3_mode = 7;               // Flag 4: Set Custom Angle
    servo_3_val = buffer[4] - 10;   // Transmit values of 10 - 190 (map to servo angle of 0 - 180)
  }else{
    servo_3_mode = buffer[4];   // TCP Transmit 0, 1, 2, 3, 4, 5, 6, to correspond to speeds -3, -2, -1, 0, 1, 2, 3
  }

  // Servo 4 Control
  if (buffer[6] >= 7){
    // Set according to Integer Range
    servo_4_mode = 7;               // Flag 4: Set Custom Angle
    servo_4_val = buffer[6] - 10;   // Transmit values of 10 - 190 (map to servo angle of 0 - 180)
  }else{
    servo_4_mode = buffer[6];   // TCP Transmit 0, 1, 2, 3, 4, 5, 6, to correspond to speeds -3, -2, -1, 0, 1, 2, 3
  }

}



void  Delayed()   
{
  int i;
  for (i = 0; i < 25; i++)
  {
    digitalWrite(ledpin1, HIGH);
    digitalWrite(ledpin2, LOW);
    delay(500);
    digitalWrite(ledpin1, LOW);
    digitalWrite(ledpin2, HIGH);
    delay(500);
  }
  
  for (i = 0; i < 10; i++)
  {
    digitalWrite(ledpin1, HIGH);
    digitalWrite(ledpin2, HIGH);
    delay(250);
    digitalWrite(ledpin1, LOW);
    digitalWrite(ledpin2, LOW);
    delay(250);
  }
  BASE_STOP;
  digitalWrite(ledpin1, LOW);
  digitalWrite(ledpin2, LOW);
}

void setup() {
    Serial.begin(9600);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    pinMode(A6, OUTPUT);
    pinMode(A7, OUTPUT);


    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(winch_en, OUTPUT);
    pinMode(winch_in1, OUTPUT);
    pinMode(winch_in2, OUTPUT);
    digitalWrite(winch_in1, LOW);
    digitalWrite(winch_in2, LOW);

    // Delayed(); 

    servo_1.attach(SERVO_1_PIN);
    servo_2.attach(SERVO_2_PIN);
    servo_3.attach(SERVO_3_PIN);
    servo_4.attach(SERVO_4_PIN);
    delay(1000);
    servo_1.write(servo_1_val);
    servo_2.write(servo_2_val);
    servo_3.write(servo_3_val);
    servo_4.write(servo_4_val);

    digitalWrite(ledpin1, HIGH);
    digitalWrite(ledpin2, LOW);
    delay(500);
    digitalWrite(ledpin1, LOW);
    digitalWrite(ledpin2, HIGH);
    delay(500);
    digitalWrite(ledpin1, LOW);
    digitalWrite(ledpin2, LOW);
    delay(500);


    // USART_init(); 
}

void loop() {
  while (1){
    // UART Timeout Checker
    if (millis()-rx_flag_millis > 100){
        rec_flag = 0;
    }

    // UART Receive Routine
    if (Serial.available()>0){
        incoming_byte = Serial.read();  // Read new byte
        if (rec_flag == 0 && incoming_byte == 0xfe){
            rec_flag = 1;
            rx_flag_millis = millis();
            packet_count = 0;
        } else {    // If Receive Flag is high
            if (incoming_byte == 0xff){     // If End of Packet Flag is received
                rec_flag = 0;
                if (packet_count == 7){
                    Communication_Decode();
                }
                packet_count = 0;
                packet_millis = millis();
            }else{
                buffer[packet_count] = incoming_byte;
                packet_count++;
            }
        }
    }
    
    if (millis()-packet_millis > 500){
        // Packet Timeout Checker
        safe_mode();
    }else if (millis() - t_loop >= DELAY){
        // Main Routine
        t_loop = millis();

        if(!servo_1.attached()){
            servo_1.detach();
            servo_1.attach(SERVO_1_PIN);
        }
        if(!servo_2.attached()){
            servo_2.detach();
            servo_2.attach(SERVO_2_PIN);
        }
        if(!servo_3.attached()){
            servo_3.detach();
            servo_3.attach(SERVO_3_PIN);
        }
        if(!servo_4.attached()){
          servo_4.detach();
          servo_4.attach(SERVO_4_PIN);
        }

        // Servo 1 Controls
        switch (servo_1_mode){
            case 7:
            // Set Custom Angle
            servo_1.write(servo_1_val);
            case 0:
            // Neg Shift towards 0 at every clock cycle
            servo_1_val = (servo_1_val <= servo_1_low_bound ? servo_1_low_bound : servo_1_val - 1);
            servo_1.write(servo_1_val);
            break;
            case 1:
            // Neg Shift towards 0 at every 2x clock cycle
            if (!two_period_state_flag){
                servo_1_val = (servo_1_val <= servo_1_low_bound ? servo_1_low_bound : servo_1_val - 1);
                servo_1.write(servo_1_val);
            }
            break;
            case 2:
            // Neg Shift towards 0 at every 3x clock cycle
            if (!three_period_state_flag){
                servo_1_val = (servo_1_val <= servo_1_low_bound ? servo_1_low_bound : servo_1_val - 1);
                servo_1.write(servo_1_val);
            }
            break;
            case 4:
            // Positive Shift towards 0 at every 3x clock cycle
            if (!three_period_state_flag){
                servo_1_val = (servo_1_val >= servo_1_high_bound ? servo_1_high_bound : servo_1_val + 1);
                servo_1.write(servo_1_val);
            }
            break;
            case 5:
            // Positive Shift towards 0 at every 2x clock cycle
            if (!two_period_state_flag){
                servo_1_val = (servo_1_val >= servo_1_high_bound ? servo_1_high_bound : servo_1_val + 1);
                servo_1.write(servo_1_val);
            }
            break;
            case 6:
            // Positive Shift towards 0 at every clock cycle
            servo_1_val = (servo_1_val >= servo_1_high_bound ? servo_1_high_bound : servo_1_val + 1);
            servo_1.write(servo_1_val);
            break;
            default:
            break;
        }

        // Servo 2 Controls
        switch (servo_2_mode){
            case 7:
            // Set Custom Angle
            servo_2.write(servo_2_val);
            case 0:
            // Neg Shift towards 0 at every clock cycle
            servo_2_val = (servo_2_val <= servo_2_low_bound ? servo_2_low_bound : servo_2_val - 1);
            servo_2.write(servo_2_val);
            break;
            case 1:
            // Neg Shift towards 0 at every 2x clock cycle
            if (!two_period_state_flag){
                servo_2_val = (servo_2_val <= servo_2_low_bound ? servo_2_low_bound : servo_2_val - 1);
                servo_2.write(servo_2_val);
            }
            break;
            case 2:
            // Neg Shift towards 0 at every 3x clock cycle
            if (!three_period_state_flag){
                servo_2_val = (servo_2_val <= servo_2_low_bound ? servo_2_low_bound : servo_2_val - 1);
                servo_2.write(servo_2_val);
            }
            break;
            case 4:
            // Positive Shift towards 0 at every 3x clock cycle
            if (!three_period_state_flag){
                servo_2_val = (servo_2_val >= servo_2_high_bound ? servo_2_high_bound : servo_2_val + 1);
                servo_2.write(servo_2_val);
            }
            break;
            case 5:
            // Positive Shift towards 0 at every 2x clock cycle
            if (!two_period_state_flag){
                servo_2_val = (servo_2_val >= servo_2_high_bound ? servo_2_high_bound : servo_2_val + 1);
                servo_2.write(servo_2_val);
            }
            break;
            case 6:
            // Positive Shift towards 0 at every clock cycle
            servo_2_val = (servo_2_val >= servo_2_high_bound ? servo_2_high_bound : servo_2_val + 1);
            servo_2.write(servo_2_val);
            break;
            default:
            break;
        }

        // Servo 3 Controls
        switch (servo_3_mode){
            case 7:
            // Set Custom Angle
            servo_3.write(servo_3_val);
            case 0:
            // Neg Shift towards 0 at every clock cycle
            servo_3_val = (servo_3_val <= servo_3_low_bound ? servo_3_low_bound : servo_3_val - 1);
            servo_3.write(servo_3_val);
            break;
            case 1:
            // Neg Shift towards 0 at every 2x clock cycle
            if (!two_period_state_flag){
                servo_3_val = (servo_3_val <= servo_3_low_bound ? servo_3_low_bound : servo_3_val - 1);
                servo_3.write(servo_3_val);
            }
            break;
            case 2:
            // Neg Shift towards 0 at every 3x clock cycle
            if (!three_period_state_flag){
                servo_3_val = (servo_3_val <= servo_3_low_bound ? servo_3_low_bound : servo_3_val - 1);
                servo_3.write(servo_3_val);
            }
            break;
            case 4:
            // Positive Shift towards 0 at every 3x clock cycle
            if (!three_period_state_flag){
                servo_3_val = (servo_3_val >= servo_3_high_bound ? servo_3_high_bound : servo_3_val + 1);
                servo_3.write(servo_3_val);
            }
            break;
            case 5:
            // Positive Shift towards 0 at every 2x clock cycle
            if (!two_period_state_flag){
                servo_3_val = (servo_3_val >= servo_3_high_bound ? servo_3_high_bound : servo_3_val + 1);
                servo_3.write(servo_3_val);
            }
            break;
            case 6:
            // Positive Shift towards 0 at every clock cycle
            servo_3_val = (servo_3_val >= servo_3_high_bound ? servo_3_high_bound : servo_3_val + 1);
            servo_3.write(servo_3_val);
            break;
            default:
            break;
        }

        // Servo 4 Controls
        switch (servo_4_mode){
          case 7:
            // Set Custom Angle
            servo_4.write(servo_4_val);
            case 0:
            // Neg Shift towards 0 at every clock cycle
            servo_4_val = (servo_4_val <= servo_4_low_bound ? servo_4_low_bound : servo_4_val - 1);
            servo_4.write(servo_4_val);
            break;
          case 1:
            // Neg Shift towards 0 at every 2x clock cycle
            if (!two_period_state_flag){
                servo_4_val = (servo_4_val <= servo_4_low_bound ? servo_4_low_bound : servo_4_val - 1);
                servo_4.write(servo_4_val);
            }
            break;
          case 2:
            // Neg Shift towards 0 at every 3x clock cycle
            if (!three_period_state_flag){
                servo_4_val = (servo_4_val <= servo_4_low_bound ? servo_4_low_bound : servo_4_val - 1);
                servo_4.write(servo_4_val);
            }
            break;
          case 4:
            // Positive Shift towards 0 at every 3x clock cycle
            if (!three_period_state_flag){
                servo_4_val = (servo_4_val >= servo_4_high_bound ? servo_4_high_bound : servo_4_val + 1);
                servo_4.write(servo_4_val);
            }
            break;
          case 5:
            // Positive Shift towards 0 at every 2x clock cycle
            if (!two_period_state_flag){
                servo_4_val = (servo_4_val >= servo_4_high_bound ? servo_4_high_bound : servo_4_val + 1);
                servo_4.write(servo_4_val);
            }
            break;
          case 6:
            // Positive Shift towards 0 at every clock cycle
            servo_4_val = (servo_4_val >= servo_4_high_bound ? servo_4_high_bound : servo_4_val + 1);
            servo_4.write(servo_4_val);
            break;
          default:
            break;
        }


        // State Machine Setting
        two_period_state_flag = (two_period_state_flag >=1 ? 0 : 1);
        three_period_state_flag = (three_period_state_flag >=2 ? 0 : three_period_state_flag+1);

    }
  } 
}

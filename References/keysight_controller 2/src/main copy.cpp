// #include <Arduino.h>
// #include <Servo.h>

// int ENA = 5;  // Enable A --> Right Speed
// int ENB = 6;  // Enable B --> Left Speed
// int IN1 = 8;  // Motor Interface 1 
// int IN2 = 7;  // Motor Interface 2 
// int IN3 = 12; // Motor Interface 3 
// int IN4 = 13; // Motor Interface 4 

// int SERVO_1_PIN = 11;
// int SERVO_2_PIN = 2;

// Servo servo1;
// Servo servo2;
// // side note: a button for 0 to 180, button for reset to original position. 


// int left_speed = 65;
// int right_speed = 65;   // 0 - 255

// int buffer[5];  
// // int buffer[7]; // motors + servo commands                   
// int serial_data;  
// int rec_flag;
// unsigned long Costtime;              


// #define BASE_FWD {digitalWrite(IN1, LOW); digitalWrite(IN2,HIGH);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);}
// #define BASE_REV {digitalWrite(IN1, HIGH); digitalWrite(IN2,LOW);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);}
// #define BASE_RIGHT {digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);}      
// #define BASE_LEFT {digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);}     
// #define BASE_STOP {digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);} 

// #define SERVO_1_INC {servo1Angle++; servo1.write(servo1Angle);}
// #define SERVO_1_DEC {servo1Angle--; servo1.write(servo1Angle);}
// #define SERVO_2_INC {servo2Angle++; servo2.write(servo2Angle);}
// #define SERVO_2_DEC {servo2Angle--; servo2.write(servo2Angle);}

// #define BAUD 9600


// void USART_init()
// {
//   SREG = 0x80;                              
//   bitSet(UCSR0B,RXCIE0);                     
//   bitSet(UCSR0B,RXEN0);                      
//   bitSet(UCSR0B,TXEN0);                      
//   bitSet(UCSR0C,UCSZ01);
//   bitSet(UCSR0C,UCSZ00);                    
//   UBRR0=(F_CPU/16/BAUD-1);                  
// }

// void UartTimeoutCheck(void)
// {
//   if (rec_flag == 1)
//   {
//     Costtime++;
//     if (Costtime == 100000)
//     {
//       rec_flag = 0;
//     }
//   }
// }

// void Communication_Decode()
// {

//   // FOR THE MOTORS //
//   // int signed_left_speed = buffer[0] - 127;    // Accept speeds of +/-127. Zero-shift the inputs
//   // int signed_right_speed = buffer[1] - 127;
  
//   // // Assign Speed
//   // left_speed = abs(signed_left_speed) * 2;
//   // right_speed = abs(signed_right_speed) * 2;
  
//   // // Assign Direction based on Signed Speed
//   // if (signed_left_speed==0 && signed_right_speed==0){
//   //   BASE_STOP;
//   // }else if (signed_left_speed > 0 && signed_right_speed > 0){
//   //   BASE_FWD;
//   // }else if (signed_left_speed < 0 && signed_right_speed < 0){
//   //   BASE_REV;
//   // }else if ((signed_left_speed > 0 && signed_right_speed <= 0)||(signed_left_speed == 0 && signed_right_speed < 0)){
//   //   BASE_RIGHT;
//   // }else if ((signed_left_speed <= 0 && signed_right_speed > 0)||(signed_left_speed < 0 && signed_right_speed == 0)){
//   //   BASE_LEFT;
//   // }

//   // analogWrite(ENB, left_speed);
//   // analogWrite(ENA, right_speed);  


//   // FOR SERVOS //

//   int signed_servo_1 = buffer[3] - 127;    // Zero-shift the inputs. Differentiate buffers from motors later
//   int signed_servo_2 = buffer[4] - 127;
  
//   if (signed_servo_1 > 0){
//     SERVO_1_INC;
//   }
//   if (signed_servo_1 < 0){
//     SERVO_1_DEC;
//   }
//   if (signed_servo_2 > 0){
//     SERVO_2_INC;
//   }
//   if (signed_servo_2 < 0){
//     SERVO_2_DEC;
//   }
// }

// void Get_uartdata(void)
// {
//    static int i;
//     serial_data = UDR0; // read the data received
//     if (rec_flag == 0)
//     {
//       if (serial_data == 0xff)
//       {
//         rec_flag = 1;
//         i = 0;
//         Costtime = 0;
//       }
//     }
//     else
//     {
//       if (serial_data == 0xff)
//       {
//         rec_flag = 0;
//         if (i == 5)
//         {
//           Communication_Decode();
//         }
//         i = 0;
//       }
//       else
//       {
//         buffer[i] = serial_data;
//         i++;
//       }
//     }
// }

// ISR(USART0_RX_vect)                     
// {
//   UCSR0B &= ~(1 << RXCIE0);         
//   Get_uartdata(); 
//   UCSR0B |=  (1 << RXCIE0);         
// }



// void setup() {
//   pinMode(ENA, OUTPUT);
//   pinMode(ENB, OUTPUT);

//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(IN3, OUTPUT);
//   pinMode(IN4, OUTPUT);

//   // analogWrite(ENB, left_speed);
//   // analogWrite(ENA, right_speed);

//   servo1.attach(SERVO_1_PIN);
//   servo2.attach(SERVO_2_PIN);
  
//   USART_init(); 
// }

// void loop() {
//   UartTimeoutCheck(); 
// }


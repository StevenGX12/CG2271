// #include <Arduino.h>
// #include <Servo.h>

// // STATE FLAG
// int two_period_state_flag = 0;
// int three_period_state_flag = 0;


// // PIN DECLARATIONS
// int ENA = 5;  // Enable A --> Right Speed
// int ENB = 6;  // Enable B --> Left Speed
// int IN1 = 8;  // Motor Interface 1 
// int IN2 = 7;  // Motor Interface 2 
// int IN3 = 12; // Motor Interface 3 
// int IN4 = 13; // Motor Interface 4 
// int ledpin1 = A6;                   // System Startup Indicator 1
// int ledpin2 = A7;                   // System Startup Indicator 2
// int SERVO_1_PIN = 11;


// int left_speed = 65;
// int right_speed = 65;   // 0 - 255

// int buffer[5];                       
// int serial_data;  
// int rec_flag;
// unsigned long Costtime;              

// // Servo Declarations
// Servo servo_1;

// int servo_1_spd = 0;      // Accept speeds of -3, -2, -1, 0, 1, 2, 3
// int servo_1_val = 0;
// #define servo_1_low_bound 0
// #define servo_1_high_bound 180

// #define BASE_FWD {digitalWrite(IN1, LOW); digitalWrite(IN2,HIGH);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);}
// #define BASE_REV {digitalWrite(IN1, HIGH); digitalWrite(IN2,LOW);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);}
// #define BASE_RIGHT {digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);}      
// #define BASE_LEFT {digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);}     
// #define BASE_STOP {digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);} 

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

// // void UartTimeoutCheck(void)
// // {
// //   if (rec_flag == 1)
// //   {
// //     Costtime++;
// //     if (Costtime == 100000)
// //     {
// //       rec_flag = 0;
// //     }
// //   }
// // }

// void UartTimeoutCheck(void)
// {
//   if (rec_flag == 1)
//   {
//     Costtime++;
//     if (Costtime == 30)
//     {
//       rec_flag = 0;
//     }
//   }
// }

// void Communication_Decode()
// {

//   // Base Control
//   int signed_left_speed = buffer[0] - 127;    // Accept speeds of +/-127. Zero-shift the inputs
//   int signed_right_speed = buffer[1] - 127;
  
//   // Assign Speed
//   left_speed = abs(signed_left_speed) * 2;
//   right_speed = abs(signed_right_speed) * 2;
  
//   // Assign Direction based on Signed Speed
//   if (signed_left_speed==0 && signed_right_speed==0){
//     BASE_STOP;
//   }else if (signed_left_speed > 0 && signed_right_speed > 0){
//     BASE_FWD;
//   }else if (signed_left_speed < 0 && signed_right_speed < 0){
//     BASE_REV;
//   }else if ((signed_left_speed > 0 && signed_right_speed <= 0)||(signed_left_speed == 0 && signed_right_speed < 0)){
//     BASE_RIGHT;
//   }else if ((signed_left_speed <= 0 && signed_right_speed > 0)||(signed_left_speed < 0 && signed_right_speed == 0)){
//     BASE_LEFT;
//   }

//   analogWrite(ENB, left_speed);
//   analogWrite(ENA, right_speed);  

//   // Servo Control
//   servo_1_spd = buffer[2] - 3;  // Send 0, 1, 2, 3, 4, 5, 6, to correspond to speeds -3, -2, -1, 0, 1, 2, 3


// }

// void Get_uartdata(void)
// {
//    static int i;
//     serial_data = UDR0;
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

// void  Delayed()   
// {
//   int i;
//   for (i = 0; i < 25; i++)
//   {
//     digitalWrite(ledpin1, HIGH);
//     digitalWrite(ledpin2, LOW);
//     delay(1000);
//     digitalWrite(ledpin1, LOW);
//     digitalWrite(ledpin2, HIGH);
//     delay(1000);
//   }
  
//   for (i = 0; i < 10; i++)
//   {
//     digitalWrite(ledpin1, HIGH);
//     digitalWrite(ledpin2, HIGH);
//     delay(500);
//     digitalWrite(ledpin1, LOW);
//     digitalWrite(ledpin2, LOW);
//     delay(500);
//   }
//   BASE_STOP;
//   digitalWrite(ledpin1, LOW);
//   digitalWrite(ledpin2, LOW);
// }

// void setup() {
//   pinMode(ENA, OUTPUT);
//   pinMode(ENB, OUTPUT);

//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(IN3, OUTPUT);
//   pinMode(IN4, OUTPUT);

//   Delayed(); 

//   servo_1.attach(SERVO_1_PIN);
//   USART_init(); 
// }

// void loop() {
//   while (1){
//     UartTimeoutCheck();

//     // Servo 1 Controls
//     switch (servo_1_spd){
//       case -3:
//         // Neg Shift towards 0 at every clock cycle
//         servo_1_val = (servo_1_val <= servo_1_low_bound ? servo_1_low_bound : servo_1_val - 1);
//         servo_1.write(servo_1_val);
//         break;
//       case -2:
//         // Neg Shift towards 0 at every 2x clock cycle
//         if (!two_period_state_flag){
//           servo_1_val = (servo_1_val <= servo_1_low_bound ? servo_1_low_bound : servo_1_val - 1);
//           servo_1.write(servo_1_val);
//         }
//         break;
//       case -1:
//         // Neg Shift towards 0 at every 3x clock cycle
//         if (!three_period_state_flag){
//           servo_1_val = (servo_1_val <= servo_1_low_bound ? servo_1_low_bound : servo_1_val - 1);
//           servo_1.write(servo_1_val);
//         }
//         break;
//       case 1:
//         // Positive Shift towards 0 at every 3x clock cycle
//         if (!three_period_state_flag){
//           servo_1_val = (servo_1_val >= servo_1_high_bound ? servo_1_high_bound : servo_1_val + 1);
//           servo_1.write(servo_1_val);
//         }
//         break;
//       case 2:
//         // Positive Shift towards 0 at every 2x clock cycle
//         if (!two_period_state_flag){
//           servo_1_val = (servo_1_val >= servo_1_high_bound ? servo_1_high_bound : servo_1_val + 1);
//           servo_1.write(servo_1_val);
//         }
//         break;
//       case 3:
//         // Positive Shift towards 0 at every clock cycle
//         servo_1_val = (servo_1_val >= servo_1_high_bound ? servo_1_high_bound : servo_1_val + 1);
//         servo_1.write(servo_1_val);
//         break;
//       default:
//         break;
//     }

//     // State Machine Setting
//     two_period_state_flag = (two_period_state_flag >=1 ? 0 : 1);
//     three_period_state_flag = (three_period_state_flag >=2 ? 0 : three_period_state_flag+1);


//     delay(10);
//   } 
// }

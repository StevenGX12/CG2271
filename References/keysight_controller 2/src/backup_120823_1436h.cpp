// #include <Arduino.h>

// int ENA = 5;  // Enable A --> Right Speed
// int ENB = 6;  // Enable B --> Left Speed
// int IN1 = 8;  // Motor Interface 1 
// int IN2 = 7;  // Motor Interface 2 
// int IN3 = 12; // Motor Interface 3 
// int IN4 = 13; // Motor Interface 4 


// int left_speed = 65;
// int right_speed = 65;   // 0 - 255

// int buffer[3];                       
// int serial_data;  
// int rec_flag;
// unsigned long Costtime;              


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
//   if (buffer[0] == 0x00)
//   {
//     int signed_left_speed;
//     int signed_right_speed;
//     signed_left_speed = buffer[1] - 127;    // Accept speeds of +/-127
//     signed_right_speed = buffer[2] - 127;
//     if (signed_left_speed > 127){
//       signed_left_speed = 127;
//     }
//     if (signed_right_speed > 127){
//       signed_right_speed = 127;
//     }
    


//     switch (buffer[1])  
//     {
//       case 0x01: BASE_FWD;    return;
//       case 0x02: BASE_REV;     return;
//       case 0x03: BASE_LEFT;     return;
//       case 0x04: BASE_RIGHT;    return;
//       case 0x00: BASE_STOP;     return;
//     }
//   }
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
//         if (i == 3)
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

//   analogWrite(ENB, left_speed);
//   analogWrite(ENA, right_speed);
//   USART_init(); 
// }

// void loop() {
//   UartTimeoutCheck(); 
// }


/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * Helper Functions
 *---------------------------------------------------------------------------*/
#define MASK(x) (1 << (x))

/*----------------------------------------------------------------------------
 * LED pins
 *---------------------------------------------------------------------------*/
#define RED_LED 31 // PortE Pin 31
#define GREEN_LED_1 0 // PortC Pin 0
#define GREEN_LED_2 3 // PortC Pin 3
#define GREEN_LED_3 4 // PortC Pin 4
#define GREEN_LED_4 5 // PortC Pin 5
#define GREEN_LED_5 6 // PortC Pin 6
#define GREEN_LED_6 7 // PortC Pin 7
#define GREEN_LED_7 10 // PortC Pin 10
#define GREEN_LED_8 11 // PortC Pin 11
 
/*----------------------------------------------------------------------------
 * Motor control
 *---------------------------------------------------------------------------*/
#define Left_Front_Motor_Pin_1 20 // Timer 1 Ch 0 PortE Pin 20
#define Left_Front_Motor_Pin_2 21 // Timer 1 Ch 1 PortE Pin 21

#define Left_Back_Motor_Pin_1 29 // Timer 0 Ch 2 PortE Pin 29
#define Left_Back_Motor_Pin_2 30 // Timer 0 Ch 3 PortE Pin 30

#define Right_Front_Motor_Pin_1 0 // Timer 0 Ch 0 PortD Pin 0
#define Right_Front_Motor_Pin_2 1 // Timer 0 Ch 1 PortD Pin 1

#define Right_Back_Motor_Pin_1 4 // Timer 0 Ch 4 PortD Pin 4
#define Right_Back_Motor_Pin_2 5 // Timer 0 Ch 5 PortD Pin 5

#define MOD_VAL 7500 //determines period of pwm pulse

//wheel movement direction states
typedef enum {LFF, LFR, LBF, LBR, RFF, RFR, RBF, RBR} wheels;

/*----------------------------------------------------------------------------
 * Buzzer
 *---------------------------------------------------------------------------*/
 #define BUZZER 1 //PortA Pin 1
 
/*----------------------------------------------------------------------------
 * Serial Comms
 *---------------------------------------------------------------------------*/
#define BAUD_RATE 9600
#define UART_TX 22 //PortE Pin 22
#define UART_RX 23 //PortE Pin 23

#define UART2_INT_PRIO 0 //UART interrupt should be of highest priority 
 
 /*----------------------------------------------------------------------------
 * Configure GPIO pins (used for LED, Motors, Buzzer)
 *---------------------------------------------------------------------------*/ 
 void initPower(void){
		 // Enable Clock to PORTA, PORTC, PORTD and PORTE
    SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK) | (SIM_SCGC5_PORTE_MASK));
 }
 
/*----------------------------------------------------------------------------
 * Configure GPIO pins (used for LEDs)
 *---------------------------------------------------------------------------*/
 void initGPIO(void)
 {  
    // Configure GPIO Pins
    PORTE->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[RED_LED] |= PORT_PCR_MUX(1);
    PORTC->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1);
    PORTC->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1);
		PORTC->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1);
		PORTC->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1);
		PORTC->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1);
		PORTC->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1);
		PORTC->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1);
		PORTC->PCR[GREEN_LED_8] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[GREEN_LED_8] |= PORT_PCR_MUX(1);
    
    // Set all to output 
    PTC->PDDR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8));
    PTE->PDDR |= MASK(RED_LED);
		
		//Enable pull-up/down resistors
		PORTE->PCR[RED_LED] |= PORT_PCR_PE_MASK;
		PORTC->PCR[GREEN_LED_1] |= PORT_PCR_PE_MASK;
		PORTC->PCR[GREEN_LED_2] |= PORT_PCR_PE_MASK;
		PORTC->PCR[GREEN_LED_3] |= PORT_PCR_PE_MASK;
		PORTC->PCR[GREEN_LED_4] |= PORT_PCR_PE_MASK;
		PORTC->PCR[GREEN_LED_5] |= PORT_PCR_PE_MASK;
		PORTC->PCR[GREEN_LED_6] |= PORT_PCR_PE_MASK;
		PORTC->PCR[GREEN_LED_7] |= PORT_PCR_PE_MASK;
		PORTC->PCR[GREEN_LED_8] |= PORT_PCR_PE_MASK;
		
		//Enable pull-up resistors
		PORTE->PCR[RED_LED] |= PORT_PCR_PS_MASK;
		PORTC->PCR[GREEN_LED_1] |= PORT_PCR_PS_MASK;
		PORTC->PCR[GREEN_LED_2] |= PORT_PCR_PS_MASK;
		PORTC->PCR[GREEN_LED_3] |= PORT_PCR_PS_MASK;
		PORTC->PCR[GREEN_LED_4] |= PORT_PCR_PS_MASK;
		PORTC->PCR[GREEN_LED_5] |= PORT_PCR_PS_MASK;
		PORTC->PCR[GREEN_LED_6] |= PORT_PCR_PS_MASK;
		PORTC->PCR[GREEN_LED_7] |= PORT_PCR_PS_MASK;
		PORTC->PCR[GREEN_LED_8] |= PORT_PCR_PS_MASK;
}
 
/*----------------------------------------------------------------------------
 * Configure PWM ports (for motor and buzzer)
 *---------------------------------------------------------------------------*/
void initPWM(void)
{
	//Configure Mode 3 for the PWM pin operation (compared to GPIO selection 1)
	PORTE->PCR[Left_Front_Motor_Pin_1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[Left_Front_Motor_Pin_1] |= PORT_PCR_MUX(3);
	
	PORTE->PCR[Left_Front_Motor_Pin_2] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[Left_Front_Motor_Pin_2] |= PORT_PCR_MUX(3);
	
	PORTE->PCR[Left_Back_Motor_Pin_1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[Left_Back_Motor_Pin_1] |= PORT_PCR_MUX(3);
	
	PORTE->PCR[Left_Back_Motor_Pin_2] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[Left_Back_Motor_Pin_2] |= PORT_PCR_MUX(3);
	
	PORTD->PCR[Right_Front_Motor_Pin_1] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[Right_Front_Motor_Pin_1] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[Right_Front_Motor_Pin_2] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[Right_Front_Motor_Pin_2] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[Right_Back_Motor_Pin_1] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[Right_Back_Motor_Pin_1] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[Right_Back_Motor_Pin_2] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[Right_Back_Motor_Pin_2] |= PORT_PCR_MUX(4);
	
	//Enable Clock Gating for Timer0 and Timer1
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	//Select clock for TPM Module
	SIM->SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //MCGFLLCLK or MCGPLLCLK/2
	SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
	
	//Set Modulo Value 20971520 / 128 = 163840 / 3276 = 50Hz
	// TPM1->MOD = 3276;
	
	//Set Modulo Value 48000000 / 128 = 375000 / 7500 = 50Hz
	// Current board uses 48Mhz
	TPM0->MOD = MOD_VAL;
	TPM1->MOD = MOD_VAL; // counter counts before resetting
	
	//Set compare values
	TPM0_C2V = 0;
	TPM0_C3V = 0;
	
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	
	TPM0_C0V = 0;
	TPM0_C1V = 0;
	
	TPM0_C4V = 0;
	TPM0_C5V = 0;
	
	//Edge-Aligned PWM
	//Update SnC resiter: CMOD = 01, PS=111 (128)
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); //Clear the bits for CMOD and PS
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); //Update correct settings
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK); //Clear the bit, operate in up counting mode/edge-aligned
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); //Clear the bits for CMOD and PS
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); //Update correct settings
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK); //Clear the bit, operate in up counting mode/edge-aligned
	
	//Enable PWM on TPM1 Channel 0 -> PTB0
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); //High True Pulses
	
	//Enable PWM on TPM1 Channel 1 -> PTB1
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); //High True Pulses
	
	//Enable PWM on TPM0 Channel 2 -> PTB0
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); //High True Pulses
	
	//Enable PWM on TPM0 Channel 3 -> PTB1
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); //High True Pulses
	
	//Enable PWM on TPM0 Channel 0 -> PTB1
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); //High True Pulses
	
	//Enable PWM on TPM0 Channel 1 -> PTB1
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); //High True Pulses
	
	//Enable PWM on TPM0 Channel 4 -> PTB1
	TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); //High True Pulses
	
	//Enable PWM on TPM0 Channel 5 -> PTB1
	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); //High True Pulses
}

 
/*----------------------------------------------------------------------------
 * Configure UART2 ports (for ESP32 serial comms)
 *---------------------------------------------------------------------------*/
void initUART2(void){
	
	uint32_t divisor, bus_clock;
	
	//enable clock to UART2
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	
	//config as UART pins
	PORTE->PCR[UART_TX] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX] |= PORT_PCR_MUX(4);
	PORTE->PCR[UART_RX] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX] |= PORT_PCR_MUX(4);
	
	//Ensure Tx and Rx are disabled before configuration
	UART2->C2 &= ~(UART_C2_RE_MASK);
	
	//Set Baud Rate to 9600
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock/(BAUD_RATE * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	// No Parity, 8-bits
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	// Configure Tx/Rx Interrupts
	UART2->C2  |= UART_C2_TIE(0);  // Tx Interrupt disabled
	UART2->C2  |= UART_C2_TCIE(0); // Tx Complete Interrupt disabled
	UART2->C2  |= UART_C2_RIE(1);    // Rx Interrupt enabled
	
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	//Enable Tx and Rx
	UART2->C2 |= (UART_C2_RE_MASK);
}

/*----------------------------------------------------------------------------
 * UART2 circular buffer for storing data
 *---------------------------------------------------------------------------*/
#define BUFFER_SIZE (32) //buffer can hold up to 2 instructions at once 
#define STARTBYTE 0x03 //marks start of an instruction set
#define ENDBYTE 0x7F //marks end of an instruction set

typedef struct {
	uint8_t data[BUFFER_SIZE];
	unsigned int head; // data read out from head, head increments on read out
	unsigned int tail; // data read into tail, increment for each data read in
	unsigned int size; // number of elements in queue
} CIRCLE_BUFF;

CIRCLE_BUFF rx_q;

//initiailise RX buffer with 0 items
void initBuffer(CIRCLE_BUFF* q){
	q->head = 0;
	q->tail = 0;
	q->size = 0;
	for(int i=0; i<BUFFER_SIZE; i++){
		q->data[i] = 0;
	}
}

//check if buffer is empty, cannot read out if empty
int bufferEmpty(CIRCLE_BUFF* q){
	if(q->size == 0 && q->tail == q->head){
		return 1;
	} else return 0;
}	

//check if buffer is full, cannot read into if full
int bufferFull(CIRCLE_BUFF* q){
	if(q->size == BUFFER_SIZE && q->tail == q->head){
		return 1;
	} else return 0;
}	

//if buffer not full, read in new data 
void readIntoBuffer(CIRCLE_BUFF* q, uint8_t d){
	if(!bufferFull(q)){
		q->data[q->tail] = d;
		q->tail++;
		q->tail %= BUFFER_SIZE;
		q->size++;
	}
}
//if buffer not empty, read out new data 
uint8_t readOutFromBuffer(CIRCLE_BUFF* q){
	unsigned char d = '\0';	//defaults to null
	if(!bufferEmpty(q)){
			d = q->data[q->head];
			q->head++;
			q->head %= BUFFER_SIZE;
			q->size--;
	}
	return d;
}

//read out data without storing them
void advanceBuffer(CIRCLE_BUFF* q){
	if(!bufferEmpty(q)){
			q->head++;
			q->head %= BUFFER_SIZE;
			q->size--;
	}
}

//checks if a full set of instruction is at the end of queue
int hasCompleteInstruction(CIRCLE_BUFF* q){
	int startIndex = q->head;
	int endIndex = q->head + 3;
	if(q->data[startIndex] == STARTBYTE && q->data[endIndex] == ENDBYTE) {
		return 1;
	}
	else return 0;
}

int hasCompleteDataPacket(CIRCLE_BUFF* q){
	if (q->size % 4 == 0){
		return 1;
	} else return 0;
}

//check byte at end of queue without reading out 
uint8_t peekBuffer(CIRCLE_BUFF* q){
	return q->data[q->head];
}

//read out one instruction (4 bytes) at a time
void decodeInstruction(CIRCLE_BUFF* q, int* leftDir, int* rightDir, int* leftSpeed, int* rightSpeed){
	//ensure buffer stores complete data packets 
	if(hasCompleteDataPacket(q)){
		uint8_t leftControl;
		uint8_t rightControl;
		//if full instruction found, extract both movement bytes
		if(hasCompleteInstruction(q)){
			advanceBuffer(q);
			leftControl = readOutFromBuffer(q);
			*leftSpeed = ((leftControl>>2) & (0b00111111)) * MOD_VAL * 0.6 / 63 + 0.4 * MOD_VAL;
			*leftDir = (leftControl & 0b00000011);
			rightControl = readOutFromBuffer(q);
			*rightSpeed = ((rightControl>>2) & (0b00111111)) * MOD_VAL * 0.6 / 63 + 0.4 * MOD_VAL;
			*rightDir = (rightControl & 0b00000011);
			advanceBuffer(q);
		}
		//keep searching for a complete instruction packet 
		else {
			while(!hasCompleteInstruction(q)){
				advanceBuffer(q);
			}
		}
	}
}


/*----------------------------------------------------------------------------
 * UART2 Interrupt for receiving and sending data 
 *---------------------------------------------------------------------------*/
volatile int data_count = 0;

void UART2_IRQHandler(){
	//if receiver ready, read data into buffer 1 byte at a time
	if(UART2->S1 & UART_S1_RDRF_MASK){
		readIntoBuffer(&rx_q, UART2->D);
		data_count++;
	}
}

/*----------------------------------------------------------------------------
 * LED Control Functions
 *---------------------------------------------------------------------------*/
void turnOnLED(int LED){
	if (LED == 31){
		PTE->PSOR = MASK(LED);
	} else {
		PTC->PSOR = MASK(LED);
	}
}

void turnOffLED(int LED){
		if (LED == 31){
		PTE->PCOR = MASK(LED);
	} else {
		PTC->PCOR = MASK(LED);
	}
}

//turn on all green led together
void turnOnAllGreenLED(){
	turnOnLED(GREEN_LED_1);
	turnOnLED(GREEN_LED_2);
	turnOnLED(GREEN_LED_3);
	turnOnLED(GREEN_LED_4);
	turnOnLED(GREEN_LED_5);
	turnOnLED(GREEN_LED_6);
	turnOnLED(GREEN_LED_7);
	turnOnLED(GREEN_LED_8);
}

//light up green LED one by one
void runningGreenLED(){
	turnOnLED(GREEN_LED_1);
	osDelay(100);
	turnOffLED(GREEN_LED_1);
	turnOnLED(GREEN_LED_2);
	osDelay(100);
	turnOffLED(GREEN_LED_2);
	turnOnLED(GREEN_LED_3);
	osDelay(100);
	turnOffLED(GREEN_LED_3);
	turnOnLED(GREEN_LED_4);
	osDelay(100);
	turnOffLED(GREEN_LED_4);
	turnOnLED(GREEN_LED_5);
	osDelay(100);
	turnOffLED(GREEN_LED_5);
	turnOnLED(GREEN_LED_6);
	osDelay(100);
	turnOffLED(GREEN_LED_6);
	turnOnLED(GREEN_LED_7);
	osDelay(100);
	turnOffLED(GREEN_LED_7);
	turnOnLED(GREEN_LED_8);
	osDelay(100);
	turnOffLED(GREEN_LED_8);
}

//flash all red LED at a period in ms
void redLEDFlash(int period){
	turnOnLED(RED_LED);
	osDelay(period);
	turnOffLED(RED_LED);
	osDelay(period);
}

/*----------------------------------------------------------------------------
 *Motor Control Functions
 *---------------------------------------------------------------------------*/

//configure PWM pulse the right motor based on direction and speed
//00: stop, 01: forward, 10: reverse
void WheelControl(wheels wheelConfig, int spd){
	switch(wheelConfig){
		case LFF:
			TPM1_C0V = spd;
			TPM1_C1V = 0;
			break;
		case LFR:
			TPM1_C0V = 0;
			TPM1_C1V = spd;
			break;
		case LBF:
			TPM0_C2V = spd;
			TPM0_C3V = 0;
			break;
		case LBR:
			TPM0_C2V = 0;
			TPM0_C3V = spd;
			break;
		case RFF:
			TPM0_C0V = spd;
			TPM0_C1V = 0;
			break;
		case RFR:
			TPM0_C0V = 0;
			TPM0_C1V = spd;
			break;
		case RBF:
			TPM0_C4V = spd;
			TPM0_C5V = 0;
			break;
		case RBR:
			TPM0_C4V = 0;
			TPM0_C5V = spd;
			break;
	}
}

void left_forward(int spd){
	WheelControl(LFF, spd);
	WheelControl(LBF, spd);
}

void right_forward(int spd){
	WheelControl(RFF, spd);
	WheelControl(RBF, spd);
}

void left_reverse(int spd){
	WheelControl(LFR, spd);
	WheelControl(LBR, spd);
}

void right_reverse(int spd){
	WheelControl(RFR, spd);
	WheelControl(RBR, spd);
}

void stop_left(){
	WheelControl(LFF, 0);
	WheelControl(LFR, 0);
	WheelControl(LBF, 0);
	WheelControl(LBR, 0);
}

void stop_right(){
	WheelControl(RFF, 0);
	WheelControl(RFR, 0);
	WheelControl(RBF, 0);
	WheelControl(RBR, 0);
}

//perform movement based on command received from ESP32
void move(int left_spd, int right_spd, int left_dir, int right_dir){

	switch(left_dir){
		case 0:
			stop_left();
			break;
		case 1:
			left_forward(left_spd);
			break;
		case 2:
			left_reverse(left_spd);
			break;
	}

	switch(right_dir){
		case 0:
			stop_right();
			break;
		case 1:
			right_forward(right_spd);
			break;
		case 2:
			right_reverse(right_spd);
			break;
	}
	
}

/*----------------------------------------------------------------------------
 * Global variables for car state and control
 *---------------------------------------------------------------------------*/
//direction of left motors
int leftDir = 0;
//direction of right motors
int rightDir = 0;
//speed of left motors
int leftSpeed = 0;
//speed of right motors
int rightSpeed = 0;

/*----------------------------------------------------------------------------
 * Semaphores for threads
 *---------------------------------------------------------------------------*/
osSemaphoreId_t motor_sem; //makes sure motor thread runs only after decode is done

/*----------------------------------------------------------------------------
 * ESP32 Parsing Thread
 *---------------------------------------------------------------------------*/
void parse_command_thread(){
	//keep trying to receive new instructions from the buffer
	for(;;){
		decodeInstruction(&rx_q, &leftDir, &rightDir, &leftSpeed, &rightSpeed);
	}
}
	
/*----------------------------------------------------------------------------
 * Red LED Control Thread
 *---------------------------------------------------------------------------*/
void red_LED_thread (void *argument) {
  for (;;) {
		//if not moving
		if(leftDir == 0 && rightDir == 0){
			redLEDFlash(250);
		} else {
			redLEDFlash(500);
		}
	}
}

/*----------------------------------------------------------------------------
 * Green LED Control Thread
 *---------------------------------------------------------------------------*/
void green_LED_thread (void *argument) {
  for (;;) {
		//if not moving
		if(leftDir == 0 && rightDir == 0){
			turnOnAllGreenLED();
		} else {
			runningGreenLED();
		}
	}
}

/*----------------------------------------------------------------------------
 * Motor Control Thread
 *---------------------------------------------------------------------------*/
void motor_thread (void *argument) {
  for (;;) {
		move(leftSpeed, rightSpeed, leftDir, rightDir);
	}
}

/*----------------------------------------------------------------------------
 * Music Control Thread
 *---------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Thread Configurations 
 *---------------------------------------------------------------------------*/
const osThreadAttr_t parse_command_thread_attr = {
  .priority = osPriorityHigh                      
};

const osThreadAttr_t motor_thread_attr = {
  .priority = osPriorityHigh                      
};

const osThreadAttr_t green_LED_thread_attr = {
  .priority = osPriorityHigh                     
};

const osThreadAttr_t red_LED_thread_attr = {
  .priority = osPriorityHigh                     
};
 
int main (void) {
	
 SystemCoreClockUpdate();
	
	initPower();
	initGPIO();
	initPWM();
	initUART2();
	initBuffer(&rx_q);
	
	move(MOD_VAL, MOD_VAL/2, 1, 2);
	
  osKernelInitialize();
	motor_sem = osSemaphoreNew(1, 0, NULL);	
  osThreadNew(parse_command_thread, NULL, &parse_command_thread_attr);
	osThreadNew(motor_thread, NULL, &motor_thread_attr);
	osThreadNew(green_LED_thread, NULL, &green_LED_thread_attr);
	osThreadNew(red_LED_thread, NULL, &red_LED_thread_attr);
  osKernelStart();                      
  for (;;) {}

}



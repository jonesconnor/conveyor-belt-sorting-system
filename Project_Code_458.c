
// Include statements
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "mtimer.h"
#include "lcd.h"
#include "LinkedQueue.h"

// Define Ranges for part differentiation
#define RANGE_1 179 // Limit between Aluminum and Steel
#define RANGE_2 711 // Steel and White
#define RANGE_3 923 // White and Black

// Define stepper port values for each position
#define STEPINCREMENT 1.8
#define POS_0 0b00011011
#define POS_1 0b00011101
#define POS_2 0b00101101
#define POS_3 0b00101011

// Size of stepper acceleration array
#define timerArraySize 22

// Different belt speeds depending if parts are in FIFO or not
#define SORTINGSPEED 0x40
#define EMPTYSPEED 0x80

// Initial delay for stepper motor homing
volatile int TIMER_DELAY = 1500;

//int stepDelay[] = {300, 233, 197, 174, 158, 145, 135, 127, 120, 114, 109, 105};
//int stepDelay[] = {2000, 1560, 1320, 1160, 1050, 870, 900, 850, 800, 760, 730, 700};
//int stepDelay[] = {1500, 1167, 987, 871, 788, 725, 675, 634, 600, 571, 545, 523, 503, 486, 470, 455, 442, 430, 419, 408};
//int stepDelay[] = {1800, 1400, 1185, 1045, 946, 870, 810, 761, 720, 685, 654, 628, 604, 583, 564, 546, 530, 516, 502, 490};
//int stepDelay[] = {1650, 1283, 1086, 958, 867, 798, 743, 698, 660, 628, 600, 575, 554, 534, 517, 501, 486, 473, 461, 449};
// Stepper acceleration profile
int stepDelay[] = {1800, 1400, 1185, 1045, 946, 870, 810, 761, 720, 685, 654, 628, 604, 583, 564, 546, 530, 516, 502, 490, 479, 468};
//int stepDelay[] = {3200, 2489, 2106, 1858, 1650, 1283, 1086, 958, 867, 798, 743, 698, 660, 628, 600, 575, 554, 534, 517, 501, 486, 473};

// Track which state is active
volatile char STATE;

// Track number of times exit sensor has been triggered so
// as not to miss an item
volatile int exitCounter = 0;

// Minimum value detected by ADC
volatile int ADC_Val = 1023;

// Flag to indicate whether the pause button has been pressed
volatile int isPaused = 0;

// USED IN REFLECTIVE STAGE BUT NOWHERE ELSE
volatile int count = 0;

// Store classification of each item
volatile int itemClass = 0;

// NOT USED
volatile int lastItem = 0;
volatile int sameItem = 0;

// Store each value output by ADC for comparison
volatile int ADC_result;

// Variable to track the phases of the stepper
volatile int curPhase = 0;

// Flag to indicate hall effect sensor triggered
volatile int HE_Found = 0;

// Track current position of stepper
volatile int Position = 0;

// Target position of stepper
volatile int targPos = 0;

// Counters to track number of sorted parts
volatile int numBL = 0;
volatile int numWH = 0;
volatile int numST = 0;
volatile int numAL = 0;

// NOT USED
volatile int stepperComplete = 1;

// Linked list elements
link *head = NULL;
link *tail = NULL;
link *newLink = NULL;
link *rtnLink = NULL;
element e;

int main(int argc, char *argv[]){
	
	// Initialize clock and mTimer
	initClockandTimer();
	
	// Initialize LCD
	InitLCD(LS_BLINK|LS_ULINE);
	LCDClear();
	
	// Initialize State
	STATE = 0;
	
	// Disables all interrupts
	cli();
	
	// Initialize Ports
	DDRD = 0b11110000;
	DDRE = 0x00;
	DDRC = 0xFF;		
	DDRA = 0xFF;
	DDRB = 0xFF;
	DDRL = 0xFF; //debugging

	// Set up INT0, INT1, INT2 for rising edge
	EICRA |= _BV(ISC01) | _BV(ISC00);
	EICRA |= _BV(ISC11) | _BV(ISC10);
	EICRA |= _BV(ISC21) | _BV(ISC20);
	
	// Set up INT3, INT4 for falling edge
	EICRA |= _BV(ISC31) /*| _BV(ISC30)*/;
	EICRB |= _BV(ISC41);
	
	// Enable External Interrupt Flags
	EIMSK |= (_BV(INT0));
	EIMSK |= (_BV(INT1));
	EIMSK |= (_BV(INT2));
	EIMSK |= (_BV(INT3));
	EIMSK |= (_BV(INT4));
	
	// Initializing PWM and setting duty cycle
	TCCR0A |= 0x03;
	//TIMSK0 |= 0x02;
	TCCR0A |= 0x80;
	TCCR0B |= 0x03;
	OCR0A = 0x00;
	
	// Configure ADC
	ADCSRA |= _BV(ADEN); 
	ADCSRA |= _BV(ADIE);
	ADMUX |= _BV(REFS0);
	
	// Enable all interrupts
	sei();
	
	// Rotate stepper until HE sensor is triggered
	initStepper(&curPhase);
	
	// Start conveyor belt
	PORTB = 0x0E;

	goto POLLING_STAGE;

	// POLLING STATE
	POLLING_STAGE:
	PORTL = 0x10;	// Indicates this state is active
	//OCR0A = 0x40;
	//PORTB = 0x0E;
	if (!isPaused) {
		//OCR0A = 0x4D; //30%
		//OCR0A = 0x67; // 40%
		//OCR0A = 0x5A; // 35%
		mTimer(5000);
		LCDClear();
		if(size(&head, &tail)==0) OCR0A = EMPTYSPEED; //fast
		else OCR0A = SORTINGSPEED;
		
		//PORTB = 0x0E;
	}
	else{
		//PORTB = 0x0F;
		mTimer(5000);
		//LCDClear();
		//LCDWriteIntXY(11,1,ADC_Val, 4);
		LCDWriteStringXY(0,0,"BL");
		LCDWriteIntXY(0,1,numBL, 2);
		LCDWriteStringXY(3,0,"WH");
		LCDWriteIntXY(3,1,numWH, 2);
		LCDWriteStringXY(6,0,"ST");
		LCDWriteIntXY(6,1,numST, 2);
		LCDWriteStringXY(9,0,"AL");
		LCDWriteIntXY(9,1,numAL, 2);
		
		LCDWriteStringXY(12,0,"BELT");
		LCDWriteIntXY(12,1,size(&head, &tail), 2);
		//LCDWriteIntXY(12,1,lastItem, -1);
		// 	//LCDWriteIntXY(0,1,sampleCount, -1);
	}
	
	
	
	

	// 	if (size(&head, &tail)==1){
	// 		if(itemClass==0) drive_stepper(0, &Position, &curPhase);
	// 		else if(itemClass==1) drive_stepper(180, &Position, &curPhase);
	// 		else if(itemClass==2) drive_stepper(-90, &Position, &curPhase);
	// 		else if(itemClass==3) drive_stepper(90, &Position, &curPhase);
	// 	}
	
	switch(STATE){
		case (0) :
		// Start DC motor
		goto POLLING_STAGE;
		break;	//not needed but syntax is correct
		case (1) :
		goto MAGNETIC_STAGE;
		break;
		case (2) :
		goto REFLECTIVE_STAGE;
		break;
		case (3) :
		goto BUCKET_STAGE;
		break;
		case (5) :
		goto END;
		default :
		goto POLLING_STAGE;
	}//switch STATE
	

	MAGNETIC_STAGE:
	// Do whatever is necessary HERE
	//PORTC = 0x01; // Just output pretty lights know you made it here
	//Reset the state variable
	STATE = 0;
	goto POLLING_STAGE;

	REFLECTIVE_STAGE:
	
	count++;
	STATE = 0;
	goto POLLING_STAGE;
	
	BUCKET_STAGE:
	
	while (exitCounter>0){
		//PORTB = 0x0F; // Brake motor
		
		dequeue(&head, &tail, &rtnLink);
		lastItem = rtnLink->e.itemCode;
		
		//PORTL = rtnLink->e.itemCode << 4;
		//LCDWriteIntXY(0,1,rtnLink->e.itemCode, 4);
		
		if(rtnLink->e.itemCode==0) {
			targPos = 0;
			numBL++;
		}
		else if(rtnLink->e.itemCode==1) {
			targPos = 180;
			numWH++;
		}
		else if(rtnLink->e.itemCode==2) {
			targPos = -90;
			numST++;
		}
		else if(rtnLink->e.itemCode==3) {
			targPos = 90;
			numAL++;
		}
		free(rtnLink); // was free(&rtnLink) and that broke everything
		drive_stepper(targPos, &Position, &curPhase);
		//PORTB = 0x0E;
		//mTimer(20000);
		exitCounter--;
	}
	STATE = 0;
	goto POLLING_STAGE;
	
	END:
	// The closing STATE ... how would you get here?
	//PORTC = 0xF0;	// Indicates this state is active
	// Stop everything here...'MAKE SAFE'
	PORTB = 0x0F;
	LCDClear();

	LCDWriteStringXY(0,0,"BL");
	LCDWriteIntXY(0,1,numBL, 2);
	LCDWriteStringXY(3,0,"WH");
	LCDWriteIntXY(3,1,numWH, 2);
	LCDWriteStringXY(6,0,"ST");
	LCDWriteIntXY(6,1,numST, 2);
	LCDWriteStringXY(9,0,"AL");
	LCDWriteIntXY(9,1,numAL, 2);
	return(0);

}

// Ramp Down
ISR(INT0_vect){
	mTimer(3000);
	if(!(PIND & 0b00000001) == 0){
		rampDown();
	}
}

// Pause button
ISR(INT1_vect){
	mTimer(3000);
	if(!(PIND & 0b00000010) == 0){
		//numBL++;
		if (!isPaused) {
			PORTB = 0x0F; // Brake motor to VCC
			isPaused = 1;
		}
		else {
			PORTB = 0x0E; // Conveyor belt forward
			isPaused = 0;
		}
	}
	while(!(PIND & 0b00000010) == 0);
	mTimer(3000);
}

// OR (Reflective) Sensor Interrupt
ISR(INT2_vect){
	mTimer(300);
	if((PIND & 0x04) != 0){
		ADC_Val = 1023;
		ADCSRA |= (1 << ADSC);
		//mTimer(3);
		STATE = 2;
	}
}

// Exit Sensor Interrupt
ISR(INT3_vect){
	/* Toggle PORTC bit 3 */
	//mTimer(300);
	if((PIND & 0x08) == 0) {
		PORTB = 0x0F;	
		//if(!sameItem) PORTB = 0x0F;		
		//if((head->e.itemCode == head->next->e.itemCode) && (head->next != NULL)) sameItem = 1;	
		//else sameItem = 0;	
		//for(double x = 0; x<30000; x++);
		OCR0A = SORTINGSPEED; //25%
		STATE = 3;
		exitCounter++;
	}
	mTimer(300);
}

ISR(INT4_vect) {
	HE_Found = 1;
}

// the interrupt will be triggered if the ADC is done ========================
ISR(ADC_vect) {
	ADC_result = ADC;
	if ((PIND & 0x04) != 0) {
		if (ADC_result < ADC_Val) {
			ADC_Val = ADC_result;
		}
		ADCSRA |= (1 << ADSC);
	}
	else {
		/*int itemClass = 0;*/
		//PORTL = 0x00;
		if (ADC_Val > RANGE_3) itemClass = 0;							// Black
		else if (ADC_Val > RANGE_2 && ADC_Val < RANGE_3) itemClass = 1; // White
		else if (ADC_Val > RANGE_1 && ADC_Val < RANGE_2) itemClass = 2; // Steel
		else itemClass = 3;												// Aluminum
		//PORTL = itemClass << 4;
		initLink(&newLink);
		newLink->e.itemCode = itemClass;
		enqueue(&head, &tail, &newLink);

		//mTimer(500);
	}
}

// If an unexpected interrupt occurs (interrupt is enabled and no handler is installed,
// which usually indicates a bug), then the default action is to reset the device by jumping
// to the reset vector. You can override this by supplying a function named BADISR_vect which
// should be defined with ISR() as such. (The name BADISR_vect is actually an alias for __vector_default.
// The latter must be used inside assembly code in case <avr/interrupt.h> is not included.
ISR(BADISR_vect) {
}


void initStepper(int *curPhase) {
	//int targetPosition = ((*curPhase) + discreteSteps) % 4;
	while(HE_Found == 0) {
		switch(*curPhase) {
			case 0:
			PORTA = POS_1;
			mTimer(TIMER_DELAY);
			*curPhase = 1;
			break;
			
			case 1:
			PORTA = POS_2;
			mTimer(TIMER_DELAY);
			*curPhase = 2;
			break;
			case 2:
			PORTA = POS_3;
			mTimer(TIMER_DELAY);
			*curPhase = 3;
			break;
			case 3:
			PORTA = POS_0;
			mTimer(TIMER_DELAY);
			*curPhase = 0;
			break;
		}
	}
}

void drive_stepper(int targPos,int *curPos, int *curPhase) {
	//take number of degrees and divide by 1.8 to convert to steps -- store in float
	//round the float to nearest integer
	//take current position and run through the number of required steps
	int degrees = targPos - (*curPos);
	if(degrees == 0){
		PORTB = 0x0E;
		mTimer(2000);
	}
	if(degrees>180) degrees-=360;
	else if(degrees<-180) degrees+=360;
	int isCW = 1;
	if (degrees < 0) {
		degrees *= -1;
		isCW = 0;
	}
	float rawSteps = degrees/STEPINCREMENT;
	int discreteSteps = rawSteps;
	if ((rawSteps - discreteSteps) > 0.5) {
		discreteSteps++;
	}
	
	int i = 0;
	for(discreteSteps; discreteSteps > 0; discreteSteps--) {
		if(i<timerArraySize && i<(rawSteps/2)){
			TIMER_DELAY = stepDelay[i];
			i++;
		}
		else if(i==discreteSteps){
			TIMER_DELAY = stepDelay[i-1];
			i--;
		}
		//if(discreteSteps<timerArraySize){
		//	TIMER_DELAY = stepDelay[discreteSteps];
		//}
		
		if(discreteSteps==40){//30 worked pretty well
			PORTB = 0x0E;
			//OCR0A = 0x40; //25%
			OCR0A = 0x60; //50%
		}
		
		else if(discreteSteps==1){
			//PORTB = 0x0F;
			OCR0A = SORTINGSPEED; //25%
			//OCR0A = 0x80; //50%
		}
		
		
		switch(*curPhase) {
			case 0:
			if(isCW) {
				PORTA = POS_1;
				mTimer(TIMER_DELAY);
				*curPhase = 1;
				break;
			}
			else {
				PORTA = POS_3;
				mTimer(TIMER_DELAY);
				*curPhase = 3;
				break;
			}
			case 1:
			if (isCW) {
				PORTA = POS_2;
				mTimer(TIMER_DELAY);
				*curPhase = 2;
				break;
			}
			else {
				PORTA = POS_0;
				mTimer(TIMER_DELAY);
				*curPhase = 0;
				break;
			}
			case 2:
			if (isCW) {
				PORTA = POS_3;
				mTimer(TIMER_DELAY);
				*curPhase = 3;
				break;
			}
			else {
				PORTA = POS_1;
				mTimer(TIMER_DELAY);
				*curPhase = 1;
				break;
			}
			case 3:
			if (isCW) {
				PORTA = POS_0;
				mTimer(TIMER_DELAY);
				*curPhase = 0;
				break;
			}
			else {
				PORTA = POS_2;
				mTimer(TIMER_DELAY);
				*curPhase = 2;
				break;
			}
		}
		*curPos=targPos;
	}
	//PORTB = 0x0E;
	//OCR0A = 0x40; //25%
	mTimer(8000);
}

void rampDown(){
	
	//set pins as outputs


	cli();//stop interrupts
	
	TCCR4A = 0x00;
	
	TCNT4  = 0;
	
	OCR4A = 0xC800;
	
	TCCR4B |= (1 << WGM12); // CTC mode
	
	TCCR4B |= (1 << CS12) | (1 << CS10); // 1024 prescaler
	
	TIMSK4 |= (1 << OCIE4A); // enable timer compare interrupt

	sei();//allow interrupts

}//end setup

ISR(TIMER4_COMPA_vect){
	STATE = 5;
}

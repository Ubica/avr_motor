#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "usvdrv_/usbdrv.h"

#define F_CPU 12000000L
#include <util/delay.h>

#define USB_MOTOR_STOP 0
#define USB_MOTOR_INIT 1

// SINGLE STEP
#define USB_MOTOR_FORWARD 3
#define USB_MOTOR_BACKWARD 4

// FULL CIRCLE
#define USB_MOTOR_FORWARD_360 5
#define USB_MOTOR_BACKWARD_360 6

// MOVE MOTOR A CERTAIN NUMBER OF STEPS
#define USB_MOTOR_STEPS 50

// SPEED
#define USB_MOTOR_SPEED_UP 100
#define USB_MOTOR_SPEED_DOWN 101

// DATA
#define USB_DATA_OUT 200
#define USB_DATA_MOTOR_SPEED 201

const uint16_t SECOND = 46875;
uint8_t speedModifier = 20;

uint8_t motorState = 1;
uint8_t motorStateChange = 1;
uint8_t motorInitialized = 0;
uint8_t lastPosition = 0;
uint8_t currentPosition = 2;
uint16_t steps = 0;

static uchar replyBuf[] = "Message from the USB device, maximum size is 255 bytes. Undefined array of characters";
static uchar replySpeed[3] = {"255"};
static uchar replyFiveDigits[] = {"65535"};

/* PORT OPERATIONS MACROS */
#define toggle(port,pos) ((port) ^= 1 << (pos))
#define setHigh(port,pos) ((port) |= 1 << (pos))
#define setLow(port,pos) ((port) &= 0xff ^ 1 << (pos))

uint16_t getValue(uchar b[]){
	uint16_t result = 0x0000;
	uint8_t first = b[0];
	uint8_t second = b[1];
	result = first;
	result = result << 8;
	result |= second;
	return result;
}

uint16_t powOf10(uint8_t pow){
	uint16_t result = 10;
	for(uint8_t i = 0;i<=pow;i++){
		result*=result;
	}
	return result;
}

void updateFiveDigitsReply(uint16_t value){
	uint8_t remainder = 0;
	uint8_t values[5] = {0,0,0,0,0};
	for(uint8_t i = 4; i>0; i--){
		uint16_t ref = powOf10(i);
		if(value>=ref){
			remainder = value % ref;
			values[i] = (value - remainder)/ref;
			value -= ref * values[i];
		}
	}
	values[0]=value;
	uint8_t r = 0;
	for(uint8_t v = 4;v>=0;v--){
		replyFiveDigits[r] = values[v]+48;
		r++;
	}
}

void updateSpeedReply(uint8_t value){
	uint8_t remainder = 0;
	uint8_t hundreds = 0;
	uint8_t tens = 0;
	if(value>=100){
		remainder = value % 100;
		hundreds = (value - remainder)/100;
		value -= hundreds * 100;
	}
	if(value>=10){
		remainder = value % 10;
		tens = (value - remainder)/10;
		value -= tens * 10;
	}
	replySpeed[0] = hundreds + 48;
	replySpeed[1] = tens + 48;
	replySpeed[2] = value + 48;
}

USB_PUBLIC uchar usbFunctionSetup(uchar data[8]) {
	usbRequest_t *rq = (void *)data; // cast data to correct type
	
	switch(rq->bRequest) { // custom command is in the bRequest field
		
		/* STOP MOTOR */
		case USB_MOTOR_STOP:
		motorState = 0;
		motorInitialized = 0;
		return 0;
		
		/* INIT MOTOR */
		case USB_MOTOR_INIT:
		motorState = 1;
		motorInitialized = 1;
		return 0;
		
		/* FORWARD STEP OF MOTOR */
		case USB_MOTOR_FORWARD:
		motorState = USB_MOTOR_FORWARD;
		steps = 1;
		return 0;
		
		/* BACKWARD STEP OF MOTOR */
		case USB_MOTOR_BACKWARD:
		motorState = USB_MOTOR_BACKWARD;
		steps = 1;
		return 0;
		
		/* FORWARD MOTION OF MOTOR */
		case USB_MOTOR_FORWARD_360:
		motorState = USB_MOTOR_FORWARD;
		steps = 48; // number of steps on the motor to do a 360
		return 0;
		
		/* BACKWARD MOTION OF MOTOR */
		case USB_MOTOR_BACKWARD_360:
		motorState = USB_MOTOR_BACKWARD;
		steps = 48; // number of steps on the motor to do a 360
		return 0;
		
		/* TESTING FORWARD MOTION OF MOTOR FOR A CERTAIN AMOUNT OF STEPS */
		case USB_MOTOR_STEPS:
		{
			//motorState = USB_MOTOR_FORWARD;
			uint16_t valueReceived = getValue(rq->wLength.bytes);
			updateFiveDigitsReply(valueReceived);
			usbMsgPtr = (usbMsgPtr_t)replyFiveDigits;
			return sizeof(replyFiveDigits);
		}
		
		/* SPEED CONTROL */
		case USB_MOTOR_SPEED_UP:
		if(speedModifier<150){
			speedModifier++;
		}
		return 0;
		
		case USB_MOTOR_SPEED_DOWN:
		if(speedModifier>1){
			speedModifier--;
		}
		return 0;
		
		/* USB DATA OUT */
		case USB_DATA_OUT:
		usbMsgPtr = (usbMsgPtr_t)replyBuf;
		return sizeof(replyBuf);
		
		/* MOTOR SPEED UPDATE */
		case USB_DATA_MOTOR_SPEED:
		updateSpeedReply(speedModifier);
		usbMsgPtr = (usbMsgPtr_t)replySpeed;
		return sizeof(replySpeed);
		
	}

	return 0; // should not get here

}

void initMotor(){ // not 100% accurate !!!!!!! depends on the direction of movement
	PORTD |= 1 << ((lastPosition+1)%4+4);
	PORTD |= 1 << (currentPosition%4+4);
}

void stopMotor(){
	PORTD &= 0xff ^ 0xf0;
	steps = 0;
	motorInitialized = 0;
}

void moveMotor(){
	setLow(PORTD,lastPosition%4+4);
	setHigh(PORTD,currentPosition%4+4);
}

void motor(){
	if(motorStateChange!=motorState){
		motorStateChange = motorState;
		{
			// invert the motor state
			int8_t difference = currentPosition - lastPosition;
			if(difference > 0 && motorState == 4)
			{
				currentPosition--;
			}
			else if (difference < 0 && motorState == 3)
			{
				currentPosition++;
			}
		}
	}
	
	if(TCNT1>SECOND/speedModifier){
		TCNT1 = 0;
		switch(motorState){
			case 0:
			stopMotor();
			return;
			
			case 1:
			// motor idle...
			if(!motorInitialized){
				initMotor();
				motorInitialized = 1;
			}
			return;
			
			case 3:
			/* double forward motion */
			if(steps>0){
				steps--;
				lastPosition = currentPosition-1;
				currentPosition++;
				moveMotor();
				} else {
				motorState = 1;
			}
			return;
			
			case 4:
			/* double backward motion */
			if(steps>0){
				steps--;
				lastPosition = currentPosition+1;
				currentPosition--;
				moveMotor();
				} else {
				motorState = 1;
			}
			return;
		}
	}
}

int main() {
	uchar i;
	setHigh(DDRB, PORTB0); // make PB0 output port
	setHigh(PORTB, PORTB0); // indicate that the micro-controller is active
	
	// motor frequency timer
	TCCR1B |= 1 << CS12;
	
	// motor output pins
	setHigh(DDRD,PORTD4);
	setHigh(DDRD,PORTD5);
	setHigh(DDRD,PORTD6);
	setHigh(DDRD,PORTD7);

	wdt_enable(WDTO_1S); // enable 1s watchdog timer

	usbInit();
	
	usbDeviceDisconnect(); // enforce re-enumeration
	for(i = 0; i<250; i++) { // wait 500 ms
		wdt_reset(); // keep the watchdog happy
		_delay_ms(2);
	}
	usbDeviceConnect();
	
	sei(); // Enable interrupts after re-enumeration
	
	initMotor();
	
	while(1) {
		wdt_reset(); // keep the watchdog happy
		usbPoll();
		
		/*motor functions*/
		motor();
	}
	
	return 0;
}
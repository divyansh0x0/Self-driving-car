#include "Sensor.hpp"
#include "Debug.hpp"
#define IN1 50
#define IN2 51
#define IN3 52
#define IN4 53
#define ARRAYLEN(arr) *(&arr + 1) - arr

#define SIZE 4

UltrasonicSensor UltrasonicSensors[SIZE] = {{2,3,FRONT_LEFT},{4,5,FRONT_RIGHT},{6,7,BACK_RIGHT},{8,9,BACK_LEFT}}; //{trig,echo,loc}
IRSensor IrSensors[SIZE] = {{40,FRONT},{41,RIGHT},{42, BACK},{43,LEFT}};


CarMovement RecommendedCarMovement{};

void setup()
{
	setDebugMode(true);
	Serial.begin(9600);

	for (int i = 0; i < ARRAYLEN(UltrasonicSensors); i++)
	{
		uint8_t trig = UltrasonicSensors[i].trigger_pin;
		uint8_t echo = UltrasonicSensors[i].echo_pin;

		pinMode(trig, OUTPUT);
		pinMode(echo, INPUT);
		digitalWrite(echo,LOW);
		digitalWrite(trig,LOW);
	}
	for (int i = 0; i < ARRAYLEN(IrSensors); i++)
	{
		uint8_t out = IrSensors[i].pin;

		pinMode(out, INPUT);
		digitalWrite(out,LOW);
	}
}

void motorControlLeftSide(short spinDir)
{
	switch (spinDir)
	{
	case MOVE_FORWARD:
		digitalWrite(IN1, HIGH);
		digitalWrite(IN2, LOW);
		break;
	case MOVE_BACKWARD:
		digitalWrite(IN1, LOW);
		digitalWrite(IN2, HIGH);
		break;
	default:
		digitalWrite(IN1, LOW);
		digitalWrite(IN2, LOW);
		break;
	}
}
void motorControlRightSide(short spinDir)
{
	switch (spinDir)
	{
	case MOVE_FORWARD:
		digitalWrite(IN3, HIGH);
		digitalWrite(IN4, LOW);
		break;
	case MOVE_BACKWARD:
		digitalWrite(IN3, LOW);
		digitalWrite(IN4, HIGH);
		break;
	default:
		digitalWrite(IN3, LOW);
		digitalWrite(IN4, LOW);
		break;
	}
}
void moveCar()
{
	short spinDir = RecommendedCarMovement.axial;
	log(spinDir > 0 ? "Moving forward" : spinDir < 0 ? "Moving back" : "Car stopped" );

	switch (RecommendedCarMovement.transverse)
	{
	case NO_MOVE:
		motorControlLeftSide(spinDir);
		motorControlRightSide(spinDir);
		break;
	case MOVE_LEFT: 
		log("Turning left");
		motorControlLeftSide(spinDir);
		motorControlRightSide(0);
		break;
	case MOVE_RIGHT:
		log("Turning right");
		motorControlLeftSide(0);
		motorControlRightSide(spinDir);
		break;
	default:
		log("Invalid direction provided, stopping car");
		motorControlLeftSide(0);
		motorControlRightSide(0);
		break;
	};
}


void loop()
{
	recommendCarMove(IrSensors, SIZE, UltrasonicSensors, SIZE, &RecommendedCarMovement, true);
	moveCar();
	delay(50);
	// uint8_t trig_pin = 10;
    // uint8_t echo_pin = 11;
	// digitalWrite(trig_pin,HIGH);
    // delayMicroseconds(10);
    // digitalWrite(trig_pin, LOW);

    // // unsigned int t1 = micros();
    // unsigned long duration_micros = pulseIn(echo_pin, HIGH); 
    // // unsigned int t2 = micros();
	// if (duration_micros / 1000 <  38){
	// 	log("Timeout");
	// }
    // unsigned long distance_cm = duration_micros /58;// division by two because it is a round trip
	// log("Triggering pin:" + String(trig_pin) + ", " + String(echo_pin) + "|| Duration (micros)" + String(duration_micros) + "|| DISTANCE: " + String(distance_cm) + "cm");
	// delay(100);
}

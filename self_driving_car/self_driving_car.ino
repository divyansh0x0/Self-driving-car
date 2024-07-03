#include "Sensor.hpp"
#define IN1 50
#define IN2 51
#define IN3 52
#define IN4 53

UltrasoundSensor UltrasonicSensors[4] = {};
IRSensor IrSensors[6] = {};

// pair of trigger and echo pins
const uint8_t kUltrasonicSensorPins[4][2] = {{22, 23}, {24, 25}, {26, 27}, {28, 29}};

const uint8_t kIrSensorPins[8] = {34, 35, 36, 37, 38, 39, 40, 41};
CarMovement RecommendedCarMovement{};

void setup()
{

	Serial.begin(9600);

	for (int i = 0; i < 4; i++)
	{
		UltrasonicSensors[i].trigger_pin = kUltrasonicSensorPins[i][0];
		pinMode(UltrasonicSensors[i].trigger_pin, OUTPUT);
		UltrasonicSensors[i].echo_pin = kUltrasonicSensorPins[i][1];
		pinMode(UltrasonicSensors[i].echo_pin, INPUT);
	}

	for (int i = 0; i < 6; i++)
	{
		IrSensors[i].pin = kIrSensorPins[i];
	}

	UltrasonicSensors[0].location = FRONT_LEFT;
	UltrasonicSensors[1].location = FRONT_RIGHT;
	UltrasonicSensors[2].location = BACK_RIGHT;
	UltrasonicSensors[3].location = BACK_LEFT;

	IrSensors[0].location = LEFT;
	IrSensors[1].location = FRONT_LEFT;
	IrSensors[2].location = FRONT;
	IrSensors[3].location = FRONT_RIGHT;
	IrSensors[4].location = RIGHT;
	IrSensors[5].location = BACK_RIGHT;
	IrSensors[6].location = BACK;
	IrSensors[7].location = BACK_LEFT;
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
	short spinDir = CarMovement.axial;

	switch (CarMovement.transverse)
	{
	case NO_MOVE:
		motorControlLeftSide(spinDir);
		motorControlLeftSide(spinDir);
		break;
	case MOVE_LEFT: 
		motorControlLeftSide(spinDir);
		motorControlRightSide(0);
		break;
	case MOVE_RIGHT:
		motorControlLeftSide(0);
		motorControlRightSide(spinDir);
		break;
	default:
		Serial.println("Invalid direction provided, stopping car");
		motorControlLeftSide(0);
		motorControlRightSide(0);
		break;
	};
}
void loop()
{

	recommendCarMove(UltrasonicSensors, IrSensors, &CarMovement);
	moveCar();
}

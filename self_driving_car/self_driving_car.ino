#include "Sensor.hpp"
UltraSoundSensor ultrasound_sensors[4] = {};
IRSenor ir_sensors[6] = {};

// pair of trigger and echo pins
uint8_t ultrasound_sensor_pins[4][2] = {{1, 2}, {3, 4}, {4, 5}, {6, 7}};

uint8_t ir_sensor_pins[7] = {8, 9, 10, 11, 12, 13, 14};

void setup()
{
	Serial.begin(9600);

	for (int i = 0; i < 4; i++)
	{
		ultrasound_sensors[i].triggerPin = ultrasound_sensor_pins[i][0];
		ultrasound_sensors[i].echoPin = ultrasound_sensor_pins[i][1];
	}

	for (int i = 0; i < 6; i++)
	{
		ir_sensors[i].pin = ir_sensor_pins[i];
	}

	ultrasound_sensors[0].location = FRONT_LEFT;
	ultrasound_sensors[1].location = FRONT_RIGHT;
	ultrasound_sensors[2].location = BACK_RIGHT;
	ultrasound_sensors[3].location = BACK_LEFT;

	ir_sensors[0].location = LEFT;
	ir_sensors[1].location = FRONT_LEFT;
	ir_sensors[2].location = FRONT;
	ir_sensors[3].location = FRONT_RIGHT;
	ir_sensors[4].location = RIGHT;
	ir_sensors[5].location = BACK_RIGHT;
	ir_sensors[6].location = BACK;
	ir_sensors[7].location = BACK_LEFT;

}

void loop()
{
}

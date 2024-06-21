#include "Sensor.hpp"
#include <Arduino.h>
#define ARRAYLEN(arr) sizeof(arr) / sizeof(arr[0])
#define SOUND_SPEED_CM_PER_MICROS 0.034
#define MIN_DISTANCE_CM 10
#define MIN_DURATION MIN_DISTANCE_CM / SOUND_SPEED_CM_PER_MICROS * 2

int ObstaclesDetectedArr[8] = {};
void readSensor(UltraSoundSensor *ultrasound_sensors, SensorData *sensor_data)
{
    int trigPin = ultrasound_sensors->triggerPin;
    int echoPin = ultrasound_sensors->triggerPin;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(trigPin, LOW);
    unsigned int t1 = micros();
    unsigned long duration_micros = pulseIn(echoPin, HIGH, MIN_DURATION); // multiply by 2 because it is a round trip
    unsigned int t2 = micros();
    if (duration_micros == 0)
    { // I believe it will never be zero except when timed out
        sensor_data->obstacle_status = NO_OBSTACLE;
        sensor_data->obstacle_distance = 0;
    }
    else
    {
        unsigned long distance_cm = duration_micros * SOUND_SPEED_CM_PER_MICROS / 2; // division by two because it is a round trip
        sensor_data->obstacle_distance = distance_cm;
        sensor_data->obstacle_status = OBSTACLE_DETECTED;
    }
}
void readSensor(IRSensor *ir_sensor, SensorData *sensor_data)
{
    int sensorState = digitalRead(ir_sensor->pin);
    if (sensorState == HIGH)
    {
        sensor_data->obstacle_status = OBSTACLE_DETECTED;
    }
    else
    {
        sensor_data->obstacle_status = NO_OBSTACLE;
    }
    sensor_data->obstacle_distance = 0;
}

bool canCarMove()
{
    return (ObstaclesDetectedArr[BACK] | ObstaclesDetectedArr[FRONT]) | (ObstaclesDetectedArr[LEFT] & ObstaclesDetectedArr[FRONT_LEFT] & ObstaclesDetectedArr[FRONT_RIGHT] & ObstaclesDetectedArr[RIGHT] & ObstaclesDetectedArr[BACK_LEFT] & ObstaclesDetectedArr[BACK_RIGHT]) == 1;
}
void recommendCarMove(UltraSoundSensor *ultrasound_sensors, IRSensor *ir_sensors, CarMovement *CarMovement)
{
    int i = 0;
    while (i < ARRAYLEN(ultrasound_sensors))
    {
        readSensor(&ultrasound_sensors[i], &ultrasound_sensors[i].data);
        ++i;
    }
    while (i < ARRAYLEN(ir_sensors))
    {
        readSensor(&ir_sensors[i], &ir_sensors[i].data);
        ++i;
    }
    short prefAxialMove = 0;
    short prefTransverseMove = 0;
    for (int i = ARRAYLEN(ir_sensors) - 1; i >= 0; i--)
    {
        IRSensor *sensor = &ir_sensors[i];
        SensorData *sensor_data = &sensor->data;
        Serial.println("-----------------------------------------------------------");
        String sensorName = "IR " + sensor->location;
        Serial.println("SENSOR_TYPE:" + sensorName);
        Serial.println("Obstacle Status " + (sensor_data->obstacle_status));
        Serial.println("-----------------------------------------------------------");

        if (sensor_data->obstacle_status == OBSTACLE_DETECTED)

            switch (sensor_data->obstacle_status)
            {
            case OBSTACLE_DETECTED:
                Serial.println("\tDistance of obstacle (cm): " + (sensor_data->obstacle_distance));
                ObstaclesDetectedArr[sensor->location] = 1;
                break;

            default:
                ObstaclesDetectedArr[sensor->location] = 0;
                break;
            }
    }
    if (!canCarMove())
    {
        CarMovement->axial = 0;
        CarMovement->transverse = 0;
        return;
    }
    // Read ultrasound sensors
    for (int i = ARRAYLEN(ultrasound_sensors) - 1; i >= 0; i--)
    {
        UltraSoundSensor *sensor = &ultrasound_sensors[i];
        SensorData *sensor_data = &sensor->data;
        Serial.println("-----------------------------------------------------------");
        String sensorName = "ULTRASONIC " + sensor->location;
        Serial.println("SENSOR_TYPE:" + sensorName);
        Serial.println("Obstacle Status " + (sensor_data->obstacle_status));
        Serial.println("-----------------------------------------------------------");

        if (sensor_data->obstacle_status == OBSTACLE_DETECTED)

            switch (sensor_data->obstacle_status)
            {
            case OBSTACLE_DETECTED:
                Serial.println("\tDistance of obstacle (cm): " + (sensor_data->obstacle_distance));
                ObstaclesDetectedArr[sensor->location] = 1;
                break;

            default:
                ObstaclesDetectedArr[sensor->location] = 0;
                break;
            }
    }

    // decide move based on data from ultrasound sensors and left right ir sensors
    // check fronts
    if ((ObstaclesDetectedArr[FRONT_LEFT] & ObstaclesDetectedArr[FRONT_RIGHT]) == 1)
    {
        // if whole forward area is empty
        CarMovement->axial = MOVE_FORWARD;
        CarMovement->transverse = NO_MOVE;
    }
    else if ((ObstaclesDetectedArr[FRONT_RIGHT] | ObstaclesDetectedArr[RIGHT]) == 1)
    {
        // if right area is empty
        CarMovement->axial = MOVE_FORWARD;
        CarMovement->transverse = MOVE_RIGHT;
    }

    else if ((ObstaclesDetectedArr[FRONT_LEFT] | ObstaclesDetectedArr[LEFT]) == 1)
    {
        // if left area is empty
        CarMovement->axial = MOVE_FORWARD;
        CarMovement->transverse = MOVE_LEFT;
    }
    // check back
    else if ((ObstaclesDetectedArr[BACK_LEFT] & ObstaclesDetectedArr[BACK_RIGHT]) == 1)
    {
        // if whole back area is empty
        CarMovement->axial = MOVE_FORWARD;
        CarMovement->transverse = MOVE_RIGHT;
    }
    else if ((ObstaclesDetectedArr[BACK_RIGHT] | ObstaclesDetectedArr[RIGHT]) == 1)
    {
        // if right area is empty
        CarMovement->axial = MOVE_BACKWARD;
        CarMovement->transverse = MOVE_RIGHT;
    }
    else if ((ObstaclesDetectedArr[FRONT_LEFT] | ObstaclesDetectedArr[LEFT]) == 1)
    {
        // if whole right area is empty
        CarMovement->axial = MOVE_BACKWARD;
        CarMovement->transverse = MOVE_LEFT;
    }
}

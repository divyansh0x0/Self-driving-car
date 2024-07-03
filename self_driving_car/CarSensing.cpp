#include "Sensor.hpp"
#include <Arduino.h>
#define ARRAYLEN(arr) sizeof(arr) / sizeof(arr[0])
#define SOUND_SPEED_CM_PER_MICROS 0.034
#define MIN_DISTANCE_CM 10
// multiply by 2 because it is a round trip
#define MAX_DURATION MIN_DISTANCE_CM / SOUND_SPEED_CM_PER_MICROS * 2 

int obstacles_detected_arr[NUM_LOCATIONS];
void readSensor(UltrasoundSensor *ultrasound_sensors, SensorData *sensor_data)
{
    int trig_pin = ultrasound_sensors->trigger_pin;
    int echo_pin = ultrasound_sensors->echo_pin;
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(2);
    digitalWrite(trig_pin, LOW);
    unsigned int t1 = micros();
    unsigned long duration_micros = pulseIn(echo_pin, HIGH, MAX_DURATION); 
    unsigned int t2 = micros();
    if (duration_micros == 0 || duration_micros > MAX_DURATION)
    {
        // Handle timeout or abnormal readings
        sensor_data->obstacle_status = NO_OBSTACLE;
        sensor_data->obstacle_distance = 0;
    }
    else
    {
        unsigned long distance_cm = duration_micros * SOUND_SPEED_CM_PER_MICROS / 2;// division by two because it is a round trip
        sensor_data->obstacle_distance = distance_cm;
        sensor_data->obstacle_status = OBSTACLE_DETECTED;
    }
}
void readSensor(IRSensor *ir_sensor, SensorData *sensor_data)
{
    int sensor_state = digitalRead(ir_sensor->pin);
    if (sensor_state == HIGH)
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
    uint8_t condition1 = (obstacles_detected_arr[BACK] | obstacles_detected_arr[FRONT]);
    uint8_t condition2 = (obstacles_detected_arr[LEFT] & obstacles_detected_arr[FRONT_LEFT] & obstacles_detected_arr[FRONT_RIGHT] & obstacles_detected_arr[RIGHT] & obstacles_detected_arr[BACK_LEFT] & obstacles_detected_arr[BACK_RIGHT]);
    return (condition1 | condition2)  == 1;
}
void printSensorData(const char *sensor_type, LocationInfo location, SensorData *sensor_data)
{
    Serial.println("-----------------------------------------------------------");
    char sensor_name[50];
    snprintf(sensor_name, sizeof(sensor_name), "%s %d", sensor_type, location);
    Serial.println("SENSOR_TYPE: " + String(sensor_name));
    Serial.println("Obstacle Status: " + String(sensor_data->obstacle_status));
    Serial.println("-----------------------------------------------------------");

    if (sensor_data->obstacle_status == OBSTACLE_DETECTED)
    {
        Serial.println("\tDistance of obstacle (cm): " + String(sensor_data->obstacle_distance));
        obstacles_detected_arr[location] = 1;
    }
    else
    {
        obstacles_detected_arr[location] = 0;
    }
}

void readUltrasoundSensors(UltrasoundSensor *sensors, size_t num_sensors)
{
    for (size_t i = 0; i < num_sensors; ++i)
    {
        readSensor(&sensors[i], &sensors[i].data);
        printSensorData("ULTRASONIC", sensors[i].location, &sensors[i].data);
    }
}

void readIRSensors(IRSensor *sensors, size_t num_sensors)
{
    for (size_t i = 0; i < num_sensors; ++i)
    {
        readSensor(&sensors[i], &sensors[i].data);
        printSensorData("IR", sensors[i].location, &sensors[i].data);
    }
}

void recommendCarMove(UltrasoundSensor *ultrasound_sensors, IRSensor *ir_sensors, CarMovement *car_move)
{
    // First read ir sensors. If the car can move then we will move on to ultrasonic sensors
    readIRSensors(ir_sensors, ARRAYLEN(ir_sensors));
    if (!canCarMove())
    {
        car_move->axial = 0;
        car_move->transverse = 0;
        return;
    }
    readUltrasoundSensors(ultrasound_sensors, ARRAYLEN(ultrasound_sensors));
    // decide move based on data from ultrasound sensors and left right ir sensors

    // check fronts
    if ((obstacles_detected_arr[FRONT_LEFT] & obstacles_detected_arr[FRONT_RIGHT]) == 1)
    {
        // if whole forward area is empty
        car_move->axial = MOVE_FORWARD;
        car_move->transverse = NO_MOVE;
    }
    else if ((obstacles_detected_arr[FRONT_RIGHT] | obstacles_detected_arr[RIGHT]) == 1)
    {
        // if right area is empty
        car_move->axial = MOVE_FORWARD;
        car_move->transverse = MOVE_RIGHT;
    }

    else if ((obstacles_detected_arr[FRONT_LEFT] | obstacles_detected_arr[LEFT]) == 1)
    {
        // if left area is empty
        car_move->axial = MOVE_FORWARD;
        car_move->transverse = MOVE_LEFT;
    }
    // check back
    else if ((obstacles_detected_arr[BACK_LEFT] & obstacles_detected_arr[BACK_RIGHT]) == 1)
    {
        // if whole back area is empty
        car_move->axial = MOVE_FORWARD;
        car_move->transverse = MOVE_RIGHT;
    }
    else if ((obstacles_detected_arr[BACK_RIGHT] | obstacles_detected_arr[RIGHT]) == 1)
    {
        // if right area is empty
        car_move->axial = MOVE_BACKWARD;
        car_move->transverse = MOVE_RIGHT;
    }
    else if ((obstacles_detected_arr[FRONT_LEFT] | obstacles_detected_arr[LEFT]) == 1)
    {
        // if whole right area is empty
        car_move->axial = MOVE_BACKWARD;
        car_move->transverse = MOVE_LEFT;
    }
}

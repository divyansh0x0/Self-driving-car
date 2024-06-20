#include "Sensor.hpp"
#include <Arduino.h>
#define ARRAYLEN(arr) sizeof(arr) / sizeof(arr[0])
#define SOUND_SPEED_CM_PER_MICROS 0.034
#define MIN_DISTANCE_CM 20
#define MIN_DURATION MIN_DISTANCE_CM / SOUND_SPEED_CM_PER_MICROS * 2
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
    if(duration_micros == 0){//I believe it will never be zero except when timed out
        sensor_data->obstacle_status = NO_OBSTACLE;
        sensor_data->obstacle_distance = 0;
    }
    else{
        unsigned long distance_cm = duration_micros * SOUND_SPEED_CM_PER_MICROS / 2; // division by two because it is a round trip
        sensor_data->obstacle_distance = distance_cm;
        sensor_data->obstacle_status = OBSTACLE_DETECTED;
    }
}
void readSensor(IRSenor *ir_sensor, SensorData *sensor_data)
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
void setPreferedMovement(SensorData **sensor_data_arr, CarMovement *CarMovement)
{
    for (int i = ARRAYLEN(sensor_data_arr) - 1; i >= 0; i--)
    {
        SensorData *sensor_data = sensor_data_arr[i];
        Serial.println("-----------------------------------------------------------");
        String sensorName = (sensor_data->sensor_type == IR ? "IR" : "ULTRASONIC");
        Serial.println("SENSOR_TYPE:" + sensorName);
        Serial.println("Obstacle Status " + (sensor_data->obstacle_status));
        if(sensor_data->obstacle_status == OBSTACLE_DETECTED)
            Serial.println("\tDistance of obstacle (cm): " + (sensor_data->obstacle_distance));

    }
}
void recommendCarMove(UltraSoundSensor **ultrasound_sensors, IRSenor **ir_sensors, CarMovement *CarMovement)
{
    SensorData* sensor_data_arr[ARRAYLEN(ultrasound_sensors) + ARRAYLEN(ir_sensors)];
    int i = 0;
    while ( i < ARRAYLEN(ultrasound_sensors))
    {
        readSensor(ultrasound_sensors[i], &ultrasound_sensors[i]->data);
        ++i;
    }
    while ( i < ARRAYLEN(ultrasound_sensors))
    {
        readSensor(ultrasound_sensors[i], &ultrasound_sensors[i]->data);
        ++i;
    }
    setPreferedMovement(sensor_data_arr, CarMovement);
}

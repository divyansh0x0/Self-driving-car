#include "Sensor.hpp"
#include "Debug.hpp"
#define ARRAYLEN(arr) sizeof(&arr + 1) - arr
#define SOUND_SPEED_CM_PER_MICROS 0.034
#define MIN_DISTANCE_CM 10
// multiply by 2 because it is a round trip
#define MAX_DURATION MIN_DISTANCE_CM / SOUND_SPEED_CM_PER_MICROS * 2

bool obstacles_detected_arr[NUM_LOCATIONS] = {};
bool cantGoForward = false;
void resetObstaclesDetectedArray()
{
    for (int i = 0; i < NUM_LOCATIONS; i++)
    {
        obstacles_detected_arr[i] = false;
    }
}
void printSensorData(String sensor_type, LocationInfo location, SensorData *sensor_data)
{
    if (sensor_data->obstacle_status == NO_OBSTACLE)
    {
        return;
    }
    log("-----------------------------------------------------------");

    sensor_type.toUpperCase();
    log("SENSOR INFO " + sensor_type);
    log("\tLocation: " + getLocationName((location)));
    log("\tObstacle Status: " + String(sensor_data->obstacle_status == OBSTACLE_DETECTED ? "Obstacle detected" : "No obstacle"));

    if (sensor_data->obstacle_status == OBSTACLE_DETECTED)
    {
        Serial.println("\tDistance of obstacle (cm): " + String(sensor_data->obstacle_distance));
    }
    log("-----------------------------------------------------------");
}
void readSensor(UltrasonicSensor &ultrasonic_sensor, SensorData &sensor_data)
{
    uint8_t trig_pin = ultrasonic_sensor.trigger_pin;
    uint8_t echo_pin = ultrasonic_sensor.echo_pin;
    digitalWrite(trig_pin, LOW);

    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
    unsigned long duration_micros = pulseIn(echo_pin, HIGH);

    unsigned long distance_cm = duration_micros * SOUND_SPEED_CM_PER_MICROS / 2; // division by two because it is a round trip
    log("pin:(" + String(trig_pin) + ", " + echo_pin + ") || Duration: " + duration_micros + " us|| Distance: " + distance_cm + "cm");
    if (duration_micros >= 36000) // echo pin resets after 36 millis, it means no object was found
    {
        // Handle timeout or abnormal readings
        sensor_data.obstacle_status = NO_OBSTACLE;
        sensor_data.obstacle_distance = 0uL;
    }
    else
    {
        sensor_data.obstacle_distance = distance_cm;
        sensor_data.obstacle_status = OBSTACLE_DETECTED;
    }

    obstacles_detected_arr[ultrasonic_sensor.location] = sensor_data.obstacle_status == OBSTACLE_DETECTED;
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
    obstacles_detected_arr[ir_sensor->location] = sensor_data->obstacle_status == OBSTACLE_DETECTED;
}
void readUltrasonicSensors(UltrasonicSensor *sensors, size_t num_sensors)
{
    for (size_t i = 0; i < num_sensors; ++i)
    {
        // log(String(sensors[i].echo_pin));
        readSensor(sensors[i], sensors[i].data);
        // printSensorData("ULTRASONIC", sensors[i].location, &sensors[i].data);
    }
}

void readIRSensors(IRSensor *sensors, size_t num_sensors)
{
    // log("Num of irs: " + String(num_sensors));
    for (size_t i = 0; i < num_sensors; ++i)
    {
        // log(String(sensors[i].pin) + "|" + String(sensors[i].location));
        readSensor(&sensors[i], &sensors[i].data);
        // printSensorData("IR", sensors[i].location, &sensors[i].data);
    }
}
void resetIRSensors(IRSensor *IrSensors, size_t count)
{
    for (size_t i = 0; i < count; i++)
    {
        IrSensors->data.obstacle_distance = 0;
        IrSensors->data.obstacle_status = NO_OBSTACLE;
    }
}

void resetUltrasonicSensors(UltrasonicSensor *ultrasonic_sensors, size_t count)
{
    for (size_t i = 0; i < count; i++)
    {
        ultrasonic_sensors->data.obstacle_distance = 0;
        ultrasonic_sensors->data.obstacle_status = NO_OBSTACLE;
    }
}
bool canCarMove()
{
    return obstacles_detected_arr[LEFT] || obstacles_detected_arr[RIGHT] || obstacles_detected_arr[FRONT] || obstacles_detected_arr[BACK] != true;
}
void recommendCarMove(IRSensor ir_sensors[], size_t count_ir_sensors, UltrasonicSensor ultrasonic_sensors[], size_t count_ultrasonic_sensors, CarMovement *car_move, bool useIrOnly)
{
    if (count_ir_sensors == 0)
    {
        log("ZERO IR SENSORS FOUND!");
        return;
    }

    readIRSensors(ir_sensors, count_ir_sensors);
    // First read ir sensors. If the car can move then we will move on to ultrasonic sensors
    car_move->axial = 0;
    car_move->transverse = 0;
    if (canCarMove())
    {
        if (useIrOnly || count_ultrasonic_sensors == 0)
        {
            for (size_t i = 0; i < NUM_LOCATIONS; i++)
            {
                Serial.print(String(obstacles_detected_arr[i]) + ",");
            }
            Serial.println();
            log("--------------------------------------------------------");
            if (!obstacles_detected_arr[FRONT] && !cantGoForward)
            { // No obstacle in front
                car_move->axial = MOVE_FORWARD;
                if (!obstacles_detected_arr[RIGHT] && !obstacles_detected_arr[LEFT]) // No obstacle in left or right
                    car_move->transverse = NO_MOVE;
                else if (!obstacles_detected_arr[RIGHT]) // No obstacle in right
                    car_move->transverse = MOVE_RIGHT;
                else if (!obstacles_detected_arr[LEFT]) // No obstacle in left
                    car_move->transverse = MOVE_LEFT;
            }
            else
            {
                cantGoForward = true;
                // If obstacle in front then move back
                car_move->axial = MOVE_BACKWARD;
                if (!obstacles_detected_arr[LEFT]){ // If no obstacle in left, turn left
                    car_move->transverse = LEFT;
                    cantGoForward = false;
                }
                else if (!obstacles_detected_arr[RIGHT]){ // if No obstacle in right, turn right
                    car_move->transverse = RIGHT;
                    cantGoForward = false;
                
                }
                else //May break
                {
                    car_move->axial = NO_MOVE;
                    car_move->transverse = NO_MOVE;
                }
            }
            if (obstacles_detected_arr[FRONT] && obstacles_detected_arr[BACK])
                car_move->axial = NO_MOVE;
        }


        else
        { // Use IR + Ultrasound
            readUltrasonicSensors(ultrasonic_sensors, count_ultrasonic_sensors);
            // decide move based on data from ultrasound sensors and left right ir sensors
            // check fronts
            if (!(obstacles_detected_arr[FRONT_LEFT] && obstacles_detected_arr[FRONT_RIGHT]))
            {
                // if whole forward area is empty
                car_move->axial = MOVE_FORWARD;
                car_move->transverse = NO_MOVE;
            }
            else if (!(obstacles_detected_arr[FRONT_RIGHT] || obstacles_detected_arr[RIGHT]))
            {
                // if right area is empty
                car_move->axial = MOVE_FORWARD;
                car_move->transverse = MOVE_RIGHT;
            }

            else if (!(obstacles_detected_arr[FRONT_LEFT] || obstacles_detected_arr[LEFT]))
            {
                // if left area is empty
                car_move->axial = MOVE_FORWARD;
                car_move->transverse = MOVE_LEFT;
            }
            // check back
            else if (!(obstacles_detected_arr[BACK_LEFT] && obstacles_detected_arr[BACK_RIGHT]))
            {
                // if whole back area is empty
                car_move->axial = MOVE_BACKWARD;
                car_move->transverse = NO_MOVE;
            }
            else if (!(obstacles_detected_arr[BACK_RIGHT] || obstacles_detected_arr[RIGHT]))
            {
                // if back right area is empty
                car_move->axial = MOVE_BACKWARD;
                car_move->transverse = MOVE_RIGHT;
            }
            else if (!(obstacles_detected_arr[FRONT_LEFT] || obstacles_detected_arr[LEFT]))
            {
                // if whole back left area is empty
                car_move->axial = MOVE_BACKWARD;
                car_move->transverse = MOVE_LEFT;
            }
        }
    }

    resetIRSensors(ir_sensors, count_ir_sensors);
    resetUltrasonicSensors(ultrasonic_sensors, count_ultrasonic_sensors);
}
String getLocationName(LocationInfo location)
{
    switch (location)
    {
    case 0:
        return "FRONT";
    case 1:
        return "FRONT RIGHT";
    case 2:
        return "FRONT LEFT";
    case 3:
        return "BACK";
    case 4:
        return "BACK RIGHT";
    case 5:
        return "BACK LEFT";
    case 6:
        return "LEFT";

    case 7:
        return "RIGHT";
    default:
        return "UNKNOWN";
    }
}

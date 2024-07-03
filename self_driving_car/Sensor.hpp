#include<inttypes.h>
#define NUM_LOCATIONS 8 //Total possible locations of sensors
#define MOVE_RIGHT 1
#define MOVE_LEFT -1
#define MOVE_FORWARD 1
#define MOVE_BACKWARD -1
#define NO_MOVE 0
enum LocationInfo{
    FRONT = 0,
    FRONT_RIGHT = 1,
    FRONT_LEFT=2,
    BACK=3,
    BACK_RIGHT=4,
    BACK_LEFT=5,
    LEFT=6,
    RIGHT=7
};
struct CarMovement{
    short axial = 0;
    short transverse = 0;
};
enum SensorType{
    ULTRASONIC,
    IR
};
enum ObstacleStatus{
    OBSTACLE_DETECTED,
    NO_OBSTACLE
};
struct SensorData{
    SensorType sensor_type;
    unsigned long obstacle_distance;
    ObstacleStatus obstacle_status;
};

struct UltrasoundSensor{
    uint8_t trigger_pin;
    uint8_t echo_pin;
    LocationInfo location;
    SensorData data {ULTRASONIC,0,NO_OBSTACLE};
};
struct IRSensor{
    uint8_t pin;
    LocationInfo location;
    SensorData data{IR,0,NO_OBSTACLE};
};

void recommendCarMove(UltrasoundSensor* ultrasound_sensors, IRSensor* ir_sensors, CarMovement* car_move);
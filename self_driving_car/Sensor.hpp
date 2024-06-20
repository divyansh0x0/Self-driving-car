enum LocationInfo{
    FRONT,
    FRONT_RIGHT,
    FRONT_LEFT,
    BACK,
    BACK_RIGHT,
    BACK_LEFT,
    LEFT,
    RIGHT
};
enum CarMovement{
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
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

struct UltraSoundSensor{
    uint8_t triggerPin;
    uint8_t echoPin;
    LocationInfo location;
    SensorData data {ULTRASONIC,0,NO_OBSTACLE};
};
struct IRSenor{
    uint8_t pin;
    LocationInfo location;
    SensorData data{IR,0,NO_OBSTACLE};
};



void recommendCarMove(UltraSoundSensor** ultrasound_sensors, IRSenor** ir_sensors, CarMovement* CarMovement);
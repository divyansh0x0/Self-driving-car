#include "Debug.hpp"
bool debugAllowed = false;
void setDebugMode(bool allowDebugging){
    debugAllowed= allowDebugging;
}
void log(StringSumHelper sum){
    if (debugAllowed){
        Serial.println(sum);
    }
}
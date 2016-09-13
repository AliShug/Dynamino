// This is extremely liable to change:
// NONE OF THIS WORKS YET


#include<Dynamino.h>
using namespace Dynamino;

AXServo servo();

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        // Wait for serial to connect
    }

    // Find servos attached on pin 2
    int id = AXServo::FindNextServo(2);
    if (id >= 0) {
        if (servo.init(2, id)) {
            Serial.print("Connected to servo with ID ");
            Serial.println(servo.getID());
        }
        else {
            Serial.println("Unable to connect");
        }
    }
    else {
        Serial.println("No servos found :(");
    }
}

void loop() {}

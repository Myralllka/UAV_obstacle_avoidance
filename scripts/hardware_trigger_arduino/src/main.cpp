#include <Arduino.h>

void setup() {
    pinMode(PIND2, OUTPUT);
}

void loop() {
    // 10 ms with 8e10^6 f_cpu for ArduinoNano gives 20ms -> 50fps
    digitalWrite(PIND2, HIGH);
    delay(10);
    digitalWrite(PIND2, LOW);
    delay(10);
}
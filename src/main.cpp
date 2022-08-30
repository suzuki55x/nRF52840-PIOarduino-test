#include <Arduino.h>

#define PIN PIN_LED1

void setup() {
    pinMode( PIN, OUTPUT );
}
void loop() {
    digitalWrite( PIN, HIGH );
    delay(1000);
    digitalWrite( PIN, LOW );
    delay(3000);
}
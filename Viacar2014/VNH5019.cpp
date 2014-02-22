#include "./VNH5019.h"
#include <WProgram.h>
#include <cmath>


Motor::Motor(int fwdPin, int backPin, int speedPin) : fwdPin(fwdPin), backPin(backPin), speedPin(speedPin)
{
	pinMode(fwdPin, OUTPUT);
	pinMode(backPin, OUTPUT);
    pinMode(speedPin, OUTPUT);
    
    // Set up PWM
    analogWriteFrequency(speedPin, 20000);
	analogWriteResolution(16);
    analogWrite(speedPin, 0);
}


void Motor::write(float value)
{
    analogWrite(speedPin, 65535 * fabs(value));
	digitalWrite(fwdPin, value > 0.f ? HIGH : LOW);
	digitalWrite(backPin, value < 0.f ? HIGH : LOW);
}


Motor& Motor::operator=(float value)
{
    write(value);
    return *this;
}


void Motor::brake(float value)
{
	digitalWrite(fwdPin, LOW);
	digitalWrite(backPin, LOW);
    analogWrite(speedPin, 65535 * fabs(value));
}

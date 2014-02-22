#include "Servo.h"
#include <WProgram.h>


Servo::Servo(int pin, bool start)
{
    pwmPin = pin;
    enabled = start;
    
    // Set up pwm
    pinMode(pwmPin, OUTPUT);
    analogWriteFrequency(pwmPin, 50);
	analogWriteResolution(16);
    
    // Set default calibration
    calibrate(1940, 1200, 60.f, -45.f);
    
    // Start at center position
    pulseWidth = center;
    analogWrite(pwmPin, 65535 * (enabled ? pulseWidth/20000.0f : 0.0f));
}


bool Servo::calibrate(int plus45, int minus45, float upperLimit, float lowerLimit)
{
    // Check if given parameters are valid
    if (upperLimit > lowerLimit && plus45 >= 0 && minus45 >= 0)
    {
        center = (plus45 + minus45) / 2;
        usPerDegree = (plus45 - center) / 45.0f;
        this->upperLimit = upperLimit;
        this->lowerLimit = lowerLimit;   
        return true;
    }
    else
    {
        return false;
    }
}


void Servo::write(float degrees)
{
    // Limit to the valid angle range
    degrees = (degrees > upperLimit ? upperLimit : (degrees < lowerLimit ? lowerLimit : degrees));
    
    pulseWidth = center + (int)(degrees * usPerDegree);
    analogWrite(pwmPin, 65535 * (enabled ? pulseWidth/20000.0f : 0.0f));
}


float Servo::read()
{
    return ((int)pulseWidth - (int)center) / usPerDegree;
}


void Servo::disable()
{
    enabled = false;
    analogWrite(pwmPin, 0);
}


void Servo::enable()
{
    enabled = true;
    analogWrite(pwmPin, 65535 * (pulseWidth/20000.0f));
}


void Servo::operator=(float degrees)
{
    write(degrees);
}


Servo::operator float()
{
    return read();
}

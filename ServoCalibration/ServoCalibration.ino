#include <WProgram.h>
#include "./RadioTerminal.h"
#include "./Servo.h"
#include <SPI.h>
#include <cstdio>


Servo steering(3);


CmdHandler* setwidth(const char* input)
{
    char output[256];
    
    int width;
    sscanf(input, "w %d", &width);
    steering.writeMicros(width);
    sprintf(output, "%d us, %f degrees", width, steering.read());
    RadioTerminal::write(output);
    
    return NULL;
}


void setup()
{
    RadioTerminal::initialize(8, 9, 10);
    RadioTerminal::reset();
    
    RadioTerminal::addCommand("w", &setwidth);
    
    steering.calibrate(1200, 1800, 60.f, -60.f);
    
    Serial.begin(115200);
}


void loop()
{
}

#include <WProgram.h>
#include "./VNH5019.h"
#include "./RadioTerminal.h"
#include "./Servo.h"
#include "./ControlLoop.h"
#include "./LowPass.h"
#include <IntervalTimer.h>
#include <SPI.h>
#include <cstdio>
#include <cmath>
#include <functional>


unsigned int loopFreq = 100;
unsigned int loopPeriodUs = 1000000 / loopFreq;
const float dt = loopPeriodUs / 1000000.f;
unsigned int lastMicros = 0;
LowPass loadPercent;

Motor motor(7, 6, 5);
Servo steering(3);

float kp, ki, kd;
float v1, v2, x1, x2;
float C5, C6, d;

ControlLoop steerLoop(dt);
LowPass error;
LowPass control;
float speed = 0.20f;
float throttle;
float turn;


class WatchHandler : public CmdHandler
{
public:
    WatchHandler(std::function<float ()> watchFun);
    virtual void sendChar(char c) { timer.end(); RadioTerminal::terminateCmd(); }
    static std::function<float ()> watch;
    IntervalTimer timer;
    static void refresh();
};

std::function<float ()> WatchHandler::watch;


CmdHandler* watch(const char* input)
{
    // Read command parameters
    if (!strncmp(input, "w v1", 8))
    {
        return new WatchHandler([&]{ return v1; });
    }
    else if (!strncmp(input, "w v2", 8))
    {
        return new WatchHandler([&]{ return v2; });
    }
    else if (!strncmp(input, "w x1", 8))
    {
        return new WatchHandler([&]{ return x1; });
    }
    else if (!strncmp(input, "w x2", 8))
    {
        return new WatchHandler([&]{ return x2; });
    }
    else if (!strncmp(input, "w error", 8))
    {
        return new WatchHandler([&]{ return float(error); });
    }
    else if (!strncmp(input, "w load", 8))
    {
        return new WatchHandler([&]{ return float(loadPercent); });
    }
    else
    {
        RadioTerminal::write("error reading input parameters");
        return NULL;
    }
}


WatchHandler::WatchHandler(std::function<float ()> watchFun)
{
    watch = watchFun;
    timer.begin(&WatchHandler::refresh, 0.2f);
}


void WatchHandler::refresh()
{
    char output[256];

    sprintf(output, "\r         \r\r\r%4.4f", watch());
    RadioTerminal::write(output);
}


CmdHandler* setkp(const char* input)
{
    char output[256];
    
    float kp;
    sscanf(input, "kp %f", &kp);
    steerLoop.setKp(kp);
    sprintf(output, "kp = %f", kp);
    RadioTerminal::write(output);
    
    return NULL;
}


CmdHandler* setki(const char* input)
{
    char output[256];
    
    float ki;
    sscanf(input, "ki %f", &ki);
    steerLoop.setKi(ki);
    sprintf(output, "ki = %f", ki);
    RadioTerminal::write(output);
    
    return NULL;
}


CmdHandler* setkd(const char* input)
{
    char output[256];
    
    float kd;
    sscanf(input, "kd %f", &kd);
    steerLoop.setKd(kd);
    sprintf(output, "kd = %f", kd);
    RadioTerminal::write(output);
    
    return NULL;
}


CmdHandler* setC5(const char* input)
{
    char output[256];
    
    sscanf(input, "c5 %f", &C5);
    sprintf(output, "c5 = %f", C5);
    RadioTerminal::write(output);
    
    return NULL;
}


CmdHandler* setC6(const char* input)
{
    char output[256];
    
    sscanf(input, "c6 %f", &C6);
    sprintf(output, "c6 = %f", C6);
    RadioTerminal::write(output);
    
    return NULL;
}


CmdHandler* setd(const char* input)
{
    char output[256];
    
    sscanf(input, "d %f", &d);
    sprintf(output, "d = %f", d);
    RadioTerminal::write(output); 
    
    return NULL;
}


CmdHandler* setspeed(const char* input)
{
    char output[256];
    
    sscanf(input, "speed %f", &speed);
    sprintf(output, "speed = %f", speed);
    RadioTerminal::write(output); 
    
    return NULL;
}


float getError(float predicted)
{
    v1 = analogRead(A0)/4096.f;
    v2 = analogRead(A1)/4096.f;
    
    x1 = volt2dist(v1);
    x2 = volt2dist(v2);
    
    float daa = abs(d + x1 - x2);
    float dab = abs(d + x1 + x2);
    float dba = abs(d - x1 - x2);
    float dbb = abs(d - x1 + x2);
    
    float aa = (x1 + x2) / 2.0f;
    float ab = (x1 - x2) / 2.0f;
    float ba = (x2 - x1) / 2.0f;
    float bb = (x1 + x2) / -2.0f;
    
    float eaa = abs(abs(aa) - predicted);
    float eab = abs(abs(ab) - predicted);
    float eba = abs(abs(ba) - predicted);
    float ebb = abs(abs(bb) - predicted);
    
    float taa = daa + eaa;
    float tab = dab + eab;
    float tba = dba + eba;
    float tbb = dbb + ebb;
    
    float m = taa;
    float res = aa;
    
    if (m > tab)
    {
        m = tab;
        res = ab;
    }
    if (m > tba)
    {
        m = tba;
        res = ba;
    }
    if (m > tbb)
    {
        m = tbb;
        res = bb;
    }
    
    return res;
}


float volt2dist(float v)
{
    const float C1 = 0.546f;
    const float C2 = 8.16f;
    const float C3 = 0.00330f;
    const float C4 = 2.43f;
    // C5 and C6 are variable, defined globally

    float eout = (v - C1) * C2;
    float vin = C3 * exp(-eout / C4);
    float xsq = C5 / vin - C6;
    return sqrt(xsq > 0.f ? xsq : 0.f);
}


int deadzone(int input, int zone)
{
    return (input > zone || input < -zone) ? input : 0;
}


void setup()
{
    RadioTerminal::initialize(8, 9, 10);
    RadioTerminal::reset();
    
    RadioTerminal::addCommand("kp", &setkp);
    RadioTerminal::addCommand("ki", &setki);
    RadioTerminal::addCommand("kd", &setkd);
    RadioTerminal::addCommand("c5", &setC5);
    RadioTerminal::addCommand("c6", &setC6);
    RadioTerminal::addCommand("d", &setd);
    RadioTerminal::addCommand("speed", &setspeed);
    RadioTerminal::addCommand("w", &watch);
    
    analogReadRes(12);
    analogReference(INTERNAL);
    analogReadAveraging(16);
    
    steering.calibrate(1940, 1200, 60.f, -45.f);
    
    Serial.begin(115200);
    
    C5 = 8.f; // Get this from testing
    C6 = 64.f; // h^2 (cm)
    d = 10.f;
    
    steerLoop.setTuning(0.5f, 0.0f, 0.05f);
    steerLoop.setOutputLimits(-45.f, 45.f);
    
    error.setCutoffFreq(10.f, dt);
    control.setCutoffFreq(1.f, dt);
    loadPercent.setFilterConst(0.9f);
    
}


void loop()
{
    error = getError(error);
    control = steerLoop.update(error);

    // Use manual steering if a controller message is present
    if (RadioTerminal::rx_controller != 0)
    {
        turn = 60.f * 0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>16)&0xff), 8); // Convert to +/-1.0f range
        throttle = 0.5f * -0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>8)&0xff), 8);
    }
    else
    {
        // Use PID control if no controller is detected
        turn = control;
        throttle = speed;
    }
    
    steering = turn;
    motor = throttle;
    
    loadPercent = micros() / loopPeriodUs;
    // Limit loop speed to a consistent value to make timing and integration simpler
    while (micros() - lastMicros < loopPeriodUs);
    lastMicros = micros();
}

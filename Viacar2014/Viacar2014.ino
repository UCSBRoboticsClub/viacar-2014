#include <WProgram.h>
#include "./VNH5019.h"
#include "./RadioTerminal.h"
#include "./Servo.h"
#include "./ControlLoop.h"
#include "./LowPass.h"
#include "./Button.h"
#include "./RingBuffer.h"
#include <IntervalTimer.h>
#include <SPI.h>
#include <cstdio>
#include <cmath>


const unsigned int loopFreq = 1000;
const unsigned int loopPeriodUs = 1000000 / loopFreq;
const float dt = 1.f / loopFreq;
unsigned int lastMicros = 0;
LowPass loadPercent;

Motor motor(7, 6, 5);
Servo steering(3);

float v1, v2, x1, x2;
const float C1 = 0.631f;
const float C2 = 6.8f;
const float C3 = 0.00330f;
const float C4 = 2.43f;
float h, d, vinmax, errorF, loadPercentF, loadMax;
unsigned int loadMaxMillis;

ControlLoop steerLoop(dt);
LowPass error;
LowPass control;
float speed;
float throttle;
float turn;

Button calSwitch(0, LOW, true);

struct DataPoint
{
    float error;
    float control;
};

RingBuffer<DataPoint, 1000> dataLog;
unsigned int cycleCount = 0;


class WatchHandler : public CmdHandler
{
public:
    WatchHandler(float* w);
    virtual void sendChar(char c) { timer.end(); RadioTerminal::terminateCmd(); }
    static float* watch;
    IntervalTimer timer;
    static void refresh();
};

float* WatchHandler::watch;


CmdHandler* watch(const char* input)
{
    // Read command parameters
    if (!strncmp(input, "w v1", 8))
    {
        return new WatchHandler(&v1);
    }
    else if (!strncmp(input, "w v2", 8))
    {
        return new WatchHandler(&v2);
    }
    else if (!strncmp(input, "w x1", 8))
    {
        return new WatchHandler(&x1);
    }
    else if (!strncmp(input, "w x2", 8))
    {
        return new WatchHandler(&x2);
    }
    else if (!strncmp(input, "w error", 8))
    {
        return new WatchHandler(&errorF);
    }
    else if (!strncmp(input, "w loadmax", 8))
    {
        return new WatchHandler(&loadMax);
    }
    else if (!strncmp(input, "w load", 8))
    {
        return new WatchHandler(&loadPercentF);
    }
    else if (!strncmp(input, "w vinmax", 8))
    {
        return new WatchHandler(&vinmax);
    }
    else
    {
        RadioTerminal::write("error reading input parameters");
        return NULL;
    }
}


WatchHandler::WatchHandler(float* w)
{
    watch = w;
    timer.begin(&WatchHandler::refresh, 200000);
}


void WatchHandler::refresh()
{
    char output[256];

    sprintf(output, "\r         \r\r\r%4.4f", *watch);
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


CmdHandler* seth(const char* input)
{
    char output[256];
    
    sscanf(input, "h %f", &h);
    sprintf(output, "h = %f", h);
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


CmdHandler* dumplogm(const char* input)
{
    char buffer[256];
    
    RadioTerminal::write("[");
    
    for (int i = dataLog.size(); i > 0; --i)
    {
        snprintf(buffer, 256, "%e,%e;\n",
                 dataLog[i].error,
                 dataLog[i].control);
        RadioTerminal::write(buffer); 
    }
    
    snprintf(buffer, 256, "%e,%e];",
             dataLog[0].error,
             dataLog[0].control);
    RadioTerminal::write(buffer); 
    
    return NULL;
}


CmdHandler* dumplog(const char* input)
{
    char buffer[256];
    
    for (int i = dataLog.size(); i >= 0; --i)
    {
        snprintf(buffer, 256, "%e,%e\n",
                 dataLog[i].error,
                 dataLog[i].control);
        RadioTerminal::write(buffer); 
    }
    
    return NULL;
}


float getError(float predicted)
{
    v1 = analogRead(A0)/4096.f;
    v2 = analogRead(A1)/4096.f;
    
    x1 = volt2dist(v1);
    x2 = volt2dist(v2);
    
    const float maxDist = 30.f;
    if (x1 > maxDist)
        x1 = maxDist;
    if (x2 > maxDist)
        x2 = maxDist;
    
    // Calculate the 4 possible locations by averaging distances
    float aa = (x1 + x2) / 2.0f;
    float ab = (x1 - x2) / 2.0f;
    float ba = (x2 - x1) / 2.0f;
    float bb = (x1 + x2) / -2.0f;
    
    // Calculate scores based on discrepancy between sensors
    float daa = fabs(d + x1 - x2);
    float dab = fabs(d + x1 + x2);
    float dba = fabs(d - x1 - x2);
    float dbb = fabs(d - x1 + x2);
    
    // Calculate scores based on deviation from predicted position
    float eaa = fabs(abs(aa) - fabs(predicted));
    float eab = fabs(abs(ab) - fabs(predicted));
    float eba = fabs(abs(ba) - fabs(predicted));
    float ebb = fabs(abs(bb) - fabs(predicted));
    
    // Add both scores (no weighting yet)
    float taa = daa + eaa;
    float tab = dab + eab;
    float tba = dba + eba;
    float tbb = dbb + ebb;
    
    // If not close to the line, weight heavily against swapping sides of the line
    if (fabs(predicted) > d || x1 + x2 > d*2.f)
    {
        const float weight = 1000000.f;
    
        if (predicted > 0.f != aa > 0.f)
            taa += weight;
        if (predicted > 0.f != ab > 0.f)
            tab += weight;
        if (predicted > 0.f != ba > 0.f)
            tba += weight;
        if (predicted > 0.f != bb > 0.f)
            tbb += weight;
    }
    
    // Choose location with the lowest score
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
    float eout = (C1 - v) * C2;
    float vin = C3 * exp(-eout * C4);
    
    if (calSwitch.pressed() && vin > vinmax)
        vinmax = vin;
        
    float xsq = vinmax / vin - 1.f;
    return h * sqrt(xsq > 0.f ? xsq : 0.f);
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
    RadioTerminal::addCommand("h", &seth);
    RadioTerminal::addCommand("d", &setd);
    RadioTerminal::addCommand("speed", &setspeed);
    RadioTerminal::addCommand("w", &watch);
    RadioTerminal::addCommand("logm", &dumplogm);
    RadioTerminal::addCommand("log", &dumplog);
    
    analogReadRes(12);
    analogReference(INTERNAL);
    analogReadAveraging(16);
    
    steering.calibrate(1200, 1800, 60.f, -60.f);
    
    Serial.begin(115200);
    
    h = 8.f;
    d = 10.f;
    vinmax = 0.045f;
    speed = 0.3f;
    
    steerLoop.setTuning(30.f, 0.0f, 20.f);
    steerLoop.setOutputLimits(-50.f, 50.f);
    steerLoop.setDerivCutoffFreq(50.f);
    
    error.setCutoffFreq(50.f, dt);
    control.setCutoffFreq(1.f, dt);
    loadPercent.setFilterConst(0.9f);
}


void loop()
{
    calSwitch.update();
    if (calSwitch.pressEdge())
        vinmax = 0.f;
        
    error.push(getError(error));
    control.push(steerLoop.update(error));

    // Use manual steering if a controller message is present
    if (RadioTerminal::rx_controller != 0)
    {
        turn = 60.f * 0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>16)&0xff), 8); // Convert to +/-1.0f range
        throttle = 0.7f * -0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>8)&0xff), 8);
    }
    else
    {
        // Use PID control if no controller is detected
        turn = control;
        throttle = speed;
    }
    
    steering = turn;
    motor = throttle;
    
    // Store log data
    if (cycleCount % (loopFreq / 10) == 0)
        dataLog.push({float(error), float(control)});
    
    // Calculate processor load information
    float currentLoadPercent = float(micros() - lastMicros) / loopPeriodUs;
    loadPercent.push(currentLoadPercent);
    loadPercentF = loadPercent;
    errorF = error;
    if (loadMax < currentLoadPercent)
    {
        loadMax = currentLoadPercent;
        loadMaxMillis = millis();
    }
    if (loadMaxMillis + 1000 < millis())
        loadMax = currentLoadPercent;
        
    ++cycleCount;
        
    // Limit loop speed to a consistent value to make timing and integration simpler
    while (micros() - lastMicros < loopPeriodUs);
    lastMicros = micros();
}

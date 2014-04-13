#include <WProgram.h>
#include "Button.h"


Button::Button(int pin, bool activeState, bool pullup) :
    pin(pin),
    activeState(activeState),
    edge(false)
{
    pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
    state = digitalRead(pin);
}


bool Button::setPullup(bool on)
{
    pinMode(pin, on ? INPUT_PULLUP : INPUT);
}


void Button::update()
{
    bool newState = digitalRead(pin);
    edge = (newState != state);
    state = newState;
}


bool Button::pressed()
{
    return (state == activeState);
}


bool Button::pressEdge()
{
    return (edge && state == activeState);
}


bool Button::releaseEdge()
{
    return (edge && state != activeState);
}

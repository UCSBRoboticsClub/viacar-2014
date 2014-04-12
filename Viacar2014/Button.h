#ifndef BUTTON_H
#define BUTTON_H


class Button
{
public:
    Button(int pin, bool activeState = true);
    void init();
    bool setPullup(bool on);
    void update();
    bool pressed();
    bool pressEdge();
    bool releaseEdge();
    
private:
    int pin;
    bool activeState;
    bool state;
    bool edge;
};

#endif // BUTTON_H
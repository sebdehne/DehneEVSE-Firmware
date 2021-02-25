
#ifndef CONTACTOR_H
#define CONTACTOR_H

class Contactor
{

private:
    /* data */
public:
    Contactor();
    void setup();

    void switchOff();
    void switchOn();
    bool isOn();
};

#endif
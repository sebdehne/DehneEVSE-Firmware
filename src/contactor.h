
#ifndef CONTACTOR_H
#define CONTACTOR_H

class ContactorClass
{

private:
public:
    ContactorClass();
    void setup();

    void switchOff();
    void switchOn();
    bool isOn();
};

extern ContactorClass Contactor;

#endif
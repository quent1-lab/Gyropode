#ifndef __MELODIE_H
#define __MELODIE_H

#include <Arduino.h>

class Melodie
{
public:
    Melodie(int pinBuzzer);
    void jouerMelodie();
    void stopperMelodie();

private:
    int pinBuzzer;
};

#endif
#ifndef __MELODIE_H
#define __MELODIE_H

#include <Arduino.h>

class Melodie
{
public:
    Melodie(int pinBuzzer);
    void jouerMelodie(int melodie[], int duree[], int tailleMelodie);
    void stopperMelodie();
    void choisirMelodie(int nb_melodie);

private:
    int pinBuzzer;
};

#endif
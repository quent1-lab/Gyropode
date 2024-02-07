#include "melodie.h"
#include "melodie_Mario_Bros.h"
#include "melodie_StarWars.h"

Melodie::Melodie(int pinBuzzer)
{
    this->pinBuzzer = pinBuzzer;
    pinMode(pinBuzzer, OUTPUT);
}

void Melodie::jouerMelodie(int melodie[], int duree[], int tailleMelodie)
{
    for (int i = 0; i < tailleMelodie; i++)
    {
        int noteDuree = 1000 / duree[i];
        tone(pinBuzzer, melodie[i], noteDuree);
        int pause = noteDuree * 1.30;
        delay(pause);
        noTone(pinBuzzer);
    }
}

void Melodie::stopperMelodie()
{
    noTone(pinBuzzer);
}

void Melodie::choisirMelodie(int nb_melodie)
{
    switch (nb_melodie)
    {
    case 1:
        jouerMelodie(melodie_Mario_Bros, duree_Mario_Bros, taille_Mario_Bros);
        break;
    case 2:
        jouerMelodie(melodie_StarWars, duree_StarWars, taille_StarWars);
        break;
    default:
        break;
    }
}
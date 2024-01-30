#ifndef ControleMoteur_h
#define ControleMoteur_h

#include <Arduino.h>

class ControleMoteur
{
public:
    ControleMoteur(int in1, int in2, int in3, int in4);
    void setVitesses(int vitesseMoteur1, int vitesseMoteur2);
    void updateMoteurs();

private:
    int IN1, IN2, IN3, IN4;
    int frequence;
    int resoltion;
    int vitesseMoteur1, vitesseMoteur2;
};

#endif

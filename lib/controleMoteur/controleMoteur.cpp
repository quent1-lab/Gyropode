#include "controleMoteur.h"

ControleMoteur::ControleMoteur(int in1, int in2, int in3, int in4)
{
    IN1 = in1;
    IN2 = in2;
    IN3 = in3;
    IN4 = in4;

    frequence = 20000;
    resoltion = 8;

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcSetup(1, frequence, resoltion); // Canal 1
    ledcSetup(2, frequence, resoltion); // Canal 2
    ledcSetup(3, frequence, resoltion); // Canal 3
    ledcSetup(4, frequence, resoltion); // Canal 4

    ledcAttachPin(IN1, 1); // Attache le canal 1 à la broche IN1
    ledcAttachPin(IN2, 2); // Attache le canal 2 à la broche IN2
    ledcAttachPin(IN3, 3); // Attache le canal 3 à la broche IN3
    ledcAttachPin(IN4, 4); // Attache le canal 4 à la broche IN4
}

void ControleMoteur::setVitesses(int vitesseMoteur1, int vitesseMoteur2)
{
    this->vitesseMoteur1 = constrain(vitesseMoteur1, -255, 255);
    this->vitesseMoteur2 = constrain(vitesseMoteur2, -255, 255);
}

void ControleMoteur::updateMoteurs()
{
    int pwm1 = map(abs(vitesseMoteur1), 0, 255, 0, 255);
    int pwm2 = map(abs(vitesseMoteur2), 0, 255, 0, 255);

   /*ledcWrite(1, 250);
    ledcWrite(2, 5);
    ledcWrite(3, 250);
    ledcWrite(4, 5);*/
    //Commande unipolaire des moteurs en PWM
    ledcWrite(1, -vitesseMoteur1 > 0 ? pwm1 : 255-pwm1);
    ledcWrite(2, -vitesseMoteur1 < 0 ? pwm1 : 255-pwm1);
    ledcWrite(3, vitesseMoteur2 > 0 ? pwm2 : 255-pwm2);
    ledcWrite(4, vitesseMoteur2 < 0 ? pwm2 : 255-pwm2);
}

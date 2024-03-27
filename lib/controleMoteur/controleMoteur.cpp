#include "controleMoteur.h"

ControleMoteur::ControleMoteur(int in1, int in2, int in3, int in4)
{
    IN1 = in1;
    IN2 = in2;
    IN3 = in3;
    IN4 = in4;

    frequence = 20000;
    resoltion = 8;
    alphaFrottement = 0.1;
    dir = 0;

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcSetup(2, frequence, resoltion); // Canal 1
    ledcSetup(3, frequence, resoltion); // Canal 2
    ledcSetup(4, frequence, resoltion); // Canal 3
    ledcSetup(5, frequence, resoltion); // Canal 4

    ledcAttachPin(IN1, 2); // Attache le canal 1 à la broche IN1
    ledcAttachPin(IN2, 3); // Attache le canal 2 à la broche IN2
    ledcAttachPin(IN3, 4); // Attache le canal 3 à la broche IN3
    ledcAttachPin(IN4, 5); // Attache le canal 4 à la broche IN4
}

void ControleMoteur::setVitesses(int vitesseMoteur1, int vitesseMoteur2)
{
    // La vitesse des moteurs est comprise entre -100 et 100 (pourcentage de la vitesse maximale)
    this->vitesseMoteur1 = constrain(vitesseMoteur1, -100, 100);
    this->vitesseMoteur2 = constrain(vitesseMoteur2, -100, 100);
}

void ControleMoteur::updateMoteurs()
{
    // Conversion de la vitesse des moteurs en commande unipolaire
    vitesseMoteur1 = map(vitesseMoteur1, -100, 100, -45, 45);
    vitesseMoteur2 = map(vitesseMoteur2, -100, 100, -45, 45);

    // Correction de la vitesse des moteurs en fonction du frottement
    float vitMotD_corrige = vitesseMoteur1 / 100.0 * (1.0 + alphaFrottement);
    float vitMotG_corrige = vitesseMoteur2 / 100.0 * (1.0 + alphaFrottement);

    float pwmD1 = 255 * (0.5 - vitMotD_corrige + dir);
    float pwmD2 = 255 * (0.5 + vitMotD_corrige - dir);
    float pwmG1 = 255 * (0.5 + vitMotG_corrige - dir);
    float pwmG2 = 255 * (0.5 - vitMotG_corrige + dir);

    Serial.printf("pwmD1: %f, pwmD2: %f, pwmG1: %f, pwmG2: %f\n", pwmD1, pwmD2, pwmG1, pwmG2);

    //Commande unipolaire des moteurs en PWM
    ledcWrite(2, pwmD1);
    ledcWrite(3, pwmD2);
    ledcWrite(4, pwmG1);
    ledcWrite(5, pwmG2);
}

void ControleMoteur::setAlphaFrottement(float alphaFrottement)
{
    this->alphaFrottement = constrain(alphaFrottement, 0, 1);
}

void ControleMoteur::setDir(float dir)
{
    this->dir = constrain(dir, -127, 127);
}

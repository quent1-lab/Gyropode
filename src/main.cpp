#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <controleMoteur.h>

#define MAX_COMMANDE 100 // Valeur maximale de la commande moteur

// ----------------------- Déclaration des variables des moteurs ---------------------

ControleMoteur moteurs(27, 25, 33, 32); // Remplacez les numéros de broches selon votre configuration
int vitesseMoteurG = 0;
int vitesseMoteurD = 0;

// ------------------------- Déclaration des variables du mpu ------------------------

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
bool mpu_ok = true;

char FlagCalcul = 0;
float Te = 10;   // période d'échantillonage en ms
float Tau = 250; // constante de temps du filtre en ms

// coefficient du filtre
float A, B;

// angle projeté et gyro
float thetaG, thetaR;

// angle filtre
float thetaGF, thetaRF, thetaFC;

// ----------------------- Déclaration des variables PID -----------------------

// Constantes du régulateur PID
const float kp = 1.0;  // Gain proportionnel
const float ki = 0.1;  // Gain intégral
const float kd = 0.01; // Gain dérivé

// Variables globales pour le PID
float erreur_cumulee = 0.0;
float erreur_precedente = 0.0;
float commande = 0.0;
float erreur = 0.0;

// ----------------------- Déclaration des fonctions -----------------------

// --------------------- Fonction de calcul des angles ---------------------

void controle(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    if (mpu_ok)
    {
      // Acquisition
      mpu.getEvent(&a, &g, &temp);

      // Calcul des angles
      thetaG = atan2(a.acceleration.y, a.acceleration.x); // Angle projeté
      thetaR = -g.gyro.z * Tau / 1000;

      // Appliquer le filtre passe-bas
      thetaGF = A * (thetaG + B * thetaGF);
      // Appliquer le filtre passe-haut sur l'angle gyro
      thetaRF = A * (thetaR + B * thetaRF);

      // Filtre complémentaire
      thetaFC = thetaGF + thetaRF;
    }

    moteurs.updateMoteurs();
    FlagCalcul = 1;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    mpu_ok = false;
    delay(10);
  }
  else
  {
    Serial.println("MPU6050 Found!");
  }

  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      10,         // tres haut niveau de priorite
      NULL        // descripteur
  );

  // calcul coeff filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te;
}

void loop()
{
  if (FlagCalcul == 1)
  {
    // Serial.printf("%5.1lf %3.1lf \n",Tau,Te);

    FlagCalcul = 0;
  }

  Serial.println("Test moteurs marche avant");
  moteurs.setVitesses(150, -100);
  moteurs.updateMoteurs();

  delay(1000);

  moteurs.setVitesses(0, 0);
  moteurs.updateMoteurs();

  delay(500);
  Serial.println("Test moteurs marche arrière");
  moteurs.setVitesses(-150, 100);
  moteurs.updateMoteurs();

  delay(1000);

  moteurs.setVitesses(0, 0);
  moteurs.updateMoteurs();
}

void debout()
{
  // Cette fonction permet de mettre le robot debout
  // Grâce à un asservissement en position

  asservissementPosition(0, thetaFC);

}

// Fonction de régulation PID en position
void asservissementPosition(float consigne, float mesure) {
    // Calcul de l'erreur
    erreur = consigne - mesure;

    // Calcul des termes PID
    float terme_prop = kp * erreur;
    erreur_cumulee += ki * erreur;
    float terme_deriv = kd * (erreur - erreur_precedente);

    // Calcul de la commande finale
    commande = terme_prop + erreur_cumulee + terme_deriv;

    // Limiter la commande pour éviter des valeurs excessives
    // (en fonction des caractéristiques de votre gyropode)
    if (commande > MAX_COMMANDE) {
        commande = MAX_COMMANDE;
    } else if (commande < -MAX_COMMANDE) {
        commande = -MAX_COMMANDE;
    }

    // Appliquer la commande aux moteurs du gyropode
    // (à adapter en fonction de votre configuration matérielle)
    moteurs.setVitesses(commande, -commande);

    // Mettre à jour l'erreur précédente pour le terme dérivé
    erreur_precedente = erreur;
}

#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <controleMoteur.h>
#include <freertos/FreeRTOS.h>
#include <melodie.h>
#include <Encodeur.h>

#define MAX_COMMANDE 100 // Valeur maximale de la commande moteur
#define MAX_COMMANDE_THETA 0.05 // Valeur maximale de la commande moteur

// ----------------------- Déclaration des variables des moteurs ---------------------

ControleMoteur moteurs(27, 25, 33, 32); // Remplacez les numéros de broches selon votre configuration
int vitesseMoteurG = 0;
int vitesseMoteurD = 0;

// ------------------------- Déclaration des variables des pins ------------------------

const int pinLed = 23;
const int pinBuzzer = 26;
const int pinEncD_A = 16;
const int pinEncD_B = 4;
const int pinEncG_A = 34;
const int pinEncG_B = 35;
const int pinBatterie = 39;

// ------------------------- Déclaration des variables pour la mélodie ------------------------

Melodie melodie(26);

// ------------------------- Déclaration des variables des encodeurs ------------------------

Encodeur encodeur(pinEncD_B, pinEncD_A, pinEncG_A, pinEncG_B);
int countD = 0;
int countG = 0;

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

float theta0 = 0.0; // angle d'équilibre

// ----------------------- Déclaration des variables PID -----------------------

// Constantes du régulateur PID
float kp = 500.0; // Gain proportionnel
float ki = 0;     // Gain intégral
float kd = 60.0;  // Gain dérivé

// Variables globales pour le PID
float terme_prop = 0.0;
float terme_deriv = 0.0;
float erreur_cumulee = 0.0;
float erreur_precedente = 0.0;
float commande = 0.0;
float erreur = 0.0;

// ----------------------- Déclaration des variables PID pour l'angle -----------------------

// Constantes du régulateur PID
float kp_t = 1.0; // Gain proportionnel
float ki_t = 0;     // Gain intégral
float kd_t = 1.0;  // Gain dérivé

// Variables globales pour le PID
float terme_prop_t = 0.0;
float terme_deriv_t = 0.0;
float erreur_cumulee_t = 0.0;
float erreur_precedente_t = 0.0;
float commande_t = 0.0;
float erreur_t = 0.0;

// ----------------------- Déclaration des fonctions -----------------------

void asservissementPosition(float consigne, float mesure);
float asservissementTheta(float consigne, float mesure);
void reception(char ch);

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

    countD = encodeur.get_countD();
    countG = encodeur.get_countG();
    encodeur.odometrie();

    float theta_consigne = asservissementTheta(0, encodeur.get_x());
    asservissementPosition(theta_consigne, thetaFC);

    moteurs.updateMoteurs();
    FlagCalcul = 1;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(pinLed, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinBatterie, INPUT);
  pinMode(pinEncD_A, INPUT);
  pinMode(pinEncD_B, INPUT);
  pinMode(pinEncG_A, INPUT);
  pinMode(pinEncG_B, INPUT);

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

  encodeur.init(0, 0, 0, 34, 255, 22, 34);

  moteurs.setAlphaFrottement(0.25);
  //melodie.choisirMelodie(1);
}

void reception(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "kp")
    {
      kp_t = valeur.toFloat();
    }
    if (commande == "kd")
    {
      kd_t = valeur.toInt();
    }
    if (commande == "ki")
    {
      ki_t = valeur.toInt();
    }
    if (commande == "af")
    {
      moteurs.setAlphaFrottement(valeur.toFloat());
    }
    if (commande == "th")
    {
      theta0 = valeur.toFloat();
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void loop()
{
  if (FlagCalcul == 1)
  {
    Serial.printf("%3.1lf %5.1lf %5.1lf %5.1lf \n", erreur, terme_prop, terme_deriv, commande);

    FlagCalcul = 0;
  }

  //Calcul de la tension de la batterie
  float tension = analogRead(pinBatterie) * (7.2 / 1023.0);
  if(tension < 6.5)
  {
    digitalWrite(pinLed, HIGH);
  }
  else
  {
    digitalWrite(pinLed, LOW);
  }

}

// Fonction de régulation PID en position
void asservissementPosition(float consigne, float mesure)
{
  // Calcul de l'erreur
  erreur = (consigne + theta0) - mesure;

  // Calcul des termes PID
  terme_prop = kp * erreur;
  erreur_cumulee += ki * erreur;
  terme_deriv = kd * (-mesure);

  // Calcul de la commande finale
  commande = terme_prop + terme_deriv + erreur_cumulee;

  // Limiter la commande pour éviter des valeurs excessives
  commande = constrain(commande, -MAX_COMMANDE, MAX_COMMANDE);

  // Appliquer la commande aux moteurs du gyropode
  // (à adapter en fonction de votre configuration matérielle)
  moteurs.setVitesses(commande, commande);

  // Mettre à jour l'erreur précédente pour le terme dérivé
  erreur_precedente = erreur;
}

float asservissementTheta(float consigne, float mesure)
{
  // Boucle d'asservissement en pas à pas pour le calcul de la consigne de la boucle en position
  // Entrée : consigne en milimètres
  // Entrée : mesure en milimètres
  // Sortie : consigne theta en radian

  // Calcul de l'erreur
  erreur_t = consigne - mesure;

  // Calcul des termes PID
  terme_prop_t = kp_t * erreur;
  terme_deriv_t = kd_t * (erreur - erreur_precedente_t);
  erreur_cumulee_t += ki_t * erreur;

  // Calcul de la commande finale
  commande_t = terme_prop + terme_deriv + erreur_cumulee_t;

  // Limiter la commande pour éviter des valeurs excessives
  commande_t = constrain(commande_t, -MAX_COMMANDE_THETA, MAX_COMMANDE_THETA);

  // Mettre à jour l'erreur précédente pour le terme dérivé
  erreur_precedente_t = erreur_t;

  // Retourner la consigne theta
  return commande_t;
}

/*void asservissementPasAPas(float consigne_x, float consigne_y)
{
  // Boucle d'asservissement en pas à pas pour le calcul de la consigne de la boucle en position

  // Calcul de l'erreur
  float erreur_x = encodeur.get_x() - encodeur.x_to_step(consigne_x);
  float erreur_y = encodeur.get_y() - encodeur.y_to_step(consigne_y);

  // Calcul des termes PID
  float terme_prop_x = kp_x * erreur_x;
  float terme_prop_y = kp_y * erreur_y;
  float terme_deriv_x = kd_x * (erreur_x - erreur_precedente_x);
  float terme_deriv_y = kd_y * (erreur_y - erreur_precedente_y);
  erreur_cumulee_x += ki_x * erreur_x;
  erreur_cumulee_y += ki_y * erreur_y;

  // Calcul de la commande finale
  float commande_x = terme_prop_x + terme_deriv_x + erreur_cumulee_x;
  float commande_y = terme_prop_y + terme_deriv_y + erreur_cumulee_y;

  // Limiter la commande pour éviter des valeurs excessives
  commande_x = constrain(commande_x, -MAX_COMMANDE_X, MAX_COMMANDE_X);
  commande_y = constrain(commande_y, -MAX_COMMANDE_Y, MAX_COMMANDE_Y);

  // Appliquer la commande aux moteurs du gyropode
  
  // Mettre à jour l'erreur précédente pour le terme dérivé
  erreur_precedente_x = erreur_x;
  erreur_precedente_y = erreur_y;
}*/

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}
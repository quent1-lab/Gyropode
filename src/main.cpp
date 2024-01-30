#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <controleMoteur.h>

// ----------------------- Déclaration des variables des moteurs ---------------------

ControleMoteur moteurs(27, 25, 33, 32);  // Remplacez les numéros de broches selon votre configuration
int vitesseMoteurG = 0;
int vitesseMoteurD = 0;

// ------------------------- Déclaration des variables du mpu ------------------------

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
bool mpu_ok = true;

char FlagCalcul = 0;
float Te = 10;    // période d'échantillonage en ms
float Tau = 1000; // constante de temps du filtre en ms

// coefficient du filtre
float A, B;
// angle projeté
float angleAccel;
// angle gyro
float angleGyro;
// angle filtre
float angleFiltre;

// ----------------------- Déclaration des fonctions -----------------------

// --------------------- Fonction de calcul des angles ---------------------

void controle(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    if(!mpu_ok) continue;
    // Acquisition

    FlagCalcul = 1;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    mpu_ok = false;
    delay(10);
  }else Serial.println("MPU6050 Found!");

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

    //Serial.printf("Angle projeté : %5.1lf  |  Angle Gyro : %5.1lf  |  Avec filtre : %5.1lf\n", angleAccel, angleGyro, angleFiltre);

    FlagCalcul = 0;
  }

  moteurs.setVitesses(150, -100);
  moteurs.updateMoteurs();

  delay(1000);

  moteurs.setVitesses(0, 0);
  moteurs.updateMoteurs();

  delay(500);

  moteurs.setVitesses(-150, 100);
  moteurs.updateMoteurs();

  delay(1000);

  moteurs.setVitesses(0, 0);
  moteurs.updateMoteurs();

  delay(500);
}

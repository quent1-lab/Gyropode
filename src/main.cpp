#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <controleMoteur.h>
#include <freertos/FreeRTOS.h>
#include <melodie.h>
#include <Encodeur.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
QueueHandle_t queue;
QueueHandle_t queueEnvoie;

#define MAX_COMMANDE 100       // Valeur maximale de la commande moteur
#define MAX_COMMANDE_THETv 1.0 // Valeur maximale de la commande moteur

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
float Tau_v = 30;

// coefficient du filtre
float A, B, A_v, B_v;

// angle projeté et gyro
float thetaG, thetaR;

// angle filtre
float thetaGF, thetaRF, thetaFC;

float theta0 = -0.01; // angle d'équilibre

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
float kp_v = 0.0;   // Gain proportionnel
float ki_v = 0.0;   // Gain intégral
float kd_v = 0.0; // Gain dérivé

// Variables globales pour le PID
float terme_prop_v = 0.0;
float terme_deriv_v = 0.0;
float erreur_cumulee_v = 0.0;
float erreur_precedente_v = 0.0;
float commande_v = 0.0;
float erreur_v = 0.0;
float max_commande_v = 0.5;

float vitesse = 0.0;
float vitesse_F = 0.0;
float vitesse_prec = 0.0;
float pos_x = 0;
float pos_x_prec = 0.0;

// ----------------------- Déclaration des fonctions -----------------------

void asservissementPosition(float consigne, float mesure);
float asservissementVitesse(float consigne, float mesure);
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
    // encodeur.odometrie();
    // pos_x = encodeur.get_x();

    pos_x = 1.0 * (countD + countG) / 2;

    vitesse = (pos_x - pos_x_prec) * 1.0 / Te;

    // Ajout d'un filtre passe bas sur la vitesse

    vitesse_F = A_v * (vitesse + B_v * vitesse_prec);
    vitesse_prec = vitesse_F;
    pos_x_prec = pos_x;

    float theta_consigne = asservissementVitesse(0, vitesse_F);
    asservissementPosition(theta_consigne, thetaFC);

    moteurs.updateMoteurs();
    FlagCalcul = 1;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

// --------------------- Fonction de bluethoot ---------------------

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  uint16_t RxTaille, i;
  if (event == ESP_SPP_SRV_OPEN_EVT)
  {
    Serial.println("Client connecté à l'adresse : ");
    for (i = 0; i < 6; i++)
    {
      Serial.printf("%02X", param->srv_open.rem_bda[i]);
      if (i < 5)
      {
        Serial.print(":");
      }
    }
    Serial.println("");
    // Créer la file d'attente lors de la connexion du client
    queue = xQueueCreate(10, sizeof(uint16_t));
  }

  if (event == ESP_SPP_DATA_IND_EVT)
  {
    RxTaille = param->data_ind.len;
    // Envoyer la taille des données à la file d'attente
    if (queue != NULL)
    {
      xQueueSend(queue, &RxTaille, portMAX_DELAY);
    }
    else
    {
      Serial.println("Erreur : la file d'attente n'a pas été créée.");
    }
  }
}

void vReceptionBT(void *pvParameters)
{
  char buffer[101]; // +1 pour le caractère nul de fin de chaîne
  uint16_t data;
  while (1)
  {
    if (queue == NULL)
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    if (xQueueReceive(queue, &data, portMAX_DELAY))
    {
      int bytesRead = SerialBT.readBytes(buffer, data);
      buffer[bytesRead] = '\0'; // Assurez-vous que la chaîne est terminée par un caractère nul
      Serial.printf("Données reçues : %s", buffer);
      for (int i = 0; i < bytesRead; i++)
      {
        reception(buffer[i]);
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void vEnvoieBT(void *pvParameters)
{
  char *data;
  while (1)
  {
    if (queueEnvoie == NULL)
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    if (xQueueReceive(queueEnvoie, &data, portMAX_DELAY))
    {
      SerialBT.printf("%s", data);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
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

  xTaskCreate(controle, "controle", 10000, NULL, 5, NULL);
  xTaskCreate(vReceptionBT, "vReceptionBT", 10000, NULL, 8, NULL);
  xTaskCreate(vEnvoieBT, "vEnvoieBT", 10000, NULL, 10, NULL);

  queueEnvoie = xQueueCreate(100, sizeof(char *));

  // calcul coeff filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te;
  A_v = 1 / (1 + Tau_v / Te);
  B_v = Tau_v / Te;

  encodeur.init(0, 0, 0, 34, 255, 22, 34);

  SerialBT.begin("ESP32_Gyro_Q"); // Nom du module bluetooth
  SerialBT.register_callback(callback);

  moteurs.setAlphaFrottement(0.25);
  // melodie.choisirMelodie(1);
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

    if (commande == "kp_v")
    {
      kp_v = valeur.toFloat();
    }
    if (commande == "kd_v")
    {
      kd_v = valeur.toFloat();
    }
    if (commande == "ki_v")
    {
      ki_v = valeur.toFloat();
    }
    if (commande == "af")
    {
      moteurs.setAlphaFrottement(valeur.toFloat());
    }
    if (commande == "th")
    {
      theta0 = valeur.toFloat();
    }
    if (commande == "to_v")
    {
      Tau_v = valeur.toInt();
      A_v = 1 / (1 + Tau_v / Te);
      B_v = Tau_v / Te;
    }
    if(commande == "max_c")
    {
      max_commande_v = valeur.toFloat();
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
    /*// Allouer de la mémoire pour la chaîne à envoyer
    char *bufferSend = (char *)malloc(100 * sizeof(char));
    if (bufferSend == NULL)
    {
      // Gérer l'erreur d'allocation de mémoire ici
    }

    sprintf(bufferSend, "theta %3.1lf %5.1lf %5.1lf %5.1lf \n", erreur_v, terme_prop_v, terme_deriv_v, commande_v);
    xQueueSend(queueEnvoie, &bufferSend, portMAX_DELAY);*/
    printf("%3.4lf %3.4lf %5.1lf %2.4lf \n", vitesse, vitesse_F, pos_x, commande_v);
    FlagCalcul = 0;

    // N'oubliez pas de libérer la mémoire une fois que vous avez fini de l'utiliser
    // free(bufferSend);
  }

  // Calcul de la tension de la batterie
  /*float tension = analogRead(pinBatterie) * (7.2 / 4095.0);
  Serial.println(tension);
  if(tension < 6.5)
  {
    digitalWrite(pinLed, HIGH);
  }
  else
  {
    digitalWrite(pinLed, LOW);
  }*/
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

float asservissementVitesse(float consigne, float mesure)
{
  // Boucle d'asservissement en pas à pas pour le calcul de la consigne de la boucle en position
  // Entrée : consigne en milimètres
  // Entrée : mesure en milimètres
  // Sortie : consigne theta en radian

  // Calcul de l'erreur
  erreur_v = consigne - mesure;

  // Calcul des termes PID
  terme_prop_v = kp_v * erreur_v;
  terme_deriv_v = kd_v * (erreur_v - erreur_precedente_v) / Te;
  erreur_cumulee_v += ki_v * erreur_v;

  // Calcul de la commande finale
  commande_v = terme_prop_v + terme_deriv_v + erreur_cumulee_v;

  // Limiter la commande pour éviter des valeurs excessives
  commande_v = constrain(commande_v, -max_commande_v, max_commande_v);

  // Mettre à jour l'erreur précédente pour le terme dérivé
  erreur_precedente_v = erreur_v;

  // Retourner la consigne theta
  return commande_v;
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}
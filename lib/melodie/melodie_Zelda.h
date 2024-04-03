#ifndef ZELDA_MELODY_H
#define ZELDA_MELODY_H

#include "pitches.h"

// Mélodie de Zelda
int melodie_Zelda[] = {
  NOTE_E5, NOTE_G5, NOTE_D5, NOTE_A4, NOTE_C5, REST, NOTE_E4, REST, NOTE_G4,
  NOTE_A4, REST, NOTE_B4, REST, NOTE_AS4, NOTE_A4, NOTE_G4, NOTE_E5, NOTE_G5,
  NOTE_D5, NOTE_A4, NOTE_C5, REST, NOTE_E4, REST, NOTE_G4, NOTE_A4, REST,
  NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_C6, NOTE_E6, NOTE_D6, NOTE_C6,
  NOTE_E5, NOTE_G5, NOTE_D5, NOTE_A4, NOTE_C5, REST, NOTE_E4, REST, NOTE_G4,
  NOTE_A4, REST, NOTE_B4, REST, NOTE_AS4, NOTE_A4, NOTE_G4, NOTE_E5, NOTE_G5,
  NOTE_D5, NOTE_A4, NOTE_C5, REST, NOTE_E4, REST, NOTE_G4, NOTE_A4, REST,
  NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_C6, NOTE_E6, NOTE_D6, NOTE_C6,
  REST, REST, REST
};

// Durées des notes de la mélodie de Zelda
int duree_Zelda[] = {
  8, 8, 8, 8, 4, 4, 4, 4, 4,
  8, 8, 8, 8, 4, 4, 4, 4, 4,
  8, 8, 8, 8, 4, 4, 4, 4, 4,
  8, 8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 4, 4, 4, 4, 4,
  8, 8, 8, 8, 4, 4, 4, 4, 4,
  8, 8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 4, 4, 4, 4, 4,
  4, 4, 4
};

// Taille de la mélodie de Zelda
int taille_Zelda = 75;

#endif // ZELDA_MELODY_H
#include "pitches.h"

// Tableau de mélodie
int melodie_Minecraft[] = {
  // Measure 1
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,
  NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_A4,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_A5,
  NOTE_G5, NOTE_F5, NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_C5, NOTE_A4, NOTE_A4,
  
  // Measure 2
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,
  NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_A4,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_A5,
  NOTE_G5, NOTE_F5, NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_C5, NOTE_A4, NOTE_A4,
  
  // Measure 3
  NOTE_E5, NOTE_C5,
  NOTE_D5, NOTE_B4,
  NOTE_C5, NOTE_A4,
  NOTE_GS4, NOTE_B4,
  NOTE_E5, NOTE_C5,
  NOTE_D5, NOTE_B4,
  NOTE_C5, NOTE_E5, NOTE_A5,
  NOTE_GS5
};

// Tableau des durées
int duree_Minecraft[] = {
  // Measure 1
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 4,
  8, 4, 4, 8, 4, 8,
  8, 4, 4, 8, 4, 8,
  4, 4, 4,
  
  // Measure 2
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 4,
  8, 4, 4, 8, 4, 8,
  8, 4, 4, 8, 4, 8,
  4, 4, 4,
  
  // Measure 3
  2, 2,
  2, 2,
  2, 2,
  2, 4,
  2, 2,
  2, 2,
  4, 2,
  2
};


// Taille de la mélodie de Tetris 8 bits
int taille_Minecraft = 93;

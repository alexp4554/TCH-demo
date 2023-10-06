/*
         __ ___  __
        |_   |  (_
        |__  |  __)

        MIT License

        Copyright (c) 2018	École de technologie supérieure

        Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify and/or merge copies of the Software, and to
   permit persons to whom the Software is furnished to do so, subject to the
   following conditions:

        The above copyright notice and this permission notice shall be included
   in all copies or substantial portions of the Software.
*/
/**
        \file
        \brief Code source de fonctions qui pilotent directement du matériel

        \author Noms de membres de l'équipe
        \author idem
        \author idem
        \author idem
        \author idem
        \date date d'aujourd'hui
*/

/* ----------------------------------------------------------------------------
Includes
---------------------------------------------------------------------------- */

#include "driver.h"
#include "utils.h"
#include <avr/io.h>
#include <stdint.h>

#include <util/delay.h>
#include <stdio.h>
#include "uart.h"

/* ----------------------------------------------------------------------------
Function definition
---------------------------------------------------------------------------- */

void adc_init(void) {

  // 1-Configuration des broches du port A à mettre en entrée
  DDRA = clear_bit(DDRA, PA0);
  DDRA = clear_bit(DDRA, PA1);
  // DDRA = clear_bit(DDRA, PA2);
  DDRA = clear_bit(DDRA, PA3);
  // DDRA = clear_bit(DDRA, PA4);

  // 2-Sélectionner la référence de tension: la tension d'alimentation
  ADMUX = clear_bit(ADMUX, REFS1);
  ADMUX = set_bit(ADMUX, REFS0);
  // 3-Choisir le format du résultat de conversion: shift a gauche pour que
  // les 8 MSB se retrouvent dans le registre ADCH (ADLAR=1)
  ADMUX = set_bit(ADMUX, ADLAR);
  // 4-Choisir le facteur de division de l'horloge
  // ( L'horloge l'ADC ne doit pas dépasser 200kHz. Avec une horloge de 8MHZ, ça
  // prend une division d'horloge de min 40. Donc 64 ou 128) */
  ADCSRA = set_bits(ADCSRA, 0b00000111);
  // 5-Activer le CAN
  ADCSRA = set_bit(ADCSRA, ADEN);
}

uint8_t adc_read(uint8_t canal) {

  // 1-Sélection de l'entrée à convertir (registre ADMUX<-canal)
  ADMUX = write_bits(ADMUX, 0b00000111, canal);

  // 2-Démarrage d'une conversion
  ADCSRA = set_bit(ADCSRA, ADSC);
  // 3-Attente de la fin de conversion
  while (read_bit(ADCSRA, ADSC))
    ;
  // 4-Lecture et renvoi du résultat
  return ADCH;
}

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

void adc_init(void) 
{
  // 1-Configuration des broches du port A à mettre en entrée
  DDRA = clear_bit(DDRA, PA0);
  DDRA = clear_bit(DDRA, PA1);
  // DDRA = clear_bit(DDRA, PA2);
  // DDRA = clear_bit(DDRA, PA3);
  // DDRA = clear_bit(DDRA, PA4);

  // 2-Sélectionner la référence de tension: la tension d'alimentation
  ADMUX = clear_bit(ADMUX, REFS1);
  ADMUX = set_bit(ADMUX, REFS0);
  
  // 3-Choisir le format du résultat de conversion: shift a gauche pour que
  // les 8 MSB se retrouvent dans le registre ADCH (ADLAR=1)
  ADMUX = set_bit(ADMUX, ADLAR);
  
  // 4-Choisir le facteur de division de l'horloge
  // ( L'horloge l'ADC ne doit pas dépasser 200kHz. Avec une horloge de 8MHZ, ça
  // prend une division d'horloge de min 40. Donc 64 ou 128)
  ADCSRA = set_bits(ADCSRA, 0b00000111);
  
  // 5-Activer le CAN
  ADCSRA = set_bit(ADCSRA, ADEN);
}

uint8_t adc_read(uint8_t canal) 
{
  // 1-Sélection de l'entrée à convertir (registre ADMUX<-canal)
  ADMUX = write_bits(ADMUX, 0b00000111, canal);

  // 2-Démarrage d'une conversion
  ADCSRA = set_bit(ADCSRA, ADSC);
  
  // 3-Attente de la fin de conversion
  while (read_bit(ADCSRA, ADSC));
  
  // 4-Lecture et renvoi du résultat
  return ADCH;
}

void pwm0_init(void) 
{
  // 1-Configuration des broches de sortie (registre DDRB:PB4<-1,PB3<-1) 
  // 1.1-Mettre les broches de la modulation de largeur d'impulsion en sortie
	DDRB = set_bits(DDRB, 0b00011000); // Broches MLI Moteurs gauches et droites
	
  // 2-Initialisation du TIMER 0
  // 2.1- Mode de comparaison : "Toggle on compare match" (registre TCCR0A:
  // COM0A1<-1,COM0A0<-0,COM0B1<-1,COM0B0<-0)
	TCCR0A  = clear_bits(TCCR0A, 0b01010000);
	TCCR0A  = set_bits(TCCR0A, 0b10100000);
	
  // 2.2- Mode du compteur :  "PWM phase correct (avec valeur TOP égale à 255)"
  //(registre TCCR0B: WGM02<-0 ; registre TCCR0A: WGM01<-0,WGM00<-1)
	TCCR0B  = clear_bits(TCCR0B, 0b00001000);
	TCCR0A  = clear_bits(TCCR0A, 0b00000010);
	TCCR0A  = set_bits(TCCR0A, 0b00000001);
	
  // 2.3- Fixer la valeur initiale du compteur 0 à 0 (registre TCNT0<-0)
	TCNT0 = clear_bits(TCNT0, 0b11111111);
	
  // 2.4- Facteur de division de fréquence : 1 (registre TCCR0B:
  // CS02<-0,CS01<-0,CS00<-1)
	TCCR0B = clear_bits(TCCR0B, 0b00000110);
	TCCR0B = set_bits(TCCR0B, 0b00000001);
}

//vitesse moteur roues droite
void pwm0_set_PB3(uint8_t limite)
{
  // Choisir le rapport cyclique en fixant la valeur de limite (registre OCROA<-limite)
  OCR0A = clear_bits(OCR0A, 0b11111111);
  OCR0A = set_bits(OCR0A, limite);
}

//vitesse moteur roues gauche
void pwm0_set_PB4(uint8_t limite) 
{
  // Choisir le rapport cyclique en fixant la valeur de limite (registre
  // OCROB<-limite)
  OCR0B = clear_bits(OCR0B, 0b11111111);
  OCR0B = set_bits(OCR0B, limite);
}


void pwm1_init(uint16_t top) 
{
  // broches de PWM en sortie (registre DDRD:PD4<-1,PD5<-1)
  DDRD = set_bit(DDRD, PD4);
  DDRD = set_bit(DDRD, PD5);

  // 2.1- Mode de comparaison : "Clear OCnA/OCnB on Compare Match when
  // up-counting...
  // ...Set OCnA/OCnB on Compare Match when downcounting."
  // (registre TCCR1A: COM1A1<-1,COM1A0<-0,COM1B1<-1,COM1B0<-0)
  TCCR1A = set_bit(TCCR1A, COM1A1);
  TCCR1A = clear_bit(TCCR1A, COM1A0);
  TCCR1A = set_bit(TCCR1A, COM1B1);
  TCCR1A = clear_bit(TCCR1A, COM1B0);

  // 2.2- Mode du compteur :  "Fast PWM mode (avec valeur TOP fixé par ICR1)"
  // (registre TCCR1B: WGM13<-1,WGM12<-1,WGM11<-1,WGM10<-0)
  TCCR1B = set_bit(TCCR1B, WGM13);
  TCCR1B = set_bit(TCCR1B, WGM12);
  TCCR1A = set_bit(TCCR1A, WGM11);
  TCCR1A = clear_bit(TCCR1A, WGM10);

  // 2.3- Fixer la valeur initiale du compteur 0 à 0 (registre TCNT1<-0)
  TCNT1 = 0;

  // 2.4- fixer la valeur maximale (TOP) du compteur 1 (registre ICR1<-top)
  ICR1 = top;

  // 2.5- Facteur de division de fréquence : 8 (registre TCCR1B:
  // CS12<-0,CS11<-1,CS10<-0) activer l'horloge avec facteur de division par 8
  TCCR1B = clear_bit(TCCR1B, CS12);
  TCCR1B = set_bit(TCCR1B, CS11);
  TCCR1B = clear_bit(TCCR1B, CS10);
}
//servo moteur pour angle de lancement
void pwm1_set_PD5(uint16_t limite) 
{
  // Choisir le rapport cyclique en fixant la valeur de limite (OCR1A<-limite)
  OCR1A = limite;
}

void pwm1_set_PD4(uint16_t limite) 
{
  // Choisir le rapport cyclique en fixant la valeur de limite (OCR1B<-limite)
  OCR1B = limite;
}

void pwm2_init() 
{
  // broches de PWM en sortie (registre DDRD:PD6<-1,PD7<-1)
  DDRD = set_bits(DDRD, 0b11000000);// Broches MLI pour moteur angle de lancement et moteur roue innertie

  // 2.1- Mode de comparaison : "Clear OC2B on Compare Match when up-counting...
  //							...Set OC2B on Compare
  // Match when down-counting."
  // (registre TCCR2A: COM2A1<-1,COM2A0<-0,COM2B1<-1,COM2B0<-0)
    TCCR2A  = clear_bits(TCCR2A, 0b01010000);
    TCCR2A  = set_bits(TCCR2A, 0b10100000);
  // 2.2- Mode du compteur :  "PWM, Phase Correct  (avec valeur TOP égale à
  // 255)" (registre TCCR2B: WGM22<-0) (registre TCCR2A: WGM21<-0,WGM20<-1)
	TCCR2B  = clear_bits(TCCR2B, 0b00001000);
	TCCR2A  = clear_bits(TCCR2A, 0b00000010);
	TCCR2A  = set_bits(TCCR2A, 0b00000001);
  // 2.3- Fixer la valeur initiale du compteur 0 à 0 (registre TCNT2<-0)
	TCNT2 = clear_bits(TCNT2, 0b11111111);
  // 2.4- Facteur de division de fréquence : 8 (registre TCCR2B:
  // CS22<-0,CS21<-1,CS20<-0)
  TCCR2B = clear_bits(TCCR0B, 0b00000101);
  TCCR2B = set_bits(TCCR0B, 0b00000010);
}

//vitesse moteur roue innertie
void pwm2_set_PD7(uint8_t limite) 
{
  // Choisir le rapport cyclique en fixant la valeur de limite (OCR2A<-limite)
    OCR2A = clear_bits(OCR2A, 0b11111111);
    OCR2A = set_bits(OCR2A, limite);
}

//vitesse moteur angle lancement
void pwm2_set_PD6(uint8_t limite) 
{
  // Choisir le rapport cyclique en fixant la valeur de limite (OCR2B<-limite)
    OCR2B = clear_bits(OCR2B, 0b11111111);
    OCR2B = set_bits(OCR2B, limite);
}

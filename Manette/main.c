/**
	@file main.c
	@brief Programme responsable du fonctionnement de la manette
	@author Samuel Laplante
	@author Alexandre Perreault
	@author Marc-Antoine Lauzon
	@author Cynthia Routhier
	@date 2023-02-03
	
	Réaliser dans le cadre du cours TCH098
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "lcd.h"
#include "utils.h"
#include "driver.h"
#include "uart.h"
#include "comms.h"


int main(void)
{
	//activation des PINS en Entree
	DDRA = clear_bit(DDRA,PA2);
	DDRA = clear_bit(DDRA,PA4);
	DDRD = clear_bit(DDRD,PD3);
	DDRD = clear_bit(DDRD,PD4);
	DDRD = clear_bit(DDRD,PD5);
	DDRD = clear_bit(DDRD,PD6);
	DDRD = clear_bit(DDRD,PD7);
	//Activation des PULL-UP des pins en Entree
	PORTA=set_bit(PORTA,PA2);
	PORTA=set_bit(PORTA,PA4);
	PORTD=set_bit(PORTD,PD3);
	PORTD=set_bit(PORTD,PD4);
	PORTD=set_bit(PORTD,PD5);
	PORTD=set_bit(PORTD,PD6);
	PORTD=set_bit(PORTD,PD7);
	//Activation des PINS des Dels En Sortie
	DDRB = set_bit(DDRB,PB0);
	DDRB = set_bit(DDRB,PB1);
	DDRB = set_bit(DDRB,PB2);
	DDRB = set_bit(DDRB,PB3);
	DDRB = set_bit(DDRB,PB4);
	//Eteindre les Dels
	PORTB=clear_bit(PORTB,PB0);
	PORTB=clear_bit(PORTB,PB1);
	PORTB=clear_bit(PORTB,PB2);
	PORTB=clear_bit(PORTB,PB3);
	PORTB=clear_bit(PORTB,PB4);
	
	//---------------attributs-----------------------------------------------------
	
	char lcdScreenText[40];//texte a afficher sur l'ecran LCD
	uint8_t joystickX;// valeur du potentiometre lineaire X du Joystick
	uint8_t joystickY;// valeur du potentiometre lineaire Y du Joystick
	uint8_t slider;// valeur du potentiometre lineaire 
	bool buttons[7]; // Valeur des 7 boutons
	
	buttons[3]=0;//mise a zero de la valeure du buttons[3] . Sinon causais des probleme lor du redemarrage de la manette
	
	bool flywheelActivated = FALSE; //indique si la roue d'inertie est activer ou desactiver
	bool button3StateChanged = TRUE;//indique si l'etat du bouton 3 a été modifier
	
	//---------------initialisation------------------------------------------------------------
	
	uart_init(UART_0);//initialisation de la communication UART
	lcd_init();// initialisation de l'�cran
	adc_init();// initialisation du CAN (ADC)
	sei();//activer les interruptions globales
	
	//----------méthodes-------------------------------------------------------------------
	
	/*
	*fonction responsable de l'ecriture sur l'ecran LCD
	*/
	void writeOnScreen()
	{
		//reset ecran
		lcd_clear_display();
		lcd_set_cursor_position(0,0);
		//ecrire sur ecran
		sprintf(lcdScreenText,"B1%d,B2%d,B3%d,B4%d,B5%d,B6%d,B7%d,S%d",buttons[0],buttons[1],buttons[2],buttons[3],buttons[4],buttons[5],buttons[6],slider);
		lcd_write_string(lcdScreenText);
	}

	/*
	*fonction responsable d'ouvrir ou de fermer les dels
	*/
	void activateLEDs()
	{
		if(flywheelActivated)//si la roue d'inertie est activée les Dels sont activées
		{
			PORTB=set_bit(PORTB,PB0);
			PORTB=set_bit(PORTB,PB1);
			PORTB=set_bit(PORTB,PB2);
			PORTB=set_bit(PORTB,PB3);
			PORTB=set_bit(PORTB,PB4);
		}	
		else//si la roue d'inertie est Désactivée les Dels sont Désactivées
		{
			PORTB=clear_bit(PORTB,PB0);
			PORTB=clear_bit(PORTB,PB1);
			PORTB=clear_bit(PORTB,PB2);
			PORTB=clear_bit(PORTB,PB3);
			PORTB=clear_bit(PORTB,PB4);
		}
	}

	/*
	* fonction responsable de lire l'etat du bouton 3 et de modifier l'etat de la roue d'inertie
	*
	* cette fonction permet au bouton 3 d'agir comme un bouton Toogle qui active ou desactive la roue d'inertie
	*/
	void activateFlywheel()
	{
		if(!read_bit(PIND,PD4))
		{
			if(button3StateChanged)
			{
				button3StateChanged=FALSE;
				flywheelActivated = !flywheelActivated;//on modifie l'etat (active/desactive) de la roue d'inertie
				buttons[3] = flywheelActivated;// buttons[3] représente l'état de la roue d'innertie
				activateLEDs();//ouvrir ou fermé les dels
			}	
		}
		else
		{
			if(!button3StateChanged);
				button3StateChanged=TRUE;
		}
	}

    //----------------------------boucle infinie-----------------------------------------------
    while (1) 
    {
		//aquisition des entrées (potentiometres lineaire et bouttons)
		joystickX = 255-adc_read(PINA0);// Lis l'état de l'entree du potentiometres lineaire X du JoyStick, l'inverse et l'inscrit dans joystickX
		joystickY = 255-adc_read(PINA1);// Lis l'état de l'entree du potentiometres lineaire Y du JoyStick, l'inverse et l'inscrit dans joystickY
		slider    = 255-adc_read(PINA3);// Lis l'état de l'entree du potentiometres lineaire, l'inverse et l'inscrit dans slider
		buttons[0]=!read_bit(PINA,PA2);	// Lis l'état de l'entree du boutton 0 et l'inscrit dans buttons[0]
		buttons[1]=!read_bit(PINA,PA4);	// Lis l'état de l'entree du boutton 1 et l'inscrit dans buttons[1]
		buttons[2]=!read_bit(PIND,PD3);	// Lis l'état de l'entree du boutton 2 et l'inscrit dans buttons[2]
		activateFlywheel();				// Lis l'état de l'entree du boutton 3 et inscrit l'etat de la roue d'inertie dans buttons[3]
		buttons[4]=!read_bit(PIND,PD5);	// Lis l'état de l'entree du boutton 4 et l'inscrit dans buttons[4]
		buttons[5]=!read_bit(PIND,PD6);	// Lis l'état de l'entree du boutton 5 et l'inscrit dans buttons[5]
		buttons[6]=!read_bit(PIND,PD7);	// Lis l'état de l'entree du boutton 6 et l'inscrit dans buttons[6]
		
		//transmission des données
		uint8_t bools = combine_bool(buttons[0],buttons[1],buttons[2],buttons[3],buttons[4],buttons[5],buttons[6]);//transforme les 7 bits des boutons en un entier non signé de 8 bits, dont le dernier bit est egal a 1
		send_packet(UART_0,joystickX,joystickY,slider,bools);//envoyer les données au Serveur (Vehicule) par Wi-fi
		
		//ecrire sur l'ecran
		writeOnScreen();
		
		_delay_ms(99);
    }
}


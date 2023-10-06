
/**
	@file main.c
	@brief Programme responsable du fonctionnement du vehicule
	@author Samuel Laplante
	@author Alexandre Perreault
	@author Marc-Antoine Lauzon
	@author Cynthia Routhier
	@date 2023-02-03
	
	Réaliser dans le cadre du cours TCH098
*/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <math.h>
#include "driver.h"
#include "utils.h"
#include "lcd.h"
#include "uart.h"
#include "comms.h"

#define DISCONNECT_TRESHOLD		30000	//nombre de cycle realiser sans avoir recu de nouvelle informations seuil oule mode deconnecté est activer
#define FLYWHEEL_SPEED			93		//vitesse de la roue d'innertie
#define SERVO_RETRACT_TIME		10		//nombre de cycle a realiser avant la retraction du servo moteur
#define MICRO_MOTOR_SPEED		255		//vitesse du Micro Moteur controllant L'elevation de la plateforme
#define SERV0_DEFAULT_POSITION	430		//position par Default du servo moteur
#define SERV0_OPEN_POSITION		880		//position du servo moteur lorsqu'il est enclencher

int main(void)
{
	//----------------attributs---------------------------------------------------------------------------
	
	//Compteur de Deconnexion d'Arret Automatique
	uint16_t disconnectCounter = 0;
	
	//texte a afficher sur l'ecran LCD
	char lcdScreenText[40];
	
	//donnees entrantes
	char receivedData[4];//tableau contenant les données brutes recus
	int8_t joyStickX=0; // valeur du potentiometre lineaire X du Joystick
	int8_t joyStickY=0;// valeur du potentiometre lineaire Y du Joystick
	uint8_t slider=0;// valeur du potentiometre lineaire 
	bool buttons[8]={0};// Valeur des 7 boutons ainsi que le dernier bit toujours recu a 1
	
	//changement d'etat des boutons
	bool alreadyPressed[7] ={FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};
		
	//lancement du disque
	uint8_t servoCounter = 0;// compteur qui incremente a chaque cycle ou le servo moteur est en position ouverte
	bool servoCounterIsActive = FALSE;// indique si le compteur du servo moteur est actif, soit qu'il incremenente a chaque cycle.
	
	//vitesse et orientation de deplacement du Vehicule
	int8_t left = 0; //contient une valeure intermediaire au calcule de la vitesse des moteurs de gauche
	int8_t right = 0;//contient une valeure intermediaire au calcule de la vitesse des moteurs de droite
	bool rightMotorsforwardMovement;
	bool leftMotorsforwardMovement;
	uint8_t leftMotorsSpeed=0;
	uint8_t rightMotorsSpeed=0; 
	
	//-------------------initialisation-----------------------------------------------------
	_delay_ms(3000);
	
	//assurer que la broche RESET du module ESP8266 est bien toujours forcée à 1.
	DDRD=set_bit(DDRD,PD2);
	PORTD=set_bit(PORTD,PD2);
	//Activation des pins de sens de rotation des moteurs
	DDRB = set_bits(DDRB, 0b00000111);
	DDRB = set_bit(DDRB,PINB);
	
	uart_init(UART_0);//initialisation de la communication UART
	sei();//activer les interruptions globales
	pwm0_init(); //initialisation de la modulation de largeur d'impulsions 0
	pwm1_init(20000);//initialisation de la modulation de largeur d'impulsions 1
	pwm2_init();//initialisation de la modulation de largeur d'impulsions 2
	lcd_init();// initialisation de l'écran
	
	//mettre le servo moteur a sa position par default
	pwm1_set_PD5(SERV0_DEFAULT_POSITION);
	
	//---------------methodes----------------------------------------------------------------------------
	
	/*
	*fonction responsable de l'ecriture sur l'ecran LCD lorsque qu'une manette est connecte
	*/
	void writeOnScreen()
	{
		//reset ecran
		lcd_clear_display();
		lcd_set_cursor_position(0,0);
		//ecrire sur ecran
		sprintf(lcdScreenText,"B1%d,B2%d,B3%d,B4%d,B5%d,B6%d,Vo%d",buttons[1],buttons[2],buttons[3],buttons[4],buttons[5],buttons[6],servoCounter);
		lcd_write_string(lcdScreenText);
	}

	/*
	*fonction responsable de l'ecriture sur l'ecran LCD lorsque qu'aucune manettes n'est connecter
	*/
	void writeOffScreen()
	{
		//reset ecran
		lcd_clear_display();
		lcd_set_cursor_position(0,0);
		//ecrire sur ecran
		lcd_write_string("404 CONNEXION NOT FOUND");
	}

	/*
	* fonction qui traduit les données du joystick en valeurs permetant de calculer la vitesse des moteurs
	*/
    void translate()
	{
		//module la vitesse avancer/reculer selon la valeur du potentiomètre linéaire
		joyStickY = joyStickY*slider/255; 
		
		//module la vitesse de rotation selon la vitesse avancer/reculer 
		// plus JoyStickY est elever, plus JoystickX est moduler/reduit	(pour avoir un meilleur contrôle des rotations)
		// JoyStickX est initiallement compris entre 0 et 127 (positif ou negatif)
		// apres modulation, JoyStickX équivaux entre 30% et 70% de sa valeure initiale	 ((127/3.175=40)|(0/3.175=0)) + 30 = 30|70
		joyStickX =	joyStickX*((127-abs(joyStickY))/3.175+30)/100;
		
		//variables necessaire à l'implémentation d'un "Arcade Drive"
		int8_t maximum = max(abs(joyStickX),abs(joyStickY));
		int8_t total = joyStickX + joyStickY;
		int8_t difference = joyStickY - joyStickX;

		if (joyStickY >= 0)
		{
			if (joyStickX >= 0)
			{
				left = maximum;
				right = difference;
			}
			else
			{
				left = total;
				right = maximum;
			}
		} 
		else 
		{
			if (joyStickX >= 0)
			{
				left = total;
				right = -maximum;
			}
			else
			{
				left = -maximum;
				right = difference;
			}
		}
	}


	/*
	* Fonction qui prend les données traduites par la fonction translate et les transformes 
	* en deux entiers non signée 8 bits et deux boolean pour la vitesse et orientation des moteurs droite et gauche
	*/
	void drive()
	{
		translate();

		rightMotorsSpeed = abs(right)*2;
		leftMotorsSpeed = abs(left)*2;

		if(right>=0)
			rightMotorsforwardMovement = TRUE;
		else
			rightMotorsforwardMovement = FALSE;
			
		if(left>=0)
			leftMotorsforwardMovement = TRUE;
		else
			leftMotorsforwardMovement = FALSE;
	}
	
	/*
	*fonction qui active les moteurs de deplacement (zoooooooooooom)
	*/
	void activateWheelMotors()
	{
		//vitesse et orientation des moteurs de droite
		pwm0_set_PB3(leftMotorsSpeed);
		if(leftMotorsforwardMovement)
			PORTB = set_bit(PORTB,PB1);
		else
			PORTB = clear_bit(PORTB,PB1);
		
		//vitesse et orientation des moteurs de gauche
		pwm0_set_PB4(rightMotorsSpeed);
		if(rightMotorsforwardMovement)
			PORTB = set_bit(PORTB,PB2);
		else
			PORTB = clear_bit(PORTB,PB2);
	}
	
	/*
	*fonction responsable du Bouton 1
	* Activer le Servo Moteur pour Lancer le disque
	*
	* Apres avoir appuyer sur le bouton, le servo moteur reste en position ouvert pendant un certain delai
	* Ce delai est reinitialiser chaque fois que le bouton est maintenu ou réenclenché.
	*/
	void checkButton1()
	{
		if(servoCounterIsActive)
		{
			servoCounter++;
			if(servoCounter>=SERVO_RETRACT_TIME)
			{
				pwm1_set_PD5(SERV0_DEFAULT_POSITION);//etat de repos
				servoCounterIsActive = FALSE;
				servoCounter = 0;
			}
		}	
		
		if(buttons[1])
		{
			if(!alreadyPressed[1])
			{
				//Action instantannee du bouton
				alreadyPressed[1]=TRUE;
				
				servoCounterIsActive = TRUE;
				pwm1_set_PD5(SERV0_OPEN_POSITION);//etat de tir
			}	
			//Action continue du bouton
			servoCounter = 0;
		}
		else
		{
			alreadyPressed[1]=FALSE;
		}	
	}
	
	/*
	* fonction responsable du Bouton 2
	* rotation a droite sans deplacement 
	*/
	void checkButton2()
	{
		if(buttons[2])
		{
			if(!alreadyPressed[2])
			{
				//Action instantannee du bouton
				alreadyPressed[2]=TRUE;
			}	
			//Action continue du bouton
			rightMotorsSpeed = 128*slider/255;
			leftMotorsSpeed= 128*slider/255;
			rightMotorsforwardMovement=FALSE;
			leftMotorsforwardMovement=TRUE;
			
		}
		else
		{
			alreadyPressed[2]=FALSE;
		}	
	}
	
	/*
	*fonction responsable du Bouton 3
	*activation/Arret de la roue d'inertie
	*/
	void checkButton3()
	{
		if(buttons[3])
		{
			if(!alreadyPressed[3])
			{
				//Action instantannee du bouton
				alreadyPressed[3]=TRUE;
			}	
			//Action continue du bouton
			pwm2_set_PD7(FLYWHEEL_SPEED);//vitesse du moteur de la roue d'inertie
			
		}
		else
		{
			alreadyPressed[3]=FALSE;
			pwm2_set_PD7(0); //vitesse du moteur de la roue d'inertie mise a zero
		}	
	}
	
	/*
	*fonction responsable du Bouton 4 
	* Monter la plateforme
	*/
	void checkButton4()
	{
		if(buttons[4])
		{
			if(!alreadyPressed[4])
			{
				//Action instantannee du bouton
				alreadyPressed[4]=TRUE;
			
				pwm2_set_PD6(MICRO_MOTOR_SPEED);//vitesse du micro moteur
				PORTB = set_bit(PORTB,PB0); //orientation du micro moteur 
			}	
			//Action continue du bouton
		}
		else
		{
			if(alreadyPressed[4])
			{
				alreadyPressed[4]=FALSE;
				pwm2_set_PD6(0); //vitesse du micro moteur mise a zero
			}
		}	
	}
	
	/*
	* fonction responsable du Bouton 5
	* rotation a gauche sans deplacement 
	*/
	void checkButton5()
	{
		if(buttons[5])
		{
			if(!alreadyPressed[5])
			{
				//Action instantannee du bouton
				alreadyPressed[5]=TRUE;
			}	
			//Action continue du bouton
			rightMotorsSpeed = 128*slider/255;
			leftMotorsSpeed= 128*slider/255;
			rightMotorsforwardMovement=TRUE;
			leftMotorsforwardMovement=FALSE;
		}
		else
		{
			alreadyPressed[5]=FALSE;
		}	
	}
	
	/*
	* fonction responsable du Bouton 6
	* descendre la plateforme
	*/
	void checkButton6()
	{
		if(buttons[6])
		{
			
			if(!alreadyPressed[6])
			{
				alreadyPressed[6]=TRUE;
				
				//Action instantannee du bouton
				pwm2_set_PD6(MICRO_MOTOR_SPEED);//vitesse du micro moteur 
				PORTB = clear_bit(PORTB,PB0);//orientation du micro moteur 
			}	
			//Action continue du bouton
		}
		else
		{
			if(alreadyPressed[6])
			{
				alreadyPressed[6]=FALSE;
				pwm2_set_PD6(0); //vitesse du micro moteur mise a zero
			}
		}	
	}
	


	/*
	*----------------------------------------------Boucle Infinie-------------------------------------------------------------------------------
	*/
	while(1)
	{
		//realise les action normales seulement si des nouvelles données ont été recus
 		if(!uart_is_rx_buffer_empty(UART_0))
 		{	
			 disconnectCounter = 0; // remise a zero du compteur de deconnexion
			 
			 //reception des donnees 
			 receive_packet(UART_0,receivedData);
			 joyStickX = receivedData[0] -128;// // transforme uint8_t en int8_t (transforme de 0 a 255 pour -128 a 127)
			 joyStickY = receivedData[1] -128; // transforme uint8_t en int8_t (transforme de 0 a 255 pour -128 a 127)
			 slider    = receivedData[2];
			 uint8_to_bool_array(receivedData[3],buttons); //transformé entier non signe de 8 bit en un table de boolean representant chaque bouttons
			 
			drive(); //determiner vitesse et orientation des moteurs de depalcement
			
			checkButton1();//lancer frisbee
			checkButton2();//rotation a Droite //doit etre apres drive()
			checkButton3();// activer/desactriver Roue inertie
			checkButton4();//monter la plateforme
			checkButton5();//rotation a gauche //doit etre apres  drive()
			checkButton6();//descendre la plateforme
			
			activateWheelMotors();
						
			writeOnScreen();
 		}	
		else
		{
			//compteur qui incremente a chaque cycle ou aucune nouvelle information n'a ete recus 
			 disconnectCounter++;
			 
			 //si le nombre de cycle sans nouvelle information recu depasse le seuil, passe en mode deconnecté
			 if(disconnectCounter>=DISCONNECT_TRESHOLD)
			 { 
				 //mise a zero de toutes les valeurs d'entree
				 joyStickX = 0;
				 joyStickY = 0;
				 slider=0;
				 buttons[0]= FALSE;
				 buttons[1]= FALSE;
				 buttons[2]= FALSE;
				 buttons[3]= FALSE;
				 buttons[4]= FALSE;
				 buttons[5]= FALSE;
				 buttons[6]= FALSE;
				 
				 drive();//determiner vitesse et orientation des moteurs de depalcement
				 
				 checkButton1();//lancer frisbee
				 checkButton2();//rotation a Droite //doit etre apres drive()
				 checkButton3();// activer/desactriver Roue inertie
				 checkButton4();//monter la plateforme
				 checkButton5();//rotation a gauche //doit etre apres  drive()
				 checkButton6();//descendre la plateforme
				 activateWheelMotors();
				 
				 writeOffScreen();
				 
				 disconnectCounter=0;
			 }
		 }
	}
}
#include "comms.h"
#include "utils.h"
#include "uart.h"
#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>

/**
	@file comms.c
	@brief Librairie responsable de la communication entre la manette (client) et le vehicule (serveur)
	@author Alexandre Perreault
	@author Samuel Laplante
	@date 2023-03-24
	
	Réaliser dans le cadre du cours TCH098 (Equipe 34)
*/

/*
*Méthode qui recoit 7 booleans en parametre et retourne un entier non signée de 8 bit
*Chaque bit de l'entier correspond respectivement a un des booleans. Le dernier bit est egal a 1
*/
uint8_t combine_bool(bool S1,bool S2,bool S3,bool S4,bool S5,bool S6,bool S7)
{
    //Fusionner les bits des boutons pour former un Byte
    uint8_t buttonsByte = (S1 << 7) | (S2 << 6) | (S3 << 5) | (S4 << 4) | (S5 << 3) | (S6 << 2) | (S7 << 1) | 1;
    return buttonsByte;
}

/*
*Méthode qui recoit 4 entiers non signé de 8 bit chacuns et envoie Au module Wi-FI un paquet les contenant en utilisant le port recu en parametre
*/
void send_packet(uart_e port, uint8_t x, uint8_t y, uint8_t potentiometre, uint8_t buttonsByte)
{
    uart_put_byte(port, 0);
	
	//Aucun Byte ne doit etre 0, Sinon problemes avec le protocole de communication
    if (x == 0) 
        x = 1;
    if (y == 0) 
        y = 1;
    if (potentiometre == 0) 
        potentiometre = 1;
    
	//envoyer le paquet d'informations
    char packet[5] = {x,y,potentiometre,buttonsByte,'\0'};
    uart_put_string(port, packet);
}

/*
*Méthode recupere les donnees recus par le module Wi-FI ,utilisant le port en parametre, et les renvoient dans le tableau de charactere recu en parametre
*/
void receive_packet(uart_e port, char* packet)
{
    //on commence par lire le buffer UART jusqu'a trouver un 0, qui signifie le début d'un packet
    //ensuite, on lit les 4 bits suivants qui sont le packet d'information qu'on souhaite
    uint8_t buffer = 1;
    
    if(uart_is_rx_buffer_empty(port)==0)
	{    
        while (buffer!=0)
        {
            buffer = uart_get_byte(port);
			_delay_ms(2);
        }
        for (int i = 0; i < 4; i++)
        {
            packet[i] = uart_get_byte(port);
			_delay_ms(1);
        } 
    }
}
/*
* transforme un Byte uint8_t en un tableau de bits
*/
void uint8_to_bool_array(uint8_t value, bool* result)
{
    for (size_t i = 0; i < 8; i++) 
        result[8 - i - 1] = (value & (1 << i)) ? TRUE : FALSE;
}


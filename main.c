
//Default TX address 0x11
//Change if necessary in nRF24L01P.h definitions
//Transmitter sample code

#include <avr/io.h>
#include <util/delay.h>
#include "nRF24L01P.h"


int main(void){

RF_START(2); // set channel number

while(1){
           
	  //For Transmitter Node
           RF_TX_TEXT("nRF24L01+ Says Hello!");   
           _delay_ms(500);
	
	/*
	  //for Receiver Node uncomment this section
	  
	  uint8_t rx_buf[32],rx_data_len=0;
		   
          if(RF_RX(rx_buf,&rx_data_len)){
	      //received data in 'rx_buf' buffer
              //do your stuff after receiving
	     }
	  _delay_ms(1);
        */
		   
        }
}


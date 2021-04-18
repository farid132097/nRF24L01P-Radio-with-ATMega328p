
//TX 0x11

#include <avr/io.h>
#include <util/delay.h>
#include "nRF24L01P.h"


int main(void){

RF_START(2); // set channel number

while(1){
           
           RF_TX_TEXT("nRF24L01+ Says Hello!");   
           _delay_ms(500);
		   
        }
}

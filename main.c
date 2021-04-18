
//TX 0x11

#include <avr/io.h>
#include <util/delay.h>
#include "nRF24L01P.h"


int main(void){

RF_START(2); // set channel number

while(1){
           
           RF_WAKE_UP(); //optional if you don't care power consumption
           RF_TX_TEXT("nRF24L01+ UD");
	   RF_SLEEP();   //optional if you don't care power consumption
		   
           _delay_ms(500);
		   
        }
}

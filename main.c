
//TX 0x11

#include <avr/io.h>
#include <util/delay.h>
#include "nRF24L01P.h"


int main(void){

RF_START(2);

while(1){
           
           RF_WAKE_UP();
		   RF_TX_TEXT("nRF24L01+ UD");
		   RF_SLEEP();
		   
		   _delay_ms(500);
		   
		   }
}
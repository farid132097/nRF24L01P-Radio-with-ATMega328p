

/*

Default PIN connections:
NRF SCK  -> ATmega328P PB5 (Hardware SPI pin, can't be changed)
NRF MISO -> ATmega328P PB4 (Hardware SPI pin, can't be changed)
NRF MOSI -> ATmega328P PB3 (Hardware SPI pin, can't be changed)
NRF CSN  -> ATmega328P PD6 (Configureable, see below definitions)
NRF CE   -> ATmega328P PD5 (Configureable, see below definitions)

--------------------------------------------------------------------------------------------------------
|    RF FUNCTIONS USED:                                                                                |
--------------------------------------------------------------------------------------------------------
void    RF_START(set_rf_channel) 
void    RF_SLEEP(void)
void    RF_WAKE_UP(void)
void    RF_TX_TEXT(char_buffer)
uint8_t RF_TX(tx_buf, set_tx_data_len, set_rx_addr)
uint8_t RF_RX(rx_buf, get_rx_data_len, rx_timeout, rx_buf_flush)
uint8_t RF_TX_ACK(tx_buf, set_tx_data_len, set_rx_addr, tx_retry)
uint8_t RF_RX_ACK(rx_buf, get_rx_data_len)
uint8_t RF_TX_GET_ACK_PACKET(tx_buf, set_tx_data_len, rx_buf, get_rx_data_len, set_rx_addr, tx_retry)
uint8_t RF_RX_GET_ACK_PACKET(rx_buf, get_rx_data_len, tx_buf, set_tx_data_len)

-------------------------------------------------------------------------------------------------------
|    Function  Prototypes Used:                                                                       |
-------------------------------------------------------------------------------------------------------
|  type   |       name         |         comment                                                      |
-------------------------------------------------------------------------------------------------------
uint8_t      set_rf_channel       single byte (max value 127)
char         char_buffer          character type array
uint8_t      tx_buf               uint8_t type array (must be 32 bytes long)
uint8_t      rx_buf               uint8_t type array (must be 32 bytes long)
uint8_t      set_tx_data_len      single byte (max value 26)
*uint8_t     get_rx_data_len      single byte (use &variable)
uint8_t      set_rx_addr          device address which will receive data; single byte (max value 255)
uint16_t     rx_timeout           set timeout in ms (max value 6500)
uint16_t     rx_buf_flush         boolean input (to flush every time before checking new data set it 1)
uint8_t      tx_retry             max retries when no acknowledgement received (max value 255)
*/



#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>



#define  RF_CSN_DDR        DDRD   /* Change CSN Data direction if necessary */
#define  RF_CSN_PORT       PORTD  /* Change CSN port if necessary */
#define  RF_CSN            6      /* Change CSN pin number if necessary */

#define  RF_CE_DDR         DDRD   /* Change CE Data direction if necessary */
#define  RF_CE_PORT        PORTD  /* Change CE port if necessary */
#define  RF_CE             5      /* Change CE pin number if necessary */

#define  RF_OWN_ADDR       0x11   /* Change Node Address */



#define  RF_SCK_DDR        DDRB   /* Do not change */
#define  RF_SCK_PORT       PORTB  /* Do not change */
#define  RF_SCK            5      /* Do not change */

#define  RF_MISO_DDR       DDRB   /* Do not change */
#define  RF_MISO_PORT      PORTB  /* Do not change */
#define  RF_MISO           4      /* Do not change */

#define  RF_MOSI_DDR       DDRB   /* Do not change */
#define  RF_MOSI_PORT      PORTB  /* Do not change */
#define  RF_MOSI           3      /* Do not change */

#define  MCU_SS_DDR        DDRB   /* Do not change */
#define  MCU_SS_PORT       PORTB  /* Do not change */
#define  MCU_SS            2      /* Do not change */

#define  RF_GENERAL_CALL   0x00   /* Do not change */
#define  RF_ACK_WAIT_MS    10     /* Do not change */
#define  RF_REG_READ       0x01   /* Do not change */
#define  RF_REG_WRITE      0x00   /* Do not change */
#define  RF_MODE_PWR_DOWN  0x02   /* Do not change */
#define  RF_MODE_RX        0x01   /* Do not change */
#define  RF_MODE_TX        0x00   /* Do not change */
#define  CRC_LSBYTE_POS    31     /* Do not change */
#define  CRC_MSBYTE_POS    30     /* Do not change */
#define  LEN_BYTE_POS      29     /* Do not change */
#define  RX_ADDR_BYTE_POS  28     /* Do not change */
#define  OWN_ADDR_BYTE_POS 27     /* Do not change */

#define  RF_CSN_LOW()      RF_CSN_PORT&=~(1<<RF_CSN)
#define  RF_CSN_HIGH()     RF_CSN_PORT|=(1<<RF_CSN)
#define  RF_CE_LOW()       RF_CE_PORT&=~(1<<RF_CE)
#define  RF_CE_HIGH()      RF_CE_PORT|=(1<<RF_CE)

#define  RF_Enable()       RF_SCK_DDR|=(1<<RF_SCK);RF_MISO_DDR&=~(1<<RF_MISO);\
                           RF_MOSI_DDR|=(1<<RF_MOSI);MCU_SS_DDR|=(1<<MCU_SS);\
                           RF_CSN_DDR|=(1<<RF_CSN);RF_CE_DDR|=(1<<RF_CE);\
			               RF_CSN_PORT|=(1<<RF_CSN);RF_CE_PORT&=~(1<<RF_CE);\
			               SPCR=(1<<SPE)|(1<<MSTR);SPSR=(1<<SPI2X);

#define  RF_Disable()      SPCR=0x00;RF_SCK_DDR|=(1<<RF_SCK);\
                           RF_MISO_DDR|=(1<<RF_MISO);RF_MOSI_DDR|=(1<<RF_MOSI);\
			               MCU_SS_DDR|=(1<<MCU_SS);RF_CSN_DDR|=(1<<RF_CSN);\
			               RF_CE_DDR|=(1<<RF_CE);\
			               RF_SCK_PORT&=~(1<<RF_SCK);RF_MISO_PORT&=~(1<<RF_MISO);\
			               RF_MOSI_PORT&=~(1<<RF_MOSI);MCU_SS_PORT&=~(1<<MCU_SS);\
                           RF_CSN_PORT&=~(1<<RF_CSN);RF_CE_PORT&=~(1<<RF_CE);\

typedef struct{
uint8_t tpid;
uint8_t rpid;
}rf_params;

rf_params rf;


uint8_t SPI_TRX(uint8_t data){
SPDR = data;
uint16_t ticks=0;
while(!(SPSR & (1 << SPIF))){_delay_us(1);ticks++;if(ticks>5000){break;}}
return SPDR;
}


uint16_t CRC16(uint16_t crc, uint8_t data){
crc=crc^((uint16_t)data<<8);
for(uint8_t i=0;i<8;i++){
  if(crc & 0x8000){crc=(crc<<1)^0x1021;}
  else{crc<<=1;}
  }
return crc;
}

void RF_RW_REG(uint8_t reg, uint8_t rw, uint8_t *data, uint8_t len){
RF_CSN_LOW();
if(rw==0){ reg|=0x20; SPI_TRX(reg);for(uint8_t i=0;i<len;i++){SPI_TRX(data[i]);}}
else     { SPI_TRX(reg);for(uint8_t i=0;i<len;i++){data[i]=SPI_TRX(0xFF);}}
RF_CSN_HIGH();
}

void RF_PWR(uint8_t state){
uint8_t buf[2];
if     (state==RF_MODE_PWR_DOWN){buf[0]=0x00;}
else if(state==RF_MODE_RX){buf[0]=0x73;RF_CE_HIGH();}
else if(state==RF_MODE_TX){buf[0]=0x72;RF_CE_LOW();}
RF_RW_REG(0x00,RF_REG_WRITE,buf,1);
}

void RF_SLEEP(void){
RF_PWR(RF_MODE_PWR_DOWN);
RF_Disable();
}

void RF_WAKE_UP(void){
RF_Enable();
RF_PWR(RF_MODE_RX);
}

void RF_START(uint8_t channel){
rf.tpid=0;
rf.rpid=0;
RF_Enable();
uint8_t rf_config[20]={0x00,0x00,0x03,0x01,0x00,channel,0x26,0x70,0x20,0x20,
                       0x00,0x00,0x00,0x00,0x00,0x00,'A','A',0xE1,0xE2};
uint8_t index=0,bytes=1,buf[5];
buf[1]='C';buf[2]='K';
for(uint8_t addr=0;addr<=0xE2;addr++){
  if     (addr==0x08){addr=0x11;}
  else if(addr==0x17){addr=0x1C;}
  else if(addr==0x1E){addr=0x0A;bytes=5;}
  else if(addr==0x0B){addr=0x10;bytes=5;}
  else if(addr==0x11){addr=0xE1;bytes=0;}
  buf[0]=rf_config[index];
  RF_RW_REG(addr,RF_REG_WRITE,buf,bytes);
  index++;
 }
RF_PWR(RF_MODE_RX);
}



uint8_t RF_TX(uint8_t *buf, uint8_t len, uint8_t rx_addr){
RF_PWR(RF_MODE_TX);
uint8_t  temp[32],temp_len=(len & 0x1F); uint16_t crc=0;
RF_RW_REG(0xE1,RF_REG_WRITE,temp,0);
for(uint8_t i=0;i<temp_len;i++){temp[i]=buf[i];}
temp[OWN_ADDR_BYTE_POS]=RF_OWN_ADDR;
temp[RX_ADDR_BYTE_POS]=rx_addr;temp[LEN_BYTE_POS]=len;
for(uint8_t i=0;i<30;i++){crc=CRC16(crc,temp[i]);}
temp[CRC_MSBYTE_POS]=(crc>>8);
temp[CRC_LSBYTE_POS]=crc;
RF_RW_REG(0xA0,RF_REG_WRITE,temp,32);
RF_CE_HIGH();
temp[0]=0;
while(!(temp[0]&(1<<4))){RF_RW_REG(0x17,RF_REG_READ,temp,1);_delay_us(100);}
RF_CE_LOW();
return crc;
}


uint8_t RF_RX(uint8_t *buf, uint8_t *len, uint16_t timeout,uint8_t flush){
RF_PWR(RF_MODE_RX);
uint8_t dummy[2];
if(flush){RF_RW_REG(0xE2,RF_REG_WRITE,dummy,0);}
uint16_t ticks=0,sts=0;

while(ticks<timeout){
sts=0; uint8_t temp[2]; uint16_t crc=0;
RF_RW_REG(0x17,RF_REG_READ,temp,1);
if(!(temp[0] & 0x01))
    {
       RF_RW_REG(0x61,RF_REG_READ,buf,32);
       for(uint8_t i=0;i<30;i++){crc=CRC16(crc,buf[i]);}
	   uint16_t calc_crc=buf[CRC_MSBYTE_POS];
	   calc_crc=calc_crc<<8;
	   calc_crc|=buf[CRC_LSBYTE_POS];
       if(crc==calc_crc){*len=(buf[LEN_BYTE_POS] & 0x1F);sts=1;break;}
    }
  _delay_us(85);
  ticks++;
 }
return sts;
}


uint8_t RF_TX_ACK(uint8_t *tbuf, uint8_t tlen, uint8_t rx_addr,uint8_t ack_req, uint8_t retry){
rf.tpid++;
if(rf.tpid>7){rf.tpid=0;}
uint8_t sts=0,rty=0,temp_len=0,temp_pid=(rf.tpid<<5),rbuf[32];

while(rty<retry)
 {
    RF_TX(tbuf,tlen|temp_pid,rx_addr);
    if(RF_RX(rbuf,&temp_len,(RF_ACK_WAIT_MS*10),0))
     {
        if((rbuf[RX_ADDR_BYTE_POS]==RF_OWN_ADDR)&&(rf.tpid==(rbuf[LEN_BYTE_POS]>>5)))
		 {
	        sts=1;
	        break;
	     }
     }
    else{rty++;}
  }
return sts;
}

uint8_t RF_RX_ACK(uint8_t *rbuf, uint8_t *rlen){
uint8_t sts=0,temp_len=0,temp_pid=0,tbuf[32],tlen=0;
if(RF_RX(rbuf,&temp_len,1,0))
 {
    if((rbuf[RX_ADDR_BYTE_POS]==RF_OWN_ADDR)||(rbuf[RX_ADDR_BYTE_POS]==RF_GENERAL_CALL))
	 {
        _delay_us(500);
        temp_pid=(rbuf[LEN_BYTE_POS] & 0xE0)>>5;
	    *rlen=temp_len;
	    RF_TX(tbuf,tlen|(temp_pid<<5),rbuf[OWN_ADDR_BYTE_POS]);
	    if(temp_pid!=rf.rpid){sts=1;}
	    rf.rpid=temp_pid;
	 }
 }
return sts;
}



uint8_t RF_TX_GET_ACK_PACKET(uint8_t *tbuf, uint8_t tlen, uint8_t *rbuf, uint8_t *rlen, uint8_t rx_addr,uint8_t retry){
rf.tpid++;
if(rf.tpid>7){rf.tpid=0;}
uint8_t sts=0,rty=0,temp_len=0,temp_pid=(rf.tpid<<5);
while(rty<retry){
  RF_TX(tbuf,tlen|temp_pid,rx_addr);
  if(RF_RX(rbuf,&temp_len,(RF_ACK_WAIT_MS*10),0)){
    if((rbuf[RX_ADDR_BYTE_POS]==RF_OWN_ADDR)&&(rf.tpid==(rbuf[LEN_BYTE_POS]>>5))){
	   *rlen=temp_len;
	   sts=1;
	   break;
	   }
   }
  else{rty++;}
  }
return sts;
}


uint8_t RF_RX_GET_ACK_PACKET(uint8_t *rbuf, uint8_t *rlen, uint8_t *tbuf, uint8_t tlen){
uint8_t sts=0,temp_len=0,temp_pid=0;
if(RF_RX(rbuf,&temp_len,1,0)){
   if((rbuf[RX_ADDR_BYTE_POS]==RF_OWN_ADDR)||(rbuf[RX_ADDR_BYTE_POS]==RF_GENERAL_CALL)){
     _delay_us(500);
     temp_pid=(rbuf[LEN_BYTE_POS] & 0xE0)>>5;
	 *rlen=temp_len;
	 RF_TX(tbuf,tlen|(temp_pid<<5),rbuf[OWN_ADDR_BYTE_POS]);
	 if(temp_pid!=rf.rpid){sts=1;}
	 rf.rpid=temp_pid;
	 }
   }
return sts;
}


void RF_TX_TEXT(char *text){
uint8_t slen=strlen(text),buf[26];
if(slen>25){slen=26;}
for(uint8_t i=0;i<slen;i++){buf[i]=text[i];}
RF_TX(buf, slen, 0xFF);
}

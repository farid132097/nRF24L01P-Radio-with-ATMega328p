

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
uint8_t RF_RX(rx_buf, get_rx_data_len)
uint8_t RF_RX_TIMEOUT(rx_buf, get_rx_data_len, rx_timeout, rx_buf_flush)
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



#define  CSN_DDR           DDRD   /* Change CSN Data direction if necessary */
#define  CSN_PORT          PORTD  /* Change CSN port if necessary */
#define  CSN               6      /* Change CSN pin number if necessary */

#define  CE_DDR            DDRD   /* Change CE Data direction if necessary */
#define  CE_PORT           PORTD  /* Change CE port if necessary */
#define  CE                5      /* Change CE pin number if necessary */

#define  OWN_ADDR          0x11   /* Change Node Address */



#define  SCK_DDR           DDRB   /* Do not change */
#define  SCK_PORT          PORTB  /* Do not change */
#define  SCK               5      /* Do not change */

#define  MISO_DDR          DDRB   /* Do not change */
#define  MISO_PORT         PORTB  /* Do not change */
#define  MISO              4      /* Do not change */

#define  MOSI_DDR          DDRB   /* Do not change */
#define  MOSI_PORT         PORTB  /* Do not change */
#define  MOSI              3      /* Do not change */

#define  MCU_SS_DDR        DDRB   /* Do not change */
#define  MCU_SS_PORT       PORTB  /* Do not change */
#define  MCU_SS            2      /* Do not change */

#define  GENERAL_CALL      0x00   /* Do not change */
#define  ACK_WAIT_MS       10     /* Do not change */
#define  READ              0x01   /* Do not change */
#define  WRITE             0x00   /* Do not change */
#define  DOWN              0x02   /* Do not change */
#define  RX                0x01   /* Do not change */
#define  TX                0x00   /* Do not change */
#define  NA                0x00   /* Do not change */
#define  CRC_LSBYTE_POS    31     /* Do not change */
#define  CRC_MSBYTE_POS    30     /* Do not change */
#define  LEN_BYTE_POS      29     /* Do not change */
#define  RX_ADDR_BYTE_POS  28     /* Do not change */
#define  OWN_ADDR_BYTE_POS 27     /* Do not change */

#define  CSN_LOW()         CSN_PORT&=~(1<<CSN)
#define  CSN_HIGH()        CSN_PORT|=(1<<CSN)
#define  CE_LOW()          CE_PORT&=~(1<<CE)
#define  CE_HIGH()         CE_PORT|=(1<<CE)

#define  RF_Enable()       SCK_DDR|=(1<<SCK);MISO_DDR&=~(1<<MISO);\
                           MOSI_DDR|=(1<<MOSI);MCU_SS_DDR|=(1<<MCU_SS);\
                           CSN_DDR|=(1<<CSN);CE_DDR|=(1<<CE);\
			               CSN_PORT|=(1<<CSN);CE_PORT&=~(1<<CE);\
			               SPCR=(1<<SPE)|(1<<MSTR);SPSR=(1<<SPI2X);

#define  RF_Disable()      SPCR=0x00;SCK_DDR|=(1<<SCK);\
                           MISO_DDR|=(1<<MISO);MOSI_DDR|=(1<<MOSI);\
			               MCU_SS_DDR|=(1<<MCU_SS);CSN_DDR|=(1<<CSN);\
			               CE_DDR|=(1<<CE);\
			               SCK_PORT&=~(1<<SCK);MISO_PORT&=~(1<<MISO);\
			               MOSI_PORT&=~(1<<MOSI);MCU_SS_PORT&=~(1<<MCU_SS);\
                           CSN_PORT&=~(1<<CSN);CE_PORT&=~(1<<CE);\

typedef struct{
uint8_t tpid;
uint8_t rpid;
}rf_params;

rf_params rf;


uint8_t SPI_TRX(uint8_t data){
SPDR = data;
while(!(SPSR & (1 << SPIF)));
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
CSN_LOW();
if(rw==0){ reg|=0x20; SPI_TRX(reg);for(uint8_t i=0;i<len;i++){SPI_TRX(data[i]);}}
else     { SPI_TRX(reg);for(uint8_t i=0;i<len;i++){data[i]=SPI_TRX(0xFF);}}
CSN_HIGH();
}

void RF_PWR(uint8_t state){
uint8_t buf[2];
if     (state==DOWN){buf[0]=0x00;}
else if(state==RX){buf[0]=0x73;CE_PORT|=(1<<CE);}
else if(state==TX){buf[0]=0x72;CE_PORT&=~(1<<CE);}
RF_RW_REG(0x00,WRITE,buf,1);
}

void RF_SLEEP(void){
RF_PWR(DOWN);
RF_Disable();
}

void RF_WAKE_UP(void){
RF_Enable();
RF_PWR(RX);
}

void RF_START(uint8_t chnl){
rf.tpid=0;
rf.rpid=0;
RF_Enable();
uint8_t buf[5];
buf[0]=0x00;  RF_RW_REG(0x00,WRITE,buf,1);
buf[0]=0x00;  RF_RW_REG(0x01,WRITE,buf,1);
buf[0]=0x03;  RF_RW_REG(0x02,WRITE,buf,1);
buf[0]=0x01;  RF_RW_REG(0x03,WRITE,buf,1);
buf[0]=0x00;  RF_RW_REG(0x04,WRITE,buf,1);
buf[0]=chnl;  RF_RW_REG(0x05,WRITE,buf,1);
buf[0]=0x26;  RF_RW_REG(0x06,WRITE,buf,1);
buf[0]=0x70;  RF_RW_REG(0x07,WRITE,buf,1);
buf[0]=32;    RF_RW_REG(0x11,WRITE,buf,1);
buf[0]=32;    RF_RW_REG(0x12,WRITE,buf,1); 
buf[0]=0x00;  RF_RW_REG(0x13,WRITE,buf,1);
buf[0]=0x00;  RF_RW_REG(0x14,WRITE,buf,1);
buf[0]=0x00;  RF_RW_REG(0x15,WRITE,buf,1);
buf[0]=0x00;  RF_RW_REG(0x16,WRITE,buf,1);
buf[0]=0x00;  RF_RW_REG(0x1C,WRITE,buf,1);
buf[0]=0x00;  RF_RW_REG(0x1D,WRITE,buf,1);
RF_RW_REG(0x10,WRITE,(uint8_t*)"ACK",5);   
RF_RW_REG(0x0A,WRITE,(uint8_t*)"ACK",5);
RF_RW_REG(0x0B,WRITE,(uint8_t*)"DP1",5);
RF_RW_REG(0xE1,WRITE,buf,0);
RF_RW_REG(0xE2,WRITE,buf,0);
_delay_ms(20);
RF_PWR(DOWN);
RF_PWR(RX);
}



uint8_t RF_TX(uint8_t *buf, uint8_t len, uint8_t rx_addr){
RF_PWR(TX);
uint8_t  temp[32],temp_len=(len & 0x1F); uint16_t crc=0;
RF_RW_REG(0xE1,WRITE,temp,0);
for(uint8_t i=0;i<32;i++){temp[i]=' ';}
for(uint8_t i=0;i<temp_len;i++){temp[i]=buf[i];}
temp[OWN_ADDR_BYTE_POS]=OWN_ADDR;
temp[RX_ADDR_BYTE_POS]=rx_addr;temp[LEN_BYTE_POS]=len;
for(uint8_t i=0;i<30;i++){crc=CRC16(crc,temp[i]);}
temp[CRC_MSBYTE_POS]=(crc>>8);
temp[CRC_LSBYTE_POS]=crc;
RF_RW_REG(0xA0,WRITE,temp,32);
CE_HIGH();
temp[0]=0;
while(!(temp[0]&(1<<4))){RF_RW_REG(0x17,READ,temp,1);_delay_us(100);}
CE_LOW();
return crc;
}


uint8_t RF_RX(uint8_t *buf, uint8_t *len){
RF_PWR(RX);
_delay_us(200);
uint8_t  sts=0,temp[2]; uint16_t crc=0;
RF_RW_REG(0x07,READ,temp,1);
uint8_t data_pipe=(temp[0] & 0x0E)>>1;
if(data_pipe<6){
  RF_RW_REG(0x17,READ,temp,1);
  if(!(temp[0] & 0x01)){
    RF_RW_REG(0x11+data_pipe,READ,temp,1);
    RF_RW_REG(0x61,READ,buf,temp[0]);
    for(uint8_t i=0;i<30;i++){crc=CRC16(crc,buf[i]);}
	uint16_t calc_crc=buf[CRC_MSBYTE_POS];
	calc_crc=calc_crc<<8;
	calc_crc|=buf[CRC_LSBYTE_POS];
    if(crc==calc_crc){*len=(buf[LEN_BYTE_POS] & 0x1F);sts=1;}
    }
  }
return sts;
}


uint8_t RF_RX_TIMEOUT(uint8_t *buf, uint8_t *len, uint16_t timeout, uint8_t flush){
RF_PWR(RX);
uint8_t dummy[2];
if(flush){RF_RW_REG(0xE2,WRITE,dummy,0);}
uint16_t ticks=0,sts=0,tmout=timeout*10;

while(ticks<tmout){
 sts=0; uint8_t temp[2]; uint16_t crc=0;
 RF_RW_REG(0x07,READ,temp,1);
 uint8_t data_pipe=(temp[0] & 0x0E)>>1;
 if(data_pipe<6)
 {
    RF_RW_REG(0x17,READ,temp,1);
    if(!(temp[0] & 0x01))
    {
       RF_RW_REG(0x11+data_pipe,READ,temp,1);
       RF_RW_REG(0x61,READ,buf,temp[0]);
       for(uint8_t i=0;i<30;i++){crc=CRC16(crc,buf[i]);}
	   uint16_t calc_crc=buf[CRC_MSBYTE_POS];
	   calc_crc=calc_crc<<8;
	   calc_crc|=buf[CRC_LSBYTE_POS];
       if(crc==calc_crc){*len=(buf[LEN_BYTE_POS] & 0x1F);sts=1;break;}
    }
  }
  _delay_us(85);
  ticks++;
 }
return sts;
}

uint8_t RF_TX_ACK(uint8_t *tbuf, uint8_t tlen, uint8_t rx_addr,uint8_t retry){
rf.tpid++;
if(rf.tpid>7){rf.tpid=0;}
uint8_t sts=0,rty=0,temp_len=0,temp_pid=(rf.tpid<<5),rbuf[32];
while(rty<retry){
  RF_TX(tbuf,tlen|temp_pid,rx_addr);
  if(RF_RX_TIMEOUT(rbuf,&temp_len,ACK_WAIT_MS,0)){
    if((rbuf[RX_ADDR_BYTE_POS]==OWN_ADDR)&&(rf.tpid==(rbuf[LEN_BYTE_POS]>>5))){
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
if(RF_RX(rbuf,&temp_len)){
   if((rbuf[RX_ADDR_BYTE_POS]==OWN_ADDR)||(rbuf[RX_ADDR_BYTE_POS]==GENERAL_CALL)){
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
  if(RF_RX_TIMEOUT(rbuf,&temp_len,ACK_WAIT_MS,0)){
    if((rbuf[RX_ADDR_BYTE_POS]==OWN_ADDR)&&(rf.tpid==(rbuf[LEN_BYTE_POS]>>5))){
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
if(RF_RX(rbuf,&temp_len)){
   if((rbuf[RX_ADDR_BYTE_POS]==OWN_ADDR)||(rbuf[RX_ADDR_BYTE_POS]==GENERAL_CALL)){
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

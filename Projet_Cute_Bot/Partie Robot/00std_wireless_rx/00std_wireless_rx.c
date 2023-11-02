#include <stdio.h>

#include <nrf.h>
#include "nrf52833.h"

static uint8_t pdu[8+1] = { 0 };


#define MOTOR_SPEED 50 // [0...100]
uint8_t I2CBUF_MOTOR_LEFT_FWD[]   = {0x99,0x01,0x01,0x01,MOTOR_SPEED,0x00,0x88};
uint8_t I2CBUF_MOTOR_LEFT_BACK[]  = {0x99,0x01,0x01,0x00,MOTOR_SPEED,0x00,0x88};

uint8_t I2CBUF_MOTOR_RIGHT_FWD[]  = {0x99,0x01,0x02,0x01,MOTOR_SPEED,0x00,0x88};
uint8_t I2CBUF_MOTOR_RIGHT_BACK[] = {0x99,0x01,0x02,0x00,MOTOR_SPEED,0x00,0x88};

uint8_t I2CBUF_MOTORS_STOP[]      = {0x99,0x09,0x03,0x00,0x00,0x00,0x88};

uint8_t I2CBUF_TURN_LEFT_FWD[]  = {0x99,0x01,0x01,0x01,20,0x00,0x88};
uint8_t I2CBUF_TURN_RIGHT_FWD[]  = {0x99,0x01,0x02,0x01,20,0x00,0x88};




#define LED_INTENSITY 0xff // [0x00...0xff]
uint8_t I2CBUF_LED_LEFT_WHITE[]   = {0x99,0x0f,0x02,LED_INTENSITY,LED_INTENSITY,LED_INTENSITY,0x88};
uint8_t I2CBUF_LED_LEFT_RED[]     = {0x99,0x0f,0x02,LED_INTENSITY,         0x00,         0x00,0x88};
uint8_t I2CBUF_LED_LEFT_GREEN[]   = {0x99,0x0f,0x02,         0x00,LED_INTENSITY,         0x00,0x88};
uint8_t I2CBUF_LED_LEFT_BLUE[]    = {0x99,0x0f,0x02,         0x00,         0x00,LED_INTENSITY,0x88};
uint8_t I2CBUF_LED_LEFT_OFF[]     = {0x99,0x0f,0x02,         0x00,         0x00,         0x00,0x88};
uint8_t I2CBUF_LED_RIGHT_WHITE[]  = {0x99,0x0f,0x01,LED_INTENSITY,LED_INTENSITY,LED_INTENSITY,0x88};
uint8_t I2CBUF_LED_RIGHT_RED[]    = {0x99,0x0f,0x01,LED_INTENSITY,         0x00,         0x00,0x88};
uint8_t I2CBUF_LED_RIGHT_GREEN[]  = {0x99,0x0f,0x01,         0x00,LED_INTENSITY,         0x00,0x88};
uint8_t I2CBUF_LED_RIGHT_BLUE[]   = {0x99,0x0f,0x01,         0x00,         0x00,LED_INTENSITY,0x88};
uint8_t I2CBUF_LED_RIGHT_OFF[]    = {0x99,0x0f,0x01,         0x00,         0x00,         0x00,0x88};
uint8_t I2CBUF_LED_LEFT_ORANGE[]   = {0x99,0x0f,0x02,LED_INTENSITY,0xa5,0x00,0x88};
uint8_t I2CBUF_LED_RIGHT_ORANGE[]   = {0x99,0x0f,0x01,LED_INTENSITY,0xa5,0x00,0x88};


void i2c_init(void) {
   //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:   0=Input
    // .... .... .... .... .... .... .... ..B. B: INPUT: 1=Disconnect
    // .... .... .... .... .... .... .... CC.. C: PULL:  0=Disabled
    // .... .... .... .... .... .DDD .... .... D: DRIVE: 6=S0D1
    // .... .... .... ..EE .... .... .... .... E: SENSE: 0=Disabled
    // xxxx xxxx xxxx xx00 xxxx x110 xxxx 0010 
    //    0    0    0    0    0    6    0    2 0x00000602
    NRF_P0->PIN_CNF[26]           = 0x00000602; // SCL (P0.26)
    NRF_P1->PIN_CNF[0]            = 0x00000602; // SDA (P1.00)

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... AAAA A: ENABLE: 5=Enabled
    // xxxx xxxx xxxx xxxx xxxx xxxx xxxx 0101 
    //    0    0    0    0    0    0    0    5 0x00000005
    NRF_TWI0->ENABLE              = 0x00000005;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA A: PIN:    26 (P0.26)
    // .... .... .... .... .... .... ..B. .... B: PORT:    0 (P0.26)
    // C... .... .... .... .... .... .... .... C: CONNECT: 0=Connected
    // 0xxx xxxx xxxx xxxx xxxx xxxx xx01 1010 
    //    0    0    0    0    0    0    1    a 0x0000001a
    NRF_TWI0->PSEL.SCL            = 0x0000001a;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA A: PIN:    00 (P1.00)
    // .... .... .... .... .... .... ..B. .... B: PORT:    1 (P1.00)
    // C... .... .... .... .... .... .... .... C: CONNECT: 0=Connected
    // 0xxx xxxx xxxx xxxx xxxx xxxx xx10 0000 
    //    0    0    0    0    0    0    2    0 0x00000020
    NRF_TWI0->PSEL.SDA            = 0x00000020;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // AAAA AAAA AAAA AAAA AAAA AAAA AAAA AAAA A: FREQUENCY: 0x01980000==K100==100 kbps
    NRF_TWI0->FREQUENCY           = 0x01980000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .AAA AAAA A: ADDRESS: 16
    // xxxx xxxx xxxx xxxx xxxx xxxx x001 0000 
    //    0    0    0    0    0    0    1    0 0x00000010
    NRF_TWI0->ADDRESS             = 0x10;
}

void i2c_send(uint8_t* buf, uint8_t buflen) {
    uint8_t i;

    i=0;
    NRF_TWI0->TXD                 = buf[i];
    NRF_TWI0->EVENTS_TXDSENT      = 0;
    NRF_TWI0->TASKS_STARTTX       = 1;
    i++;
    while(i<buflen) {
        while(NRF_TWI0->EVENTS_TXDSENT==0);
        NRF_TWI0->EVENTS_TXDSENT  = 0;
        NRF_TWI0->TXD             = buf[i];
        i++;
    }
    while(NRF_TWI0->EVENTS_TXDSENT==0);
    NRF_TWI0->TASKS_STOP     = 1;
}

void wait() {
    volatile uint32_t a;
    for (a=0; a<1000000; a++);
}

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;

        if (NRF_RADIO->CRCSTATUS != RADIO_CRCSTATUS_CRCSTATUS_CRCOk) {
            puts("Invalid CRC");
        } else {
               switch (pdu[2]) {


              case 0x01 : 
                   i2c_send(I2CBUF_MOTOR_LEFT_FWD,    sizeof(I2CBUF_MOTOR_LEFT_FWD));
                   i2c_send(I2CBUF_MOTOR_RIGHT_FWD,   sizeof(I2CBUF_MOTOR_RIGHT_FWD));
                   break;

                 
              case 0x02 :
                 i2c_send(I2CBUF_MOTOR_LEFT_BACK,   sizeof(I2CBUF_MOTOR_LEFT_BACK));
                 i2c_send(I2CBUF_MOTOR_RIGHT_BACK,  sizeof(I2CBUF_MOTOR_RIGHT_BACK));
                 break;


              case 0x03 : 
                i2c_send(I2CBUF_MOTORS_STOP,       sizeof(I2CBUF_MOTORS_STOP));
                 break;

              case 0x04 : 
                i2c_send(I2CBUF_LED_LEFT_WHITE,      sizeof(I2CBUF_LED_LEFT_WHITE));
                i2c_send(I2CBUF_LED_RIGHT_WHITE,     sizeof(I2CBUF_LED_RIGHT_WHITE));
                break;

              case 0x05 : 
                i2c_send(I2CBUF_LED_LEFT_OFF,      sizeof(I2CBUF_LED_LEFT_OFF));
                i2c_send(I2CBUF_LED_RIGHT_OFF,     sizeof(I2CBUF_LED_RIGHT_OFF));
                break;

              case 0x06 :
                i2c_send(I2CBUF_LED_RIGHT_OFF,      sizeof(I2CBUF_LED_RIGHT_OFF));
                i2c_send(I2CBUF_LED_LEFT_OFF,      sizeof(I2CBUF_LED_LEFT_OFF));
                i2c_send(I2CBUF_LED_RIGHT_ORANGE,   sizeof(I2CBUF_LED_RIGHT_ORANGE));
                i2c_send(I2CBUF_MOTOR_LEFT_FWD,    sizeof(I2CBUF_MOTOR_LEFT_FWD));
                i2c_send(I2CBUF_TURN_RIGHT_FWD,    sizeof(I2CBUF_TURN_RIGHT_FWD));

                break;

              
              case 0x09 :
                  i2c_send(I2CBUF_LED_RIGHT_OFF,      sizeof(I2CBUF_LED_RIGHT_OFF));
                  i2c_send(I2CBUF_LED_LEFT_OFF,      sizeof(I2CBUF_LED_LEFT_OFF));
                  i2c_send(I2CBUF_LED_LEFT_ORANGE,    sizeof(I2CBUF_LED_LEFT_ORANGE));
                  i2c_send(I2CBUF_MOTOR_RIGHT_FWD,   sizeof(I2CBUF_MOTOR_RIGHT_FWD));    
                  i2c_send(I2CBUF_TURN_LEFT_FWD,   sizeof(I2CBUF_TURN_LEFT_FWD));
                 break;  

   
          }   
      }
                    
  }
}




// TATATATATTATATATATATTATATATATATATATTATATATATAT
#define NOTE_NONE      0 // 246.94 Hz
#define NOTE_SI_2  16198 // 246.94 Hz
#define NOTE_DO_3  15289 // 261.63 Hz
#define NOTE_RE_3  13621 // 293.66 Hz
#define NOTE_MI_3  12135 // 329.63 Hz
#define NOTE_FA_3  11454 // 349.23 Hz
#define NOTE_SOL_3 10204 // 392.00 Hz
#define NOTE_LA_3   9091 // 440.00 Hz
#define NOTE_SI_3   8099 // 493.88 Hz
#define NOTE_DO_4   7645 // 523.25 Hz

uint16_t song[] = {

    NOTE_RE_3, NOTE_RE_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_MI_3, NOTE_MI_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_SOL_3, NOTE_SOL_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_MI_3, NOTE_MI_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_SI_3, NOTE_SI_3,NOTE_SI_3, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_SI_3, NOTE_SI_3,NOTE_SI_3, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_LA_3, NOTE_LA_3,NOTE_LA_3, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,

    NOTE_RE_3, NOTE_RE_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_MI_3, NOTE_MI_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_SOL_3, NOTE_SOL_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_MI_3, NOTE_MI_3,NOTE_MI_3, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_LA_3, NOTE_LA_3,NOTE_LA_3, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_LA_3, NOTE_LA_3,NOTE_LA_3, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_SOL_3, NOTE_SOL_3,NOTE_SOL_3, NOTE_SOL_3, NOTE_SOL_3, NOTE_SOL_3, NOTE_SOL_3, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,
    
    NOTE_RE_3, NOTE_RE_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_MI_3, NOTE_MI_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_SOL_3, NOTE_SOL_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_MI_3, NOTE_MI_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_SOL_3, NOTE_SOL_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_LA_3, NOTE_FA_3,NOTE_FA_3, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,
    
    NOTE_RE_3, NOTE_RE_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_RE_3, NOTE_RE_3,NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_LA_3, NOTE_LA_3,NOTE_LA_3, NOTE_LA_3, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,NOTE_NONE, NOTE_NONE, NOTE_NONE,
    NOTE_SOL_3, NOTE_SOL_3,NOTE_SOL_3, NOTE_SOL_3, NOTE_SOL_3, NOTE_NONE, NOTE_NONE, NOTE_NONE,

};
uint16_t beat;

uint16_t pwm_comp[1];

void pwm_init(void) {
    
    // configure P0.00 as output
    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:     1=Output
    // .... .... .... .... .... .... .... ..B. B: INPUT:   1=Disconnect
    // .... .... .... .... .... .... .... CC.. C: PULL:    0=Disabled
    // .... .... .... .... .... .DDD .... .... D: DRIVE:   0=S0S1
    // .... .... .... ..EE .... .... .... .... E: SENSE:   0=Disabled
    // xxxx xxxx xxxx xx00 xxxx xxxx xxxx 0011 
    //    0    0    0    0    0    0    0    3 0x00000003
    NRF_P0->PIN_CNF[0]            = 0x00000003;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA A: PIN:      00 (P0.00)
    // .... .... .... .... .... .... ..B. .... B: PORT:     0  (P0.00)
    // C... .... .... .... .... .... .... .... C: CONNECT:  0=Connected
    // 0xxx xxxx xxxx xxxx xxxx xxxx xx00 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->PSEL.OUT[0]         = 0x00000000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: ENABLE:   1=Enabled
    // 0xxx xxxx xxxx xxxx xxxx xxxx xxxx xxx1 
    //    0    0    0    0    0    0    0    1 0x00000001
    NRF_PWM0->ENABLE              = 0x00000001;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: UPDOWN:   0=Up
    // 0xxx xxxx xxxx xxxx xxxx xxxx xxxx xxx0 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->MODE                = 0x00000000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... .AAA A: PRESCALER: 0=DIV_1
    // 0xxx xxxx xxxx xxxx xxxx xxxx xxxx x000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->PRESCALER           = 2;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... AAAA AAAA AAAA AAAA A: CNT:      0=Disabled
    // 0xxx xxxx xxxx xxxx 0000 0000 0000 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->LOOP                = 0x00000000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ..AA A: LOAD:     0=Common
    // .... .... .... .... .... ...B .... .... B: MODE:     0=RefreshCount
    // 0xxx xxxx xxxx xxxx xxxx xxx0 xxxx xx00 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->DECODER             = 0x00000000;
    NRF_PWM0->SEQ[0].PTR          = (uint32_t)(pwm_comp);
    NRF_PWM0->SEQ[0].CNT          = (sizeof(pwm_comp) / sizeof(uint16_t));

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... AAAA AAAA AAAA AAAA AAAA AAAA A: CNT:      0=Continuous
    // 0xxx xxxx 0000 0000 0000 0000 0000 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->SEQ[0].REFRESH      = 0;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... AAAA AAAA AAAA AAAA AAAA AAAA A: CNT
    // 0xxx xxxx 0000 0000 0000 0000 0000 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->SEQ[0].ENDDELAY     = 0;
}

void pwm_setperiod(uint16_t period) {

    if(NRF_PWM0->EVENTS_SEQSTARTED[0]==1) {
        NRF_PWM0->EVENTS_STOPPED  = 0;
        NRF_PWM0->TASKS_STOP      = 0x00000001;
        while(NRF_PWM0->EVENTS_STOPPED==0);
    }

    NRF_PWM0->COUNTERTOP          = period;
    pwm_comp[0]                   = period/2;

    NRF_PWM0->EVENTS_SEQSTARTED[0]=0;
    NRF_PWM0->TASKS_SEQSTART[0]   = 0x00000001;
    while(NRF_PWM0->EVENTS_SEQSTARTED[0]==0);
}

void rtc_init(void) {
    
    // configure/start the RTC
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // xxxx xxxx xxxx FEDC xxxx xxxx xxxx xxBA (C=compare 0)
    // 0000 0000 0000 0001 0000 0000 0000 0000 
    //    0    0    0    1    0    0    0    0 0x00010000
    NRF_RTC0->EVTENSET                 = 0x00010000;       // enable compare 0 event routing
    NRF_RTC0->INTENSET                 = 0x00010000;       // enable compare 0 interrupts

    // enable interrupts
    NVIC_SetPriority(RTC0_IRQn, 1);
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);

    // have RTC tick every second
    NRF_RTC0->CC[0]                    = 800;//1317;             // 32768==1 s --> 1317==40.1ms
    NRF_RTC0->TASKS_START              = 0x00000001;       // start RTC0
}

void RTC0_IRQHandler(void) {
    
    if (NRF_RTC0->EVENTS_COMPARE[0] == 0x00000001 ) {
        // handle compare[0]

        // clear flag
        NRF_RTC0->EVENTS_COMPARE[0]    = 0x00000000;

        // clear COUNTER
        NRF_RTC0->TASKS_CLEAR          = 0x00000001;

        pwm_setperiod(song[beat]);
        beat = (beat+1)%(sizeof(song) / sizeof(uint16_t));
    }
}

// TATATATTATATATATATTATATATA




int main(void) {
    
    i2c_init();
    pwm_init();
    rtc_init();  //c'est pour le son 
    // confiureg HF clock
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

    // configure radio
    NRF_RADIO->MODE          = (  RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER       = (  RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->PCNF0         = (                              8 << RADIO_PCNF0_LFLEN_Pos)          |
                               (                              1 << RADIO_PCNF0_S0LEN_Pos)          |
                               (                              0 << RADIO_PCNF0_S1LEN_Pos)          |
                               (                              2 << RADIO_PCNF0_CILEN_Pos)          |
                               (     RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos)           |
                               (                              3 << RADIO_PCNF0_TERMLEN_Pos);
    NRF_RADIO->PCNF1         = (                    sizeof(pdu) << RADIO_PCNF1_MAXLEN_Pos)         |
                               (                              0 << RADIO_PCNF1_STATLEN_Pos)        |
                               (                              3 << RADIO_PCNF1_BALEN_Pos)          |
                               (      RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos)         |
                               (   RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->BASE0         = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS     = 0UL;
    NRF_RADIO->RXADDRESSES   = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->TIFS          = 0;
    NRF_RADIO->CRCCNF        = (         RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos)           |
                               (     RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT       = 0xFFFFUL;
    NRF_RADIO->CRCPOLY       = 0x00065b; // CRC poly: x^16 + x^12^x^5 + 1
    NRF_RADIO->FREQUENCY     = 40;
    NRF_RADIO->PACKETPTR     = (uint32_t)pdu;

    // receive
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |
                        (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);
    NRF_RADIO->TASKS_RXEN    = 1;

    NRF_RADIO->INTENCLR = 0xffffffff;
    NVIC_EnableIRQ(RADIO_IRQn);
    NRF_RADIO->INTENSET = (RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos);

    while(1) {
        __WFE();
    }
}
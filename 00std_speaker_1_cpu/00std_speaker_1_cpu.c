#include "nrf52833.h"

void wait(volatile uint32_t b) {
    volatile uint32_t a;
    for (a=0; a<b; a++);
}

int main(void) {
    
    // set speaker pin as output
    NRF_P0->PIN_CNF[0] = 0x00000003; // P0.00
    volatile uint32_t b = 8000;
    while(1) {
        NRF_P0->OUTSET = (0x00000001 << 0); // speaker pin high
        wait(b);
        NRF_P0->OUTCLR = (0x00000001 << 0); // speaker pin low
        wait(b);

        if (b==20000) {
          b = 8000;
        }else {
          b = b + 100;
        }
    }
}
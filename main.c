/*
 * WS2812B: Drive a single WS2812B using SPI.
 * Serial interface is included for debugging purposes
 */


#include "stm32f042.h"
#include "serial.h"
#include "spi.h"

// The following are 'declared' in the linker script
extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;

void delay(int);

void delay(int dly) {
    while( dly--);
}

void configPins() {
    RCC_AHBENR |= BIT18; // power up PORTB
    GPIOB_MODER |= BIT6; // make bit3 an output
    GPIOB_MODER &= ~BIT7; // make bit3 an output
}

void writeWS2812B(unsigned long value) {
    // have to expand each bit to 3 bits so that we can then output 110 for WS2812B logic '1' and 100 for WS2812B logic '0'
    uint8_t data[9];

    for (int i = 0; i < 3; i++) {
        int index = 0;
        uint32_t encoding = 0;
        while (index < 8) {
            encoding = encoding << 3;
            if (value & BIT23) {
                encoding |= 0b110;
            } else {
                encoding |= 0b100;
            }
            value = value << 1;
            index++;

            data[3*i + 0] = ((encoding >> 16) & 0xff);
            data[3*i + 1] = ((encoding >> 8) & 0xff);
            data[3*i + 2] = (encoding & 0xff);
        }
    }

    // Now send out the 24 (x3) bits to the SPI bus
    writeSPI(data, 9);

}

unsigned long getRainbow() {   // Cycle through the colours of the rainbow (non-uniform brightness however)
    // Inspired by : http://academe.co.uk/2012/04/arduino-cycling-through-colours-of-the-rainbow/
    static unsigned red = 255;
    static unsigned green = 0;
    static unsigned blue = 0;
    static int state = 0;
    switch (state) {
    case 0:{
        green++;
        if (green == 255)
            state = 1;
        break;
    }
    case 1:{
        red--;
        if (red == 0)
            state = 2;
        break;
    }
    case 2:{
        blue++;
        if (blue == 255)
            state = 3;
        break;
    }
    case 3:{
        green--;
        if (green == 0)
            state = 4;
        break;
    }
    case 4:{
        red++;
        if (red == 255)
            state = 5;
        break;
    }
    case 5:{
        blue--;
        if (blue == 0)
            state = 0;
        break;
    }
    }
    return (green << 16) + (red << 8) + blue;
}


void initClock() {
    if (FLASH_ACR == 0) {
        // This is potentially a dangerous function as it could
        // result in a system with an invalid clock signal - result: a stuck system
        // Set the PLL up
        // First ensure PLL is disabled
        RCC_CR &= ~BIT24;
        while( (RCC_CR & BIT25)); // wait for PLL ready to be cleared
        // set PLL multiplier to 12 (yielding 48MHz)
        // Warning here: if system clock is greater than 24MHz then wait-state(s) need to be
        // inserted into Flash memory interface
        FLASH_ACR |= BIT0;
        FLASH_ACR &=~(BIT2 | BIT1);

        // Turn on FLASH prefetch buffer
        FLASH_ACR |= BIT4;
        RCC_CFGR &= ~(BIT21 | BIT20 | BIT19 | BIT18);
        RCC_CFGR |= (BIT21 | BIT19 );

        // Need to limit ADC clock to below 14MHz so will change ADC prescaler to 4
        RCC_CFGR |= BIT14;

        // Do the following to push HSI clock out on PA8 (MCO)
        // for measurement purposes.  Should be 8MHz or thereabouts (verified with oscilloscope)
        /*
          RCC_CFGR |= ( BIT26 | BIT24 );
          RCC_AHBENR |= BIT17;
          GPIOA_MODER |= BIT17;
        */

        // and turn the PLL back on again
        RCC_CR |= BIT24;
        // set PLL as system clock source
        RCC_CFGR |= BIT1;
    }
}

int main() {
    initClock(); // set clock to 48 MHz, turn on flash prefetch, etc.
    initUART(9600);  // Set serial port to 9600,n,8,1
    configPins(); // Set up the pin to drive the onboard LDE
    initSPI(); // set up the SPI bus


    while(1) {
        GPIOB_ODR |= BIT3;	// LED on
        writeWS2812B(getRainbow()); // Output a colour Format: GGRRBB
        delay(5000); // Wait for a while so we can see it
        GPIOB_ODR &= ~BIT3; // LED off
    }
    return 0;
}

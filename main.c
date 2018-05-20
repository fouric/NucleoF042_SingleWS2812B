/*
 * WS2812B: Drive a single WS2812B using SPI.
 * Serial interface is included for debugging purposes
 */


#include "stm32f042.h"
#include "serial.h"
#include "spi.h"
#include "config.h"

void delay(int);

void delay(int dly) {
    while(dly--);
}

int main() {
    initClock(); // set clock to 48 MHz, turn on flash prefetch, etc.
    configPins(); // Set up the pin to drive the onboard LDE
    spi_init(); // set up the SPI bus

    uint8_t data[2];
    data[0] = 0x55;
    data[1] = 0x54;

    GPIOB_ODR |= BIT3;
    delay(1000000);
    spi_write(data, 2);
    GPIOB_ODR &= ~BIT3;

    /*
    while(1) {
        GPIOB_ODR |= BIT3;	// flame on
        // spi_write(data, count);
        delay(500000);
        GPIOB_ODR &= ~BIT3; // flame off
        delay(500000);
    }
    */
    return 0;
}

#include "stm32f042.h"
#include "config.h"

void configPins() {
    RCC_AHBENR |= BIT18; // power up PORTB
    GPIOB_MODER |= BIT6; // make bit3 an output
    GPIOB_MODER &= ~BIT7; // make bit3 an output
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

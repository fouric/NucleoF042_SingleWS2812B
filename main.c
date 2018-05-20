/*
 * WS2812B: Drive a single WS2812B using SPI.
 * Serial interface is included for debugging purposes
 */


#include "stm32f042.h"
#include "serial.h"
#include "spi.h"
void delay(int);

void delay(int dly) {
  while( dly--);
}

void configPins() {
  // Power up PORTB
  RCC_AHBENR |= BIT18;
  GPIOB_MODER |= BIT6; // make bit3  an output
  GPIOB_MODER &= ~BIT7; // make bit3  an output
}

void writeWS2812B(unsigned long Value) {
  // have to expand each bit to 3 bits
  // Can then output 110 for WS2812B logic '1'
  // and 100 for WS2812B logic '0'
  uint32_t Encoding=0;
  uint8_t SPI_Data[9];
  int Index;

  // Process the GREEN byte
  Index=0;
  Encoding=0;
  while (Index < 8)
    {
      Encoding = Encoding << 3;
      if (Value & BIT23)
        {
          Encoding |= 0b110;
        }
      else
        {
          Encoding |= 0b100;
        }
      Value = Value << 1;
      Index++;

    }
  SPI_Data[0] = ((Encoding >> 16) & 0xff);
  SPI_Data[1] = ((Encoding >> 8) & 0xff);
  SPI_Data[2] = (Encoding & 0xff);

  // Process the RED byte
  Index=0;
  Encoding=0;
  while (Index < 8)
    {
      Encoding = Encoding << 3;
      if (Value & BIT23)
        {
          Encoding |= 0b110;
        }
      else
        {
          Encoding |= 0b100;
        }
      Value = Value << 1;
      Index++;

    }
  SPI_Data[3] = ((Encoding >> 16) & 0xff);
  SPI_Data[4] = ((Encoding >> 8) & 0xff);
  SPI_Data[5] = (Encoding & 0xff);

  // Process the BLUE byte
  Index=0;
  Encoding=0;
  while (Index < 8) {
    Encoding = Encoding << 3;
    if (Value & BIT23) {
      Encoding |= 0b110;
    } else {
      Encoding |= 0b100;
    }
    Value = Value << 1;
    Index++;

  }
  SPI_Data[6] = ((Encoding >> 16) & 0xff);
  SPI_Data[7] = ((Encoding >> 8) & 0xff);
  SPI_Data[8] = (Encoding & 0xff);

  // Now send out the 24 (x3) bits to the SPI bus
  writeSPI(SPI_Data,9);

}

unsigned long getRainbow() {   // Cycle through the colours of the rainbow (non-uniform brightness however)
  // Inspired by : http://academe.co.uk/2012/04/arduino-cycling-through-colours-of-the-rainbow/
  static unsigned Red = 255;
  static unsigned Green = 0;
  static unsigned Blue = 0;
  static int State = 0;
  switch (State) {
  case 0:{
    Green++;
    if (Green == 255)
      State = 1;
    break;
  }
  case 1:{
    Red--;
    if (Red == 0)
      State = 2;
    break;
  }
  case 2:{
    Blue++;
    if (Blue == 255)
      State = 3;
    break;
  }
  case 3:{
    Green--;
    if (Green == 0)
      State = 4;
    break;
  }
  case 4:{
    Red++;
    if (Red == 255)
      State = 5;
    break;
  }
  case 5:{
    Blue --;
    if (Blue == 0)
      State = 0;
    break;
  }
  }
  return (Green << 16) + (Red << 8) + Blue;
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
  //unsigned count=0;

  initClock();

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

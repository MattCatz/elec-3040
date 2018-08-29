/*==============================================================*/
/* Matthew Cather and Jacob Davis                               */
/* ELEC 3040/3050 - Lab 3                                       */
/* Two counters, one going up and one going down.               */
/*==============================================================*/

#include "STM32L1xx.h" /* Microcontroller information */

void delay(void);
void count(unsigned char* counter, unsigned char direction);
void setup_pins(void);

int main() {
  unsigned char status = 0; // start/stop
  unsigned char direction = 0; // inital direction
  unsigned char count_one = 0;
  unsigned char count_two = 0;

  setup_pins();
  
  while(1) {
    status = (GPIOA->IDR & 0x00000001); //PA1
    direction = (GPIOA->IDR & 0x00000002); //PA2;
    if (status) {
      count(&count_one, direction);
      count(&count_two, ~direction);
      update_leds();
    }
    delay();
  }
}

/*----------------------------------------------------------*/
/* Delay function - do nothing for about 1/2 second         */
/*----------------------------------------------------------*/
void delay () {
  int i,j;
  for (i=0; i<10; i++) { //outer loop
    for (j=0; j<20000; j++) { //inner loop
      asm("nop"); //dummy operation for single-step test
    } //do nothing
  }
}

/*---------------------------------------------------*/
/* Incriment or deincriment toggle mod 10 either up  */
/* or down.                                          */
/*---------------------------------------------------*/
void count(unsigned char* counter, unsigned char direction) {
  if (direction) {
    *counter = (*counter + 1) % 10;
  } else {
    *counter = (*counter + (10 - 1)) % 10;
  }  
}

/*---------------------------------------------------*/
/* Update the LEDS used in the lab                   */
/*---------------------------------------------------*/
void update_leds() {
  unsigned short leds = (count_two & 0x0F) << 4; // PC[7:4]
  leds += (count_one & 0x0F); // PC[3:0]
  GPIOC->BSRR |= (~leds) << 16; // clear bits
  GPIOC->BSRR |= leds; // write bits	
}

/*---------------------------------------------------*/
/* Initialize GPIO pins used in the program          */
/*---------------------------------------------------*/
void setup_pins () {
  /* Configure PA1 and PA2 as input pin to read push button */
  RCC->AHBENR |= 0x01; /* Enable GPIOA clock (bit 0) */
  GPIOA->MODER &= ~(0x0000003C); /* General purpose input mode */
  /* Configure PC[0,3] as output pins to drive LEDs */
  RCC->AHBENR |= 0x04; /* Enable GPIOC clock (bit 2) */
  GPIOC->MODER &= ~(0x0000FFFF); /* Clear PC[0,3] mode bits */
  GPIOC->MODER |= (0x00005555); /* General purpose output mode*/
}


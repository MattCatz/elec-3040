/*==============================================================*/
/* Matthew Cather and Jacob Davis                               */
/* ELEC 3040/3050 - Lab 2                                       */
/* Toggle LED1 while button pressed, with short delay inserted  */
/*==============================================================*/

#include "STM32L1xx.h" /* Microcontroller information */

void delay();
void count(unsigned char direction);
void setup_pins();

unsigned char toggle;

int main() {
  unsigned char status = 0; // start/stop
  unsigned char direction = 0; // inital direction
  
  setup_pins();
  
  while(1) {
    status = (GPIOA->IDR & 0x00000001); //PA1
    direction = (GPIOA->IDR & 0x00000010); //PA2;
    if (status == 1) {
      count(direction);
    }
    delay();
  }
}

/*----------------------------------------------------------*/
/* Delay function - do nothing for about 1/2 second */
/*----------------------------------------------------------*/
void delay () {
  int i,j,n;
  for (i=0; i<10; i++) { //outer loop
    for (j=0; j<20000; j++) { //inner loop
      n = j; //dummy operation for single-step test
    } //do nothing
  }
}

/*---------------------------------------------------*/
/* Incriment of deincriment toggle mod 10 */
/* then update the update the LEDs */
/*---------------------------------------------------*/
void count(unsigned char direction) {
  if (direction) {
    toggle = (toggle + 1) % 10;
  } else {
    toggle = (toggle + (10 - 1)) % 10;
  }
  
  GPIOC->BSRR |= (~toggle & 0x0F) < 4; // clear bits
  GPIOC->BSRR |= (toggle & 0x0F); // write bits
}

/*---------------------------------------------------*/
/* Initialize GPIO pins used in the program */
/*---------------------------------------------------*/
void PinSetup () {
  /* Configure PA1 and PA2 as input pin to read push button */
  RCC->AHBENR |= 0x01; /* Enable GPIOA clock (bit 0) */
  GPIOA->MODER &= ~(0x0000003C); /* General purpose input mode */
  /* Configure PC[0,3] as output pins to drive LEDs */
  RCC->AHBENR |= 0x04; /* Enable GPIOC clock (bit 2) */
  GPIOC->MODER &= ~(0x000000FF); /* Clear PC[0,3] mode bits */
  GPIOC->MODER |= (0x00000055); /* General purpose output mode*/
}
/*==============================================================*/
/* Matthew Cather and Jacob Davis                               */
/* ELEC 3040/3050 - Lab 3                                       */
/* Two counters, one going up and one going down.               */
/*==============================================================*/

#include "STM32L1xx.h" /* Microcontroller information */

void delay(void);
void count(unsigned char* counter, unsigned char direction);
void setup_pins(void);

unsigned char count_one = 0;
unsigned char count_two = 0;

int main() {
  unsigned char status = 0; // start/stop
  unsigned char direction = 0; // inital direction


  setup_pins();
  
  while(1) {
    status = READ_BIT(GPIOA->IDR, GPIO_IDR_IDR_1); //PA1
    direction = READ_BIT(GPIOA->IDR, GPIO_IDR_IDR_2); //PA2;
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
  SET_BIT(GPIOC->BSRR, (~leds) << 16); // turn off LEDs
  SET_BIT(GPIOC->BSRR, leds); // turn on LEDs
}

/*---------------------------------------------------*/
/* Initialize GPIO pins used in the program          */
/*---------------------------------------------------*/
void setup_pins () {
  /* Configure PA1 and PA2 as input pin to read push button */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock (bit 0)
  CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODER1 | GPIO_MODER_MODER2)); // set to input mode
  /* Configure PC[0,3] as output pins to drive LEDs */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); // Enable GPIOC clock (bit 2) */
  CLEAR_BIT(GPIOC->MODER, 0x0000FFFF); /* Clear PC[0,3] mode bits */
  SET_BIT(GPIOC->MODER, 0x00005555); /* General purpose output mode*/
}


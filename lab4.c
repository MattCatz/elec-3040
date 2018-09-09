/*==============================================================*/
/* Matthew Cather and Jacob Davis                               */
/* ELEC 3040/3050 - Lab 3                                       */
/* Two counters, one going up and one going down.               */
/*==============================================================*/

#include "STM32L1xx.h" /* Microcontroller information */

static unsigned char UP = 0;
static unsigned char DOWN = 1;

void delay(void);
void count(unsigned char* counter, unsigned char direction);
void setup_pins(void);
void update_leds(void); 

struct {
  unsigned char count;
  unsigned char direction;
} typedef counter;

counter count_one = 0;
counter count_two = 0;

int main() {
  __disable_irq();

  setup_pins();
  setup_timer();
  setup_interrupts();

  __enableirq();
  
  while(1);
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
void count(counter* count) {
  if (count->direction == UP) {
    count->count = (count->count + 1) % 10;
  } else {
    count->count = (count->count + (10 - 1)) % 10;
  } //TODO make sure this is correct
}

/*---------------------------------------------------*/
/* Update the LEDS used in the lab                   */
/*---------------------------------------------------*/
void update_leds(counter* count) {
  //TODO
  unsigned short leds = (count_two & 0x0F) << 4; // PC[7:4]
  leds += (count_one & 0x0F); // PC[3:0]
  SET_BIT(GPIOC->BSRRH, (~counter) << 16); // turn off LEDs
  SET_BIT(GPIOC->BSRRL, leds); // turn on LEDs
}

/*---------------------------------------------------*/
/* Initialize GPIO pins used in the program          */
/*---------------------------------------------------*/
void setup_pins () {
  /* Configure PA1 and PA2 as input pin to read push button */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock (bit 0)
  CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODER1 | GPIO_MODER_MODER2)); // set to input mode
  /* Configure PC[0,7] as output pins to drive LEDs */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); // Enable GPIOC clock (bit 2) */
  CLEAR_BIT(GPIOC->MODER, 0x0000FFFF); /* Clear PC[0,7] mode bits */
  SET_BIT(GPIOC->MODER, 0x00005555); /* General purpose output mode*/
}

void setup_interupts() {
  /* Configure GPIO A0 as external inturupt 0 */
  CLEAR_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI0);
  SET_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI0_PA);

  /* Configure GPIO A1 as external inturupt 1 */
  CLEAR_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1);
  SET_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1_PA);  

  /* Unmask EXTI0 and EXTI1 and set both to rising edge trigger*/
  SET_BIT(EXTI->IMR, (EXTI_IMR_MR0 & EXTI_IMR_MR1));
  SET_BIT(EXTI->RTSR, (EXTI_RTSR_TR0 & EXTI_RTSR_TR1));

  /* Clear pending bit for EXTI0 and EXTI1 */
  CLEAR_BIT(EXTI->PR, (EXTI_PR_PR0 & EXTI_PR_PR1));
}

void setup_timers() {
  
}

void EXTI0_IRQHandler() {
  /* Hanndle Pushbutton 0 */
  /* Acknowledge interupt */
  CLEAR_BIT(EXTI->PE, EXTI_PR_PR0)
  count_two.direction = UP;
}

void EXTI1_IRQHandler() {
  /* Hanndle Pushbutton 1*/
  /* Acknowledge interupt */
  CLEAR_BIT(EXTI->PE, EXTI_PR_PR1)
  count_two.direction = DOWN;
}

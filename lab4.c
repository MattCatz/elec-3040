/*==============================================================*/
/* Matthew Cather and Jacob Davis                               */
/* ELEC 3040/3050 - Lab 4                                       */
/* Two counters, one updates once every delay (~1/2 sec) the    */
/* other every 2 delays (1 sec). Pushbuttons are used to switch */
/* the direction of the second counter.                         */
/*==============================================================*/

#include "STM32L1xx.h" /* Microcontroller information */

#define UP 1;
#define DOWN 0;
#define TRUE 1;
#define FALSE 0;

struct {
  unsigned char count;
  unsigned char direction;
  unsigned char disable;
} typedef counter;

void delay(void);
void count(counter* count);
void setup_pins(void);
void update_leds(void);
void setup_interupts(void);
void counter_factory(counter* c);

counter count_one;
counter count_two;
unsigned char LED8;
unsigned char LED9;

int main() {
  setup_pins();
  setup_interupts();
  counter_factory(&count_one);
  counter_factory(&count_two);
  GPIOC->BSRR = GPIO_BSRR_BS_9; 
	LED8 = 0;
	LED9 = 1;

  __enable_irq();

  while(1) {
		delay();
		count_two.disable = ~count_two.disable;
    count(&count_one);
    count(&count_two);
    update_leds();
  };
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
  if (count->disable) return;
  if (count->direction) {
    count->count = (count->count + 1) % 10; //UP
  } else {
    count->count = (count->count + (10 - 1)) % 10; //DOWN
  }
}

/*---------------------------------------------------*/
/* Update the LEDS used in the lab                   */
/*---------------------------------------------------*/
void update_leds(void) {
  unsigned short leds = 0;
  leds = (count_two.count & 0x0F) << 4; // PC[7:4]
  leds += (count_one.count & 0x0F); // PC[3:0]
  SET_BIT(GPIOC->BSRR, (~leds & 0xFF) << 16); // turn off LEDs
  SET_BIT(GPIOC->BSRR, leds); // turn on LEDs
}

/*---------------------------------------------------*/
/* Initialize GPIO pins used in the program          */
/*---------------------------------------------------*/
void setup_pins () {
  /* Configure PA0 and PA1 as input pin to read push buttons */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock (bit 0)
  CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODER0 | GPIO_MODER_MODER1)); // set to input mode
  /* Configure PC[0,9] as output pins to drive LEDs */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); // Enable GPIOC clock (bit 2) */
  CLEAR_BIT(GPIOC->MODER, 0x000FFFFF); /* Clear PC[0,9] mode bits */
  SET_BIT(GPIOC->MODER, 0x00055555); /* General purpose output mode*/
}

void setup_interupts() {
  /* Configure GPIO A0 as external inturupt 0 */
  CLEAR_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI0);
  SET_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI0_PA);

  /* Configure GPIO A1 as external inturupt 1 */
  CLEAR_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1);
  SET_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1_PA);

  /* Unmask EXTI0 and EXTI1 and set both to rising edge trigger*/
  SET_BIT(EXTI->IMR, (EXTI_IMR_MR0 | EXTI_IMR_MR1));
  SET_BIT(EXTI->RTSR, (EXTI_RTSR_TR0 | EXTI_RTSR_TR1));

  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(EXTI1_IRQn);

  /* Clear pending bit for EXTI0 and EXTI1 */
  SET_BIT(EXTI->PR, (EXTI_PR_PR0 | EXTI_PR_PR1));

  NVIC_ClearPendingIRQ(EXTI0_IRQn);
  NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void small_delay () {
  int i;
  for (i=0; i<100000; i++) { //outer loop
    asm("nop"); //dummy operation for single-step test
  }
}

void EXTI0_IRQHandler() {
  /* Hanndle Pushbutton 0 */
  /* Acknowledge interupt */
  
  count_two.direction = DOWN;
	if (LED8) {
		GPIOC->BSRR = GPIO_BSRR_BR_8;
		LED8 = 0;
	} else {
		GPIOC->BSRR = GPIO_BSRR_BS_8; //Set PC8=1 to turn ON blue LED
		LED8 = 1;
	}
  //GPIOC->BSRR = GPIO_BSRR_BR_9; //Reset PC9=0 to turn OFF green LED
	small_delay();
	SET_BIT(EXTI->PR, EXTI_PR_PR0);
  NVIC_ClearPendingIRQ(EXTI0_IRQn);
}

void EXTI1_IRQHandler() {
  /* Hanndle Pushbutton 1*/
  /* Acknowledge interupt */
  SET_BIT(EXTI->PR, EXTI_PR_PR1);
  count_two.direction = UP;
	if (LED9) {
		GPIOC->BSRR = GPIO_BSRR_BR_9;
		LED9 = 0;
	} else {
		GPIOC->BSRR = GPIO_BSRR_BS_9; //Set PC9=1 to turn ON blue LED
		LED9 = 1;
	}
  //GPIOC->BSRR = GPIO_BSRR_BR_8; //Reset PC8=0 to turn OFF blue LED
	small_delay();
  NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void counter_factory(counter* c) {
  c->count = 0;
  c->direction = UP;
  c->disable = FALSE;
}

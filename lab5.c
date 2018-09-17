/*==============================================================*/
/* Matthew Cather and Jacob Davis                               */
/* ELEC 3040/3050 - Lab 5                                       */
/* Outputs a counter on PC[0,3] about every second. If there is */
/* a keypad event then it will output the key pressed over the  */
/* PC[0,3] instead for 5 cycles. Keypresses are detected using  */
/* an external interrupt on PA1.                                */
/*==============================================================*/

#include "STM32L1xx.h" /* Microcontroller information */

/*
 * Wraps all the keypad into one structure. Row and column hold the
 * value [1,4]. The key number can be found by multipying the row and
 * column value. Event tells how long to display key value.
 */
struct {
  unsigned char row;
  unsigned char column;
  unsigned char event;
} typedef matrix_keypad;

void delay(void);
void setup_pins(void);
void update_leds(unsigned char value);
void setup_interupts(void);
void keypad_factory(matrix_keypad* k);

matrix_keypad keypad;

/*
 * First everything is configured/initilized. Specifically notice
 * that all columns are grounded so keypresses can be detected.
 * After that we enter the event loop. The idea here is that if there is
 * an event then we want to display it for N cycles, where N is specified
 * as the value of keypad.event. If there is no event then we will display
 * the count instead. It was constructed this was to handle keys pressed
 * during keypress events.
 */
int main() {
  setup_pins();
  setup_interupts();
  keypad_factory(&keypad);
  unsigned char count = 0;
  SET_BIT(GPIOB->BSRR, 0x0000F000); //Ground all columns

  __enable_irq();

  while(1) {
    delay();
    count = (count + 1) % 10;
    if (keypad.event--) {
      update_leds(row * column);
    } else {
      update_leds(count);
    }
  };
}

/*
 * Delay function - do nothing for about 1 second         
 */
void delay () {
  int i,j;
  for (i=0; i<20; i++) {
    for (j=0; j<20000; j++) {
      asm("nop");
    }
  }
}

/*
 * Update the LEDS connected to PC[0,4].                   
 */
void update_leds(unsigned char value) {
  SET_BIT(GPIOC->BSRR, (~value & 0xFF) << 16); // turn off LEDs not used
  SET_BIT(GPIOC->BSRR, value); // turn on LEDs
}

/*
 * Initialize GPIO pins used in the program.
 * PA1 -> input to detect keypress
 * PB[0,3] -> input to detect row of keypress
 * PB[4,7] -> output to short column lines of keypad
 * PC[0,3] -> output count/keypress
 */
void setup_pins () {
  /* Configure PA1 as input pin to listen for row lines */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock (bit 0)
  CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER1); // set to input mode

  /*Configure PB[0,3] as keypad row lines, input*/
  SET_BIT(RCC-> RCC_AHBENR_GPIOBEN);
  CLEAR_BIT(GPIOB->MODER, 0x000000FF);
  /*This enables the pullup resistor so we can read the row lines*/
  MODIFY_REGISTER(GPIOB->PUPDR, 0x000000FF, 0x00000055);
  
  /*Configure PB[4,7] as keypad column lines, output*/
  MODIFY_REGISTER(GPIOB->MODER, 0x0000FF00, 0x00005500);

  /* Configure PC[0,3] as output pins to drive LEDs */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); // Enable GPIOC clock (bit 2) */
  CLEAR_BIT(GPIOC->MODER, 0x000000FF); /* Clear PC[0,3] mode bits */
  SET_BIT(GPIOC->MODER, 0x00000055); /* General purpose output mode*/
}

/*
 * Setup external inturupt on PA1 to detect a keypress.
 */
void setup_interupts() {
  /* Configure GPIO A1 as external inturupt 1 */
  CLEAR_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1);
  SET_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1_PA);

  /* Unmask EXTI1 and set both to rising edge trigger*/
  SET_BIT(EXTI->IMR,  EXTI_IMR_MR1);
  SET_BIT(EXTI->RTSR, EXTI_RTSR_TR1);

  NVIC_EnableIRQ(EXTI1_IRQn);

  /* Clear pending bit for EXTI1 */
  SET_BIT(EXTI->PR, EXTI_PR_PR1);

  NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

/*
 * Small delay used to mitigate timing issues with interfacing with keypad.
 */
void small_delay() {
  int i;
  for (i=0; i<10; i++) {
    asm("nop");
  }
}

/*
 * This might be a little to clever to actually work. What we want to do is
 * loop over the columns grounding one at a time and checking to see if one
 * of the rows is shorted. If nothing is found then the the row and column
 * are set to zero to signify invalid key and event is set to zero so the 
 * main event loop does not pick up the keypress.
 */
void EXTI1_IRQHandler() {
  /* Hanndle Pushbutton 1*/
  /* Acknowledge interupt */
  SET_BIT(EXTI->PR, EXTI_PR_PR1);
  
  COLUMN_MASK = [GPIO_BSRR_BR_0, GPIO_BSRR_BR_1, GPIO_BSRR_BR_2, GPIO_BSRR_BR_3];
  ROW_MASK = [GPIO_IDR_IDR_0, GPIO_IDR_IDR_1, GPIO_IDR_IDR_2, GPIO_IDR_IDR_3];

  for (keypad.column=1;keypad.column<5;keypad.column++) {
    SET_BIT(GPIO->BSRR, 0x0000000F);
    SET_BIT(GPIO->BSRR, COLUMN_MASK[keypad.column]);
    small_delay();
    for (keypad.row=1;keypad.row<5;keypad.row++) {
      if (!READ_BIT(GPIOB->IDR, ROW_MASK[keypad.row])) {
        keypad.event = 5;
        NVIC_ClearPendingIRQ(EXTI1_IRQn);
        return;
      }
    }
  }
  
  keypad.row = 0;
  keypad.column = 0;
  keypad.event = 0;
  NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void keypad_factory(matrix_keypad* k) {
  k->row = 0;
  k->column = 0;
  k->event = 0;
}

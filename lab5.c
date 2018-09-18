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
 * value [1,4]. The key number can be found by indexing into the keys
 * array using row and column. Event tells how long to display key value.
 */
struct {
  unsigned char row;
  unsigned char column;
  unsigned char event;
  const unsigned char row1[4];
  const unsigned char row2[4];
  const unsigned char row3[4];
  const unsigned char row4[4];
  const unsigned char* keys[];
} typedef matrix_keypad;

void delay(void);
void setup_pins(void);
void update_leds(unsigned char value);
void setup_interupts(void);
// Begin DEBUG
void debug_leds(unsigned char value);
// End DEBUG

matrix_keypad keypad = {
  .row = ~0,  //Initialize rows to MAX
  .column = ~0, //Initialize columns to MAX
  .event = 0,
  .row1 = {1, 2, 3, 0xA},
  .row2 = {4, 5, 6, 0xB},
  .row3 = {7, 8, 9, 0xC},
  .row4 = {0xE, 0, 0xF, 0xD},
  .keys = {keypad.row1, keypad.row2, keypad.row3, keypad.row4},
};

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
  unsigned char count = 0;
  SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns

  __enable_irq();

  while(1) {
    delay();
    if (keypad.event) {
      keypad.event--;
			update_leds(keypad.keys[keypad.column][keypad.row]);
    } else {
			update_leds(count);
		}
		count = (count + 1) % 10;
		// Begin DEBUG
		debug_leds(count % 10);
		// Eend DEBUG
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

void debug_leds(unsigned char value) {
	SET_BIT(GPIOC->BSRR, (~value & 0xFF00) << 16); // turn off LEDs not used
  SET_BIT(GPIOC->BSRR, value << 4); // turn on LEDs
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
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
  CLEAR_BIT(GPIOB->MODER, 0x000000FF);
  /*This enables the pullup resistor so we can read the row lines*/
  MODIFY_REG(GPIOB->PUPDR, 0x000000FF, 0x00000055);
  
  /*Configure PB[4,7] as keypad column lines, output*/
  MODIFY_REG(GPIOB->MODER, 0x0000FF00, 0x00005500);

  /* Configure PC[0,3] as output pins to drive LEDs */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); // Enable GPIOC clock (bit 2) */
  CLEAR_BIT(GPIOC->MODER, 0x000000FF); /* Clear PC[0,3] mode bits */
  SET_BIT(GPIOC->MODER, 0x00000055); /* General purpose output mode*/
	
	// Begin DEBUG
	/* Configure PC[4,7] as output pins to drive LEDs */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); // Enable GPIOC clock (bit 2) */
  CLEAR_BIT(GPIOC->MODER, 0x0000FF00); /* Clear PC[0,3] mode bits */
  SET_BIT(GPIOC->MODER, 0x00005500); /* General purpose output mode*/
	// End DEBUG
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
  SET_BIT(EXTI->FTSR, EXTI_FTSR_TR1);

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
  for (i=0; i<100; i++) {
    asm("nop");
  }
}

/*
 * This might be a little to clever to actually work. What we want to do is
 * loop over the columns grounding one at a time and checking to see if one
 * of the rows is shorted. If nothing is found then the the row and column
 * are set to MAX to signify invalid key and event is set to zero so the 
 * main event loop does not pick up the keypress.
 */
void EXTI1_IRQHandler() {
  /* Hanndle Pushbutton 1*/
  /* Acknowledge interupt */
  SET_BIT(EXTI->PR, EXTI_PR_PR1);
  
  const int COLUMN_MASK[] = {GPIO_BSRR_BR_4, GPIO_BSRR_BR_5, GPIO_BSRR_BR_6, GPIO_BSRR_BR_7};
  const int ROW_MASK[] = {GPIO_IDR_IDR_0, GPIO_IDR_IDR_1, GPIO_IDR_IDR_2, GPIO_IDR_IDR_3};

  for (keypad.column=0;keypad.column<4;keypad.column++) {
		//unsigned int DEBUG_BSRR = 0;
    SET_BIT(GPIOB->BSRR, 0x000000F0);
    SET_BIT(GPIOB->BSRR, COLUMN_MASK[keypad.column]);
		//GPIOB->BSRR = DEBUG_BSRR;			
    for (keypad.row=0;keypad.row<4;keypad.row++) {
			small_delay();
			//unsigned short DEBUG_IDR = GPIOB->IDR;
			unsigned short temp = READ_BIT(GPIOB->IDR, ROW_MASK[keypad.row]);
      if (!temp) {
        keypad.event = 4;
        update_leds(keypad.keys[keypad.row][keypad.column]);  //Display keys pressed via LEDs
        SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns
        NVIC_ClearPendingIRQ(EXTI1_IRQn);
        return;
      }
    }
  }
  
  keypad.row = ~0;
  keypad.column = ~0;
  //keypad.event = 0;
  SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns
  NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

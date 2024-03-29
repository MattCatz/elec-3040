/*==============================================================*/
/* Matthew Cather and Jacob Davis                               */
/* ELEC 3040/3050 - Lab 9                                      */
/*==============================================================*/

#include "STM32L1xx.h" /* Microcontroller information */

/*
 * Wraps all the keypad into one structure. Row and column hold the
 * value [1,4]. The key number can be found by indexing into the keys
 * array using row and column. Event tells how long to display key value.
 */
 
enum port_mode {
   Input = 0x00000000,
   Output = 0x55555555,
   Alternate = 0xAAAAAAAA,
   Analog = 0xFFFFFFFF
};

enum pupd {
   None = 0x00000000,
   Up = 0x55555555,
   Down = 0xAAAAAAAA
};

void setup_keypad(void);
void setup_pwm_gen(void);
void setup_speed_ctr(void);

double period;

int main() {
   SET_BIT(RCC->CR,RCC_CR_HSION); // Turn on 16MHz HSI oscillator
   while ((RCC->CR & RCC_CR_HSIRDY) == 0); // Wait until HSI ready
   SET_BIT(RCC->CFGR, RCC_CFGR_SW_HSI); // Select HSI as system clock
   
   SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock (bit 0)
   SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
   
   setup_pins();
   setup_interupts();
   setup_timers();
   
   period = 0;
   
   __enable_irq();
   
   while(1) {
      asm("WFI"); // Idle forever
   }
}

void  setup_keypad() {
   /* Configure PA1 as input pin to listen for row lines */
   CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER1); // set to input mode
   
   /*Configure PB[0,3] as keypad row lines, input*/
   MODIFY_REG(GPIOB->MODER, 0x000000FF, Input);
   /*This enables the pullup resistor so we can read the row lines*/
   MODIFY_REG(GPIOB->PUPDR, 0x000000FF, UP);
   
   /*Configure PB[4,7] as keypad column lines, output*/
   MODIFY_REG(GPIOB->MODER, 0x0000FF00, Output);
  
   SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns
   
   /* Configure GPIO A1 as external inturupt 1 */
   MODIFY_REG(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1, SYSCFG_EXTICR1_EXTI1_PA);
   
   /* Unmask EXTI1 and set both to rising edge trigger*/
   SET_BIT(EXTI->IMR,  EXTI_IMR_MR1);
   SET_BIT(EXTI->FTSR, EXTI_FTSR_TR1);
   
   
   NVIC_EnableIRQ(EXTI1_IRQn);
   
   /* Clear pending bit for EXTI1 */
   SET_BIT(EXTI->PR, EXTI_PR_PR1);
   NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void setup_pwm_gen() {
   /* Change PA6 to altrnative function mode */
   MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER6, Alternate);
   /* Change the alternate function of PA6 to be the CC*/
   MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFRL6, 0x03000000);
   
	/* Timer 10 */
   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN); //enable clock source
   TIM10->ARR = 999; //set auto reload. assumes 16MHz
   TIM10->PSC = 159; //set prescale.
   TIM10->CCR1 = 10; //Set compair value
   TIM10->CNT = 0;
   MODIFY_REG(TIM10->CCMR1, TIM_CCMR1_CC1S, TIM_CCMR1_CC1S); // Capture compair select
   MODIFY_REG(TIM10->CCMR1, TIM_CCMR1_OC1M, 0x0060); // Active to inactive
   SET_BIT(TIM10->CCER, TIM_CCER_CC1E); // drive output pin
  
   SET_BIT(TIM10->CR1, TIM_CR1_CEN); //enable counting
}

void speed_controller() {
   /* Change PA7 to altrnative function mode */
   MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7, Alternate);
   
   
   /* Change the alternate function of PA7 to be the CC*/
   MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFRL7, 0x30000000);
   
   /*This enables the pullup resistor so we can read input capture*/
   MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR7, UP);
   
   NVIC_ClearPendingIRQ(TIM11_IRQn);  // Clear Pending Status of Timer IRQ
   NVIC_EnableIRQ(TIM11_IRQn);
   
   /* Timer 11 */
   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN); //enable clock source
   
   /* CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER) */
   CLEAR_BIT(TIM11->CCER, TIM_CCER_CC1E);
   
   CLEAR_BIT(TIM11->CCER, TIM_CCER_CC1P); // Set to falling edge
   
   MODIFY_REG(TIM11->CCMR1, TIM_CCMR1_CC1S, 0x01); // Capture compair select input
   TIM11->ARR = 0xFFFF; // value > max period to prevent update event before capture
   TIM11->PSC = 0x9F; //set prescale.
   
   /* Enabling Timer Interupts */
   SET_BIT(TIM11->DIER, TIM_DIER_CC1IE); // enable capture compair interupts
   
   CLEAR_BIT(TIM11->SR,TIM_SR_CC1IF); // clear capture/compair 1 interupt flag
   
   /* CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER) */
   SET_BIT(TIM11->CCER, TIM_CCER_CC1E); // drive output pin
   
   SET_BIT(TIM11->CR1, TIM_CR1_CEN); //enable counting
}

void TIM11_IRQHandler() {
   CLEAR_BIT(TIM11->SR, TIM_SR_CC1IF);
  
   TIM11->CNT = 0;
   period = (TIM11->CCR1*TIM11->PSC*(1/16000000.0); // Becuase we use the high speed clock
   
   NVIC_ClearPendingIRQ(TIM11_IRQn);
}


/**
 *
 * From here down is keypad stuff
 *
 */

/*
 * Small delay used to mitigate timing issues with interfacing with keypad.
 */
void small_delay() {
  int i;
  for (i=0; i<10; i++) {
    asm("nop");
  }
}

struct {
  unsigned char row;
  unsigned char column;
  const unsigned char row1[4];
  const unsigned char row2[4];
  const unsigned char row3[4];
  const unsigned char row4[4];
  const unsigned char* keys[];
} typedef keypad_decoder;

struct {
  unsigned char value;
  unsigned char event;
} typedef matrix_keypad;

keypad_decoder decoder = {
  .row = ~0,  //Initialize rows to MAX
  .column = ~0, //Initialize columns to MAX
  .row1 = {1, 2, 3, 0xA},
  .row2 = {4, 5, 6, 0xB},
  .row3 = {7, 8, 9, 0xC},
  .row4 = {0xE, 0, 0xF, 0xD},
  .keys = {decoder.row1, decoder.row2, decoder.row3, decoder.row4}
};

matrix_keypad keypad = {
 .value = 0,
 .event = 0,
};

 const int COLUMN_MASK[] = {GPIO_BSRR_BR_4, GPIO_BSRR_BR_5, GPIO_BSRR_BR_6, GPIO_BSRR_BR_7};
 const int ROW_MASK[] = {GPIO_IDR_IDR_0, GPIO_IDR_IDR_1, GPIO_IDR_IDR_2, GPIO_IDR_IDR_3};

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
 

  for (decoder.column=0;decoder.column<4;decoder.column++) {
    SET_BIT(GPIOB->BSRR, 0x000000F0);
    SET_BIT(GPIOB->BSRR, COLUMN_MASK[decoder.column]);
    for (decoder.row=0;decoder.row<4;decoder.row++) {
      small_delay();
      if (!READ_BIT(GPIOB->IDR, ROW_MASK[decoder.row])) {
        keypad.value = decoder.keys[decoder.row][decoder.column];
        keypad.event = 1;
        // Cheap way to break out of nested loop
        decoder.row = 5;
        decoder.column = 5;
      }
    }
  }
  
   if (keypad.event == 1 && keypad.value < 11) {
      TIM10->CCR1 = keypad.value * (TIM10->ARR + 1) / 10;;
      keypad.event = 0;
   }
	 
  SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns
  NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

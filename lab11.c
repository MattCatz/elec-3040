/*==============================================================*/
/* Matthew Cather and Jacob Davis                               */
/* ELEC 3040/3050 - Lab 11                                      */
/*==============================================================*/

#include "STM32L1xx.h" /* Microcontroller information */

#define MODIFY(REG, MASK, NEWVALUE)  WRITE_REG((REG), (((READ_REG(REG)) & (~(MASK))) | (NEWVALUE & MASK)))

/* Port Mode */
#define   Input = 0x00000000
#define   Output = 0x55555555
#define   Alternate = 0xAAAAAAAA
#define   Analog = 0xFFFFFFFF

/* Pull-up/down Mode */
#define   None = 0x00000000
#define   Up = 0x55555555
#define   Down = 0xAAAAAAAA

void delay(void);
void setup_pins(void);
void setup_interupts(void);
void setup_timers(void);
void update_leds(unsigned char value);
void setup_speed_ctr(void);

double period;
double amplitude;
double voltage;
volatile double avg;
double alpha;
int sample;
int window[100], position;

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
   SET_BIT(RCC->CR,RCC_CR_HSION); // Turn on 16MHz HSI oscillator
   while ((RCC->CR & RCC_CR_HSIRDY) == 0); // Wait until HSI ready
   SET_BIT(RCC->CFGR, RCC_CFGR_SW_HSI); // Select HSI as system clock
   
   SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock
   SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN); // Enable GPIOC clock
   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN); // Enable ADC clock
   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN); //enable clock source
   
   setup_keypad();
   setup_pwm_gen();
   setup_speed_ctr();

  __enable_irq();

  while(1) {
	asm("nop");
  };
}

void setup_speed_ctr() {
   /* Change PA7 to alalog mode */
   MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7, (0xFFFFFFFF & GPIO_MODER_MODER7));
   
   SET_BIT(ADC1->CR2, ADC_CR2_CONT); // Enable continuous conversion
   MODIFY_REG(ADC1->SQR5, ADC_SQR5_SQ1, 0x7); // Use PA7 for sampling
   SET_BIT(ADC1->CR2, ADC_CR2_ADON); // Turn on converter

   while ((ADC1->SR & ADC_SR_ADONS) == 0); // Wait for converter to power on
}

void  setup_keypad() {
   /* Configure PA1 as input pin to listen for row lines */
   CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER1); // set to input mode
   
   /*Configure PB[0,3] as keypad row lines, input*/
   MODIFY(GPIOB->MODER, 0x000000FF, Input);
   /*This enables the pullup resistor so we can read the row lines*/
   MODIFY(GPIOB->PUPDR, 0x000000FF, UP);
   
   /*Configure PB[4,7] as keypad column lines, output*/
   MODIFY(GPIOB->MODER, 0x0000FF00, Output);
  
   SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns
   
   /* Configure GPIO A1 as external inturupt 1 */
   MODIFY(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1, SYSCFG_EXTICR1_EXTI1_PA);
   
   /* Unmask EXTI1 and set both to rising edge trigger*/
   SET_BIT(EXTI->IMR,  EXTI_IMR_MR1);
   SET_BIT(EXTI->FTSR, EXTI_FTSR_TR1);
   
   SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns
   
   NVIC_EnableIRQ(EXTI1_IRQn);
   
   /* Clear pending bit for EXTI1 */
   SET_BIT(EXTI->PR, EXTI_PR_PR1);
   NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void setup_pwm_gen() {
   /* Change PA6 to alternative function mode */
   MODIFY(GPIOA->MODER, GPIO_MODER_MODER6, Alternate);
   /* Change the alternate function of PA6 to be the CC*/
   MODIFY(GPIOA->AFR[0], GPIO_AFRL_AFRL6, 0x03000000);
   
	/* Timer 10 */
   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN); //enable clock source
   TIM10->ARR = 999; //set auto reload. assumes 16MHz
   TIM10->PSC = 159; //set prescale.
   TIM10->CCR1 = 10; //Set compare value
   TIM10->CNT = 0;
   MODIFY(TIM10->CCMR1, TIM_CCMR1_CC1S, TIM_CCMR1_CC1S); // Capture compare select
   MODIFY(TIM10->CCMR1, TIM_CCMR1_OC1M, 0x0060); // Active to inactive
   SET_BIT(TIM10->CCER, TIM_CCER_CC1E); // drive output pin
  
   SET_BIT(TIM10->CR1, TIM_CR1_CEN); //enable counting
}

void speed_controller() {
   /* Change PA7 to analog mode */
   MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7, (0xFFFFFFFF & GPIO_MODER_MODER7));
   
   /* Change the alternate function of PA7 to be the CC*/
   MODIFY(GPIOA->AFR[0], GPIO_AFRL_AFRL7, 0x30000000);
   
   /*This enables the pullup resistor so we can read input capture*/
   MODIFY(GPIOA->PUPDR, GPIO_PUPDR_PUPDR7, UP);
   
   MODIFY_REG(ADC1->SQR5, ADC_SQR5_SQ1, 0x7); // Use PA7 for sampling
   SET_BIT(ADC1->CR2, ADC_CR2_ADON); // Turn on converter
   
   NVIC_ClearPendingIRQ(TIM11_IRQn);  // Clear Pending Status of Timer IRQ
   NVIC_EnableIRQ(TIM11_IRQn);
   
   /* Timer 11 */
   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN); //enable clock source
   TIM11->ARR = 8190; //set auto reload.
   TIM11->PSC=0xFF; //set prescale.
   TIM11->CNT=0;
   SET_BIT(TIM11->DIER, TIM_DIER_UIE); //enable interrupts
}

void TIM11_IRQHandler() {
   CLEAR_BIT(TIM11->SR, TIM_SR_UIF);

   SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
   while(~(ADC1->SR & ADC_SR_EOC)));
   data = ADC1->DR;
   if (data_size < 100) {
      graph[data_index] = data;
	  data_index++;
   }
   
   TIM11->SR = ~TIM_SR_UIF;
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

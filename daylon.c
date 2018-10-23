/*====================================================*/
/* Daylon G. Hester & Charles Ayers*/
/* ELEC 3040: Lab 9, Program 1 */
/* Tachometer Reading - Frequency */
/*====================================================*/
#include "STM32L1xx.h" 			 // Microcontroller information

/* Global Variables */

unsigned char count1 = 0;    // Value of tenths of a second being displayed on LEDs (DIOs 0-3)
unsigned char stopwatch = 0; // Status of stopwatch: 0-stopped, 1-counting

	unsigned char row;         // Value for row to be used when reading keypad
	unsigned char r, c, i;     // Value used for counting in delay loops
	int j;
	unsigned char key = 0;     // Value of key on keyboard (points to array entry)
	
	char keypad[4][4] =        // 4x4 Array containing all keys on keypad
		{{1, 2, 3, 0xA},         // Row 1 of Keypad 
		{4, 5, 6, 0xB},          // Row 2 of Keypad 
		{7, 8, 9, 0xC},          // Row 3 of Keypad 
		{0xF, 0, 0xE, 0xD}};     // Row 4 of Keypad (*=15, #=14)
		
	char row_mask[] = {0x0E, 0x0D, 0x0B, 0x07};
	char column_mask[] = {0xEF, 0xDF, 0xBF, 0x7F};
	
	int period = 0;

/*---------------------------------------------------*/
/* GPIO Pin Initialization                           */
/*---------------------------------------------------*/

void PinSetup () {
 /*Set clock to 16 MHz*/
	RCC->CR |= RCC_CR_HSION;                       // Turn on 16MHz HSI oscillator
	while ((RCC->CR & RCC_CR_HSIRDY) == 0);        // Wait until HSI ready
	RCC->CFGR |= RCC_CFGR_SW_HSI;                  // Select HSI as system 	
	
 /* Configure PA1 as an input*/
 RCC->AHBENR  |= 0x01; 					// Enable GPIOA clock (bit 0)
 GPIOA->MODER &= ~(0x0000000C); // General purpose input mode
	
 /* Configure PB0-3 as inputs from keypad and PB4-7 as outputs to keypad*/
 RCC->AHBENR  |= 0x02; 					// Enable GPIOB clock (bit 1)
 GPIOB->MODER &= ~(0x000000FF); // General purpose input mode	PB[0-3] (Rows)
 GPIOB->MODER &= ~(0x0000FF00); // Clear Bits	PB[4-7] (Columns)
 GPIOB->MODER |=  (0x00005500); // General purpose ouput mode	PB[4-7] (Columns)
 GPIOB->PUPDR	&= ~(0x000000FF); // Clear Pull-up/down for PB[0-3] (Rows)
 GPIOB->PUPDR	|=  (0x00000055); // Activate Pull-up for PB[0-3] (Rows)
	
 /* Configure PC3-PC0 as output pins to drive LEDs */
 RCC->AHBENR  |= 0x04; 					// Enable GPIOC clock (bit 2)
 GPIOC->MODER &= ~(0x000000FF); // Clear PC3-PC0 mode bits
 GPIOC->MODER |=  (0x00000055); // General purpose output mode
	
 /*Initialize Columns to zero*/
 GPIOB->BSRR = 0x00F0 << 16;    //Reset PB[4-7] to "ground" all columns
	
/*Configure PA7 as alternate function pin for the input capture*/	
GPIOA->MODER  &= ~(0x0000C000); // Clear PA7 Mode
GPIOA->MODER  |=  (0x00008000); // Set PA7 to Alternate Function Mode
GPIOA->AFR[0] &= ~(0xF0000000); // Clear Alternate Function Settings for PA7
GPIOA->AFR[0] |= 	(0x30000000); // Set PA7 to Alternate Function 3 (Timers 9-11)
GPIOA->PUPDR  &= ~(0x0000C000); // Clear Pull-up/Pull-down for PA7
GPIOA->PUPDR  |=  (0x00004000); // Activate Pull-up for PA7

/*----------------------------------------------------------*/
/* Configure External Interrupt Sources                     */
/*----------------------------------------------------------*/	
	
 SYSCFG->EXTICR[0]&= 0xFF0F; //Clear EXTI1 Bit Field
 SYSCFG->EXTICR[0]|= 0x0000; //Set PA1

 EXTI->FTSR |= 0x0002;       //Set Bit 1 as Falling Edge Triggered
 EXTI->IMR  |= 0x0002;       //Masks (enables) the interrupt for this line
 EXTI->PR   |= 0x0002;       //Clear Bit to indicate no interrupt has occured
	
 NVIC_EnableIRQ(EXTI1_IRQn);       //Enable IRQ on Bit Line 1
 NVIC_ClearPendingIRQ(EXTI1_IRQn); //Clear Pending Bit Line 1
 NVIC_SetPriority(EXTI1_IRQn,1);   //Set Priority of IRQ to 1
 NVIC_ClearPendingIRQ(EXTI1_IRQn); //Clear Pending Status of NVIC
 
 /*Configure Timer Interrupt, but don't enable yet*/
 
 RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; //Enable Clock Timer 10
 
 GPIOA->MODER  &= ~(0x00003000); // Clear PA6 Mode
 GPIOA->MODER  |=   0x00002000;  // Set PA6 to Alternate Function Mode
 GPIOA->AFR[0] &= ~(0x0F000000); // Clear Alternate function settings for PA6
 GPIOA->AFR[0] |=   0x03000000;  // Set PA6 to Alternate Function 3 (Timers 9-11)

 TIM10->PSC    = 99;      // Set Prescaler Value for 200 Hz Operation
 TIM10->ARR    = 799;     // Set Autoreload Value
 TIM10->CR1   |= 0x01;    // Enable counting
 TIM10->CNT    = 0;       // Set Timer Count = 0
 
 TIM10->CCMR1 &= ~(0x13); // Clear Capture/Compare Mode Register
 TIM10->CCMR1 |=   0x60;  // Set to output & active to inactive
 
 TIM10->CCER  &=  (0x03); // Clear Capture/Compare Enable Register
 TIM10->CCER  |=   0x01;  // Set to output & active high
 
 NVIC_ClearPendingIRQ(TIM10_IRQn); // TIM10 = IRQ26
 NVIC_SetPriority(TIM10_IRQn,0);   // Set Priority of Timer IRQ to 0

 /*Enable Interrupt for Timer 10*/
 TIM10->DIER |= TIM_DIER_UIE; //Enable TIM10 to signal an interrupt
 NVIC_EnableIRQ(TIM10_IRQn);  //TIM10 = IRQ26
 TIM10->SR &= ~0x01;                // Clear UIF
 NVIC_ClearPendingIRQ(TIM10_IRQn);  // Clear Pending Status of Timer IRQ  
 
 /*Configure Timer Interrupt for Timer 11*/
 
 RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; //Enable Clock Timer 11
 
 TIM11->PSC    = 0x9F;      // Set Prescaler Value so that each count is 10us
 TIM11->ARR    = 0xFFFF;   // Set Autoreload Value
 //TIM11->CNT    = 0;       // Set Timer Count = 0
 
  /*Enable Interrupt for Timer 11*/
 TIM11->DIER |= TIM_DIER_UIE;   // Enable TIM11 to signal an interrupt
 TIM11->DIER |= TIM_DIER_CC1IE; // Enable Capture/Compare 1 to signal an interrupt 
 TIM11->SR &= ~TIM_SR_UIF;       // Clear UIF
 TIM11->SR &= ~TIM_SR_CC1IF;    // Clear Capture Compare
 TIM11->CCMR1 |=   0x01;        // Set to input
 TIM11->CCER  |=   TIM_CCER_CC1E;  // Enable Capture Compare
 TIM11->CR1   |= TIM_CR1_CEN;    // Enable counting
 
 NVIC_ClearPendingIRQ(TIM11_IRQn);  // Clear Pending Status of Timer IRQ  
 NVIC_EnableIRQ(TIM11_IRQn);
 NVIC_SetPriority(TIM11_IRQn,0);   // Set Priority of Timer IRQ to 0
 
 __enable_irq(); //Enable Interrupts
}


void PWM_Select ()
{
	if(key < 11)
	{   
			if(key > 0)
			{
				TIM10->CCR1 = (key * 80 + 1);
			}
			else
			{
			TIM10->CCR1 = (0);    // Adjust pulse width based on key pressed
			}
		  count1 = key;                  // Store key value to be output
  }
}

void display ()
{
  GPIOC->ODR = 0;        //Clear LEDs
	GPIOC->ODR = (count1); //Output Count values to LEDs	
}

void get_key ()
{
		for(c=0; c<4; c++)               // Sweep through columns, grounding one at the time
 {  
		GPIOB->ODR = column_mask[c];     // Ground a single column 
		
		for (i=0; i<20; i++) {}          // Short Delay after grounding columns
		
		row = (GPIOB->IDR) & (0x0F);     // Store row value
		
	  for (r=0; r<4; r++)
			{
				if (row == row_mask[r])
				{
					key = keypad[r][c];        // Get key value
				}
			}
			
		}
  for (j=0; j<40000; j++) { 
	}	
}

void TIM10_IRQHandler() {
		
	TIM10->SR &= ~0x01;                // Clear UIF
	NVIC_ClearPendingIRQ(TIM10_IRQn);  // Clear Pending Status of Timer IRQ
}

void TIM11_IRQHandler() {
	
  if (TIM11->SR & 0x01)  // Set period = 0 if interrupt is triggered by an overflow
	{
		period = 0;           
	}
	else
	{
		period = 10*(TIM11->CCR1); // Get period of waveform from Count period*T = period (s)
	}
	 
	TIM11->CNT = 0;               // Reset Count
	
  TIM11->SR &= ~TIM_SR_UIF;       // Clear UIF
  TIM11->SR &= ~TIM_SR_CC1IF;    // Clear Capture Compare
	NVIC_ClearPendingIRQ(TIM11_IRQn);  // Clear Pending Status of Timer IRQ
}

void EXTI1_IRQHandler () {
	
	get_key ();
	PWM_Select();
	display();
	
  GPIOB->BSRR = 0x00F0 << 16;        // Reset PB[4-7] to "ground" all columns
	EXTI->PR   |= 0x0002;	             // Reset Pending Status of interrupt (PA1)
	NVIC_ClearPendingIRQ(EXTI1_IRQn);  // Reset pending status of interrupt NVIC
	return;
}

/*------------------------------------------------*/
/* Main Program                                   */
/*------------------------------------------------*/
int main(void) {
  PinSetup(); 									     // Configure GPIO pins
 /* Endless loop */
while(1) 
	{
	}
}

//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

//void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myDAC_Init(void);
void myADC_Init(void);
void myLCD_Init(void);
void mySPI_Init(void);
//void myGPIOC_Init(void);


// Your global variables...
//int second_edge;

// Input signal edge detection flag
volatile unsigned int edge_flag = 0;

//Variables for frequencies
volatile unsigned int frequency_curr = 0;
volatile unsigned int frequency_fresh =0;
volatile unsigned int frequency_ability =5;

//Variables for resistors
volatile unsigned int resistor_curr = 0;
volatile unsigned int resistor_fresh =0;
volatile unsigned int resistor_ability =5;
volatile unsigned int resistor_prescalar =1221;

volatile unsigned int control_Disable = 0x0;
volatile unsigned int control_Enable = 0x8;
volatile unsigned int delay = 4800;

volatile unsigned int value_Disable = 0x4;
volatile unsigned int value_Enable = 0xC;
unsigned int adc_value;


// LCD data and settings arrays
volatile unsigned int LCDvalue[2][8];
volatile unsigned int LCDinit[2][16];
volatile unsigned int LCDconf[8];


int main(int argc, char* argv[])
{
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	second_edge = 0;

//	myGPIOA_Init();		/* Initialize I/O port PA */
	myADC_Init();		// Initialize ADC//
	myDAC_Init();		// Initialize DAC//
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	mySPI_Init();		//Initialize SPI//
	myLCD_Init(); 		//Initialize LCD//
//	myGPIOC_Init();		//Initialize

	while (1)
	{


		// Nothing is going on here...
	}

	return 0;

}

void myDAC_Init()	{

	//Enable clock for DAC//
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;


	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	/* Ensure no pull-up/pull-down for PA3 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR3);

	/* Ensure no pull-up/pull-down for PA4 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	/* Ensure no pull-up/pull-down for PA5 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

	/* Configure PA1 as input NE555 (PIN 3)*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	//PA3 as output for LCK signal , to J5 (pin 25)
	GPIOA->MODER |= GPIO_MODER_MODER3;

	//initiate PA4 for analog mode for timer circuit
	GPIOA->MODER |= GPIO_MODER_MODER4;

	//initiate PA5 for analog mode for POT
	GPIOA->MODER |= GPIO_MODER_MODER5;

	trace_printf("DONE Configure GPIO in DAC\n");


	// Enable Control Register
	DAC->CR |= DAC_CR_EN1;

	//write analog voltage
	//ASK ABOUT THIS ONE ALONG WITH THE ADC READ TOO (LAST LINE)
	DAC->DHR12R1 = DAC_DHR12R1_DACC1DHR;
	//trace_printf("DONE Initialize DAC\n");
}

void myADC_Init()	{

	//Enable clock for ADC//
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	//Enable calibration for ADC
	ADC1->CR |= RCC_APB2ENR_ADCEN;

	while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL);
	//trace_printf("ADC Calibrated. \n");

	// continuous conversion mode && overun mode
	ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD;

	//right aligned data
	ADC1->CFGR1 &= ~(ADC_CFGR1_ALIGN);

	// enable ADC to channel 5
	ADC1->CHSELR |= ADC_CHSELR_CHSEL5;

	// enable ADC1
	ADC1->CR |= ADC_CR_ADEN;

	// sampling rate
	//ASK ABOUT THIS ONE SMPR0, SMPR!,SMPR2
	//ADC1->SMPR |= ADC_SMPR1_SMPR;

	// wait until it is ready
	while ((ADC1->ISR & ADC_ISR_ADRDY)!=ADC_ISR_ADRDY);

	// trigger conversion (start)
	ADC1->CR |= ADC_CR_ADSTART;
	adc_value = ADC1->DR;// = ADC_DR_DATA;
	//trace_printf("ADC Value %d\n", adc_value);

	//trace_printf("ADC Complete\n");





}


/*void myGPIOA_Init()
{
	 //Enable clock for GPIOA peripheral
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	 //Configure PA1 as input
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	GPIOA->MODER &= ~(GPIO_MODER_MODER3);

	//Ensure no pull-up/pull-down for PA1
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

}*/

/*void myGPIOC_Init()	{

	 //Enable clock for GPIOA peripheral
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure PA1 as input
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	GPIOA->MODER &= ~(GPIO_MODER_MODER3);

	// Ensure no pull-up/pull-down for PA1
	// Relevant register: GPIOA->PUPDR
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

}*/

void myTIM2_Init()
{
	trace_printf("Timer2 Start Initalizing....\n");
	/* Enable clock for TIM2 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = (uint16_t)0x0001;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;

	TIM2->CR1 |= TIM_CR1_CEN;
	//trace_printf("Done Initializing Timer2\n");
}

void myEXTI_Init()
{
	trace_printf("Initializing EXTI......\n");
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= (SYSCFG_EXTICR1_EXTI1_PA);


	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	//trace_printf("DONE Initializing EXTI\n");
}

void mySPI_Init(){
	trace_printf("Initializing SPI......\n");
	// Declaring SPI AND GPIO Init Values
	SPI_InitTypeDef SPI_InitValueInfo;
	//trace_printf("Initializing SPI_InitValueInfo......\n");
	SPI_InitTypeDef* SPI_InitValue= &SPI_InitValueInfo;
	//trace_printf("Initializing SPI_InitValue......\n");
	GPIO_InitTypeDef GPIO_InitValue;
	//trace_printf("Initializing GPIO_InitValue......\n");

	//Initialize Structures
	GPIO_StructInit(&GPIO_InitValue);
	//trace_printf("Initializing GPIO_StructInit......\n");

	/* Enable clock for TIM2 peripheral */
		// Relevant register: RCC->APB1ENR
		//RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		// Enable clock for GPIOB peripheral
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		//trace_printf("SPI Pin3......\n");
		//Pin 3
		GPIO_InitValue.GPIO_Pin= GPIO_Pin_3 ;
		GPIO_InitValue.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitValue.GPIO_Mode= GPIO_Mode_AF;
		GPIO_Init(GPIOB,&GPIO_InitValue);
		//trace_printf("SPI Pin5......\n");
		//Pin 5
		GPIO_InitValue.GPIO_Pin= GPIO_Pin_5 ;
		GPIO_InitValue.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitValue.GPIO_Mode= GPIO_Mode_AF;
		GPIO_Init(GPIOB,&GPIO_InitValue);

		//Configure alternate function mode
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_0);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_0);

		//Pin 4
		//GPIO_InitValue.GPIO_Pin= GPIO_Pin_4 ;

		//HC595_Write(SPIsenddata());
		//Write_cmd();
		// Initializing SPI values

		//ENABLE CLOCK FOR SPI1
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

		SPI_InitValue->SPI_Direction = SPI_Direction_1Line_Tx;
		SPI_InitValue->SPI_Mode= SPI_Mode_Master;
		SPI_InitValue->SPI_DataSize= SPI_DataSize_8b;
		SPI_InitValue->SPI_CPOL= SPI_CPOL_Low;
		SPI_InitValue->SPI_CPHA= SPI_CPHA_1Edge;
		SPI_InitValue->SPI_NSS= SPI_NSS_Soft;
		SPI_InitValue->SPI_BaudRatePrescaler= SPI_BaudRatePrescaler_2;
		SPI_InitValue->SPI_FirstBit= SPI_FirstBit_MSB;
		SPI_InitValue->SPI_CRCPolynomial=7;

		//SPI1 Initialize and Enable
		SPI_Init(SPI1, SPI_InitValue);
		SPI_Cmd(SPI1, ENABLE);
		//trace_printf("DONE Initializing SPI......\n");

}

void mySPI_WriteControl(unsigned int DATA) {

	// 1st Write to SPI
		//trace_printf("START 1st WriteControl to SPI\n");
		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));
		//trace_printf("SPI1 is ready\n");
		// Write to SPI
		SPI_SendData8(SPI1, (control_Disable << 4 | DATA));

		// Wait Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < delay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

	// 2nd Write to SPI

		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (control_Enable << 4 | DATA));

		// W it Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < delay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

	// 3rd Write to SPI

		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (control_Disable << 4 | DATA));

		// Wait Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < delay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

		//trace_printf("DONE WriteControl to SPI\n");
}

//Handshake
void mySPI_WriteData (unsigned int DATA) {

	// 1st Write to SPI
		//trace_printf("START WriteData to SPI\n");
		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;
		//trace_printf("START 1st WriteData to SPI\n");
		// Wait until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE)!= SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (value_Disable << 4 | DATA));

		// Wait until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < delay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

	// 2nd Write to SPI
		//trace_printf("START 2nd WriteData to SPI\n");
		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (value_Enable << 4 | DATA));

		// Wait Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < delay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

	// 3rd Write to SPI
		//trace_printf("START 3rd WriteData to SPI\n");
		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (value_Disable << 4 | DATA));

		// Wait Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < delay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;
		//trace_printf("Done WriteData to SPI\n");
}

void myLCD_Init()	{
	trace_printf("Initializing LCD......\n");
	//LCD TO 4 BIT
	// Configure 8 bit 2 lines

	// Hold interrupts from signal input
	EXTI->IMR &= ~(EXTI_IMR_MR1);
	//trace_printf("interrupt disabled\n");
	// Configure 4-bit interface
	mySPI_WriteControl(2);
	//trace_printf("configure 4 bit interface\n");




	// Populate array of LCD configuration settings
		LCDconf[7] = 2;
		LCDconf[6] = 8;
		LCDconf[5] = 0;
		LCDconf[4] = 12;
		LCDconf[3] = 0;
		LCDconf[2] = 6;
		LCDconf[1] = 0;
		LCDconf[0] = 1;
		//trace_printf("DONE configure LCD array\n");

		///DEBUG THI PART!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		// Write LCD configuration settings
		for(int CONFindex=8; CONFindex<=1; CONFindex--){
			//trace_printf("CONFindex: %d\n", CONFindex);
			mySPI_WriteControl(LCDconf[CONFindex - 1]);
		}
		//trace_printf("END LCD configure settings\n");
		// LCD is 2x8 , system can only output 4 bits at a time ( 8 bits are broken into 4 bits, therefore each row has 16 elements )
		// Initiate row 1
		LCDinit[0][15] = 4;
		LCDinit[0][14] = 6;
		LCDinit[0][13] = 3;
		LCDinit[0][12] = 10;
		LCDinit[0][11] = 2;
		LCDinit[0][10] = 0;
		LCDinit[0][9] = 2;
		LCDinit[0][8] = 0;
		LCDinit[0][7] = 2;
		LCDinit[0][6] = 0;
		LCDinit[0][5] = 2;
		LCDinit[0][4] = 0;
		LCDinit[0][3] = 4;
		LCDinit[0][2] = 8;
		LCDinit[0][1] = 7;
		LCDinit[0][0] = 10;

		// initiate row 2
		LCDinit[1][15] = 5;
		LCDinit[1][14] = 2;
		LCDinit[1][13] = 3;
		LCDinit[1][12] = 10;
		LCDinit[1][11] = 2;
		LCDinit[1][10] = 0;
		LCDinit[1][9] = 2;
		LCDinit[1][8] = 0;
		LCDinit[1][7] = 2;
		LCDinit[1][6] = 0;
		LCDinit[1][5] = 2;
		LCDinit[1][4] = 0;
		LCDinit[1][3] = 4;
		LCDinit[1][2] = 15;
		LCDinit[1][1] = 6;
		LCDinit[1][0] = 8;
		//trace_printf("DONE configure LCD 2D arrays\n");
		// Write default LCD display values
		volatile unsigned int DDRAMindex = 8;
		volatile unsigned int ROWindex = 0;
		//trace_printf("ROWindex While loop\n");
		while (ROWindex < 2) {

			// Set initial DDRAM position
			mySPI_WriteControl(DDRAMindex);
			mySPI_WriteControl(0);

		    volatile int COLUMNindex = 16;
		  //  trace_printf("Column index While loop\n");
		    while(COLUMNindex > 0){
		    	mySPI_WriteData(LCDinit[ROWindex][COLUMNindex - 1]);
		        COLUMNindex--;
		    }
		   // trace_printf("Done COLUMNindex && ROWindex \n");

		   ROWindex++;
		   DDRAMindex = 12;

		}


		// Enable interrupts from signal input
		EXTI->IMR |= EXTI_IMR_MR1;
		//trace_printf("DONE Initializing LCD......");
	}





void my_Res_and_Freq_Init(unsigned int value_loc, int i ){
	//trace_printf("Begin my_Res_and_Freq_Init function\n");
	if (value_loc <= 0) {
	        LCDvalue[i][7] = 3;
	        LCDvalue[i][6] = 0;
	        LCDvalue[i][5] = 3;
	        LCDvalue[i][4] = 0;
	        LCDvalue[i][3] = 3;
	        LCDvalue[i][2] = 0;
	        LCDvalue[i][1] = 3;
	        LCDvalue[i][0] = 0;
	    } else {

	        // Populate output array for display of frequency value.
	        while (value_loc > 0) {
	        	volatile int j = 0;
	            while (j < 8) {
	                // Generate output ASCII value (Upper half, in hexadecimal format).
	            	 LCDvalue[i][j] = ((value_loc % 10) + 30) % 10;
	                j++;
	                // Generate output ASCII value (Lower half, in hexadecimal format).
	                LCDvalue[i][j] = (((value_loc % 10) + 30)/10) % 10;
	                value_loc /= 10;
	                j++;
	            }
	        }
	    }

}

void myValue_Output (){
	//trace_printf("Begin myValue_Output\n");
	// Hold interrupts from signal input
		EXTI->IMR &= ~(EXTI_IMR_MR1);

		//set circumstances when LCD should be changed (if new and existing value different or if existing value greater than tolerance value)
		if ((ADC1->DR > (resistor_fresh + resistor_ability)) || (ADC1->DR < (resistor_fresh - resistor_ability))) {
			resistor_fresh = ADC1->DR;
		}

	    // Assign DAC output value
		DAC->DHR12R1 = resistor_fresh;

		unsigned int freq_loc = frequency_fresh;
		my_Res_and_Freq_Init(freq_loc, 0);

		unsigned int res_loc = resistor_fresh * resistor_prescalar /1000;
		my_Res_and_Freq_Init(res_loc, 1);

		volatile unsigned int DDRAM_pos = 8;
		volatile unsigned int line_index = 0;
		while (line_index < 2) {

			// Set initial DDRAM position
			mySPI_WriteControl(DDRAM_pos);
			mySPI_WriteControl(2);

			volatile int column_index = 8;
			while (column_index > 0) {
				mySPI_WriteData(LCDvalue[line_index][column_index - 1]);
				column_index--;
		}

			line_index++;
		   DDRAM_pos = 12;

		}

		// Enable interrupts from signal input
		EXTI->IMR |= EXTI_IMR_MR1;
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** TIM2 data register overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler(){
	//trace_printf("Begin EXTI\n");
	// Your local variables...
	unsigned int time_count;

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0){
		//trace_printf("Inside EXTI1 interrupt\n");
		if (edge_flag == 0) {
			//trace_printf("edge flag = 0\n");

			TIM2->CNT = 0x00000000;
			time_count = 0;
			//counting timer pulses
			TIM2->CR1 |= TIM_CR1_CEN;

			//setting edge flag
			edge_flag=1;
		} else{
			//trace_printf("edge flag = 1\n");
			// Stop counting timer pulses
			TIM2->CR1 &= ~(TIM_CR1_CEN);

			// Hold interrupts from signal input
			EXTI->IMR &= ~(EXTI_IMR_MR1);

			// Read out count register (TIM2->CNT)
			time_count = TIM2->CNT;

			// Clear edge detection flag
			edge_flag = 0;
			// Ensure displayed data has value
			if (time_count < SystemCoreClock){
				frequency_curr = SystemCoreClock/time_count;

				//set circumstances when LCD should be changed (if new and existing value different or if existing value greater than tolerance value)
				if((frequency_curr > ( frequency_fresh + frequency_ability)) || (frequency_curr < (frequency_fresh -frequency_ability))){
					frequency_fresh = frequency_curr;
				}
				//Output to LCD panel
				myValue_Output();
			}else{
					//trace_printf("edge flag unknown\n");
					frequency_curr = 1000000000 / ( time_count / SystemCoreClock);
					//set circumstances when LCD should be changed (if new and existing value different or if existing value greater than tolerance value)
					if((frequency_curr > ( frequency_fresh + frequency_ability)) || (frequency_curr < (frequency_fresh -frequency_ability))){
							frequency_fresh = frequency_curr;
					}
					myValue_Output();
			}
		}
		//re-enable interrupt
		EXTI->IMR |= EXTI_IMR_MR1;
	}
	//clear EXTI interrupt pending flag
	EXTI->PR |= EXTI_PR_PR1;
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"


// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


// Clock prescaler for TIM2 timer: no prescaling
#define myTIM2_PRESCALER ((uint16_t)0x0)

// Maximum possible setting for overflow
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIO_Init(void);
void myDAC_Init(void);
void myADC_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void mySPI_Init(void);
void myLCD_Init(void);
void mySPI_WriteControl(unsigned int DATA);
void mySPI_WriteData(unsigned int DATA);
void myData_Output(void);

// Input signal edge detection flag
volatile unsigned int edgeFLAG = 0;

// Execution delay variable
volatile unsigned int HOLDdelay = 4800;

// Raw data storage variables
volatile unsigned int freqRAW = 0;
volatile unsigned int resRAW = 0;

// Frequency value LCD update tolerance
volatile unsigned int freqTOLERANCE = 5;

// Resistance value LCD update tolerance and correction prescaler
volatile unsigned int resTOLERANCE = 5;
volatile unsigned int resPRESCALER = 1221;

// LCD data and settings arrays
volatile unsigned int LCDdata[2][8];
volatile unsigned int LCDinit[2][16];
volatile unsigned int LCDconfig[8];

// LCD enable/disable instruction processing settings
volatile unsigned int dataOUTdisable = 0x4;
volatile unsigned int dataOUTenable = 0xC;
volatile unsigned int controlOUTdisable = 0x0;
volatile unsigned int controlOUTenable = 0x8;

int main(int argc, char* argv[]) {

	trace_printf("System clock: %u Hz\n\n", SystemCoreClock);

	myGPIO_Init();		// Initialize I/O ports PA, PB
	myDAC_Init();		// Initialize DAC
	myADC_Init();		// Initialize ADC
	myTIM2_Init();		// Initialize timer TIM2
	myEXTI_Init();		// Initialize EXTI
	mySPI_Init();		// Initialize SPI
	myLCD_Init();		// Initialize LCD display

	trace_printf("\nRunning.\n");

	while (1) {

        // All events triggered by interrupts
	}

	return 0;

}

void myGPIO_Init() {

	// Enable clock for GPIOA & GPIOB peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// Ensure no pull-up/pull-down for PA1, PA3, PA5, PB3, PB5
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

	// Configure PA1 as input for frequency signal, to NE555/3
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	// Configure PA3 as output for LCK signal, to J5/25
	GPIOA->MODER |= GPIO_MODER_MODER3_0;

	// Configure PA4 for analog mode, to timer circuit
	GPIOA->MODER |= GPIO_MODER_MODER4;

	// Configure PA5 for analog mode, from potentiometer
	GPIOA->MODER |= GPIO_MODER_MODER5;

	// Configure PB3 for AF0 mode, to J5/21
	GPIOB->MODER |= GPIO_MODER_MODER3_1;
	GPIOB->AFR[2] &= ~(GPIO_AFRL_AFR3);

	// Configure PB5 for AF0 mode, to J5/17
	GPIOB->MODER |= GPIO_MODER_MODER5_1;
	GPIOB->AFR[2] &= ~(GPIO_AFRL_AFR5);

	trace_printf("GPIO Initialized. \n");
}

void myDAC_Init() {

    // Enable clock for DAC
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // Enable DAC channel 1
    DAC->CR |= DAC_CR_EN1;

	trace_printf("DAC Initialized. \n");
}

void myADC_Init() {

    // Enable clock for ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    // Start calibration of ADC
    ADC1->CR |= ADC_CR_ADCAL;

    while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL);
	trace_printf("ADC Calibrated. \n");

	// Configure ADC for continuous conversion, to retain previous
    // value when overrun is detected, and right-aligned data
    ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);
	ADC1->CFGR1 &= ~(ADC_CFGR1_ALIGN);

    // Enable ADC channel 5
    ADC1->CHSELR = ADC_CHSELR_CHSEL5;
	ADC1->CR |= ADC_CR_ADEN;

    while ((ADC1->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY);
    trace_printf("ADC Initialized. \n");

    // Start ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;

}

void myTIM2_Init() {

	// Enable clock for TIM2 peripheral
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// Configure TIM2: buffer auto-reload, count up, stop on overflow,
    // enable update events, interrupt on overflow only
	TIM2->CR1 = ((uint16_t)0x008C);

	// Set clock prescaler value
	TIM2->PSC = myTIM2_PRESCALER;

    // Set auto-reloaded delay
	TIM2->ARR = myTIM2_PERIOD;

	// Update timer registers
	TIM2->EGR = ((uint16_t)0x0001);

	// Assign TIM2 interrupt priority = 0 in NVIC
	NVIC_SetPriority(TIM2_IRQn, 0);

	// Enable TIM2 interrupts in NVIC
	NVIC_EnableIRQ(TIM2_IRQn);

	// Enable update interrupt generation
	TIM2->DIER |= TIM_DIER_UIE;

	// Start counting timer pulses
	TIM2->CR1 |= TIM_CR1_CEN;

}

void myEXTI_Init() {
	// Map EXTI1 line to PA1
	SYSCFG->EXTICR[0] &= 0x0000FF0F;

	// EXTI1 line interrupts: set rising-edge trigger
	EXTI->RTSR |= EXTI_RTSR_TR1;

	// Unmask interrupts from EXTI1 line
	EXTI->IMR |= EXTI_IMR_MR1;

	// Assign EXTI1 interrupt priority = 0 in NVIC
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	// Enable EXTI1 interrupts in NVIC
	NVIC_EnableIRQ(EXTI0_1_IRQn);

}

void mySPI_Init() {

	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	// Enable clock for SPI
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// Setup values for SPI initialization
	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	// Initialize, enable SPI
	SPI_Init(SPI1, SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);

	trace_printf("SPI Initialized. \n");

}

void mySPI_WriteControl(unsigned int DATA) {

	// 1st Write to SPI

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (controlOUTdisable << 4 | DATA));

		// Wait Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < HOLDdelay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

	// 2nd Write to SPI

		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (controlOUTenable << 4 | DATA));

		// W it Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < HOLDdelay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

	// 3rd Write to SPI

		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (controlOUTdisable << 4 | DATA));

		// Wait Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < HOLDdelay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;
}

void mySPI_WriteData (unsigned int DATA) {

	// 1st Write to SPI

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

		// Wait until SPI1 is ready
		While (((SPI1->SR & SPI_SR_TXE)! = SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (dataOUTdisable << 4 | DATA));

		// Wait until SPI is ready
		While ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < HOLDdelay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

	// 2nd Write to SPI

		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (dataOUTenable << 4 | DATA));

		// Wait Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < HOLDdelay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;

	// 3rd Write to SPI

		// Wait Until SPI1 is ready
		while (((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) || ((SPI1->SR & SPI_SR_BSY) != 0));

		// Write to SPI
		SPI_SendData8(SPI1, (dataOUTdisable << 4 | DATA));

		// Wait Until SPI is ready
		while ((SPI1->SR & SPI_SR_BSY) != 0);

		// Force LCK signal to 1
		GPIOA->BSRR |= GPIO_BSRR_BS_3;

		// Wait for LCD to process data
		for (unsigned int i = 0; i < HOLDdelay; i++);

		// Force LCK signal to 0
		GPIOA->BSRR |= GPIO_BSRR_BR_3;
}

void myLCD_Init() {

	// Hold interrupts from signal input
	EXTI->IMR &= ~(EXTI_IMR_MR1);

	// Configure 4-bit interface
	mySPI_WriteControl(2);

	// Populate array of LCD configuration settings
	LCDconfig[7] = 2;
	LCDconfig[6] = 8;
	LCDconfig[5] = 0;
	LCDconfig[4] = 12;
	LCDconfig[3] = 0;
	LCDconfig[2] = 6;
	LCDconfig[1] = 0;
	LCDconfig[0] = 1;

	// Write LCD configuration settings
	volatile int CONFIGindex = 8;
	while (CONFIGindex > 0) {
		mySPI_WriteControl(LCDconfig[CONFIGindex - 1]);
		CONFIGindex--;
	}

	// Populate array of default LCD frequency display settings
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

	// Populate array of default LCD resistance display settings
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

	// Write default LCD display values
	volatile unsigned int DDRAMindex = 8;
	volatile unsigned int ROWindex = 0;
	while (ROWindex < 2) {

		// Set initial DDRAM position
		mySPI_WriteControl(DDRAMindex);
		mySPI_WriteControl(0);

	    volatile int COLUMNindex = 16;
	    while(COLUMNindex > 0){
	    	mySPI_WriteData(LCDinit[ROWindex][COLUMNindex - 1]);
	        COLUMNindex--;
	    }

	   ROWindex++;
	   DDRAMindex = 12;

	}

	trace_printf("LCD Initialized.\n");

	// Enable interrupts from signal input
	EXTI->IMR |= EXTI_IMR_MR1;

}

// This handler is declared in system/src/cmsis/vectors_stm32f0xx.c
void TIM2_IRQHandler() {

	// Check if update interrupt flag is indeed set
	if ((TIM2->SR & TIM_SR_UIF) != 0) {
		trace_printf("Error: Timer ""TIM2"" data register overflow.\n");

		// Clear update interrupt flag
		TIM2->SR &= ~(TIM_SR_UIF);

		// Restart stopped timer
		TIM2->CR1 |= TIM_CR1_CEN;

	}
}

void myData_Output() {

	// Hold interrupts from signal input
	EXTI->IMR &= ~(EXTI_IMR_MR1);

    // Only change LCD display and DAC output if difference between new value and existing value
    // is greater than tolerance value
	if ((ADC1->DR > (resRAW + resTOLERANCE)) || (ADC1->DR < (resRAW - resTOLERANCE))) {
		resRAW = ADC1->DR;
	}

    // Assign DAC output value
	DAC->DHR12R1 = resRAW;

    // Output "0" if input value is zero.
    unsigned int freqLOCAL = freqRAW;
	if (freqLOCAL <= 0) {
        LCDdata[0][7] = 3;
        LCDdata[0][6] = 0;
        LCDdata[0][5] = 3;
        LCDdata[0][4] = 0;
        LCDdata[0][3] = 3;
        LCDdata[0][2] = 0;
        LCDdata[0][1] = 3;
        LCDdata[0][0] = 0;
    } else {

        // Populate output array for display of frequency value.
        while (freqLOCAL > 0) {
        	volatile int index = 0;
            while (index < 8) {
                // Generate output ASCII value (Upper half, in hexadecimal format).
                LCDdata[0][index] = ((freqLOCAL % 10) + 30) % 10;
                index++;
                // Generate output ASCII value (Lower half, in hexadecimal format).
                LCDdata[0][index] = (((freqLOCAL % 10) + 30)/10) % 10;
                freqLOCAL /= 10;
                index++;
            }
        }
    }

    // Output "0" if input value is zero.
    unsigned int resLOCAL = resRAW * resPRESCALER / 1000;
    if (resLOCAL <= 0) {
    	LCDdata[1][7] = 3;
    	LCDdata[1][6] = 0;
    	LCDdata[1][5] = 3;
    	LCDdata[1][4] = 0;
    	LCDdata[1][3] = 3;
    	LCDdata[1][2] = 0;
    	LCDdata[1][1] = 3;
    	LCDdata[1][0] = 0;

    } else {

        // Populate output array for display of resistance value.
        while (resLOCAL > 0) {
        	volatile int index = 0;
            while (index < 8) {
            	// Generate output ASCII value (Upper half, in hexadecimal format).
            	LCDdata[1][index] = ((resLOCAL % 10) + 30) % 10;
            	index++;
            	// Generate output ASCII value (Lower half, in hexadecimal format).
            	LCDdata[1][index] = (((resLOCAL % 10) + 30)/10) % 10;
            	resLOCAL /= 10;
            	index++;
            }
        }
    }

	// Write LCD display values
    volatile unsigned int DDRAMindex = 8;
    volatile unsigned int ROWindex = 0;
    while (ROWindex < 2) {

    	// Set initial DDRAM position
    	mySPI_WriteControl(DDRAMindex);
    	mySPI_WriteControl(2);

        volatile int COLUMNindex = 8;
        while (COLUMNindex > 0) {
        	mySPI_WriteData(LCDdata[ROWindex][COLUMNindex - 1]);
            COLUMNindex--;
}

       ROWindex++;
       DDRAMindex = 12;

    }

    // Enable interrupts from signal input
    EXTI->IMR |= EXTI_IMR_MR1;
}


// This handler is declared in system/src/cmsis/vectors_stm32f0xx.c
void EXTI0_1_IRQHandler() {

	unsigned int TIMERcount;
	unsigned int freqLOCAL;

	// Check if EXTI1 interrupt pending flag is set
	if ((EXTI->PR & EXTI_PR_PR1) != 0) {

        if (edgeFLAG == 0) {

			// Clear count register and TIMERcount variable
			TIM2->CNT = 0x00000000;
            TIMERcount = 0;

			// Start counting timer pulses
			TIM2->CR1 |= TIM_CR1_CEN;

			// Set edge detection flag
			edgeFLAG = 1;

		} else {

			// Stop counting timer pulses
			TIM2->CR1 &= ~(TIM_CR1_CEN);

			// Hold interrupts from signal input
			EXTI->IMR &= ~(EXTI_IMR_MR1);

			// Read out count register (TIM2->CNT)
			TIMERcount = TIM2->CNT;

            // Clear edge detection flag
            edgeFLAG = 0;

			// Ensure displayed data has value
			if (TIMERcount < SystemCoreClock){

                // Calculate signal frequency
				freqLOCAL = SystemCoreClock / TIMERcount;

                // Only change LCD display if difference between new value and existing value
                // is greater than tolerance value
				if ((freqLOCAL > (freqRAW + freqTOLERANCE)) || (freqLOCAL < (freqRAW - freqTOLERANCE))) {
		  freqRAW = freqLOCAL;
				}

                // Send data to LCD
                myData_Output();

            } else {

                // Calculate signal frequency
		freqLOCAL = 1000000000 / (TIMERcount / SystemCoreClock);

                // Only change LCD display if difference between new value and existing value
                // is greater than tolerance value
			if ((freqLOCAL > (freqRAW + freqTOLERANCE)) || (freqLOCAL < (resRAW - freqTOLERANCE))) {
					freqRAW = freqLOCAL;
				}

                // Send data to LCD
                myData_Output();
			}

			// Enable interrupts from signal input
			EXTI->IMR |= EXTI_IMR_MR1;

		}

		// Clear EXTI1 interrupt pending flag (EXTI->PR)
		EXTI->PR |= EXTI_PR_PR1;

	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------



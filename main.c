#include "stm32f4xx.h"

void delay_ms(uint32_t ms);
unsigned char selectChar(unsigned char dig);
void timer2_init(void);
volatile int displayNum = 0;
volatile int currentDigit = 0;
volatile int toClear = 0;


#define Button1		(GPIOA->IDR & (1<<8)) //for the modes of the washing machine
#define Button2		(GPIOB->IDR & (1<<2)) //increment for the wash timer
int duration =0;
uint8_t WashMode=0; // 0 being as Off and 1 being On
uint8_t pb1Flag=0 , pb2Flag=0; //Debouncing flags

void clock_init_96MHz(void) {
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY));

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	FLASH->ACR |= FLASH_ACR_LATENCY_3WS;

	RCC->PLLCFGR = (12 << RCC_PLLCFGR_PLLM_Pos) | (96 << RCC_PLLCFGR_PLLN_Pos) |
			(0 << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLSRC_HSE | (4 << RCC_PLLCFGR_PLLQ_Pos);

	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void timer2_init(void) {

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->PSC = 16000 - 1;
	TIM2->ARR = 5 - 1;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM2_IRQn);
	__enable_irq();
}

void gpio_init(void) {
	RCC->AHB1ENR |= 0x3;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	//GPIO
	GPIOA->MODER &= ~(0xFFFFFF);
	GPIOA->MODER |= 0x555555;
	GPIOA->MODER &= ~(3<<16);
	GPIOB->MODER &= ~(3 << (6 * 2));
	GPIOB->MODER |=  (2 << (6 * 2));
	GPIOB->MODER |=  (1 << (5 * 2));
	GPIOB->AFR[0] &= ~(0xF << (6 * 4));
	GPIOB->AFR[0] |=  (2 << (6 * 4));

	//GPIO for interrupts
	__disable_irq();
	GPIOB->PUPDR &= ~(3 << (2 * 2));       // Clear PUPDR2
	GPIOB->PUPDR |=  (2 << (2 * 2));       // Set pull-down on PB2

	GPIOA->PUPDR &= ~(3 << (2 * 8));       // Clear PUPDR2
	GPIOA->PUPDR |=  (2 << (2 * 8));       // Set pull-down on PB2

	SYSCFG->EXTICR[0] &= ~(0xF << 8);
	SYSCFG->EXTICR[0] |= (0x1 << 8);  // Set EXTI2 to Port B

	EXTI->IMR |= (1 << 2);
	EXTI->RTSR |= (1 << 2);

	NVIC_EnableIRQ(EXTI2_IRQn);
	//	__enable_irq();
}

void tim4_pwm_init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	TIM4->PSC = 3600 - 1;
	TIM4->ARR = 555 - 1;

	TIM4->CCMR1 |= (6 << 4);
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->CR1 |= TIM_CR1_ARPE;
	TIM4->CCR1 = 20;

	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) {
		unsigned int hexCombination;
		unsigned char tensDigit;
		unsigned char unitsDigit;

		if (toClear == 1) {
			GPIOA->ODR = 0x00;
			toClear = 0;
		} else if (displayNum >= 10) {
			tensDigit = displayNum/10;
			unitsDigit = displayNum%10;
			if (currentDigit == 0) {
				hexCombination = selectChar(tensDigit);
				hexCombination &= ~(1<<9);
				hexCombination |= (1<<10);
				GPIOA->ODR = hexCombination;
				toClear = 1;
				currentDigit = 1;
			} else {
				hexCombination = selectChar(unitsDigit);
				hexCombination &= ~(1<<10);
				hexCombination |= (1<<9);
				GPIOA->ODR = hexCombination;
				toClear = 1;
				currentDigit = 0;
			}
		} else {
			hexCombination = selectChar(displayNum);
			hexCombination &= ~(1<<10);
			hexCombination |= (1<<9);
			GPIOA->ODR = hexCombination;
			toClear = 1;
		}

		TIM2->SR &= ~TIM_SR_UIF;
	}
}


int main(void) {
	gpio_init();
	clock_init_96MHz();
	timer2_init();
	tim4_pwm_init();
	int washCounter = 1;
	displayNum = 88;

	while (1) {
		if (Button1 && !pb1Flag){
			delay_ms(250);
			washCounter++;
			if (WashMode == 0){ // the machine is off
				if (washCounter == 2) {duration = 20;}
				else if (washCounter == 3) {duration = 40;}
				else {duration = 60; washCounter = 1;}
				pb1Flag=1;
				displayNum = duration;
			}
		} else {
			pb1Flag=0; //reseting the flag for debouncing
			}

		//Wash Mode logic
		if (WashMode == 1){
			displayNum = duration;
			delay_ms(1350);
			GPIOB->ODR &= ~(1<<5);
			if (TIM4->CCR1 == 20) {
				TIM4->CCR1 = 71;
			} else {TIM4->CCR1 = 20;}
			duration--;
			if (duration < 0){
				WashMode = 0;
			}
		}else{
			GPIOB->ODR |= (1<<5);    //turn Off the motor
			TIM4->CCR1 = 71;         // Servo max angle
			}
	}
}

void EXTI2_IRQHandler(void)
{

	    WashMode ^= 1; // toggle
	    EXTI->PR = (1 << 2); // clear

}
void delay_ms(uint32_t ms) {
	SysTick->LOAD = 96000 - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = 5;
	for (uint32_t i = 0; i < ms; i++) {
		while ((SysTick->CTRL & 0x10000) == 0);
	}
	SysTick->CTRL = 0;
}
unsigned char selectChar(unsigned char item) {
	if (item >= 0 && item <=9) {
		static const unsigned char digits[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F}; //a list of hex for 0-9
		return digits[item];
	} else if (item == 'E') {
		return 0x79; //hex for E only really here for the interrupt case
	} else {
		return 0x00;
	}
}

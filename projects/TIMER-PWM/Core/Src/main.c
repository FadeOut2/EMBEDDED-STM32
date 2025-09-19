#include "stm32f767xx.h"
#include <stdint.h>

#define TIM3_PSC   95u
#define TIM3_ARR   999u
#define COMMON_ANODE 0

#define FADE_MS    800u   // fade duration in ms
#define WAIT_PERIODS 1u    // 1 update per PWM period (1 ms @ 1kHz)

static void clocks_enable(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->AHB1ENR; (void)RCC->APB1ENR;
}

static void gpio_pc6_pc7_pc8_af2(void){
    // PC6/7/8 -> AF2 TIM3
    GPIOC->MODER &= ~((3u<<(6*2))|(3u<<(7*2))|(3u<<(8*2)));
    GPIOC->MODER |=  ((2u<<(6*2))|(2u<<(7*2))|(2u<<(8*2)));
    GPIOC->OTYPER  &= ~((1u<<6)|(1u<<7)|(1u<<8));
    GPIOC->OSPEEDR |=  ((3u<<(6*2))|(3u<<(7*2))|(3u<<(8*2)));
    GPIOC->PUPDR   &= ~((3u<<(6*2))|(3u<<(7*2))|(3u<<(8*2)));
    GPIOC->AFR[0] &= ~((0xFu<<(6*4))|(0xFu<<(7*4)));
    GPIOC->AFR[0] |=  ((0x2u<<(6*4))|(0x2u<<(7*4)));
    GPIOC->AFR[1] &= ~(0xFu<<((8-8)*4));
    GPIOC->AFR[1] |=  (0x2u<<((8-8)*4));
}

static void tim3_init_pwm(void){
    TIM3->PSC = TIM3_PSC;
    TIM3->ARR = TIM3_ARR;
    TIM3->CR1 |= TIM_CR1_ARPE;

    // CH1 = PC6
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk|TIM_CCMR1_OC1PE);
    TIM3->CCMR1 |=  (6u<<TIM_CCMR1_OC1M_Pos)|TIM_CCMR1_OC1PE;
    // CH2 = PC7 (unused, left disabled)
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk|TIM_CCMR1_OC2PE);
    TIM3->CCMR1 |=  (6u<<TIM_CCMR1_OC2M_Pos)|TIM_CCMR1_OC2PE;
    // CH3 = PC8
    TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk|TIM_CCMR2_OC3PE);
    TIM3->CCMR2 |=  (6u<<TIM_CCMR2_OC3M_Pos)|TIM_CCMR2_OC3PE;

    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;

    TIM3->EGR  = TIM_EGR_UG;
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E;
    TIM3->CR1 |= TIM_CR1_CEN;
}

static void wait_update(uint32_t n){
    while(n--){
        while((TIM3->SR & TIM_SR_UIF)==0){}
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

static inline uint16_t apply_polarity(uint16_t duty){
#if COMMON_ANODE
    return TIM3_ARR - duty;
#else
    return duty;
#endif
}

int main(void){
    clocks_enable();
    gpio_pc6_pc7_pc8_af2();
    tim3_init_pwm();

    uint32_t t_ms = 0;
    uint8_t  swap = 0;

    for(;;){
        wait_update(WAIT_PERIODS);
        t_ms++;
        if(t_ms >= FADE_MS){
            t_ms = 0;
            swap ^= 1; // swap roles
        }

        // duty goes 0..ARR linearly across FADE_MS
        uint16_t duty = (uint16_t)((t_ms * TIM3_ARR) / (FADE_MS - 1u));
        if(duty > TIM3_ARR) duty = TIM3_ARR;
        uint16_t inv = TIM3_ARR - duty;

        if(!swap){
            TIM3->CCR1 = apply_polarity(duty); // PC6 fading up
            TIM3->CCR3 = apply_polarity(inv);  // PC8 fading down
        }else{
            TIM3->CCR1 = apply_polarity(inv);  // PC6 fading down
            TIM3->CCR3 = apply_polarity(duty); // PC8 fading up
        }
    }
}

void _init(void){}
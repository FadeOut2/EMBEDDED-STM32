// STM32F767ZI — TIM3 cross-fade between PC6 and PC8
// PC6 = TIM3_CH1, PC7 = TIM3_CH2 (configured, disabled), PC8 = TIM3_CH3

#include "stm32f767xx.h"
#include <stdint.h>

#define TIM3_PSC     95u     // 96 MHz/(95+1)=1 MHz tick
#define TIM3_ARR     999u    // 1 MHz/(999+1)=1 kHz PWM

#define COMMON_ANODE 0       // 0: common-cathode, 1: common-anode (invert duty)

#define FADE_MS      2000u   // duration of one cross-fade (ms)
#define WAIT_PERIODS 1u      // wait this many PWM periods per step (1 -> ~1 ms per step)

static void clocks_enable(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->AHB1ENR; (void)RCC->APB1ENR;
}

static void gpio_pc6_pc7_pc8_af2(void){
    // PC6/7/8 -> Alternate Function
    GPIOC->MODER &= ~((3u<<(6*2))|(3u<<(7*2))|(3u<<(8*2)));
    GPIOC->MODER |=  ((2u<<(6*2))|(2u<<(7*2))|(2u<<(8*2)));
    GPIOC->OTYPER  &= ~((1u<<6)|(1u<<7)|(1u<<8));                   // Push-pull
    GPIOC->OSPEEDR |=  ((3u<<(6*2))|(3u<<(7*2))|(3u<<(8*2)));       // Very high
    GPIOC->PUPDR   &= ~((3u<<(6*2))|(3u<<(7*2))|(3u<<(8*2)));       // No pull
    // AF2 for TIM3
    GPIOC->AFR[0] &= ~((0xFu<<(6*4))|(0xFu<<(7*4)));
    GPIOC->AFR[0] |=  ((0x2u<<(6*4))|(0x2u<<(7*4)));                // PC6/PC7
    GPIOC->AFR[1] &= ~(0xFu<<((8-8)*4));
    GPIOC->AFR[1] |=  (0x2u<<((8-8)*4));                            // PC8
}

static void tim3_init_pwm(void){
    TIM3->PSC  = TIM3_PSC;
    TIM3->ARR  = TIM3_ARR;
    TIM3->CR1 |= TIM_CR1_ARPE;  // ARR preload

    // CH1 (PC6) PWM1 + preload
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk|TIM_CCMR1_OC1PE);
    TIM3->CCMR1 |=  (6u<<TIM_CCMR1_OC1M_Pos)|TIM_CCMR1_OC1PE;

    // CH2 (PC7) prepared but unused (kept off)
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk|TIM_CCMR1_OC2PE);
    TIM3->CCMR1 |=  (6u<<TIM_CCMR1_OC2M_Pos)|TIM_CCMR1_OC2PE;

    // CH3 (PC8) PWM1 + preload
    TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk|TIM_CCMR2_OC3PE);
    TIM3->CCMR2 |=  (6u<<TIM_CCMR2_OC3M_Pos)|TIM_CCMR2_OC3PE;

    // Start with 0 duty
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;

    TIM3->EGR  = TIM_EGR_UG;            // latch preloads

    // Enable outputs for CH1 and CH3 only
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E;   // (leave CC2E cleared)

    TIM3->CR1 |= TIM_CR1_CEN;           // start timer
}

// wait for n update events (each = one PWM period @ 1 kHz)
static void wait_update(uint32_t n){
    while(n--){
        while((TIM3->SR & TIM_SR_UIF)==0){}
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

static inline uint16_t apply_polarity(uint16_t duty){
#if COMMON_ANODE
    return (TIM3_ARR - duty);
#else
    return duty;
#endif
}

int main(void){
    clocks_enable();
    gpio_pc6_pc7_pc8_af2();
    tim3_init_pwm();

    // cross-fade state
    uint32_t t_ms = 0;          // 0..FADE_MS-1
    uint8_t  swap = 0;          // 0: PC6 fades up, PC8 fades down; 1: swapped

    for(;;){
        wait_update(WAIT_PERIODS);
        // advance "time"
        t_ms++;
        if(t_ms >= FADE_MS){
            t_ms = 0;
            swap ^= 1;          // swap roles every FADE_MS
        }

        // linear ramp 0..ARR across FADE_MS
        // dutyA rises 0->ARR, dutyB falls ARR->0 (perfect cross-fade)
        uint16_t dutyA = (uint16_t)((t_ms * (TIM3_ARR)) / (FADE_MS - 1u));
        if(dutyA > TIM3_ARR) dutyA = TIM3_ARR;
        uint16_t dutyB = (uint16_t)(TIM3_ARR - dutyA);

        if(!swap){
            // PC6 up, PC8 down
            TIM3->CCR1 = apply_polarity(dutyA);  // PC6
            TIM3->CCR3 = apply_polarity(dutyB);  // PC8
        }else{
            // PC6 down, PC8 up
            TIM3->CCR1 = apply_polarity(dutyB);  // PC6
            TIM3->CCR3 = apply_polarity(dutyA);  // PC8
        }
    }
}
void _init(void){}
#include "stm32f7xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "clockerf7/clock.h"

void clock_was_updated(const clock_values_t *current_clocks, clock_status_t status){
    // Сюда приедем, при каждом изменении частоты
    asm("nop");
}
int main(void) {
    // Включаем OverDrive
    HAL_PWREx_EnableOverDrive();
    SCB_EnableICache();
    SCB_EnableDCache();

    clock_set_systick(1000);
    clock_status_t c_status = clock_start_manual(CLOCK_SOURCE_PLL, 25000000, 25, 432, 2);

    HAL_Init();
    __HAL_RCC_GPIOK_CLK_ENABLE();
    GPIO_InitTypeDef g;
    g.Pin = GPIO_PIN_3;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOK, &g);



    while (1) {
        HAL_Delay(1000);
        HAL_GPIO_TogglePin(GPIOK, GPIO_PIN_3);

    }


}

void SysTick_Handler(void) {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
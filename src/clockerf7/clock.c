/**
* \file
* \date 12.08.2018
* \authors Alexander A. Kuzkin <xbevice@gmail.com>
*/

#include "clock.h"
#include <stm32f7xx.h>


#define HSE_FAIL (1<<0)
#define ADVANCED_CLOCK (1<<1) // пользователь сам задаст множители
static volatile uint8_t clock_flags = 0;

clock_values_t clock_values = {
        HSI_VALUE,   // System
        HSI_VALUE,   // AHB
        HSI_VALUE,   // APB1
        HSI_VALUE,   // APB2
        CLOCK_SOURCE_HSI,
        0,           // частота SysTick
        0,           // частота HSE

};
typedef struct {
    clock_ahb_pre_t ahb;
    clock_apb_pre_t apb1;
    clock_apb_pre_t apb2;
} _pre_t;

static _pre_t deviders[3] = {
        {CLOCK_AHB_PRE_1, CLOCK_APB_PRE_1, CLOCK_APB_PRE_1}, // HSI
        {CLOCK_AHB_PRE_1, CLOCK_APB_PRE_1, CLOCK_APB_PRE_1}, // HSE
        {CLOCK_AHB_PRE_1, CLOCK_APB_PRE_4, CLOCK_APB_PRE_2}, // PLL
};


static volatile uint32_t
        _pll_m = 0,
        _pll_p = 0,
        _pll_n = 0; // входящий делитель, выходящий делитель и множитель
static volatile clock_source_t target_source = CLOCK_SOURCE_HSI;
static volatile uint32_t target_pllmul = 0xFF;
static volatile clock_source_t pll_source = CLOCK_SOURCE_HSI;

uint32_t _pll_calc();

clock_status_t clock_select_source(clock_source_t src);

clock_source_t _clock_get_current_source();

void _clock_update();

clock_status_t clock_start_manual(
        clock_source_t source,
        uint32_t hse_clock,
        uint32_t pll_m,
        uint32_t pll_n,
        uint32_t pll_p) {
    uint8_t update_pll = 0;
    if (source == CLOCK_SOURCE_PLL) {
        // проверяем множетили, если будем работать с PLL
        if (pll_m < 2 || pll_m > 63)
            return CLOCK_INVALID;
        if (pll_n < 50 || pll_n > 432)
            return CLOCK_INVALID;
        switch (pll_p) {
            case 2:
            case 4:
            case 6:
            case 8:
                break;
            default:
                return CLOCK_INVALID;
        }

        if (hse_clock / pll_m * pll_n / pll_p > 216000000)
            return CLOCK_INVALID;

        if (_pll_n != pll_n)
            update_pll = 1;
        if (_pll_m != pll_m)
            update_pll = 1;
        if (_pll_p != pll_p)
            update_pll = 1;
    } else {
        _pll_p = 0;
        _pll_m = 0;
        _pll_n = 0;

    }
    __enable_irq(); // включаем прерывания
    RCC->CR |= RCC_CR_CSSON; // включаем защиту
    RCC->CIR |= RCC_CIR_HSERDYIE | RCC_CIR_PLLRDYIE; // включаем прерывание по готовности HSE и PLL
    NVIC_EnableIRQ(RCC_IRQn);

    target_source = source;
    clock_values.HSEClock = hse_clock;


    if (update_pll) {
        // нужно погасить PLL если включен
        _pll_p = pll_p;
        _pll_m = pll_m;
        _pll_n = pll_n;


        if (clock_values.Source == CLOCK_SOURCE_PLL) {
            clock_select_source(CLOCK_SOURCE_HSI); // перейдем на HSI
        }
        if (RCC->CR & RCC_CR_PLLON)
            RCC->CR &= ~RCC_CR_PLLON; // выключим PLL если включен

        // Запишем новый множитель
        //PLL уже выключен, запишем новые данные
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLN_Msk);
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
        RCC->PLLCFGR |= _pll_m;
        RCC->PLLCFGR |= _pll_n << RCC_PLLCFGR_PLLN_Pos;
        RCC->PLLCFGR |= ((_pll_p >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos;
    }
    if (target_source != _clock_get_current_source())
        switch (target_source) {
            case CLOCK_SOURCE_HSE:
                if (clock_select_source(CLOCK_SOURCE_HSE) == CLOCK_NOT_READY)
                    RCC->CR |= RCC_CR_HSEON; // заводим HSE, прерывание уже включено target установлен
                break;
            case CLOCK_SOURCE_PLL:
                if (RCC->CR & RCC_CR_HSERDY) {
                    if (RCC->CR & RCC_CR_PLLRDY)
                        clock_select_source(CLOCK_SOURCE_PLL);
                    else
                        RCC->CR |= RCC_CR_PLLON;

                } else
                    RCC->CR |= RCC_CR_HSEON;// заводим HSE

                break;
            case CLOCK_SOURCE_HSI:
                clock_select_source(CLOCK_SOURCE_HSI); // он типа всегда готов
                break;
        }
    return CLOCK_OK;
}
void _set_flash_latency(uint32_t fq);
void _clock_set_pre(clock_ahb_pre_t ahb_pre, clock_apb_pre_t apb1_pre, clock_apb_pre_t apb2_pre, uint8_t update) {
    // Проверим, не превышаем ли мы частоту APB1?
    if (_clock_get_current_source() == CLOCK_SOURCE_PLL) {
        //TODO: сделать проверку частоты

    }
    // проверим, нужно ли изменить частоты?
    if (((RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos != ahb_pre)
        ||
        ((RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos != apb1_pre)
        ||
        ((RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos != apb2_pre)
            ) {
        _set_flash_latency(216000000);
        RCC->CFGR = (RCC->CFGR & (~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk)))
                    | (ahb_pre << RCC_CFGR_HPRE_Pos) | (apb1_pre << RCC_CFGR_PPRE1_Pos) |
                    (apb2_pre << RCC_CFGR_PPRE2_Pos);
        if (update) {
            _clock_update(); // ну и обновим частоты если нужно
            _set_flash_latency(clock_values.SystemAHBClock);
        }
    }
}

void _set_flash_latency(uint32_t fq) {
    static uint32_t  last_fq =0;
    if(last_fq == fq)
        return;
    last_fq = fq;

    if (fq <= 30000000) {
        FLASH->ACR &= ~(FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN);               // выключаем буффер предвыборки
        FLASH->ACR &= ((uint32_t) ~FLASH_ACR_LATENCY); // выключаем задержку чтения
        return;
    }
    // в любых других случаях
    FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;               // выключаем буффер предвыборки
    uint8_t ws = 0;
    if (fq > 210000000)
        ws = 7;
    else
        while (fq > 30000000U) {
            ws++;
            fq -= 30000000U;

        }
    FLASH->ACR = (FLASH->ACR  & ~ FLASH_ACR_LATENCY_Msk) | ((ws)<<FLASH_ACR_LATENCY_Pos);


}

clock_status_t clock_select_source(clock_source_t src) {
    //с флешем мы считаем, что работаем с 3.3V, поэтому будем будем смотреть соотв. колонку стр 179 RM0385
    if (clock_values.Source == src)
        return CLOCK_OK; // мы как бы уже
    switch (src) {
        default:
        case CLOCK_SOURCE_HSI:
            if (!(RCC->CR & RCC_CR_HSIRDY))
                return CLOCK_NOT_READY;
            RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON);  // потушим PLL и HSE
            RCC->CFGR &= ~RCC_CFGR_SW_Msk;                 // переходим на на HSI

            break;
        case CLOCK_SOURCE_HSE:
            if (!(RCC->CR & RCC_CR_HSERDY))
                return CLOCK_NOT_READY; // HSE не готов, выходим
            RCC->CR &= ~(RCC_CR_PLLON); // потушим PLL
            // переходим на HSE
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_HSE;
            break;
        case CLOCK_SOURCE_PLL:
            if (!(RCC->CR & RCC_CR_PLLRDY))
                return CLOCK_NOT_READY;
            // пока установим максимальную, после обновления всех частот задержка установится оптимальной
            _set_flash_latency(216000000);
            // переходим на PLL
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
            break;

    }
    clock_source_t current_source = _clock_get_current_source();
    _clock_set_pre(deviders[current_source].ahb, deviders[current_source].apb1, deviders[current_source].apb2, 0);
    _clock_update();
    _set_flash_latency(clock_values.SystemAHBClock);

    return CLOCK_OK;
}


static void _update_systick() {
    SysTick_Config(clock_values.SystemAHBClock / clock_values.SysTickClock);

}

void clock_set_systick(uint32_t clock) {
    clock_values.SysTickClock = clock;
    _update_systick();
}

clock_status_t
clock_set_pre(clock_source_t source, clock_ahb_pre_t ahb_pre, clock_apb_pre_t apb1_pre, clock_apb_pre_t apb2_pre) {
    deviders[source].ahb = ahb_pre;
    deviders[source].apb1 = apb1_pre;
    deviders[source].apb2 = apb2_pre;
    if (source == clock_values.Source)
        _clock_set_pre(deviders[clock_values.Source].ahb, deviders[clock_values.Source].apb1,
                       deviders[clock_values.Source].apb2, 1);
    return CLOCK_OK;
}

uint32_t _pll_calc() {
    uint32_t pll_in_fq;
    if (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC)
        pll_in_fq = clock_values.HSEClock;
    else
        pll_in_fq = HSI_VALUE;
    // считаем множители-делители
    uint32_t p = 2U << ((RCC->PLLCFGR & RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos);
    uint32_t n = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
    uint32_t m = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos;

    return pll_in_fq / m * n / p;
}

clock_source_t _clock_get_current_source() {
    switch (RCC->CFGR & RCC_CFGR_SWS_Msk) {
        default:
        case RCC_CFGR_SWS_HSI:
            return CLOCK_SOURCE_HSI;
        case RCC_CFGR_SWS_HSE:
            return CLOCK_SOURCE_HSE;
        case RCC_CFGR_SWS_PLL:
            return CLOCK_SOURCE_PLL;
    }
}

void _clock_update() {
    // пересчитаем все клоки и сурсы

    switch (RCC->CFGR & RCC_CFGR_SWS_Msk) {
        default:
        case RCC_CFGR_SWS_HSI:
            clock_values.Source = CLOCK_SOURCE_HSI;
            SystemCoreClock = HSI_VALUE;
            break;
        case RCC_CFGR_SWS_HSE:
            clock_values.Source = CLOCK_SOURCE_HSE;
            SystemCoreClock = clock_values.HSEClock;
            break;
        case RCC_CFGR_SWS_PLL:
            SystemCoreClock = _pll_calc();
            clock_values.Source = CLOCK_SOURCE_PLL;
            break;
    }

    clock_values.SystemCoreClock = SystemCoreClock;

    if (!(RCC_CFGR_HPRE_3 & RCC->CFGR))
        clock_values.SystemAHBClock = clock_values.SystemCoreClock; // бит сброшен - шина на частоте ядра
    else {
        uint8_t x = (uint8_t) ((RCC->CFGR & (RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_2))
                >> RCC_CFGR_HPRE_Pos);
        if (x > 3)x++;
        clock_values.SystemAHBClock = clock_values.SystemCoreClock >> (x + 1);
    }

    if (!(RCC_CFGR_PPRE1_2 & RCC->CFGR))
        clock_values.SystemAPB1Clock = clock_values.SystemAHBClock;
    else {
        uint8_t x = (uint8_t) ((RCC->CFGR & (RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1)) >> RCC_CFGR_PPRE1_Pos);
        clock_values.SystemAPB1Clock = clock_values.SystemAHBClock >> (x + 1);
    }

    if (!(RCC_CFGR_PPRE2_2 & RCC->CFGR))
        clock_values.SystemAPB2Clock = clock_values.SystemAHBClock;
    else {
        uint8_t x = (uint8_t) ((RCC->CFGR & (RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_1)) >> RCC_CFGR_PPRE2_Pos);
        clock_values.SystemAPB2Clock = clock_values.SystemAHBClock >> (x + 1);
    }

    // обновляем частоту SysTick
    _update_systick();

    // дергаем коллбек
    clock_was_updated(&clock_values, clock_flags & HSE_FAIL ? CLOCK_FAIL : CLOCK_OK);
}


void clock_security_isr() {
    if (RCC->CIR & RCC_CIR_CSSF) {
        RCC->CIR |= RCC_CIR_CSSC; // сбрасываем бит
        clock_flags |= HSE_FAIL;
        clock_source_t current_source = _clock_get_current_source();
        _clock_set_pre(deviders[current_source].ahb, deviders[current_source].apb1, deviders[current_source].apb2, 1);
    }
}


void NMI_Handler(void) {
    clock_security_isr();
}

__attribute__((__weak__))
void clock_was_updated(const clock_values_t *current_clocks, clock_status_t status) {

}


void RCC_IRQHandler(void) {
    // мы сюда прибегаем когда заводятся HSE или PLL
    if (RCC->CIR & RCC_CIR_HSERDYF) {
        RCC->CIR |= RCC_CIR_HSERDYC; // сбрасываем флаг
        if (target_source == CLOCK_SOURCE_PLL && !(RCC->CR & RCC_CR_PLLRDY)) {
            RCC->CR |= RCC_CR_PLLON;
            return;
        }
        clock_select_source(target_source);
    }
    if (RCC->CIR & RCC_CIR_PLLRDYF) {
        RCC->CIR |= RCC_CIR_PLLRDYC;
        clock_select_source(target_source);
    }

    asm("nop;");
}
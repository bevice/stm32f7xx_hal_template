/**
* \file
* \date 12.08.2018
* \authors Alexander A. Kuzkin <xbevice@gmail.com>
*/

#ifndef STM32_CLOCKER_
#define STM32_CLOCKER_

#include <stdint.h>

#define CLOCK_DEFAULT_SYSTICK_FQ 1000


#ifndef HSI_VALUE
#define HSI_VALUE 16000000ul
#endif

typedef enum {
    CLOCK_OK = 0,
    CLOCK_TIMEOUT,
    CLOCK_NOT_READY,
    CLOCK_FAIL,
    CLOCK_INVALID,
} clock_status_t;

typedef enum {
    CLOCK_SOURCE_HSI = 0,
    CLOCK_SOURCE_HSE = 1,
    CLOCK_SOURCE_PLL = 2,
} clock_source_t;





typedef enum {
    CLOCK_APB_PRE_1 = 0,
    CLOCK_APB_PRE_2 = 0b100,
    CLOCK_APB_PRE_4 = 0b101,
    CLOCK_APB_PRE_8 = 0b110,
    CLOCK_APB_PRE_16 = 0b111,
} clock_apb_pre_t;

typedef enum {
    CLOCK_AHB_PRE_1 = 0,
    CLOCK_AHB_PRE_2 = 0b1000,
    CLOCK_AHB_PRE_4 = 0b1001,
    CLOCK_AHB_PRE_8 = 0b1010,
    CLOCK_AHB_PRE_16 = 0b1011,
    CLOCK_AHB_PRE_64 = 0b1100,
    CLOCK_AHB_PRE_128 = 0b1101,
    CLOCK_AHB_PRE_256 = 0b1110,
    CLOCK_AHB_PRE_512 = 0b1111,

} clock_ahb_pre_t;

typedef enum
{
    CLOCK_PLL_P_2 = 0b00,
    CLOCK_PLL_P_4 = 0b01,
    CLOCK_PLL_P_6 = 0b10,
    CLOCK_PLL_P_8 = 0b11,
} pll_p_t;

typedef volatile struct {
    uint32_t SystemCoreClock;
    uint32_t SystemAHBClock;
    uint32_t SystemAPB1Clock;
    uint32_t SystemAPB2Clock;
    uint32_t SysTickClock;
    uint32_t HSEClock;
    clock_source_t Source;

} clock_values_t;

typedef uint32_t clock_pll_mul_t;
extern clock_values_t clock_values;

/**
 * Установка предделителей для шин для выбранного источника.
 * При изменении источника тактовых сингалов переключение предделителей произойдет автоматически
 * в соответсвии с заданными значениями. Если кристалл сейчас работает на source - изменение
 * произойдет незамедлительно. Если частоты изменились после пересчета вызовется коллбек clock_was_updated(...)
 *
 * @param source Выбранный источник
 * @param ahb_pre Делитель для шины AHB (относительно SystemClock)
 * @param apb1_pre Делитель для шины APB1 (относительно AHB),
 *                 если частота шины получается более 36МГц (ahb_pre = 1, apb1_pre = 1)
 *                 делитель будет установлен в CLOCK_APB_PRE_2
 * @param apb2_pre Делитель для шины APB2 (относительно AHB)
 * @return
 */
clock_status_t clock_set_pre(clock_source_t source,
                             clock_ahb_pre_t ahb_pre,
                             clock_apb_pre_t apb1_pre,
                             clock_apb_pre_t apb2_pre);

/**
 * Настройка и запуск тактового генератора. Переход на выбранный источник произойдет сразу же по возможности,
 * или в прерывании по готовности источника. После перехода вызовется коллбек clock_was_updated(...)
 * Если выбран источник HSI - PLL и HSE будет отключен.
 *
 * @param source Выбранный источник тактовых сигналов
 * @param hse_clock частота в герцах внешнего кварцевого резонатора (HSE)
 * @param pll_m входящий делитель [2...63], рекомендованая частота должна лежать в диапазоне 1-2МГц, рекомендована 2МГц
 * @param pll_n множитель, [50...432]
 * @param pll_p выходящий делитель [2,4,6,8]
 * @return
 */
clock_status_t clock_start_manual(
        clock_source_t source,
        uint32_t hse_clock,
        uint32_t pll_m,
        uint32_t pll_n,
        uint32_t pll_p);




/**
 * Коллбек, вызывается при любых изменениях частот.
 * @param current_clocks структура содержащая текущие частоты шин и ядра
 * @param status статус, если произошел срыв генерации HSE/PLL - CLOCK_FAIL
 */
void clock_was_updated(const clock_values_t *current_clocks, clock_status_t status);
void clock_set_systick(uint32_t clock);
#endif

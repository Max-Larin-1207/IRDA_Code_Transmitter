/*********************************************************************************************************************************************
 * IRDA_Code_Transmitter.c
 * Драйвер генератора ИК-сигнала по протоколу NEC (эмуляция пульта ДУ)
 * Программная среда: STM32CubeIDE
 * Автор: Максимус Великолепный, =MxL=
 * Компания: Вертикаль, она же «Палитра»
 * Дата создания: 27 июля 2026
 * Драйвер написан без использования дебильной библиотеки HAL, непосредственно на регистрах
 ********************************************************************************************************************************************/

#include "IRDA_Code_Transmitter.h"		// Привязка программы к заголовочному файлу

// Драйвер активируется, если он разрешён программой IR_TRANSMITTER_EN, а также определён таймер IRDA_TIMER и выход IR_TX_OUT_Pin:
#if IR_TRANSMITTER_EN && defined (IRDA_TIMER) && defined (IR_TX_OUT_Pin)

// Установки дополнительных макросов портов для макросов управления пинами МК:
#if IR_TRASMIT_OUT_POLAR_MODE
// Для инвертированного режима выхода:
#define IR_TX_OUT_Pin_ON	RESET_PORT_PIN (IR_TX_OUT_GPIO_Port, IR_TX_OUT_Pin)
#define IR_TX_OUT_Pin_OFF	SET_1_PORT_PIN (IR_TX_OUT_GPIO_Port, IR_TX_OUT_Pin)
#else
// Для прямого режима выхода:
#define	IR_TX_OUT_Pin_ON	SET_1_PORT_PIN (IR_TX_OUT_GPIO_Port, IR_TX_OUT_Pin)
#define IR_TX_OUT_Pin_OFF	RESET_PORT_PIN (IR_TX_OUT_GPIO_Port, IR_TX_OUT_Pin)
#endif

// Объявление констант:
#if IR_TRASMIT_GPIO_DR_INIT_EN
// Маска регистров конфигурации порта:
static const uint32_t IR_TX_OUT_Pin_Mode_Mask = GPIO_REG_PIN_MASK (IR_TX_OUT_Pin);
#if (GPIO_CONF_REG_TYPE == 1)
static const volatile uint32_t *IR_TX_OUT_Conf_Reg_Pntr = (IR_TX_OUT_Pin < 0x0100) ? &IR_TX_OUT_GPIO_Port->CRL : &IR_TX_OUT_GPIO_Port->CRH;
#endif
#endif

// Объявление общей структуры данных:
IR_Trsmt_Data_Struct IR_Trsmt_Data = {0};


/*** Внутренние функции *********************************************************************************************************************/

__attribute__((always_inline)) static inline void IR_TX_Set_Timer_ARR (uint32_t duration_us)	// Установка счётного регистра таймера
{
	TIMER (IRDA_TIMER)->ARR = duration_us - 1;
}

static void IR_TX_Start_Frame (void)	// Запуск нового кадра (стартовый импульс)
{
	IR_TX_OUT_Pin_ON;		// Включение выхода
	IR_TX_Set_Timer_ARR (NEC_LEAD_PULSE);
	IR_Trsmt_Data.mode = 1; // Переход на: Стартовый импульс
}


/*** Внешние функции драйвера ***************************************************************************************************************/

void IR_Transmitter_Init (void)	// Инициализация и настройка вывода и таймера
{
#if IR_TRASMIT_GPIO_DR_INIT_EN
#if (GPIO_CONF_REG_TYPE == 1)	// Для F1 серий
	*(uint32_t*) IR_TX_OUT_Conf_Reg_Pntr &= ~ IR_TX_OUT_Pin_Mode_Mask;
	*(uint32_t*) IR_TX_OUT_Conf_Reg_Pntr |= GPIO_CONF_OUTPUT_PP_MEDIUM << GPIO_REG_PIN_POS (IR_TX_OUT_Pin);
#elif (GPIO_CONF_REG_TYPE == 2)	// Для остальных серий
	IR_TX_OUT_GPIO_Port->MODER &= ~ IR_TX_OUT_Pin_Mode_Mask;
	IR_TX_OUT_GPIO_Port->MODER |= GPIO_CONF_MODER_OUTPUT << GPIO_REG_PIN_POS (IR_TX_OUT_Pin);
	IR_TX_OUT_GPIO_Port->OTYPER &= ~ IR_TX_OUT_Pin;
	IR_TX_OUT_GPIO_Port->OSPEEDR |= GPIO_CONF_OSPEEDR_MEDIUM << GPIO_REG_PIN_POS (IR_TX_OUT_Pin);
	IR_TX_OUT_GPIO_Port->PUPDR &= ~ IR_TX_OUT_Pin_Mode_Mask;
#endif
#endif

	IR_TX_OUT_Pin_OFF;	// Начальное состояние выхода - выключено

	// Инициализация таймера IRDA_TIMER:
	TIM_CLK_ENABLE (IRDA_TIMER);	// Включение тактирования (активация) таймера

	// Настройки таймера:
#if	TIM_CLOCK_DIV1 || TIM_COUNT_MODE_UP
	TIMER (IRDA_TIMER)->CR1 |= TIM_CLOCK_DIV1 | TIM_COUNT_MODE_UP;
#endif
	uint32_t IRDA_TIMER_prescaler = SystemCoreClock / 1000000U;
	TIMER (IRDA_TIMER)->PSC = IRDA_TIMER_prescaler - 1;	// Прескаляр на 1 мкс
	TIMER (IRDA_TIMER)->ARR = 1000;	// Таймер в режиме отсчета милисекунд
	TIMER (IRDA_TIMER)->DIER |= TIM_IT_UPD_EN;	// Включение прерываний по переполнению

	TIM_STOP (IRDA_TIMER);	// Таймер пока не запускаем

	// Установка прерываний таймера:
	__NVIC_SetPriority (TIM_IRQn (IRDA_TIMER), IRDA_TIMER_IRQ_PRIORITY);
	__NVIC_EnableIRQ (TIM_IRQn (IRDA_TIMER));
}

void IR_Transmitter_Start (uint32_t address, uint32_t command, bool repeat_en, uint8_t repeat_cnt)	// Отправка кода NEC
{
	if (IR_Trsmt_Data.mode) IR_Transmitter_Stop ();	// Останавливаем прежнюю передачу, если была запущена

	// Начальные установки генерации кода:
	IR_Trsmt_Data.tx_code = 	// Формирование 32-битного кода NEC
#if IR_TRASMIT_CODE_16BIT_MODE
		((address & 0xFFFF) << 16) | (command & 0xFFFF);
#else
		((address & 0xFF) << 24) | (((~address) & 0xFF) << 16) | ((command & 0xFF) << 8) | ((~command) & 0xFF);
#endif
	IR_Trsmt_Data.repeat_en = repeat_en;
	IR_Trsmt_Data.repeat_cnt = repeat_cnt;
	IR_Trsmt_Data.bit_cnt = 32;				// 32 бита данных

	TIM_RESET (IRDA_TIMER);		// Начальный сброс таймера

	IR_TX_Start_Frame ();		// Запуск первого кадра

	TIM_RUN (IRDA_TIMER);		// Запуск таймера
}

void IR_Transmitter_Stop (void)	// Остановка генерации кода
{
	TIM_STOP (IRDA_TIMER);
	IR_TX_OUT_Pin_OFF;
	IR_Trsmt_Data.mode = 0;
	IR_Trsmt_Data.repeat_en = false;
}

void IR_Transmitter_Timer_IRQHandler (void)	// Обработчик прерываний таймера IRDA_TIMER
{
	// Переход из предыдущих режимов в новый:
	switch (IR_Trsmt_Data.mode)
	{
	case 1:	// Стартовый импульс (9 мс) закончился:
		IR_TX_OUT_Pin_OFF;		// Выключение выхода
		IR_TX_Set_Timer_ARR ((IR_Trsmt_Data.repeat_en && (IR_Trsmt_Data.bit_cnt == 0)) ? NEC_REPEAT_SPACE : NEC_LEAD_SPACE);
		IR_Trsmt_Data.mode = 2;	// Переход на: Стартовая пауза
		break;
	case 2:	// Стартовая пауза закончилась:
		IR_TX_OUT_Pin_ON;		// Включение выхода
		IR_TX_Set_Timer_ARR (NEC_BIT_PULSE);
		IR_Trsmt_Data.mode = 3;	// Переход на: Импульс бита
		break;
	case 3:	// Импульс бита закончился:
		IR_TX_OUT_Pin_OFF;		// Выключение выхода
		bool act_bit = (IR_Trsmt_Data.tx_code >> IR_Trsmt_Data.bit_cnt) & 1;	// Читаем текущий бит
		IR_TX_Set_Timer_ARR (act_bit ? NEC_BIT_ONE_SPACE : NEC_BIT_ZERO_SPACE);
		IR_Trsmt_Data.mode = 4;	// Переход на: Пауза между битами
		break;
	case 4:	// Пауза между битами закончилась:
		IR_TX_OUT_Pin_ON;		// Включение выхода
		IR_Trsmt_Data.bit_cnt --;	// Отсчёт битов
		// Передача всех битов продолжается:
		if (IR_Trsmt_Data.bit_cnt > 0)
		{
			IR_TX_Set_Timer_ARR (NEC_BIT_PULSE);
			IR_Trsmt_Data.mode = 3;	// Переход на: Импульс бита (0.56 мс carrier)
		}
		// Все биты переданы:
		else
		{
			IR_TX_Set_Timer_ARR (NEC_STOP_PULSE);
			IR_Trsmt_Data.mode = 5; // Переход на: Стоп-импульс
		}
		break;
	case 5:	// Стоп-импульс:
		IR_TX_OUT_Pin_OFF;			// Выключение выхода
		IR_Trsmt_Data.repeat_timer = IR_REPEAT_INTERVAL_MS - 1;		// Устанавливаем таймер паузы
		IR_TX_Set_Timer_ARR (1000);	// Превращаем ведущий таймер в таймер милисекунд
		IR_Trsmt_Data.mode = 6;		// Переход на: Пауза между кодами
		TIM_RESET (IRDA_TIMER);		// Начальный сброс таймера
		break;
	case 6:	// Пауза после кода закончилась:
		if (IR_Trsmt_Data.repeat_timer) IR_Trsmt_Data.repeat_timer --;	// Отсчет таймера автоповтора
		// Отправление повтора и переключение на стартовый импульс:
		else if (IR_Trsmt_Data.repeat_en)
		{
			if (IR_Trsmt_Data.repeat_cnt)
			{
				IR_Trsmt_Data.repeat_cnt --;
				if (IR_Trsmt_Data.repeat_cnt) IR_TX_Set_Lead_Pulse ();	// Начало генерации кода, установка стартового импульса (9 мс carrier)
				else IR_Transmitter_Stop ();	// Повторы закончились, остановка генерации кода
			}
			// Бесконечная отправка повторов:
			else IR_TX_Set_Lead_Pulse ();	// Начало генерации кода, установка стартового импульса (9 мс carrier)
		}
		else IR_Transmitter_Stop ();	// Одиночный код закончен
		break;
	default: IR_Transmitter_Stop ();	// Невпопад, остановка генерации кода
	}

	TIM_IRQ_UPD_CLEAR (IRDA_TIMER);	// Сброс флага прерывания по переполнению
}

#endif /* IR_TRANSMITTER_EN && defined (IRDA_TIMER) && defined (IR_TX_OUT_Pin) */

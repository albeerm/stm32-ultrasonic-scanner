/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>

#define STEP_TRIGGER 10

ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

enum modos { MANUAL, AUTOMATICO };
volatile enum modos modo;

enum medEstados { IDLE, MEASURING };
volatile enum medEstados estadoL;

uint32_t valor_adc;
uint32_t prev_adc;
volatile uint16_t t1 = 0;
volatile uint16_t t2 = 0;
volatile uint16_t pulse_width = 0;
volatile uint8_t rising_edge = 1;
volatile uint16_t distance_cm = 0;
uint32_t light_mode = 0;
volatile uint8_t distance_ready = 0;
uint32_t blink_counter;
uint32_t auto_blink;
volatile int servo_angle = 0;
int auto_positions[5] = {-90, -45, 0, 45, 90};
volatile uint8_t auto_index = 0;
volatile uint32_t auto_counter = 0;
volatile uint8_t auto_running = 0;
volatile uint32_t auto_timer = 0;
volatile uint16_t auto_distances[5];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

void EXTI15_10_IRQHandler(void);
void EXTI4_IRQHandler(void);
void ADC1_IRQHandler(void);
void TIM3_IRQHandler(void);
void servo_set_auto(int angle);
void send_trigger(void);
int _write(int file, char *ptr, int len);

void ledMON(void) { GPIOA->BSRR = (1 << 5); }
void ledMOFF(void) { GPIOA->BSRR = (1 << 5) << 16; }
void ledMedON(void) { GPIOB->BSRR = (1 << 5); }
void ledMedOFF(void) { GPIOB->BSRR = (1 << 5) << 16; }
void led1ON(void) { GPIOC->BSRR = (1 << 0); }
void led1OFF(void) { GPIOC->BSRR = (1 << 0) << 16; }
void led2ON(void) { GPIOC->BSRR = (1 << 1); }
void led2OFF(void) { GPIOC->BSRR = (1 << 1) << 16; }
void led3ON(void) { GPIOC->BSRR = (1 << 2); }
void led3OFF(void) { GPIOC->BSRR = (1 << 2) << 16; }
void led4ON(void) { GPIOC->BSRR = (1 << 3); }
void led4OFF(void) { GPIOC->BSRR = (1 << 3) << 16; }
void led5ON(void) { GPIOC->BSRR = (1 << 4); }
void led5OFF(void) { GPIOC->BSRR = (1 << 4) << 16; }

void EXTI15_10_IRQHandler(void)
{
    uint16_t lectura = (EXTI->PR & (1 << 13));
    if (lectura != 0) {
        if (auto_running == 0) {
            modo = (modo + 1) % 2;
        }
        EXTI->PR |= (1 << 13);
    }
}

void EXTI4_IRQHandler(void)
{
    uint16_t read = (EXTI->PR & (1 << 4));
    if (read != 0) {
        estadoL = (estadoL + 1) % 2;
        EXTI->PR |= (1 << 4);
    }
}

void ADC1_IRQHandler(void)
{
    uint16_t lectura_adc = (ADC1->SR & (1 << 1));
    if (lectura_adc != 0) {
        valor_adc = ADC1->DR;
    }
}

void TIM3_IRQHandler(void)
{
    uint16_t lectura_led = (TIM3->SR & (1 << 2));
    uint16_t lectura_trigger = (TIM3->SR & (1 << 3));
    uint16_t lectura_echo = (TIM3->SR & (1 << 4));

    if (lectura_led != 0) {
        blink_counter++;
        if (modo == MANUAL) {
            if (blink_counter >= 10) {
                ledMedOFF();
                TIM3->DIER &= ~(1 << 2);
            }
        }
        if (modo == AUTOMATICO && auto_blink) {
            if (blink_counter >= 10) {
                if (GPIOB->ODR & (1 << 5)) ledMedOFF();
                else ledMedON();
                blink_counter = 0;
            }
            if (auto_running) {
                auto_timer++;
                if (auto_timer >= 40) {
                    auto_index++;
                    if (auto_index < 5) {
                        servo_set_auto(auto_positions[auto_index]);
                        printf("Angle position: %d\r\n", auto_positions[auto_index]);
                        send_trigger();
                    } else {
                        auto_running = 0;
                        auto_blink = 0;
                        servo_set_auto(0);
                        ledMedOFF();
                        printf("Distance: [");
                        for (int i = 0; i < 5; i++) {
                            if (auto_distances[i] >= 200) printf("X");
                            else printf("%d", auto_distances[i]);
                            if (i < 4) printf(",");
                        }
                        printf("] cm \r\n");
                    }
                    auto_timer = 0;
                }
            }
        }
        TIM3->CCR2 = TIM3->CNT + 50000;
        TIM3->SR &= ~(1 << 2);
    }

    if (lectura_trigger != 0) {
        TIM3->CCMR2 &= ~(7 << 4);
        TIM3->CCMR2 |= (4 << 4);
        TIM3->CCER &= ~(1 << 8);
        TIM3->SR &= ~(1 << 3);
    }

    if (lectura_echo != 0) {
        if (rising_edge == 1) {
            t1 = TIM3->CCR4;
            TIM3->CCER |= (1 << 13);
            rising_edge = 0;
        } else {
            t2 = TIM3->CCR4;
            TIM3->CCER &= ~(1 << 13);
            rising_edge = 1;
        }
        if (t2 != 0) {
            if (t2 < t1) t2 += 0xFFFF;
            pulse_width = t2 - t1;
        }
        TIM3->SR &= ~(1 << 4);
    }
}

void start_manual_led(void)
{
    ledMedON();
    blink_counter = 0;
    auto_blink = 0;
    TIM3->DIER |= (1 << 2);
    TIM3->CCR2 = TIM3->CNT + 50000;
}

void start_auto_led(void)
{
    blink_counter = 0;
    auto_blink = 1;
    TIM3->DIER |= (1 << 2);
    TIM3->CCR2 = TIM3->CNT + 50000;
}

void check_angle(void)
{
    int16_t angle = ((valor_adc * 180) / 4095) - 90;
    printf("The angle position:%d\r\n", angle);
}

void send_trigger(void)
{
    TIM3->CCMR2 &= ~(7 << 4);
    TIM3->CCMR2 |= (5 << 4);
    TIM3->CCER |= (1 << 8);
    TIM3->CCR3 = TIM3->CNT + STEP_TRIGGER;
}

void check_distance(void)
{
    if (t1 != 0 && t2 != 0) {
        if (pulse_width > 38000) distance_cm = 200;
        else distance_cm = pulse_width / 58;
        t1 = 0;
        t2 = 0;
        distance_ready = 1;
    }
}

void check_dmode(void)
{
    if (distance_cm <= 10) light_mode = 5;
    else if (distance_cm <= 20) light_mode = 4;
    else if (distance_cm <= 40) light_mode = 3;
    else if (distance_cm <= 60) light_mode = 2;
    else if (distance_cm <= 80) light_mode = 1;
    else light_mode = 0;
}

void set_dmode(void)
{
    switch (light_mode) {
    case 5:
        led1ON(); led2ON(); led3ON(); led4ON(); led5ON();
        break;
    case 4:
        led1ON(); led2ON(); led3ON(); led4ON(); led5OFF();
        break;
    case 3:
        led1ON(); led2ON(); led3ON(); led4OFF(); led5OFF();
        break;
    case 2:
        led1ON(); led2ON(); led3OFF(); led4OFF(); led5OFF();
        break;
    case 1:
        led1ON(); led2OFF(); led3OFF(); led4OFF(); led5OFF();
        break;
    default:
        led1OFF(); led2OFF(); led3OFF(); led4OFF(); led5OFF();
        break;
    }
}

void servo_set_angle(void)
{
    uint16_t pulse = 1000 + (valor_adc * 1000) / 4095;
    if (pulse < 1000) pulse = 1000;
    if (pulse > 2000) pulse = 2000;
    TIM2->CCR1 = pulse;
}

void servo_set_auto(int angle)
{
    uint16_t pulse = 1500 + (angle * 500) / 90;
    if (pulse < 1000) pulse = 1000;
    if (pulse > 2000) pulse = 2000;
    TIM2->CCR1 = pulse;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

    GPIOC->MODER &= ~(1 << (0 * 2 + 1));
    GPIOC->MODER |= (1 << (0 * 2));
    GPIOC->MODER &= ~(1 << (1 * 2 + 1));
    GPIOC->MODER |= (1 << (1 * 2));
    GPIOC->MODER &= ~(1 << (2 * 2 + 1));
    GPIOC->MODER |= (1 << (2 * 2));
    GPIOC->MODER &= ~(1 << (3 * 2 + 1));
    GPIOC->MODER |= (1 << (3 * 2));
    GPIOC->MODER &= ~(1 << (4 * 2 + 1));
    GPIOC->MODER |= (1 << (4 * 2));

    GPIOA->MODER |= (1 << (1 * 2 + 1));
    GPIOA->MODER |= (1 << (1 * 2));
    ADC1->CR2 = 0;
    ADC1->CR1 &= ~(3 << 24);
    ADC1->CR1 |= (1 << 5);
    ADC1->CR1 &= ~(1 << 8);
    ADC1->CR2 &= ~(1 << 30);
    ADC1->CR2 |= (1 << 10);
    ADC1->CR2 &= ~(1 << 11);
    ADC1->CR2 |= (1 << 1);
    ADC1->CR2 |= (1 << 4);
    ADC1->SQR1 = 0;
    ADC1->SQR5 = 0x00000001;
    ADC1->CR2 |= (1 << 0);
    while ((ADC1->SR & 0x0040) == 0) ;
    ADC1->CR2 |= 0x40000000;
    NVIC->ISER[0] |= (1 << 18);

    GPIOA->MODER &= ~(1 << (5 * 2 + 1));
    GPIOA->MODER |= (1 << (5 * 2));
    GPIOC->MODER &= ~(1 << (13 * 2 + 1));
    GPIOC->MODER &= ~(1 << (13 * 2));
    GPIOB->MODER &= ~(1 << (5 * 2 + 1));
    GPIOB->MODER |= (1 << (5 * 2));
    GPIOB->MODER &= ~(1 << (4 * 2 + 1));
    GPIOB->MODER &= ~(1 << (4 * 2));
    GPIOB->PUPDR &= ~(3 << (4 * 2));
    GPIOB->PUPDR |= (2 << (4 * 2));

    SYSCFG->EXTICR[3] &= ~(0xF << 4);
    SYSCFG->EXTICR[3] |= (0x2 << 4);
    EXTI->IMR |= (1 << 13);
    EXTI->RTSR |= (1 << 13);
    EXTI->FTSR &= ~(1 << 13);
    NVIC->ISER[1] |= (1 << 8);

    SYSCFG->EXTICR[1] &= ~(0xF << 0);
    SYSCFG->EXTICR[1] |= (0x1 << 0);
    EXTI->IMR |= (1 << 4);
    EXTI->RTSR |= (1 << 4);
    EXTI->FTSR &= ~(1 << 4);
    NVIC->ISER[0] |= (1 << 10);

    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->SMCR = 0;
    TIM3->PSC = 31;
    TIM3->CNT = 0;
    TIM3->ARR = 0xFFFF;
    TIM3->CCMR1 = 0;
    TIM3->CCER &= ~(1 << 5);
    TIM3->DIER |= (1 << 2);

    GPIOC->MODER &= ~(3 << (8 * 2));
    GPIOC->MODER |= (2 << (8 * 2));
    GPIOC->AFR[1] &= ~(0xF << (0 * 4));
    GPIOC->AFR[1] |= (0x2 << (0 * 4));
    TIM3->CCMR2 &= ~(3 << 0);
    TIM3->CCMR2 &= ~(1 << 3);
    TIM3->CCER &= ~(1 << 9);
    TIM3->CCER &= ~(1 << 8);
    TIM3->DIER |= (1 << 3);

    GPIOC->MODER &= ~(3 << (9 * 2));
    GPIOC->MODER |= (2 << (9 * 2));
    GPIOC->AFR[1] &= ~(0xF << (1 * 4));
    GPIOC->AFR[1] |= (0x2 << (1 * 4));
    TIM3->CCMR2 &= ~(3 << 8);
    TIM3->CCMR2 |= (1 << 8);
    TIM3->CCER &= ~(1 << 13);
    TIM3->CCER |= (1 << 12);
    TIM3->DIER |= (1 << 4);
    TIM3->EGR |= 1;
    TIM3->SR = 0;
    TIM3->CR1 |= 1;
    NVIC->ISER[0] |= (1 << 29);

    TIM2->CR1 = 0;
    TIM2->CR2 = 0;
    TIM2->SMCR = 0;
    TIM2->PSC = 31;
    TIM2->CNT = 0;
    TIM2->ARR = 20000;
    GPIOA->MODER &= ~(3 << (0 * 2));
    GPIOA->MODER |= (2 << (0 * 2));
    GPIOA->AFR[0] &= ~(0xF << (0 * 4));
    GPIOA->AFR[0] |= (1 << (0 * 4));
    TIM2->CCMR1 &= ~(7 << 4);
    TIM2->CCMR1 |= (6 << 4);
    TIM2->CCMR1 |= (1 << 3);
    TIM2->CCER |= (1 << 0);
    TIM2->CCR1 = 1500;
    TIM2->CR1 |= (1 << 7);
    TIM2->EGR |= 1;
    TIM2->CR1 |= 1;

    ledMOFF();
    modo = MANUAL;
    enum modos Modo_prev = -1;
    enum medEstados Med_prev = -1;
    estadoL = IDLE;

    while (1) {
        if (modo != Modo_prev) {
            Modo_prev = modo;
            switch (modo) {
            case MANUAL:
                ledMOFF();
                printf("Modo manual\r\n");
                servo_set_angle();
                break;
            case AUTOMATICO:
                ledMON();
                printf("Modo automatico\r\n");
                break;
            }
        }
        if (modo == MANUAL) servo_set_angle();

        if (estadoL != Med_prev) {
            Med_prev = estadoL;
            switch (estadoL) {
            case IDLE:
                break;
            case MEASURING:
                printf("Inicio Medida:\r\n");
                if (modo == MANUAL) {
                    start_manual_led();
                    send_trigger();
                    check_angle();
                }
                if (modo == AUTOMATICO) {
                    start_auto_led();
                    auto_running = 1;
                    auto_index = 0;
                    auto_timer = 0;
                    for (int i = 0; i < 5; i++) auto_distances[i] = 0;
                    servo_set_auto(auto_positions[auto_index]);
                    printf("Angle position: %d\r\n", auto_positions[auto_index]);
                    send_trigger();
                }
                estadoL = IDLE;
                break;
            }
        }

        check_distance();
        if (distance_ready) {
            check_dmode();
            set_dmode();
            if (modo == AUTOMATICO && auto_running) {
                if (distance_cm >= 200) auto_distances[auto_index] = 200;
                else auto_distances[auto_index] = distance_cm;
            }
            if (distance_cm < 200) printf("Distance: %d cm\r\n", distance_cm);
            else printf("Distance: No obstacle detected\r\n");
            distance_ready = 0;
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

static void MX_ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
    hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
    hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.NbrOfConversion = 1;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    if (HAL_ADC_Init(&hadc) != HAL_OK) Error_Handler();
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM4_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) Error_Handler();
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

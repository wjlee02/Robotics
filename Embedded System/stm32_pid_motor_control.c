#include "main.h"
#include <math.h>

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

volatile float target_angle = 0.0;
volatile float current_angle = 0.0;
volatile float error = 0.0;
volatile float integral = 0.0;
volatile float derivative = 0.0;
volatile float previous_error = 0.0;
volatile float angle_per_transition = 0.6923;

volatile float KP = 500.0;
volatile float KI = 0.1;
volatile float KD = 1.4;

volatile uint32_t encA_prev = 0;
volatile uint32_t encB_prev = 0;

volatile uint32_t encA_curr = 0;
volatile uint32_t encB_curr = 0;

uint32_t duty = 0;

volatile uint32_t current_time = 0;
volatile uint32_t prevTime = 0;
volatile float dt = 0.0f;

volatile uint32_t last_interrupt_time = 0;

volatile uint8_t experiment_started = 0;

#define DEBOUNCE_DELAY 1

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void update_motor_control();
void Update_Target_Angle(uint32_t current_time);
void Forward(void);
void Backward(void);
void Stop(void);

void update_motor_control() {
    dt = (current_time - prevTime) / 1000.0f;
    prevTime = current_time;

    error = target_angle - current_angle;

    float P = KP * error;
    integral += KI * error * dt;
    float D = KD * (error - previous_error) / dt;

    float outputPID = P + integral + D;

    outputPID = outputPID < 0.0 ? -outputPID : outputPID;
    outputPID = outputPID / 360.0;
    outputPID = 1.0 < outputPID ? 1.0 : outputPID;

    duty = (uint32_t)(outputPID * htim2.Init.Period);

    if (0 < error) {
        Forward();
        TIM2->CCR1 = duty;
    } else if (0 > error) {
        Backward();
        TIM2->CCR1 = duty;
    } else {
        Stop();
    }

    previous_error = error;
}

void Update_Target_Angle(uint32_t current_time) {
      
      if (!experiment_started) {
        target_angle = 0.0f;
        return;
    }
      
    if (current_time <= 41000) {
        if (current_time <= 21000) {
            if (current_time < 1000) target_angle = 0.0f;
            else if (current_time < 2000) target_angle = 180.0f;
            else if (current_time < 3000) target_angle = 0.0f;
            else if (current_time < 4000) target_angle = -180.0f;
            else if (current_time < 5000) target_angle = 0.0f;
            else if (current_time < 6000) target_angle = 360.0f;
            else if (current_time < 7000) target_angle = 0.0f;
            else if (current_time < 8000) target_angle = -360.0f;
            else if (current_time < 9000) target_angle = 0.0f;
            else if (current_time < 10000) target_angle = 120.0f;
            else if (current_time < 11000) target_angle = 240.0f;
            else if (current_time < 12000) target_angle = 360.0f;
            else if (current_time < 13000) target_angle = 240.0f;
            else if (current_time < 14000) target_angle = 120.0f;
            else if (current_time < 15000) target_angle = 0.0f;
            else if (current_time < 16000) target_angle = -120.0f;
            else if (current_time < 17000) target_angle = -240.0f;
            else if (current_time < 18000) target_angle = -360.0f;
            else if (current_time < 19000) target_angle = -240.0f;
            else if (current_time < 20000) target_angle = -120.0f;
            else if (current_time < 21000) target_angle = 0.0f;
        } else {
            if (current_time >= 21000 && current_time < 27000) {
                target_angle = -120.0f * sinf((current_time - 21000) * 0.001f);
            } else if (current_time >= 27000 && current_time < 33000) {
                target_angle = -240.0f * sinf((current_time - 27000) * 0.001f);
            } else if (current_time >= 33000 && current_time < 40000) {
                target_angle = -360.0f * sinf((current_time - 33000) * 0.001f);
            } else {
                target_angle = 0.0f;
            }
        }
    } else {
        target_angle = current_angle;
    }
}

void Forward(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

void Backward(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

void Stop(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

int main(void) {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_Base_Stop_IT(&htim3);
      
      experiment_started = 0;
      
    while (!experiment_started) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) {
            experiment_started = 1;
            HAL_TIM_Base_Start_IT(&htim3);
            current_time = HAL_GetTick();
            prevTime = current_time;
        }
    }
      while (1) {
      }   
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 63;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 63;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM3 && experiment_started) {
        current_time = HAL_GetTick();
        Update_Target_Angle(current_time);
        update_motor_control();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (!experiment_started) {
        return;
    }

    uint32_t current_interrupt_time = HAL_GetTick();
    if (current_interrupt_time - last_interrupt_time < DEBOUNCE_DELAY) {
        return;
    }

    last_interrupt_time = current_interrupt_time;
    if (GPIO_Pin == GPIO_PIN_5 || GPIO_Pin == GPIO_PIN_6) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) {
            encA_curr = 1;
        } else {
            encA_curr = 0;
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
            encB_curr = 1;
        } else {
            encB_curr = 0;
        }

        volatile int prev_state = (encA_prev << 1) | encB_prev;
        volatile int curr_state = (encA_curr << 1) | encB_curr;
        int transition = ((prev_state << 2) | curr_state);

        if (experiment_started) {  // ?? ?? ??? current_angle ????
            switch (transition) {
                case 0b0001:
                case 0b0111:
                case 0b1110:
                case 0b1000:
                    current_angle -= angle_per_transition;
                    break;

                case 0b0010:
                case 0b1011:
                case 0b1101:
                case 0b0100:
                    current_angle += angle_per_transition;
                    break;

                default:
                    break;
            }
        }

        encA_prev = encA_curr;
        encB_prev = encB_curr;
        Update_Target_Angle(current_time);
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

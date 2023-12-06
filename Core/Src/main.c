/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//////////////////////////////////// Librerias utilizadas
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include "stm32f1xx_hal.h"
////////////////////////////////////

//////////////////////////////////// Definiciones
#define ValorMinCCR 99
#define RAD_TO_DEG 57.2957795131f
#define MPU6050_ADDRESS 0xD0
#define ACELERACION_RAMP_INICIO 0.1
#define ACELERACION_RAMP_FINAL 0.1
#define Buf_Max 20
#define X_MAX_DIFF 485
#define Y_MAX_DIFF 485
#define MAX_SPEED 510
#define MAX_SPEED_X 500
#define MAX_SPEED_Y 1000
#define CORRECTION_VALUE 5
#define PWM_MIN 50
#define PWM_MAX 100
#define X_CENTER 510
#define Y_CENTER 510
#define JOYSTICK_TOLERANCE 30
#define ALPHA 0.999
#define THRESHOLD 0.001
#define JOYSTICK_DEADZONE 300
#define UMBRAL_MOVIMIENTO 2.0
#define NUM_LECTURAS 3
#define UMBRAL_CONSISTENCIA 2.0
#define Angle_0 0

#define INTERVALO_ECO 500
#define MAX_INTENTOS_SIN_RESPUESTA 3

////////////////////////////////////

//////////////////////////////////// Variables globales

volatile uint32_t contadorEco = 0;
volatile uint8_t intentosSinRespuesta = 0;
float ultimaPosicionSignificativa = 0;
float lecturasAngulo[NUM_LECTURAS] = {0};
int indiceLectura = 0;
#define ALPHA_FILTRO 0.1
#define UMBRAL_VIBRACION 1.2
static float axFiltrado = 0, ayFiltrado = 0, azFiltrado = 0;

enum estado {
    ENERGIZADA,
    ANDANDO,
	BLOQUEADA,
};
volatile enum estado ESTADO;
const char* stateStr = NULL;
const char* OK = "K\r";
typedef struct {
    float ax, ay, az, gx, gy, gz;
    float Acc;
    float Gy;
    float Angle;
} MPU6050_Data;
MPU6050_Data mpuData;

typedef struct {
    uint8_t xData[5];
    uint8_t yData[5];
    uint16_t x;
    uint16_t y;
} JoystickData;
JoystickData joystick;

typedef struct {
    uint16_t velocidadPrev;
    uint16_t velocidadObjetivo;
    uint16_t CCR;
    uint8_t paso;
    uint16_t contador;
    uint8_t dir;
    uint8_t dirPrev;
    uint16_t diferenciaprincipal;
    uint16_t Prescaler;
    uint16_t flag;
} MotorState;
MotorState motor1State, motor2State;

uint8_t Buffer[Buf_Max];
uint8_t RData;

uint32_t Frec=0;

uint16_t valor_pwm_silla;
uint16_t valor_pwm_sensor;
uint8_t obstaculo = 1;
uint8_t command_index = 0;

uint8_t Min_CCR = 49;
uint16_t Angulo;

uint8_t data_tx[2];
uint8_t data_rx[15];

float gyroRate = 0.0;
float Angleprev;
float complementaryAngle = 0.0;
uint32_t previousTime = 0;
uint8_t flagMPU;
volatile uint8_t procesarMPU = 0;
////////////////////////////////////

//////////////////////////////////// Declaracion de funciones
void Interpretar_Comando(void);
void ProcessReceivedData(uint8_t);
void HandleCommandE(void);
void HandleCombinedXYCommand(void);
void HandleCommandE(void);
void HandleCommandA(void);
void HandleCommandB(void);
void SendCurrentState(void);

void Accionar_Motor_Silla (float);
void Accionar_Motor_Sensor();

void MPU6050_Init(void);
void MPU6050_Reset(void);
void AccionarMPU(void);
uint8_t MPU6050_Read_Data(void);
float MPU6050_Get_ax(void);
float MPU6050_Get_ay(void);
float MPU6050_Get_az(void);
float MPU6050_Get_Gx(void);
float MPU6050_Get_Gy(void);
float MPU6050_Get_Gz(void);
float MPU6050_Get_Temperature(void);

void InicializarMotorState(MotorState *);
uint16_t Valor_Prescaler(uint16_t, MotorState *);
void calcularVelocidadMotores(void);
void Rampa_Motor(MotorState *, TIM_HandleTypeDef *, uint32_t, uint16_t, uint16_t);
void calcularpaso(MotorState *);

bool isJoystickAtCenterY(void);
bool isJoystickAtCenterX(void);

float CalcularPromedio(float *, int);
bool VerificarConsistencia(float *, int, float);
float filtroPasoBajo(float, float, float);

void MiDelay(uint32_t);
////////////////////////////////////

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  InicializarMotorState(&motor1State);
  InicializarMotorState(&motor2State);

  ESTADO = ENERGIZADA;

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // MOTOR SENSOR
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // MOTOR SILLA


  HAL_GPIO_WritePin(GPIOA, M1_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, M2_ENABLE_Pin, GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart1,&RData, 1);

  MPU6050_Init();

  SendCurrentState();
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // MOTOR SENSOR
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4); // MOTOR SILLA
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (procesarMPU) {
		AccionarMPU();
		procesarMPU = 0;
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3599;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 143;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 749;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M1_DIR_Pin|M2_DIR_Pin|M1_ENABLE_Pin|M2_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_DIR_Pin M2_DIR_Pin M1_ENABLE_Pin M2_ENABLE_Pin */
  GPIO_InitStruct.Pin = M1_DIR_Pin|M2_DIR_Pin|M1_ENABLE_Pin|M2_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INTERRUPCION_OBS_Pin */
  GPIO_InitStruct.Pin = INTERRUPCION_OBS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(INTERRUPCION_OBS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//////////////////////////////////// Comunicacion por UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        ProcessReceivedData(RData);
        HAL_UART_Receive_IT(&huart1, &RData, 1);
    }
}

//////////////////////////////////// Inicializar valores de los motores en 0
void InicializarMotorState(MotorState *motor) {
    motor->velocidadPrev = 0;
    motor->velocidadObjetivo = 0;
    motor->paso = 0;
    motor->contador = 0;
    motor->dir = GPIO_PIN_RESET;
    motor->flag = 0;
}

//////////////////////////////////// Procesamiento de mensajes
void ProcessReceivedData(uint8_t data) {

   	if (data == '\n') {
		Buffer[command_index] = '\0';
		Interpretar_Comando();
		command_index = 0;
		memset(Buffer, 0, sizeof(Buffer));
	} else if (command_index < Buf_Max - 1) {
		Buffer[command_index++] = data;
	}
}

void Interpretar_Comando() {
    if (Buffer[0] == 'X' && strchr((char*)Buffer, 'Y')) {
        HandleCombinedXYCommand();
    } else {
        switch (Buffer[0]) {
            case 'A': HandleCommandA(); break;
            case 'B': HandleCommandB(); break;
            case 'E': HandleCommandE(); break;
            case 'K': intentosSinRespuesta = 0; break;
            default: break;
        }
    }
    if (ESTADO == ANDANDO) {
        calcularVelocidadMotores();
        Accionar_Motor_Sensor();
    }
}

void HandleCombinedXYCommand() {

    char* yPosition = strchr((char*)Buffer, 'Y');

    if (yPosition) {
        *yPosition = '\0';
        joystick.x = atoi((char*)(Buffer + 2));
        joystick.y = atoi((char*)(yPosition + 2));
    }
}

void HandleCommandA() {

	joystick.x = 510;
	joystick.y = 510;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // MOTOR SENSOR
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // MOTOR SILLA
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // MOTOR 1
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // MOTOR 2


    HAL_GPIO_WritePin(GPIOA, M1_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, M2_ENABLE_Pin, GPIO_PIN_RESET);

    ESTADO = ANDANDO;
    SendCurrentState();
}

void HandleCommandB() {
    ESTADO = BLOQUEADA;
	joystick.x = 510;
	joystick.y = 510;
    calcularVelocidadMotores();
    Accionar_Motor_Sensor();

    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // MOTOR SENSOR
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // MOTOR SILLA
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // MOTOR 1
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // MOTOR 2

    HAL_GPIO_WritePin(GPIOA, M1_ENABLE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, M2_ENABLE_Pin, GPIO_PIN_SET);

    SendCurrentState();
}

void HandleCommandE() {
    ESTADO = ENERGIZADA;
	joystick.x = 510;
	joystick.y = 510;
    calcularVelocidadMotores();
    Accionar_Motor_Sensor();


    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // MOTOR SENSOR
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4); // MOTOR SILLA
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); // MOTOR 1
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); // MOTOR 2

    HAL_GPIO_WritePin(GPIOA, M1_ENABLE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, M2_ENABLE_Pin, GPIO_PIN_SET);

    SendCurrentState();
}
////////////////////////////////////
//////////////////////////////////// Envio de estado por UART
void SendCurrentState() {

    switch (ESTADO) {
        case ANDANDO:
            stateStr = "A\r";
            break;
        case BLOQUEADA:
            stateStr = "B\r";
            break;
        case ENERGIZADA:
            stateStr = "E\r";
            break;
        default:
            break;
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)stateStr, strlen(stateStr), HAL_MAX_DELAY);
}

//////////////////////////////////// Calculo de la velocidad de los motores
void calcularVelocidadMotores() {

    int16_t diffX = joystick.x - X_CENTER;
    int16_t diffY = joystick.y - Y_CENTER;

    if(isJoystickAtCenterX() && isJoystickAtCenterY()) {
        motor1State.velocidadObjetivo = 0;
        motor2State.velocidadObjetivo = 0;
        motor1State.contador= 1;
        motor2State.contador= 1;
        motor1State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor1State.velocidadPrev);
		motor2State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor2State.velocidadPrev);

		return;
    }
    else if(!obstaculo && diffY > 0) {
        motor1State.velocidadObjetivo = 0;
        motor2State.velocidadObjetivo = 0;
        motor1State.contador= 1;
		motor2State.contador= 1;
		motor1State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor1State.velocidadPrev);
		motor2State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor2State.velocidadPrev);

		return;
    }
    else if(isJoystickAtCenterY()) {
        if(obstaculo) {
            if(diffX > 0) {
                motor1State.velocidadObjetivo = abs(diffX/2);
                motor2State.velocidadObjetivo = 0;
                motor1State.dir = GPIO_PIN_SET;
            } else {
                motor1State.velocidadObjetivo = 0;
                motor2State.velocidadObjetivo = abs(diffX/2);
				motor2State.dir = GPIO_PIN_RESET;
            }
        } else {
            motor1State.velocidadObjetivo = 0;
            motor2State.velocidadObjetivo = 0;
        }
        motor1State.contador= 1;
		motor2State.contador= 1;
		motor1State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor1State.velocidadPrev);
		motor2State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor2State.velocidadPrev);

		return;
    }
    else if(obstaculo && isJoystickAtCenterX()) {
        motor1State.velocidadObjetivo = abs(diffY);
        motor2State.velocidadObjetivo = abs(diffY);
        motor1State.dir = diffY > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
        motor2State.dir = diffY > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;
        motor1State.contador= 1;
		motor2State.contador= 1;
		motor1State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor1State.velocidadPrev);
		motor2State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor2State.velocidadPrev);

		return;
    }
    else{
    motor1State.dir = diffY > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
    motor2State.dir = diffY > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;

    //////////////////////////////////// Ajuste proporcional de velocidad
    float magnitude = sqrt(diffX * diffX + diffY * diffY);  // Calcula la magnitud del vector de diferencia.
    float factorXFromCenter = (float)abs(diffX) / X_MAX_DIFF;  // Calcula el factor proporcional en X.
    float normalizedMagnitude = (magnitude / sqrt(X_MAX_DIFF * X_MAX_DIFF + Y_MAX_DIFF * Y_MAX_DIFF)) * MAX_SPEED;  // Normaliza la magnitud y la ajusta a la velocidad máxima.
    float speedAdjust = factorXFromCenter * 0.5f * normalizedMagnitude;  // Determina el ajuste de velocidad en base al factor X.

    motor1State.velocidadObjetivo = normalizedMagnitude + (diffX > 0 ? speedAdjust : -speedAdjust);
    motor2State.velocidadObjetivo = normalizedMagnitude - (diffX > 0 ? speedAdjust : -speedAdjust);

    motor1State.contador= 1;
	motor2State.contador= 1;
    motor1State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor1State.velocidadPrev);
	motor2State.diferenciaprincipal= abs(motor1State.velocidadObjetivo - motor2State.velocidadPrev);
	return;
    }
}

bool isJoystickAtCenterX() {
    return (joystick.x > (X_CENTER - JOYSTICK_TOLERANCE) && joystick.x < (X_CENTER + JOYSTICK_TOLERANCE));
}
bool isJoystickAtCenterY() {
    return (joystick.y > (Y_CENTER - JOYSTICK_TOLERANCE) && joystick.y < (Y_CENTER + JOYSTICK_TOLERANCE));
}

void calcularpaso(MotorState *motor)
{	uint8_t valorpaso=0;
	uint16_t diferencia=0;

	if (motor->dir == motor->dirPrev)
	{
		diferencia=abs(motor->velocidadPrev - motor->velocidadObjetivo);
	}
	else
	{
		diferencia = abs (motor->velocidadObjetivo + motor->velocidadPrev);
	}
	if (((motor->diferenciaprincipal)/2) > (motor->contador)){

		if ((motor->contador)>400)
		{
			valorpaso=5;
		}
		else if ((motor->contador)>250)
		{
			valorpaso=4;
		}
		else if ((motor->contador)>150)
		{
			valorpaso=3;
		}
		else if ((motor->contador)>50)
		{
			valorpaso=2;
		}
		else if ((motor->contador)>0)
		{
			valorpaso=1;
		}
	}
	else{

		if (diferencia>400)
		{
			valorpaso=5;
		}
		else if (diferencia>250)
		{
			valorpaso=4;
		}
		else if (diferencia>150)
		{
			valorpaso=3;
		}
		else if (diferencia>50)
		{
			valorpaso=2;
		}
		else if (diferencia>0)
		{
			valorpaso=1;
		}
	}
	motor->paso=valorpaso;
}

uint16_t Valor_Prescaler(uint16_t valor, MotorState *motor)
{
	if (valor<2) valor=0;
	if(valor > 510) valor = 510;

	Frec = abs(round((valor * 120 / 460) + 30));

    uint16_t Prescaler = round((10000 / Frec ) - 1);

    return Prescaler;
}

void Rampa_Motor(MotorState *motor, TIM_HandleTypeDef *htimx, uint32_t channel, uint16_t direccionPin, uint16_t ENABLE_Pin) {

	if (motor->dir != motor->dirPrev) {
        if (motor->velocidadPrev != 0) {
            motor->velocidadPrev--;
            uint32_t Prescaler_value = round(Valor_Prescaler(motor->velocidadPrev, motor));
            __HAL_TIM_SET_PRESCALER(htimx, Prescaler_value);

            motor->Prescaler = Prescaler_value;
        } else if (motor->velocidadPrev == 0) {
            HAL_GPIO_WritePin(GPIOA, direccionPin, motor->dir);
            motor->dirPrev = motor->dir;
        }
    } else {
        int16_t diferencia = motor->velocidadObjetivo - motor->velocidadPrev;
        if (diferencia != 0) {
            motor->velocidadPrev += (diferencia > 0) ? 1 : -1;
            uint32_t Prescaler_value = round(Valor_Prescaler(motor->velocidadPrev, motor));
            __HAL_TIM_SET_PRESCALER(htimx, Prescaler_value);
            motor->Prescaler = Prescaler_value;
        }
    }
    if (motor->velocidadPrev == 0) {
        HAL_GPIO_WritePin(GPIOA, ENABLE_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, ENABLE_Pin, GPIO_PIN_RESET);
    }
    motor->contador++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2){
  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
  	  motor1State.flag ++;
    }
    if (htim->Instance == TIM4){
  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
  	  motor2State.flag ++;
    }
    if (htim->Instance == TIM1 && ESTADO == ANDANDO) {

    	calcularpaso(&motor1State);
    	Rampa_Motor(&motor1State, &htim2, TIM_CHANNEL_3, M1_DIR_Pin, M1_ENABLE_Pin);

    	calcularpaso(&motor2State);
    	Rampa_Motor(&motor2State, &htim4, TIM_CHANNEL_3, M2_DIR_Pin, M2_ENABLE_Pin);
    }
    else if (htim->Instance == TIM1 && ((ESTADO == BLOQUEADA || ESTADO == ENERGIZADA) && motor1State.velocidadObjetivo==0 && motor2State.velocidadObjetivo==0 && motor1State.velocidadPrev!=0 && motor2State.velocidadPrev!=0)){

		calcularpaso(&motor1State);
		Rampa_Motor(&motor1State, &htim2, TIM_CHANNEL_3, M1_DIR_Pin, M1_ENABLE_Pin);

		calcularpaso(&motor2State);
		Rampa_Motor(&motor2State, &htim4, TIM_CHANNEL_3, M2_DIR_Pin, M2_ENABLE_Pin);
	}

    if (htim->Instance == TIM1 && (ESTADO == BLOQUEADA || ESTADO == ANDANDO))
    {
    	procesarMPU = 1;
    }

    if (htim->Instance == TIM1) {
            contadorEco++;
            if (contadorEco >= INTERVALO_ECO) {
                contadorEco = 0;
                HAL_UART_Transmit(&huart1, (uint8_t*)OK, strlen(OK), HAL_MAX_DELAY);
                intentosSinRespuesta++;
                if (intentosSinRespuesta > MAX_INTENTOS_SIN_RESPUESTA) {
                	joystick.x = 510;
                	joystick.y = 510;

                    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // MOTOR SENSOR
                    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4); // MOTOR SILLA
                    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); // MOTOR 1
                    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); // MOTOR 2

                    HAL_GPIO_WritePin(GPIOA, M1_ENABLE_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOA, M2_ENABLE_Pin, GPIO_PIN_SET);
                    ESTADO = ENERGIZADA;
                    SendCurrentState();
                }
            }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        obstaculo = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;

        if (motor1State.dirPrev == 1 || !motor2State.dirPrev){
        	motor1State.velocidadObjetivo = 0;
        	motor2State.velocidadObjetivo = 0;
        }
    }
}

void MiDelay(uint32_t delayMilliseconds) {
    uint32_t startTick = HAL_GetTick();
    while (HAL_GetTick() - startTick < delayMilliseconds) {
    }
}


void AccionarMPU() {
    flagMPU = MPU6050_Read_Data();  // Lee datos del MPU6050 y almacena el resultado en flagMPU.

    if (flagMPU == 0) {  // Si la lectura falla,
        MPU6050_Reset();
        MiDelay(100);
        MPU6050_Init();
    } else {  // Si la lectura es exitosa,
        uint32_t currentTime = HAL_GetTick();  // Obtiene el tiempo actual.
        uint32_t deltaTime = currentTime - previousTime;  // Calcula el tiempo transcurrido desde la última lectura.
        previousTime = currentTime;  // Actualiza el tiempo anterior con el actual.

        // Filtra las lecturas de aceleración en cada eje usando un filtro de paso bajo.
        axFiltrado = filtroPasoBajo(MPU6050_Get_ax(), axFiltrado, ALPHA_FILTRO);
        ayFiltrado = filtroPasoBajo(MPU6050_Get_ay(), ayFiltrado, ALPHA_FILTRO);
        azFiltrado = filtroPasoBajo(MPU6050_Get_az(), azFiltrado, ALPHA_FILTRO);

        float magnitud = sqrt(axFiltrado * axFiltrado + ayFiltrado * ayFiltrado + azFiltrado * azFiltrado);  // Calcula la magnitud de la aceleración filtrada.

        if (magnitud < UMBRAL_VIBRACION) {  // Si la magnitud está por debajo de un umbral de vibración,
            float denom = sqrt(pow(ayFiltrado, 2) + pow(azFiltrado, 2));  // Calcula el denominador para el ángulo de aceleración.
            if (denom < 0.001) {  // Si el denominador es demasiado pequeño,
                return;  // Termina la función para evitar división por cero.
            }

            // Calcula el ángulo de aceleración y la tasa del giroscopio.
            float accelAngle = atan(-1 * axFiltrado / denom) * RAD_TO_DEG;
            gyroRate = MPU6050_Get_Gx();
            // Calcula un ángulo complementario usando un filtro complementario.
            complementaryAngle = (1 - ALPHA) * (complementaryAngle + gyroRate * deltaTime) + ALPHA * accelAngle;

            // Almacena el ángulo complementario en un arreglo para análisis.
            lecturasAngulo[indiceLectura] = complementaryAngle;
            indiceLectura = (indiceLectura + 1) % NUM_LECTURAS;  // Avanza en el índice de lectura.

            // Verifica la consistencia de las lecturas del ángulo.
            if (VerificarConsistencia(lecturasAngulo, NUM_LECTURAS, UMBRAL_CONSISTENCIA)) {
                float promedioAngulo = CalcularPromedio(lecturasAngulo, NUM_LECTURAS);  // Calcula el promedio de los ángulos.

                // Si el promedio difiere significativamente de la última posición significativa,
                if (abs(promedioAngulo - ultimaPosicionSignificativa) > UMBRAL_MOVIMIENTO) {
                    Accionar_Motor_Silla(promedioAngulo);  // Actúa sobre el motor de la silla.
                    ultimaPosicionSignificativa = promedioAngulo;  // Actualiza la última posición significativa.
                }
            }
        }
    }
}

float filtroPasoBajo(float entrada, float salidaAnterior, float alpha) {
    // Combina la entrada actual con la salida anterior basándose en el factor alpha.
    // Esto suaviza la señal, permitiendo pasar solo las frecuencias bajas.
    return alpha * entrada + (1 - alpha) * salidaAnterior;
}


bool VerificarConsistencia(float *array, int longitud, float umbral) {
    float min = array[0];  // Inicializa el mínimo con el primer elemento del array.
    float max = array[0];

    // Recorre el array para encontrar los valores mínimo y máximo.
    for (int i = 1; i < longitud; i++) {
        if (array[i] < min) min = array[i];  // Actualiza el mínimo si encuentra un valor más bajo.
        if (array[i] > max) max = array[i];  // Actualiza el máximo si encuentra un valor más alto.
    }

    // Devuelve true si la diferencia entre el máximo y el mínimo es menor o igual al umbral.
    return (max - min) <= umbral;
}


float CalcularPromedio(float *array, int longitud) {
    float suma = 0.0;  // Inicializa la suma de los elementos del array.

    // Suma todos los elementos del array.
    for (int i = 0; i < longitud; i++) {
        suma += array[i];
    }

    // Devuelve el promedio de los elementos del array.
    return suma / longitud;
}


void Accionar_Motor_Silla(float angulo)
{
    if (angulo < -35) {
        angulo = -35;
    } else if (angulo > 35) {
        angulo = 35;
    }
    valor_pwm_silla = (-angulo * 80 / 30) + 750;
    TIM3->CCR4 = valor_pwm_silla;
}

void Accionar_Motor_Sensor(){
	uint16_t x = joystick.x;
	if (joystick.x > 470 && joystick.x  < 550)
	{
		x  = 1022/2;
	}
	valor_pwm_sensor = (-x * 300/1022) + 900;
	TIM3->CCR1 = valor_pwm_sensor;
}

void MPU6050_Init(void) {
    data_tx[0] = 0x6B;  // Dirección del registro de control de potencia.
    data_tx[1] = 0x00;  // Valor para activar el MPU6050 (desactivar modo de reposo).
    // Envía los datos al MPU6050 utilizando I2C.
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MPU6050_ADDRESS, data_tx, 2, 100);
}


void MPU6050_Reset(void) {
    uint8_t data[2];
    data[0] = 0x6B;  // Dirección del registro de control de potencia.
    data[1] = 0x80;  // Valor para resetear el MPU6050.
    // Envía los datos al MPU6050 para resetearlo.
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MPU6050_ADDRESS, data, 2, 100);
}


uint8_t MPU6050_Read_Data(void) {
    HAL_StatusTypeDef status;
    uint8_t regAddress = 0x3B;  // Dirección del primer registro de datos del acelerómetro.

    // Envía la dirección del registro al MPU6050.
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MPU6050_ADDRESS, &regAddress, 1, 100);

    if (status != HAL_OK) {
        return 0;  // Retorna 0 si hay un error en la transmisión.
    }

    // Recibe los datos del MPU6050.
    status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)MPU6050_ADDRESS, data_rx, 14, 100);

    // Retorna 1 si la lectura es exitosa, de lo contrario 0.
    return (status == HAL_OK) ? 1 : 0;
}


float MPU6050_Get_ax(void)
{
	return (float)(((int16_t)(data_rx[0]<<8 | data_rx[1]))/(float)16384);
}

float MPU6050_Get_ay(void)
{
	return (float)(((int16_t)(data_rx[2]<<8 | data_rx[3]))/(float)16384);
}

float MPU6050_Get_az(void)
{
	return (float)(((int16_t)(data_rx[4]<<8 | data_rx[5]))/(float)16384);
}

float MPU6050_Get_Gx(void)
{
	return (float)(((int16_t)(data_rx[10]<<8 | data_rx[11]))/(float)131);
}

float MPU6050_Get_Gy(void)
{
	return (float)(((int16_t)(data_rx[12]<<8 | data_rx[13]))/(float)131);
}

float MPU6050_Get_Gz(void)
{
	return (float)(((int16_t)(data_rx[8]<<8 | data_rx[9]))/(float)131);
}

float MPU6050_Get_Temperature(void)
{
	return (float)(((int16_t)(data_rx[6]<<8 | data_rx[7]))/(float)340 + (float)36.53);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
//#include "GY85.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
flags_t timer_flags;
int time_ms = 0;
// counters for speed calculation
uint16_t front_wheel_interrupt_count = 0;
uint16_t rear_wheel_interrupt_count = 0;

// ipnut values for open loop control
uint16_t front_wheel_pwmval = 0;
uint16_t rear_wheel_pwmval = 0;
uint8_t is_go_backward = 0;
uint16_t tim3_pwmval_1 = 3600;
float steering_percentage = 0;


// IMU
//IMU_t IMU_car;
//uint16_t gyro_Z_raw[FILTER_N];
//float filtered_angle_rate = 0;
//float yaw_rate_buffer[YAW_RATE_BUF_SIZE];
//int yaw_rate_index = 0;

// map
map_t map;

// variables for pid control
PID_t image2steerPercent_PID_str;
PID_t front_wheel_PID_str;
PID_t rear_wheel_PID_str;
PID_t dis2spd_PID_str;
PID_t dis2spd_PID_str_back;

// UART
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_main_buffer[MAIN_BUFFER_SIZE];

// camera
camera_t AMB82;
int camera_update_fail_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void following_loop_new();

void check_and_set_servo_pwmval(uint16_t* pwmval);
void check_and_set_DCmotor_pwm();

void PID_str_init(PID_t *PID, float Kp, float Ki, float Kd, float targe_val, float dt, float max, float min);
void PID_control(PID_t *PID, float new_val);

void wheel_speed_control();
void steer_control_image();

void steer_by_percentage(float);

void update_camera(camera_t *camera, uint8_t rx_data[23]);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  // uart
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE);
	// PID
	PID_str_init(&image2steerPercent_PID_str, Kp_image2turnPercent, Ki_image2turnPercent, Kd_image2turnPercent, 0, dt_image2turnPercent, 100, -100);
	PID_str_init(&front_wheel_PID_str, Kp_speed, Ki_speed, Kd_speed, 0, dt_speed, PID_speed_max, PID_speed_min);
	PID_str_init(&rear_wheel_PID_str, Kp_speed, Ki_speed, Kd_speed, 0, dt_speed, PID_speed_max, PID_speed_min);
	PID_str_init(&dis2spd_PID_str, Kp_dis_ctrl, Ki_dis_ctrl, Kd_dis_ctrl, FOLLOWING_DIST, dt_dis_ctrl, MaxOutput_dis_ctrl, MinOutput_dis_ctrl);
	PID_str_init(&dis2spd_PID_str_back, Kp_dis_ctrl_back, Ki_dis_ctrl_back, Kd_dis_ctrl_back, FOLLOWING_DIST, dt_dis_ctrl, MaxOutput_dis_ctrl, MinOutput_dis_ctrl);

	init_timer_flags(timer_flags);

	//GY85_Init(&IMU_car);
	AMB82_INIT();
	MAP_INIT(map);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	following_loop_new();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 12000-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 30-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 48000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 3600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void check_and_set_servo_pwmval(uint16_t* pwmval){
	//check
	if(*pwmval > TIM3_PWMVal_MAX) *pwmval = TIM3_PWMVal_MAX;
	if(*pwmval < TIM3_PWMVal_MIN) *pwmval = TIM3_PWMVal_MIN;
	//set
	__HAL_TIM_SetCompare(servo_PWM_TIM, servo_PWM_CHANNEL, *pwmval);
}

void check_and_set_DCmotor_pwm(){
	//check
	if (front_wheel_pwmval > TIM1_PWMVal_MAX) front_wheel_pwmval = TIM1_PWMVal_MAX;
	if (rear_wheel_pwmval > TIM1_PWMVal_MAX) rear_wheel_pwmval = TIM1_PWMVal_MAX;
	if (front_wheel_pwmval < TIM1_PWMVal_MIN) front_wheel_pwmval = TIM1_PWMVal_MIN;
	if (rear_wheel_pwmval < TIM1_PWMVal_MIN) rear_wheel_pwmval = TIM1_PWMVal_MIN;
	//set
	SET_FRONT_MOTOR_PWM(front_wheel_pwmval);
	SET_REAR_MOTOR_PWM(rear_wheel_pwmval);
	if(is_go_backward){
		HAL_GPIO_WritePin(MOTOR_DIR_CTRL_GPIO, FRONT_MOTOR_DIR_CTRL_PIN, FRONT_MOTOR_DIR_CTRL_BACKWARD);
		HAL_GPIO_WritePin(MOTOR_DIR_CTRL_GPIO, REAR_MOTOR_DIR_CTRL_PIN, REAR_MOTOR_DIR_CTRL_BACKWARD);
	}
	else{
		HAL_GPIO_WritePin(MOTOR_DIR_CTRL_GPIO, FRONT_MOTOR_DIR_CTRL_PIN, FRONT_MOTOR_DIR_CTRL_FORWARD);
		HAL_GPIO_WritePin(MOTOR_DIR_CTRL_GPIO, REAR_MOTOR_DIR_CTRL_PIN, REAR_MOTOR_DIR_CTRL_FORWARD);
	}
}

void PID_str_init(PID_t *PID, float Kp, float Ki, float Kd, float targe_val, float dt, float max, float min){
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->target_val = targe_val;
	PID->dt = dt;
	for (int i = 0; i < PID_BUFFERSIZE; i++){
		PID->err_buffer[i] = 0;
	}
	PID->cir_queue_index = 0;
	PID->max_ctrl_val = max;
	PID->min_ctrl_val = min;
	PID->ctrl_val = 0;
}

void PID_control(PID_t *PID, float new_val){
	float P, I, D, d_err;
	float err_sum = 0;
	PID->err_buffer[PID->cir_queue_index] = PID->target_val - new_val;
	for (int i = 0; i < PID_BUFFERSIZE; i++){
		err_sum += PID->err_buffer[i];
	}
	if (PID->cir_queue_index == 0){
		d_err = PID->err_buffer[PID->cir_queue_index] - PID->err_buffer[PID_BUFFERSIZE - 1];
	}
	else{
		d_err = PID->err_buffer[PID->cir_queue_index] - PID->err_buffer[PID->cir_queue_index - 1];
	}
	P = PID->err_buffer[PID->cir_queue_index] * PID->Kp;
	I = err_sum * PID->Ki;
	D = PID->Kd * d_err / PID->dt;
	PID->ctrl_val = P + I + D;
	if (PID->ctrl_val > PID->max_ctrl_val){
		PID->ctrl_val = PID->max_ctrl_val;
	}
	else if (PID->ctrl_val < PID->min_ctrl_val){
		PID->ctrl_val = PID->min_ctrl_val;
	}
	PID->cir_queue_index ++;
	if (PID->cir_queue_index == PID_BUFFERSIZE){
		PID->cir_queue_index = 0;
	}
}

void wheel_speed_control(map_t * map){
	PID_control(&front_wheel_PID_str, (float)front_wheel_interrupt_count);
	PID_control(&rear_wheel_PID_str, (float)rear_wheel_interrupt_count);
	if (is_go_backward){
		map->motor_int_count -= (int)(0.5*(front_wheel_interrupt_count + rear_wheel_interrupt_count));
	}
	else {
		map->motor_int_count += (int)(0.5*(front_wheel_interrupt_count + rear_wheel_interrupt_count));
	}
	front_wheel_interrupt_count = 0;
	rear_wheel_interrupt_count = 0;

	front_wheel_pwmval += front_wheel_PID_str.ctrl_val;
	rear_wheel_pwmval += rear_wheel_PID_str.ctrl_val;

	check_and_set_DCmotor_pwm();
}

void steer_control_image(){
	if (is_go_backward){
		steering_percentage = -image2steerPercent_PID_str.ctrl_val;
	}
	else{
		steering_percentage = image2steerPercent_PID_str.ctrl_val;
	}
	steer_by_percentage(steering_percentage);
}

//input: percentage > 0:
// 100 -> 12.5
// 0 -> 2.5
void steer_by_percentage(float percentage){
	uint16_t pwmval = 0;
	pwmval = (uint16_t)(TIM3_PWMVal_MIN + TIM3_PWMVal_RANGE*(0.5 + percentage/200.0));
	check_and_set_servo_pwmval(&pwmval);
}

void update_camera(camera_t *camera, uint8_t rx_data[23]){
	camera->updated = 1;

	camera->low_left[coord_x] = 0;
	camera->low_left[coord_y] = 0;
	camera->up_right[coord_x] = 0;
	camera->up_right[coord_y] = 0;
	camera->confidence = 0;

	camera->low_left[coord_x] += (rx_data[x_min_byte	] - 48) * 1000;
	camera->low_left[coord_x] += (rx_data[x_min_byte + 1] - 48) * 100;
	camera->low_left[coord_x] += (rx_data[x_min_byte + 2] - 48) * 10;
	camera->low_left[coord_x] += (rx_data[x_min_byte + 3] - 48);

	camera->low_left[coord_y] += (rx_data[y_min_byte	] - 48) * 1000;
	camera->low_left[coord_y] += (rx_data[y_min_byte + 1] - 48) * 100;
	camera->low_left[coord_y] += (rx_data[y_min_byte + 2] - 48) * 10;
	camera->low_left[coord_y] += (rx_data[y_min_byte + 3] - 48);

	camera->up_right[coord_x] += (rx_data[x_max_byte	] - 48) * 1000;
	camera->up_right[coord_x] += (rx_data[x_max_byte + 1] - 48) * 100;
	camera->up_right[coord_x] += (rx_data[x_max_byte + 2] - 48) * 10;
	camera->up_right[coord_x] += (rx_data[x_max_byte + 3] - 48);

	camera->up_right[coord_y] += (rx_data[y_max_byte	] - 48) * 1000;
	camera->up_right[coord_y] += (rx_data[y_max_byte + 1] - 48) * 100;
	camera->up_right[coord_y] += (rx_data[y_max_byte + 2] - 48) * 10;
	camera->up_right[coord_y] += (rx_data[y_max_byte + 3] - 48);

	camera->center[coord_x] = (camera->low_left[coord_x] + camera->up_right[coord_x]) / 2;
	camera->center[coord_y] = (camera->low_left[coord_y] + camera->up_right[coord_y]) / 2;

	camera->confidence += (rx_data[confidence_byte] - 48) * 10;
	camera->confidence += (rx_data[confidence_byte + 1] - 48);


	camera->diag_length = pow(camera->low_left[coord_x] - camera->up_right[coord_x], 2) + \
						  pow(camera->low_left[coord_y] - camera->up_right[coord_y], 2);

}

void AMB82_CalcObjPos(camera_t *camera, map_t *map) {
	// +x: forward
//	float angle = (camera->center[0] - 960) / AMB82_FOV;
	float angle = 0;
	float distance = 23367.28/(camera->up_right[1]-camera->low_left[1]);//未修正
// distance = -14.04+1.18*distance;//�???�修�?
	distance = 0.1054+0.9397579*distance+8.86754e-4*pow(distance, 2);//二�?�修�?
// distance = -15.5162+1.367*distance-2.5996e-3*pow(distance, 2)+8.6963e-6*pow(distance, 3);//三�?�修�?
// distance = 3.89298+0.64038*distance+6.6809e-3*pow(distance, 2)-3.994e-5*pow(distance, 3)+8.97e-8*pow(distance, 4);//??��?�修�?
// distance = 467.69487-21.4*distance+0.39583*pow(distance, 2)-3.25866e-3*pow(distance, 3)+1.26646e-5*pow(distance, 4)-1.871e-8*pow(distance, 5);//五�?�修�?
	// save point
	map->queue_index++;
	if (map->queue_index == MAP_POINT_NUM) {
		map->queue_index = 0;
	}
	map->points[map->queue_index][0] = distance * cos(RAD2DEG * angle);
	map->points[map->queue_index][1] = distance * sin(RAD2DEG * angle);

//	camera->distance[camera->dist_cir_que_index] = distance;
//	camera->dist_cir_que_index++;
//	if(camera->dist_cir_que_index == CAM_DIST_CIR_QUE_LENGTH) {
//		camera->dist_cir_que_index = 0;
//	}
//	Map_SavePosition(map, x, y, (uint8_t)camera->confidence);
}

void map_linear_predeict(float *predicted_angle, float *predicted_distance, map_t *map, float moved_angle_rad, float moved_dis_cm) {
	float x_predict, y_predict;
	float q[2];
	if (moved_angle_rad >= 0.035 || moved_angle_rad <= -0.035) {
		q[0] = (moved_dis_cm/moved_angle_rad)*(sin(moved_angle_rad)*cos(moved_angle_rad) + (sin(moved_angle_rad)*cos(moved_angle_rad) - sin(moved_angle_rad)));
		q[1] = (moved_dis_cm/moved_angle_rad)*(sin(moved_angle_rad)*sin(moved_angle_rad) + (cos(moved_angle_rad)*cos(moved_angle_rad) - cos(moved_angle_rad)));
	}
	else {
		q[0] = moved_dis_cm;
		q[1] = 0;
	}
	for (int i = 0; i < MAP_POINT_NUM; i++){
		if (i != map->queue_index){
			map->points[i][0] -= q[0];
			map->points[i][1] -= q[1];
		}
	}

	if (map->queue_index == 0){
//		x_predict = 2 * map->points[map->queue_index][0] - map->points[MAP_POINT_NUM - 1][0];
//		y_predict = 2 * map->points[map->queue_index][1] - map->points[MAP_POINT_NUM - 1][1];
		x_predict = map->points[map->queue_index][0] + 0.8*(map->points[map->queue_index][0] - map->points[MAP_POINT_NUM - 1][0]);
		y_predict = map->points[map->queue_index][1] + 0.8*(map->points[map->queue_index][1] - map->points[MAP_POINT_NUM - 1][1]);

	}
	else {
//		x_predict = 2 * map->points[map->queue_index][0] - map->points[map->queue_index - 1][0];
//		y_predict = 2 * map->points[map->queue_index][1] - map->points[map->queue_index - 1][1];
		x_predict = map->points[map->queue_index][0] + 0.8*(map->points[map->queue_index][0] - map->points[map->queue_index - 1][0]);
		y_predict = map->points[map->queue_index][1] + 0.8*(map->points[map->queue_index][1] - map->points[map->queue_index - 1][1]);
	}

	*predicted_angle = RAD2DEG * atan(y_predict/x_predict);
	//*predicted_distance = sqrt(x_predict*x_predict + y_predict*y_predict);
	*predicted_distance = x_predict;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance == USART2){
		memcpy(rx_main_buffer, rx_buffer, RX_BUFFER_SIZE);
		update_camera(&AMB82, rx_main_buffer);
		if(AMB82.confidence > 70) {
			AMB82_CalcObjPos(&AMB82, &map);
		}
//		imageDiag2speedTarget(AMB82.diag_length);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}

// 中位值平均濾波法（又稱防脈衝干擾平均濾波法）（算法1）
float yaw_rate_filter(float *buffer) {
  int i, j;
  float temp;
  float filter_sum = 0;

	// insertion sort
  for (i = 1; i < YAW_RATE_BUF_SIZE; i++) {
		temp = buffer[i];
		for (j = i - 1; j >= 0 && buffer[j] > temp; j--) {
			buffer[j + 1] = buffer[j];
		}
		buffer[j + 1] = temp;
  }

  // 去除最大最小極值後求平均
  for(i = 2; i < YAW_RATE_BUF_SIZE - 2; i++) filter_sum += buffer[i];
  return filter_sum / (float)(YAW_RATE_BUF_SIZE - 4);
}

void following_loop_new() {
	timer_flags.freeze_for_test = 0;
	while (1)
	{
		// read IMU f = 100hz
		if(timer_flags.do_read_IMU) {
			timer_flags.do_read_IMU = 0;
			//GY85_Read_IMU(&IMU_car);
//			yaw_rate_buffer[yaw_rate_index] = IMU_car.GyroZ;
//			yaw_rate_index = (yaw_rate_index + 1) % YAW_RATE_BUF_SIZE;
		}

		// speed control f= 10hz
		if(timer_flags.do_PID_wheel){
			timer_flags.do_PID_wheel = 0;
			//IMU_car.yaw += 0.1 * (yaw_rate_filter(yaw_rate_buffer) - IMU_car.yaw_rate_bias);
			wheel_speed_control(&map);
			check_and_set_DCmotor_pwm();
		}

		// f = 5hz
		// updatae speed and steer control
		if (timer_flags.cam_data_update) {
			timer_flags.cam_data_update = 0;
			if (AMB82.updated == 0){
				camera_update_fail_count++;
				if (camera_update_fail_count > MAX_CAMERA_FAIL_COUNT){
					change_wheel_PID_target_val(0);
				}
			}
			else{
				AMB82.updated = 0;
				camera_update_fail_count = 0;
				float dist ,angle;
	//			map_linear_predeict(&angle, &dist, &map, IMU_car.yaw/RAD2DEG, map.motor_int_count*pulse2dis);
				map_linear_predeict(&angle, &dist, &map, 0, map.motor_int_count*pulse2dis);
				//IMU_car.yaw = 0;
				map.motor_int_count = 0;
				// distance control
				DIST_CTRL();
				// steering control
				STEER_CTRL_IMG();
			}
		}
	}
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

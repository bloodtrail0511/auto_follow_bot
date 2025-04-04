/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>

#define RAD2DEG     57.295779
// TIM1 pwm: DC moter speed control
#define TIM1_PWMVal_MAX 11400
#define TIM1_PWMVal_MIN 0
// TIM3 pwm: steering servo control
// val/48000
#define TIM3_PWMVAL_INIT 3600
#define TIM3_PWMVal_MAX 5600
#define TIM3_PWMVal_MIN 1600
#define TIM3_PWMVal_RANGE 4000

#define servo_PWM_TIM (&htim3)
#define servo_PWM_CHANNEL TIM_CHANNEL_1

#define DCMotor_PWM_TIM (&htim1)
#define SET_FRONT_MOTOR_PWM(val){\
	__HAL_TIM_SetCompare(DCMotor_PWM_TIM, TIM_CHANNEL_1, val);\
}
#define SET_REAR_MOTOR_PWM(val){\
	__HAL_TIM_SetCompare(DCMotor_PWM_TIM, TIM_CHANNEL_2, val);\
}

#define MOTOR_DIR_CTRL_GPIO GPIOA
#define FRONT_MOTOR_DIR_CTRL_PIN GPIO_PIN_10
#define FRONT_MOTOR_DIR_CTRL_FORWARD GPIO_PIN_RESET
#define FRONT_MOTOR_DIR_CTRL_BACKWARD GPIO_PIN_SET
#define REAR_MOTOR_DIR_CTRL_PIN GPIO_PIN_11
#define REAR_MOTOR_DIR_CTRL_FORWARD GPIO_PIN_RESET
#define REAR_MOTOR_DIR_CTRL_BACKWARD GPIO_PIN_SET

// UART
#define RX_BUFFER_SIZE 23
#define MAIN_BUFFER_SIZE 23
#define YAW_RATE_BUF_SIZE 10

#define PID_BUFFERSIZE 10
typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float target_val;
	float err_buffer[PID_BUFFERSIZE];
	int cir_queue_index;
	float dt;
	float max_ctrl_val;
	float min_ctrl_val;
	float ctrl_val;
}PID_t;


// camera redated define
#define CAM_DIST_CIR_QUE_LENGTH 1
#define FOLLOWING_DIST 100 // cm
#define MAX_CAMERA_FAIL_COUNT 10	// 5 * seconds

// AMB82
#define x_min_byte 		0
#define y_min_byte 		5
#define x_max_byte		10
#define y_max_byte		15
#define confidence_byte	20

#define coord_x 0
#define coord_y 1

#define AMB82_FOV 35.0

typedef struct{
	uint8_t updated;
	int center[2];
	int low_left[2];
	int up_right[2];
	int confidence;
	long int diag_length;
//	float distance[CAM_DIST_CIR_QUE_LENGTH];
//	uint8_t dist_cir_que_index;
}camera_t;

#define AMB82_INIT(){\
	AMB82.updated = 0;\
	AMB82.center[0] = 960;\
	AMB82.center[1] = 540;\
	AMB82.confidence = 100;\
}

#define MAP_POINT_NUM 2
typedef struct {
	uint8_t queue_index;
	float_t points[MAP_POINT_NUM][2];
	uint8_t points_confidence[MAP_POINT_NUM];
	int motor_int_count;
}map_t;

#define MAP_INIT(map_str) {\
	map_str.queue_index = 0;\
	for (int i = 0; i < MAP_POINT_NUM; i++) {\
		map_str.points[i][0] = FOLLOWING_DIST;\
		map_str.points[i][1] = 0;\
		map_str.points_confidence[i] = 0;\
		map.motor_int_count = 0;\
	}\
}

typedef struct{
	uint8_t do_PID_wheel;
	uint8_t do_record_gyro;
	uint8_t cam_data_update;
	uint8_t stop_test;
	uint8_t freeze_for_test;
	uint8_t do_read_IMU;
}flags_t;

#define init_timer_flags(flags){\
	flags.do_PID_wheel = 0;\
	flags.do_record_gyro = 0;\
	flags.cam_data_update = 0;\
	flags.stop_test = 0;\
	flags.freeze_for_test = 1;\
	flags.do_read_IMU = 0;\
}

#define Kp_image2turnPercent 0.12
#define Ki_image2turnPercent 0.0001
#define Kd_image2turnPercent 0.05
#define dt_image2turnPercent 0.2 // second

#define encoder_sample_hz 10
#define Kp_speed 80
#define Ki_speed 1
#define Kd_speed 2
#define dt_speed (1/(float)encoder_sample_hz)
#define PID_speed_max TIM1_PWMVal_MAX
#define PID_speed_min (-TIM1_PWMVal_MAX)

#define Kp_dis_ctrl -2.5
#define Ki_dis_ctrl -0.0001
#define Kd_dis_ctrl -0.001

#define Kp_dis_ctrl_back -4	//-2.33
#define Ki_dis_ctrl_back -0.000101654
#define Kd_dis_ctrl_back -0.001

#define MaxOutput_dis_ctrl 500
#define MinOutput_dis_ctrl -500
#define dt_dis_ctrl 0.2

#define pulsenum2rpm (0.08536 * encoder_sample_hz)
#define spd2pulsenum 0.294397
#define pulse2dis 0.3396799

#define change_wheel_PID_target_val(new_target){\
	front_wheel_PID_str.target_val = new_target;\
	rear_wheel_PID_str.target_val = new_target;\
}

#define DISTPID_CTRL_VAL_2_SPEED_TARGET(val){\
	if (val < 0) {\
		is_go_backward = 1;\
		change_wheel_PID_target_val(-spd2pulsenum * val);\
	}\
	else {\
		is_go_backward = 0;\
		change_wheel_PID_target_val(spd2pulsenum * val);\
	}\
}

#define DIST_CTRL(){\
	PID_control(&dis2spd_PID_str, dist);\
	PID_control(&dis2spd_PID_str_back, dist);\
	if (dist - FOLLOWING_DIST > 10) {\
		DISTPID_CTRL_VAL_2_SPEED_TARGET(dis2spd_PID_str.ctrl_val);\
	}\
	else if (dist - FOLLOWING_DIST < -10) {\
		DISTPID_CTRL_VAL_2_SPEED_TARGET(dis2spd_PID_str_back.ctrl_val);\
	}\
	else {\
		DISTPID_CTRL_VAL_2_SPEED_TARGET(0);\
	}\
}
	/*steering control*/
#define STEER_CTRL_IMG(){\
	PID_control(&image2steerPercent_PID_str, (float)AMB82.center[coord_x] - 960);\
	if (is_go_backward){\
		steer_by_percentage(-image2steerPercent_PID_str.ctrl_val);\
	}\
	else {\
		steer_by_percentage(image2steerPercent_PID_str.ctrl_val);\
	}\
}

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

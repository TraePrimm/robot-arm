/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern RobotState g_robot_state;
extern MotionCommand g_command;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlLoop */
osThreadId_t ControlLoopHandle;
uint32_t ControlLoopBuffer[ 512 ];
osStaticThreadDef_t ControlLoopControlBlock;
const osThreadAttr_t ControlLoop_attributes = {
  .name = "ControlLoop",
  .cb_mem = &ControlLoopControlBlock,
  .cb_size = sizeof(ControlLoopControlBlock),
  .stack_mem = &ControlLoopBuffer[0],
  .stack_size = sizeof(ControlLoopBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SafetyMonitor */
osThreadId_t SafetyMonitorHandle;
uint32_t SafetyMonitorBuffer[ 256 ];
osStaticThreadDef_t SafetyMonitorControlBlock;
const osThreadAttr_t SafetyMonitor_attributes = {
  .name = "SafetyMonitor",
  .cb_mem = &SafetyMonitorControlBlock,
  .cb_size = sizeof(SafetyMonitorControlBlock),
  .stack_mem = &SafetyMonitorBuffer[0],
  .stack_size = sizeof(SafetyMonitorBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MotionPlanner */
osThreadId_t MotionPlannerHandle;
uint32_t MotionPlannerBuffer[ 1024 ];
osStaticThreadDef_t MotionPlannerControlBlock;
const osThreadAttr_t MotionPlanner_attributes = {
  .name = "MotionPlanner",
  .cb_mem = &MotionPlannerControlBlock,
  .cb_size = sizeof(MotionPlannerControlBlock),
  .stack_mem = &MotionPlannerBuffer[0],
  .stack_size = sizeof(MotionPlannerBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ROS2 */
osThreadId_t ROS2Handle;
uint32_t ROS2Buffer[ 2048 ];
osStaticThreadDef_t ROS2ControlBlock;
const osThreadAttr_t ROS2_attributes = {
  .name = "ROS2",
  .cb_mem = &ROS2ControlBlock,
  .cb_size = sizeof(ROS2ControlBlock),
  .stack_mem = &ROS2Buffer[0],
  .stack_size = sizeof(ROS2Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Diagnostics */
osThreadId_t DiagnosticsHandle;
uint32_t DiagnosticsBuffer[ 512 ];
osStaticThreadDef_t DiagnosticsControlBlock;
const osThreadAttr_t Diagnostics_attributes = {
  .name = "Diagnostics",
  .cb_mem = &DiagnosticsControlBlock,
  .cb_size = sizeof(DiagnosticsControlBlock),
  .stack_mem = &DiagnosticsBuffer[0],
  .stack_size = sizeof(DiagnosticsBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Homing */
osThreadId_t HomingHandle;
uint32_t HomingBuffer[ 512 ];
osStaticThreadDef_t HomingControlBlock;
const osThreadAttr_t Homing_attributes = {
  .name = "Homing",
  .cb_mem = &HomingControlBlock,
  .cb_size = sizeof(HomingControlBlock),
  .stack_mem = &HomingBuffer[0],
  .stack_size = sizeof(HomingBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartControlLoopTask(void *argument);
void StartSafetyTask(void *argument);
void StartMotionPlannerTask(void *argument);
void StartRosTask(void *argument);
void StartDiagTask(void *argument);
void StartHomingTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ControlLoop */
  ControlLoopHandle = osThreadNew(StartControlLoopTask, NULL, &ControlLoop_attributes);

  /* creation of SafetyMonitor */
  SafetyMonitorHandle = osThreadNew(StartSafetyTask, NULL, &SafetyMonitor_attributes);

  /* creation of MotionPlanner */
  MotionPlannerHandle = osThreadNew(StartMotionPlannerTask, NULL, &MotionPlanner_attributes);

  /* creation of ROS2 */
  ROS2Handle = osThreadNew(StartRosTask, NULL, &ROS2_attributes);

  /* creation of Diagnostics */
  DiagnosticsHandle = osThreadNew(StartDiagTask, NULL, &Diagnostics_attributes);

  /* creation of Homing */
  HomingHandle = osThreadNew(StartHomingTask, NULL, &Homing_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlLoopTask */
/**
* @brief Function implementing the ControlLoop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlLoopTask */
void StartControlLoopTask(void *argument)
{
  /* USER CODE BEGIN StartControlLoopTask */
  TickType_t lastWake = osKernelGetTickCount();
  const TickType_t period = 1;

  for(;;)
  {
      // CONTROL LOOP WILL LIVE HERE

      // Fake motor
      g_robot_state.joint_pos[0] += 0.001f;

      float error = g_command.joint_target[0] - g_robot_state.joint_pos[0];

      float control = error * 1.0f; // fake P controller


      osDelayUntil(lastWake + period);
      lastWake += period;
  }
  /* USER CODE END StartControlLoopTask */
}

/* USER CODE BEGIN Header_StartSafetyTask */
/**
* @brief Function implementing the SafetyMonitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSafetyTask */
void StartSafetyTask(void *argument)
{
  /* USER CODE BEGIN StartSafetyTask */
  /* Infinite loop */
  for(;;)
  {
      if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET)
      {
          g_robot_state.faults = 1;
      }

      osDelay(10);
  }
  /* USER CODE END StartSafetyTask */
}

/* USER CODE BEGIN Header_StartMotionPlannerTask */
/**
* @brief Function implementing the MotionPlanner thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotionPlannerTask */
void StartMotionPlannerTask(void *argument)
{
  /* USER CODE BEGIN StartMotionPlannerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotionPlannerTask */
}

/* USER CODE BEGIN Header_StartRosTask */
/**
* @brief Function implementing the ROS2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRosTask */
void StartRosTask(void *argument)
{
  /* USER CODE BEGIN StartRosTask */
  /* Infinite loop */
  for(;;)
  {
      g_command.joint_target[0] = 1.0f; // simulate incoming command

      osDelay(10);
  }
  /* USER CODE END StartRosTask */
}

/* USER CODE BEGIN Header_StartDiagTask */
/**
* @brief Function implementing the Diagnostics thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDiagTask */
void StartDiagTask(void *argument)
{
  /* USER CODE BEGIN StartDiagTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDiagTask */
}

/* USER CODE BEGIN Header_StartHomingTask */
/**
* @brief Function implementing the Homing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHomingTask */
void StartHomingTask(void *argument)
{
  /* USER CODE BEGIN StartHomingTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHomingTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


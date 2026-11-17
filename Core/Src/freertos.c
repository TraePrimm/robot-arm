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
#include "as5047p.h"
#include "spi.h"
#include "robot_arm.h"
#include "ros_interface.h"
#include <rmw_microros/rmw_microros.h>   
#include <uxr/client/transport.h>        
#include <rcutils/allocator.h>
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
   RobotArm_t g_arm;
   RosInterface_t g_ros;
   volatile bool     g_net_ready = false;
   volatile uint32_t g_heap_free = 0;
   volatile uint32_t j0_ready = 0;
   volatile uint32_t j0_tmc   = 0;
volatile uint16_t g_enc_raw = 0;   /* encoder probe */
volatile uint32_t g_enc_ok = 0, g_enc_err = 0, g_enc_diaagc = 0;
   volatile uint32_t g_fault_code = 0;   /* 1 = stack overflow, 2 = malloc failed */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlLoop */
osThreadId_t ControlLoopHandle;
uint32_t ControlLoopBuffer[ 1024 ];
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
uint32_t ROS2Buffer[ 3000 ];
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
bool   cubemx_transport_open(struct uxrCustomTransport *t);
bool   cubemx_transport_close(struct uxrCustomTransport *t);
size_t cubemx_transport_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout, uint8_t *err);
void  *microros_allocate(size_t size, void *state);
void   microros_deallocate(void *ptr, void *state);
void  *microros_reallocate(void *ptr, size_t size, void *state);
void  *microros_zero_allocate(size_t n, size_t size, void *state);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartControlLoopTask(void *argument);
void StartSafetyTask(void *argument);
void StartMotionPlannerTask(void *argument);
void StartRosTask(void *argument);
void StartDiagTask(void *argument);
void StartHomingTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{

    (void)xTask; (void)pcTaskName;
    g_fault_code = 1;
    __disable_irq();
    for(;;) {}

   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{

    g_fault_code = 2;
    __disable_irq();
    for(;;) {}


   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
    g_net_ready = true;
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
  RobotArm_Init(&g_arm); // TODO: check return value once joints 2-5 are wired

  TickType_t lastWake = osKernelGetTickCount();
  const uint32_t period_ticks = 1; // 1ms -> 1kHz control loop

  for(;;)
  {
    j0_ready = g_arm.joints[0].ready ? 1u : 0u;
    j0_tmc   = (uint32_t)Joint_GetTmcStatus(&g_arm.joints[0]);
    RobotArm_Update(&g_arm, 0.001f);
    lastWake += period_ticks;
    osDelayUntil(lastWake);
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
    osDelay(1);
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
  while (!g_net_ready) { osDelay(50); }

  rmw_uros_set_custom_transport(false, (void *)"192.168.50.1",
      cubemx_transport_open, cubemx_transport_close,
      cubemx_transport_write, cubemx_transport_read);

  rcutils_allocator_t a = rcutils_get_zero_initialized_allocator();
  a.allocate      = microros_allocate;
  a.deallocate    = microros_deallocate;
  a.reallocate    = microros_reallocate;
  a.zero_allocate = microros_zero_allocate;
  if (!rcutils_set_default_allocator(&a)) { for(;;) osDelay(1000); }

  bool ok = RosInterface_Init(&g_ros, &g_arm);   /* blocks until the agent answers */
  g_heap_free = xPortGetFreeHeapSize();
  if (ok) { RosInterface_Task(NULL); }
  for(;;) { osDelay(1000); }        
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
  /* Encoder probe (read-only). Joint 0 is open-loop, so nothing else uses SPI1.
   * Remove before closed loop: one reader per bus. */
  AS5047P_t enc;
  AS5047P_Init(&enc, &hspi1, GPIOA, GPIO_PIN_4);
  for(;;)
  {
    uint16_t v;
    if (AS5047P_ReadAngleRaw(&enc, &v)) { g_enc_raw = v; g_enc_ok++; } else { g_enc_err++; }
    if (AS5047P_ReadRegister(&enc, 0x3FFC, &v)) { g_enc_diaagc = v; }
    osDelay(20);
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


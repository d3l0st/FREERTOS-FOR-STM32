/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "geometry_msgs/msg/twist.h"
#include <nav_msgs/msg/odometry.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64.h>

#include <usart.h>
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
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  {
    /* USER CODE BEGIN 5 */
  
    // micro-ROS configuration
  
    rmw_uros_set_custom_transport(
      true,
      (void *) &huart3,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);
  
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;
  
    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__); 
    }
  
    // micro-ROS app
  
    rcl_publisher_t publisher;
    rcl_publisher_t left_rpm_publisher;
    rcl_publisher_t right_rpm_publisher;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rcl_subscription_t subscriber;
    rcl_subscription_t cmd_vel_sub;
    rcl_timer_t timer;
    rcl_timer_t ControlTimer;

    rclc_executor_t executor_pub;
    rclc_executor_t executor_enc_l_pub;
    rclc_executor_t executor_enc_r_pub;
    rclc_executor_t executor_rpm_pub;
    rclc_executor_t executor_sub;
    

    std_msgs__msg__Int32 msg;
    std_msgs__msg__Float32 left_enc_rpm;
    std_msgs__msg__Float32 right_enc_rpm;
    std_msgs__msg__Int32 msg_sub;
    geometry_msgs__msg__Twist msg_cmd_vel;
    nav_msgs__msg__Odometry odom_msg;
    Odometry odometry;

  
    allocator = rcl_get_default_allocator();
  
    //create init_options
    rclc_support_init(&support, 0, NULL, &allocator);
  
    // create node
    rclc_node_init_default(&node, "micro_ros_node", "", &support);

     // create subscriber
     rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "microros_subscriber");

    // create subscriber cmd_vel
    rclc_subscription_init_best_effort(
      &cmd_vel_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel");

    // create publisher odom
    rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom");
  
    msg.data = 0;
  
    for(;;)
    {
      rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
      if (ret != RCL_RET_OK)
      {
        printf("Error publishing (line %d)\n", __LINE__); 
      }
      
      msg.data++;
      osDelay(10);
    }

    // create publisher left_enc_rpm
     rclc_publisher_init_default(
      &left_rpm_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "left_enc_rpm");
  
    // create publisher right_enc_rpm
     rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "right_enc_rpm");

    // create timer

    const unsigned int samplingT = 20;
    const unsigned int timer_timeout = 10;  

    rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback);

    // create timer   
    rclc_timer_init_default(
        &ControlTimer,
        &support,
        RCL_MS_TO_NS(samplingT),
        MotorControll_callback);

    // create executor
    rclc_executor_init(&executor_pub, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor_pub, &timer);

    rclc_executor_init(&executor_rpm_pub, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor_rpm_pub, &ControlTimer);

    rclc_executor_init(&executor_sub, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor_sub, &subscriber,         &msg_sub,     &subscription_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor_sub, &cmd_vel_sub,        &msg_cmd_vel, &cmd_vel_callback,      ON_NEW_DATA);
    msg_pub.data = 0;
    /* USER CODE END 5 */
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


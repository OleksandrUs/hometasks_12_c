/*
 * homework, main.c
 *
 * Purpose: create project which contains inter-task communication and synchronisation
 * (RTOS Task Notifications, Stream and Message Buffers, Queues, Binary Semaphores,
 * Counting Semaphores, Mutexes).
 * 
 * In the program a few tasks are created. Two tasks are used to read the acceleration values 
 * from LSM303DLHC sensor, which is on the STM32F303 Discovery Board, along the X and Y axes respectively.
 * A mutex is used to synchronize these tasks and prevent them from reading data from the sensor simultaniously.
 * The LSM303DLHC is connected to the microcontroller via I2C bus. These raw values (in hardware units)
 * are put into queues. For acceleration values along X and Y axes two separate queues are used.
 * Another task, which has a role of a gateeeper, reads acceleration values from both queues,
 * converts them to the physical values of acceleration in fractions of gravitational acceleration
 * and formats a string (array of char), which is put into a stream buffer and passed to another task.
 * This task is also converts the raw acceleration values into X and Y indexes that are used to index
 * 2D array (matrix). The 2D array contains the pin numbers LEDs connected to. This indexes are packed
 * into a struct instance and passed to the LEDs controller task by means of another queue. The LEDs controller
 * task uses received indexes to determine which LED should be switched on. Changes in the slope of the
 * Discovery board will cause changes in the position of a switched on LED. So, moving (circular motion) 
 * the Discovery Board creates nice visual effects on the LEDs. There is a task that reads the available data from 
 * the stream buffer and transmits them via USART2 (USB - RS232 converter is needed to check this functionality).
 * There are also two tasks that are used to test the LEDs. One of these tasks reads the user button state
 * and notifies the other task wheter the button pressed or released. The event group is used for notifications.
 * If the button is pressed, the tasks that reads data from the sensor, process these data and controls the LEDs
 * are suspended, and all the LEDs are switched on. It allows to check if all the LEDs work. Releasing the
 * user button resumes suspended threads execution. When the button is pressed, the text message is transmitted
 * via USART2. To synchronize the tasks that use USART2 to transmit text messages a semaphore is used.
 * 
 *
 * @author Oleksandr Ushkarenko
 * @version 1.0 03/10/2021
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "stdio.h"
#include "string.h"

/*
 * These identifiers are used to determine the microcontroller pins
 * the eight color LEDs connected to.
 */
#define BLUE_LED_1		GPIO_PIN_8
#define RED_LED_1 		GPIO_PIN_9
#define ORANGE_LED_1 	GPIO_PIN_10
#define GREEN_LED_1		GPIO_PIN_11
#define BLUE_LED_2 		GPIO_PIN_12
#define RED_LED_2 		GPIO_PIN_13
#define ORANGE_LED_2	GPIO_PIN_14
#define GREEN_LED_2 	GPIO_PIN_15
#define ALL_LED_PINS (GREEN_LED_1 | ORANGE_LED_1 | BLUE_LED_1 | RED_LED_1 | GREEN_LED_2 | ORANGE_LED_2 | BLUE_LED_2 | RED_LED_2)

/*
 * The threshold below which acceleration does not affect the change in the state of the LEDs.
 */
#define ACCEL_THRESHOLD 						1024

/*
 * The 2D array (matrix) has dimensions 5x5. Therefore, the indexes are between 0 and 4. 
 */
#define PIN_ARRAY_MAX_INDEX 				4
#define PIN_ARRAY_MIN_INDEX 				0

/*
 * The default acceleration values that are used before the data is read from the sensor. 
 */
#define DEFAULT_X_ACCEL_VALUE 			0
#define DEFAULT_Y_ACCEL_VALUE 			0

/*
 * This identifier is used to adjust the sensitivity; the more value, the less sensitivity.
 */
#define SENSITIVITY 								256

/*
 * The period of the button state reading.
 */
#define BUTTON_READ_STATE_PERIOD 		100

/* 
 * These identifiers are used to describe LSM303DLHC sensor's register addresses and setting values.
 */
#define LSM303DLHC_ADDRESS 					0x32
#define LSM303DLHC_SETTINGS 				0x47 // 50 Hz, normal mode, XYZ axes enabled
#define LSM303DLHC_SETTINGS_ADDR 		0x20
#define LSM303DLHC_RESOLUTION 			0x90 // high resolution
#define LSM303DLHC_RESOLUTION_ADDR 	0x21
#define LSM303DLHC_X_REG_LOW				0x28
#define LSM303DLHC_X_REG_HIGH 			0x29
#define LSM303DLHC_Y_REG_LOW 				0x2A
#define LSM303DLHC_Y_REG_HIGH 			0x2B

/*
 * The timeout and period values for reading data via I2C bus.
 */
#define I2C_TIMEOUT 								10
#define I2C_READ_DATA_PERIOD 				50
#define I2C_ATTEMPTS_NUM 						1

#define QUEUE_LENGTH 								4
#define STACK_SIZE 									128		// in 4-byte words
#define PRIORITY_NORMAL 						1

/*
 * Codes for button pressed and button released events.
 */
#define BUTTON_PRESSED 							0x01
#define BUTTON_RELEASED 						0x02

#define SEMAPHORE_MAX_COUNT 				1
#define SEMAPHORE_INITIAL_COUNT 		1

/*
 * The sizes of an array of char and a stream buffer.
 */
#define MAX_STR_LENGTH 							32
#define STREAM_BUFFER_LENGTH 				64

/*
 * UART timeout value.
 */
#define UART_TX_TIMEOUT 						100

/*
 * This identifier is used to convert raw data to physical units.
 */
#define G_FACTOR 										(9.81 / 16384)

/*
 * The structure is used as a wrapper for x and y indexex of the 2D array that
 * contains LED pin numbers.
 */
struct matrix_indexes
{
	uint8_t x;
	uint8_t y;
};

/*
 * The 2D array contains pin numbers LEDs connected to. This array is used to control
 * the state of the LEDs on the Discovery Board. The LED that is switched on depends on
 * the Discovery Board orientation. The acceleration values along X and Y axes read from 
 * the sensor converts to the indexes and used to get LED pin number.
 */
uint16_t leds[5][5] = {
{BLUE_LED_2, RED_LED_2, RED_LED_2, RED_LED_2, ORANGE_LED_2},
{GREEN_LED_1, BLUE_LED_1, RED_LED_1, ORANGE_LED_1, GREEN_LED_2},
{GREEN_LED_1, GREEN_LED_2, GREEN_LED_1, GREEN_LED_1, GREEN_LED_2},
{GREEN_LED_1, ORANGE_LED_2, RED_LED_2, BLUE_LED_2, GREEN_LED_2},
{ORANGE_LED_1, RED_LED_1, RED_LED_1, RED_LED_1, BLUE_LED_1}
};
	
// I2C and UART handles (these variables created by STM32CubeMX)
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

// Function prototypes (description of each function is above its implementation below)
void accel_init(void);
int8_t map_accel_to_array_index(int16_t acceleration);
void read_x_value_task(void *param);
void read_y_value_task(void *param);
void transmit_value_task(void *param);
void transmit_button_state_task(void *param);
void led_gatekeeper_task(void *param);
void led_controller_task(void *param);
void read_button_state_task(void *param);
void led_test_task(void *param);
void suspend_tasks(void);
void resume_tasks(void);

// Function prototypes created by STM32CubeMX
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

// Task handles
TaskHandle_t read_x_value_task_handle;
TaskHandle_t read_y_value_task_handle;
TaskHandle_t transmit_value_task_handle;
TaskHandle_t transmit_button_state_task_handle;
TaskHandle_t led_gatekeeper_task_handle;
TaskHandle_t led_controller_task_handle;
TaskHandle_t read_button_state_task_handle;
TaskHandle_t led_test_task_handle;

// Queue handles
QueueHandle_t accel_X_queue_handle;
QueueHandle_t accel_Y_queue_handle;
QueueHandle_t matrix_indexes_queue_handle;

// Event groups handle
EventGroupHandle_t event_group_handle;

// Semaphore and mutex handles
SemaphoreHandle_t sensor_mutex;
SemaphoreHandle_t uart_semaphore;

// Stream buffer handle
StreamBufferHandle_t stream_buffer_handle;

/*
 * The main function of the program (the entry point).
 * Demonstration of using inter-task communication and synchronisation.
 */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

	accel_init();
	
	accel_X_queue_handle = xQueueCreate(QUEUE_LENGTH, sizeof(int16_t));
	accel_Y_queue_handle = xQueueCreate(QUEUE_LENGTH, sizeof(int16_t));
	matrix_indexes_queue_handle = xQueueCreate(QUEUE_LENGTH, sizeof(struct matrix_indexes));
	
	if((accel_X_queue_handle == NULL) || (accel_Y_queue_handle == NULL) || (matrix_indexes_queue_handle == NULL)){
		Error_Handler();
	}
	
	if(xTaskCreate(read_x_value_task, "Read X value task", STACK_SIZE, NULL, PRIORITY_NORMAL, &read_x_value_task_handle) != pdPASS){
		Error_Handler();
	}
	
	if(xTaskCreate(read_y_value_task, "Read Y value task", STACK_SIZE, NULL, PRIORITY_NORMAL, &read_y_value_task_handle) != pdPASS){
		Error_Handler();
	}
	
	if(xTaskCreate(transmit_value_task, "Transmitt value task", STACK_SIZE, NULL, PRIORITY_NORMAL, &transmit_value_task_handle) != pdPASS){
		Error_Handler();
	}
	
	if(xTaskCreate(transmit_button_state_task, "Transmitt button state task", STACK_SIZE, NULL, PRIORITY_NORMAL, &transmit_button_state_task_handle) != pdPASS){
		Error_Handler();
	}
	
	if(xTaskCreate(led_gatekeeper_task, "LED gatekeeper task", STACK_SIZE, NULL, PRIORITY_NORMAL, &led_gatekeeper_task_handle) != pdPASS){
		Error_Handler();
	}
	
	if(xTaskCreate(led_controller_task, "LED controller task", STACK_SIZE, NULL, PRIORITY_NORMAL, &led_controller_task_handle) != pdPASS){
		Error_Handler();
	}

	if(xTaskCreate(read_button_state_task, "Read button state task", STACK_SIZE, NULL, PRIORITY_NORMAL, &read_button_state_task_handle) != pdPASS){
		Error_Handler();
	}
	
	if(xTaskCreate(led_test_task, "LED test task", STACK_SIZE, NULL, PRIORITY_NORMAL, &led_test_task_handle) != pdPASS){
		Error_Handler();
	}

	event_group_handle = xEventGroupCreate();
	sensor_mutex = xSemaphoreCreateMutex();
	uart_semaphore = xSemaphoreCreateCounting(SEMAPHORE_MAX_COUNT, SEMAPHORE_INITIAL_COUNT);
	
	if((event_group_handle == NULL) || (sensor_mutex == NULL) || (uart_semaphore == NULL)){
		Error_Handler();
	}
	
	stream_buffer_handle = xStreamBufferCreate(STREAM_BUFFER_LENGTH, MAX_STR_LENGTH);
	if(stream_buffer_handle == NULL){
		Error_Handler();
	}
	
	vTaskStartScheduler();
	
	while(1){
		
  }
}

/*
 * The function checks the LSM303DLHC presence on the I2C bus and waits unless the sensor is ready.
 * When the sensor is ready, it is confogured by writing values to the configuration registers.
 * The frequency of acceleration measurement is 50 Hz, normal mode, XYZ axes enabled, high resolution mode.
 * If an error occurs during I2C communication process, the error handling function is called.
 */
void accel_init(void)
{
	HAL_StatusTypeDef status;
	while(1) { 	// wait...
    status = HAL_I2C_IsDeviceReady(&hi2c1, LSM303DLHC_ADDRESS, I2C_ATTEMPTS_NUM, HAL_MAX_DELAY);
    if(status == HAL_OK){
      break;
		}
  }
	
	uint8_t settings = LSM303DLHC_SETTINGS;
	
	if(HAL_I2C_Mem_Write(&hi2c1, LSM303DLHC_ADDRESS, LSM303DLHC_SETTINGS_ADDR, I2C_MEMADD_SIZE_8BIT, &settings, 1, I2C_TIMEOUT) != HAL_OK){
		Error_Handler();
	}
  
	settings = LSM303DLHC_RESOLUTION;
	if(HAL_I2C_Mem_Write(&hi2c1, LSM303DLHC_ADDRESS, LSM303DLHC_RESOLUTION_ADDR, I2C_MEMADD_SIZE_8BIT, &settings, 1, I2C_TIMEOUT) != HAL_OK){
		Error_Handler();
	}
}

/*
 * This function converts the raw acceleration data to the array index. 
 * The result of convertion (i.e. an array index) is used to detrmine which LED should be switched on.
 */ 
int8_t map_accel_to_array_index(int16_t acceleration)
{
	int8_t index;
	index = (int16_t) acceleration / SENSITIVITY + 2;
	
	if(index > PIN_ARRAY_MAX_INDEX){
			index = PIN_ARRAY_MAX_INDEX;
	} else if(index < PIN_ARRAY_MIN_INDEX){
			index = PIN_ARRAY_MIN_INDEX;
	}
	
	return index;
}

/*
 * This task function reads from the sensor the acceleration value along the X axis,
 * and puts it into the queue. The mutex is used for tasks synchronization.
 *
 * @param a value that is passed as the parameter to the created task.
 */
void read_x_value_task(void *param)
{
	uint8_t data[2];
	int16_t x_accel;
	HAL_StatusTypeDef status[2];

	while(1){
		xSemaphoreTake(sensor_mutex, portMAX_DELAY);
		// HIGH and LOW bytes of 16-bit acceleration value are read separately due to the fact
		// that the sensor does NOT increment automatically the data register address after the procedure of reading. 
		status[0] = HAL_I2C_Mem_Read(&hi2c1, LSM303DLHC_ADDRESS, LSM303DLHC_X_REG_LOW, I2C_MEMADD_SIZE_8BIT, &data[0], 1, I2C_TIMEOUT);
		status[1] = HAL_I2C_Mem_Read(&hi2c1, LSM303DLHC_ADDRESS, LSM303DLHC_X_REG_HIGH, I2C_MEMADD_SIZE_8BIT, &data[1], 1, I2C_TIMEOUT);
		
		if((status[0] == HAL_OK) && (status[1] == HAL_OK)){
			
			// constructing the 16-bit variable from two 8-bit values (HIGH and LOW bytes merging).
			x_accel = (data[1] << 8) | data[0];	
			
			// If the Discovery Board is almost in the horisontal position, and if 
			// the aceleration value is lower then the threshold, the read value is set to its default
			// value to prevent LEDs blinking from the measurement noise.
			if((x_accel < ACCEL_THRESHOLD) && (x_accel > -ACCEL_THRESHOLD)){
				x_accel = DEFAULT_X_ACCEL_VALUE;
			}

			xQueueSend(accel_X_queue_handle, &x_accel, portMAX_DELAY);
			xSemaphoreGive(sensor_mutex);
		}
			vTaskDelay(I2C_READ_DATA_PERIOD);
	}
}

/*
 * This task function reads from the sensor the acceleration value along the Y axis,
 * and puts it into the queue. The mutex is used for tasks synchronization.
 *
 * @param a value that is passed as the parameter to the created task.
 */
void read_y_value_task(void *param)
{
	uint8_t data[2];
	int16_t y_accel;
	HAL_StatusTypeDef status[2];

	while(1){
		
		xSemaphoreTake(sensor_mutex, portMAX_DELAY);
		// HIGH and LOW bytes of 16-bit acceleration value are read separately due to the fact
		// that the sensor does NOT increment automatically the data register address after the procedure of reading. 
		status[0] = HAL_I2C_Mem_Read(&hi2c1, LSM303DLHC_ADDRESS, LSM303DLHC_Y_REG_LOW, I2C_MEMADD_SIZE_8BIT, &data[0], 1, I2C_TIMEOUT);
		status[1] = HAL_I2C_Mem_Read(&hi2c1, LSM303DLHC_ADDRESS, LSM303DLHC_Y_REG_HIGH, I2C_MEMADD_SIZE_8BIT, &data[1], 1, I2C_TIMEOUT);
		if((status[0] == HAL_OK) && (status[1] == HAL_OK)){

			// constructing the 16-bit variable from two 8-bit values (HIGH and LOW bytes merging).
			y_accel = (data[1] << 8) | data[0];
			
			// If the Discovery Board is almost in the horisontal position, and if 
			// the aceleration value is lower then the threshold, the read value is set to its default
			// value to prevent LEDs blinking from the measurement noise.
			if((y_accel < ACCEL_THRESHOLD) && (y_accel > -ACCEL_THRESHOLD)){
				y_accel = DEFAULT_Y_ACCEL_VALUE;
			}

			xQueueSend(accel_Y_queue_handle, &y_accel, portMAX_DELAY);
			xSemaphoreGive(sensor_mutex);
		}
			vTaskDelay(I2C_READ_DATA_PERIOD);
		}
}

/*
 * This task function reads bytes (chars) from the stream buffer and transmits them via USART.
 * The semaphore is used to synchronize tasks (there is one more task that uses the same USART).
 *
 * @param a value that is passed as the parameter to the created task.
 */
void transmit_value_task(void *param)
{
	size_t size;
	char text[STREAM_BUFFER_LENGTH];
	while(1){
		size = xStreamBufferReceive(stream_buffer_handle, (void*)text, STREAM_BUFFER_LENGTH, portMAX_DELAY);
		text[size] = '\0';
		
		xSemaphoreTake(uart_semaphore, portMAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*)text, strlen(text), portMAX_DELAY);
		xSemaphoreGive(uart_semaphore);
	}
}

/*
 * The task function reads periodically the user button state and transmits the text message
 * via USART if the user button is pressed. The semaphore is used to synchronize tasks.
 *
 * @param a value that is passed as the parameter to the created task.
 */
void transmit_button_state_task(void *param)
{
	const char str[] = "Button PRESSED\n\r";
  while(1){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET){
			xSemaphoreTake(uart_semaphore, portMAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), UART_TX_TIMEOUT);
			xSemaphoreGive(uart_semaphore);	
			vTaskDelay(BUTTON_READ_STATE_PERIOD);
		}
	}
}

/*
 * This task function reads the acceleration raw values along X and Y axes from two different queues,
 * formats the text string that contains the information about acceleration in physical units and
 * puts it into the stream buffer, converts the acceleration values into 2D array indexes and passes
 * them to the led controller task function (having previously copied them into the structure fields)
 * by means of the other queue.
 *
 * @param a value that is passed as the parameter to the created task.
 */
void led_gatekeeper_task(void *param)
{
	struct matrix_indexes indexes;
	int16_t x_accel;
	int16_t y_accel;
	char str[MAX_STR_LENGTH];

	while(1){
		if((xQueueReceive(accel_X_queue_handle, &x_accel, portMAX_DELAY) == pdPASS) &&
			 (xQueueReceive(accel_Y_queue_handle, &y_accel, portMAX_DELAY) == pdPASS)){	
				
				sprintf(str, "x = %.1f g; y = %.1f g\r\n", (float)x_accel * G_FACTOR, (float)y_accel * G_FACTOR);
				xStreamBufferSend(stream_buffer_handle, str, strlen(str), portMAX_DELAY);
					
				indexes.x = map_accel_to_array_index(x_accel);
				indexes.y = map_accel_to_array_index(y_accel);
					
				xQueueSend(matrix_indexes_queue_handle, &indexes, portMAX_DELAY);
		}
	}
}

/*
 * This task function reads the data from the queue thas contains indexes of the 2D array,
 * which contains pin numbers LEDs connected to, and uses it to determine which LED should be
 * switched on. The indexes of the array are dependent on the acceleration values read from the sensor,
 * so moving the Discovery Board causes changes in the LEDs states.
 *
 * @param a value that is passed as the parameter to the created task.
 */
void led_controller_task(void *param)
{		
	struct matrix_indexes indexes;
	while(1){
		if(xQueueReceive(matrix_indexes_queue_handle, &indexes, portMAX_DELAY) == pdTRUE){
			HAL_GPIO_WritePin(GPIOE, ALL_LED_PINS, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, leds[indexes.x][indexes.y], GPIO_PIN_SET);
		}
	}
}

/*
 * This task function reads the user button state and notifies another task by means
 * of an event group whether the button has been pressed or released. 
 *
 * @param a value that is passed as the parameter to the created task.
 */
void read_button_state_task(void *param)
{
	while(1){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET){
			xEventGroupSetBits(event_group_handle, BUTTON_PRESSED);
		}
		else{
			xEventGroupSetBits(event_group_handle, BUTTON_RELEASED);
		}
		vTaskDelay(BUTTON_READ_STATE_PERIOD);
	}
}

/*
 * The task function waits for notification about user button state changes.
 * If the user button has been pressed (event bits being waited for became set),
 * the other tasks are suspended and all the LEDs are switched on. If the user button has been released,
 * all the LEDs are switched off and the other tasks are resumed. Suspending other tasks
 * prevents LEDs from blinking.
 *
 * @param a value that is passed as the parameter to the created task.
 */
void led_test_task(void *param)
{
	EventBits_t event_group_value;
  while(1){
   event_group_value = xEventGroupWaitBits(event_group_handle, BUTTON_PRESSED, pdTRUE, pdTRUE, portMAX_DELAY);                               
   if((event_group_value & BUTTON_PRESSED) != 0){
			suspend_tasks();
			HAL_GPIO_WritePin(GPIOE, ALL_LED_PINS, GPIO_PIN_SET);
   }
	 
	 event_group_value = xEventGroupWaitBits(event_group_handle, BUTTON_RELEASED, pdTRUE, pdTRUE, portMAX_DELAY);
   if((event_group_value & BUTTON_RELEASED) != 0){
			HAL_GPIO_WritePin(GPIOE, ALL_LED_PINS, GPIO_PIN_RESET);
			resume_tasks();
   }
 }
}

/*
 * The function suspends the tasks that read data from the sensor, transmits data via USART,
 * processes data and controls the LEDs states.
 */
void suspend_tasks(void)
{
	vTaskSuspend(read_x_value_task_handle);
	vTaskSuspend(read_y_value_task_handle);
	vTaskSuspend(transmit_value_task_handle);
	vTaskSuspend(led_gatekeeper_task_handle);
	vTaskSuspend(led_controller_task_handle);
}

/*
 * The function resumes the tasks that read data from the sensor, transmits data via USART,
 * processes data and controls the LEDs states.
 */
void resume_tasks(void)
{
	vTaskResume(read_x_value_task_handle);
	vTaskResume(read_y_value_task_handle);
	vTaskResume(transmit_value_task_handle);
	vTaskResume(led_gatekeeper_task_handle);
	vTaskResume(led_controller_task_handle);
}

/*=================================================================================*/
/*========= All the functions below were created by means of STM32CubeMX ==========*/
/*=================================================================================*/

/*
 * System Clock Configuration
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * I2C1 Initialization Function
 */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * USART2 Initialization Function
 */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * GPIO Initialization Function
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 PE11
                           PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/*
 * Period elapsed callback in non blocking mode
 * This function is called when TIM8 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM8) {
    HAL_IncTick();
  }
}

/*
 * This function is executed in case of error occurrence.
 * Two RED LEDs are switched on, and the other LEDs are switched off.
 */
void Error_Handler(void)
{
  __disable_irq();
	HAL_GPIO_WritePin(GPIOE, BLUE_LED_1 | BLUE_LED_2 | ORANGE_LED_1 | ORANGE_LED_2 | GREEN_LED_1 | GREEN_LED_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, RED_LED_1 | RED_LED_2, GPIO_PIN_SET);
  while (1)
  {
  }
}

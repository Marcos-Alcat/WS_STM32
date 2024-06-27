/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "FLASH_PAGE_F1.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

QueueHandle_t CM;


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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int read_sw1_state = 0;

typedef struct
{
	uint32_t maximo;
	uint32_t minimo;
}secuencias;

uint32_t estados = 0;

secuencias secuencia[3];
secuencias secuencia_cola;

uint32_t valores_max[5] = {450,300,100,200,2}; ///VALORES QUE RECIBE DE COLA
uint32_t valores_min[5] = {400,30,10,0,1}; ///VALORES QUE RECIBE DE COLA

int tarea = 0;

TaskHandle_t xTarea_config_Handle = NULL,xTarea_memoria_Handle=NULL;

void Tarea_config( void *pvParameters )
{
	unsigned portBASE_TYPE uxPriority;
	uxPriority = uxTaskPriorityGet( NULL );

	secuencias cola,envio;
	int contador=0;
	for( ;; )
	{
		 if(read_sw1_state)
		 {
			 envio.maximo = 78;
			 envio.minimo = 7;
			 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			 xQueueReceive(CM,&cola,portMAX_DELAY);
			 xQueueSend(CM, &envio, portMAX_DELAY);
			 vTaskPrioritySet( xTarea_memoria_Handle, 4 );
			 /// vTaskPrioritySet( NULL, 1 ); LE BAJO LA PRIORIDAD ASI LA MEMORIA VUELVE GRABA , BAJA Y QUEDAN LAS OTRAS 4 CON MAS PRIORIDAD
		 }
		else
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		 xQueuePeek(CM,&cola,portMAX_DELAY);
		 ///xQueueSend(my_queue, &envio, portMAX_DELAY); X2 ENVIO A ALARMA Y A PANTALLA

		 tarea = 2;

	}
}


void Tarea_memoria( void *pvParameters )
{
unsigned portBASE_TYPE uxPriority;
uxPriority = uxTaskPriorityGet( NULL );

secuencias recepcion;
for( ;; )
	{
	/// Tarea memoria ///////////////////////
		  tarea = 1;
		  if(estados==0)
		  {
			  /// solo de prueba /////////////////////////////////////////////////

				Flash_Write_NUM(0x08005C10, 100);
				Flash_Write_NUM(0x08006010, 0);

				Flash_Write_NUM(0x08006410, 70);
				Flash_Write_NUM(0x08006810, 30);

				Flash_Write_NUM(0x08006C10, 40);
				Flash_Write_NUM(0x08007010, 25);
			  /// //////////////////////////////////////////////////////////////

			  ///inicializacion memoria //////////////////////////////////
			   secuencia[2].maximo = Flash_Read_NUM(0x08006C10);
			   secuencia[2].minimo = Flash_Read_NUM(0x08007010);

			   secuencia[1].maximo = Flash_Read_NUM(0x08006410);
			   secuencia[1].minimo = Flash_Read_NUM(0x08006810);

			   secuencia[0].maximo = Flash_Read_NUM(0x08005C10);
			   secuencia[0].minimo = Flash_Read_NUM(0x08006010);
			  /// //////////////////////////////////////////////////////////////////////////////////
			  /// inicializa - carga : ARRANCA EL SISTEMA CON EL VALOR ULTIMO GUARDADO DE LA SECUENCIA EN MEMORIA, Y SE CARGA A LA COLA CM

			  xQueueSend(CM, &secuencia[0], portMAX_DELAY);
			  /// cambia estado
			  estados = 1;
		  }
		  else
		  {
			  /// CADA VES QUE RECIBE UN DATO DE LA COLA CM LO GUARDA EN PRIMER LUGAR , LOS DATOS ANTERIORWS LOS DESPLAZA 1 Y LUEGO GUARAD EN MEMORIA Y LUEGO SE BLOQUEA
			  xQueuePeek(CM,&recepcion,portMAX_DELAY);
			  /// graba

			  secuencia[2] = secuencia[1];
			  Flash_Write_NUM(0x08006C10, secuencia[2].maximo);
			  Flash_Write_NUM(0x08007010, secuencia[2].minimo);

			  secuencia[1] = secuencia[0];
			  Flash_Write_NUM(0x08006410, secuencia[1].maximo);
			  Flash_Write_NUM(0x08006810, secuencia[1].minimo);

			  secuencia[0] = recepcion;
			  Flash_Write_NUM(0x08005C10, recepcion.maximo);
			  Flash_Write_NUM(0x08006010, recepcion.minimo);
			  /// si lo hago al reves pierdo datos

			  vTaskPrioritySet( xTarea_config_Handle, 1 );

		  }
		  /// ///////////////////////////
		  vTaskPrioritySet( NULL, 1 );
	}
}



void Tareas_restantes( void *pvParameters )
{
	unsigned portBASE_TYPE uxPriority;
	uxPriority = uxTaskPriorityGet( NULL );

	secuencias recepcion;
	for( ;; )
	{
		tarea = 3;
		if(read_sw1_state)
		 {
			 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			 vTaskPrioritySet( xTarea_config_Handle, 3 );
		 }
		else
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
}





int main( void )
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

	xTaskCreate( Tarea_memoria, "Tarea_memoria", 1000, NULL, 4, &xTarea_memoria_Handle );///MAX_priority???????
	xTaskCreate( Tarea_config, "Tarea_config", 1000, NULL, 3, &xTarea_config_Handle );
	xTaskCreate( Tareas_restantes, "Tareas_restantes", 1000, NULL, 2, NULL );
	/* El manipulador de tarea es el último parámetro. */
	CM = xQueueCreate(1,sizeof(secuencias));
	/* Inicio el Scheduler para que last areas comiencen a ejecutarse. */
	vTaskStartScheduler();

	for( ;; );
}


/// int valor;
/// valor = Flash_Read_NUM(0x08007410); ////COMPROBAR QUE CUANDO LEO UNA HOJA SIN ESCRIBIR VALE 0
/// QUE PASA SI PONGO MAXIMO 40 Y MIN 60, O SEA MAX < MIN???????????????


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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(read_sw1_state) read_sw1_state = 0;
	else read_sw1_state = 1;

	//HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
}

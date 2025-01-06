/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	IDLE ,
	NS_GREEN_EW_RED,
	NS_YELLOW_EW_RED,
	NS_RED_EW_GREEN,
	NS_RED_EW_YELLOW,
	NS_RED_EW_RED,
	PED_CROSSING,
} TrafficState;
TrafficState currentState = IDLE;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GREEN_TIME 5000     // Duration for the green light in milliseconds
#define YELLOW_TIME 2000    // Duration for the yellow light in milliseconds
#define PED_TIME 8000       // Duration for pedestrian crossing in milliseconds
#define ALL_RED_TIME 1000   // Duration for all lights red (safety delay) in milliseconds

// Define GPIO pins for traffic light LEDs
#define NS_GREEN_LED GPIO_PIN_2  // GPIO pin for North-South green light
#define NS_YELLOW_LED GPIO_PIN_1 // GPIO pin for North-South yellow light
#define NS_RED_LED GPIO_PIN_0    // GPIO pin for North-South red light
#define EW_GREEN_LED GPIO_PIN_5  // GPIO pin for East-West green light
#define EW_YELLOW_LED GPIO_PIN_6 // GPIO pin for East-West yellow light
#define EW_RED_LED GPIO_PIN_7    // GPIO pin for East-West red light
#define PED_GREEN_LED GPIO_PIN_3 // GPIO pin for pedestrian green light
#define PED_RED_LED GPIO_PIN_4   // GPIO pin for pedestrian red light

// Define GPIO pin for pedestrian button
#define PED_BUTTON_PIN GPIO_PIN_8 // GPIO pin for pedestrian crossing button
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Global variables

/**
 * @brief Flag to indicate a pedestrian crossing request.
 *        Set to 1 when the pedestrian button is pressed, and cleared after handling the request.
 */
volatile uint8_t PedestrianRequest = 0;

/**
 * @brief Stores the timestamp of the last state transition.
 *        Used to calculate the elapsed time for each traffic light state.
 */
uint32_t lastTransitionTime = 0;

/**
 * @brief Determines the next direction for the traffic lights after the all-red state.
 *        - 1: East-West direction will turn green next.
 *        - 0: North-South direction will turn green next.
 */
uint8_t nextDirection = 1; // Default set to East-West as the next green direction after North-South

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// Function prototypes for traffic light control system

/**
 * @brief Implements the Finite State Machine (FSM) for the traffic light system.
 *        Handles state transitions based on timing and pedestrian requests.
 */
void trafficLightFSM(void);

/**
 * @brief Updates the traffic light LEDs based on the current state.
 *        Turns on/off the appropriate LEDs for the traffic lights and pedestrian signals.
 * @param state The current state of the traffic light system.
 */
void updateLights(TrafficState state);


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
	/* USER CODE BEGIN 2 */
	lastTransitionTime = HAL_GetTick();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		trafficLightFSM();
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void trafficLightFSM(void) {
	uint32_t currentTime = HAL_GetTick();   // Get the current system time in milliseconds

	switch (currentState) {
	case IDLE :
		currentState = NS_GREEN_EW_RED;    	// Start with North-South green and East-West red
		updateLights(currentState);			// Update the lights to reflect the current state
		break;

	case NS_GREEN_EW_RED:
		if ((currentTime - lastTransitionTime) >= GREEN_TIME) {// Check if the green time has elapsed
			currentState = NS_YELLOW_EW_RED;	// Transition to North-South yellow and East-West red
			lastTransitionTime = currentTime;	// Update the transition time
			updateLights(currentState);			// Update the lights to reflect the current state
		}
		break;

	case NS_YELLOW_EW_RED:
		if ((currentTime - lastTransitionTime) >= YELLOW_TIME) {// Check if the yellow time has elapsed
			currentState = NS_RED_EW_RED;			// Transition to all red state
			lastTransitionTime = currentTime;		// Update the transition time
			updateLights(currentState);				// Update the lights to reflect the current state
		}
		break;

	case NS_RED_EW_RED:
		if ((currentTime - lastTransitionTime) >= ALL_RED_TIME) {// Check if the all-red time has elapsed
			if (PedestrianRequest) {	// Check if there is a pedestrian request
				PedestrianRequest = 0;	// Clear the pedestrian request flag
				currentState = PED_CROSSING;	// Transition to pedestrian crossing state
			} else if (nextDirection == 0) { // If the next direction is North-South
				nextDirection = 1;		// Alternate to East-West for the next cycle
				currentState = NS_GREEN_EW_RED;	// Transition to North-South green and East-West red
			} else { // If the next direction is East-West
				nextDirection = 0; 		// Alternate to North-South for the next cycle
				currentState = NS_RED_EW_GREEN;	// Transition to North-South red and East-West green
			}
			lastTransitionTime = currentTime;	// Update the transition time
			updateLights(currentState);			// Update the lights to reflect the current state
		}
		break;

	case NS_RED_EW_GREEN:
		if ((currentTime - lastTransitionTime) >= GREEN_TIME) {// Check if the green time has elapsed
			currentState = NS_RED_EW_YELLOW;	// Transition to North-South red and East-West yellow
			lastTransitionTime = currentTime;	// Update the transition time
			updateLights(currentState);			// Update the lights for the new state
		}
		break;

	case NS_RED_EW_YELLOW:
		if ((currentTime - lastTransitionTime) >= YELLOW_TIME) {// Check if the yellow time has elapsed
			currentState = NS_RED_EW_RED;		// Transition to all red state
			lastTransitionTime = currentTime;	// Update the transition time
			updateLights(currentState);			// Update the lights for the new state
		}
		break;

	case PED_CROSSING:
		if ((currentTime - lastTransitionTime) >= PED_TIME) {// Check if the pedestrian crossing time has elapsed
			currentState = NS_RED_EW_RED;		// Transition back to the all red state
			lastTransitionTime = currentTime;	// Update the transition time
			updateLights(currentState);			// Update the lights for the new state
		}
		break;

	default:
		currentState = NS_GREEN_EW_RED;			// Default state if something unexpected occurs
		break;
	}
}

void updateLights(TrafficState state) {
	// Turn off all LEDs
	HAL_GPIO_WritePin(GPIOA, NS_GREEN_LED | NS_YELLOW_LED | NS_RED_LED |
			EW_GREEN_LED | EW_YELLOW_LED | EW_RED_LED |
			PED_GREEN_LED | PED_RED_LED, GPIO_PIN_RESET);

	// Turn on the LEDs for the current state
	switch (state) {
	case NS_GREEN_EW_RED:
		HAL_GPIO_WritePin(GPIOA, NS_GREEN_LED | EW_RED_LED, GPIO_PIN_SET); // Turn on NS green and EW red
		HAL_GPIO_WritePin(GPIOA, PED_RED_LED, GPIO_PIN_SET); // Keep pedestrian light red
		break;

	case NS_YELLOW_EW_RED:
		HAL_GPIO_WritePin(GPIOA, NS_YELLOW_LED | EW_RED_LED, GPIO_PIN_SET); // Turn on NS yellow and EW red
		HAL_GPIO_WritePin(GPIOA, PED_RED_LED, GPIO_PIN_SET); // Keep pedestrian light red
		break;

	case NS_RED_EW_GREEN:
		HAL_GPIO_WritePin(GPIOA, NS_RED_LED | EW_GREEN_LED, GPIO_PIN_SET); // Turn on NS red and EW green
		HAL_GPIO_WritePin(GPIOA, PED_RED_LED, GPIO_PIN_SET); // Keep pedestrian light red
		break;

	case NS_RED_EW_YELLOW:
		HAL_GPIO_WritePin(GPIOA, NS_RED_LED | EW_YELLOW_LED, GPIO_PIN_SET); // Turn on NS red and EW yellow
		HAL_GPIO_WritePin(GPIOA, PED_RED_LED, GPIO_PIN_SET); // Keep pedestrian light red
		break;

	case PED_CROSSING:
		HAL_GPIO_WritePin(GPIOA, NS_RED_LED | EW_RED_LED | PED_GREEN_LED, GPIO_PIN_SET); // All directions red, pedestrian green
		break;

	case NS_RED_EW_RED:
		HAL_GPIO_WritePin(GPIOA, NS_RED_LED | EW_RED_LED, GPIO_PIN_SET); // Turn on NS red and EW red
		HAL_GPIO_WritePin(GPIOA, PED_RED_LED, GPIO_PIN_SET); // Keep pedestrian light red
		break;
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PED_BUTTON_PIN) {// Check if the interrupt was caused by the pedestrian button
		PedestrianRequest = 1; // Set pedestrian request
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

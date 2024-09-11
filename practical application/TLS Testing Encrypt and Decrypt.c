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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rng.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../Middlewares/Third_Party/wolfSSL_wolfSSL_wolfSSL/wolfssl/wolfssl/wolfcrypt/rsa.h"
#include "../Middlewares/Third_Party/wolfSSL_wolfSSL_wolfSSL/wolfssl/wolfssl/wolfcrypt/aes.h"

#include "../Middlewares/Third_Party/wolfSSL_wolfSSL_wolfSSL/wolfssl/wolfssl/wolfcrypt/cryptocb.h"
#include "../Middlewares/Third_Party/wolfSSL_wolfSSL_wolfSSL/wolfssl/wolfssl/wolfcrypt/cmac.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#if 0
RsaKey rsaPublicKey;
/* holds the raw data from the key, maybe from a file like RsaPublicKey.der */
byte publicKeyBuffer[] = {0};

RsaKey rsaPrivateKey;
/* hold the raw data from the key, maybe from a file like RsaPrivateKey.der */
byte privateKeyBuffer[] = {0};

word32 idx = 0;                /*where to start reading into the buffer*/

byte in[] = {0}; /* plain text to encrypt */
byte out[128];
byte plain[128];

word32 outLen = BUFFER_E;

WC_RNG rng;

#endif

#if 1	/* AES 256 */
	#define AES_128_KEY_SIZE 16
	#define AES_128_IV_SIZE 16

	Aes enc;
	Aes dec;
	void* hint = NULL;
	int devId = INVALID_DEVID;

	int AesRet = 0;
	uint8_t AES_key[AES_128_KEY_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	uint8_t iv[AES_128_IV_SIZE]  = {0x0, 0x1, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

	// Multiple of 16 bytes
	uint8_t UART_msg[AES_BLOCK_SIZE] = {0x0, 0x1, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	uint8_t UART_msgCipher[AES_BLOCK_SIZE] = {0x00};
	uint8_t UART_msgPlain [AES_BLOCK_SIZE] = {0x00};

	int DecRet = 0;
#endif

#if 0 /* CMAC */
	#define CMAC_128_KEY_SIZE 16
	#define CMAC_SIZE 8
	#define MSG_SIZE  8

	Cmac cmac[1];
	uint8_t key[CMAC_128_KEY_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

	uint8_t mac[CMAC_SIZE];
	word32 macSize = 8;

	uint8_t msg[MSG_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

	int cmacRet = 0;
	int onSuccessFlag = 0;
#endif

#if 1 /* CMAC and AES 256 CBC Mode */

#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_RNG_Init();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

#if 0
  wc_RsaPublicKeyDecode(publicKeyBuffer, &idx, &rsaPublicKey, sizeof(publicKeyBuffer));

  wc_InitRng(&rng);

  outLen = wc_RsaPublicEncrypt(in, sizeof(in), out, sizeof(out), &rsaPublicKey, &rng);
  if(0 == outLen){ }

  outLen = wc_RsaPrivateKeyDecode(privateKeyBuffer, &idx, &rsaPrivateKey, sizeof(privateKeyBuffer));
  if(0 == outLen){ }

  word32 plainSz = wc_RsaPrivateDecrypt(out, outLen, plain, sizeof(plain), &rsaPrivateKey);
#endif

#if 1
  /*
   * https://www.wolfssl.com/
   * https://www.wolfssl.com/documentation/manuals/wolfssl/group__AES.html
   * */

  AesRet = wc_AesInit(&enc, hint, devId);
  if (AesRet == 0) {
	  AesRet |= wc_AesSetKey(&enc, AES_key, AES_BLOCK_SIZE, iv, AES_ENCRYPTION);
	  if (AesRet == 0) {
		  AesRet |= wc_AesSetIV(&enc, iv);
		  if (AesRet == 0) {
			  AesRet |= wc_AesCbcEncrypt(&enc, UART_msgCipher, UART_msg, sizeof(UART_msg));
			  if (AesRet == 0) {
				  AesRet = wc_AesInit(&dec, hint, devId);
				  AesRet |= wc_AesSetKey(&dec, AES_key, AES_BLOCK_SIZE, iv, AES_DECRYPTION);
				  AesRet |= wc_AesSetIV(&dec, iv);
				  if (AesRet == 0) {
					  AesRet |= wc_AesCbcDecrypt(&dec, UART_msgPlain, UART_msgCipher, sizeof(UART_msgCipher));
					  	  if (AesRet == 0) {
					  		  DecRet = 1;
					  	  }
				  }
			  }
		  }
	  }
  }
#endif

#if 0
  /*
   * https://www.wolfssl.com/
   * https://www.wolfssl.com/documentation/manuals/wolfssl/group__CMAC.html
   * */
  cmacRet = wc_InitCmac(cmac, key, CMAC_128_KEY_SIZE, WC_CMAC_AES, NULL);
  cmacRet |= wc_AesCmacGenerate(mac, &macSize, msg, MSG_SIZE, key, CMAC_128_KEY_SIZE);
  cmacRet |= wc_AesCmacVerify(mac, macSize, msg, MSG_SIZE, key, CMAC_128_KEY_SIZE);

  if(cmacRet == 0){
	  onSuccessFlag = 1;
  }

#endif

#if 0
  	/* Sender */
	AesRet = wc_AesInit(&enc, hint, devId);
    if (AesRet == 0) {
  	  AesRet |= wc_AesSetKey(&enc, AES_key, AES_BLOCK_SIZE, iv, AES_ENCRYPTION);
  	  if (AesRet == 0) {
  		  AesRet |= wc_AesSetIV(&enc, iv);
  		  if (AesRet == 0) {
  			  AesRet |= wc_AesCbcEncrypt(&enc, UART_msgCipher, UART_msg, sizeof(UART_msg));
  			  if (AesRet == 0) {
  				  cmacRet = wc_InitCmac(cmac, key, CMAC_128_KEY_SIZE, WC_CMAC_AES, NULL);
  				  cmacRet |= wc_AesCmacGenerate(mac, &macSize, UART_msgCipher, sizeof(UART_msgCipher), key, CMAC_128_KEY_SIZE);
  			  }
  		  }
  	  }
    }

    /* Receiver */
    if(cmacRet == 0){
    	cmacRet |= wc_AesCmacVerify(mac, macSize, UART_msgCipher, sizeof(UART_msgCipher), key, CMAC_128_KEY_SIZE);
    	if(cmacRet == 0){
    		AesRet = wc_AesInit(&dec, hint, devId);
    		AesRet |= wc_AesSetKey(&dec, AES_key, AES_BLOCK_SIZE, iv, AES_DECRYPTION);
    		AesRet |= wc_AesSetIV(&dec, iv);
    		if (AesRet == 0) {
    			AesRet |= wc_AesCbcDecrypt(&dec, UART_msgPlain, UART_msgCipher, sizeof(UART_msgCipher));
    			if (AesRet == 0) {
    				DecRet = 1;
    			}
    		}
    	}
    }
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

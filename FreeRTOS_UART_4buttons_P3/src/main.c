
/* Standard includes. */
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "stm32f4xx.h"

#define mainQUEUE_RECEIVE_TASK_PRIORITY        ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY    ( configMAX_PRIORITIES - 1 )
#define antiBounce_MS						(200/portTICK_RATE_MS)
#define mainQUEUE_SEND_PERIOD_MS            ( 200 / portTICK_RATE_MS )
#define mainSOFTWARE_TIMER_PERIOD_MS        ( 1000 / portTICK_RATE_MS )
#define mainQUEUE_LENGTH                    ( 5 )

/*-----------------------------------------------------------*/
static void prvSetupHardware( void );
static void vLED( void *pvParameters );
static void prvSemTask1( void *pvParameters );
static void prvSemTask2( void *pvParameters );
static void prvSemTask3( void *pvParameters );
static void prvSemTask4( void *pvParameters );
/*-----------------------------------------------------------*/

static xQueueHandle xQueue = NULL;
static xSemaphoreHandle xESem1 = NULL;
static xSemaphoreHandle xESem2 = NULL;
static xSemaphoreHandle xESem3 = NULL;
static xSemaphoreHandle xESem4 = NULL;

static volatile uint32_t ulCountOfTimerCallbackExecutions = 0;
static volatile uint32_t ulCountOfItemsReceivedOnQueue = 0;
static volatile uint32_t ulCountOfReceivedSemaphores = 0;

/*-----------------------------------------------------------*/

int main(void)
{
 xTimerHandle xExampleSoftwareTimer = NULL;

    prvSetupHardware();

    xQueue = xQueueCreate(     mainQUEUE_LENGTH,
                            sizeof( uint32_t ) );

    vQueueAddToRegistry( xQueue, ( signed char * ) "MainQueue" );
    vSemaphoreCreateBinary( xESem1 );
    vSemaphoreCreateBinary( xESem2 );
    vSemaphoreCreateBinary( xESem3 );
    vSemaphoreCreateBinary( xESem4 );

    vQueueAddToRegistry( xESem1, ( signed char * ) "x   EventSem1" );
    vQueueAddToRegistry( xESem2, ( signed char * ) "x   EventSem2" );
    vQueueAddToRegistry( xESem1, ( signed char * ) "x   EventSem3" );
    vQueueAddToRegistry( xESem2, ( signed char * ) "x   EventSem4" );

    xTaskCreate(     vLED,
                    ( signed char * ) "LEDS",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    mainQUEUE_RECEIVE_TASK_PRIORITY,
                    NULL );

    xTaskCreate(     prvSemTask1,
                    ( signed char * ) "Sem",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    mainEVENT_SEMAPHORE_TASK_PRIORITY,
                    NULL );

       xTaskCreate(     prvSemTask2,
                       ( signed char * ) "Sem2",
                       configMINIMAL_STACK_SIZE,
                       NULL,
                       mainEVENT_SEMAPHORE_TASK_PRIORITY,
                       NULL );

	  xTaskCreate(     prvSemTask3,
					  ( signed char * ) "Sem3",
					  configMINIMAL_STACK_SIZE,
					  NULL,
					  mainEVENT_SEMAPHORE_TASK_PRIORITY,
					  NULL );

	 xTaskCreate(     prvSemTask4,
					 ( signed char * ) "Sem4",
					 configMINIMAL_STACK_SIZE,
					 NULL,
					 mainEVENT_SEMAPHORE_TASK_PRIORITY,
					 NULL );

    xTimerStart( xExampleSoftwareTimer, 0 );
    vTaskStartScheduler();
    for( ;; );

 return 0;
}

static void vLED( void *pvParameters )
{
	uint32_t ulReceivedValue;
	uint16_t pin_toogle;
	int i_max,i;
	uint16_t pin_conv=1;

	    for( ;; ){
	        xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );
	        if(ulReceivedValue){
	        	i_max = ulReceivedValue - 45;	//-48(conv char to int) & +3(offset pins)
	        	pin_conv=1;
	        	for(i=0;i<i_max;i++){	//calculate pin register
	        		pin_conv*=2;
	        	}
	            ulCountOfItemsReceivedOnQueue++;
	            pin_toogle = !GPIO_ReadInputDataBit(GPIOB, pin_conv);
	            GPIO_WriteBit(GPIOB,pin_conv, pin_toogle); //0x000008, 8 pin_toogle
	            ulReceivedValue = 0;
	        }
	    }
}

/*-----------------------------------------------------------*/

void USART1_IRQHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
    	char cIn = USART1->DR;
    	if( (cIn == '1') || (cIn == '2') || (cIn == '3') || (cIn == '4')){
			xQueueSendFromISR( xQueue, &cIn, &xHigherPriorityTaskWoken );
    	}
    }
 }

/*-----------------------------------------------------------*/
static void prvSemTask1( void *pvParameters )
{
    uint32_t toSend;

    xSemaphoreTake( xESem1, 0 );
    int first;
    for( ;; )
    {
        xSemaphoreTake( xESem1, portMAX_DELAY );
        NVIC_DisableIRQ(EXTI0_IRQn);
        first=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0);

        if(first)
        {
        	toSend = 49;
            xQueueSend( xQueue, &toSend, 0 );
        }
        vTaskDelay(antiBounce_MS);
        NVIC_EnableIRQ(EXTI0_IRQn);
    }
}

/*-----------------------------------------------------------*/

static void prvSemTask2( void *pvParameters )
{
    uint32_t toSend;
    int first;
    xSemaphoreTake( xESem2, 0 );
    for( ;; )
    {
        xSemaphoreTake( xESem2, portMAX_DELAY );
        NVIC_DisableIRQ(EXTI1_IRQn);
        first=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1);

        if(first)
        {
        	toSend = 50;
            xQueueSend( xQueue, &toSend, 0 );
        }
        vTaskDelay(antiBounce_MS);
        NVIC_EnableIRQ(EXTI1_IRQn);

    }
}

/*-----------------------------------------------------------*/

static void prvSemTask3( void *pvParameters )
{
    uint32_t toSend;

    xSemaphoreTake( xESem3, 0 );
    int first;
    for( ;; )
    {
        xSemaphoreTake( xESem3, portMAX_DELAY );
        NVIC_DisableIRQ(EXTI2_IRQn);
        first=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2);
        if(first)
        {
        	toSend = 51;
            xQueueSend( xQueue, &toSend, 0 );
        }
        vTaskDelay(antiBounce_MS);
        NVIC_EnableIRQ(EXTI2_IRQn);

    }
}

/*-----------------------------------------------------------*/

static void prvSemTask4( void *pvParameters )
{
    uint32_t toSend;

    xSemaphoreTake( xESem4, 0 );

    for( ;; )
    {
        xSemaphoreTake( xESem4, portMAX_DELAY );
        int first;
        NVIC_DisableIRQ(EXTI3_IRQn);
        first=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3);
        if(first)
        {
        	toSend = 52;
            xQueueSend( xQueue, &toSend, 0 );
        }
        vTaskDelay(antiBounce_MS);
        NVIC_EnableIRQ(EXTI3_IRQn);

    }
}

/*-----------------------------------------------------------*/
void EXTI0_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		xSemaphoreGiveFromISR( xESem1, &xHigherPriorityTaskWoken );
	}
}

/*-----------------------------------------------------------*/
void EXTI1_IRQHandler(void)
{

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);
		xSemaphoreGiveFromISR( xESem2, &xHigherPriorityTaskWoken );
	}
}

/*-----------------------------------------------------------*/
void EXTI2_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line2);
		xSemaphoreGiveFromISR( xESem3, &xHigherPriorityTaskWoken );
	}
}
/*-----------------------------------------------------------*/
void EXTI3_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);
		xSemaphoreGiveFromISR( xESem4, &xHigherPriorityTaskWoken );
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    NVIC_SetPriorityGrouping( 0 );

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	//----------ISRs - PinD
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //----------LEDS - PinB
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //----------EXTI Line0
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //----------EXTI Line1
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //----------EXTI Line2
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);

    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //----------EXTI Line3
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource3);

    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	//----------USART
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	// Initialize pins as alternating function

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStruct);
	USART_Cmd(USART1, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

}

void vApplicationMallocFailedHook( void )
{
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

    xFreeStackSpace = xPortGetFreeHeapSize();

    if( xFreeStackSpace > 100 )
    {

    }
}

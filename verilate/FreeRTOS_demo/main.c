
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
//void vApplicationTickHook(void);

static void Task1_Handler(void *pvParameters);
static void Task2_Handler(void *pvParameters);

//static SemaphoreHandle_t xSemaphore = NULL;

int main( void )
{
	int res;
	printf("FreeRTOS on RISC-V\r\n");

	// xSemaphore = xSemaphoreCreateBinary(); //xQueueGenericCreate

	// if(xSemaphore != NULL)
	// {
	// 	res = xTaskCreate(Task1_Handler, "Task1", 256, NULL, 2, NULL);
	// 	if(res != pdPASS) printf("task1 res=%d\r\n",res); //1=succ
	// 	res = xTaskCreate(Task2_Handler, "Task2", 256, NULL, 2, NULL);
	// 	if(res != pdPASS) printf("task2 res=%d\r\n",res); //1=succ
		
	// 	xSemaphoreGive(xSemaphore);
	// 	vTaskStartScheduler();
	// }else{
	// 	printf("semaphore fail!\r\n");
	// }

	//printf("hi");
	res = xTaskCreate(Task1_Handler, "Task1", 256, NULL, 2, NULL);
	//printf("lo");
	if(res != pdPASS) printf("task1 res=%d\r\n",res); //1=succ
	res = xTaskCreate(Task2_Handler, "Task2", 256, NULL, 2, NULL);
	if(res != pdPASS) printf("task2 res=%d\r\n",res); //1=succ

	vTaskStartScheduler();
}

static void Task1_Handler(void *pvParameters)
{
	char msg[64] = "Hello, Task 1\r\n";

	while(1)
	{
		printf(msg);
		vTaskDelay(5);
	}
	// while(1)
	// {
	// 	if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
	// 	{
	// 		printf("%s",msg);
	// 		xSemaphoreGive(xSemaphore);
	// 	}
	// 	vTaskDelay(2);
	// }
}

static void Task2_Handler(void *pvParameters)
{
	char msg[64] = "Hello, Task 2\r\n";

	//int i = 0;
	while(1)
	{
		// if(i==1){
		// 	vTaskSuspend(Task1_Handler);
		// }
		// if(i==4){
		// 	vTaskResume(Task1_Handler);
		// }
		// i++;
		printf(msg);
		vTaskDelay(8);
	}

	
	// while(1)
	// {
	// 	if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
	// 	{
	// 		printf("%s",msg);
	// 		xSemaphoreGive(xSemaphore);
	// 		// if(i==1){
	// 		// 	vTaskSuspend(Task1_Handler);
	// 		// }
	// 		// if(i==4){
	// 		// 	vTaskResume(Task1_Handler);
	// 		// }
	// 		// i++;
	// 	}
	// 	vTaskDelay(3);
	// }
}

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

void vApplicationIdleHook( void )
{
	//printf("idle task.");
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*
void vApplicationTickHook( void )
{
	// The tests in the full demo expect some interaction with interrupts.
	#if( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
	{
		extern void vFullDemoTickHook( void );
		vFullDemoTickHook();
	}
	#endif
}
*/

void vAssertCalled( void )
{
	taskDISABLE_INTERRUPTS();
	printf("assert\r\n");
	while(1)
	{
		__asm volatile( "NOP" );
	}
}

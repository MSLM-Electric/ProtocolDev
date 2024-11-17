/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* FreeRTOS.org includes. */
#include "../../Win32-simulator-MSVC/FreeRTOSv10_Source/include/FreeRTOS.h"
#include "../../Win32-simulator-MSVC/FreeRTOSv10_Source/include/task.h"
#include "../../Win32-simulator-MSVC/FreeRTOSv10_Source/include/timers.h"

/* Demo includes. */
#include "../../Win32-simulator-MSVC/Supporting_Functions/supporting_functions.h"

#include "../../ExternalLibs/cmsis_os/cmsis_os.h"
#include "../../ExternalLibs/BitLogger/BitLogger.h"
//#include "../../ExternalLibs/SimpleTimer/SimpleTimerWP.h"
#include "SpecLibs/SimpleTimerWP.h"
#include "../../ExternalLibs/type_def.h"
#include <stdio.h>

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xffffff )

/* The number of the simulated interrupt used in this example.  Numbers 0 to 2
are used by the FreeRTOS Windows port itself, so 3 is the first number available
to the application. */
#define mainINTERRUPT_NUMBER	3
#define timerINTERRUPT_NUMBER   mainINTERRUPT_NUMBER+1

#define ONE_SHOT_TIMER 0
#define PERIODIC_TIMER 1

/* The task functions. */
void vMasterCoreImmit( void *pvParameters );
void vSlaveCoreImmit(void* pvParameters);
void TimerInterruptionSimulate(void* pvParameters);
void xPortSysTickHandler(void);
#ifndef CMSIS_OS_ENABLE
static U32_ms osKernelSysTick(void);
#endif // CMSIS_OS_ENABLE


static /*or extern*/ BitLoggerList_t BugsBitList;
//static Timerwp_t BugScannerTimer;
//static Timerwp_t BugBitReportTimer;
static u16 BitPos(u16 Bit);

/* The service routine for the (simulated) interrupt.  This is the interrupt
that the task will be synchronized with. */
static uint32_t ulExampleInterruptHandler(void);
static uint32_t ulTimerInterruptHandler(void);

static uint32_t ulMasterRXinterfaceUnitInterrupt(void);
static uint32_t ulMasterTXinterfaceUnitInterrupt(void);
static uint32_t ulSlaveRXinterfaceUnitInterrupt(void);
static uint32_t ulSlaveTXinterfaceUnitInterrupt(void);
static uint32_t ulTimerOfMasterInterrupt(void);
static uint32_t ulTimerOfSlaveInterrupt(void);

static uint32_t waitRXing/*FromMasterBySlave*/(void);
static uint32_t waitTXingToSlaveByMaster  (void);
static uint32_t waitRXingFromSlaveByMaster(void);
static uint32_t waitTXingToMasterBySlave(void);

#include "SpecLibs/HardwareInterfaceUnit/HardwareInterfaceUnit.h"
#include "SpecLibs/HardwareInterfaceUnit/port.h"

osPoolDef(mpool, 8, portsBuffer_t);
extern osPoolId mpool;
osMessageQDef(MsgBox, 8, portsBuffer_t);
extern osMessageQId MsgBox;
TaskHandle_t xMasterTXsignal;
TaskHandle_t xSlaveTXsignal;

/*---------------SOME CONFIGS FOR DEMONSTRATION--------------*/
#include "debug_print.h"
#ifdef ENABLE_PRINTING_BACKGROUND_PROCESS
#define PRINT_BACKGROUND_PROCESS(e, s) DEBUG_PRINTM(e, s)
#else
#define PRINT_BACKGROUND_PROCESS(e, s)
#endif // ENABLE_PRINTING_BACKGROUND_PROCESS

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

int main( int argc, char **argv  )
{
	/* Create one of the two tasks. */
	/*Master's Background application runner*/
	xTaskCreate(vMasterCoreImmit,		/* Pointer to the function that implements the task. */
					"Master Unit",	/* Text name for the task.  This is to facilitate debugging only. */
					2000,		/* Stack depth - most small microcontrollers will use much less stack than this. */
					NULL,		/* We are not using the task parameter. */
					1,			/* This task will run at priority 1. */
					NULL );		/* We are not using the task handle. */
	/*Slave's Background application runner*/
	xTaskCreate(vSlaveCoreImmit, "Slave Unit", 2000, NULL, 1, NULL);
	
	/*
	waitRXingFromMasterBySlave, waits by    osMailGet() -> event -> generates the SlaveRXinterruption() and sets the osSignalSet() to MasterTX
	waitTXingToSlaveByMaster,   waits by osSignalWait() -> event -> generates the MasterTXinterruption()
	waitRXingFromSlaveByMaster, waits by    osMailGet() -> event -> generates the MasterRXinterruption() and sets the osSignalSet() to SlaveTX
	waitTXingToMasterBySlave, waits by osSignalWait() -> event -> generates the SlaveTXinterruption()
	*/
	xTaskCreate(waitRXing/*FromMasterBySlave*/, "", 1000, NULL, 1, NULL);
	xTaskCreate(waitTXingToSlaveByMaster,   "", 500, NULL, 1, &xMasterTXsignal);
	//xTaskCreate(waitRXingFromSlaveByMaster, "", 500, NULL, 1, NULL);
	xTaskCreate(waitTXingToMasterBySlave, "", 500, NULL, 1, &xSlaveTXsignal);

	xTaskCreate(TimerInterruptionSimulate, "Timer Interrupt", 100, NULL, 1, NULL);
	init_simulatePROCESSOR_MODES(); //!for using cmsis_os funcs
	/* Install the handler for the software interrupt.  The syntax necessary
		to do this is dependent on the FreeRTOS port being used.  The syntax
		shown here can only be used with the FreeRTOS Windows port, where such
		interrupts are only simulated. */
	vPortSetInterruptHandler(mainINTERRUPT_NUMBER, ulExampleInterruptHandler);
	vPortSetInterruptHandler(timerINTERRUPT_NUMBER, ulTimerInterruptHandler);
	//vPortSetInterruptHandler(mainINTERRUPT_NUMBER, ulMasterRXinterfaceUnitInterrupt);
	//vPortSetInterruptHandler(mainINTERRUPT_NUMBER, ulMasterTXinterfaceUnitInterrupt);
	//vPortSetInterruptHandler(mainINTERRUPT_NUMBER, ulSlaveRXinterfaceUnitInterrupt);
	//vPortSetInterruptHandler(mainINTERRUPT_NUMBER, ulSlaveTXinterfaceUnitInterrupt);
	//vPortSetInterruptHandler(timerINTERRUPT_NUMBER, ulTimerOfMasterInterrupt);
	//vPortSetInterruptHandler(timerINTERRUPT_NUMBER, ulTimerOfSlaveInterrupt);

	mpool = osPoolCreate(osPool(mpool));
	MsgBox = osMessageCreate(osMessageQ(MsgBox), NULL);
	osSignalClear(xMasterTXsignal, 0x0001);
	osSignalClear(xSlaveTXsignal, 0x0002);

	/* Start the scheduler to start the tasks executing. */
	vTaskStartScheduler();	

	/* The following line should never be reached because vTaskStartScheduler() 
	will only return if there was not enough FreeRTOS heap memory available to
	create the Idle and (if configured) Timer tasks.  Heap management, and
	techniques for trapping heap exhaustion, are described in the book text. */
	for( ;; );
	return 0;
}
/*-----------------------------------------------------------*/

InterfacePortHandle_t MasterPort;
InterfacePortHandle_t SlavePort;

// Interrupt on receive
SLAVE_ISR(USART_RXC_vect)
{
	ReceiveInterrupt(&SlavePort);
}

// Interrupt on Transmit
SLAVE_ISR(USART_TXC_vect)
{
	TransmitInterrupt(&SlavePort);
}

// Interrupt on receive
MASTER_ISR(USART_RXC_vect)
{
	ReceiveInterrupt(&MasterPort);
}

// Interrupt on Transmit
MASTER_ISR(USART_TXC_vect)
{
	TransmitInterrupt(&MasterPort);
}
  
void vMasterCoreImmit( void *pvParameters )
{
	const char *pcTaskName = "Master unit running!\r\n";
	volatile uint32_t ul;
	static char buffer[200];
	u32 testVar = 0;
	const char msg[] = "Assalamu aleykum RAFIQ - \0\0\0\0\0\0\0\0\0\0\n";
	u8 putNumPos = 25;

	InitMasterPort(&MasterPort);
	MasterPort.communicationPeriod = 1000;
	MasterPort.Status setBITS(PORT_READY);
	//MasterPort.ReceivingTimer.setVal = (U32_ms)100;
	Timert_t CommunPeriod;
	InitTimerWP(&CommunPeriod, NULL);
	LaunchTimerWP(MasterPort.communicationPeriod, &CommunPeriod);
	//sprintf(MasterPort.BufferToSend, "Assalamu aleykum RAFIQ!\n");
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		PRINT_BACKGROUND_PROCESS(0, pcTaskName );

		/* Delay for a period. */
		vTaskDelay(10);
		if (IsTimerWPRinging(&CommunPeriod)) {
			RestartTimerWP(&CommunPeriod);
			sprintf(&msg[putNumPos], "%d\n", testVar);
			if (Write(&MasterPort, msg, sizeof(/*buffer*/msg)) < 0)
				vPrintString("Master Write failed\n");
			testVar++;
		}
		SendingTimerHandle(&MasterPort);
		if ((MasterPort.Status & (PORT_BUSY | PORT_SENDED_ALL)) == ONLY PORT_SENDED_ALL) {
			MasterPort.Status clearBITS(PORT_SENDED_ALL);
			memset(MasterPort.BufferRecved, 0, sizeof(MasterPort.BufferRecved));
			Recv(&MasterPort, buffer, sizeof(buffer));
		}
		ReceivingTimerHandle(&MasterPort);
		if (MasterPort.Status & PORT_RECEIVED_ALL) {
			DEBUG_PRINTM(1, MasterPort.BufferRecved);
			MasterPort.Status clearBITS(PORT_RECEIVED_ALL);
		}
	}
}
/*-----------------------------------------------------------*/

void vSlaveCoreImmit(void* pvParameters)
{
	const char* pcTaskName = "Slave unit running!\r\n";
	volatile uint32_t ul;
	static char buffer[200];

	InitSlavePort(&SlavePort);
	SlavePort.Status setBITS(PORT_READY);
	/* As per most tasks, this task is implemented in an infinite loop. */
	for (;; )
	{
		/* Print out the name of this task. */
		PRINT_BACKGROUND_PROCESS(0, pcTaskName);
		
		/* Delay for a period. */
		vTaskDelay(10);
		if (NOT (SlavePort.Status & PORT_BUSY)) {
			memset(SlavePort.BufferRecved, 0, sizeof(SlavePort.BufferRecved));
			Recv(&SlavePort, buffer, sizeof(buffer));
		}
		ReceivingTimerHandle(&SlavePort);
		if (SlavePort.Status & PORT_RECEIVED_ALL) {
			DEBUG_PRINTM(1, SlavePort.BufferRecved);
			Write(&SlavePort, "Slave've got your msg!\n", 24);
		}
	}
}

static uint32_t waitRXing/*FromMasterBySlave*/(void)
{
	//Init
	portsBuffer_t* ifsPort;
	osEvent event;

	for (;;) {
		event = osMessageGet(MsgBox, osWaitForever);
		if (event.status == osEventMessage) {
			ifsPort = event.value.p;
#if defined(_M_X64)
			if ((uint64_t)event.value.p & (uint64_t)MsgBox & 0xFFFFFFFF00000000) {
#endif
				if (ifsPort->Port == &MasterPort) {
					//SlaveRXinterruption()
					osPoolFree(mpool, ifsPort);
					HWPortN[SLAVENO].BUFFER = ifsPort->BUFFER;
					//ReceiveInterrupt(&SlavePort);
					SLAVE_CallISR(USART_RXC_vect);
					/*I'v got from master then Master transmitted:*/
					osSignalSet(/*MasterTXinterrupt()*/xMasterTXsignal, 0x0001);
				}else if (ifsPort->Port == &SlavePort) {
					//MasterRXinterruption()
					osPoolFree(mpool, ifsPort);
					HWPortN[MASTERNO].BUFFER = ifsPort->BUFFER;
					//ReceiveInterrupt(&MasterPort);
					MASTER_CallISR(USART_RXC_vect);
					/*I've got from slave then Slave transmitted*/
					osSignalSet(/*SlaveTXinterrupt()*/xSlaveTXsignal, 0x0002);
				}
#if defined(_Mx64)
			}
#endif
		}
	}
}

static uint32_t waitTXingToSlaveByMaster(void)
{
	//portsBuffer_t* ifsPort;
	osEvent event;
	for (;;) {
		event = osSignalWait(0x0001, osWaitForever);
		if (event.status == osEventSignal) {
			//MasterTXinterruption()
			//TransmitInterrupt(&MasterPort);
			MASTER_CallISR(USART_TXC_vect);
			//osDelay(2);
		}
		osSignalClear(xMasterTXsignal, 0x0001);
	}
}

static uint32_t waitTXingToMasterBySlave(void)
{
	//portsBuffer_t* ifsPort;
	osEvent event;
	for (;;) {
		event = osSignalWait(0x0002, osWaitForever);
		if (event.status == osEventSignal) {
			//SlaveTXinterruption()
			//TransmitInterrupt(&SlavePort);
			SLAVE_CallISR(USART_TXC_vect);
			//osDelay(2);
		}
		osSignalClear(xSlaveTXsignal, 0x0002);
	}
}

/*Not used and not created task!*/
static uint32_t waitRXingFromSlaveByMaster(void)
{
	//Init
	portsBuffer_t* ifsPort;
	osEvent event;

	for (;;) {
		event = osMessageGet(MsgBox, osWaitForever);
		if (event.status == osEventMessage) {
			ifsPort = event.value.p;
			if ((uint64_t)event.value.p & (uint64_t)MsgBox & 0xFFFFFFFF00000000) {
				if (ifsPort->Port == &SlavePort) {
					//MasterRXinterruption()
					osPoolFree(mpool, ifsPort);
					HWPortN[MASTERNO].BUFFER = ifsPort->BUFFER;
					ReceiveInterrupt(&MasterPort);
					//osDelay(10);
					/*I've got from slave then Slave transmitted*/
					osSignalSet(/*SlaveTXinterrupt()*/xSlaveTXsignal, 0x0002);
				}
			}
		}
	}
}

/*-----------------------------------------------------------*/

static uint32_t ulExampleInterruptHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;

	/* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
	it will get set to pdTRUE inside the interrupt safe API function if a
	context switch is required. */
	xHigherPriorityTaskWoken = pdFALSE;

	/* 'Give' the semaphore to unblock the task. */
	//xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);

	/* Pass the xHigherPriorityTaskWoken value into portYIELD_FROM_ISR().  If
	xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
	then calling portYIELD_FROM_ISR() will request a context switch.  If
	xHigherPriorityTaskWoken is still pdFALSE then calling
	portYIELD_FROM_ISR() will have no effect.  The implementation of
	portYIELD_FROM_ISR() used by the Windows port includes a return statement,
	which is why this function does not explicitly return a value. */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static uint32_t ulTimerInterruptHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	/*user code*/
	simulatePROCESSOR_HANDLER_MODE();
	someExternalTick++;
	simulatePROCESSOR_THREAD_MODE();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void TimerInterruptionSimulate(void* pvParameters)
{
	for (;;) {
		vTaskDelay(1);
		vPortGenerateSimulatedInterrupt(mainINTERRUPT_NUMBER);
		vPortGenerateSimulatedInterrupt(timerINTERRUPT_NUMBER);
	}
}

static uint32_t ulMasterRXinterfaceUnitInterrupt(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	/*user code*/
	simulatePROCESSOR_HANDLER_MODE();

	simulatePROCESSOR_THREAD_MODE();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static uint32_t ulMasterTXinterfaceUnitInterrupt(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	/*user code*/
	simulatePROCESSOR_HANDLER_MODE();

	simulatePROCESSOR_THREAD_MODE();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static uint32_t ulSlaveRXinterfaceUnitInterrupt(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	/*user code*/
	simulatePROCESSOR_HANDLER_MODE();

	simulatePROCESSOR_THREAD_MODE();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static uint32_t ulSlaveTXinterfaceUnitInterrupt(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	/*user code*/
	simulatePROCESSOR_HANDLER_MODE();

	simulatePROCESSOR_THREAD_MODE();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static uint32_t ulTimerOfMasterInterrupt(void) 
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	/*user code*/
	simulatePROCESSOR_HANDLER_MODE();

	simulatePROCESSOR_THREAD_MODE();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static uint32_t ulTimerOfSlaveInterrupt(void) 
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	/*user code*/
	simulatePROCESSOR_HANDLER_MODE();

	simulatePROCESSOR_THREAD_MODE();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

extern void vApplicationIdleHook(void)
{
	vPrintString("Idle task\n");
	return;
}

extern void vApplicationDaemonTaskStartupHook(void)
{
	return;
}

#ifndef CMSIS_OS_ENABLE
static U32_ms osKernelSysTick(void)
{
#ifdef DEBUG_ON_VS
	return (U32_ms)GetTickCount();
#endif
}
#endif

void xPortSysTickHandler(void)
{
#ifdef DEBUG_ON_VS
	return (U32_ms)GetTickCount();
#endif // DEBUG_ON_VS
}

static u16 BitPos(u16 Bit)
{
	u16 res = 0;
	while ((Bit >> res) > 1) {
		res++;
	}
	return res;
}
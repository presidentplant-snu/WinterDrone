/*
 * FreeRTOS Kernel V10.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Copyright (C) 2014 - 2020 Xilinx, Inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
	extern "C" {
#endif

/* BSP includes. */
#include "xil_types.h"
#include "xpseudo_asm.h"
#include "FreeRTOSConfig.h"

#if defined (versal) && !defined(ARMR5)
#define GICv3
#else
#define GICv2
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the given hardware
 * and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	size_t
#define portBASE_TYPE	long

typedef portSTACK_TYPE StackType_t;
typedef portBASE_TYPE BaseType_t;
typedef uint64_t UBaseType_t;

typedef uint64_t TickType_t;

#define portMAX_DELAY ( ( TickType_t ) 0xffffffffffffffff )


/*-----------------------------------------------------------*/

/* Hardware specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT			16
#define portPOINTER_SIZE_TYPE 		uint64_t
#define portCRITICAL_NESTING_IN_TCB 1

/* Task utilities. */

extern uint64_t ullPortYieldRequired[];
/* Called at the end of an ISR that can cause a context switch. */
#define portEND_SWITCHING_ISR( xSwitchRequired )\
{												\
												\
	if( xSwitchRequired != pdFALSE )			\
	{											\
		ullPortYieldRequired[portGET_CORE_ID()] = pdTRUE;			\
	}											\
}

#define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )
#if defined( GUEST )
	#define portYIELD() __asm volatile ( "SVC 0" ::: "memory" )
#else
	#define portYIELD() __asm volatile ( "SMC 0" ::: "memory" )
#endif


static inline UBaseType_t uxDisableInterrupts()
{
    unsigned long flags;

    __asm volatile (
        "mrs %0, daif\n"
        "msr daifset, #2\n"
        : "=r" (flags)
        :
        : "memory"
    );

    return flags;
}

static inline void vEnableInterrupts()
{
    __asm volatile (
        "msr daifclr, #2\n"
        :
        :
        : "memory"
    );
}

static inline void vRestoreInterrupts(UBaseType_t flags)
{
    __asm volatile (
        "and x2, %0, #128\n"
        "mrs x1, daif\n"
        "bic x1, x1, #128\n"
        "orr x1, x1, x2\n"
        "msr daif, x1\n"
        :
        : "r" (flags)
        : "x0","x1","x2","memory"
    );
}


static inline BaseType_t xPortGetCoreID()
{
   register BaseType_t xCoreID;

   __asm volatile (
       "mrs  x0, mpidr_el1\n"
       "and  %0, x0, #0xff\n"
       : "=r" (xCoreID)
       :
       : "memory", "x0"
   );

   return xCoreID;
}


/*-----------------------------------------------------------
 * Critical section control
 *----------------------------------------------------------*/
extern void vTaskEnterCritical( void );
extern void vTaskExitCritical( void );
extern UBaseType_t vTaskEnterCriticalFromISR( void );
extern void vTaskExitCriticalFromISR( UBaseType_t uxSavedInterruptStatus );

#define portENTER_CRITICAL()		            vTaskEnterCritical();
#define portEXIT_CRITICAL()			            vTaskExitCritical();
#define portENTER_CRITICAL_FROM_ISR()           vTaskEnterCriticalFromISR()
#define portEXIT_CRITICAL_FROM_ISR( x )         vTaskExitCriticalFromISR( x )

#define portDISABLE_INTERRUPTS()                uxDisableInterrupts()
#define portENABLE_INTERRUPTS()		            vEnableInterrupts()
#define portSET_INTERRUPT_MASK()                uxDisableInterrupts()
#define portCLEAR_INTERRUPT_MASK(x)             vRestoreInterrupts(x)

#define portHAS_NESTED_INTERRUPTS               1
UBaseType_t uxPortSetInterruptMask( void );
void vPortClearInterruptMask( UBaseType_t );
#define portSET_INTERRUPT_MASK_FROM_ISR()       uxPortSetInterruptMask()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)    vPortClearInterruptMask(x)


/* Task function macros as described on the FreeRTOS.org WEB site.  These are
not required for this port but included in case common demo code that uses these
macros is used. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters )	void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters )	void vFunction( void *pvParameters )

/* Prototype of the FreeRTOS tick handler.  This must be installed as the
handler for whichever peripheral is used to generate the RTOS tick. */
void FreeRTOS_Tick_Handler( void );

/*
 * Installs pxHandler as the interrupt handler for the peripheral specified by
 * the ucInterruptID parameter.
 *
 * ucInterruptID:
 *
 * The ID of the peripheral that will have pxHandler assigned as its interrupt
 * handler.  Peripheral IDs are defined in the xparameters.h header file, which
 * is itself part of the BSP project.
 *
 * pxHandler:
 *
 * A pointer to the interrupt handler function itself.  This must be a void
 * function that takes a (void *) parameter.
 *
 * pvCallBackRef:
 *
 * The parameter passed into the handler function.  In many cases this will not
 * be used and can be NULL.  Some times it is used to pass in a reference to
 * the peripheral instance variable, so it can be accessed from inside the
 * handler function.
 *
 * pdPASS is returned if the function executes successfully.  Any other value
 * being returned indicates that the function did not execute correctly.
 */
BaseType_t xPortInstallInterruptHandler( uint8_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef );

/*
 * Enables the interrupt, within the interrupt controller, for the peripheral
 * specified by the ucInterruptID parameter.
 *
 * ucInterruptID:
 *
 * The ID of the peripheral that will have its interrupt enabled in the
 * interrupt controller.  Peripheral IDs are defined in the xparameters.h header
 * file, which is itself part of the BSP project.
 */
void vPortEnableInterrupt( uint8_t ucInterruptID );

/*
 * Disables the interrupt, within the interrupt controller, for the peripheral
 * specified by the ucInterruptID parameter.
 *
 * ucInterruptID:
 *
 * The ID of the peripheral that will have its interrupt disabled in the
 * interrupt controller.  Peripheral IDs are defined in the xparameters.h header
 * file, which is itself part of the BSP project.
 */
void vPortDisableInterrupt( uint8_t ucInterruptID );

/* Any task that uses the floating point unit MUST call vPortTaskUsesFPU()
before any floating point instructions are executed. */
#if( configUSE_TASK_FPU_SUPPORT != 2 )
void vPortTaskUsesFPU( void );
#else
	/* Each task has an FPU context already, so define this function away to
	 * 	nothing to prevent it being called accidentally. */
	#define vPortTaskUsesFPU()
#endif
#define portTASK_USES_FLOATING_POINT() vPortTaskUsesFPU()

#define portLOWEST_INTERRUPT_PRIORITY ( ( ( uint32_t ) configUNIQUE_INTERRUPT_PRIORITIES ) - 1UL )
#define portLOWEST_USABLE_INTERRUPT_PRIORITY ( portLOWEST_INTERRUPT_PRIORITY - 1UL )

/* Architecture specific optimisations. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
	#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

	/* Store/clear the ready priorities in a bit map. */
	#define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
	#define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

	/*-----------------------------------------------------------*/

	#define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) uxTopPriority = ( 31 - __builtin_clz( uxReadyPriorities ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

#ifdef configASSERT
	void vPortValidateInterruptPriority( void );
	#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID() 	vPortValidateInterruptPriority()
#endif /* configASSERT */

#define portNOP()               __asm volatile( "NOP" )
#define portMEMORY_BARRIER()    __asm volatile( "" ::: "memory" )

/* port for SMP */
#define portGET_CORE_ID()       xPortGetCoreID()

static inline void vAssertIfInIsr()
{
    extern volatile uint64_t uxCoreInIsr[configNUMBER_OF_CORES];
    UBaseType_t x = uxDisableInterrupts();
    BaseType_t xCoreId = xPortGetCoreID();
    configASSERT(uxCoreInIsr[xCoreId] == 0);
    vRestoreInterrupts(x);
}
#define portASSERT_IF_IN_ISR()  vAssertIfInIsr()

/*-----------------------------------------------------------
 * Critical section locks
 *----------------------------------------------------------*/
#define ISR_LOCK                (0u)
#define TASK_LOCK               (1u)

extern void vPortRecursiveLock(BaseType_t xCoreID, uint32_t ulLockNum, BaseType_t uxAcquire);


#define portRELEASE_ISR_LOCK( xCoreID )     vPortRecursiveLock(( xCoreID ), ISR_LOCK, pdFALSE)
#define portGET_ISR_LOCK( xCoreID )         vPortRecursiveLock(( xCoreID ), ISR_LOCK, pdTRUE)

#define portRELEASE_TASK_LOCK( xCoreID )    vPortRecursiveLock(( xCoreID ), TASK_LOCK, pdFALSE)
#define portGET_TASK_LOCK( xCoreID )        vPortRecursiveLock(( xCoreID ), TASK_LOCK, pdTRUE)

extern void vInterruptCore(uint32_t ulInterruptID, uint32_t ulCoreID);
/* Use PPI 0 as the yield core interrupt. */
#define portYIELD_CORE_INT_ID       0
#define portYIELD_CORE( xCoreID )   vInterruptCore(portYIELD_CORE_INT_ID, (uint32_t)xCoreID)


/* The number of bits to shift for an interrupt priority is dependent on the
number of bits implemented by the interrupt controller. */

#if configUNIQUE_INTERRUPT_PRIORITIES == 16
	#define portPRIORITY_SHIFT 4
	#define portMAX_BINARY_POINT_VALUE	3
#elif configUNIQUE_INTERRUPT_PRIORITIES == 32
	#define portPRIORITY_SHIFT 3
	#define portMAX_BINARY_POINT_VALUE	2
#elif configUNIQUE_INTERRUPT_PRIORITIES == 64
	#define portPRIORITY_SHIFT 2
	#define portMAX_BINARY_POINT_VALUE	1
#elif configUNIQUE_INTERRUPT_PRIORITIES == 128
	#define portPRIORITY_SHIFT 1
	#define portMAX_BINARY_POINT_VALUE	0
#elif configUNIQUE_INTERRUPT_PRIORITIES == 256
	#define portPRIORITY_SHIFT 0
	#define portMAX_BINARY_POINT_VALUE	0
#else
	#error Invalid configUNIQUE_INTERRUPT_PRIORITIES setting.  configUNIQUE_INTERRUPT_PRIORITIES must be set to the number of unique priorities implemented by the target hardware
#endif


#if defined(GICv2)
/* Interrupt controller access addresses. */
#define portICCPMR_PRIORITY_MASK_OFFSET                     ( 0x04 )
#define portICCIAR_INTERRUPT_ACKNOWLEDGE_OFFSET             ( 0x0C )
#define portICCEOIR_END_OF_INTERRUPT_OFFSET                 ( 0x10 )
#define portICCBPR_BINARY_POINT_OFFSET                      ( 0x08 )
#define portICCRPR_RUNNING_PRIORITY_OFFSET                  ( 0x14 )
#define portICD_SGIR_OFFSET                                 ( 0xF00 )

#define portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS 		( configINTERRUPT_CONTROLLER_BASE_ADDRESS + configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET )
#define portICCPMR_PRIORITY_MASK_REGISTER 					( *( ( volatile uint32_t * ) ( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCPMR_PRIORITY_MASK_OFFSET ) ) )
#define portICCIAR_INTERRUPT_ACKNOWLEDGE_REGISTER_ADDRESS 	( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCIAR_INTERRUPT_ACKNOWLEDGE_OFFSET )
#define portICCEOIR_END_OF_INTERRUPT_REGISTER_ADDRESS 		( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCEOIR_END_OF_INTERRUPT_OFFSET )
#define portICCPMR_PRIORITY_MASK_REGISTER_ADDRESS 			( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCPMR_PRIORITY_MASK_OFFSET )
#define portICCBPR_BINARY_POINT_REGISTER 					( *( ( const volatile uint32_t * ) ( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCBPR_BINARY_POINT_OFFSET ) ) )
#define portICCRPR_RUNNING_PRIORITY_REGISTER 				( *( ( const volatile uint32_t * ) ( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCRPR_RUNNING_PRIORITY_OFFSET ) ) )
#define portICD_SGIR_ADDRESS                                ( configINTERRUPT_CONTROLLER_BASE_ADDRESS + portICD_SGIR_OFFSET )
#endif
#endif /* PORTMACRO_H */

#ifdef __cplusplus
	} /* extern C */
#endif

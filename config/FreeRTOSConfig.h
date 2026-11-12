#pragma once

// Configuration FreeRTOS pour Cortex-M4F.
// Allocation statique uniquement : pas de tas, la mémoire de chaque tâche est connue à l'édition de liens.

#include <stdint.h>

extern uint32_t SystemCoreClock;

// Ordonnanceur
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      1000
#define configMAX_PRIORITIES                    8
#define configMINIMAL_STACK_SIZE                128
#define configMAX_TASK_NAME_LEN                 16
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TIME_SLICING                  1

// Mémoire
#define configSUPPORT_STATIC_ALLOCATION     1
#define configSUPPORT_DYNAMIC_ALLOCATION    0
#define configKERNEL_PROVIDED_STATIC_MEMORY 1

// Fonctionnalités
#define configUSE_MUTEXES             1
#define configUSE_RECURSIVE_MUTEXES   0
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_TASK_NOTIFICATIONS  1
#define configUSE_QUEUE_SETS          0
#define configQUEUE_REGISTRY_SIZE     0
#define configUSE_TIMERS              0
#define configTIMER_TASK_STACK_DEPTH  configMINIMAL_STACK_SIZE // exigé par tasks.c même sans timers
#define configUSE_CO_ROUTINES         0
#define configUSE_NEWLIB_REENTRANT    0

// Crochets et diagnostic
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0
#define configUSE_MALLOC_FAILED_HOOK    0
#define configCHECK_FOR_STACK_OVERFLOW  2
#define configUSE_TRACE_FACILITY        1
#define configGENERATE_RUN_TIME_STATS   0
#define configRECORD_STACK_HIGH_ADDRESS 1

// Priorités d'interruption : 4 bits sur STM32F4 (0 = la plus haute).
// Les interruptions de priorité 0 à 4 ne sont jamais masquées par le noyau et ne doivent pas appeler
// FreeRTOS.
#define configPRIO_BITS                              4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY                                                                 \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

// API incluses
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskDelayUntil             1
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTaskGetCurrentTaskHandle   1

void rtos_assert_failed(const char *file, int line);
#define configASSERT(x)                                                                                      \
    do {                                                                                                     \
        if (!(x))                                                                                            \
            rtos_assert_failed(__FILE__, __LINE__);                                                          \
    } while (0)

// Branche le port FreeRTOS sur les noms de vecteurs CMSIS.
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

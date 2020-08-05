/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
   threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
   byte pool, and block pool.  */

#include "tx_api.h"
#include "printf.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "os_hal_gpio.h"

#include "hw/azure_sphere_learning_path.h"

static const uintptr_t GPT_BASE = 0x21030000;
float distance_left;

#define DEMO_STACK_SIZE         1024
#define DEMO_BYTE_POOL_SIZE     9120
#define DEMO_BLOCK_POOL_SIZE    100
#define DEMO_QUEUE_SIZE         100


/* Define the ThreadX object control blocks...  */

TX_THREAD               thread_measure_distance;
TX_THREAD               thread_0;
TX_THREAD               thread_1;
TX_THREAD               thread_2;
TX_THREAD               thread_3;
TX_THREAD               thread_4;
TX_THREAD               thread_5;
TX_THREAD               thread_6;
TX_THREAD               thread_7;
TX_QUEUE                queue_0;
TX_SEMAPHORE            semaphore_0;
TX_MUTEX                mutex_0;
TX_EVENT_FLAGS_GROUP    event_flags_0;
TX_BYTE_POOL            byte_pool_0;
TX_BLOCK_POOL           block_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];


/* Define the counters used in the demo application...  */

ULONG                   thread_0_counter;
ULONG                   thread_1_counter;
ULONG                   thread_1_messages_sent;
ULONG                   thread_2_counter;
ULONG                   thread_2_messages_received;
ULONG                   thread_3_counter;
ULONG                   thread_4_counter;
ULONG                   thread_5_counter;
ULONG                   thread_6_counter;
ULONG                   thread_7_counter;


/* Define thread prototypes.  */

void    thread_measure_distance_entry(ULONG thread_input);


/* Define main entry point.  */

int main(void)
{

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}


/* Define what the initial system looks like.  */

void    tx_application_define(void *first_unused_memory)
{

CHAR    *pointer;


    /* Create a byte memory pool from which to allocate the thread stacks.  */
    tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);

    /* Put system definition stuff in here, e.g. thread creates and other assorted
       create information.  */

    /* Allocate the stack for thread 0.  */
    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);


    /* Allocate the stack for thread 1.  */
    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);

    //tx_thread_create(&thread_measure_distance, "thread measure distance", thread_measure_distance_entry, 0,
    //    pointer, DEMO_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

    /* Create threads 1 and 2. These threads pass information through a ThreadX 
       message queue.  It is also interesting to note that these threads have a time
       slice.  */
    tx_thread_create(&thread_1, "thread 1", thread_measure_distance_entry, 1,  
            pointer, DEMO_STACK_SIZE, 
            16, 16, 4, TX_AUTO_START);

    /* Allocate the stack for thread 2.  */
    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);



}

// https://embeddedartistry.com/blog/2017/02/17/implementing-malloc-with-threadx/
// overrides for malloc and free required for srand and rand
void* malloc(size_t size)
{
    void* ptr = NULL;

    if (size > 0)
    {
        // We simply wrap the threadX call into a standard form
        uint8_t r = tx_byte_allocate(&byte_pool_0, &ptr, size,
            TX_WAIT_FOREVER);

        if (r != TX_SUCCESS)
        {
            ptr = NULL;
        }
    }
    //else NULL if there was no size

    return ptr;
}

void free(void* ptr)
{
    if (ptr)
    {
        //We simply wrap the threadX call into a standard form
        //uint8_t r = tx_byte_release(ptr);
        tx_byte_release(ptr);
    }
}


void WriteReg32(uintptr_t baseAddr, size_t offset, uint32_t value)
{
    *(volatile uint32_t*)(baseAddr + offset) = value;
}

uint32_t ReadReg32(uintptr_t baseAddr, size_t offset)
{
    return *(volatile uint32_t*)(baseAddr + offset);
}

void Gpt3_WaitUs(int microseconds)
{
    // GPT3_INIT = initial counter value
    WriteReg32(GPT_BASE, 0x54, 0x0);

    // GPT3_CTRL
    uint32_t ctrlOn = 0x0;
    ctrlOn |= (0x19) << 16; // OSC_CNT_1US (default value)
    ctrlOn |= 0x1;          // GPT3_EN = 1 -> GPT3 enabled
    WriteReg32(GPT_BASE, 0x50, ctrlOn);

    // GPT3_CNT
    while (ReadReg32(GPT_BASE, 0x58) < microseconds)
    {
        // empty.
    }

    // GPT_CTRL -> disable timer
    WriteReg32(GPT_BASE, 0x50, 0x0);
}

bool readInput(u8 pin)
{
    os_hal_gpio_data value = 0;
    mtk_os_hal_gpio_get_input(pin, &value);
    return value == OS_HAL_GPIO_DATA_HIGH;
}

float get_distance(u8 pin, unsigned long timeoutMicroseconds)
{
    uint32_t pulseBegin, pulseEnd;

    mtk_os_hal_gpio_set_direction(pin, OS_HAL_GPIO_DIR_OUTPUT);	// set for output
    mtk_os_hal_gpio_set_output(pin, OS_HAL_GPIO_DATA_LOW);		// pull low
    Gpt3_WaitUs(2);

    mtk_os_hal_gpio_set_output(pin, OS_HAL_GPIO_DATA_HIGH);		// pull high
    Gpt3_WaitUs(5);

    // GPT3_CTRL - starts microsecond resolution clock
    uint32_t ctrlOn = 0x0;
    ctrlOn |= (0x19) << 16; // OSC_CNT_1US (default value)
    ctrlOn |= 0x1;          // GPT3_EN = 1 -> GPT3 enabled
    WriteReg32(GPT_BASE, 0x50, ctrlOn);

    mtk_os_hal_gpio_set_direction(pin, OS_HAL_GPIO_DIR_INPUT);	// set for input

    while (readInput(pin))		// wait for any previous pulse to end
    {
        if (ReadReg32(GPT_BASE, 0x58) > timeoutMicroseconds)
        {
            return NAN;
        }
    }

    while (!readInput(pin))		// wait for the pulse to start
    {
        pulseBegin = ReadReg32(GPT_BASE, 0x58);
        if (ReadReg32(GPT_BASE, 0x58) > timeoutMicroseconds)
        {
            return NAN;
        }
    }

    pulseBegin = ReadReg32(GPT_BASE, 0x58);

    while (readInput(pin))		// wait for the pulse to stop
    {
        if (ReadReg32(GPT_BASE, 0x58) > timeoutMicroseconds)
        {
            return NAN;
        }
    }

    pulseEnd = ReadReg32(GPT_BASE, 0x58);

    // GPT_CTRL -> disable timer
    WriteReg32(GPT_BASE, 0x50, 0x0);

    return (pulseEnd - pulseBegin) / 58.0; //  29 / 2;
}

void    thread_measure_distance_entry(ULONG thread_input)
{

UINT    status;

    /* This thread simply sends messages to a queue shared by thread 2.  */
    while(1)
    {

        distance_left = get_distance(HCSR04_LEFT, 100000);
    }
}
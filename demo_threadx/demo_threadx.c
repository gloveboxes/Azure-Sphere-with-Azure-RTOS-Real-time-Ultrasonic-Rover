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

bool HLAppReady = false;
float distance_left, distance_right, newDistanceLeft, newDistanceRight;
bool newDataReady = false;

enum LEDS
{
	RED,
	GREEN,
	BLUE
};

static enum LEDS current_led = RED;
static int leds[] = { LED_RED, LED_GREEN, LED_BLUE };


#define DEMO_STACK_SIZE         1024
#define DEMO_BYTE_POOL_SIZE     9120
#define DEMO_BLOCK_POOL_SIZE    100
#define DEMO_QUEUE_SIZE         100


/* Define the ThreadX object control blocks...  */

TX_THREAD               thread_measure_distance;
TX_BYTE_POOL            byte_pool_0;

UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];


/* Define thread prototypes.  */
void    thread_measure_distance_entry(ULONG thread_input);


int main(void)
{

	/* Enter the ThreadX kernel.  */
	tx_kernel_enter();
}


/* Define what the initial system looks like.  */

void	tx_application_define(void* first_unused_memory)
{
	CHAR* pointer;

	/* Create a byte memory pool from which to allocate the thread stacks.  */
	tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);

	/* Put system definition stuff in here, e.g. thread creates and other assorted
	   create information.  */

	   /* Allocate the stack for thread 1.  */
	tx_byte_allocate(&byte_pool_0, (VOID**)&pointer, DEMO_STACK_SIZE, TX_NO_WAIT);

	tx_thread_create(&thread_measure_distance, "thread 1", thread_measure_distance_entry, 0,
		pointer, DEMO_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
}


static	int gpio_output(u8 gpio_no, u8 level)
{
	int ret;

	ret = mtk_os_hal_gpio_request(gpio_no);
	if (ret != 0)
	{
		printf("request gpio[%d] fail\n", gpio_no);
		return ret;
	}
	mtk_os_hal_gpio_set_direction(gpio_no, OS_HAL_GPIO_DIR_OUTPUT);
	mtk_os_hal_gpio_set_output(gpio_no, level);
	ret = mtk_os_hal_gpio_free(gpio_no);
	if (ret != 0)
	{
		printf("free gpio[%d] fail\n", gpio_no);
		return ret;
	}
	return 0;
}

void	set_distance_indicator(int centimeters)
{
	static enum LEDS previous_led = RED;

	current_led = centimeters > 20 ? GREEN : centimeters > 10 ? BLUE : RED;

	if (previous_led != current_led)
	{
		gpio_output(leds[(int)previous_led], true); // turn off old current colour
		previous_led = current_led;
	}

	gpio_output(leds[(int)current_led], false);
}

// https://embeddedartistry.com/blog/2017/02/17/implementing-malloc-with-threadx/
// overrides for malloc and free required for srand and rand
void	*malloc(size_t size)
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

void	free(void* ptr)
{
	if (ptr)
	{
		//We simply wrap the threadX call into a standard form
		//uint8_t r = tx_byte_release(ptr);
		tx_byte_release(ptr);
	}
}

void	write_reg32(uintptr_t baseAddr, size_t offset, uint32_t value)
{
	*(volatile uint32_t*)(baseAddr + offset) = value;
}

uint32_t	read_reg32(uintptr_t baseAddr, size_t offset)
{
	return *(volatile uint32_t*)(baseAddr + offset);
}

void	gpt3_wait_microseconds(int microseconds)
{
	// GPT3_INIT = initial counter value
	write_reg32(GPT_BASE, 0x54, 0x0);

	// GPT3_CTRL
	uint32_t ctrlOn = 0x0;
	ctrlOn |= (0x19) << 16; // OSC_CNT_1US (default value)
	ctrlOn |= 0x1;          // GPT3_EN = 1 -> GPT3 enabled
	write_reg32(GPT_BASE, 0x50, ctrlOn);

	// GPT3_CNT
	while (read_reg32(GPT_BASE, 0x58) < microseconds)
	{
		// empty.
	}

	// GPT_CTRL -> disable timer
	write_reg32(GPT_BASE, 0x50, 0x0);
}

bool	read_input(u8 pin)
{
	os_hal_gpio_data value = 0;
	mtk_os_hal_gpio_get_input(pin, &value);
	return value == OS_HAL_GPIO_DATA_HIGH;
}

float	get_distance(u8 pin, unsigned long timeoutMicroseconds)
{
	uint32_t pulseBegin, pulseEnd;

	mtk_os_hal_gpio_set_direction(pin, OS_HAL_GPIO_DIR_OUTPUT);	// set for output
	mtk_os_hal_gpio_set_output(pin, OS_HAL_GPIO_DATA_LOW);		// pull low
	gpt3_wait_microseconds(2);

	mtk_os_hal_gpio_set_output(pin, OS_HAL_GPIO_DATA_HIGH);		// pull high
	gpt3_wait_microseconds(5);

	// GPT3_CTRL - starts microsecond resolution clock
	uint32_t ctrlOn = 0x0;
	ctrlOn |= (0x19) << 16; // OSC_CNT_1US (default value)
	ctrlOn |= 0x1;          // GPT3_EN = 1 -> GPT3 enabled
	write_reg32(GPT_BASE, 0x50, ctrlOn);

	mtk_os_hal_gpio_set_direction(pin, OS_HAL_GPIO_DIR_INPUT);	// set for input

	while (read_input(pin))		// wait for any previous pulse to end
	{
		if (read_reg32(GPT_BASE, 0x58) > timeoutMicroseconds)
		{
			write_reg32(GPT_BASE, 0x50, 0x0);	// GPT_CTRL -> disable timer
			return NAN;
		}
	}

	while (!read_input(pin))		// wait for the pulse to start
	{
		pulseBegin = read_reg32(GPT_BASE, 0x58);
		if (read_reg32(GPT_BASE, 0x58) > timeoutMicroseconds)
		{
			write_reg32(GPT_BASE, 0x50, 0x0);	// GPT_CTRL -> disable timer
			return NAN;
		}
	}

	pulseBegin = read_reg32(GPT_BASE, 0x58);

	while (read_input(pin))		// wait for the pulse to stop
	{
		if (read_reg32(GPT_BASE, 0x58) > timeoutMicroseconds)
		{
			write_reg32(GPT_BASE, 0x50, 0x0);	// GPT_CTRL -> disable timer
			return NAN;
		}
	}

	pulseEnd = read_reg32(GPT_BASE, 0x58);
	
	write_reg32(GPT_BASE, 0x50, 0x0);	// GPT_CTRL -> disable timer

	return (pulseEnd - pulseBegin) / 58.0; //  (29 / 2);
}

void	thread_measure_distance_entry(ULONG thread_input)
{

	gpt3_wait_microseconds(5000000);

	while (true)
	{
		distance_left = get_distance(HCSR04_LEFT, 3000);
		distance_right = get_distance(HCSR04_RIGHT, 3000);

		if (!isnan(distance_left))
		{
			set_distance_indicator((int)distance_left);
			newDistanceLeft = distance_left;
			newDataReady = true;
		}
		else
		{
			for (size_t i = 0; i < 3; i++)
			{
				gpio_output(leds[i], true);
			}
		}

		tx_thread_sleep(1);  // for some reason this is hanging
	}
}

/*
 *
 * interface to microchip asf support for same70
 *
 */
#ifndef same70_H_
#define same70_H_
#define DONT_USE_CMSIS_INIT
#define __SAME70Q20__
#include <asf/same70q20.h>
#include <io_armv7.h>
#include <mpu_armv7.h>

#define MAX_PERIPHERAL_ID    					68
#define SAME70_MAXIMUM_CLOCK_FREQUENCY		(300000000.0)
#define ENABLE_INTERRUPTS	\
	do {	\
		__DSB();	\
		__ISB();	\
		__DMB();	\
		__enable_irq();	\
	} while (0)

#define DISABLE_INTERRUPTS	\
	do {	\
		__disable_irq();	\
		__DSB();	\
		__ISB();	\
		__DMB();	\
	} while (0)

#define HIGHEST_INTERRUPT_PRIORITY			0
#define HIGH_INTERRUPT_PRIORITY				1
#define NORMAL_INTERRUPT_PRIORITY			2
#define LOW_INTERRUPT_PRIORITY				3
#define LOWEST_INTERRUPT_PRIORITY			((1 << __NVIC_PRIO_BITS) - 1)
#define EVENT_LOOP_INTERRUPT_PRIORITY		LOWEST_INTERRUPT_PRIORITY

typedef struct PACK_STRUCTURE stm32_io_dma_channel {
	IO_DMA_CHANNEL_STRUCT_MEMBERS
	
	struct PACK_STRUCTURE {
		uint32_t dma_block:2;			// 1|2
		uint32_t channel_number:6;		//
		uint32_t peripheral_id:8;		// ??
		uint32_t priority:2;				// 0 .. 3
		uint32_t direction:1;			//
		uint32_t circular:1;				//
		uint32_t :12;						//
	} bit;
	uint32_t peripheral_address;
} stm32_io_dma_channel_t;

#define stm32_io_dma_peripheral_id(ch)						(ch)->bit.peripheral_id
#define stm32_io_dma_channel_number(ch)					(ch)->bit.channel_number
#define stm32_io_dma_channel_peripheral_address(ch)	(ch)->peripheral_address

void	same70_register_dma_channel (io_t*,io_dma_channel_t*);

#include <same70_flash.h>
#include <same70_pins.h>
#include <same70_clocks.h>
#include <same70_uart.h>
#include <same70_mpu.h>
#include <same70_timer.h>


#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------

#endif /* IMPLEMENT_IO_CPU */
#endif
/*
------------------------------------------------------------------------------
This software is available under 2 licenses -- choose whichever you prefer.
------------------------------------------------------------------------------
ALTERNATIVE A - MIT License
Copyright (c) 2020 Gregor Bruce
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------------
ALTERNATIVE B - Public Domain (www.unlicense.org)
This is free and unencumbered software released into the public domain.
Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
software, either in source code form or as a compiled binary, for any purpose,
commercial or non-commercial, and by any means.
In jurisdictions that recognize copyright laws, the author or authors of this
software dedicate any and all copyright interest in the software to the public
domain. We make this dedication for the benefit of the public at large and to
the detriment of our heirs and successors. We intend this dedication to be an
overt act of relinquishment in perpetuity of all present and future rights to
this software under copyright law.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
------------------------------------------------------------------------------
*/


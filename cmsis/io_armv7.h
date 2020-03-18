/*
 *
 * io capabilities common to armv7 cpus
 *
 */
#ifndef io_armv7_H_
#define io_armv7_H_

enum {
	IO_BACK_ROOM_CREATED = 0,
	IO_BACK_ROOM_RUNNING = 1,
	IO_BACK_ROOM_EXITED = 2,
};

//
// Back rooms
// 
typedef struct PACK_STRUCTURE {

	// also have {r4-r11} here:
	uint32_t r[8];

	uint32_t uR0;
	uint32_t uR1;
	uint32_t uR2;
	uint32_t uR3;

	uint32_t uR12;
	uint32_t uLR;
	uint32_t uPC;
	uint32_t uPSR;
	
} armv7_basic_exception_frame_t;

//
// NB: asm code depends on particular offests in struct io_backroom.
//
typedef struct PACK_STRUCTURE io_backroom {
	armv7_basic_exception_frame_t *stack;		// #0
	uint32_t main_lr;									// #4
	uint32_t process_lr;								// #8
	uint32_t status;									// #12
	io_t *io;
	void *user_value;
} io_backroom_t;

#define io_backroom_status(b)	(b)->status

io_backroom_t* create_io_backroom (io_t*,void (*)(io_backroom_t*),void*,uint32_t);
void free_io_backroom (io_backroom_t*);
bool	switch_to_backroom (io_backroom_t*);

//
// when called in a backroom will return to main with 
// execution continuing from the most recent call
// to switch_to_backroom()
//
ALWAYS_INLINE_FUNCTION void
yield_to_main (io_backroom_t *this) {
	register int r0 __ASM ("r0") = (int) this;
	__ASM volatile (
		"SVC 0x18 \n" : : "r"(r0)
	);
}

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------

//
// When called from main, execution will resume in the backroom directly
// following the last call to yield_to_main().
//
bool
switch_to_backroom (io_backroom_t *this) {
	if (io_backroom_status(this) == IO_BACK_ROOM_CREATED) {
		io_backroom_status(this) = IO_BACK_ROOM_RUNNING;
	}
	
	if (io_backroom_status(this) != IO_BACK_ROOM_EXITED) {
		register int r0 __ASM ("r0") = (int) this;
		__ASM volatile (
			"SVC 0x18 \n" : : "r"(r0)
		);
		return true;
	} else {
		return false;
	}
}

static void
exit_backroom_stage_2 (io_backroom_t *this) {
	io_backroom_status(this) = IO_BACK_ROOM_EXITED;
	yield_to_main (this);
	// should not happen
	while(1);
}

//
// need naked because we need a deterministic value for psp
//
__attribute__ ((naked)) void
exit_backroom (void) {
	io_backroom_t *this  = *((io_backroom_t**) __get_PSP());
	exit_backroom_stage_2 (this);
	// should not happen
	while(1);
}

__attribute__ ((naked)) void
goto_backroom (void) {
	__ASM volatile (
		"bx r2	\n"
	);
}

io_backroom_t*
create_io_backroom (
	io_t *io,void (*entry)(io_backroom_t*),void *user_value,uint32_t stack_size
) {
	io_backroom_t *this = io_byte_memory_allocate (
		io_get_byte_memory (io),sizeof (io_backroom_t) + stack_size + sizeof(uint32_t)
	);
	
	if (this) {
		uint32_t stack_top = (uint32_t) (this + 1) + stack_size - sizeof(armv7_basic_exception_frame_t);
		
		// must be 8-byte aligned
		if (stack_top & 0x00000007) {
			stack_top -= (8 - (stack_top & 0x00000007));
			*((uint32_t*) ((void*)(this + 1) + stack_size - 4)) = (uint32_t) this;
		} else {
			*((uint32_t*) ((void*)(this + 1) + stack_size)) = (uint32_t) this;
		}
		
		this->io = io;
		this->process_lr = 0xfffffffd;
		this->main_lr = 0;
		this->status = IO_BACK_ROOM_CREATED;
		this->user_value = user_value;
		this->stack = (void*) stack_top;
		this->stack->uR2 = (uint32_t) entry;
		this->stack->uR1 = (uint32_t) user_value;
		this->stack->uR0 = (uint32_t) this;
		this->stack->uPC = (uint32_t) goto_backroom;
		this->stack->uLR = ((uint32_t) exit_backroom | 0x1UL);
		this->stack->uPSR = 0x01000000UL;	//Thumb mode
	}
	
	return this;
}

void
free_io_backroom (io_backroom_t *this) {
	if (io_backroom_status(this) != IO_BACK_ROOM_RUNNING) {
		io_byte_memory_free (io_get_byte_memory (this->io),this);
	} else {
		io_panic (this->io,0);
	}
}

//
// service interrupt call: 
//
// NB:
//
// Must be triggered from SVC call that also load a pointer 
// to the backroom into r0.
//
// Depends on fixed offsets into io_backroom_t.
//
__attribute__ ((naked)) void
io_backroom_service_call (void) {
	//
	// r0 = backroom address
	//
	__ASM volatile (
		"	mov  r2,#0x04					\n"
		"	and  r2,lr,r2					\n"
		"	cbnz r2,_sw_to_main			\n"
	"_sw_to_backroom:"
		"  stmdb sp!, {r4-r11}			\n"	// save main context 'top' registers
		"  tst lr, #0x10					\n" 	// push high vfp registers if main was using the FPU
		"  it eq								\n"
		"  vstmdbeq sp!, {s16-s31}		\n"
		"  str  lr,[r0,#4]				\n"	// save main lr
		"  ldr  r2,[r0,#0]				\n"	// get saved psp
		"  ldr  lr,[r0,#8]				\n"	// get saved lr
		"  ldmia r2!, {r4-r11}			\n"	// restore process context
		"  tst lr, #0x10					\n" 	// restore high vfp registers if process was using the FPU
		"  it eq								\n"
		"  vldmiaeq r2!, {s16-s31}		\n"
		"	msr  psp, r2					\n"	// set psp from saved stack pointer
		"	bx   lr							\n"
	"_sw_to_main:"
		"	mrs  r1,psp 					\n"
		"  stmdb r1!, {r4-r11}			\n"	// save process context 'top' registers
		"  tst lr, #0x10					\n" 	// push high vfp registers if process was using the FPU
		"  it eq								\n"
		"  vstmdbeq r1!, {s16-s31}		\n"
		"  str  r1,[r0,#0] 				\n"	// store the process stack pointer
		"  str  lr,[r0,#8] 				\n"	// save the lr value to return to the process-sp
		"	ldr  r1,[r0,#4] 				\n"	// r1 now holds the 'lr' value to return to the main-sp
		"	mrs  r2,msp 					\n"	// get msp
		"  tst r1, #0x10					\n"	// restore high vfp registers if main was using the FPU
		"  it eq								\n"
		"  vldmiaeq r2!, {s16-s31}		\n"
		"  ldmia r2!, {r4-r11}			\n"
		"	msr  msp,r2 					\n"	// get back to the original msp
		"	bx   r1							\n"
	);
}
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

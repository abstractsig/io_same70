/*
 *
 * io same70 cpu
 *
 */
#ifndef io_cpu_H_
#define io_cpu_H_
#include <io_core.h>
#include <same70.h>

typedef struct PACK_STRUCTURE same70_time_clock {

	Tc *timer;
	uint32_t high_count;

	float64_t reference_frequency;
	
	int32_t interrupt_number;
	io_cpu_clock_pointer_t channel_clock[3];
	io_cpu_clock_pointer_t reference_clock;
	io_event_t alarm;
	io_t *io;
	
} same70_time_clock_t;

void	start_same70_time_clock (io_t*);

typedef struct same70_trng {
	io_cpu_clock_pointer_t clock;
	io_cell_t cell;

	Trng *registers;
	
	uint32_t started;
	
} same70_trng_t;

void	start_same70_trng (io_t*);

//
// cpu io
//
#define SAME70_IO_CPU_STRUCT_MEMBERS \
	IO_STRUCT_MEMBERS				\
	io_value_memory_t *vm;\
	io_byte_memory_t *bm;\
	uint32_t in_event_thread;\
	io_value_pipe_t *tasks;\
	uint32_t prbs_state[4]; \
	same70_time_clock_t tc;\
	io_dma_channel_t *dma_channel_list;\
	io_cpu_clock_pointer_t dma_clock;\
	uint32_t first_run;\
	same70_trng_t trng;\
	/**/

typedef struct PACK_STRUCTURE io_same70_cpu {
	SAME70_IO_CPU_STRUCT_MEMBERS
} io_same70_cpu_t;

void	initialise_cpu_io (io_t*);

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// same70 Implementation
//
//-----------------------------------------------------------------------------
#include <same70_time_clock.h>
#include <same70_dma.h>
#include <same70_trng.h>

#define NUMBER_OF_ARM_INTERRUPT_VECTORS	16L
#define NUMBER_OF_NRF_INTERRUPT_VECTORS	69L
#define NUMBER_OF_INTERRUPT_VECTORS	(NUMBER_OF_ARM_INTERRUPT_VECTORS + NUMBER_OF_NRF_INTERRUPT_VECTORS)

static io_interrupt_handler_t cpu_interrupts[NUMBER_OF_INTERRUPT_VECTORS];

static void
null_interrupt_handler (void *w) {
	while(1);
}

//
// io methods
//
static io_byte_memory_t*
same70_io_get_byte_memory (io_t *io) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;
	return this->bm;
}

static io_value_memory_t*
same70_io_get_stvm (io_t *io) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;
	return this->vm;
}

static void
same70_do_gc (io_t *io,int32_t count) {
	io_value_memory_do_gc (io_get_short_term_value_memory (io),count);
}

static void
same70_signal_task_pending (io_t *io) {
	// no action required
}

static void
same70_signal_event_pending (io_t *io) {
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

static bool
same70_enqueue_task (io_t *io,vref_t r_task) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;
	return io_value_pipe_put_value (this->tasks,r_task);
}

static bool
same70_do_next_task (io_t *io) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;
	vref_t r_task;
	if (io_value_pipe_get_value (this->tasks,&r_task)) {
		vref_t const *argv;
		uint32_t argc;
		
		if (io_vector_value_get_values (r_task,&argc,&argv)) {
			
			// ...
		
			return true;
		}
	}
	return false;
}

static bool
same70_enter_critical_section (io_t *env) {
	uint32_t interrupts_are_enabled = !(__get_PRIMASK() & 0x1);
	DISABLE_INTERRUPTS;
	return interrupts_are_enabled;
}

void
same70_exit_critical_section (io_t *env,bool were_enabled) {
	if (were_enabled) {
		ENABLE_INTERRUPTS;
	}
}

static bool
same70_is_in_event_thread (io_t *io) {
	return ((io_same70_cpu_t*) io)->in_event_thread;
}

static void
same70_wait_for_event (io_t *io) {
	__WFI();
}

static void
same70_for_all_events (io_t *io) {
	io_event_t *event;
	io_alarm_t *alarm;
	do {
		ENTER_CRITICAL_SECTION(io);
		event = io->events;
		alarm = io->alarms;
		EXIT_CRITICAL_SECTION(io);
	} while (
			event != &s_null_io_event
		&&	alarm != &s_null_io_alarm
	);
}

static void	
same70_register_interrupt_handler (
	io_t *io,int32_t number,io_interrupt_action_t handler,void *user_value
) {
	io_interrupt_handler_t *i = (
		cpu_interrupts + number + NUMBER_OF_ARM_INTERRUPT_VECTORS
	);
	i->action = handler;
	i->user_value = user_value;
}

static bool	
same70_unregister_interrupt_handler (
	io_t *io,int32_t number,io_interrupt_action_t handler
) {
	io_interrupt_handler_t *i = (
		cpu_interrupts + number + NUMBER_OF_ARM_INTERRUPT_VECTORS
	);
	if (i->action == handler) {
		i->action = null_interrupt_handler;
		i->user_value = io;
		return true;
	} else {
		return false;
	}
}

INLINE_FUNCTION uint32_t prbs_rotl(const uint32_t x, int k) {
	return (x << k) | (x >> (32 - k));
}

static uint32_t
same70_get_prbs_random_u32 (io_t *io) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;
	uint32_t *s = this->prbs_state;
	bool h = enter_io_critical_section (io);
	const uint32_t result = prbs_rotl (s[0] + s[3], 7) + s[0];

	const uint32_t t = s[1] << 9;

	s[2] ^= s[0];
	s[3] ^= s[1];
	s[1] ^= s[2];
	s[0] ^= s[3];

	s[2] ^= t;

	s[3] = prbs_rotl (s[3], 11);

	exit_io_critical_section (io,h);
	return result;
}

static bool
same70_is_first_run (io_t *io) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;
	return this->first_run;
}

static void
same70_panic (io_t *io,int code) {
	DISABLE_INTERRUPTS;
	while (1);
}

static void
same70_log (io_t *io,char const *fmt,va_list va) {
	// ...
}

void
add_io_implementation_cpu_methods (io_implementation_t *io_i) {
	add_io_implementation_core_methods (io_i);

	io_i->is_first_run = same70_is_first_run;
	io_i->get_byte_memory = same70_io_get_byte_memory;
	io_i->get_short_term_value_memory = same70_io_get_stvm;
	io_i->do_gc = same70_do_gc;
	io_i->get_random_u32 = same70_get_random_u32;
	io_i->get_next_prbs_u32 = same70_get_prbs_random_u32;
	io_i->signal_task_pending = same70_signal_task_pending;
	io_i->enqueue_task = same70_enqueue_task;
	io_i->do_next_task = same70_do_next_task;
	io_i->signal_event_pending = same70_signal_event_pending;
	io_i->enter_critical_section = same70_enter_critical_section;
	io_i->exit_critical_section = same70_exit_critical_section;
	io_i->in_event_thread = same70_is_in_event_thread;
	io_i->wait_for_event = same70_wait_for_event;
	io_i->get_time = same70_get_time,
	io_i->enqueue_alarm = same70_enqueue_alarm;
	io_i->dequeue_alarm = same70_dequeue_alarm;
	io_i->register_interrupt_handler = same70_register_interrupt_handler;
	io_i->unregister_interrupt_handler = same70_unregister_interrupt_handler;
	io_i->wait_for_all_events = same70_for_all_events;
	io_i->set_io_pin_output = same70_set_io_pin_to_output,
	io_i->set_io_pin_input = same70_set_io_pin_to_input,
	io_i->set_io_pin_interrupt = same70_set_io_pin_interrupt,
	io_i->set_io_pin_alternate = same70_set_io_pin_to_alternate,
	io_i->read_from_io_pin = same70_read_io_input_pin,
	io_i->write_to_io_pin = same70_write_to_io_pin,
	io_i->toggle_io_pin = same70_toggle_io_pin,
	io_i->valid_pin = same70_io_pin_test_valid,
	io_i->release_io_pin = same70_io_pin_release,
	io_i->panic = same70_panic;
	io_i->log = same70_log;
}

static void
event_thread (void *io) {
	io_same70_cpu_t *this = io;
	this->in_event_thread = true;
	while (next_io_event (io));
	this->in_event_thread = false;
}

static void
hard_fault (void *io) {
	DISABLE_INTERRUPTS;
	while(1);
}

void
initialise_cpu_io (io_t *io) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;

	this->in_event_thread = false;
	this->first_run = same70_io_config_is_first_run();

	io_cpu_clock_start (io,io_get_core_clock(io));

	register_io_interrupt_handler (io,PendSV_IRQn,event_thread,io);
	register_io_interrupt_handler (io,HardFault_IRQn,hard_fault,io);

	start_same70_time_clock (io);
	
	this->dma_channel_list = &null_dma_channel;
}

static void
initialise_ram_interrupt_vectors (void) {
	io_interrupt_handler_t *i = cpu_interrupts;
	io_interrupt_handler_t *e = i + NUMBER_OF_INTERRUPT_VECTORS;
	while (i < e) {
		i->action = null_interrupt_handler;
		i->user_value = NULL;
		i++;
	}
}

static void
initialise_c_runtime (void) {
	extern uint32_t ld_start_of_sdata_in_flash;
	extern uint32_t ld_start_of_sdata_in_ram,ld_end_of_sdata_in_ram;
	extern uint32_t ld_start_of_bss,ld_end_of_bss;

	uint32_t *src = &ld_start_of_sdata_in_flash;
	uint32_t *dest = &ld_start_of_sdata_in_ram;

	while(dest < &ld_end_of_sdata_in_ram) *dest++ = *src++;
	dest = &ld_start_of_bss;
	while(dest < &ld_end_of_bss) *dest++ = 0;

	// fill stack/heap region of RAM with a pattern
	extern uint32_t ld_end_of_static_ram_allocations;
	uint32_t *end = (uint32_t*) __get_MSP();
	dest = &ld_end_of_static_ram_allocations;
	while (dest < end) {
		*dest++ = 0xdeadc0de;
	}
	
	initialise_ram_interrupt_vectors ();
}

static void
tune_cpu (void) {
	// see GPNVM, TCM is disabled?
	
	#if (__FPU_USED == 1)
	//
	// get here even with soft-abi
	//
	// enable FPU, full access to CP10 and CP11
	SCB->CPACR |= (
			(3UL << 10*2)
		|	(3UL << 11*2)
	);

	// lazy stack pushing for floats
	FPU->FPCCR |= (
			FPU_FPCCR_ASPEN_Msk
		| FPU_FPCCR_LSPEN_Msk
	);
	#endif
}

int main(void);

void
same70_core_reset (void) {
	initialise_c_runtime ();
	tune_cpu ();
	main ();
	while (1);
}

static void
handle_io_cpu_interrupt (void) {
	io_interrupt_handler_t const *interrupt = &cpu_interrupts[
		SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk
	];
	interrupt->action(interrupt->user_value);
}

extern uint32_t ld_top_of_c_stack;
__attribute__ ((section(".isr_vector")))
const void* s_flash_vector_table[NUMBER_OF_INTERRUPT_VECTORS] = {
	&ld_top_of_c_stack,
	same70_core_reset,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	io_backroom_service_call,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,

	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
};

#endif /* IMPLEMENT_IO_CPU */
#ifdef IMPLEMENT_VERIFY_IO_CPU
TEST_BEGIN(test_io_random_1) {
	uint32_t rand[3];

/*	rand[0] = io_get_random_u32(TEST_IO);
	rand[1] = io_get_random_u32(TEST_IO);
	rand[2] = io_get_random_u32(TEST_IO);


	if (rand[0] == rand[1])	rand[1] = io_get_random_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_random_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_random_u32(TEST_IO);

	if (rand[1] == rand[2]) rand[2] = io_get_random_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_random_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_random_u32(TEST_IO);

	VERIFY(rand[0] != rand[1],NULL);
	VERIFY(rand[1] != rand[2],NULL);
*/
	rand[0] = io_get_next_prbs_u32(TEST_IO);
	rand[1] = io_get_next_prbs_u32(TEST_IO);
	rand[2] = io_get_next_prbs_u32(TEST_IO);

	if (rand[0] == rand[1])	rand[1] = io_get_next_prbs_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_next_prbs_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_next_prbs_u32(TEST_IO);

	if (rand[1] == rand[2]) rand[2] = io_get_next_prbs_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_next_prbs_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_next_prbs_u32(TEST_IO);

	VERIFY(rand[0] != rand[1],NULL);
	VERIFY(rand[1] != rand[2],NULL);
}
TEST_END

static void
test_io_events_1_ev (io_event_t *ev) {
	*((uint32_t*) ev->user_value) = 1;
}

TEST_BEGIN(test_io_events_1) {
	volatile uint32_t a = 0;
	io_event_t ev;
		
	initialise_io_event (&ev,test_io_events_1_ev,(void*) &a);

	io_enqueue_event (TEST_IO,&ev);
	while (a == 0);
	VERIFY (a == 1,NULL);
}
TEST_END

TEST_BEGIN(test_time_clock_alarms_1) {
	volatile uint32_t a = 0;
	io_alarm_t alarm;
	io_event_t ev;
	io_time_t t;
	initialise_io_event (&ev,test_io_events_1_ev,(void*) &a);
	
	t = io_get_time (TEST_IO);
	initialise_io_alarm (
		&alarm,&ev,&ev,
		(io_time_t) {t.nanoseconds + millisecond_time(200).nanoseconds}
	);

	io_enqueue_alarm (TEST_IO,&alarm);
	
	while (a == 0);
	VERIFY (a == 1,NULL);	
}
TEST_END

static void
test_io_backroom_1_entry (io_backroom_t *p) {
	(*((int*) p->user_value))++;
}

TEST_BEGIN(test_io_backroom_1) {
	io_byte_memory_t *bm = io_get_byte_memory (TEST_IO);
	memory_info_t begin,end;
	io_backroom_t *br;
	int r = 0;
	
	io_byte_memory_get_info (bm,&begin);

	br = create_io_backroom (TEST_IO,test_io_backroom_1_entry,&r,256);
	VERIFY (switch_to_backroom (br) && r == 1,NULL);

	free_io_backroom (br);

	io_byte_memory_get_info (bm,&end);
	VERIFY (end.used_bytes == begin.used_bytes,NULL);
}
TEST_END

static void
test_io_backroom_2_entry (io_backroom_t *p) {
	int *c = p->user_value;
	
	while (*c < 2) {
		*c = *c + 1;
		yield_to_main (p);
	}
}

TEST_BEGIN(test_io_backroom_2) {
	io_byte_memory_t *bm = io_get_byte_memory (TEST_IO);
	memory_info_t begin,end;
	io_backroom_t *br;
	int r = 0;
	
	io_byte_memory_get_info (bm,&begin);

	br = create_io_backroom (TEST_IO,test_io_backroom_2_entry,&r,256);
	while (switch_to_backroom (br));
	
	VERIFY (r == 2,NULL);

	free_io_backroom (br);

	io_byte_memory_get_info (bm,&end);
	VERIFY (end.used_bytes == begin.used_bytes,NULL);
}
TEST_END

static void
test_io_backroom_3_entry (io_backroom_t *br) {
	int *c = br->user_value;
	
	while (1) {
		*c = *c + 1;
		if (*c >= 2) break;
		yield_to_main (br);
	};
}

TEST_BEGIN(test_io_backroom_3) {
	io_byte_memory_t *bm = io_get_byte_memory (TEST_IO);
	memory_info_t begin,end;
	io_backroom_t *p1,*p2;
	int r = 0, s = 1;
	
	io_byte_memory_get_info (bm,&begin);

	p1 = create_io_backroom (TEST_IO,test_io_backroom_3_entry,&r,256);
	p2 = create_io_backroom (TEST_IO,test_io_backroom_3_entry,&s,256);

	VERIFY (switch_to_backroom (p2),NULL);
	VERIFY (r == 0 && s == 2,NULL);
	VERIFY (switch_to_backroom (p1),NULL);
	VERIFY (r == 1 && s == 2,NULL);
	
	VERIFY (switch_to_backroom (p1),NULL);
	
	VERIFY (s == 2 && r == 2,NULL);
	VERIFY (!switch_to_backroom (p2),NULL);
	VERIFY (s == 2 && r == 2,NULL);

	VERIFY (!switch_to_backroom (p1),NULL);
	VERIFY (!switch_to_backroom (p2),NULL);

	free_io_backroom (p1);
	free_io_backroom (p2);

	io_byte_memory_get_info (bm,&end);
	VERIFY (end.used_bytes == begin.used_bytes,NULL);
}
TEST_END

UNIT_SETUP(setup_io_cpu_unit_test) {
	return VERIFY_UNIT_CONTINUE;
}

UNIT_TEARDOWN(teardown_io_cpu_unit_test) {
}

void
io_cpu_unit_test (V_unit_test_t *unit) {
	static V_test_t const tests[] = {
		test_io_events_1,
		test_time_clock_alarms_1,
#if 1
		test_io_backroom_1,
		test_io_backroom_2,
		test_io_backroom_3,
#endif
		0
	};
	unit->name = "io cpu";
	unit->description = "io cpu unit test";
	unit->tests = tests;
	unit->setup = setup_io_cpu_unit_test;
	unit->teardown = teardown_io_cpu_unit_test;
}
#define IO_CPU_UNIT_TESTS \
	io_cpu_unit_test, \
	/**/
#else
#define IO_CPU_UNIT_TESTS
#endif /* IMPLEMENT_VERIFY_IO_CPU */
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

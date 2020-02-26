/*
 *
 * io same70 cpu
 *
 */
#ifndef io_cpu_H_
#define io_cpu_H_
#include <io_core.h>
#include <same70q20.h>

typedef struct PACK_STRUCTURE same70_main_rc_oscillator {
	IO_CPU_CLOCK_SOURCE_STRUCT_MEMBERS
	float64_t frequency;
} same70_main_rc_oscillator_t;

extern EVENT_DATA io_cpu_clock_implementation_t same70_main_rc_oscillator_implementation;

typedef struct same70_core_clock {
	IO_CPU_DEPENDANT_CLOCK_STRUCT_MEMBERS
} same70_core_clock_t;

extern EVENT_DATA io_cpu_clock_implementation_t same70_core_clock_implementation;

//
// pins
//
typedef union PACK_STRUCTURE {
	io_pin_t io;
	uint32_t u32;
	struct PACK_STRUCTURE {
		uint32_t port_id:4;
		uint32_t number:6;
		uint32_t multiplex:6;
		uint32_t interrupt_sense:3;
		uint32_t interrupt_mode:1;
		uint32_t eic_channel:4;
		uint32_t active_level:1;
		uint32_t initial_state:1;
		uint32_t pull_mode:2;
		uint32_t drive_strength:1;
		uint32_t :3;
	} sam;
} sam_io_pin_t;


#define SAME70_IO_CPU_STRUCT_MEMBERS \
	IO_STRUCT_MEMBERS				\
	io_value_memory_t *vm;\
	io_byte_memory_t *bm;\
	uint32_t in_event_thread;\
	io_value_pipe_t *tasks;\
	/**/

typedef struct PACK_STRUCTURE io_same70_cpu {
	SAME70_IO_CPU_STRUCT_MEMBERS
} io_same70_cpu_t;

void	initialise_cpu_io (io_t*);


#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// same70 Implementtaion
//
//-----------------------------------------------------------------------------

#define NUMBER_OF_ARM_INTERRUPT_VECTORS	16L
#define NUMBER_OF_NRF_INTERRUPT_VECTORS	69L
#define NUMBER_OF_INTERRUPT_VECTORS	(NUMBER_OF_ARM_INTERRUPT_VECTORS + NUMBER_OF_NRF_INTERRUPT_VECTORS)

static io_interrupt_handler_t cpu_interrupts[NUMBER_OF_INTERRUPT_VECTORS];

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


static void
null_interrupt_handler (void *w) {
	while(1);
}

static float64_t
same70_main_rc_oscillator_get_frequency (io_cpu_clock_pointer_t this) {
	same70_main_rc_oscillator_t const *c = (same70_main_rc_oscillator_t const*) (
		io_cpu_clock_ro_pointer (this)
	);
	return c->frequency;
}

static bool
same70_main_rc_oscillator_start (io_cpu_clock_pointer_t this) {
	return true;
/*
	if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == 0) {
		RCC_HSEConfig(RCC_HSE_ON);
		if (RCC_WaitForHSEStartUp () == SUCCESS) {
			return true;
		} else {
			return false;
		}
	} else {
		
	}
*/
}

EVENT_DATA io_cpu_clock_implementation_t same70_main_rc_oscillator_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_frequency = same70_main_rc_oscillator_get_frequency,
	.link_input_to_output = NULL,
	.link_output_to_input = NULL,
	.start = same70_main_rc_oscillator_start,
	.stop = NULL,
};

bool
same70_clock_is_main_rc (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_main_rc_oscillator_implementation);
}

static float64_t
same70_core_clock_get_frequency (io_cpu_clock_pointer_t clock) {
	same70_core_clock_t const *this = (same70_core_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	return io_cpu_clock_get_frequency (this->input);
}

static bool
same70_core_clock_start (io_cpu_clock_pointer_t clock) {
	if (io_cpu_dependant_clock_start_input (clock)) {

		return true;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t same70_core_clock_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_frequency = same70_core_clock_get_frequency,
	.link_input_to_output = NULL,
	.link_output_to_input = NULL,
	.start = same70_core_clock_start,
	.stop = NULL,
};

//
// pins
//
Pio* port_map[] = {
	PIOA,
	PIOB,
	PIOC,
	PIOD,
	PIOE,
};


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

static uint32_t
same70_get_random_u32 (io_t *io) {
	uint32_t r = 0;
	return r;
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

	io_i->get_byte_memory = same70_io_get_byte_memory;
	io_i->get_short_term_value_memory = same70_io_get_stvm;
	io_i->do_gc = same70_do_gc;
	io_i->get_random_u32 = same70_get_random_u32;
	io_i->signal_task_pending = same70_signal_task_pending;
	io_i->enqueue_task = same70_enqueue_task;
//	io_i->do_next_task = same70_do_next_task;
	io_i->signal_event_pending = same70_signal_event_pending;
	io_i->enter_critical_section = same70_enter_critical_section;
	io_i->exit_critical_section = same70_exit_critical_section;
	io_i->in_event_thread = same70_is_in_event_thread;
	io_i->wait_for_event = same70_wait_for_event;
//	io_i->get_time = same70_get_time,
//	io_i->enqueue_alarm = nrf_time_clock_enqueue_alarm;
//	io_i->dequeue_alarm = nrf_time_clock_dequeue_alarm;
	io_i->register_interrupt_handler = same70_register_interrupt_handler;
	io_i->unregister_interrupt_handler = same70_unregister_interrupt_handler;
	io_i->wait_for_all_events = same70_for_all_events;
//	io_i->set_io_pin_output = same70_set_io_pin_to_output,
//	io_i->set_io_pin_input = same70_set_io_pin_to_input,
//	io_i->set_io_pin_interrupt = same70_set_io_pin_interrupt,
//	io_i->set_io_pin_alternate = io_pin_nop,
//	io_i->read_from_io_pin = same70_read_io_input_pin,
//	io_i->write_to_io_pin = same70_write_to_io_pin,
//	io_i->toggle_io_pin = same70_toggle_io_pin,
//	io_i->valid_pin = same70_io_pin_is_valid,
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

	io_cpu_clock_start (io_get_core_clock(io));

	register_io_interrupt_handler (io,PendSV_IRQn,event_thread,io);
	register_io_interrupt_handler (io,HardFault_IRQn,hard_fault,io);
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
  /* enable FPU if available and used */
  SCB->CPACR |= ((3UL << 10*2) |             /* set CP10 Full Access               */
                 (3UL << 11*2)  );           /* set CP11 Full Access               */
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
	handle_io_cpu_interrupt,
};


#endif /* IMPLEMENT_IO_CPU */
#ifdef IMPLEMENT_VERIFY_IO_CPU

#define IO_CPU_UNIT_TESTS \
	/**/
#else
#define IO_CPU_UNIT_TESTS
#endif /* IMPLEMENT_VERIFY_IO_CPU */
#endif

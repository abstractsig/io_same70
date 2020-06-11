/*
 *
 * same70 Pio
 *
 */
#ifndef same70_pins_H_
#define same70_pins_H_

enum {
	SAME70_PIOA = 0,
	SAME70_PIOB,
	SAME70_PIOC,
	SAME70_PIOD,
	SAME70_PIOE,
	
	SAME70_INVALID_PORT_ID
};

typedef union PACK_STRUCTURE {
	io_pin_t io;
	uint32_t u32;
	struct PACK_STRUCTURE {
		uint32_t port_id:3;
		uint32_t number:5;
		uint32_t is_output:1;
		uint32_t active_level:1;
		uint32_t initial_state:1;
		uint32_t drive_strength:1;
		uint32_t peripheral_1:1;
		uint32_t peripheral_2:1;
		uint32_t multi_driver:1;
		uint32_t pull_up:1;
		uint32_t pull_down:1;
		uint32_t debounce:1;
		uint32_t schmitt:1;
		uint32_t interrupt:1;
		uint32_t interrupt_edge:1;
		uint32_t interrupt_rising:1;
		uint32_t interrupt_falling:1;
		uint32_t :9;
	} sam;
} same70_io_pin_t;

#define same70_io_pin_port_number(pin)				(pin).sam.port_id
#define same70_io_pin_number(pin)					(pin).sam.number
#define same70_io_pin_is_output(pin)				(pin).sam.is_output
#define same70_io_pin_pull_up(pin)					(pin).sam.pull_up
#define same70_io_pin_pull_down(pin)				(pin).sam.pull_down
#define same70_io_pin_peripheral_1(pin)			(pin).sam.peripheral_1
#define same70_io_pin_peripheral_2(pin)			(pin).sam.peripheral_2
#define same70_io_pin_multi_driver(pin)			(pin).sam.multi_driver
#define same70_io_pin_active_level(pin)			(pin).sam.active_level
#define same70_io_pin_initial_state(p)				(p).sam.initial_state
#define same70_io_pin_drive_strength(p)			(p).sam.drive_strength
#define same70_io_pin_interrupt(pin)				(pin).sam.interrupt
#define same70_io_pin_interrupt_edge(pin)			(pin).sam.interrupt_edge
#define same70_io_pin_interrupt_rising(pin)		(pin).sam.interrupt_rising
#define same70_io_pin_interrupt_falling(pin)		(pin).sam.interrupt_falling

#define same70_io_pin_is_null(pin)					(same70_io_pin_port_number(pin) == SAME70_INVALID_PORT_ID)
#define same70_io_pin_is_valid(pin)					(!same70_io_pin_is_null(pin))

#define IO_PIN_INACTIVE 								0
#define IO_PIN_ACTIVE 									1

#define IO_PIN_ACTIVE_LOW								0
#define IO_PIN_ACTIVE_HIGH								1

#define IO_PIN_LOW_DRIVE 								0
#define IO_PIN_HIGH_DRIVE								1

#define def_same70_null_io_pin() (same70_io_pin_t) {.sam.port_id = SAME70_INVALID_PORT_ID}

#define def_same70_io_output_pin(Port,Pin_number,Active,Initial,D) (same70_io_pin_t) {\
		.sam.port_id = Port,\
		.sam.number = Pin_number,\
		.sam.is_output = 1,\
		.sam.active_level = Active,\
		.sam.initial_state = Initial,\
		.sam.drive_strength = D,\
		.sam.peripheral_1 = 0,\
		.sam.peripheral_2 = 0,\
		.sam.multi_driver = 0,\
		.sam.pull_up = 0,\
		.sam.pull_down = 0,\
		.sam.debounce = 0,\
		.sam.schmitt = 0,\
		.sam.interrupt = 0,\
		.sam.interrupt_edge = 0,\
		.sam.interrupt_rising = 0,\
		.sam.interrupt_falling = 0,\
	}

#define def_same70_io_input_pin(Port,Pin_number,Active,PU,PD) (same70_io_pin_t) {\
		.sam.port_id = Port,\
		.sam.number = Pin_number,\
		.sam.is_output = 0,\
		.sam.active_level = 0,\
		.sam.initial_state = 0,\
		.sam.drive_strength = IO_PIN_LOW_DRIVE,\
		.sam.peripheral_1 = 0,\
		.sam.peripheral_2 = 0,\
		.sam.multi_driver = 0,\
		.sam.pull_up = PU,\
		.sam.pull_down = PD,\
		.sam.debounce = 0,\
		.sam.schmitt = 0,\
		.sam.interrupt = 0,\
		.sam.interrupt_edge = 0,\
		.sam.interrupt_rising = 0,\
		.sam.interrupt_falling = 0,\
	}

#define def_same70_io_interrupt_pin(Port,Pin_number,Active,PU,PD,E,R,F) (same70_io_pin_t) {\
		.sam.port_id = Port,\
		.sam.number = Pin_number,\
		.sam.is_output = 0,\
		.sam.active_level = 0,\
		.sam.initial_state = 0,\
		.sam.drive_strength = IO_PIN_LOW_DRIVE,\
		.sam.peripheral_1 = 0,\
		.sam.peripheral_2 = 0,\
		.sam.multi_driver = 0,\
		.sam.pull_up = PU,\
		.sam.pull_down = PD,\
		.sam.debounce = 0,\
		.sam.schmitt = 0,\
		.sam.interrupt = 1,\
		.sam.interrupt_edge = E,\
		.sam.interrupt_rising = R,\
		.sam.interrupt_falling = F,\
	}

#define def_same70_io_alternate_pin(Port,Pin_number,Output,Active,Initial,A,B) (same70_io_pin_t) {\
		.sam.port_id = Port,\
		.sam.number = Pin_number,\
		.sam.is_output = Output,\
		.sam.active_level = Active,\
		.sam.initial_state = Initial,\
		.sam.drive_strength = IO_PIN_LOW_DRIVE,\
		.sam.peripheral_1 = A,\
		.sam.peripheral_2 = B,\
		.sam.multi_driver = 0,\
		.sam.pull_up = 0,\
		.sam.pull_down = 0,\
		.sam.debounce = 0,\
		.sam.schmitt = 0,\
		.sam.interrupt = 0,\
		.sam.interrupt_edge = 0,\
		.sam.interrupt_rising = 0,\
		.sam.interrupt_falling = 0,\
	}

#define def_same70_io_peripheral_A_output_pin(Port,Pin_number,Active,Initial) \
		def_same70_io_alternate_pin(Port,Pin_number,1,Active,Initial,0,0)

#define def_same70_io_peripheral_A_input_pin(Port,Pin_number,Active,Initial) \
		def_same70_io_alternate_pin(Port,Pin_number,0,Active,Initial,0,0)


typedef struct io_cell io_cell_t;
typedef struct io_cell_implementation io_cell_implementation_t;

struct io_cell {
	void *user_value;
	void (*request_to_stop) (io_cell_t*);
	void (*request_to_increase_effort) (io_cell_t*);
	void (*request_to_decrease_effort) (io_cell_t*);
};

typedef struct same70_pio_port {
	io_cpu_clock_pointer_t clock;
	io_cell_t cell;

	Pio *registers;
	uint32_t active_pins;
	
	int32_t interrupt_number;
	
	io_interrupt_handler_t interrupt_handler[32];
	
} same70_pio_port_t;

#define decl_same70_pio_port(R,C,I) {\
		.registers = R,\
		.clock = C,\
		.interrupt_number = I,\
		.active_pins = 0,\
		.interrupt_handler = {{0}},\
	}

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------
extern same70_pio_port_t same70_pio_ports[];

#define same70_io_pin_port(pin)	(&same70_pio_ports[same70_io_pin_port_number(pin)])
#define same70_io_pin_pio(pin)	same70_io_pin_port(pin)->registers

uint32_t
same70_pio_port_active_pin_count (same70_pio_port_t *port) {
	return __builtin_popcount(port->active_pins);
}

bool
same70_pio_port_pin_is_active (same70_pio_port_t *port,same70_io_pin_t pin) {
	uint32_t mask = 1 << same70_io_pin_number(pin);
	return (port->active_pins & mask) != 0;
}

bool
same70_pio_port_activate_pin (io_t *io,same70_io_pin_t pin) {
	same70_pio_port_t *port = same70_io_pin_port (pin);
	uint32_t mask = 1 << same70_io_pin_number(pin);
	
	if ((port->active_pins & mask) == 0) {
		if (io_cpu_clock_start (io,port->clock)) {
			port->active_pins |= mask;
			return true;
		} else {
			return false;
		}
	} else {
		return true;
	}
}

void
same70_pio_port_deactivate_pin (io_t *io,same70_io_pin_t pin) {
	same70_pio_port_t *port = same70_io_pin_port (pin);
	uint32_t mask = 1 << same70_io_pin_number(pin);
	
	if ((port->active_pins & mask) != 0) {
		port->registers->PIO_PER = mask;		// disable peripheral control
		port->registers->PIO_ODR = mask;		// disable output
		port->registers->PIO_PUDR = mask;	// disable pullup
		port->registers->PIO_PPDDR = mask;	// disable pulldown
		port->active_pins &= ~mask;
		if (same70_pio_port_active_pin_count (port) == 0) {
			io_cpu_clock_stop (io,port->clock);
		}
	}
}

INLINE_FUNCTION void
pio_disable_write_protection (Pio *pio) {
	pio->PIO_WPMR = (
			0
		|	PIO_WPMR_WPKEY_PASSWD
	);
}

INLINE_FUNCTION void
pio_enable_write_protection (Pio *pio) {
	pio->PIO_WPMR = (
			1
		|	PIO_WPMR_WPKEY_PASSWD
	);
}

void
same70_write_to_io_pin (io_t *io,io_pin_t rpin,int32_t state) {
	same70_io_pin_t pin = {rpin};
	Pio *pio = same70_io_pin_pio (pin);
	
	if (state ^ same70_io_pin_active_level (pin)) {
		pio->PIO_CODR = 1 << same70_io_pin_number(pin);
	} else {
		pio->PIO_SODR = 1 << same70_io_pin_number(pin);
	}
}

void
same70_set_io_pin_to_output (io_t *io,io_pin_t rpin) {
	same70_io_pin_t pin = {rpin};
	Pio *pio = same70_io_pin_pio (pin);
	uint32_t mask = 1 << same70_io_pin_number(pin);
	
	if (same70_pio_port_activate_pin (io,pin)) {
	}
	
	pio_disable_write_protection (pio);

	// disable peripheral control
	pio->PIO_PER = mask;

	pio->PIO_MDDR = mask;
	pio->PIO_IDR = mask;
	if (same70_io_pin_pull_up (pin)) {
		pio->PIO_PUER = mask;
	} else {
		pio->PIO_PUDR = mask;
	}
	if (same70_io_pin_pull_down (pin)) {
		pio->PIO_PPDER = mask;
	} else {
		pio->PIO_PPDDR = mask;
	}
	pio->PIO_OWER = mask;
	pio->PIO_IDR = mask;
	
	same70_write_to_io_pin (io,rpin,same70_io_pin_initial_state (pin));
	pio->PIO_OER = mask;
	
	pio_enable_write_protection (pio);
}

static void
setup_same70_pin_as_input (Pio *pio,same70_io_pin_t pin,uint32_t mask) {
	pio->PIO_PER = mask;	// disable peripheral control
	pio->PIO_ODR = mask;	// disable output

	if (same70_io_pin_pull_up (pin)) {
		pio->PIO_PUER = mask;
	} else {
		pio->PIO_PUDR = mask;
	}
	
	if (same70_io_pin_pull_down (pin)) {
		pio->PIO_PPDER = mask;
	} else {
		pio->PIO_PPDDR = mask;
	}
}

void
same70_set_io_pin_to_input (io_t *io,io_pin_t rpin) {
	same70_io_pin_t pin = {rpin};
	Pio *pio = same70_io_pin_pio (pin);
	uint32_t mask = 1 << same70_io_pin_number(pin);

	if (same70_pio_port_activate_pin (io,pin)) {
	}
	
	pio_disable_write_protection (pio);
	
	setup_same70_pin_as_input (pio,pin,mask);
	
	pio_enable_write_protection (pio);
}

void
same70_set_io_pin_to_alternate (io_t *io,io_pin_t rpin) {
	same70_io_pin_t pin = {rpin};
	Pio *pio = same70_io_pin_pio (pin);
	uint32_t mask = 1 << same70_io_pin_number(pin);

	pio_disable_write_protection (pio);

	if (same70_io_pin_is_output (pin)) {
		same70_write_to_io_pin (io,rpin,same70_io_pin_initial_state (pin));
	}
	
	pio->PIO_ABCDSR[0] = (
		same70_io_pin_peripheral_1 (pin) << same70_io_pin_number(pin)
	);
	pio->PIO_ABCDSR[1] = (
		same70_io_pin_peripheral_2 (pin) << same70_io_pin_number(pin)
	);

	// enable peripheral control
	pio->PIO_PDR = mask;
	
	pio_enable_write_protection (pio);
}

int32_t
same70_read_io_input_pin (io_t *io,io_pin_t rpin) {
	same70_io_pin_t pin = {rpin};
	Pio *pio = same70_io_pin_pio (pin);
	uint32_t mask = 1 << same70_io_pin_number(pin);

	return (
			(pio->PIO_PDSR & mask)
		==	(same70_io_pin_active_level (pin) << same70_io_pin_number(pin))
	);
}

static void
same70_io_pin_interrupt_handler (void *user_value) {
	same70_pio_port_t *port = user_value;
	volatile uint32_t status = port->registers->PIO_ISR;
	
	while (status) {
		uint32_t pin_number = __builtin_ctz(status);
		io_interrupt_handler_t *h = &port->interrupt_handler[pin_number];
		if (h->action != NULL) {
			h->action(h);
		} else {
			// this is bad
		}
		
		status &= ~(1 << pin_number);
	}
}

static void
same70_set_io_pin_interrupt (io_t *io,io_pin_t rpin,io_interrupt_handler_t *h) {
	same70_io_pin_t pin = {rpin};
	uint32_t number = same70_io_pin_number(pin);
	same70_pio_port_t *port = &same70_pio_ports[number];

	if (same70_io_pin_interrupt(pin)) {
		if (port->interrupt_handler[number].action == NULL) {
			port->interrupt_handler[number] = *h;
		}
		
		if (same70_pio_port_activate_pin (io,pin)) {
			Pio *pio = port->registers;
			uint32_t mask = 1 << same70_io_pin_number(pin);

			setup_same70_pin_as_input (pio,pin,mask);

			if (pio->PIO_IMR == 0) {
				volatile uint32_t status = pio->PIO_ISR;
				UNUSED(status);
				register_io_interrupt_handler (
					io,port->interrupt_number,same70_io_pin_interrupt_handler,port
				);
				NVIC_SetPriority (port->interrupt_number,NORMAL_INTERRUPT_PRIORITY);
				NVIC_ClearPendingIRQ (port->interrupt_number);
				NVIC_EnableIRQ (port->interrupt_number);
			}
			
			if (same70_io_pin_interrupt_edge(pin)) {
				pio->PIO_ESR = mask;
			} else {
				pio->PIO_LSR = mask;
			}
			
			if (same70_io_pin_interrupt_rising(pin)) {
				pio->PIO_REHLSR = mask;
			}
			
			if (same70_io_pin_interrupt_falling(pin)) {
				pio->PIO_FELLSR = mask;
			}
			
			pio->PIO_IER = mask;
		}
	}
}

void
same70_io_pin_release (io_t *io,io_pin_t rpin) {
	same70_io_pin_t pin = {rpin};
	same70_pio_port_deactivate_pin (io,pin);
}

bool
same70_io_pin_test_valid (io_t *io,io_pin_t rpin) {
	same70_io_pin_t pin = {rpin};
	return same70_io_pin_is_valid (pin);
}

void
same70_toggle_io_pin (io_t *io,io_pin_t rpin) {
	same70_io_pin_t pin = {rpin};
	Pio *pio = same70_io_pin_pio (pin);
	uint32_t mask = 1 << same70_io_pin_number(pin);
	
	if (pio->PIO_ODSR & mask) {
		pio->PIO_CODR = mask;
	} else {
		pio->PIO_SODR = mask;
	}
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


/*
 *
 * for date/time
 *
 */
#ifndef same70_time_clock_H_
#define same70_time_clock_H_

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------

static uint64_t
same70_time_clock_read_64bit_count (same70_time_clock_t *this) {
	io_time_t time = {0};
	Tc *timer = this->timer;
	
	do {
		time.u16[3] = this->high_count;
		time.u16[2] = timer->TC_CHANNEL[2].TC_CV;
		time.u16[1] = timer->TC_CHANNEL[1].TC_CV;
		time.u16[0] = timer->TC_CHANNEL[0].TC_CV;
	} while (
			time.u16[3] != this->high_count
		&&	time.u16[2] != timer->TC_CHANNEL[2].TC_CV
		&& time.u16[1] != timer->TC_CHANNEL[1].TC_CV
	);
	
	return time.u64;
}

bool
set_64bit_time_clock_alarm_time (same70_time_clock_t *this) {
	if (this->io->alarms != &s_null_io_alarm) {
		int64_t next_alarm_count = (int64_t) (
				(float64_t) this->io->alarms->when.ns 
			/	this->reference_frequency
		);
		int64_t current_count = same70_time_clock_read_64bit_count (this) + 2000;
		if (next_alarm_count > current_count) {
			Tc *timer = this->timer;
			tc_disable_write_protection (timer);
			timer->TC_CHANNEL[0].TC_RB = (uint32_t) next_alarm_count;
			tc_enable_write_protection (timer);
			return true;
		} else {
			// alarm is not in future
			return false;
		}
	} else {
		return false;
	}
}

static io_time_t
same70_time_clock_get_time (same70_time_clock_t *this) {
	return (io_time_t) {
		.nanoseconds = (
				(float64_t) same70_time_clock_read_64bit_count(this) 
			*	this->reference_frequency
		),
	};
}

static bool
process_next_alarm (same70_time_clock_t *this) {
	if (this->io->alarms != &s_null_io_alarm) {
		volatile io_time_t t = same70_time_clock_get_time (this);
		
		if (t.nanoseconds >= this->io->alarms->when.nanoseconds) {
			io_alarm_t *alarm = this->io->alarms;
			this->io->alarms = this->io->alarms->next_alarm;
			alarm->next_alarm = NULL;
			alarm->at->event_handler (alarm->at);
			
			//
			// tollerance check ...
			//
			return true;
		} else {
			//while(1) {}
		}
	}
	return false;
}

static void
process_alarm_queue (io_event_t *ev) {
	same70_time_clock_t *this = ev->user_value;
	uint32_t count = 0;
	
	while (process_next_alarm(this)) {
		count++;
	}
	
	if (count) {
		set_64bit_time_clock_alarm_time (this);
	}
}
volatile uint32_t cap_val = 0;

static void
same70_time_clock_compare_interrupt (void *user_value) {
	same70_time_clock_t *this = user_value;
	
	cap_val = this->timer->TC_CHANNEL[0].TC_CV;
	
	volatile uint32_t status = this->timer->TC_CHANNEL[0].TC_SR;
	
	if (status & TC_SR_CPBS) {
		io_enqueue_event (this->io,&this->alarm);
	} else {
//		io_panic (this->io,IO_PANIC_SOMETHING_BAD_HAPPENED);
	}
}

static void
same70_time_clock_counter_overflow (void *user_value) {
	same70_time_clock_t *time_clock = user_value;
	uint32_t status = time_clock->timer->TC_CHANNEL[2].TC_SR;
	if (status & TC_SR_CPCS) {
		time_clock->high_count++;
	} else {
		io_panic (time_clock->io,IO_PANIC_SOMETHING_BAD_HAPPENED);
	}
}

//
// time
//
void
same70_time_clock_apply_timer_settings (Tc *timer) {

	timer->TC_CHANNEL[0].TC_CMR = (
			TC_CMR_WAVE
		|	TC_CMR_WAVSEL_UP
		|	TC_CMR_TCCLKS_TIMER_CLOCK1
		|	TC_CMR_ASWTRG_CLEAR
		|	TC_CMR_ACPA_SET
		|	TC_CMR_ACPC_CLEAR
		|	TC_CMR_EEVT_XC0
	);
	timer->TC_CHANNEL[0].TC_RA = 0x7fff;
	timer->TC_CHANNEL[0].TC_RB = 0x0000;
	timer->TC_CHANNEL[0].TC_RC = 0;
	timer->TC_CHANNEL[0].TC_EMR = 0;
	timer->TC_CHANNEL[0].TC_IER = TC_IER_CPBS;  // RB compare

	timer->TC_CHANNEL[1].TC_CMR = (
			TC_CMR_WAVE
		|	TC_CMR_WAVSEL_UP
		|	TC_CMR_TCCLKS_XC1
		|	TC_CMR_ASWTRG_CLEAR
		|	TC_CMR_ACPA_SET
		|	TC_CMR_ACPC_CLEAR
	);
	timer->TC_CHANNEL[1].TC_RA = 0x7fff;
	timer->TC_CHANNEL[1].TC_RB = 0;
	timer->TC_CHANNEL[1].TC_RC = 0;
	timer->TC_CHANNEL[1].TC_EMR = 0;

	timer->TC_CHANNEL[2].TC_CMR = (
			TC_CMR_WAVE
		|	TC_CMR_WAVSEL_UP
		|	TC_CMR_TCCLKS_XC2
		|	TC_CMR_ASWTRG_CLEAR
		|	TC_CMR_ACPA_SET
		|	TC_CMR_ACPC_CLEAR
	);
	timer->TC_CHANNEL[2].TC_RA = 0x7fff;
	timer->TC_CHANNEL[2].TC_RB = 0;
	timer->TC_CHANNEL[2].TC_RC = 0;
	timer->TC_CHANNEL[2].TC_EMR = 0;
	timer->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;  // RC compare
}

//
// We need to trigger channels 1 & 2 and generate one
// input clock pluse to get them reset. This avoids 
// using the first 'real' pulse to reset the channels
// which would mean the time would lag by(1 << 32) clock
// cycles.
//
void
same70_time_clock_initialise_chain (Tc *timer) {
	// connect in parallel for start
	timer->TC_BMR = (
			0
		|	TC_BMR_TC0XC0S_TIOA2
		|	TC_BMR_TC1XC1S_TIOA0
		|	TC_BMR_TC2XC2S_TIOA0
	);

	timer->TC_CHANNEL[0].TC_CMR |= TC_CMR_CPCDIS;
	timer->TC_CHANNEL[0].TC_RA = 1;
	timer->TC_CHANNEL[0].TC_RC = 2;

	timer->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN;
	timer->TC_CHANNEL[2].TC_CCR = TC_CCR_CLKEN;
	timer->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;

	timer->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG;
	timer->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG;
	timer->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG;

	while (timer->TC_CHANNEL[0].TC_SR & TC_SR_CLKSTA);
	
	timer->TC_CHANNEL[0].TC_CMR &= ~TC_CMR_CPCDIS;
	timer->TC_CHANNEL[0].TC_RA = 0x7fff;
	timer->TC_CHANNEL[0].TC_RC = 0;
}

void
start_same70_time_clock (io_t *io) {
	same70_time_clock_t *time_clock = &((io_same70_cpu_t*) io)->tc;
	Tc *timer = time_clock->timer;
	
	time_clock->io = io;
	initialise_io_event (&time_clock->alarm,process_alarm_queue,time_clock);
	
	if (
			io_cpu_clock_start (io,time_clock->channel_clock[0])
		&&	io_cpu_clock_start (io,time_clock->channel_clock[1])
		&&	io_cpu_clock_start (io,time_clock->channel_clock[2])
		&&	io_cpu_clock_start (io,time_clock->reference_clock)
	) {
		time_clock->reference_frequency = 1000000000.0 / io_cpu_clock_get_current_frequency (
			time_clock->reference_clock
		);
		
		timer->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
		timer->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKDIS;
		timer->TC_CHANNEL[2].TC_CCR = TC_CCR_CLKDIS;
		
		tc_disable_write_protection (timer);
		
		same70_time_clock_apply_timer_settings (timer);
		same70_time_clock_initialise_chain (timer);

		timer->TC_BMR = (
				0
			|	TC_BMR_TC0XC0S_TIOA2	// connection to XC0 not used
			|	TC_BMR_TC1XC1S_TIOA0	// XC1 connected to CH[0] A output
			|	TC_BMR_TC2XC2S_TIOA1	// XC2 connected to CH[1] A output
		);
		
		// 	if (io_cpu_clock_is_pck_clock (time_clock->reference_clock) 
		// && same70_pck_clock_get_clock_number(time_clock->reference_clock) == 6|7
	
		tc_enable_write_protection (timer);
		
		int32_t irqn = time_clock->interrupt_number + 2;
		register_io_interrupt_handler (
			io,irqn,same70_time_clock_counter_overflow,time_clock
		);
		NVIC_SetPriority (irqn,HIGHEST_INTERRUPT_PRIORITY);
		NVIC_ClearPendingIRQ (irqn);
		NVIC_EnableIRQ (irqn);

		irqn = time_clock->interrupt_number;
		register_io_interrupt_handler (
			io,irqn,same70_time_clock_compare_interrupt,time_clock
		);
		NVIC_SetPriority (irqn,HIGH_INTERRUPT_PRIORITY);
		NVIC_ClearPendingIRQ (irqn);
		NVIC_EnableIRQ (irqn);

		timer->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;
		timer->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG;
	}
}

io_time_t
same70_get_time (io_t *io) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;
	return same70_time_clock_get_time (&this->tc);
}

void
same70_enqueue_alarm (io_t *io,io_alarm_t *alarm) {
	same70_time_clock_t *this = &((io_same70_cpu_t*) io)->tc;
	
	ENTER_CRITICAL_SECTION(io);

	if (alarm->when.nanoseconds < this->io->alarms->when.nanoseconds) {
		alarm->next_alarm = this->io->alarms;
		this->io->alarms = alarm;
		if (!set_64bit_time_clock_alarm_time (this)) {
			io_panic (io,0);
		}
	} else {
		io_alarm_t *pre = this->io->alarms;
		while (alarm->when.nanoseconds > pre->when.nanoseconds) {
			if (pre->next_alarm == &s_null_io_alarm) {
				break;
			}
			pre = pre->next_alarm;
		}
		alarm->next_alarm = pre->next_alarm;
		pre->next_alarm = alarm;
	}

	EXIT_CRITICAL_SECTION(io);
}

void
same70_dequeue_alarm (io_t *io,io_alarm_t *alarm) {
	if (alarm->next_alarm != NULL) {
		ENTER_CRITICAL_SECTION (io);
		if (alarm == io->alarms) {
			same70_time_clock_t *this = &((io_same70_cpu_t*) io)->tc;
			io->alarms = io->alarms->next_alarm;
			set_64bit_time_clock_alarm_time (this);
		} else {
			io_alarm_t *pre = io->alarms;
			while (pre) {
				if (alarm == pre->next_alarm) {
					pre->next_alarm = alarm->next_alarm;
					break;
				}
				pre = pre->next_alarm;
			}
		}
		alarm->next_alarm = NULL;
		EXIT_CRITICAL_SECTION (io);
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



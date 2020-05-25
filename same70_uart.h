/*
 *
 * uart socket 
 *
 */
#ifndef same70_uart_H_
#define same70_uart_H_

typedef struct PACK_STRUCTURE same70_uart {
	IO_SOCKET_STRUCT_MEMBERS

	io_encoding_implementation_t const *encoding;
	io_cpu_clock_pointer_t peripheral_clock;
	Uart *uart_registers;

	io_encoding_pipe_t *tx_pipe;
	stm32_io_dma_channel_t tx_dma_channel;
	
	io_byte_pipe_t *rx_pipe;
	
	same70_io_pin_t tx_pin;
	same70_io_pin_t rx_pin;

	IRQn_Type interrupt_number;
	uint32_t baud_rate;

} same70_uart_t;

extern EVENT_DATA io_socket_implementation_t same70_uart_implementation;

//
// Pins
//

#define UART0_rx_pin		def_same70_io_peripheral_A_input_pin(SAME70_PIOA,9,IO_PIN_ACTIVE_LOW,IO_PIN_INACTIVE)
#define UART0_tx_pin		def_same70_io_peripheral_A_output_pin(SAME70_PIOA,10,IO_PIN_ACTIVE_LOW,IO_PIN_INACTIVE)


#define UART_THR_OFFSET		0x1c


#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------

INLINE_FUNCTION void
uart_disable_write_protection (Uart *this) {
	this->UART_WPMR = (
			0
		|	UART_WPMR_WPKEY_PASSWD
	);
}

INLINE_FUNCTION void
uart_enable_write_protection (Uart *this) {
	this->UART_WPMR = (
			1
		|	UART_WPMR_WPKEY_PASSWD
	);
}

static void	same70_uart_tx_dma_complete (io_event_t *ev);
static void	same70_uart_tx_dma_error (io_event_t *ev);

static void
same70_uart_interrupt (void *user_value) {
}

static io_socket_t*
same70_uart_initialise (io_socket_t *socket,io_t *io,io_settings_t const *C) {
	same70_uart_t *this = (same70_uart_t*) socket;

	this->io = io;
	this->encoding = C->encoding;
	
	this->tx_pipe = mk_io_encoding_pipe (
		io_get_byte_memory(io),io_settings_transmit_pipe_length(C)
	);

	this->rx_pipe = mk_io_byte_pipe (
		io_get_byte_memory(io),io_settings_receive_pipe_length(C)
	);
	
	initialise_io_event (
		&this->tx_dma_channel.complete,same70_uart_tx_dma_complete,this
	);
	initialise_io_event (
		&this->tx_dma_channel.error,same70_uart_tx_dma_error,this
	);

	register_io_interrupt_handler (
		io,this->interrupt_number,same70_uart_interrupt,this
	);
	
	same70_register_dma_channel (io,(io_dma_channel_t*) &this->tx_dma_channel);
	
	return (io_socket_t*) this;
}

static void
same70_uart_set_baud_rate (same70_uart_t *this) {
	
	io_cpu_clock_start (this->io,this->peripheral_clock);
	
	if (same70_clock_is_derrived_from_pcr (this->peripheral_clock)) {
		this->uart_registers->UART_MR = (
				UART_MR_FILTER_DISABLED
			|	UART_MR_CHMODE_NORMAL
			|	UART_MR_PAR_NO
			|	UART_MR_BRSRCCK_PERIPH_CLK
		);
	} else if (same70_clock_is_derrived_from_pck (this->peripheral_clock)) {
		this->uart_registers->UART_MR = (
				UART_MR_FILTER_DISABLED
			|	UART_MR_CHMODE_NORMAL
			|	UART_MR_PAR_NO
			|	UART_MR_BRSRCCK_PMC_PCK
		);
	} else {
		// panic
	}

	float64_t f = io_cpu_clock_get_current_frequency (this->peripheral_clock);
	this->uart_registers->UART_BRGR = (uint32_t) round (
		f / ((float64_t) this->baud_rate * 16.0)
	);

}

INLINE_FUNCTION bool
same70_uart_is_tx_ready (Uart* uart) {
	return (uart->UART_SR & UART_SR_TXRDY) != 0;
}

void
same70_uart_put_char (Uart* uart, uint8_t c) {
	while (!same70_uart_is_tx_ready (uart)) {}
	uart->UART_THR = c;
}

static bool
same70_uart_open (io_socket_t *socket,io_socket_open_flag_t flag) {
	same70_uart_t *this = (same70_uart_t*) socket;
	Uart *uart = this->uart_registers;
	
	if ((uart->UART_IMR & UART_IMR_RXRDY) == 0) {
		uart_disable_write_protection (uart);
		
		uart->UART_CR = (
				UART_CR_TXDIS
			|	UART_CR_RXDIS
			|	UART_CR_RSTSTA
			|	UART_CR_RSTRX
			|	UART_CR_RSTTX
		);
		uart->UART_IDR = 0xffffffffUL;

		io_set_pin_to_alternate (this->io,this->rx_pin.io);
		io_set_pin_to_alternate (this->io,this->tx_pin.io);
		
		same70_uart_set_baud_rate (this);

		uart->UART_CR = (
				UART_CR_TXEN
			|	UART_CR_RXEN
		);

		uart_enable_write_protection (uart);
	}
	
	return false;
}

static void
same70_uart_close (io_socket_t *socket) {
	same70_uart_t *this = (same70_uart_t*) socket;
	Uart *uart = this->uart_registers;
	
	while ((uart->UART_SR & UART_SR_TXRDY) != 0);
	uart->UART_CR = (
			UART_CR_TXDIS
		|	UART_CR_RXDIS
	);
	
	// stop clock
	
	release_io_pin (this->io,this->rx_pin.io);
	release_io_pin (this->io,this->tx_pin.io);
}

static io_encoding_t*
same70_uart_new_message (io_socket_t *socket) {
	same70_uart_t *this = (same70_uart_t*) socket;
	return reference_io_encoding (
		new_io_encoding (this->encoding,io_get_byte_memory(this->io))
	);
}

static bool
same70_uart_output_next_buffer (same70_uart_t *this) {
	io_encoding_t *next;
	if (io_encoding_pipe_peek (this->tx_pipe,&next)) {
		const uint8_t *byte,*end;
		io_encoding_get_content (next,&byte,&end);
		io_dma_transfer_to_peripheral (
			(io_dma_channel_t*) &this->tx_dma_channel,byte,end - byte
		);
		return true;
	} else {
		return false;
	}
}

static void
same70_uart_tx_dma_complete (io_event_t *ev) {
	same70_uart_t *this = ev->user_value;
	
	if (!io_encoding_pipe_pop_encoding (this->tx_pipe)) {
		io_panic (this->io,IO_PANIC_SOMETHING_BAD_HAPPENED);
	}
	
	if (
			!same70_uart_output_next_buffer (this)
//		&& io_event_is_valid (io_pipe_event (this->tx_pipe))
	) {
//		io_enqueue_event (this->io,io_pipe_event (this->tx_pipe));
	}
}

static void
same70_uart_tx_dma_error (io_event_t *ev) {
}

static bool
same70_uart_send_message (io_socket_t *socket,io_encoding_t *encoding) {
	if (is_io_binary_encoding (encoding)) {
		same70_uart_t *this = (same70_uart_t*) socket;
		if (io_encoding_pipe_put_encoding (this->tx_pipe,encoding)) {
			if (io_encoding_pipe_count_occupied_slots (this->tx_pipe) == 1) {
				same70_uart_output_next_buffer (this);
			}
			return true;
		} else {
			unreference_io_encoding (encoding);
			return false;
		}
	} else {
		return false;
	}
}

static size_t
same70_uart_mtu (io_socket_t const *socket) {
	return 1024;
}

EVENT_DATA io_socket_implementation_t same70_uart_implementation = {
	SPECIALISE_IO_SOCKET_IMPLEMENTATION (
		&io_physical_socket_implementation
	)
	.initialise = same70_uart_initialise,
	.open = same70_uart_open,
	.close = same70_uart_close,
	.new_message = same70_uart_new_message,
	.send_message = same70_uart_send_message,
	.mtu = same70_uart_mtu,
};

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


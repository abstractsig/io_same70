/*
 *
 * same70 internal flash
 *
 */
#ifndef same70_flash_H_
#define same70_flash_H_

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------

#define SAME70_PERSISTANT_MEMORY_SECTION __attribute__ ((section(".io_config")))

static SAME70_PERSISTANT_MEMORY_SECTION io_persistant_state_t io_state = {
	.first_run_flag = IO_FIRST_RUN_SET,
	.power_cycles = 0,
	.uid = {{0}},
	.secret = {{0}},
	.shared = {{0}},
};

INLINE_FUNCTION void
eefc_disable_write_protection (Efc* eefc) {
	eefc->EEFC_WPMR = (
			0
		|	EEFC_WPMR_WPKEY_PASSWD
	);
}

INLINE_FUNCTION void
eefc_enable_write_protection (Efc* eefc) {
	eefc->EEFC_WPMR = (
			1
		|	EEFC_WPMR_WPKEY_PASSWD
	);
}

void
eefc_set_flash_wait_states (Efc* eefc, uint8_t cycles) {
	eefc_disable_write_protection (eefc);
	eefc->EEFC_FMR = (eefc->EEFC_FMR & ~EEFC_FMR_FWS_Msk) | EEFC_FMR_FWS(cycles);
	eefc_enable_write_protection (eefc);
}

uint32_t
eefc_get_gpnv_bits (void) {
	volatile uint32_t size;
	
	EFC->EEFC_FCR = (
			EEFC_FCR_FCMD_GGPB
		|	EEFC_FCR_FKEY_PASSWD
	);
	
	size = EFC->EEFC_FRR;
	
	return size;
}

uint32_t
eefc_get_flash_page_size (Efc* eefc) {
	volatile uint32_t size;
	
	eefc->EEFC_FCR = (
			EEFC_FCR_FCMD_GETD
		|	EEFC_FCR_FKEY_PASSWD
	);
	
	size = eefc->EEFC_FRR;
	size = eefc->EEFC_FRR;
	size = eefc->EEFC_FRR;
	
	return size;
}

static bool
same70_write_io_persistant_state (io_persistant_state_t *new_state) {
	uint32_t *write_address = (uint32_t*) &io_state;
	uint32_t *data = (uint32_t*) new_state;
	uint32_t page_number = (
		(((uint32_t) &io_state) - IFLASH_ADDR)/IFLASH_PAGE_SIZE
	);
	uint32_t words = io_config_u32_size();
	
	while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) == 0);

	// do this in page-size chunks
	while (words > 0) {
		uint32_t chunk = MIN (128,words);
		words -= chunk;
		
		for (int i = 0; i < chunk; i++) {
			*write_address++ = *data++;
		}

		EFC->EEFC_FCR = (
				EEFC_FCR_FCMD_EWP
			|	EEFC_FCR_FARG (page_number)
			|	EEFC_FCR_FKEY_PASSWD
		);
		while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) == 0);

		EFC->EEFC_FCR = (
				EEFC_FCR_FCMD_WP
			|	EEFC_FCR_FARG (page_number)
			|	EEFC_FCR_FKEY_PASSWD
		);
		while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) == 0);
		
		page_number += 1;
	}
	
	return true;
}

static bool
same70_io_config_clear_first_run (void) {
	if (io_state.first_run_flag == IO_FIRST_RUN_SET) {
		io_persistant_state_t new_ioc = io_state;
		new_ioc.first_run_flag = IO_FIRST_RUN_CLEAR;
		same70_write_io_persistant_state (&new_ioc);
		return true;
	} else {
		return false;
	}
}

static bool
same70_io_config_is_first_run (void) {
	bool first = (io_state.first_run_flag == IO_FIRST_RUN_SET);
	same70_io_config_clear_first_run ();
	return first;
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



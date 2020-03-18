/*
 *
 * same70 has a single, 24 channel dma controller
 *
 */
#ifndef same70_dma_H_
#define same70_dma_H_
#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------

static void
same70_xdmac_interurpt (void *user_value) {
	io_same70_cpu_t *this = user_value;
	io_dma_channel_t *channel = this->dma_channel_list;
	
	while (channel) {
		volatile uint32_t status = XDMAC->XDMAC_CHID[0].XDMAC_CIS;	// clears pend
		
		if ((status & XDMAC_CIS_BIS) != 0) {
			io_enqueue_event (user_value,&channel->complete);
		} else {
			// anything else is an error
			//while(22);
			//io_enqueue_event (user_value,&channel->error);
		}
		
		channel = channel->next_channel;
	}
}

bool
initialise_same70_xdmac (io_t *io) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;

	io_cpu_clock_start (io,this->dma_clock);
	
	register_io_interrupt_handler (
		io,XDMAC_IRQn,same70_xdmac_interurpt,io
	);

	NVIC_SetPriority (XDMAC_IRQn,NORMAL_INTERRUPT_PRIORITY);
	NVIC_ClearPendingIRQ (XDMAC_IRQn);
	NVIC_EnableIRQ (XDMAC_IRQn);

	return true;
}

void
same70_register_dma_channel (io_t *io,io_dma_channel_t *channel) {
	io_same70_cpu_t *this = (io_same70_cpu_t*) io;
	if (channel->next_channel == NULL) {
		channel->next_channel = this->dma_channel_list;
		this->dma_channel_list = channel;
	}
}

static void
same70_io_dma_transfer_from_peripheral (
	io_dma_channel_t *channel,void *dest,uint32_t length
) {
	stm32_io_dma_channel_t *sch = (stm32_io_dma_channel_t*) channel;
	XdmacChid *ch = XDMAC->XDMAC_CHID + stm32_io_dma_channel_number(sch);
	
	ch->XDMAC_CSA = stm32_io_dma_channel_peripheral_address(sch);
	ch->XDMAC_CDA = (uint32_t) dest;
	
	ch->XDMAC_CUBC = length;
	
	ch->XDMAC_CC |= (
			XDMAC_CC_TYPE_PER_TRAN
		|	XDMAC_CC_MBSIZE_SINGLE
		|	XDMAC_CC_SAM_FIXED_AM
		|	XDMAC_CC_DAM_INCREMENTED_AM
		|	XDMAC_CC_DSYNC_PER2MEM
		|	XDMAC_CC_CSIZE_CHK_1
		|	XDMAC_CC_DWIDTH_BYTE
		|	XDMAC_CC_SIF_AHB_IF1
		|	XDMAC_CC_DIF_AHB_IF1
		|	XDMAC_CC_PERID(stm32_io_dma_peripheral_id(sch))
		|	XDMAC_CC_SWREQ_HWR_CONNECTED
	);

	ch->XDMAC_CNDC = 0;
	ch->XDMAC_CBC = 0;
	ch->XDMAC_CDS_MSP = 0;
	ch->XDMAC_CSUS = 0;
	ch->XDMAC_CDUS = 0;
	
	ch->XDMAC_CIE |= XDMAC_CIE_BIE;
	
	XDMAC->XDMAC_GE |= (1 << stm32_io_dma_channel_number(sch)); //XDMAC_GS.STx is cleared by hardware
	
}

static void
same70_io_dma_transfer_to_peripheral (
	io_dma_channel_t *channel,void const *src,uint32_t size
) {
	stm32_io_dma_channel_t *sch = (stm32_io_dma_channel_t*) channel;
	XdmacChid *ch = XDMAC->XDMAC_CHID + stm32_io_dma_channel_number(sch);
	
	ch->XDMAC_CDA = stm32_io_dma_channel_peripheral_address(sch);
	ch->XDMAC_CSA = (uint32_t) src;
	
	ch->XDMAC_CUBC = size;
	
	ch->XDMAC_CC |= (
			XDMAC_CC_TYPE_PER_TRAN
		|	XDMAC_CC_MBSIZE_SINGLE
		|	XDMAC_CC_DAM_FIXED_AM
		|	XDMAC_CC_SAM_INCREMENTED_AM
		|	XDMAC_CC_DSYNC_MEM2PER
		|	XDMAC_CC_CSIZE_CHK_1
		|	XDMAC_CC_DWIDTH_BYTE
		|	XDMAC_CC_SIF_AHB_IF1
		|	XDMAC_CC_DIF_AHB_IF1
		|	XDMAC_CC_PERID(stm32_io_dma_peripheral_id(sch))
		|	XDMAC_CC_SWREQ_HWR_CONNECTED
	);

	ch->XDMAC_CNDC = 0;
	ch->XDMAC_CBC = 0;
	ch->XDMAC_CDS_MSP = 0;
	ch->XDMAC_CSUS = 0;
	ch->XDMAC_CDUS = 0;
	
	ch->XDMAC_CIE |= XDMAC_CIE_BIE;
	XDMAC->XDMAC_GIE |= (1 << stm32_io_dma_channel_number(sch));
	XDMAC->XDMAC_GE |= (1 << stm32_io_dma_channel_number(sch));
}

EVENT_DATA io_dma_channel_implementation_t same70_dma_channel_implementation = {
	.transfer_from_peripheral = same70_io_dma_transfer_from_peripheral,
	.transfer_to_peripheral = same70_io_dma_transfer_to_peripheral,
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

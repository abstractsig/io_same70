/*
 * 
 * dma for the STM32L4xx: need to shift to stm32 architecture
 *
 */
#include <architecture/peripherals.h>


void		event__dma1_interrupt_handler(t_interrupt*);
void		event__dma2_interrupt_handler(t_interrupt*);


/*
 *-----------------------------------------------------------------------------
 *
 * attach_dma_channel --
 *
 * Do all the stuff to make this channel go.
 *
 *-----------------------------------------------------------------------------
 */
TASK_FUNCTION void
attach_dma_channel (t_dma_channel *dma) {
	DMA_Channel_TypeDef *dma_channel = stm32_dma_channel_registers(dma);
	t_stm32_dma_channel *channel = (t_stm32_dma_channel*) dma;
	uint32_t temp_config;

	switch (stm32_dma_channel_block(channel)) {
		case 1:
			__HAL_RCC_DMA1_CLK_ENABLE();
			
			modify_cpu_register_value (
				DMA1_CSELR->CSELR,
				5,
				(stm32_dma_channel_number(channel) - 1) * 4,
				stm32_dma_channel_peripheral(channel)
			);

			channel->call = event__dma1_interrupt_handler;

		break;
		
		case 2:
			__HAL_RCC_DMA2_CLK_ENABLE();

			modify_cpu_register_value (
				DMA2_CSELR->CSELR,
				5,
				(stm32_dma_channel_number(channel) - 1) * 4,
				stm32_dma_channel_peripheral(channel)
			);

			channel->call = event__dma2_interrupt_handler;

		break;
		
		default:
			nib_panic("invalid dma block");
		break;
	}
	
	temp_config = dma_channel->CCR;
	modify_cpu_register_value (
		temp_config,2,12,stm32_dma_channel_priority(channel)
	);
	modify_cpu_register_value (
		temp_config,1,4,stm32_dma_channel_direction(channel)
	);

	modify_cpu_register_value (temp_config,1,14,0);		// not mem-to-mem
	modify_cpu_register_value (temp_config,2,10,0);		// mem size 8 bit
	modify_cpu_register_value (temp_config,2,8,0);		// periph size8 bit
	modify_cpu_register_value (temp_config,1,7,1);		// memory increment
	modify_cpu_register_value (temp_config,1,6,0);		// periph increment
	modify_cpu_register_value (temp_config,1,5,0);		// not circular	
	modify_cpu_register_value (temp_config,1,3,0);		// transfer error interrupt
	modify_cpu_register_value (temp_config,1,2,0);		// half transfer interrupt
	modify_cpu_register_value (temp_config,1,1,1);		// transfer complete interrupt
	modify_cpu_register_value (temp_config,1,0,0);		// enable
	dma_channel->CCR = temp_config;	

	dma_channel->CNDTR = 0;	
	dma_channel->CPAR = stm32_dma_channel_peripheral_address(channel);	
	dma_channel->CMAR = 0;	

	set_nib_interrupt_handler_by_index (
		stm32_dma_channel_nib_interrupt(channel),
		(t_interrupt*) channel
	);
}

/*
 *-----------------------------------------------------------------------------
 *
 * detach_dma_channel --
 *
 * Unset stuff to free this channel's resources
 *
 *-----------------------------------------------------------------------------
 */
TASK_FUNCTION void
detach_dma_channel (t_dma_channel *channel) {
	unset_nib_interrupt_handler_by_index (
		stm32_dma_channel_nib_interrupt((t_stm32_dma_channel*)channel),
		(t_interrupt*) channel
	);
	dma_interrupt_disable (channel);
}

/*
 *-----------------------------------------------------------------------------
 *
 * dma_write_transfer --
 *
 * Start a transfer.
 *
 * Arguments
 * =========
 * this			dma channel
 * data			data source
 * size			length of data array (max = (1 << 16) - 1)
 *
 *-----------------------------------------------------------------------------
 */
EVENT_FUNCTION void
dma_write_transfer (t_dma_channel *this,uint8_t const *data,uint32_t size) {
	DMA_Channel_TypeDef *dma_channel = stm32_dma_channel_registers(this);
	dma_channel->CMAR = (uint32_t) data;
	dma_channel->CNDTR = size;
	dma_channel->CCR |= 1;
}

/*
 *-----------------------------------------------------------------------------
 *
 * dma_read_transfer --
 *
 * Start a transfer.
 *
 * Arguments
 * =========
 * this			dma channel
 * data			location to store data
 * size			length of data array (max = (1 << 16) - 1)
 *
 *-----------------------------------------------------------------------------
 */
EVENT_FUNCTION void
dma_read_transfer(t_dma_channel *dma,uint8_t *data,uint32_t size) {
	DMA_Channel_TypeDef *dma_channel = stm32_dma_channel_registers(dma);
	dma_channel->CMAR = (uint32_t) data;
	dma_channel->CNDTR = size;	
	dma_channel->CCR |= DMA_CCR_EN;
}

/*
 *-----------------------------------------------------------------------------
 *
 * dma_read_transfer --
 *
 * Start a transfer.
 *
 * Arguments
 * =========
 * this			dma channel
 * data			location to store data
 * size			length of data array (max = (1 << 16) - 1)
 *
 *-----------------------------------------------------------------------------
 */
EVENT_FUNCTION uint32_t
dma_stop_transfer(t_dma_channel *dma) {
	DMA_Channel_TypeDef *dma_channel = stm32_dma_channel_registers(dma);
	uint32_t count = dma_channel->CNDTR;
	dma_channel->CCR &= ~DMA_CCR_EN;
	return count;
}

/*
 *-----------------------------------------------------------------------------
 *
 * event__dma1_interrupt_handler --
 *
 *  does the interrupt match the dma channel?
 *
 *-----------------------------------------------------------------------------
 */
EVENT_FUNCTION void
event__dma1_interrupt_handler (t_interrupt *irh) {
	t_stm32_dma_channel *channel = (t_stm32_dma_channel*) irh;
	uint32_t status = DMA1->ISR >> ((stm32_dma_channel_number(channel) - 1) * 4);

	DMA1->IFCR = 1 << ((stm32_dma_channel_number(channel) - 1) * 4);
	
	if (status & 0x08) {
		// transfer error
		event__from_hardware(channel->error);
	}

	if (status & 0x04) {
		// half transfer complete
	}


	if (status & 0x02) {
		// transfer complete
		event__from_hardware(channel->complete);
	}
}

/*
 *-----------------------------------------------------------------------------
 *
 * event__dma2_interrupt_handler --
 *
 * 
 *
 *-----------------------------------------------------------------------------
 */
EVENT_FUNCTION void
event__dma2_interrupt_handler(t_interrupt *irh) {
	t_stm32_dma_channel *chan = (t_stm32_dma_channel*) irh;
	uint32_t status = (DMA2->ISR >> ((stm32_dma_channel_number(chan) - 1) * 4));
	
	if (status & 0xf) {
		DMA2->IFCR = 0xf << ((stm32_dma_channel_number(chan) - 1) * 4);
	} else {
		// not me
	}
	
}

/*
 *-----------------------------------------------------------------------------
 *
 * dma_interrupt_enable --
 *
 *-----------------------------------------------------------------------------
 */
EVENT_FUNCTION void
dma_interrupt_enable (t_dma_channel *chan) {

	int32_t irqn = stm32_dma_channel_interrupt((t_stm32_dma_channel*) chan);

	NVIC_ClearPendingIRQ(irqn);
	NVIC_SetPriority(irqn,NORMAL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(irqn);

}

/*
 *-----------------------------------------------------------------------------
 *
 * dma_interrupt_disable --
 *
 *-----------------------------------------------------------------------------
 */
EVENT_FUNCTION void
dma_interrupt_disable (t_dma_channel *chan) {
	NVIC_DisableIRQ (
		stm32_dma_channel_interrupt((t_stm32_dma_channel*) chan)
	);
}

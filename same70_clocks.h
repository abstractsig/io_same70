/*
 *
 * Cpu clocks and power manager
 *
 */
#ifndef same70_clocks_H_
#define same70_clocks_H_

//
// clocks
//
// see Figure 31-1 in PMC section of TRM for pictute
//
typedef struct PACK_STRUCTURE same70_main_rc_oscillator {
	IO_CPU_CLOCK_SOURCE_STRUCT_MEMBERS
	uint32_t frequency;
} same70_main_rc_oscillator_t;

typedef struct PACK_STRUCTURE same70_main_crystal_oscillator {
	IO_CPU_CLOCK_SOURCE_STRUCT_MEMBERS
	float64_t crystal_frequency;
	uint8_t startup_time;
} same70_main_crystal_oscillator_t;

typedef struct PACK_STRUCTURE same70_main_clock {
	IO_CPU_CLOCK_FUNCTION_STRUCT_MEMBERS
	float64_t divisor;
} same70_main_clock_t;

typedef struct PACK_STRUCTURE same70_master_clock_controller {
	IO_CPU_CLOCK_FUNCTION_STRUCT_MEMBERS
	uint32_t prescaler;
//	uint32_t divisor;
} same70_master_clock_controller_t;

typedef struct PACK_STRUCTURE same70_master_clock_divider {
	IO_CPU_CLOCK_FUNCTION_STRUCT_MEMBERS
	uint32_t divisor;
} same70_master_clock_divider_t;

typedef struct PACK_STRUCTURE same70_plla_clock {
	IO_CPU_CLOCK_FUNCTION_STRUCT_MEMBERS
	float64_t output_frequency;
} same70_plla_clock_t;

typedef struct PACK_STRUCTURE same70_processor_clock {
	IO_CPU_DEPENDANT_CLOCK_STRUCT_MEMBERS
} same70_processor_clock_t;

// PCR clock
typedef struct PACK_STRUCTURE same70_pcr_clock {
	IO_CPU_CLOCK_FUNCTION_STRUCT_MEMBERS
} same70_pcr_clock_t;

// PCK clock
typedef struct PACK_STRUCTURE same70_pck_clock {
	IO_CPU_CLOCK_FUNCTION_STRUCT_MEMBERS
	uint32_t clock_number;
	uint32_t divisor;
} same70_pck_clock_t;

uint32_t same70_pck_clock_get_clock_number(io_cpu_clock_pointer_t);

typedef struct PACK_STRUCTURE same70_peripheral_clock {
	IO_CPU_DEPENDANT_CLOCK_STRUCT_MEMBERS
	float64_t frequency;
	uint32_t peripheral_id;
} same70_peripheral_clock_t;

extern EVENT_DATA io_cpu_clock_implementation_t same70_main_rc_oscillator_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t same70_main_crystal_oscillator_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t same70_main_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t same70_master_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t same70_master_clock_divider_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t same70_plla_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t same70_pcr_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t same70_pck_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t same70_processor_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t same70_pcr_peripheral_clock_implementation;

INLINE_FUNCTION bool
io_cpu_clock_is_main_rc_oscillator (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_main_rc_oscillator_implementation);
}

INLINE_FUNCTION bool
io_cpu_clock_is_main_crystal_oscillator (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_main_crystal_oscillator_implementation);
}

INLINE_FUNCTION bool
io_cpu_clock_is_main_clock (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_main_clock_implementation);
}

INLINE_FUNCTION bool
io_cpu_clock_is_plla_clock (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_plla_clock_implementation);
}

INLINE_FUNCTION bool
io_cpu_clock_is_master_clock_controller (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_master_clock_implementation);
}

INLINE_FUNCTION bool
io_cpu_clock_is_master_clock_divider (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_master_clock_divider_implementation);
}

INLINE_FUNCTION bool
io_cpu_clock_is_pcr_clock (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_pcr_clock_implementation);
}

INLINE_FUNCTION bool
io_cpu_clock_is_pck_clock (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_pck_clock_implementation);
}

INLINE_FUNCTION bool
io_cpu_clock_is_pcr_peripheral_clock (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_pcr_peripheral_clock_implementation);
}

INLINE_FUNCTION bool
same70_clock_is_derrived_from_pcr (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_is_derrived_from (clock,&same70_pcr_clock_implementation);
}

INLINE_FUNCTION bool
same70_clock_is_derrived_from_pck (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_is_derrived_from (clock,&same70_pck_clock_implementation);
}

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------
void
pmc_osc_enable_main_xtal (uint32_t ul_xtal_startup_time) {
	uint32_t mor = PMC->CKGR_MOR;
	mor &= ~(
			CKGR_MOR_MOSCXTBY
		|	CKGR_MOR_MOSCXTEN
	);
	mor |= (
			CKGR_MOR_KEY_PASSWD 
		|	CKGR_MOR_MOSCXTEN 
		|	CKGR_MOR_MOSCXTST(ul_xtal_startup_time)
	);
	PMC->CKGR_MOR = mor;
	
	// Wait the main Xtal to stabilize 
	while (!(PMC->PMC_SR & PMC_SR_MOSCXTS));
}

INLINE_FUNCTION void
pmc_disable_write_protection (void) {
	PMC->PMC_WPMR = (
			0
		|	PMC_WPMR_WPKEY_PASSWD
	);
}

INLINE_FUNCTION void
pmc_enable_write_protection (void) {
	PMC->PMC_WPMR = (
			1
		|	PMC_WPMR_WPKEY_PASSWD
	);
}

INLINE_FUNCTION void
pmc_disable_all_programmable_clocks (void) {
	PMC->PMC_SCDR = PMC_SCDR_PCK0 | PMC_SCDR_PCK1 | PMC_SCDR_PCK2;
}

/** Bit mask for peripheral clocks (PCER0) */
#define PMC_MASK_STATUS0        (0xFFFFFFFC)

/** Bit mask for peripheral clocks (PCER1) */
#define PMC_MASK_STATUS1        (0xFFFFFFFF)

uint32_t
pmc_disable_periph_clk(uint32_t ul_id) {
#if defined(REG_PMC_PCR) && !SAMG55
	uint32_t pcr;
	PMC->PMC_PCR = ul_id & 0x7F;
	pcr = PMC->PMC_PCR | PMC_PCR_CMD;
	PMC->PMC_PCR = pcr;
	return 0;
#else
	if (ul_id > MAX_PERIPHERAL_ID) {
		return 1;
	}

	if (ul_id < 32) {
		if ((PMC->PMC_PCSR0 & (1u << ul_id)) == (1u << ul_id)) {
			PMC->PMC_PCDR0 = 1 << ul_id;
		}
#if (SAM3S || SAM3XA || SAM4S || SAM4E || SAM4C || SAM4CM || SAM4CP || SAMG55 || SAMV71 \
		|| SAMV70 || SAME70 || SAMS70)
	} else {
		ul_id -= 32;
		if ((PMC->PMC_PCSR1 & (1u << ul_id)) == (1u << ul_id)) {
			PMC->PMC_PCDR1 = 1 << ul_id;
		}
#endif
	}
	return 0;
#endif /* defined(REG_PMC_PCR) && !SAMG55 */
}

void
pmc_disable_all_peripheral_clocks (void)
{
	PMC->PMC_PCDR0 = PMC_MASK_STATUS0;
	while ((PMC->PMC_PCSR0 & PMC_MASK_STATUS0) != 0);

	PMC->PMC_PCDR1 = PMC_MASK_STATUS1;
	while ((PMC->PMC_PCSR1 & PMC_MASK_STATUS1) != 0);

	for (uint32_t id = 64; id <= 0x7F; id ++) {
		pmc_disable_periph_clk(id);
	}
}

#define PMC_TIMEOUT             (4096)

uint32_t
pmc_switch_mck_to_pllack (uint32_t ul_pres) {
	uint32_t ul_timeout;

	PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
	for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
			--ul_timeout) {
		if (ul_timeout == 0) {
			return 1;
		}
	}

	PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
			PMC_MCKR_CSS_PLLA_CLK;

	for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
			--ul_timeout) {
		if (ul_timeout == 0) {
			return 1;
		}
	}

	return 0;
}

void
pmc_set_mck_divider (uint32_t divider) {
	PMC->PMC_MCKR |= (
			(PMC->PMC_MCKR & ~PMC_MCKR_MDIV_Msk) 
		|	((divider << PMC_MCKR_MDIV_Pos) & PMC_MCKR_MDIV_Msk)
	);
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
}

void
pmc_set_mck_prescaler (uint32_t prescaler) {
	PMC->PMC_MCKR |= (
			(PMC->PMC_MCKR & ~PMC_MCKR_PRES_Msk) 
		|	((prescaler << PMC_MCKR_PRES_Pos) & PMC_MCKR_PRES_Msk) 
	);
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
}


static float64_t
same70_main_rc_oscillator_get_current_frequency (io_cpu_clock_pointer_t clock) {
	same70_main_rc_oscillator_t const *this = (same70_main_rc_oscillator_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);

	// 0.0 if not running !!
	
	return this->frequency;
}

static float64_t
same70_main_rc_oscillator_get_expected_frequency (io_cpu_clock_pointer_t clock) {
	same70_main_rc_oscillator_t const *this = (same70_main_rc_oscillator_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	return this->frequency;
}

static bool
same70_main_rc_oscillator_start (io_t *io,io_cpu_clock_pointer_t clock) {
	same70_main_rc_oscillator_t const *this = (same70_main_rc_oscillator_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	uint32_t select = 0;
	
	switch (this->frequency) {
		case 4000000:
			select = CKGR_MOR_MOSCRCF_4_MHz;
		break;
		
		case 8000000:
			select = CKGR_MOR_MOSCRCF_8_MHz;
		break;
		
		case 12000000:
			select = CKGR_MOR_MOSCRCF_12_MHz;
		break;
		
		default:
			return false;
	}
	UNUSED(select);
	
	if (select != (PMC->CKGR_MOR & CKGR_MOR_MOSCRCF_Msk)) {
		uint32_t mor = PMC->CKGR_MOR;
		mor &= ~CKGR_MOR_MOSCRCF_Msk;
		mor |= select;
		
		pmc_disable_write_protection ();
		PMC->CKGR_MOR = (
				mor
			|	CKGR_MOR_KEY_PASSWD
		);
		pmc_enable_write_protection ();
		return true;
	}
	
	return true;
}

EVENT_DATA io_cpu_clock_implementation_t same70_main_rc_oscillator_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = same70_main_rc_oscillator_get_current_frequency,
	.get_expected_frequency = same70_main_rc_oscillator_get_expected_frequency,
	.get_input = io_cpu_clock_get_input_nop,
	.iterate_outputs = io_cpu_clock_iterate_outputs_nop,
	.start = same70_main_rc_oscillator_start,
	.stop = NULL,
};

bool
same70_clock_is_main_rc (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&same70_main_rc_oscillator_implementation);
}

static float64_t
same70_main_crystal_oscillator_get_current_frequency (io_cpu_clock_pointer_t this) {
	same70_main_crystal_oscillator_t const *c = (same70_main_crystal_oscillator_t const*) (
		io_cpu_clock_ro_pointer (this)
	);
	// !! 0.0 if not running
	return c->crystal_frequency;
}

static float64_t
same70_main_crystal_oscillator_get_expected_frequency (io_cpu_clock_pointer_t this) {
	same70_main_crystal_oscillator_t const *c = (same70_main_crystal_oscillator_t const*) (
		io_cpu_clock_ro_pointer (this)
	);
	return c->crystal_frequency;
}

static bool
same70_main_crystal_oscillator_start (io_t *io,io_cpu_clock_pointer_t clock) {
	if ((PMC->CKGR_MOR & CKGR_MOR_MOSCXTEN) == 0) {
		same70_main_crystal_oscillator_t const *this =  (
			(same70_main_crystal_oscillator_t const*) (
				io_cpu_clock_ro_pointer (clock)
			)
		);
		pmc_osc_enable_main_xtal (this->startup_time);
		return true;
	} else {
		return true;
	}
}

EVENT_DATA io_cpu_clock_implementation_t same70_main_crystal_oscillator_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = same70_main_crystal_oscillator_get_current_frequency,
	.get_expected_frequency = same70_main_crystal_oscillator_get_expected_frequency,
	.get_input = io_cpu_clock_get_input_nop,
	.iterate_outputs = io_cpu_clock_iterate_outputs_nop,
	.start = same70_main_crystal_oscillator_start,
	.stop = NULL,
};

static float64_t
same70_main_clock_get_current_frequency (io_cpu_clock_pointer_t clock) {
	same70_main_clock_t const *this = (same70_main_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	return io_cpu_clock_get_current_frequency (this->input)/this->divisor;
}

static float64_t
same70_main_clock_get_expected_frequency (io_cpu_clock_pointer_t clock) {
	same70_main_clock_t const *this = (same70_main_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	return io_cpu_clock_get_expected_frequency (this->input)/this->divisor;
}

//
// main clock (or main oscillator) selects either crystal or rc oscillator
//
static bool
same70_main_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	io_cpu_dependant_clock_t const *this = (io_cpu_dependant_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	if (io_cpu_clock_start (io,this->input)) {
		if (io_cpu_clock_is_main_crystal_oscillator (this->input)) {
			if ((PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) == 0) {
				pmc_disable_write_protection ();
				PMC->CKGR_MOR |= (
						CKGR_MOR_MOSCSEL
					|	CKGR_MOR_KEY_PASSWD
				);
				
				while ( (PMC->PMC_SR & PMC_SR_MOSCSELS) == 0);
				while ((PMC->CKGR_MCFR & CKGR_MCFR_MAINFRDY) == 0);
				
				//
				// check main clock frequency = (PMC->CKGR_MCFR & CKGR_MCFR_MAINF_Msk)
				// must be > 0
				//

				if ( (PMC->CKGR_MCFR & CKGR_MCFR_MAINF_Msk) > 0) {
					pmc_enable_write_protection ();
					return true;
				} else {
					uint32_t mor = PMC->CKGR_MOR;
					mor &= ~CKGR_MOR_MOSCSEL;					
					PMC->CKGR_MOR = (
							mor
						|	CKGR_MOR_KEY_PASSWD
					);
					pmc_enable_write_protection ();
					return false;
				}
			} else {
				return true;
			}
		} else if (io_cpu_clock_is_main_rc_oscillator (this->input)) {
			if ((PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) != 0) {
				uint32_t mor = PMC->CKGR_MOR;
				pmc_disable_write_protection ();
				mor &= ~CKGR_MOR_MOSCSEL;					
				PMC->CKGR_MOR = (
						mor
					|	CKGR_MOR_KEY_PASSWD
				);
				pmc_enable_write_protection ();
				return true;
			} else {
				return true;
			}
		} else {
			return false;
		}
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t same70_main_clock_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = same70_main_clock_get_current_frequency,
	.get_expected_frequency = same70_main_clock_get_expected_frequency,
	.get_input = io_cpu_dependant_clock_get_input,
	.iterate_outputs = io_cpu_clock_iterate_outputs_nop,
	.start = same70_main_clock_start,
	.stop = NULL,
};

static float64_t
same70_plla_clock_get_current_frequency (io_cpu_clock_pointer_t clock) {
	same70_plla_clock_t const *this = (same70_plla_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	return (
			io_cpu_clock_get_current_frequency (this->input)
		*	(float64_t) ((PMC->CKGR_PLLAR & CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos)
		/	(float64_t) ((PMC->CKGR_PLLAR & CKGR_PLLAR_DIVA_Msk) >> CKGR_PLLAR_DIVA_Pos)
	);
}

static float64_t
same70_plla_clock_get_expected_frequency (io_cpu_clock_pointer_t clock) {
	same70_plla_clock_t const *this = (same70_plla_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	return this->output_frequency;
}

//
// input frequency [8MHz.. 32MHz]
// output frequency [160MHz .. 500 MHz]
//
static bool
same70_plla_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	same70_plla_clock_t const *this = (same70_plla_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	
	if (
			io_math_compare_float64_le (
				this->output_frequency,SAME70_MAXIMUM_CLOCK_FREQUENCY
			)
		&& 
			io_math_compare_float64_ge (
				this->output_frequency,160000000.0
			)
	) {
		if (
				io_cpu_clock_is_main_clock (this->input)
			&&	io_cpu_clock_start (io,this->input)
		) {
			if ((PMC->CKGR_PLLAR & CKGR_PLLAR_DIVA_Msk) == 0) {
				uint32_t reg = PMC->CKGR_PLLAR;
				uint32_t gcd = gcd_uint32 (
					(uint32_t) this->output_frequency,
					(uint32_t) io_cpu_clock_get_current_frequency (this->input)
				);
				
				uint32_t mul = (uint32_t) this->output_frequency / gcd;
				uint32_t div = (uint32_t) io_cpu_clock_get_current_frequency (this->input) / gcd;
								
				reg &= ~CKGR_PLLAR_MULA_Msk | ~CKGR_PLLAR_DIVA_Msk;
				reg |= (
						((mul << CKGR_PLLAR_MULA_Pos) & CKGR_PLLAR_MULA_Msk)
					|	((div << CKGR_PLLAR_DIVA_Pos) & CKGR_PLLAR_DIVA_Msk)
				);
				
				pmc_disable_write_protection ();
				PMC->CKGR_PLLAR = (
						reg
					|	CKGR_PLLAR_ONE
				);
				pmc_enable_write_protection ();
				
				while ((PMC->PMC_SR & PMC_SR_LOCKA) == 0);
				
			} else {
				// else already going, have parameters changed?
			}
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t same70_plla_clock_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = same70_plla_clock_get_current_frequency,
	.get_expected_frequency = same70_plla_clock_get_expected_frequency,
	.get_input = io_cpu_dependant_clock_get_input,
	.iterate_outputs = io_cpu_clock_iterate_outputs_nop,
	.start = same70_plla_clock_start,
	.stop = NULL,
};

static float64_t
same70_master_clock_get_current_frequency (io_cpu_clock_pointer_t clock) {
	same70_master_clock_controller_t const *this = (
		(same70_master_clock_controller_t const*) (
			io_cpu_clock_ro_pointer (clock)
		)
	);
	uint32_t prescale = (PMC->PMC_MCKR & PMC_MCKR_PRES_Msk) >> PMC_MCKR_PRES_Pos;
	float64_t div = 1.0,f = io_cpu_clock_get_current_frequency (this->input);
	
	switch (prescale) {
		case 0:		break;
		case 1:		div = 2.0; break;
		case 2:		div = 4.0; break;
		case 3:		div = 8.0; break;
		case 4:		div = 16.0; break;
		case 5:		div = 32.0; break;
		case 6:		div = 64.0; break;
		case 7:		div = 128.0; break;

		default:
			f = 1.0;
		break;
	}
	
	return f/div;
}

static float64_t
same70_master_clock_get_expected_frequency (io_cpu_clock_pointer_t clock) {
	same70_master_clock_controller_t const *this = (
		(same70_master_clock_controller_t const*) (
			io_cpu_clock_ro_pointer (clock)
		)
	);
	float64_t div = 1.0;
	switch (this->prescaler) {
		default:
		case 0:		div = 1.0; break;
		case 1:		div = 2.0; break;
		case 2:		div = 4.0; break;
		case 3:		div = 8.0; break;
		case 4:		div = 16.0; break;
		case 5:		div = 32.0; break;
		case 6:		div = 64.0; break;
		case 7:		div = 128.0; break;
	}
	
	return (
			io_cpu_clock_get_expected_frequency (this->input)
		/	div
	);
}

uint8_t
get_flash_wait_states(uint32_t frequency) {
	/* Embedded Flash Wait States for Worst-Case Conditions */
	/* VDDIO 3.0V */
	const uint32_t max_freq[] = {
		23000000, 46000000, 69000000, 92000000, 115000000, 138000000 };
	int i;
	for (i = 0; i < SIZEOF(max_freq); i++)
		if (max_freq[i] >= frequency)
			break;
	return i;
}

static bool
same70_master_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	same70_master_clock_controller_t const *this = (
		(same70_master_clock_controller_t const*) (
			io_cpu_clock_ro_pointer (clock)
		)
	);
	if (io_cpu_clock_start (io,this->input)) {
		uint32_t src = (PMC->PMC_MCKR & PMC_MCKR_CSS_Msk) >> PMC_MCKR_CSS_Pos;
		
		// select source, main clock, slow slock plla clock or upll clock

		if (io_cpu_clock_is_main_clock (this->input)) {
			if (src != PMC_MCKR_CSS_MAIN_CLK) {
				// switch 

				while ((PMC->PMC_SR & PMC_SR_MCKRDY) == 0);
			}
			return true;
		} else if (io_cpu_clock_is_plla_clock (this->input)) {
			if (src != PMC_MCKR_CSS_PLLA_CLK) {

				pmc_disable_write_protection ();
				pmc_set_mck_prescaler (this->prescaler);
				
				// must also set divisor here, find divisor in output
				pmc_set_mck_divider (1);
				
				pmc_switch_mck_to_pllack (PMC_MCKR_PRES(PMC_MCKR_PRES_CLK_1));
				pmc_enable_write_protection ();
			}
			return true;
		} else {
			// not supported yet
			return false;
		}
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t same70_master_clock_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = same70_master_clock_get_current_frequency,
	.get_expected_frequency = same70_master_clock_get_expected_frequency,
	.get_input = io_cpu_dependant_clock_get_input,
	.iterate_outputs = io_cpu_clock_function_iterate_outputs,
	.start = same70_master_clock_start,
	.stop = NULL,
};

static float64_t
same70_master_clock_divider_get_current_frequency (io_cpu_clock_pointer_t clock) {
	same70_master_clock_controller_t const *this = (
		(same70_master_clock_controller_t const*) (
			io_cpu_clock_ro_pointer (clock)
		)
	);
	uint32_t mdiv = ((PMC->PMC_MCKR & PMC_MCKR_MDIV_Msk) >> PMC_MCKR_MDIV_Pos) + 1;
	return (
			io_cpu_clock_get_current_frequency (this->input)
		/	(float64_t) mdiv
	);
}

static float64_t
same70_master_clock_divider_get_expected_frequency (io_cpu_clock_pointer_t clock) {
	same70_master_clock_divider_t const *this = (
		(same70_master_clock_divider_t const*) (
			io_cpu_clock_ro_pointer (clock)
		)
	);
	return (
			io_cpu_clock_get_current_frequency (this->input)
		/	(float64_t) this->divisor
	);
}

static bool
same70_master_clock_divider_start (io_t *io,io_cpu_clock_pointer_t clock) {
	same70_master_clock_divider_t const *this = (
		(same70_master_clock_divider_t const*) (
			io_cpu_clock_ro_pointer (clock)
		)
	);
	if (io_cpu_clock_start (io,this->input)) {
		return true;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t same70_master_clock_divider_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = same70_master_clock_divider_get_current_frequency,
	.get_expected_frequency = same70_master_clock_divider_get_expected_frequency,
	.get_input = io_cpu_dependant_clock_get_input,
	.iterate_outputs = io_cpu_clock_function_iterate_outputs,
	.start = same70_master_clock_divider_start,
	.stop = NULL,
};

static bool
same70_plla_pcr_clock_output_iterator (
	io_cpu_clock_pointer_t output_clock,void *user_value
) {
	
	if (io_cpu_clock_is_pcr_peripheral_clock (output_clock)) {
		io_cpu_clock_pointer_t *clock = user_value;
		same70_peripheral_clock_t const *out = (same70_peripheral_clock_t const*) (
			io_cpu_clock_ro_pointer (output_clock)
		);

		float64_t input_frequency = io_cpu_clock_get_current_frequency (*clock);
		uint32_t divisor = (uint32_t) (input_frequency / out->frequency);
		
		if (divisor > 0 && divisor < 512) {
			PMC->PMC_PCR = (
					PMC_PCR_GCLKEN
				|	PMC_PCR_EN
				|	PMC_PCR_GCLKDIV (divisor - 1)
				|	PMC_PCR_GCLKCSS_PLLA_CLK
				|	PMC_PCR_PID (out->peripheral_id)
				|	PMC_PCR_CMD	// write
			);
		}
	}
	
	return true;
}

//
// PCR
//
static bool
same70_pcr_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	same70_pcr_clock_t const *this = (
		(same70_pcr_clock_t const*) io_cpu_clock_ro_pointer (clock)
	);
	if (io_cpu_clock_start (io,this->input)) {
		//
		// PCR clock can only be supplied by master clock divcider
		//
//		if (io_cpu_clock_is_plla_clock (this->input)) {
		if (io_cpu_clock_is_master_clock_divider (this->input)) {
			io_cpu_clock_iterate_outputs (
				clock,same70_plla_pcr_clock_output_iterator,&clock
			);
		} else {
			return false;
		}
		return true;
	} else {
		return false;
	}
}

static float64_t
same70_pcr_clock_get_current_frequency (io_cpu_clock_pointer_t clock) {
	same70_pcr_clock_t const *this = (
		(same70_pcr_clock_t const*) io_cpu_clock_ro_pointer (clock)
	);
	return io_cpu_clock_get_current_frequency (this->input);
}

static float64_t
same70_pcr_clock_get_expected_frequency (io_cpu_clock_pointer_t clock) {
	same70_pcr_clock_t const *this = (
		(same70_pcr_clock_t const*) io_cpu_clock_ro_pointer (clock)
	);
	return io_cpu_clock_get_expected_frequency (this->input);
}

EVENT_DATA io_cpu_clock_implementation_t same70_pcr_clock_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = same70_pcr_clock_get_current_frequency,
	.get_expected_frequency = same70_pcr_clock_get_expected_frequency,
	.get_input = io_cpu_dependant_clock_get_input,
	.iterate_outputs = io_cpu_clock_function_iterate_outputs,
	.start = same70_pcr_clock_start,
	.stop = NULL,
};

//
// PCK clocks
//
static bool
same70_pck_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	same70_pck_clock_t const *this = (
		(same70_pck_clock_t const*) (
			io_cpu_clock_ro_pointer (clock)
		)
	);
	if (io_cpu_clock_start (io,this->input)) {
		__IO uint32_t *reg = PMC->PMC_PCK + this->clock_number;
		
		pmc_disable_write_protection ();

		if (io_cpu_clock_is_master_clock_controller (this->input)) {
			*reg = (
					PMC_PCK_CSS_MCK
				|	PMC_PCK_PRES (this->divisor - 1)
			);
		} else if (io_cpu_clock_is_plla_clock (this->input)) {
			*reg = (
					PMC_PCK_CSS_PLLA_CLK
				|	PMC_PCK_PRES (this->divisor - 1)
			);
		} else {
			pmc_enable_write_protection ();
			return false;
		}

		PMC->PMC_SCER = (1 << this->clock_number) << 8;
		pmc_enable_write_protection ();
		return true;
	} else {
		return false;
	}
}

static float64_t
same70_pck_clock_get_current_frequency (io_cpu_clock_pointer_t clock) {
	same70_pck_clock_t const *this = (
		(same70_pck_clock_t const*) (
			io_cpu_clock_ro_pointer (clock)
		)
	);
	float64_t f = io_cpu_clock_get_current_frequency (this->input);
	return f/this->divisor;
}

static float64_t
same70_pck_clock_get_expected_frequency (io_cpu_clock_pointer_t clock) {
	same70_pck_clock_t const *this = (
		(same70_pck_clock_t const*) io_cpu_clock_ro_pointer (clock)
	);
	float64_t f = io_cpu_clock_get_expected_frequency (this->input);
	return f/this->divisor;
}

uint32_t
same70_pck_clock_get_clock_number (io_cpu_clock_pointer_t clock) {
	same70_pck_clock_t const *this = (
		(same70_pck_clock_t const*) io_cpu_clock_ro_pointer (clock)
	);
	return this->clock_number;
}

EVENT_DATA io_cpu_clock_implementation_t same70_pck_clock_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = same70_pck_clock_get_current_frequency,
	.get_expected_frequency = same70_pck_clock_get_expected_frequency,
	.get_input = io_cpu_dependant_clock_get_input,
	.iterate_outputs = io_cpu_clock_iterate_outputs_nop,
	.start = same70_pck_clock_start,
	.stop = NULL,
};

static bool
same70_processor_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	io_cpu_dependant_clock_t const *this = (io_cpu_dependant_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);

	eefc_set_flash_wait_states (
		EFC, get_flash_wait_states (
			(uint32_t) io_cpu_clock_get_expected_frequency (this->input)
		)
	);

	if (io_cpu_dependant_clock_start_input (io,clock)) {

		// select source
		
		return true;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t same70_processor_clock_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = io_dependant_cpu_clock_get_current_frequency,
	.get_expected_frequency = io_dependant_cpu_clock_get_expected_frequency,
	.get_input = io_cpu_dependant_clock_get_input,
	.iterate_outputs = io_cpu_clock_iterate_outputs_nop,
	.start = same70_processor_clock_start,
	.stop = NULL,
};

static bool
same70_peripheral_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	same70_peripheral_clock_t const *this = (same70_peripheral_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);

	if (io_cpu_dependant_clock_start_input (io,clock)) {
		pmc_disable_write_protection ();

		if (this->peripheral_id < 32) {
			PMC->PMC_PCER0 = 1 << this->peripheral_id;
		} else {
			PMC->PMC_PCER1 = 1 << (this->peripheral_id - 32);
		}
		
		pmc_enable_write_protection ();
		
		return true;
	} else {
		return false;
	}
}

static float64_t
same70_peripheral_clock_get_current_frequency (io_cpu_clock_pointer_t clock) {
	same70_peripheral_clock_t const *this = (
		(same70_peripheral_clock_t const*) (
			io_cpu_clock_ro_pointer (clock)
		)
	);
	float64_t f = io_cpu_clock_get_current_frequency (this->input);
	if (io_cpu_clock_is_master_clock_divider (this->input)) {
		//
		// divisor is stored on input
		//

		float64_t input_frequency = io_cpu_clock_get_current_frequency (this->input);
		uint32_t divisor = (uint32_t) (input_frequency / this->frequency);
		f /= divisor;
	}
	
	return f;
}

EVENT_DATA io_cpu_clock_implementation_t same70_pcr_peripheral_clock_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_power_domain = get_always_on_io_power_domain,
	.get_current_frequency = same70_peripheral_clock_get_current_frequency,
	.get_expected_frequency = io_dependant_cpu_clock_get_expected_frequency,
	.get_input = io_cpu_dependant_clock_get_input,
	.iterate_outputs = io_cpu_clock_iterate_outputs_nop,
	.start = same70_peripheral_clock_start,
	.stop = NULL,
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

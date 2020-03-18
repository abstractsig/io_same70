/*
 *
 * mpu
 *
 */
#ifndef same70_mpu_H_
#define same70_mpu_H_

#define MPU_REGION_ENABLE_EXECUTE			.enable_execute = 0
#define MPU_REGION_DISABLE_EXECUTE			.enable_execute = 1

#define MPU_REGION_NO_ACCESS					.access_permission = 0
#define MPU_REGION_PRIVILEGED_ACCESS		.access_permission = 1
#define MPU_REGION_READ_ONLY					.access_permission = 2
#define MPU_REGION_FULL_ACCESS				.access_permission = 3

#define MPU_REGION_ENABLE						.enable = 1
#define MPU_REGION_DISABLE						.enable = 0

#define MPU_REGION_SIZE(S)						.size = ((S) - 1)

#define MPU_BASE_ADDRESS(addr)				((addr) >> MPU_RBAR_ADDR_Pos)

#define MPU_SUBREGION_ENABLED					0
#define MPU_SUBREGION_DISABLED				1

typedef union PACK_STRUCTURE mpu_rbar {
	uint32_t all;
	struct PACK_STRUCTURE {
		uint32_t region:4;
		uint32_t valid:1;
		uint32_t base_address:27;
	} rbar;
} mpu_rbar_t;

typedef union PACK_STRUCTURE mpu_rasr {
	uint32_t all;
	struct PACK_STRUCTURE {
		uint32_t enable:1;
		uint32_t size:5;
		uint32_t :2;
		uint32_t subregion_0_enable:1;
		uint32_t subregion_1_enable:1;
		uint32_t subregion_2_enable:1;
		uint32_t subregion_3_enable:1;
		uint32_t subregion_4_enable:1;
		uint32_t subregion_5_enable:1;
		uint32_t subregion_6_enable:1;
		uint32_t subregion_7_enable:1;
		uint32_t B:1;
		uint32_t C:1;
		uint32_t S:1;
		uint32_t TEX:3;
		uint32_t :2;
		uint32_t access_permission:3;
		uint32_t :1;
		uint32_t enable_execute:1;
		uint32_t :3;
	} rasr;
} mpu_rasr_t;

typedef struct MPU_Region {
	mpu_rbar_t rbar;
	mpu_rasr_t rasr;
} MPU_Region_t;

#define MPU_NORMAL_NO_CACHE			 		.TEX = 1,.C = 0,.B = 0,.S = 1
#define MPU_NORMAL_NO_WRITE_ALLOATE 		.TEX = 0,.C = 1,.B = 1,.S = 1
#define MPU_NORMAL_READ_WRITE_ALLOATE 		.TEX = 1,.C = 1,.B = 1,.S = 1
#define MPU_NORMAL_STRONGLY_ORDERED		 	.TEX = 0,.C = 0,.B = 1,.S = 1

/* Device Shareable */
#define MPU_ATTR_DEVICE           (MPU_RASR_TEX(0) | MPU_RASR_B)

#define MPU_ENABLE_ALL_SUBREGIONS	\
			.subregion_0_enable = MPU_SUBREGION_ENABLED,	\
			.subregion_1_enable = MPU_SUBREGION_ENABLED,	\
			.subregion_2_enable = MPU_SUBREGION_ENABLED,	\
			.subregion_3_enable = MPU_SUBREGION_ENABLED,	\
			.subregion_4_enable = MPU_SUBREGION_ENABLED,	\
			.subregion_5_enable = MPU_SUBREGION_ENABLED,	\
			.subregion_6_enable = MPU_SUBREGION_ENABLED,	\
			.subregion_7_enable = MPU_SUBREGION_ENABLED,

#define MPU_ENABLE_SUBREGIONS(R0,R1,R2,R3,R4,R5,R6,R7)	\
			.subregion_0_enable = R0,	\
			.subregion_1_enable = R1,	\
			.subregion_2_enable = R2,	\
			.subregion_3_enable = R3,	\
			.subregion_4_enable = R4,	\
			.subregion_5_enable = R5,	\
			.subregion_6_enable = R6,	\
			.subregion_7_enable = R7,

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------

bool
mpu_is_enabled (void) {
	return (MPU->CTRL & (MPU_CTRL_ENABLE_Msk << MPU_CTRL_ENABLE_Pos)) != 0;
}

void
mpu_enable (void) {
	 MPU->CTRL |= (MPU_CTRL_ENABLE_Msk << MPU_CTRL_ENABLE_Pos);
}

void
mpu_disable (void) {
	 MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
}

void
set_device_mpu_configuration (MPU_Region_t const *table,uint32_t len) {
	for (int i = 0; i < len; i++, table++) {
		MPU->RBAR = table->rbar.all;
		MPU->RASR = table->rasr.all;
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



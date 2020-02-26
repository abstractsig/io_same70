/**
 * \file
 *
 * Copyright (c) 2015-2019 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef _SAME70Q20_
#define _SAME70Q20_

/** \addtogroup SAME70Q20_definitions SAME70Q20 definitions
  This file defines all structures and symbols for SAME70Q20:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - PIO definitions
*/
/*@{*/

#ifdef __cplusplus
 extern "C" {
#endif 

#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
#include <stdint.h>
#endif

/* ************************************************************************** */
/*   CMSIS DEFINITIONS FOR SAME70Q20 */
/* ************************************************************************** */
/** \addtogroup SAME70Q20_cmsis CMSIS Definitions */
/*@{*/

/**< Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M7 Processor Exceptions Numbers ******************************/
  NonMaskableInt_IRQn   = -14, /**<  2 Non Maskable Interrupt                */
  HardFault_IRQn        = -13, /**<  3 HardFault Interrupt                   */
  MemoryManagement_IRQn = -12, /**<  4 Cortex-M7 Memory Management Interrupt */
  BusFault_IRQn         = -11, /**<  5 Cortex-M7 Bus Fault Interrupt         */
  UsageFault_IRQn       = -10, /**<  6 Cortex-M7 Usage Fault Interrupt       */
  SVCall_IRQn           = -5,  /**< 11 Cortex-M7 SV Call Interrupt           */
  DebugMonitor_IRQn     = -4,  /**< 12 Cortex-M7 Debug Monitor Interrupt     */
  PendSV_IRQn           = -2,  /**< 14 Cortex-M7 Pend SV Interrupt           */
  SysTick_IRQn          = -1,  /**< 15 Cortex-M7 System Tick Interrupt       */
/******  SAME70Q20 specific Interrupt Numbers *********************************/
  
  SUPC_IRQn            =  0, /**<  0 SAME70Q20 Supply Controller (SUPC) */
  RSTC_IRQn            =  1, /**<  1 SAME70Q20 Reset Controller (RSTC) */
  RTC_IRQn             =  2, /**<  2 SAME70Q20 Real Time Clock (RTC) */
  RTT_IRQn             =  3, /**<  3 SAME70Q20 Real Time Timer (RTT) */
  WDT_IRQn             =  4, /**<  4 SAME70Q20 Watchdog Timer (WDT) */
  PMC_IRQn             =  5, /**<  5 SAME70Q20 Power Management Controller (PMC) */
  EFC_IRQn             =  6, /**<  6 SAME70Q20 Enhanced Embedded Flash Controller (EFC) */
  UART0_IRQn           =  7, /**<  7 SAME70Q20 UART 0 (UART0) */
  UART1_IRQn           =  8, /**<  8 SAME70Q20 UART 1 (UART1) */
  PIOA_IRQn            = 10, /**< 10 SAME70Q20 Parallel I/O Controller A (PIOA) */
  PIOB_IRQn            = 11, /**< 11 SAME70Q20 Parallel I/O Controller B (PIOB) */
  PIOC_IRQn            = 12, /**< 12 SAME70Q20 Parallel I/O Controller C (PIOC) */
  USART0_IRQn          = 13, /**< 13 SAME70Q20 USART 0 (USART0) */
  USART1_IRQn          = 14, /**< 14 SAME70Q20 USART 1 (USART1) */
  USART2_IRQn          = 15, /**< 15 SAME70Q20 USART 2 (USART2) */
  PIOD_IRQn            = 16, /**< 16 SAME70Q20 Parallel I/O Controller D (PIOD) */
  PIOE_IRQn            = 17, /**< 17 SAME70Q20 Parallel I/O Controller E (PIOE) */
  HSMCI_IRQn           = 18, /**< 18 SAME70Q20 Multimedia Card Interface (HSMCI) */
  TWIHS0_IRQn          = 19, /**< 19 SAME70Q20 Two Wire Interface 0 HS (TWIHS0) */
  TWIHS1_IRQn          = 20, /**< 20 SAME70Q20 Two Wire Interface 1 HS (TWIHS1) */
  SPI0_IRQn            = 21, /**< 21 SAME70Q20 Serial Peripheral Interface 0 (SPI0) */
  SSC_IRQn             = 22, /**< 22 SAME70Q20 Synchronous Serial Controller (SSC) */
  TC0_IRQn             = 23, /**< 23 SAME70Q20 Timer/Counter 0 (TC0) */
  TC1_IRQn             = 24, /**< 24 SAME70Q20 Timer/Counter 1 (TC1) */
  TC2_IRQn             = 25, /**< 25 SAME70Q20 Timer/Counter 2 (TC2) */
  TC3_IRQn             = 26, /**< 26 SAME70Q20 Timer/Counter 3 (TC3) */
  TC4_IRQn             = 27, /**< 27 SAME70Q20 Timer/Counter 4 (TC4) */
  TC5_IRQn             = 28, /**< 28 SAME70Q20 Timer/Counter 5 (TC5) */
  AFEC0_IRQn           = 29, /**< 29 SAME70Q20 Analog Front End 0 (AFEC0) */
  DACC_IRQn            = 30, /**< 30 SAME70Q20 Digital To Analog Converter (DACC) */
  PWM0_IRQn            = 31, /**< 31 SAME70Q20 Pulse Width Modulation 0 (PWM0) */
  ICM_IRQn             = 32, /**< 32 SAME70Q20 Integrity Check Monitor (ICM) */
  ACC_IRQn             = 33, /**< 33 SAME70Q20 Analog Comparator (ACC) */
  USBHS_IRQn           = 34, /**< 34 SAME70Q20 USB Host / Device Controller (USBHS) */
  MCAN0_INT0_IRQn      = 35, /**< 35 SAME70Q20 Controller Area Network (MCAN0) */
  MCAN0_INT1_IRQn      = 36, /**< 36 SAME70Q20 Controller Area Network (MCAN0) */
  MCAN1_INT0_IRQn      = 37, /**< 37 SAME70Q20 Controller Area Network (MCAN1) */
  MCAN1_INT1_IRQn      = 38, /**< 38 SAME70Q20 Controller Area Network (MCAN1) */
  GMAC_IRQn            = 39, /**< 39 SAME70Q20 Ethernet MAC (GMAC) */
  AFEC1_IRQn           = 40, /**< 40 SAME70Q20 Analog Front End 1 (AFEC1) */
  TWIHS2_IRQn          = 41, /**< 41 SAME70Q20 Two Wire Interface 2 HS (TWIHS2) */
  SPI1_IRQn            = 42, /**< 42 SAME70Q20 Serial Peripheral Interface 1 (SPI1) */
  QSPI_IRQn            = 43, /**< 43 SAME70Q20 Quad I/O Serial Peripheral Interface (QSPI) */
  UART2_IRQn           = 44, /**< 44 SAME70Q20 UART 2 (UART2) */
  UART3_IRQn           = 45, /**< 45 SAME70Q20 UART 3 (UART3) */
  UART4_IRQn           = 46, /**< 46 SAME70Q20 UART 4 (UART4) */
  TC6_IRQn             = 47, /**< 47 SAME70Q20 Timer/Counter 6 (TC6) */
  TC7_IRQn             = 48, /**< 48 SAME70Q20 Timer/Counter 7 (TC7) */
  TC8_IRQn             = 49, /**< 49 SAME70Q20 Timer/Counter 8 (TC8) */
  TC9_IRQn             = 50, /**< 50 SAME70Q20 Timer/Counter 9 (TC9) */
  TC10_IRQn            = 51, /**< 51 SAME70Q20 Timer/Counter 10 (TC10) */
  TC11_IRQn            = 52, /**< 52 SAME70Q20 Timer/Counter 11 (TC11) */
  AES_IRQn             = 56, /**< 56 SAME70Q20 AES (AES) */
  TRNG_IRQn            = 57, /**< 57 SAME70Q20 True Random Generator (TRNG) */
  XDMAC_IRQn           = 58, /**< 58 SAME70Q20 DMA (XDMAC) */
  ISI_IRQn             = 59, /**< 59 SAME70Q20 Camera Interface (ISI) */
  PWM1_IRQn            = 60, /**< 60 SAME70Q20 Pulse Width Modulation 1 (PWM1) */
  FPU_IRQn             = 61, /**< 61 SAME70Q20 Floating Point Unit Registers (FPU) */
  SDRAMC_IRQn          = 62, /**< 62 SAME70Q20 SDRAM Controller (SDRAMC) */
  RSWDT_IRQn           = 63, /**< 63 SAME70Q20 Reinforced Safety Watchdog Timer (RSWDT) */
  CCW_IRQn             = 64, /**< 64 SAME70Q20 System Control Registers (SystemControl) */
  CCF_IRQn             = 65, /**< 65 SAME70Q20 System Control Registers (SystemControl) */
  GMAC_Q1_IRQn         = 66, /**< 66 SAME70Q20 Gigabit Ethernet MAC (GMAC) */
  GMAC_Q2_IRQn         = 67, /**< 67 SAME70Q20 Gigabit Ethernet MAC (GMAC) */
  IXC_IRQn             = 68, /**< 68 SAME70Q20 Floating Point Unit Registers (FPU) */

  PERIPH_COUNT_IRQn    = 74  /**< Number of peripheral IDs */
} IRQn_Type;

typedef struct _DeviceVectors
{
  /* Stack pointer */
  void* pvStack;
  
  /* Cortex-M handlers */
  void* pfnReset_Handler;
  void* pfnNMI_Handler;
  void* pfnHardFault_Handler;
  void* pfnMemManage_Handler;
  void* pfnBusFault_Handler;
  void* pfnUsageFault_Handler;
  void* pfnReserved1_Handler;
  void* pfnReserved2_Handler;
  void* pfnReserved3_Handler;
  void* pfnReserved4_Handler;
  void* pfnSVC_Handler;
  void* pfnDebugMon_Handler;
  void* pfnReserved5_Handler;
  void* pfnPendSV_Handler;
  void* pfnSysTick_Handler;

  /* Peripheral handlers */
  void* pfnSUPC_Handler;   /*  0 Supply Controller */
  void* pfnRSTC_Handler;   /*  1 Reset Controller */
  void* pfnRTC_Handler;    /*  2 Real Time Clock */
  void* pfnRTT_Handler;    /*  3 Real Time Timer */
  void* pfnWDT_Handler;    /*  4 Watchdog Timer */
  void* pfnPMC_Handler;    /*  5 Power Management Controller */
  void* pfnEFC_Handler;    /*  6 Enhanced Embedded Flash Controller */
  void* pfnUART0_Handler;  /*  7 UART 0 */
  void* pfnUART1_Handler;  /*  8 UART 1 */
  void* pvReserved9;
  void* pfnPIOA_Handler;   /* 10 Parallel I/O Controller A */
  void* pfnPIOB_Handler;   /* 11 Parallel I/O Controller B */
  void* pfnPIOC_Handler;   /* 12 Parallel I/O Controller C */
  void* pfnUSART0_Handler; /* 13 USART 0 */
  void* pfnUSART1_Handler; /* 14 USART 1 */
  void* pfnUSART2_Handler; /* 15 USART 2 */
  void* pfnPIOD_Handler;   /* 16 Parallel I/O Controller D */
  void* pfnPIOE_Handler;   /* 17 Parallel I/O Controller E */
  void* pfnHSMCI_Handler;  /* 18 Multimedia Card Interface */
  void* pfnTWIHS0_Handler; /* 19 Two Wire Interface 0 HS */
  void* pfnTWIHS1_Handler; /* 20 Two Wire Interface 1 HS */
  void* pfnSPI0_Handler;   /* 21 Serial Peripheral Interface 0 */
  void* pfnSSC_Handler;    /* 22 Synchronous Serial Controller */
  void* pfnTC0_Handler;    /* 23 Timer/Counter 0 */
  void* pfnTC1_Handler;    /* 24 Timer/Counter 1 */
  void* pfnTC2_Handler;    /* 25 Timer/Counter 2 */
  void* pfnTC3_Handler;    /* 26 Timer/Counter 3 */
  void* pfnTC4_Handler;    /* 27 Timer/Counter 4 */
  void* pfnTC5_Handler;    /* 28 Timer/Counter 5 */
  void* pfnAFEC0_Handler;  /* 29 Analog Front End 0 */
  void* pfnDACC_Handler;   /* 30 Digital To Analog Converter */
  void* pfnPWM0_Handler;   /* 31 Pulse Width Modulation 0 */
  void* pfnICM_Handler;    /* 32 Integrity Check Monitor */
  void* pfnACC_Handler;    /* 33 Analog Comparator */
  void* pfnUSBHS_Handler;  /* 34 USB Host / Device Controller */
  void* pfnMCAN0_INT0_Handler;                   /* 35  Controller Area Network (MCAN0) */
  void* pfnMCAN0_INT1_Handler;                   /* 36  Controller Area Network (MCAN0) */
  void* pfnMCAN1_INT0_Handler;                   /* 37  Controller Area Network (MCAN1) */
  void* pfnMCAN1_INT1_Handler;                   /* 38  Controller Area Network (MCAN1) */
  void* pfnGMAC_Handler;   /* 39 Ethernet MAC */
  void* pfnAFEC1_Handler;  /* 40 Analog Front End 1 */
  void* pfnTWIHS2_Handler; /* 41 Two Wire Interface 2 HS */
  void* pfnSPI1_Handler;   /* 42 Serial Peripheral Interface 1 */
  void* pfnQSPI_Handler;   /* 43 Quad I/O Serial Peripheral Interface */
  void* pfnUART2_Handler;  /* 44 UART 2 */
  void* pfnUART3_Handler;  /* 45 UART 3 */
  void* pfnUART4_Handler;  /* 46 UART 4 */
  void* pfnTC6_Handler;    /* 47 Timer/Counter 6 */
  void* pfnTC7_Handler;    /* 48 Timer/Counter 7 */
  void* pfnTC8_Handler;    /* 49 Timer/Counter 8 */
  void* pfnTC9_Handler;    /* 50 Timer/Counter 9 */
  void* pfnTC10_Handler;   /* 51 Timer/Counter 10 */
  void* pfnTC11_Handler;   /* 52 Timer/Counter 11 */
  void* pvReserved53;
  void* pvReserved54;
  void* pvReserved55;
  void* pfnAES_Handler;    /* 56 AES */
  void* pfnTRNG_Handler;   /* 57 True Random Generator */
  void* pfnXDMAC_Handler;  /* 58 DMA */
  void* pfnISI_Handler;    /* 59 Camera Interface */
  void* pfnPWM1_Handler;   /* 60 Pulse Width Modulation 1 */
  void* pfnFPU_Handler;    /* 61 Floating Point Unit Registers (FPU) */
  void* pfnSDRAMC_Handler; /* 62 SDRAM Controller (SDRAMC) */
  void* pfnRSWDT_Handler;  /* 63 Reinforced Safety Watchdog Timer (RSWDT) */
  void* pfnCCW_Handler;    /* 64 System Control Registers (SystemControl) */
  void* pfnCCF_Handler;    /* 65 System Control Registers (SystemControl) */
  void* pfnGMAC_Q1_Handler;/* 66 Gigabit Ethernet MAC (GMAC) */
  void* pfnGMAC_Q2_Handler;/* 67 Gigabit Ethernet MAC (GMAC) */
  void* pfnIXC_Handler;    /* 68 Floating Point Unit Registers (FPU) */
  void* pvReserved69;
  void* pvReserved70;
  void* pvReserved71;
  void* pvReserved72;
  void* pvReserved73;
} DeviceVectors;

/* Cortex-M7 core handlers */
void Reset_Handler      ( void );
void NMI_Handler        ( void );
void HardFault_Handler  ( void );
void MemManage_Handler  ( void );
void BusFault_Handler   ( void );
void UsageFault_Handler ( void );
void SVC_Handler        ( void );
void DebugMon_Handler   ( void );
void PendSV_Handler     ( void );
void SysTick_Handler    ( void );

/* Peripherals handlers */
void ACC_Handler        ( void );
void AES_Handler        ( void );
void AFEC0_Handler      ( void );
void AFEC1_Handler      ( void );
void CCF_Handler        ( void );
void CCW_Handler        ( void );
void DACC_Handler       ( void );
void EFC_Handler        ( void );
void FPU_Handler        ( void );
void GMAC_Handler       ( void );
void HSMCI_Handler      ( void );
void ICM_Handler        ( void );
void ISI_Handler        ( void );
void IXC_Handler        ( void );
void MCAN0_INT0_Handler ( void );
void MCAN0_INT1_Handler ( void );
void MCAN1_INT0_Handler ( void );
void MCAN1_INT1_Handler ( void );
void PIOA_Handler       ( void );
void PIOB_Handler       ( void );
void PIOC_Handler       ( void );
void PIOD_Handler       ( void );
void PIOE_Handler       ( void );
void PMC_Handler        ( void );
void PWM0_Handler       ( void );
void PWM1_Handler       ( void );
void GMAC_Q1_Handler    ( void );
void GMAC_Q2_Handler    ( void );
void QSPI_Handler       ( void );
void RSTC_Handler       ( void );
void RSWDT_Handler      ( void );
void RTC_Handler        ( void );
void RTT_Handler        ( void );
void SDRAMC_Handler     ( void );
void SPI0_Handler       ( void );
void SPI1_Handler       ( void );
void SSC_Handler        ( void );
void SUPC_Handler       ( void );
void TC0_Handler        ( void );
void TC1_Handler        ( void );
void TC2_Handler        ( void );
void TC3_Handler        ( void );
void TC4_Handler        ( void );
void TC5_Handler        ( void );
void TC6_Handler        ( void );
void TC7_Handler        ( void );
void TC8_Handler        ( void );
void TC9_Handler        ( void );
void TC10_Handler       ( void );
void TC11_Handler       ( void );
void TRNG_Handler       ( void );
void TWIHS0_Handler     ( void );
void TWIHS1_Handler     ( void );
void TWIHS2_Handler     ( void );
void UART0_Handler      ( void );
void UART1_Handler      ( void );
void UART2_Handler      ( void );
void UART3_Handler      ( void );
void UART4_Handler      ( void );
void USART0_Handler     ( void );
void USART1_Handler     ( void );
void USART2_Handler     ( void );
void USBHS_Handler      ( void );
void WDT_Handler        ( void );
void XDMAC_Handler      ( void );

/**
 * \brief Configuration of the Cortex-M7 Processor and Core Peripherals 
 */

#define __CM7_REV              0x0000 /**< SAME70Q20 core revision number ([15:8] revision number, [7:0] patch number) */
#define __MPU_PRESENT          1      /**< SAME70Q20 does provide a MPU */
#define __NVIC_PRIO_BITS       3      /**< SAME70Q20 uses 3 Bits for the Priority Levels */
#define __FPU_PRESENT          1      /**< SAME70Q20 does provide a FPU                */
#define __FPU_DP               1      /**< SAME70Q20 Double precision FPU              */
#define __ICACHE_PRESENT       1      /**< SAME70Q20 does provide an Instruction Cache */
#define __DCACHE_PRESENT       1      /**< SAME70Q20 does provide a Data Cache         */
#define __DTCM_PRESENT         1      /**< SAME70Q20 does provide a Data TCM           */
#define __ITCM_PRESENT         1      /**< SAME70Q20 does provide an Instruction TCM   */
#define __Vendor_SysTickConfig 0      /**< Set to 1 if different SysTick Config is used */
#define __SAM_M7_REVB		   0	  /**< SAME70Q20 Revision A */

/*
 * \brief CMSIS includes
 */

#include <core_cm7.h>
#if !defined DONT_USE_CMSIS_INIT
//#include "system_same70.h"
#endif /* DONT_USE_CMSIS_INIT */

/*@}*/

/* ************************************************************************** */
/**  SOFTWARE PERIPHERAL API DEFINITION FOR SAME70Q20 */
/* ************************************************************************** */
/** \addtogroup SAME70Q20_api Peripheral Software API */
/*@{*/
/*
#include "component/acc.h"
#include "component/aes.h"
#include "component/afec.h"
#include "component/chipid.h"
#include "component/dacc.h"
#include "component/efc.h"
#include "component/gmac.h"
#include "component/gpbr.h"
#include "component/hsmci.h"
#include "component/icm.h"
#include "component/isi.h"
#include "component/matrix.h"
#include "component/mcan.h"
#include "component/pio.h"
#include "component/pmc.h"
#include "component/pwm.h"
#include "component/qspi.h"
#include "component/rstc.h"
#include "component/rswdt.h"
#include "component/rtc.h"
#include "component/rtt.h"
#include "component/sdramc.h"
#include "component/smc.h"
#include "component/spi.h"
#include "component/ssc.h"
#include "component/supc.h"
#include "component/tc.h"
#include "component/trng.h"
#include "component/twihs.h"
#include "component/uart.h"
#include "component/usart.h"
#include "component/usbhs.h"
#include "component/utmi.h"
#include "component/wdt.h"
#include "component/xdmac.h"
*/
/*@}*/

#ifndef _SAME70_PIO_COMPONENT_
#define _SAME70_PIO_COMPONENT_

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Parallel Input/Output Controller */
/* ============================================================================= */
/** \addtogroup SAME70_PIO Parallel Input/Output Controller */
/*@{*/

#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
/** \brief Pio hardware registers */
typedef struct {
  __O  uint32_t PIO_PER;       /**< \brief (Pio Offset: 0x0000) PIO Enable Register */
  __O  uint32_t PIO_PDR;       /**< \brief (Pio Offset: 0x0004) PIO Disable Register */
  __I  uint32_t PIO_PSR;       /**< \brief (Pio Offset: 0x0008) PIO Status Register */
  __I  uint32_t Reserved1[1];
  __O  uint32_t PIO_OER;       /**< \brief (Pio Offset: 0x0010) Output Enable Register */
  __O  uint32_t PIO_ODR;       /**< \brief (Pio Offset: 0x0014) Output Disable Register */
  __I  uint32_t PIO_OSR;       /**< \brief (Pio Offset: 0x0018) Output Status Register */
  __I  uint32_t Reserved2[1];
  __O  uint32_t PIO_IFER;      /**< \brief (Pio Offset: 0x0020) Glitch Input Filter Enable Register */
  __O  uint32_t PIO_IFDR;      /**< \brief (Pio Offset: 0x0024) Glitch Input Filter Disable Register */
  __I  uint32_t PIO_IFSR;      /**< \brief (Pio Offset: 0x0028) Glitch Input Filter Status Register */
  __I  uint32_t Reserved3[1];
  __O  uint32_t PIO_SODR;      /**< \brief (Pio Offset: 0x0030) Set Output Data Register */
  __O  uint32_t PIO_CODR;      /**< \brief (Pio Offset: 0x0034) Clear Output Data Register */
  __IO uint32_t PIO_ODSR;      /**< \brief (Pio Offset: 0x0038) Output Data Status Register */
  __I  uint32_t PIO_PDSR;      /**< \brief (Pio Offset: 0x003C) Pin Data Status Register */
  __O  uint32_t PIO_IER;       /**< \brief (Pio Offset: 0x0040) Interrupt Enable Register */
  __O  uint32_t PIO_IDR;       /**< \brief (Pio Offset: 0x0044) Interrupt Disable Register */
  __I  uint32_t PIO_IMR;       /**< \brief (Pio Offset: 0x0048) Interrupt Mask Register */
  __I  uint32_t PIO_ISR;       /**< \brief (Pio Offset: 0x004C) Interrupt Status Register */
  __O  uint32_t PIO_MDER;      /**< \brief (Pio Offset: 0x0050) Multi-driver Enable Register */
  __O  uint32_t PIO_MDDR;      /**< \brief (Pio Offset: 0x0054) Multi-driver Disable Register */
  __I  uint32_t PIO_MDSR;      /**< \brief (Pio Offset: 0x0058) Multi-driver Status Register */
  __I  uint32_t Reserved4[1];
  __O  uint32_t PIO_PUDR;      /**< \brief (Pio Offset: 0x0060) Pull-up Disable Register */
  __O  uint32_t PIO_PUER;      /**< \brief (Pio Offset: 0x0064) Pull-up Enable Register */
  __I  uint32_t PIO_PUSR;      /**< \brief (Pio Offset: 0x0068) Pad Pull-up Status Register */
  __I  uint32_t Reserved5[1];
  __IO uint32_t PIO_ABCDSR[2]; /**< \brief (Pio Offset: 0x0070) Peripheral Select Register */
  __I  uint32_t Reserved6[2];
  __O  uint32_t PIO_IFSCDR;    /**< \brief (Pio Offset: 0x0080) Input Filter Slow Clock Disable Register */
  __O  uint32_t PIO_IFSCER;    /**< \brief (Pio Offset: 0x0084) Input Filter Slow Clock Enable Register */
  __I  uint32_t PIO_IFSCSR;    /**< \brief (Pio Offset: 0x0088) Input Filter Slow Clock Status Register */
  __IO uint32_t PIO_SCDR;      /**< \brief (Pio Offset: 0x008C) Slow Clock Divider Debouncing Register */
  __O  uint32_t PIO_PPDDR;     /**< \brief (Pio Offset: 0x0090) Pad Pull-down Disable Register */
  __O  uint32_t PIO_PPDER;     /**< \brief (Pio Offset: 0x0094) Pad Pull-down Enable Register */
  __I  uint32_t PIO_PPDSR;     /**< \brief (Pio Offset: 0x0098) Pad Pull-down Status Register */
  __I  uint32_t Reserved7[1];
  __O  uint32_t PIO_OWER;      /**< \brief (Pio Offset: 0x00A0) Output Write Enable */
  __O  uint32_t PIO_OWDR;      /**< \brief (Pio Offset: 0x00A4) Output Write Disable */
  __I  uint32_t PIO_OWSR;      /**< \brief (Pio Offset: 0x00A8) Output Write Status Register */
  __I  uint32_t Reserved8[1];
  __O  uint32_t PIO_AIMER;     /**< \brief (Pio Offset: 0x00B0) Additional Interrupt Modes Enable Register */
  __O  uint32_t PIO_AIMDR;     /**< \brief (Pio Offset: 0x00B4) Additional Interrupt Modes Disable Register */
  __I  uint32_t PIO_AIMMR;     /**< \brief (Pio Offset: 0x00B8) Additional Interrupt Modes Mask Register */
  __I  uint32_t Reserved9[1];
  __O  uint32_t PIO_ESR;       /**< \brief (Pio Offset: 0x00C0) Edge Select Register */
  __O  uint32_t PIO_LSR;       /**< \brief (Pio Offset: 0x00C4) Level Select Register */
  __I  uint32_t PIO_ELSR;      /**< \brief (Pio Offset: 0x00C8) Edge/Level Status Register */
  __I  uint32_t Reserved10[1];
  __O  uint32_t PIO_FELLSR;    /**< \brief (Pio Offset: 0x00D0) Falling Edge/Low-Level Select Register */
  __O  uint32_t PIO_REHLSR;    /**< \brief (Pio Offset: 0x00D4) Rising Edge/High-Level Select Register */
  __I  uint32_t PIO_FRLHSR;    /**< \brief (Pio Offset: 0x00D8) Fall/Rise - Low/High Status Register */
  __I  uint32_t Reserved11[1];
  __I  uint32_t PIO_LOCKSR;    /**< \brief (Pio Offset: 0x00E0) Lock Status */
  __IO uint32_t PIO_WPMR;      /**< \brief (Pio Offset: 0x00E4) Write Protection Mode Register */
  __I  uint32_t PIO_WPSR;      /**< \brief (Pio Offset: 0x00E8) Write Protection Status Register */
  __I  uint32_t Reserved12[4];
  __I  uint32_t PIO_VERSION;    /**< \brief (Pio Offset: 0x00FC) Version Register */
  __IO uint32_t PIO_SCHMITT;   /**< \brief (Pio Offset: 0x0100) Schmitt Trigger Register */
  __I  uint32_t Reserved13[5];
  __IO uint32_t PIO_DRIVER;     /**< \brief (Pio Offset: 0x0118) I/O Drive Register */
  __I  uint32_t Reserved14[13];
  __IO uint32_t PIO_PCMR;      /**< \brief (Pio Offset: 0x0150) Parallel Capture Mode Register */
  __O  uint32_t PIO_PCIER;     /**< \brief (Pio Offset: 0x0154) Parallel Capture Interrupt Enable Register */
  __O  uint32_t PIO_PCIDR;     /**< \brief (Pio Offset: 0x0158) Parallel Capture Interrupt Disable Register */
  __I  uint32_t PIO_PCIMR;     /**< \brief (Pio Offset: 0x015C) Parallel Capture Interrupt Mask Register */
  __I  uint32_t PIO_PCISR;     /**< \brief (Pio Offset: 0x0160) Parallel Capture Interrupt Status Register */
  __I  uint32_t PIO_PCRHR;     /**< \brief (Pio Offset: 0x0164) Parallel Capture Reception Holding Register */
} Pio;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */
/* -------- PIO_PER : (PIO Offset: 0x0000) PIO Enable Register -------- */
#define PIO_PER_P0 (0x1u << 0) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P1 (0x1u << 1) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P2 (0x1u << 2) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P3 (0x1u << 3) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P4 (0x1u << 4) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P5 (0x1u << 5) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P6 (0x1u << 6) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P7 (0x1u << 7) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P8 (0x1u << 8) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P9 (0x1u << 9) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P10 (0x1u << 10) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P11 (0x1u << 11) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P12 (0x1u << 12) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P13 (0x1u << 13) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P14 (0x1u << 14) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P15 (0x1u << 15) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P16 (0x1u << 16) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P17 (0x1u << 17) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P18 (0x1u << 18) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P19 (0x1u << 19) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P20 (0x1u << 20) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P21 (0x1u << 21) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P22 (0x1u << 22) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P23 (0x1u << 23) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P24 (0x1u << 24) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P25 (0x1u << 25) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P26 (0x1u << 26) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P27 (0x1u << 27) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P28 (0x1u << 28) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P29 (0x1u << 29) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P30 (0x1u << 30) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P31 (0x1u << 31) /**< \brief (PIO_PER) PIO Enable */
/* -------- PIO_PDR : (PIO Offset: 0x0004) PIO Disable Register -------- */
#define PIO_PDR_P0 (0x1u << 0) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P1 (0x1u << 1) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P2 (0x1u << 2) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P3 (0x1u << 3) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P4 (0x1u << 4) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P5 (0x1u << 5) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P6 (0x1u << 6) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P7 (0x1u << 7) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P8 (0x1u << 8) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P9 (0x1u << 9) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P10 (0x1u << 10) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P11 (0x1u << 11) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P12 (0x1u << 12) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P13 (0x1u << 13) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P14 (0x1u << 14) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P15 (0x1u << 15) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P16 (0x1u << 16) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P17 (0x1u << 17) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P18 (0x1u << 18) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P19 (0x1u << 19) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P20 (0x1u << 20) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P21 (0x1u << 21) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P22 (0x1u << 22) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P23 (0x1u << 23) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P24 (0x1u << 24) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P25 (0x1u << 25) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P26 (0x1u << 26) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P27 (0x1u << 27) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P28 (0x1u << 28) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P29 (0x1u << 29) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P30 (0x1u << 30) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P31 (0x1u << 31) /**< \brief (PIO_PDR) PIO Disable */
/* -------- PIO_PSR : (PIO Offset: 0x0008) PIO Status Register -------- */
#define PIO_PSR_P0 (0x1u << 0) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P1 (0x1u << 1) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P2 (0x1u << 2) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P3 (0x1u << 3) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P4 (0x1u << 4) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P5 (0x1u << 5) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P6 (0x1u << 6) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P7 (0x1u << 7) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P8 (0x1u << 8) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P9 (0x1u << 9) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P10 (0x1u << 10) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P11 (0x1u << 11) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P12 (0x1u << 12) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P13 (0x1u << 13) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P14 (0x1u << 14) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P15 (0x1u << 15) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P16 (0x1u << 16) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P17 (0x1u << 17) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P18 (0x1u << 18) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P19 (0x1u << 19) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P20 (0x1u << 20) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P21 (0x1u << 21) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P22 (0x1u << 22) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P23 (0x1u << 23) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P24 (0x1u << 24) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P25 (0x1u << 25) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P26 (0x1u << 26) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P27 (0x1u << 27) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P28 (0x1u << 28) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P29 (0x1u << 29) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P30 (0x1u << 30) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P31 (0x1u << 31) /**< \brief (PIO_PSR) PIO Status */
/* -------- PIO_OER : (PIO Offset: 0x0010) Output Enable Register -------- */
#define PIO_OER_P0 (0x1u << 0) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P1 (0x1u << 1) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P2 (0x1u << 2) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P3 (0x1u << 3) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P4 (0x1u << 4) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P5 (0x1u << 5) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P6 (0x1u << 6) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P7 (0x1u << 7) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P8 (0x1u << 8) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P9 (0x1u << 9) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P10 (0x1u << 10) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P11 (0x1u << 11) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P12 (0x1u << 12) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P13 (0x1u << 13) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P14 (0x1u << 14) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P15 (0x1u << 15) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P16 (0x1u << 16) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P17 (0x1u << 17) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P18 (0x1u << 18) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P19 (0x1u << 19) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P20 (0x1u << 20) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P21 (0x1u << 21) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P22 (0x1u << 22) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P23 (0x1u << 23) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P24 (0x1u << 24) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P25 (0x1u << 25) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P26 (0x1u << 26) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P27 (0x1u << 27) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P28 (0x1u << 28) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P29 (0x1u << 29) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P30 (0x1u << 30) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P31 (0x1u << 31) /**< \brief (PIO_OER) Output Enable */
/* -------- PIO_ODR : (PIO Offset: 0x0014) Output Disable Register -------- */
#define PIO_ODR_P0 (0x1u << 0) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P1 (0x1u << 1) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P2 (0x1u << 2) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P3 (0x1u << 3) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P4 (0x1u << 4) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P5 (0x1u << 5) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P6 (0x1u << 6) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P7 (0x1u << 7) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P8 (0x1u << 8) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P9 (0x1u << 9) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P10 (0x1u << 10) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P11 (0x1u << 11) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P12 (0x1u << 12) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P13 (0x1u << 13) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P14 (0x1u << 14) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P15 (0x1u << 15) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P16 (0x1u << 16) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P17 (0x1u << 17) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P18 (0x1u << 18) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P19 (0x1u << 19) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P20 (0x1u << 20) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P21 (0x1u << 21) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P22 (0x1u << 22) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P23 (0x1u << 23) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P24 (0x1u << 24) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P25 (0x1u << 25) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P26 (0x1u << 26) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P27 (0x1u << 27) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P28 (0x1u << 28) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P29 (0x1u << 29) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P30 (0x1u << 30) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P31 (0x1u << 31) /**< \brief (PIO_ODR) Output Disable */
/* -------- PIO_OSR : (PIO Offset: 0x0018) Output Status Register -------- */
#define PIO_OSR_P0 (0x1u << 0) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P1 (0x1u << 1) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P2 (0x1u << 2) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P3 (0x1u << 3) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P4 (0x1u << 4) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P5 (0x1u << 5) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P6 (0x1u << 6) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P7 (0x1u << 7) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P8 (0x1u << 8) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P9 (0x1u << 9) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P10 (0x1u << 10) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P11 (0x1u << 11) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P12 (0x1u << 12) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P13 (0x1u << 13) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P14 (0x1u << 14) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P15 (0x1u << 15) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P16 (0x1u << 16) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P17 (0x1u << 17) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P18 (0x1u << 18) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P19 (0x1u << 19) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P20 (0x1u << 20) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P21 (0x1u << 21) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P22 (0x1u << 22) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P23 (0x1u << 23) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P24 (0x1u << 24) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P25 (0x1u << 25) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P26 (0x1u << 26) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P27 (0x1u << 27) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P28 (0x1u << 28) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P29 (0x1u << 29) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P30 (0x1u << 30) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P31 (0x1u << 31) /**< \brief (PIO_OSR) Output Status */
/* -------- PIO_IFER : (PIO Offset: 0x0020) Glitch Input Filter Enable Register -------- */
#define PIO_IFER_P0 (0x1u << 0) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P1 (0x1u << 1) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P2 (0x1u << 2) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P3 (0x1u << 3) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P4 (0x1u << 4) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P5 (0x1u << 5) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P6 (0x1u << 6) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P7 (0x1u << 7) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P8 (0x1u << 8) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P9 (0x1u << 9) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P10 (0x1u << 10) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P11 (0x1u << 11) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P12 (0x1u << 12) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P13 (0x1u << 13) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P14 (0x1u << 14) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P15 (0x1u << 15) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P16 (0x1u << 16) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P17 (0x1u << 17) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P18 (0x1u << 18) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P19 (0x1u << 19) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P20 (0x1u << 20) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P21 (0x1u << 21) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P22 (0x1u << 22) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P23 (0x1u << 23) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P24 (0x1u << 24) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P25 (0x1u << 25) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P26 (0x1u << 26) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P27 (0x1u << 27) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P28 (0x1u << 28) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P29 (0x1u << 29) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P30 (0x1u << 30) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P31 (0x1u << 31) /**< \brief (PIO_IFER) Input Filter Enable */
/* -------- PIO_IFDR : (PIO Offset: 0x0024) Glitch Input Filter Disable Register -------- */
#define PIO_IFDR_P0 (0x1u << 0) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P1 (0x1u << 1) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P2 (0x1u << 2) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P3 (0x1u << 3) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P4 (0x1u << 4) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P5 (0x1u << 5) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P6 (0x1u << 6) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P7 (0x1u << 7) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P8 (0x1u << 8) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P9 (0x1u << 9) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P10 (0x1u << 10) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P11 (0x1u << 11) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P12 (0x1u << 12) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P13 (0x1u << 13) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P14 (0x1u << 14) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P15 (0x1u << 15) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P16 (0x1u << 16) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P17 (0x1u << 17) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P18 (0x1u << 18) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P19 (0x1u << 19) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P20 (0x1u << 20) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P21 (0x1u << 21) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P22 (0x1u << 22) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P23 (0x1u << 23) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P24 (0x1u << 24) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P25 (0x1u << 25) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P26 (0x1u << 26) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P27 (0x1u << 27) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P28 (0x1u << 28) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P29 (0x1u << 29) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P30 (0x1u << 30) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P31 (0x1u << 31) /**< \brief (PIO_IFDR) Input Filter Disable */
/* -------- PIO_IFSR : (PIO Offset: 0x0028) Glitch Input Filter Status Register -------- */
#define PIO_IFSR_P0 (0x1u << 0) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P1 (0x1u << 1) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P2 (0x1u << 2) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P3 (0x1u << 3) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P4 (0x1u << 4) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P5 (0x1u << 5) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P6 (0x1u << 6) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P7 (0x1u << 7) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P8 (0x1u << 8) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P9 (0x1u << 9) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P10 (0x1u << 10) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P11 (0x1u << 11) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P12 (0x1u << 12) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P13 (0x1u << 13) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P14 (0x1u << 14) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P15 (0x1u << 15) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P16 (0x1u << 16) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P17 (0x1u << 17) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P18 (0x1u << 18) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P19 (0x1u << 19) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P20 (0x1u << 20) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P21 (0x1u << 21) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P22 (0x1u << 22) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P23 (0x1u << 23) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P24 (0x1u << 24) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P25 (0x1u << 25) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P26 (0x1u << 26) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P27 (0x1u << 27) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P28 (0x1u << 28) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P29 (0x1u << 29) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P30 (0x1u << 30) /**< \brief (PIO_IFSR) Input Filter Status */
#define PIO_IFSR_P31 (0x1u << 31) /**< \brief (PIO_IFSR) Input Filter Status */
/* -------- PIO_SODR : (PIO Offset: 0x0030) Set Output Data Register -------- */
#define PIO_SODR_P0 (0x1u << 0) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P1 (0x1u << 1) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P2 (0x1u << 2) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P3 (0x1u << 3) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P4 (0x1u << 4) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P5 (0x1u << 5) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P6 (0x1u << 6) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P7 (0x1u << 7) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P8 (0x1u << 8) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P9 (0x1u << 9) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P10 (0x1u << 10) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P11 (0x1u << 11) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P12 (0x1u << 12) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P13 (0x1u << 13) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P14 (0x1u << 14) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P15 (0x1u << 15) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P16 (0x1u << 16) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P17 (0x1u << 17) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P18 (0x1u << 18) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P19 (0x1u << 19) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P20 (0x1u << 20) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P21 (0x1u << 21) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P22 (0x1u << 22) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P23 (0x1u << 23) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P24 (0x1u << 24) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P25 (0x1u << 25) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P26 (0x1u << 26) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P27 (0x1u << 27) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P28 (0x1u << 28) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P29 (0x1u << 29) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P30 (0x1u << 30) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P31 (0x1u << 31) /**< \brief (PIO_SODR) Set Output Data */
/* -------- PIO_CODR : (PIO Offset: 0x0034) Clear Output Data Register -------- */
#define PIO_CODR_P0 (0x1u << 0) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P1 (0x1u << 1) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P2 (0x1u << 2) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P3 (0x1u << 3) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P4 (0x1u << 4) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P5 (0x1u << 5) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P6 (0x1u << 6) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P7 (0x1u << 7) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P8 (0x1u << 8) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P9 (0x1u << 9) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P10 (0x1u << 10) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P11 (0x1u << 11) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P12 (0x1u << 12) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P13 (0x1u << 13) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P14 (0x1u << 14) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P15 (0x1u << 15) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P16 (0x1u << 16) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P17 (0x1u << 17) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P18 (0x1u << 18) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P19 (0x1u << 19) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P20 (0x1u << 20) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P21 (0x1u << 21) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P22 (0x1u << 22) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P23 (0x1u << 23) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P24 (0x1u << 24) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P25 (0x1u << 25) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P26 (0x1u << 26) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P27 (0x1u << 27) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P28 (0x1u << 28) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P29 (0x1u << 29) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P30 (0x1u << 30) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P31 (0x1u << 31) /**< \brief (PIO_CODR) Clear Output Data */
/* -------- PIO_ODSR : (PIO Offset: 0x0038) Output Data Status Register -------- */
#define PIO_ODSR_P0 (0x1u << 0) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P1 (0x1u << 1) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P2 (0x1u << 2) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P3 (0x1u << 3) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P4 (0x1u << 4) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P5 (0x1u << 5) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P6 (0x1u << 6) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P7 (0x1u << 7) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P8 (0x1u << 8) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P9 (0x1u << 9) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P10 (0x1u << 10) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P11 (0x1u << 11) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P12 (0x1u << 12) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P13 (0x1u << 13) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P14 (0x1u << 14) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P15 (0x1u << 15) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P16 (0x1u << 16) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P17 (0x1u << 17) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P18 (0x1u << 18) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P19 (0x1u << 19) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P20 (0x1u << 20) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P21 (0x1u << 21) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P22 (0x1u << 22) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P23 (0x1u << 23) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P24 (0x1u << 24) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P25 (0x1u << 25) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P26 (0x1u << 26) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P27 (0x1u << 27) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P28 (0x1u << 28) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P29 (0x1u << 29) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P30 (0x1u << 30) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P31 (0x1u << 31) /**< \brief (PIO_ODSR) Output Data Status */
/* -------- PIO_PDSR : (PIO Offset: 0x003C) Pin Data Status Register -------- */
#define PIO_PDSR_P0 (0x1u << 0) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P1 (0x1u << 1) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P2 (0x1u << 2) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P3 (0x1u << 3) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P4 (0x1u << 4) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P5 (0x1u << 5) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P6 (0x1u << 6) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P7 (0x1u << 7) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P8 (0x1u << 8) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P9 (0x1u << 9) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P10 (0x1u << 10) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P11 (0x1u << 11) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P12 (0x1u << 12) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P13 (0x1u << 13) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P14 (0x1u << 14) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P15 (0x1u << 15) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P16 (0x1u << 16) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P17 (0x1u << 17) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P18 (0x1u << 18) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P19 (0x1u << 19) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P20 (0x1u << 20) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P21 (0x1u << 21) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P22 (0x1u << 22) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P23 (0x1u << 23) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P24 (0x1u << 24) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P25 (0x1u << 25) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P26 (0x1u << 26) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P27 (0x1u << 27) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P28 (0x1u << 28) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P29 (0x1u << 29) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P30 (0x1u << 30) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P31 (0x1u << 31) /**< \brief (PIO_PDSR) Output Data Status */
/* -------- PIO_IER : (PIO Offset: 0x0040) Interrupt Enable Register -------- */
#define PIO_IER_P0 (0x1u << 0) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P1 (0x1u << 1) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P2 (0x1u << 2) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P3 (0x1u << 3) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P4 (0x1u << 4) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P5 (0x1u << 5) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P6 (0x1u << 6) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P7 (0x1u << 7) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P8 (0x1u << 8) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P9 (0x1u << 9) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P10 (0x1u << 10) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P11 (0x1u << 11) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P12 (0x1u << 12) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P13 (0x1u << 13) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P14 (0x1u << 14) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P15 (0x1u << 15) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P16 (0x1u << 16) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P17 (0x1u << 17) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P18 (0x1u << 18) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P19 (0x1u << 19) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P20 (0x1u << 20) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P21 (0x1u << 21) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P22 (0x1u << 22) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P23 (0x1u << 23) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P24 (0x1u << 24) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P25 (0x1u << 25) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P26 (0x1u << 26) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P27 (0x1u << 27) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P28 (0x1u << 28) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P29 (0x1u << 29) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P30 (0x1u << 30) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P31 (0x1u << 31) /**< \brief (PIO_IER) Input Change Interrupt Enable */
/* -------- PIO_IDR : (PIO Offset: 0x0044) Interrupt Disable Register -------- */
#define PIO_IDR_P0 (0x1u << 0) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P1 (0x1u << 1) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P2 (0x1u << 2) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P3 (0x1u << 3) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P4 (0x1u << 4) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P5 (0x1u << 5) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P6 (0x1u << 6) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P7 (0x1u << 7) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P8 (0x1u << 8) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P9 (0x1u << 9) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P10 (0x1u << 10) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P11 (0x1u << 11) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P12 (0x1u << 12) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P13 (0x1u << 13) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P14 (0x1u << 14) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P15 (0x1u << 15) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P16 (0x1u << 16) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P17 (0x1u << 17) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P18 (0x1u << 18) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P19 (0x1u << 19) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P20 (0x1u << 20) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P21 (0x1u << 21) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P22 (0x1u << 22) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P23 (0x1u << 23) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P24 (0x1u << 24) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P25 (0x1u << 25) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P26 (0x1u << 26) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P27 (0x1u << 27) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P28 (0x1u << 28) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P29 (0x1u << 29) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P30 (0x1u << 30) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P31 (0x1u << 31) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
/* -------- PIO_IMR : (PIO Offset: 0x0048) Interrupt Mask Register -------- */
#define PIO_IMR_P0 (0x1u << 0) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P1 (0x1u << 1) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P2 (0x1u << 2) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P3 (0x1u << 3) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P4 (0x1u << 4) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P5 (0x1u << 5) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P6 (0x1u << 6) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P7 (0x1u << 7) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P8 (0x1u << 8) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P9 (0x1u << 9) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P10 (0x1u << 10) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P11 (0x1u << 11) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P12 (0x1u << 12) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P13 (0x1u << 13) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P14 (0x1u << 14) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P15 (0x1u << 15) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P16 (0x1u << 16) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P17 (0x1u << 17) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P18 (0x1u << 18) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P19 (0x1u << 19) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P20 (0x1u << 20) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P21 (0x1u << 21) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P22 (0x1u << 22) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P23 (0x1u << 23) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P24 (0x1u << 24) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P25 (0x1u << 25) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P26 (0x1u << 26) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P27 (0x1u << 27) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P28 (0x1u << 28) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P29 (0x1u << 29) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P30 (0x1u << 30) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P31 (0x1u << 31) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
/* -------- PIO_ISR : (PIO Offset: 0x004C) Interrupt Status Register -------- */
#define PIO_ISR_P0 (0x1u << 0) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P1 (0x1u << 1) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P2 (0x1u << 2) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P3 (0x1u << 3) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P4 (0x1u << 4) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P5 (0x1u << 5) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P6 (0x1u << 6) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P7 (0x1u << 7) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P8 (0x1u << 8) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P9 (0x1u << 9) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P10 (0x1u << 10) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P11 (0x1u << 11) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P12 (0x1u << 12) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P13 (0x1u << 13) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P14 (0x1u << 14) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P15 (0x1u << 15) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P16 (0x1u << 16) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P17 (0x1u << 17) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P18 (0x1u << 18) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P19 (0x1u << 19) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P20 (0x1u << 20) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P21 (0x1u << 21) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P22 (0x1u << 22) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P23 (0x1u << 23) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P24 (0x1u << 24) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P25 (0x1u << 25) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P26 (0x1u << 26) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P27 (0x1u << 27) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P28 (0x1u << 28) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P29 (0x1u << 29) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P30 (0x1u << 30) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P31 (0x1u << 31) /**< \brief (PIO_ISR) Input Change Interrupt Status */
/* -------- PIO_MDER : (PIO Offset: 0x0050) Multi-driver Enable Register -------- */
#define PIO_MDER_P0 (0x1u << 0) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P1 (0x1u << 1) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P2 (0x1u << 2) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P3 (0x1u << 3) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P4 (0x1u << 4) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P5 (0x1u << 5) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P6 (0x1u << 6) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P7 (0x1u << 7) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P8 (0x1u << 8) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P9 (0x1u << 9) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P10 (0x1u << 10) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P11 (0x1u << 11) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P12 (0x1u << 12) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P13 (0x1u << 13) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P14 (0x1u << 14) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P15 (0x1u << 15) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P16 (0x1u << 16) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P17 (0x1u << 17) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P18 (0x1u << 18) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P19 (0x1u << 19) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P20 (0x1u << 20) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P21 (0x1u << 21) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P22 (0x1u << 22) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P23 (0x1u << 23) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P24 (0x1u << 24) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P25 (0x1u << 25) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P26 (0x1u << 26) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P27 (0x1u << 27) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P28 (0x1u << 28) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P29 (0x1u << 29) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P30 (0x1u << 30) /**< \brief (PIO_MDER) Multi-drive Enable */
#define PIO_MDER_P31 (0x1u << 31) /**< \brief (PIO_MDER) Multi-drive Enable */
/* -------- PIO_MDDR : (PIO Offset: 0x0054) Multi-driver Disable Register -------- */
#define PIO_MDDR_P0 (0x1u << 0) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P1 (0x1u << 1) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P2 (0x1u << 2) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P3 (0x1u << 3) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P4 (0x1u << 4) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P5 (0x1u << 5) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P6 (0x1u << 6) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P7 (0x1u << 7) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P8 (0x1u << 8) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P9 (0x1u << 9) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P10 (0x1u << 10) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P11 (0x1u << 11) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P12 (0x1u << 12) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P13 (0x1u << 13) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P14 (0x1u << 14) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P15 (0x1u << 15) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P16 (0x1u << 16) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P17 (0x1u << 17) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P18 (0x1u << 18) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P19 (0x1u << 19) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P20 (0x1u << 20) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P21 (0x1u << 21) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P22 (0x1u << 22) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P23 (0x1u << 23) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P24 (0x1u << 24) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P25 (0x1u << 25) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P26 (0x1u << 26) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P27 (0x1u << 27) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P28 (0x1u << 28) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P29 (0x1u << 29) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P30 (0x1u << 30) /**< \brief (PIO_MDDR) Multi-drive Disable */
#define PIO_MDDR_P31 (0x1u << 31) /**< \brief (PIO_MDDR) Multi-drive Disable */
/* -------- PIO_MDSR : (PIO Offset: 0x0058) Multi-driver Status Register -------- */
#define PIO_MDSR_P0 (0x1u << 0) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P1 (0x1u << 1) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P2 (0x1u << 2) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P3 (0x1u << 3) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P4 (0x1u << 4) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P5 (0x1u << 5) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P6 (0x1u << 6) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P7 (0x1u << 7) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P8 (0x1u << 8) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P9 (0x1u << 9) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P10 (0x1u << 10) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P11 (0x1u << 11) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P12 (0x1u << 12) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P13 (0x1u << 13) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P14 (0x1u << 14) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P15 (0x1u << 15) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P16 (0x1u << 16) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P17 (0x1u << 17) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P18 (0x1u << 18) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P19 (0x1u << 19) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P20 (0x1u << 20) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P21 (0x1u << 21) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P22 (0x1u << 22) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P23 (0x1u << 23) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P24 (0x1u << 24) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P25 (0x1u << 25) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P26 (0x1u << 26) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P27 (0x1u << 27) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P28 (0x1u << 28) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P29 (0x1u << 29) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P30 (0x1u << 30) /**< \brief (PIO_MDSR) Multi-drive Status */
#define PIO_MDSR_P31 (0x1u << 31) /**< \brief (PIO_MDSR) Multi-drive Status */
/* -------- PIO_PUDR : (PIO Offset: 0x0060) Pull-up Disable Register -------- */
#define PIO_PUDR_P0 (0x1u << 0) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P1 (0x1u << 1) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P2 (0x1u << 2) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P3 (0x1u << 3) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P4 (0x1u << 4) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P5 (0x1u << 5) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P6 (0x1u << 6) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P7 (0x1u << 7) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P8 (0x1u << 8) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P9 (0x1u << 9) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P10 (0x1u << 10) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P11 (0x1u << 11) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P12 (0x1u << 12) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P13 (0x1u << 13) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P14 (0x1u << 14) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P15 (0x1u << 15) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P16 (0x1u << 16) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P17 (0x1u << 17) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P18 (0x1u << 18) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P19 (0x1u << 19) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P20 (0x1u << 20) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P21 (0x1u << 21) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P22 (0x1u << 22) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P23 (0x1u << 23) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P24 (0x1u << 24) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P25 (0x1u << 25) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P26 (0x1u << 26) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P27 (0x1u << 27) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P28 (0x1u << 28) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P29 (0x1u << 29) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P30 (0x1u << 30) /**< \brief (PIO_PUDR) Pull-Up Disable */
#define PIO_PUDR_P31 (0x1u << 31) /**< \brief (PIO_PUDR) Pull-Up Disable */
/* -------- PIO_PUER : (PIO Offset: 0x0064) Pull-up Enable Register -------- */
#define PIO_PUER_P0 (0x1u << 0) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P1 (0x1u << 1) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P2 (0x1u << 2) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P3 (0x1u << 3) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P4 (0x1u << 4) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P5 (0x1u << 5) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P6 (0x1u << 6) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P7 (0x1u << 7) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P8 (0x1u << 8) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P9 (0x1u << 9) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P10 (0x1u << 10) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P11 (0x1u << 11) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P12 (0x1u << 12) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P13 (0x1u << 13) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P14 (0x1u << 14) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P15 (0x1u << 15) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P16 (0x1u << 16) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P17 (0x1u << 17) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P18 (0x1u << 18) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P19 (0x1u << 19) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P20 (0x1u << 20) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P21 (0x1u << 21) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P22 (0x1u << 22) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P23 (0x1u << 23) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P24 (0x1u << 24) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P25 (0x1u << 25) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P26 (0x1u << 26) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P27 (0x1u << 27) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P28 (0x1u << 28) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P29 (0x1u << 29) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P30 (0x1u << 30) /**< \brief (PIO_PUER) Pull-Up Enable */
#define PIO_PUER_P31 (0x1u << 31) /**< \brief (PIO_PUER) Pull-Up Enable */
/* -------- PIO_PUSR : (PIO Offset: 0x0068) Pad Pull-up Status Register -------- */
#define PIO_PUSR_P0 (0x1u << 0) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P1 (0x1u << 1) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P2 (0x1u << 2) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P3 (0x1u << 3) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P4 (0x1u << 4) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P5 (0x1u << 5) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P6 (0x1u << 6) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P7 (0x1u << 7) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P8 (0x1u << 8) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P9 (0x1u << 9) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P10 (0x1u << 10) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P11 (0x1u << 11) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P12 (0x1u << 12) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P13 (0x1u << 13) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P14 (0x1u << 14) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P15 (0x1u << 15) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P16 (0x1u << 16) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P17 (0x1u << 17) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P18 (0x1u << 18) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P19 (0x1u << 19) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P20 (0x1u << 20) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P21 (0x1u << 21) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P22 (0x1u << 22) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P23 (0x1u << 23) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P24 (0x1u << 24) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P25 (0x1u << 25) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P26 (0x1u << 26) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P27 (0x1u << 27) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P28 (0x1u << 28) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P29 (0x1u << 29) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P30 (0x1u << 30) /**< \brief (PIO_PUSR) Pull-Up Status */
#define PIO_PUSR_P31 (0x1u << 31) /**< \brief (PIO_PUSR) Pull-Up Status */
/* -------- PIO_ABCDSR[2] : (PIO Offset: 0x0070) Peripheral Select Register -------- */
#define PIO_ABCDSR_P0 (0x1u << 0) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P1 (0x1u << 1) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P2 (0x1u << 2) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P3 (0x1u << 3) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P4 (0x1u << 4) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P5 (0x1u << 5) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P6 (0x1u << 6) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P7 (0x1u << 7) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P8 (0x1u << 8) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P9 (0x1u << 9) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P10 (0x1u << 10) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P11 (0x1u << 11) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P12 (0x1u << 12) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P13 (0x1u << 13) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P14 (0x1u << 14) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P15 (0x1u << 15) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P16 (0x1u << 16) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P17 (0x1u << 17) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P18 (0x1u << 18) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P19 (0x1u << 19) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P20 (0x1u << 20) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P21 (0x1u << 21) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P22 (0x1u << 22) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P23 (0x1u << 23) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P24 (0x1u << 24) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P25 (0x1u << 25) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P26 (0x1u << 26) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P27 (0x1u << 27) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P28 (0x1u << 28) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P29 (0x1u << 29) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P30 (0x1u << 30) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
#define PIO_ABCDSR_P31 (0x1u << 31) /**< \brief (PIO_ABCDSR[2]) Peripheral Select */
/* -------- PIO_IFSCDR : (PIO Offset: 0x0080) Input Filter Slow Clock Disable Register -------- */
#define PIO_IFSCDR_P0 (0x1u << 0) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P1 (0x1u << 1) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P2 (0x1u << 2) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P3 (0x1u << 3) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P4 (0x1u << 4) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P5 (0x1u << 5) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P6 (0x1u << 6) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P7 (0x1u << 7) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P8 (0x1u << 8) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P9 (0x1u << 9) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P10 (0x1u << 10) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P11 (0x1u << 11) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P12 (0x1u << 12) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P13 (0x1u << 13) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P14 (0x1u << 14) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P15 (0x1u << 15) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P16 (0x1u << 16) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P17 (0x1u << 17) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P18 (0x1u << 18) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P19 (0x1u << 19) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P20 (0x1u << 20) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P21 (0x1u << 21) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P22 (0x1u << 22) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P23 (0x1u << 23) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P24 (0x1u << 24) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P25 (0x1u << 25) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P26 (0x1u << 26) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P27 (0x1u << 27) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P28 (0x1u << 28) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P29 (0x1u << 29) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P30 (0x1u << 30) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
#define PIO_IFSCDR_P31 (0x1u << 31) /**< \brief (PIO_IFSCDR) Peripheral Clock Glitch Filtering Select */
/* -------- PIO_IFSCER : (PIO Offset: 0x0084) Input Filter Slow Clock Enable Register -------- */
#define PIO_IFSCER_P0 (0x1u << 0) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P1 (0x1u << 1) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P2 (0x1u << 2) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P3 (0x1u << 3) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P4 (0x1u << 4) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P5 (0x1u << 5) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P6 (0x1u << 6) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P7 (0x1u << 7) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P8 (0x1u << 8) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P9 (0x1u << 9) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P10 (0x1u << 10) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P11 (0x1u << 11) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P12 (0x1u << 12) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P13 (0x1u << 13) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P14 (0x1u << 14) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P15 (0x1u << 15) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P16 (0x1u << 16) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P17 (0x1u << 17) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P18 (0x1u << 18) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P19 (0x1u << 19) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P20 (0x1u << 20) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P21 (0x1u << 21) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P22 (0x1u << 22) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P23 (0x1u << 23) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P24 (0x1u << 24) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P25 (0x1u << 25) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P26 (0x1u << 26) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P27 (0x1u << 27) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P28 (0x1u << 28) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P29 (0x1u << 29) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P30 (0x1u << 30) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
#define PIO_IFSCER_P31 (0x1u << 31) /**< \brief (PIO_IFSCER) Slow Clock Debouncing Filtering Select */
/* -------- PIO_IFSCSR : (PIO Offset: 0x0088) Input Filter Slow Clock Status Register -------- */
#define PIO_IFSCSR_P0 (0x1u << 0) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P1 (0x1u << 1) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P2 (0x1u << 2) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P3 (0x1u << 3) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P4 (0x1u << 4) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P5 (0x1u << 5) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P6 (0x1u << 6) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P7 (0x1u << 7) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P8 (0x1u << 8) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P9 (0x1u << 9) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P10 (0x1u << 10) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P11 (0x1u << 11) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P12 (0x1u << 12) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P13 (0x1u << 13) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P14 (0x1u << 14) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P15 (0x1u << 15) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P16 (0x1u << 16) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P17 (0x1u << 17) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P18 (0x1u << 18) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P19 (0x1u << 19) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P20 (0x1u << 20) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P21 (0x1u << 21) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P22 (0x1u << 22) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P23 (0x1u << 23) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P24 (0x1u << 24) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P25 (0x1u << 25) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P26 (0x1u << 26) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P27 (0x1u << 27) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P28 (0x1u << 28) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P29 (0x1u << 29) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P30 (0x1u << 30) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P31 (0x1u << 31) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
/* -------- PIO_SCDR : (PIO Offset: 0x008C) Slow Clock Divider Debouncing Register -------- */
#define PIO_SCDR_DIV_Pos 0
#define PIO_SCDR_DIV_Msk (0x3fffu << PIO_SCDR_DIV_Pos) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV(value) ((PIO_SCDR_DIV_Msk & ((value) << PIO_SCDR_DIV_Pos)))
/* -------- PIO_PPDDR : (PIO Offset: 0x0090) Pad Pull-down Disable Register -------- */
#define PIO_PPDDR_P0 (0x1u << 0) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P1 (0x1u << 1) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P2 (0x1u << 2) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P3 (0x1u << 3) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P4 (0x1u << 4) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P5 (0x1u << 5) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P6 (0x1u << 6) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P7 (0x1u << 7) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P8 (0x1u << 8) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P9 (0x1u << 9) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P10 (0x1u << 10) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P11 (0x1u << 11) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P12 (0x1u << 12) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P13 (0x1u << 13) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P14 (0x1u << 14) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P15 (0x1u << 15) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P16 (0x1u << 16) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P17 (0x1u << 17) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P18 (0x1u << 18) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P19 (0x1u << 19) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P20 (0x1u << 20) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P21 (0x1u << 21) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P22 (0x1u << 22) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P23 (0x1u << 23) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P24 (0x1u << 24) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P25 (0x1u << 25) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P26 (0x1u << 26) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P27 (0x1u << 27) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P28 (0x1u << 28) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P29 (0x1u << 29) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P30 (0x1u << 30) /**< \brief (PIO_PPDDR) Pull-Down Disable */
#define PIO_PPDDR_P31 (0x1u << 31) /**< \brief (PIO_PPDDR) Pull-Down Disable */
/* -------- PIO_PPDER : (PIO Offset: 0x0094) Pad Pull-down Enable Register -------- */
#define PIO_PPDER_P0 (0x1u << 0) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P1 (0x1u << 1) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P2 (0x1u << 2) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P3 (0x1u << 3) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P4 (0x1u << 4) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P5 (0x1u << 5) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P6 (0x1u << 6) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P7 (0x1u << 7) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P8 (0x1u << 8) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P9 (0x1u << 9) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P10 (0x1u << 10) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P11 (0x1u << 11) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P12 (0x1u << 12) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P13 (0x1u << 13) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P14 (0x1u << 14) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P15 (0x1u << 15) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P16 (0x1u << 16) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P17 (0x1u << 17) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P18 (0x1u << 18) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P19 (0x1u << 19) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P20 (0x1u << 20) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P21 (0x1u << 21) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P22 (0x1u << 22) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P23 (0x1u << 23) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P24 (0x1u << 24) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P25 (0x1u << 25) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P26 (0x1u << 26) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P27 (0x1u << 27) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P28 (0x1u << 28) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P29 (0x1u << 29) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P30 (0x1u << 30) /**< \brief (PIO_PPDER) Pull-Down Enable */
#define PIO_PPDER_P31 (0x1u << 31) /**< \brief (PIO_PPDER) Pull-Down Enable */
/* -------- PIO_PPDSR : (PIO Offset: 0x0098) Pad Pull-down Status Register -------- */
#define PIO_PPDSR_P0 (0x1u << 0) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P1 (0x1u << 1) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P2 (0x1u << 2) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P3 (0x1u << 3) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P4 (0x1u << 4) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P5 (0x1u << 5) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P6 (0x1u << 6) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P7 (0x1u << 7) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P8 (0x1u << 8) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P9 (0x1u << 9) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P10 (0x1u << 10) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P11 (0x1u << 11) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P12 (0x1u << 12) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P13 (0x1u << 13) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P14 (0x1u << 14) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P15 (0x1u << 15) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P16 (0x1u << 16) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P17 (0x1u << 17) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P18 (0x1u << 18) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P19 (0x1u << 19) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P20 (0x1u << 20) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P21 (0x1u << 21) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P22 (0x1u << 22) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P23 (0x1u << 23) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P24 (0x1u << 24) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P25 (0x1u << 25) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P26 (0x1u << 26) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P27 (0x1u << 27) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P28 (0x1u << 28) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P29 (0x1u << 29) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P30 (0x1u << 30) /**< \brief (PIO_PPDSR) Pull-Down Status */
#define PIO_PPDSR_P31 (0x1u << 31) /**< \brief (PIO_PPDSR) Pull-Down Status */
/* -------- PIO_OWER : (PIO Offset: 0x00A0) Output Write Enable -------- */
#define PIO_OWER_P0 (0x1u << 0) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P1 (0x1u << 1) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P2 (0x1u << 2) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P3 (0x1u << 3) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P4 (0x1u << 4) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P5 (0x1u << 5) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P6 (0x1u << 6) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P7 (0x1u << 7) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P8 (0x1u << 8) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P9 (0x1u << 9) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P10 (0x1u << 10) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P11 (0x1u << 11) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P12 (0x1u << 12) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P13 (0x1u << 13) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P14 (0x1u << 14) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P15 (0x1u << 15) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P16 (0x1u << 16) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P17 (0x1u << 17) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P18 (0x1u << 18) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P19 (0x1u << 19) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P20 (0x1u << 20) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P21 (0x1u << 21) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P22 (0x1u << 22) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P23 (0x1u << 23) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P24 (0x1u << 24) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P25 (0x1u << 25) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P26 (0x1u << 26) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P27 (0x1u << 27) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P28 (0x1u << 28) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P29 (0x1u << 29) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P30 (0x1u << 30) /**< \brief (PIO_OWER) Output Write Enable */
#define PIO_OWER_P31 (0x1u << 31) /**< \brief (PIO_OWER) Output Write Enable */
/* -------- PIO_OWDR : (PIO Offset: 0x00A4) Output Write Disable -------- */
#define PIO_OWDR_P0 (0x1u << 0) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P1 (0x1u << 1) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P2 (0x1u << 2) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P3 (0x1u << 3) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P4 (0x1u << 4) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P5 (0x1u << 5) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P6 (0x1u << 6) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P7 (0x1u << 7) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P8 (0x1u << 8) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P9 (0x1u << 9) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P10 (0x1u << 10) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P11 (0x1u << 11) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P12 (0x1u << 12) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P13 (0x1u << 13) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P14 (0x1u << 14) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P15 (0x1u << 15) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P16 (0x1u << 16) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P17 (0x1u << 17) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P18 (0x1u << 18) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P19 (0x1u << 19) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P20 (0x1u << 20) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P21 (0x1u << 21) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P22 (0x1u << 22) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P23 (0x1u << 23) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P24 (0x1u << 24) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P25 (0x1u << 25) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P26 (0x1u << 26) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P27 (0x1u << 27) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P28 (0x1u << 28) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P29 (0x1u << 29) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P30 (0x1u << 30) /**< \brief (PIO_OWDR) Output Write Disable */
#define PIO_OWDR_P31 (0x1u << 31) /**< \brief (PIO_OWDR) Output Write Disable */
/* -------- PIO_OWSR : (PIO Offset: 0x00A8) Output Write Status Register -------- */
#define PIO_OWSR_P0 (0x1u << 0) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P1 (0x1u << 1) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P2 (0x1u << 2) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P3 (0x1u << 3) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P4 (0x1u << 4) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P5 (0x1u << 5) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P6 (0x1u << 6) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P7 (0x1u << 7) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P8 (0x1u << 8) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P9 (0x1u << 9) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P10 (0x1u << 10) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P11 (0x1u << 11) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P12 (0x1u << 12) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P13 (0x1u << 13) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P14 (0x1u << 14) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P15 (0x1u << 15) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P16 (0x1u << 16) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P17 (0x1u << 17) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P18 (0x1u << 18) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P19 (0x1u << 19) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P20 (0x1u << 20) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P21 (0x1u << 21) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P22 (0x1u << 22) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P23 (0x1u << 23) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P24 (0x1u << 24) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P25 (0x1u << 25) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P26 (0x1u << 26) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P27 (0x1u << 27) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P28 (0x1u << 28) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P29 (0x1u << 29) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P30 (0x1u << 30) /**< \brief (PIO_OWSR) Output Write Status */
#define PIO_OWSR_P31 (0x1u << 31) /**< \brief (PIO_OWSR) Output Write Status */
/* -------- PIO_AIMER : (PIO Offset: 0x00B0) Additional Interrupt Modes Enable Register -------- */
#define PIO_AIMER_P0 (0x1u << 0) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P1 (0x1u << 1) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P2 (0x1u << 2) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P3 (0x1u << 3) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P4 (0x1u << 4) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P5 (0x1u << 5) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P6 (0x1u << 6) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P7 (0x1u << 7) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P8 (0x1u << 8) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P9 (0x1u << 9) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P10 (0x1u << 10) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P11 (0x1u << 11) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P12 (0x1u << 12) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P13 (0x1u << 13) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P14 (0x1u << 14) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P15 (0x1u << 15) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P16 (0x1u << 16) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P17 (0x1u << 17) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P18 (0x1u << 18) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P19 (0x1u << 19) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P20 (0x1u << 20) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P21 (0x1u << 21) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P22 (0x1u << 22) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P23 (0x1u << 23) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P24 (0x1u << 24) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P25 (0x1u << 25) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P26 (0x1u << 26) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P27 (0x1u << 27) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P28 (0x1u << 28) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P29 (0x1u << 29) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P30 (0x1u << 30) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
#define PIO_AIMER_P31 (0x1u << 31) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable */
/* -------- PIO_AIMDR : (PIO Offset: 0x00B4) Additional Interrupt Modes Disable Register -------- */
#define PIO_AIMDR_P0 (0x1u << 0) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P1 (0x1u << 1) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P2 (0x1u << 2) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P3 (0x1u << 3) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P4 (0x1u << 4) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P5 (0x1u << 5) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P6 (0x1u << 6) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P7 (0x1u << 7) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P8 (0x1u << 8) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P9 (0x1u << 9) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P10 (0x1u << 10) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P11 (0x1u << 11) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P12 (0x1u << 12) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P13 (0x1u << 13) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P14 (0x1u << 14) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P15 (0x1u << 15) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P16 (0x1u << 16) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P17 (0x1u << 17) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P18 (0x1u << 18) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P19 (0x1u << 19) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P20 (0x1u << 20) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P21 (0x1u << 21) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P22 (0x1u << 22) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P23 (0x1u << 23) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P24 (0x1u << 24) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P25 (0x1u << 25) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P26 (0x1u << 26) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P27 (0x1u << 27) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P28 (0x1u << 28) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P29 (0x1u << 29) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P30 (0x1u << 30) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
#define PIO_AIMDR_P31 (0x1u << 31) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable */
/* -------- PIO_AIMMR : (PIO Offset: 0x00B8) Additional Interrupt Modes Mask Register -------- */
#define PIO_AIMMR_P0 (0x1u << 0) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P1 (0x1u << 1) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P2 (0x1u << 2) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P3 (0x1u << 3) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P4 (0x1u << 4) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P5 (0x1u << 5) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P6 (0x1u << 6) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P7 (0x1u << 7) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P8 (0x1u << 8) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P9 (0x1u << 9) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P10 (0x1u << 10) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P11 (0x1u << 11) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P12 (0x1u << 12) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P13 (0x1u << 13) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P14 (0x1u << 14) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P15 (0x1u << 15) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P16 (0x1u << 16) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P17 (0x1u << 17) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P18 (0x1u << 18) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P19 (0x1u << 19) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P20 (0x1u << 20) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P21 (0x1u << 21) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P22 (0x1u << 22) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P23 (0x1u << 23) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P24 (0x1u << 24) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P25 (0x1u << 25) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P26 (0x1u << 26) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P27 (0x1u << 27) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P28 (0x1u << 28) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P29 (0x1u << 29) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P30 (0x1u << 30) /**< \brief (PIO_AIMMR) IO Line Index */
#define PIO_AIMMR_P31 (0x1u << 31) /**< \brief (PIO_AIMMR) IO Line Index */
/* -------- PIO_ESR : (PIO Offset: 0x00C0) Edge Select Register -------- */
#define PIO_ESR_P0 (0x1u << 0) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P1 (0x1u << 1) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P2 (0x1u << 2) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P3 (0x1u << 3) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P4 (0x1u << 4) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P5 (0x1u << 5) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P6 (0x1u << 6) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P7 (0x1u << 7) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P8 (0x1u << 8) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P9 (0x1u << 9) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P10 (0x1u << 10) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P11 (0x1u << 11) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P12 (0x1u << 12) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P13 (0x1u << 13) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P14 (0x1u << 14) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P15 (0x1u << 15) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P16 (0x1u << 16) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P17 (0x1u << 17) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P18 (0x1u << 18) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P19 (0x1u << 19) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P20 (0x1u << 20) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P21 (0x1u << 21) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P22 (0x1u << 22) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P23 (0x1u << 23) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P24 (0x1u << 24) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P25 (0x1u << 25) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P26 (0x1u << 26) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P27 (0x1u << 27) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P28 (0x1u << 28) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P29 (0x1u << 29) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P30 (0x1u << 30) /**< \brief (PIO_ESR) Edge Interrupt Selection */
#define PIO_ESR_P31 (0x1u << 31) /**< \brief (PIO_ESR) Edge Interrupt Selection */
/* -------- PIO_LSR : (PIO Offset: 0x00C4) Level Select Register -------- */
#define PIO_LSR_P0 (0x1u << 0) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P1 (0x1u << 1) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P2 (0x1u << 2) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P3 (0x1u << 3) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P4 (0x1u << 4) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P5 (0x1u << 5) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P6 (0x1u << 6) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P7 (0x1u << 7) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P8 (0x1u << 8) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P9 (0x1u << 9) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P10 (0x1u << 10) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P11 (0x1u << 11) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P12 (0x1u << 12) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P13 (0x1u << 13) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P14 (0x1u << 14) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P15 (0x1u << 15) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P16 (0x1u << 16) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P17 (0x1u << 17) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P18 (0x1u << 18) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P19 (0x1u << 19) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P20 (0x1u << 20) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P21 (0x1u << 21) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P22 (0x1u << 22) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P23 (0x1u << 23) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P24 (0x1u << 24) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P25 (0x1u << 25) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P26 (0x1u << 26) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P27 (0x1u << 27) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P28 (0x1u << 28) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P29 (0x1u << 29) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P30 (0x1u << 30) /**< \brief (PIO_LSR) Level Interrupt Selection */
#define PIO_LSR_P31 (0x1u << 31) /**< \brief (PIO_LSR) Level Interrupt Selection */
/* -------- PIO_ELSR : (PIO Offset: 0x00C8) Edge/Level Status Register -------- */
#define PIO_ELSR_P0 (0x1u << 0) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P1 (0x1u << 1) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P2 (0x1u << 2) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P3 (0x1u << 3) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P4 (0x1u << 4) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P5 (0x1u << 5) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P6 (0x1u << 6) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P7 (0x1u << 7) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P8 (0x1u << 8) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P9 (0x1u << 9) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P10 (0x1u << 10) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P11 (0x1u << 11) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P12 (0x1u << 12) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P13 (0x1u << 13) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P14 (0x1u << 14) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P15 (0x1u << 15) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P16 (0x1u << 16) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P17 (0x1u << 17) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P18 (0x1u << 18) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P19 (0x1u << 19) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P20 (0x1u << 20) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P21 (0x1u << 21) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P22 (0x1u << 22) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P23 (0x1u << 23) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P24 (0x1u << 24) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P25 (0x1u << 25) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P26 (0x1u << 26) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P27 (0x1u << 27) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P28 (0x1u << 28) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P29 (0x1u << 29) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P30 (0x1u << 30) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
#define PIO_ELSR_P31 (0x1u << 31) /**< \brief (PIO_ELSR) Edge/Level Interrupt Source Selection */
/* -------- PIO_FELLSR : (PIO Offset: 0x00D0) Falling Edge/Low-Level Select Register -------- */
#define PIO_FELLSR_P0 (0x1u << 0) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P1 (0x1u << 1) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P2 (0x1u << 2) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P3 (0x1u << 3) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P4 (0x1u << 4) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P5 (0x1u << 5) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P6 (0x1u << 6) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P7 (0x1u << 7) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P8 (0x1u << 8) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P9 (0x1u << 9) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P10 (0x1u << 10) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P11 (0x1u << 11) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P12 (0x1u << 12) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P13 (0x1u << 13) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P14 (0x1u << 14) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P15 (0x1u << 15) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P16 (0x1u << 16) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P17 (0x1u << 17) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P18 (0x1u << 18) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P19 (0x1u << 19) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P20 (0x1u << 20) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P21 (0x1u << 21) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P22 (0x1u << 22) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P23 (0x1u << 23) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P24 (0x1u << 24) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P25 (0x1u << 25) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P26 (0x1u << 26) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P27 (0x1u << 27) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P28 (0x1u << 28) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P29 (0x1u << 29) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P30 (0x1u << 30) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
#define PIO_FELLSR_P31 (0x1u << 31) /**< \brief (PIO_FELLSR) Falling Edge/Low-Level Interrupt Selection */
/* -------- PIO_REHLSR : (PIO Offset: 0x00D4) Rising Edge/High-Level Select Register -------- */
#define PIO_REHLSR_P0 (0x1u << 0) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P1 (0x1u << 1) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P2 (0x1u << 2) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P3 (0x1u << 3) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P4 (0x1u << 4) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P5 (0x1u << 5) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P6 (0x1u << 6) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P7 (0x1u << 7) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P8 (0x1u << 8) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P9 (0x1u << 9) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P10 (0x1u << 10) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P11 (0x1u << 11) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P12 (0x1u << 12) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P13 (0x1u << 13) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P14 (0x1u << 14) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P15 (0x1u << 15) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P16 (0x1u << 16) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P17 (0x1u << 17) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P18 (0x1u << 18) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P19 (0x1u << 19) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P20 (0x1u << 20) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P21 (0x1u << 21) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P22 (0x1u << 22) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P23 (0x1u << 23) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P24 (0x1u << 24) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P25 (0x1u << 25) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P26 (0x1u << 26) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P27 (0x1u << 27) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P28 (0x1u << 28) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P29 (0x1u << 29) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P30 (0x1u << 30) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
#define PIO_REHLSR_P31 (0x1u << 31) /**< \brief (PIO_REHLSR) Rising Edge/High-Level Interrupt Selection */
/* -------- PIO_FRLHSR : (PIO Offset: 0x00D8) Fall/Rise - Low/High Status Register -------- */
#define PIO_FRLHSR_P0 (0x1u << 0) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P1 (0x1u << 1) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P2 (0x1u << 2) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P3 (0x1u << 3) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P4 (0x1u << 4) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P5 (0x1u << 5) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P6 (0x1u << 6) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P7 (0x1u << 7) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P8 (0x1u << 8) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P9 (0x1u << 9) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P10 (0x1u << 10) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P11 (0x1u << 11) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P12 (0x1u << 12) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P13 (0x1u << 13) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P14 (0x1u << 14) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P15 (0x1u << 15) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P16 (0x1u << 16) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P17 (0x1u << 17) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P18 (0x1u << 18) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P19 (0x1u << 19) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P20 (0x1u << 20) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P21 (0x1u << 21) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P22 (0x1u << 22) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P23 (0x1u << 23) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P24 (0x1u << 24) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P25 (0x1u << 25) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P26 (0x1u << 26) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P27 (0x1u << 27) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P28 (0x1u << 28) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P29 (0x1u << 29) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P30 (0x1u << 30) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
#define PIO_FRLHSR_P31 (0x1u << 31) /**< \brief (PIO_FRLHSR) Edge/Level Interrupt Source Selection */
/* -------- PIO_LOCKSR : (PIO Offset: 0x00E0) Lock Status -------- */
#define PIO_LOCKSR_P0 (0x1u << 0) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P1 (0x1u << 1) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P2 (0x1u << 2) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P3 (0x1u << 3) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P4 (0x1u << 4) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P5 (0x1u << 5) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P6 (0x1u << 6) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P7 (0x1u << 7) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P8 (0x1u << 8) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P9 (0x1u << 9) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P10 (0x1u << 10) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P11 (0x1u << 11) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P12 (0x1u << 12) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P13 (0x1u << 13) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P14 (0x1u << 14) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P15 (0x1u << 15) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P16 (0x1u << 16) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P17 (0x1u << 17) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P18 (0x1u << 18) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P19 (0x1u << 19) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P20 (0x1u << 20) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P21 (0x1u << 21) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P22 (0x1u << 22) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P23 (0x1u << 23) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P24 (0x1u << 24) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P25 (0x1u << 25) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P26 (0x1u << 26) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P27 (0x1u << 27) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P28 (0x1u << 28) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P29 (0x1u << 29) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P30 (0x1u << 30) /**< \brief (PIO_LOCKSR) Lock Status */
#define PIO_LOCKSR_P31 (0x1u << 31) /**< \brief (PIO_LOCKSR) Lock Status */
/* -------- PIO_WPMR : (PIO Offset: 0x00E4) Write Protection Mode Register -------- */
#define PIO_WPMR_WPEN (0x1u << 0) /**< \brief (PIO_WPMR) Write Protection Enable */
#define PIO_WPMR_WPKEY_Pos 8
#define PIO_WPMR_WPKEY_Msk (0xffffffu << PIO_WPMR_WPKEY_Pos) /**< \brief (PIO_WPMR) Write Protection Key */
#define PIO_WPMR_WPKEY(value) ((PIO_WPMR_WPKEY_Msk & ((value) << PIO_WPMR_WPKEY_Pos)))
#define   PIO_WPMR_WPKEY_PASSWD (0x50494Fu << 8) /**< \brief (PIO_WPMR) Writing any other value in this field aborts the write operation of the WPEN bit. Always reads as 0. */
/* -------- PIO_WPSR : (PIO Offset: 0x00E8) Write Protection Status Register -------- */
#define PIO_WPSR_WPVS (0x1u << 0) /**< \brief (PIO_WPSR) Write Protection Violation Status */
#define PIO_WPSR_WPVSRC_Pos 8
#define PIO_WPSR_WPVSRC_Msk (0xffffu << PIO_WPSR_WPVSRC_Pos) /**< \brief (PIO_WPSR) Write Protection Violation Source */
/* -------- PIO_VERSION : (PIO Offset: 0x00FC) Version Register -------- */
#define PIO_VERSION_VERSION_Pos 0
#define PIO_VERSION_VERSION_Msk (0xfffu << PIO_VERSION_VERSION_Pos) /**< \brief (PIO_VERSION) Hardware Module Version */
#define PIO_VERSION_MFN_Pos 16
#define PIO_VERSION_MFN_Msk (0x7u << PIO_VERSION_MFN_Pos) /**< \brief (PIO_VERSION) Metal Fix Number */
/* -------- PIO_SCHMITT : (PIO Offset: 0x0100) Schmitt Trigger Register -------- */
#define PIO_SCHMITT_SCHMITT0 (0x1u << 0) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT1 (0x1u << 1) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT2 (0x1u << 2) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT3 (0x1u << 3) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT4 (0x1u << 4) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT5 (0x1u << 5) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT6 (0x1u << 6) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT7 (0x1u << 7) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT8 (0x1u << 8) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT9 (0x1u << 9) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT10 (0x1u << 10) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT11 (0x1u << 11) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT12 (0x1u << 12) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT13 (0x1u << 13) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT14 (0x1u << 14) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT15 (0x1u << 15) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT16 (0x1u << 16) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT17 (0x1u << 17) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT18 (0x1u << 18) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT19 (0x1u << 19) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT20 (0x1u << 20) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT21 (0x1u << 21) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT22 (0x1u << 22) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT23 (0x1u << 23) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT24 (0x1u << 24) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT25 (0x1u << 25) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT26 (0x1u << 26) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT27 (0x1u << 27) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT28 (0x1u << 28) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT29 (0x1u << 29) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT30 (0x1u << 30) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
#define PIO_SCHMITT_SCHMITT31 (0x1u << 31) /**< \brief (PIO_SCHMITT) Schmitt Trigger Control */
/* -------- PIO_DRIVER : (PIO Offset: 0x0118) I/O Drive Register -------- */
#define PIO_DRIVER_LINE0 (0x1u << 0) /**< \brief (PIO_DRIVER) Drive of PIO Line 0 */
#define   PIO_DRIVER_LINE0_LOW_DRIVE (0x0u << 0) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE0_HIGH_DRIVE (0x1u << 0) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE1 (0x1u << 1) /**< \brief (PIO_DRIVER) Drive of PIO Line 1 */
#define   PIO_DRIVER_LINE1_LOW_DRIVE (0x0u << 1) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE1_HIGH_DRIVE (0x1u << 1) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE2 (0x1u << 2) /**< \brief (PIO_DRIVER) Drive of PIO Line 2 */
#define   PIO_DRIVER_LINE2_LOW_DRIVE (0x0u << 2) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE2_HIGH_DRIVE (0x1u << 2) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE3 (0x1u << 3) /**< \brief (PIO_DRIVER) Drive of PIO Line 3 */
#define   PIO_DRIVER_LINE3_LOW_DRIVE (0x0u << 3) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE3_HIGH_DRIVE (0x1u << 3) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE4 (0x1u << 4) /**< \brief (PIO_DRIVER) Drive of PIO Line 4 */
#define   PIO_DRIVER_LINE4_LOW_DRIVE (0x0u << 4) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE4_HIGH_DRIVE (0x1u << 4) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE5 (0x1u << 5) /**< \brief (PIO_DRIVER) Drive of PIO Line 5 */
#define   PIO_DRIVER_LINE5_LOW_DRIVE (0x0u << 5) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE5_HIGH_DRIVE (0x1u << 5) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE6 (0x1u << 6) /**< \brief (PIO_DRIVER) Drive of PIO Line 6 */
#define   PIO_DRIVER_LINE6_LOW_DRIVE (0x0u << 6) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE6_HIGH_DRIVE (0x1u << 6) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE7 (0x1u << 7) /**< \brief (PIO_DRIVER) Drive of PIO Line 7 */
#define   PIO_DRIVER_LINE7_LOW_DRIVE (0x0u << 7) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE7_HIGH_DRIVE (0x1u << 7) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE8 (0x1u << 8) /**< \brief (PIO_DRIVER) Drive of PIO Line 8 */
#define   PIO_DRIVER_LINE8_LOW_DRIVE (0x0u << 8) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE8_HIGH_DRIVE (0x1u << 8) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE9 (0x1u << 9) /**< \brief (PIO_DRIVER) Drive of PIO Line 9 */
#define   PIO_DRIVER_LINE9_LOW_DRIVE (0x0u << 9) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE9_HIGH_DRIVE (0x1u << 9) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE10 (0x1u << 10) /**< \brief (PIO_DRIVER) Drive of PIO Line 10 */
#define   PIO_DRIVER_LINE10_LOW_DRIVE (0x0u << 10) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE10_HIGH_DRIVE (0x1u << 10) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE11 (0x1u << 11) /**< \brief (PIO_DRIVER) Drive of PIO Line 11 */
#define   PIO_DRIVER_LINE11_LOW_DRIVE (0x0u << 11) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE11_HIGH_DRIVE (0x1u << 11) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE12 (0x1u << 12) /**< \brief (PIO_DRIVER) Drive of PIO Line 12 */
#define   PIO_DRIVER_LINE12_LOW_DRIVE (0x0u << 12) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE12_HIGH_DRIVE (0x1u << 12) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE13 (0x1u << 13) /**< \brief (PIO_DRIVER) Drive of PIO Line 13 */
#define   PIO_DRIVER_LINE13_LOW_DRIVE (0x0u << 13) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE13_HIGH_DRIVE (0x1u << 13) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE14 (0x1u << 14) /**< \brief (PIO_DRIVER) Drive of PIO Line 14 */
#define   PIO_DRIVER_LINE14_LOW_DRIVE (0x0u << 14) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE14_HIGH_DRIVE (0x1u << 14) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE15 (0x1u << 15) /**< \brief (PIO_DRIVER) Drive of PIO Line 15 */
#define   PIO_DRIVER_LINE15_LOW_DRIVE (0x0u << 15) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE15_HIGH_DRIVE (0x1u << 15) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE16 (0x1u << 16) /**< \brief (PIO_DRIVER) Drive of PIO Line 16 */
#define   PIO_DRIVER_LINE16_LOW_DRIVE (0x0u << 16) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE16_HIGH_DRIVE (0x1u << 16) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE17 (0x1u << 17) /**< \brief (PIO_DRIVER) Drive of PIO Line 17 */
#define   PIO_DRIVER_LINE17_LOW_DRIVE (0x0u << 17) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE17_HIGH_DRIVE (0x1u << 17) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE18 (0x1u << 18) /**< \brief (PIO_DRIVER) Drive of PIO Line 18 */
#define   PIO_DRIVER_LINE18_LOW_DRIVE (0x0u << 18) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE18_HIGH_DRIVE (0x1u << 18) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE19 (0x1u << 19) /**< \brief (PIO_DRIVER) Drive of PIO Line 19 */
#define   PIO_DRIVER_LINE19_LOW_DRIVE (0x0u << 19) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE19_HIGH_DRIVE (0x1u << 19) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE20 (0x1u << 20) /**< \brief (PIO_DRIVER) Drive of PIO Line 20 */
#define   PIO_DRIVER_LINE20_LOW_DRIVE (0x0u << 20) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE20_HIGH_DRIVE (0x1u << 20) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE21 (0x1u << 21) /**< \brief (PIO_DRIVER) Drive of PIO Line 21 */
#define   PIO_DRIVER_LINE21_LOW_DRIVE (0x0u << 21) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE21_HIGH_DRIVE (0x1u << 21) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE22 (0x1u << 22) /**< \brief (PIO_DRIVER) Drive of PIO Line 22 */
#define   PIO_DRIVER_LINE22_LOW_DRIVE (0x0u << 22) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE22_HIGH_DRIVE (0x1u << 22) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE23 (0x1u << 23) /**< \brief (PIO_DRIVER) Drive of PIO Line 23 */
#define   PIO_DRIVER_LINE23_LOW_DRIVE (0x0u << 23) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE23_HIGH_DRIVE (0x1u << 23) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE24 (0x1u << 24) /**< \brief (PIO_DRIVER) Drive of PIO Line 24 */
#define   PIO_DRIVER_LINE24_LOW_DRIVE (0x0u << 24) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE24_HIGH_DRIVE (0x1u << 24) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE25 (0x1u << 25) /**< \brief (PIO_DRIVER) Drive of PIO Line 25 */
#define   PIO_DRIVER_LINE25_LOW_DRIVE (0x0u << 25) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE25_HIGH_DRIVE (0x1u << 25) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE26 (0x1u << 26) /**< \brief (PIO_DRIVER) Drive of PIO Line 26 */
#define   PIO_DRIVER_LINE26_LOW_DRIVE (0x0u << 26) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE26_HIGH_DRIVE (0x1u << 26) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE27 (0x1u << 27) /**< \brief (PIO_DRIVER) Drive of PIO Line 27 */
#define   PIO_DRIVER_LINE27_LOW_DRIVE (0x0u << 27) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE27_HIGH_DRIVE (0x1u << 27) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE28 (0x1u << 28) /**< \brief (PIO_DRIVER) Drive of PIO Line 28 */
#define   PIO_DRIVER_LINE28_LOW_DRIVE (0x0u << 28) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE28_HIGH_DRIVE (0x1u << 28) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE29 (0x1u << 29) /**< \brief (PIO_DRIVER) Drive of PIO Line 29 */
#define   PIO_DRIVER_LINE29_LOW_DRIVE (0x0u << 29) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE29_HIGH_DRIVE (0x1u << 29) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE30 (0x1u << 30) /**< \brief (PIO_DRIVER) Drive of PIO Line 30 */
#define   PIO_DRIVER_LINE30_LOW_DRIVE (0x0u << 30) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE30_HIGH_DRIVE (0x1u << 30) /**< \brief (PIO_DRIVER) Highest drive */
#define PIO_DRIVER_LINE31 (0x1u << 31) /**< \brief (PIO_DRIVER) Drive of PIO Line 31 */
#define   PIO_DRIVER_LINE31_LOW_DRIVE (0x0u << 31) /**< \brief (PIO_DRIVER) Lowest drive */
#define   PIO_DRIVER_LINE31_HIGH_DRIVE (0x1u << 31) /**< \brief (PIO_DRIVER) Highest drive */
/* -------- PIO_PCMR : (PIO Offset: 0x0150) Parallel Capture Mode Register -------- */
#define PIO_PCMR_PCEN (0x1u << 0) /**< \brief (PIO_PCMR) Parallel Capture Mode Enable */
#define PIO_PCMR_DSIZE_Pos 4
#define PIO_PCMR_DSIZE_Msk (0x3u << PIO_PCMR_DSIZE_Pos) /**< \brief (PIO_PCMR) Parallel Capture Mode Data Size */
#define PIO_PCMR_DSIZE(value) ((PIO_PCMR_DSIZE_Msk & ((value) << PIO_PCMR_DSIZE_Pos)))
#define   PIO_PCMR_DSIZE_BYTE (0x0u << 4) /**< \brief (PIO_PCMR) The reception data in the PIO_PCRHR is a byte (8-bit) */
#define   PIO_PCMR_DSIZE_HALFWORD (0x1u << 4) /**< \brief (PIO_PCMR) The reception data in the PIO_PCRHR is a half-word (16-bit) */
#define   PIO_PCMR_DSIZE_WORD (0x2u << 4) /**< \brief (PIO_PCMR) The reception data in the PIO_PCRHR is a word (32-bit) */
#define PIO_PCMR_ALWYS (0x1u << 9) /**< \brief (PIO_PCMR) Parallel Capture Mode Always Sampling */
#define PIO_PCMR_HALFS (0x1u << 10) /**< \brief (PIO_PCMR) Parallel Capture Mode Half Sampling */
#define PIO_PCMR_FRSTS (0x1u << 11) /**< \brief (PIO_PCMR) Parallel Capture Mode First Sample */
/* -------- PIO_PCIER : (PIO Offset: 0x0154) Parallel Capture Interrupt Enable Register -------- */
#define PIO_PCIER_DRDY (0x1u << 0) /**< \brief (PIO_PCIER) Parallel Capture Mode Data Ready Interrupt Enable */
#define PIO_PCIER_OVRE (0x1u << 1) /**< \brief (PIO_PCIER) Parallel Capture Mode Overrun Error Interrupt Enable */
#define PIO_PCIER_ENDRX (0x1u << 2) /**< \brief (PIO_PCIER) End of Reception Transfer Interrupt Enable */
#define PIO_PCIER_RXBUFF (0x1u << 3) /**< \brief (PIO_PCIER) Reception Buffer Full Interrupt Enable */
/* -------- PIO_PCIDR : (PIO Offset: 0x0158) Parallel Capture Interrupt Disable Register -------- */
#define PIO_PCIDR_DRDY (0x1u << 0) /**< \brief (PIO_PCIDR) Parallel Capture Mode Data Ready Interrupt Disable */
#define PIO_PCIDR_OVRE (0x1u << 1) /**< \brief (PIO_PCIDR) Parallel Capture Mode Overrun Error Interrupt Disable */
#define PIO_PCIDR_ENDRX (0x1u << 2) /**< \brief (PIO_PCIDR) End of Reception Transfer Interrupt Disable */
#define PIO_PCIDR_RXBUFF (0x1u << 3) /**< \brief (PIO_PCIDR) Reception Buffer Full Interrupt Disable */
/* -------- PIO_PCIMR : (PIO Offset: 0x015C) Parallel Capture Interrupt Mask Register -------- */
#define PIO_PCIMR_DRDY (0x1u << 0) /**< \brief (PIO_PCIMR) Parallel Capture Mode Data Ready Interrupt Mask */
#define PIO_PCIMR_OVRE (0x1u << 1) /**< \brief (PIO_PCIMR) Parallel Capture Mode Overrun Error Interrupt Mask */
#define PIO_PCIMR_ENDRX (0x1u << 2) /**< \brief (PIO_PCIMR) End of Reception Transfer Interrupt Mask */
#define PIO_PCIMR_RXBUFF (0x1u << 3) /**< \brief (PIO_PCIMR) Reception Buffer Full Interrupt Mask */
/* -------- PIO_PCISR : (PIO Offset: 0x0160) Parallel Capture Interrupt Status Register -------- */
#define PIO_PCISR_DRDY (0x1u << 0) /**< \brief (PIO_PCISR) Parallel Capture Mode Data Ready */
#define PIO_PCISR_OVRE (0x1u << 1) /**< \brief (PIO_PCISR) Parallel Capture Mode Overrun Error */
/* -------- PIO_PCRHR : (PIO Offset: 0x0164) Parallel Capture Reception Holding Register -------- */
#define PIO_PCRHR_RDATA_Pos 0
#define PIO_PCRHR_RDATA_Msk (0xffffffffu << PIO_PCRHR_RDATA_Pos) /**< \brief (PIO_PCRHR) Parallel Capture Mode Reception Data */

/*@}*/


#endif /* _SAME70_PIO_COMPONENT_ */

/* ************************************************************************** */
/*   REGISTER ACCESS DEFINITIONS FOR SAME70Q20 */
/* ************************************************************************** */
/** \addtogroup SAME70Q20_reg Registers Access Definitions */
/*@{*/

/*
#include "instance/hsmci.h"
#include "instance/ssc.h"
#include "instance/spi0.h"
#include "instance/tc0.h"
#include "instance/tc1.h"
#include "instance/tc2.h"
#include "instance/twihs0.h"
#include "instance/twihs1.h"
#include "instance/pwm0.h"
#include "instance/usart0.h"
#include "instance/usart1.h"
#include "instance/usart2.h"
#include "instance/mcan0.h"
#include "instance/mcan1.h"
#include "instance/usbhs.h"
#include "instance/afec0.h"
#include "instance/dacc.h"
#include "instance/acc.h"
#include "instance/icm.h"
#include "instance/isi.h"
#include "instance/gmac.h"
#include "instance/tc3.h"
#include "instance/spi1.h"
#include "instance/pwm1.h"
#include "instance/twihs2.h"
#include "instance/afec1.h"
#include "instance/aes.h"
#include "instance/trng.h"
#include "instance/xdmac.h"
#include "instance/qspi.h"
#include "instance/smc.h"
#include "instance/sdramc.h"
#include "instance/matrix.h"
#include "instance/utmi.h"
#include "instance/pmc.h"
#include "instance/uart0.h"
#include "instance/chipid.h"
#include "instance/uart1.h"
#include "instance/efc.h"
#include "instance/pioa.h"
#include "instance/piob.h"
#include "instance/pioc.h"
#include "instance/piod.h"
#include "instance/pioe.h"
#include "instance/rstc.h"
#include "instance/supc.h"
#include "instance/rtt.h"
#include "instance/wdt.h"
#include "instance/rtc.h"
#include "instance/gpbr.h"
#include "instance/rswdt.h"
#include "instance/uart2.h"
#include "instance/uart3.h"
#include "instance/uart4.h"
*/
/*@}*/

/* ************************************************************************** */
/*   PERIPHERAL ID DEFINITIONS FOR SAME70Q20 */
/* ************************************************************************** */
/** \addtogroup SAME70Q20_id Peripheral Ids Definitions */
/*@{*/

#define ID_SUPC   ( 0) /**< \brief Supply Controller (SUPC) */
#define ID_RSTC   ( 1) /**< \brief Reset Controller (RSTC) */
#define ID_RTC    ( 2) /**< \brief Real Time Clock (RTC) */
#define ID_RTT    ( 3) /**< \brief Real Time Timer (RTT) */
#define ID_WDT    ( 4) /**< \brief Watchdog Timer (WDT) */
#define ID_PMC    ( 5) /**< \brief Power Management Controller (PMC) */
#define ID_EFC    ( 6) /**< \brief Enhanced Embedded Flash Controller (EFC) */
#define ID_UART0  ( 7) /**< \brief UART 0 (UART0) */
#define ID_UART1  ( 8) /**< \brief UART 1 (UART1) */
#define ID_SMC    ( 9) /**< \brief Static Memory Controller (SMC) */
#define ID_PIOA   (10) /**< \brief Parallel I/O Controller A (PIOA) */
#define ID_PIOB   (11) /**< \brief Parallel I/O Controller B (PIOB) */
#define ID_PIOC   (12) /**< \brief Parallel I/O Controller C (PIOC) */
#define ID_USART0 (13) /**< \brief USART 0 (USART0) */
#define ID_USART1 (14) /**< \brief USART 1 (USART1) */
#define ID_USART2 (15) /**< \brief USART 2 (USART2) */
#define ID_PIOD   (16) /**< \brief Parallel I/O Controller D (PIOD) */
#define ID_PIOE   (17) /**< \brief Parallel I/O Controller E (PIOE) */
#define ID_HSMCI  (18) /**< \brief Multimedia Card Interface (HSMCI) */
#define ID_TWIHS0 (19) /**< \brief Two Wire Interface 0 HS (TWIHS0) */
#define ID_TWIHS1 (20) /**< \brief Two Wire Interface 1 HS (TWIHS1) */
#define ID_SPI0   (21) /**< \brief Serial Peripheral Interface 0 (SPI0) */
#define ID_SSC    (22) /**< \brief Synchronous Serial Controller (SSC) */
#define ID_TC0    (23) /**< \brief Timer/Counter 0 (TC0) */
#define ID_TC1    (24) /**< \brief Timer/Counter 1 (TC1) */
#define ID_TC2    (25) /**< \brief Timer/Counter 2 (TC2) */
#define ID_TC3    (26) /**< \brief Timer/Counter 3 (TC3) */
#define ID_TC4    (27) /**< \brief Timer/Counter 4 (TC4) */
#define ID_TC5    (28) /**< \brief Timer/Counter 5 (TC5) */
#define ID_AFEC0  (29) /**< \brief Analog Front End 0 (AFEC0) */
#define ID_DACC   (30) /**< \brief Digital To Analog Converter (DACC) */
#define ID_PWM0   (31) /**< \brief Pulse Width Modulation 0 (PWM0) */
#define ID_ICM    (32) /**< \brief Integrity Check Monitor (ICM) */
#define ID_ACC    (33) /**< \brief Analog Comparator (ACC) */
#define ID_USBHS  (34) /**< \brief USB Host / Device Controller (USBHS) */
#define ID_MCAN0  (35) /**< \brief MCAN Controller 0 (MCAN0) */
#define ID_MCAN1  (37) /**< \brief MCAN Controller 1 (MCAN1) */
#define ID_GMAC   (39) /**< \brief Ethernet MAC (GMAC) */
#define ID_AFEC1  (40) /**< \brief Analog Front End 1 (AFEC1) */
#define ID_TWIHS2 (41) /**< \brief Two Wire Interface 2 HS (TWIHS2) */
#define ID_SPI1   (42) /**< \brief Serial Peripheral Interface 1 (SPI1) */
#define ID_QSPI   (43) /**< \brief Quad I/O Serial Peripheral Interface (QSPI) */
#define ID_UART2  (44) /**< \brief UART 2 (UART2) */
#define ID_UART3  (45) /**< \brief UART 3 (UART3) */
#define ID_UART4  (46) /**< \brief UART 4 (UART4) */
#define ID_TC6    (47) /**< \brief Timer/Counter 6 (TC6) */
#define ID_TC7    (48) /**< \brief Timer/Counter 7 (TC7) */
#define ID_TC8    (49) /**< \brief Timer/Counter 8 (TC8) */
#define ID_TC9    (50) /**< \brief Timer/Counter 9 (TC9) */
#define ID_TC10   (51) /**< \brief Timer/Counter 10 (TC10) */
#define ID_TC11   (52) /**< \brief Timer/Counter 11 (TC11) */
#define ID_AES    (56) /**< \brief AES (AES) */
#define ID_TRNG   (57) /**< \brief True Random Generator (TRNG) */
#define ID_XDMAC  (58) /**< \brief DMA (XDMAC) */
#define ID_ISI    (59) /**< \brief Camera Interface (ISI) */
#define ID_PWM1   (60) /**< \brief Pulse Width Modulation 1 (PWM1) */
#define ID_SDRAMC (62) /**< \brief SDRAM Controller (SDRAMC) */
#define ID_RSWDT  (63) /**< \brief Reinforced Secure Watchdog Timer (RSWDT) */
#define ID_IXC    (68) /**< \brief Floating Point Unit - IXC (ARM) */

#define ID_PERIPH_COUNT (74) /**< \brief Number of peripheral IDs */
/*@}*/

/* ************************************************************************** */
/*   BASE ADDRESS DEFINITIONS FOR SAME70Q20 */
/* ************************************************************************** */
/** \addtogroup SAME70Q20_base Peripheral Base Address Definitions */
/*@{*/

#if (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
#define HSMCI  (0x40000000U) /**< \brief (HSMCI ) Base Address */
#define SSC    (0x40004000U) /**< \brief (SSC   ) Base Address */
#define SPI0   (0x40008000U) /**< \brief (SPI0  ) Base Address */
#define TC0    (0x4000C000U) /**< \brief (TC0   ) Base Address */
#define TC1    (0x40010000U) /**< \brief (TC1   ) Base Address */
#define TC2    (0x40014000U) /**< \brief (TC2   ) Base Address */
#define TWIHS0 (0x40018000U) /**< \brief (TWIHS0) Base Address */
#define TWIHS1 (0x4001C000U) /**< \brief (TWIHS1) Base Address */
#define PWM0   (0x40020000U) /**< \brief (PWM0  ) Base Address */
#define USART0 (0x40024000U) /**< \brief (USART0) Base Address */
#define USART1 (0x40028000U) /**< \brief (USART1) Base Address */
#define USART2 (0x4002C000U) /**< \brief (USART2) Base Address */
#define MCAN0  (0x40030000U) /**< \brief (MCAN0 ) Base Address */
#define MCAN1  (0x40034000U) /**< \brief (MCAN1 ) Base Address */
#define USBHS  (0x40038000U) /**< \brief (USBHS ) Base Address */
#define AFEC0  (0x4003C000U) /**< \brief (AFEC0 ) Base Address */
#define DACC   (0x40040000U) /**< \brief (DACC  ) Base Address */
#define ACC    (0x40044000U) /**< \brief (ACC   ) Base Address */
#define ICM    (0x40048000U) /**< \brief (ICM   ) Base Address */
#define ISI    (0x4004C000U) /**< \brief (ISI   ) Base Address */
#define GMAC   (0x40050000U) /**< \brief (GMAC  ) Base Address */
#define TC3    (0x40054000U) /**< \brief (TC3   ) Base Address */
#define SPI1   (0x40058000U) /**< \brief (SPI1  ) Base Address */
#define PWM1   (0x4005C000U) /**< \brief (PWM1  ) Base Address */
#define TWIHS2 (0x40060000U) /**< \brief (TWIHS2) Base Address */
#define AFEC1  (0x40064000U) /**< \brief (AFEC1 ) Base Address */
#define AES    (0x4006C000U) /**< \brief (AES   ) Base Address */
#define TRNG   (0x40070000U) /**< \brief (TRNG  ) Base Address */
#define XDMAC  (0x40078000U) /**< \brief (XDMAC ) Base Address */
#define QSPI   (0x4007C000U) /**< \brief (QSPI  ) Base Address */
#define SMC    (0x40080000U) /**< \brief (SMC   ) Base Address */
#define SDRAMC (0x40084000U) /**< \brief (SDRAMC) Base Address */
#define MATRIX (0x40088000U) /**< \brief (MATRIX) Base Address */
#define UTMI   (0x400E0400U) /**< \brief (UTMI  ) Base Address */
#define PMC    (0x400E0600U) /**< \brief (PMC   ) Base Address */
#define UART0  (0x400E0800U) /**< \brief (UART0 ) Base Address */
#define CHIPID (0x400E0940U) /**< \brief (CHIPID) Base Address */
#define UART1  (0x400E0A00U) /**< \brief (UART1 ) Base Address */
#define EFC    (0x400E0C00U) /**< \brief (EFC   ) Base Address */
#define PIOA   (0x400E0E00U) /**< \brief (PIOA  ) Base Address */
#define PIOB   (0x400E1000U) /**< \brief (PIOB  ) Base Address */
#define PIOC   (0x400E1200U) /**< \brief (PIOC  ) Base Address */
#define PIOD   (0x400E1400U) /**< \brief (PIOD  ) Base Address */
#define PIOE   (0x400E1600U) /**< \brief (PIOE  ) Base Address */
#define RSTC   (0x400E1800U) /**< \brief (RSTC  ) Base Address */
#define SUPC   (0x400E1810U) /**< \brief (SUPC  ) Base Address */
#define RTT    (0x400E1830U) /**< \brief (RTT   ) Base Address */
#define WDT    (0x400E1850U) /**< \brief (WDT   ) Base Address */
#define RTC    (0x400E1860U) /**< \brief (RTC   ) Base Address */
#define GPBR   (0x400E1890U) /**< \brief (GPBR  ) Base Address */
#define RSWDT  (0x400E1900U) /**< \brief (RSWDT ) Base Address */
#define UART2  (0x400E1A00U) /**< \brief (UART2 ) Base Address */
#define UART3  (0x400E1C00U) /**< \brief (UART3 ) Base Address */
#define UART4  (0x400E1E00U) /**< \brief (UART4 ) Base Address */
#else
#define HSMCI  ((Hsmci  *)0x40000000U) /**< \brief (HSMCI ) Base Address */
#define SSC    ((Ssc    *)0x40004000U) /**< \brief (SSC   ) Base Address */
#define SPI0   ((Spi    *)0x40008000U) /**< \brief (SPI0  ) Base Address */
#define TC0    ((Tc     *)0x4000C000U) /**< \brief (TC0   ) Base Address */
#define TC1    ((Tc     *)0x40010000U) /**< \brief (TC1   ) Base Address */
#define TC2    ((Tc     *)0x40014000U) /**< \brief (TC2   ) Base Address */
#define TWIHS0 ((Twihs  *)0x40018000U) /**< \brief (TWIHS0) Base Address */
#define TWIHS1 ((Twihs  *)0x4001C000U) /**< \brief (TWIHS1) Base Address */
#define PWM0   ((Pwm    *)0x40020000U) /**< \brief (PWM0  ) Base Address */
#define USART0 ((Usart  *)0x40024000U) /**< \brief (USART0) Base Address */
#define USART1 ((Usart  *)0x40028000U) /**< \brief (USART1) Base Address */
#define USART2 ((Usart  *)0x4002C000U) /**< \brief (USART2) Base Address */
#define MCAN0  ((Mcan   *)0x40030000U) /**< \brief (MCAN0 ) Base Address */
#define MCAN1  ((Mcan   *)0x40034000U) /**< \brief (MCAN1 ) Base Address */
#define USBHS  ((Usbhs  *)0x40038000U) /**< \brief (USBHS ) Base Address */
#define AFEC0  ((Afec   *)0x4003C000U) /**< \brief (AFEC0 ) Base Address */
#define DACC   ((Dacc   *)0x40040000U) /**< \brief (DACC  ) Base Address */
#define ACC    ((Acc    *)0x40044000U) /**< \brief (ACC   ) Base Address */
#define ICM    ((Icm    *)0x40048000U) /**< \brief (ICM   ) Base Address */
#define ISI    ((Isi    *)0x4004C000U) /**< \brief (ISI   ) Base Address */
#define GMAC   ((Gmac   *)0x40050000U) /**< \brief (GMAC  ) Base Address */
#define TC3    ((Tc     *)0x40054000U) /**< \brief (TC3   ) Base Address */
#define SPI1   ((Spi    *)0x40058000U) /**< \brief (SPI1  ) Base Address */
#define PWM1   ((Pwm    *)0x4005C000U) /**< \brief (PWM1  ) Base Address */
#define TWIHS2 ((Twihs  *)0x40060000U) /**< \brief (TWIHS2) Base Address */
#define AFEC1  ((Afec   *)0x40064000U) /**< \brief (AFEC1 ) Base Address */
#define AES    ((Aes    *)0x4006C000U) /**< \brief (AES   ) Base Address */
#define TRNG   ((Trng   *)0x40070000U) /**< \brief (TRNG  ) Base Address */
#define XDMAC  ((Xdmac  *)0x40078000U) /**< \brief (XDMAC ) Base Address */
#define QSPI   ((Qspi   *)0x4007C000U) /**< \brief (QSPI  ) Base Address */
#define SMC    ((Smc    *)0x40080000U) /**< \brief (SMC   ) Base Address */
#define SDRAMC ((Sdramc *)0x40084000U) /**< \brief (SDRAMC) Base Address */
#define MATRIX ((Matrix *)0x40088000U) /**< \brief (MATRIX) Base Address */
#define UTMI   ((Utmi   *)0x400E0400U) /**< \brief (UTMI  ) Base Address */
#define PMC    ((Pmc    *)0x400E0600U) /**< \brief (PMC   ) Base Address */
#define UART0  ((Uart   *)0x400E0800U) /**< \brief (UART0 ) Base Address */
#define CHIPID ((Chipid *)0x400E0940U) /**< \brief (CHIPID) Base Address */
#define UART1  ((Uart   *)0x400E0A00U) /**< \brief (UART1 ) Base Address */
#define EFC    ((Efc    *)0x400E0C00U) /**< \brief (EFC   ) Base Address */
#define PIOA   ((Pio    *)0x400E0E00U) /**< \brief (PIOA  ) Base Address */
#define PIOB   ((Pio    *)0x400E1000U) /**< \brief (PIOB  ) Base Address */
#define PIOC   ((Pio    *)0x400E1200U) /**< \brief (PIOC  ) Base Address */
#define PIOD   ((Pio    *)0x400E1400U) /**< \brief (PIOD  ) Base Address */
#define PIOE   ((Pio    *)0x400E1600U) /**< \brief (PIOE  ) Base Address */
#define RSTC   ((Rstc   *)0x400E1800U) /**< \brief (RSTC  ) Base Address */
#define SUPC   ((Supc   *)0x400E1810U) /**< \brief (SUPC  ) Base Address */
#define RTT    ((Rtt    *)0x400E1830U) /**< \brief (RTT   ) Base Address */
#define WDT    ((Wdt    *)0x400E1850U) /**< \brief (WDT   ) Base Address */
#define RTC    ((Rtc    *)0x400E1860U) /**< \brief (RTC   ) Base Address */
#define GPBR   ((Gpbr   *)0x400E1890U) /**< \brief (GPBR  ) Base Address */
#define RSWDT  ((Rswdt  *)0x400E1900U) /**< \brief (RSWDT ) Base Address */
#define UART2  ((Uart   *)0x400E1A00U) /**< \brief (UART2 ) Base Address */
#define UART3  ((Uart   *)0x400E1C00U) /**< \brief (UART3 ) Base Address */
#define UART4  ((Uart   *)0x400E1E00U) /**< \brief (UART4 ) Base Address */
#endif /* (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */
/*@}*/

/* ************************************************************************** */
/*   PIO DEFINITIONS FOR SAME70Q20 */
/* ************************************************************************** */
/** \addtogroup SAME70Q20_pio Peripheral Pio Definitions */
/*@{*/

#ifndef _SAME70Q20_PIO_
#define _SAME70Q20_PIO_

#define PIO_PA0                   (1u << 0)  /**< \brief Pin Controlled by PA0 */
#define PIO_PA1                   (1u << 1)  /**< \brief Pin Controlled by PA1 */
#define PIO_PA2                   (1u << 2)  /**< \brief Pin Controlled by PA2 */
#define PIO_PA3                   (1u << 3)  /**< \brief Pin Controlled by PA3 */
#define PIO_PA4                   (1u << 4)  /**< \brief Pin Controlled by PA4 */
#define PIO_PA5                   (1u << 5)  /**< \brief Pin Controlled by PA5 */
#define PIO_PA6                   (1u << 6)  /**< \brief Pin Controlled by PA6 */
#define PIO_PA7                   (1u << 7)  /**< \brief Pin Controlled by PA7 */
#define PIO_PA8                   (1u << 8)  /**< \brief Pin Controlled by PA8 */
#define PIO_PA9                   (1u << 9)  /**< \brief Pin Controlled by PA9 */
#define PIO_PA10                  (1u << 10) /**< \brief Pin Controlled by PA10 */
#define PIO_PA11                  (1u << 11) /**< \brief Pin Controlled by PA11 */
#define PIO_PA12                  (1u << 12) /**< \brief Pin Controlled by PA12 */
#define PIO_PA13                  (1u << 13) /**< \brief Pin Controlled by PA13 */
#define PIO_PA14                  (1u << 14) /**< \brief Pin Controlled by PA14 */
#define PIO_PA15                  (1u << 15) /**< \brief Pin Controlled by PA15 */
#define PIO_PA16                  (1u << 16) /**< \brief Pin Controlled by PA16 */
#define PIO_PA17                  (1u << 17) /**< \brief Pin Controlled by PA17 */
#define PIO_PA18                  (1u << 18) /**< \brief Pin Controlled by PA18 */
#define PIO_PA19                  (1u << 19) /**< \brief Pin Controlled by PA19 */
#define PIO_PA20                  (1u << 20) /**< \brief Pin Controlled by PA20 */
#define PIO_PA21                  (1u << 21) /**< \brief Pin Controlled by PA21 */
#define PIO_PA22                  (1u << 22) /**< \brief Pin Controlled by PA22 */
#define PIO_PA23                  (1u << 23) /**< \brief Pin Controlled by PA23 */
#define PIO_PA24                  (1u << 24) /**< \brief Pin Controlled by PA24 */
#define PIO_PA25                  (1u << 25) /**< \brief Pin Controlled by PA25 */
#define PIO_PA26                  (1u << 26) /**< \brief Pin Controlled by PA26 */
#define PIO_PA27                  (1u << 27) /**< \brief Pin Controlled by PA27 */
#define PIO_PA28                  (1u << 28) /**< \brief Pin Controlled by PA28 */
#define PIO_PA29                  (1u << 29) /**< \brief Pin Controlled by PA29 */
#define PIO_PA30                  (1u << 30) /**< \brief Pin Controlled by PA30 */
#define PIO_PA31                  (1u << 31) /**< \brief Pin Controlled by PA31 */
#define PIO_PB0                   (1u << 0)  /**< \brief Pin Controlled by PB0 */
#define PIO_PB1                   (1u << 1)  /**< \brief Pin Controlled by PB1 */
#define PIO_PB2                   (1u << 2)  /**< \brief Pin Controlled by PB2 */
#define PIO_PB3                   (1u << 3)  /**< \brief Pin Controlled by PB3 */
#define PIO_PB4                   (1u << 4)  /**< \brief Pin Controlled by PB4 */
#define PIO_PB5                   (1u << 5)  /**< \brief Pin Controlled by PB5 */
#define PIO_PB6                   (1u << 6)  /**< \brief Pin Controlled by PB6 */
#define PIO_PB7                   (1u << 7)  /**< \brief Pin Controlled by PB7 */
#define PIO_PB8                   (1u << 8)  /**< \brief Pin Controlled by PB8 */
#define PIO_PB9                   (1u << 9)  /**< \brief Pin Controlled by PB9 */
#define PIO_PB12                  (1u << 12) /**< \brief Pin Controlled by PB12 */
#define PIO_PB13                  (1u << 13) /**< \brief Pin Controlled by PB13 */
#define PIO_PC0                   (1u << 0)  /**< \brief Pin Controlled by PC0 */
#define PIO_PC1                   (1u << 1)  /**< \brief Pin Controlled by PC1 */
#define PIO_PC2                   (1u << 2)  /**< \brief Pin Controlled by PC2 */
#define PIO_PC3                   (1u << 3)  /**< \brief Pin Controlled by PC3 */
#define PIO_PC4                   (1u << 4)  /**< \brief Pin Controlled by PC4 */
#define PIO_PC5                   (1u << 5)  /**< \brief Pin Controlled by PC5 */
#define PIO_PC6                   (1u << 6)  /**< \brief Pin Controlled by PC6 */
#define PIO_PC7                   (1u << 7)  /**< \brief Pin Controlled by PC7 */
#define PIO_PC8                   (1u << 8)  /**< \brief Pin Controlled by PC8 */
#define PIO_PC9                   (1u << 9)  /**< \brief Pin Controlled by PC9 */
#define PIO_PC10                  (1u << 10) /**< \brief Pin Controlled by PC10 */
#define PIO_PC11                  (1u << 11) /**< \brief Pin Controlled by PC11 */
#define PIO_PC12                  (1u << 12) /**< \brief Pin Controlled by PC12 */
#define PIO_PC13                  (1u << 13) /**< \brief Pin Controlled by PC13 */
#define PIO_PC14                  (1u << 14) /**< \brief Pin Controlled by PC14 */
#define PIO_PC15                  (1u << 15) /**< \brief Pin Controlled by PC15 */
#define PIO_PC16                  (1u << 16) /**< \brief Pin Controlled by PC16 */
#define PIO_PC17                  (1u << 17) /**< \brief Pin Controlled by PC17 */
#define PIO_PC18                  (1u << 18) /**< \brief Pin Controlled by PC18 */
#define PIO_PC19                  (1u << 19) /**< \brief Pin Controlled by PC19 */
#define PIO_PC20                  (1u << 20) /**< \brief Pin Controlled by PC20 */
#define PIO_PC21                  (1u << 21) /**< \brief Pin Controlled by PC21 */
#define PIO_PC22                  (1u << 22) /**< \brief Pin Controlled by PC22 */
#define PIO_PC23                  (1u << 23) /**< \brief Pin Controlled by PC23 */
#define PIO_PC24                  (1u << 24) /**< \brief Pin Controlled by PC24 */
#define PIO_PC25                  (1u << 25) /**< \brief Pin Controlled by PC25 */
#define PIO_PC26                  (1u << 26) /**< \brief Pin Controlled by PC26 */
#define PIO_PC27                  (1u << 27) /**< \brief Pin Controlled by PC27 */
#define PIO_PC28                  (1u << 28) /**< \brief Pin Controlled by PC28 */
#define PIO_PC29                  (1u << 29) /**< \brief Pin Controlled by PC29 */
#define PIO_PC30                  (1u << 30) /**< \brief Pin Controlled by PC30 */
#define PIO_PC31                  (1u << 31) /**< \brief Pin Controlled by PC31 */
#define PIO_PD0                   (1u << 0)  /**< \brief Pin Controlled by PD0 */
#define PIO_PD1                   (1u << 1)  /**< \brief Pin Controlled by PD1 */
#define PIO_PD2                   (1u << 2)  /**< \brief Pin Controlled by PD2 */
#define PIO_PD3                   (1u << 3)  /**< \brief Pin Controlled by PD3 */
#define PIO_PD4                   (1u << 4)  /**< \brief Pin Controlled by PD4 */
#define PIO_PD5                   (1u << 5)  /**< \brief Pin Controlled by PD5 */
#define PIO_PD6                   (1u << 6)  /**< \brief Pin Controlled by PD6 */
#define PIO_PD7                   (1u << 7)  /**< \brief Pin Controlled by PD7 */
#define PIO_PD8                   (1u << 8)  /**< \brief Pin Controlled by PD8 */
#define PIO_PD9                   (1u << 9)  /**< \brief Pin Controlled by PD9 */
#define PIO_PD10                  (1u << 10) /**< \brief Pin Controlled by PD10 */
#define PIO_PD11                  (1u << 11) /**< \brief Pin Controlled by PD11 */
#define PIO_PD12                  (1u << 12) /**< \brief Pin Controlled by PD12 */
#define PIO_PD13                  (1u << 13) /**< \brief Pin Controlled by PD13 */
#define PIO_PD14                  (1u << 14) /**< \brief Pin Controlled by PD14 */
#define PIO_PD15                  (1u << 15) /**< \brief Pin Controlled by PD15 */
#define PIO_PD16                  (1u << 16) /**< \brief Pin Controlled by PD16 */
#define PIO_PD17                  (1u << 17) /**< \brief Pin Controlled by PD17 */
#define PIO_PD18                  (1u << 18) /**< \brief Pin Controlled by PD18 */
#define PIO_PD19                  (1u << 19) /**< \brief Pin Controlled by PD19 */
#define PIO_PD20                  (1u << 20) /**< \brief Pin Controlled by PD20 */
#define PIO_PD21                  (1u << 21) /**< \brief Pin Controlled by PD21 */
#define PIO_PD22                  (1u << 22) /**< \brief Pin Controlled by PD22 */
#define PIO_PD23                  (1u << 23) /**< \brief Pin Controlled by PD23 */
#define PIO_PD24                  (1u << 24) /**< \brief Pin Controlled by PD24 */
#define PIO_PD25                  (1u << 25) /**< \brief Pin Controlled by PD25 */
#define PIO_PD26                  (1u << 26) /**< \brief Pin Controlled by PD26 */
#define PIO_PD27                  (1u << 27) /**< \brief Pin Controlled by PD27 */
#define PIO_PD28                  (1u << 28) /**< \brief Pin Controlled by PD28 */
#define PIO_PD29                  (1u << 29) /**< \brief Pin Controlled by PD29 */
#define PIO_PD30                  (1u << 30) /**< \brief Pin Controlled by PD30 */
#define PIO_PD31                  (1u << 31) /**< \brief Pin Controlled by PD31 */
#define PIO_PE0                   (1u << 0)  /**< \brief Pin Controlled by PE0 */
#define PIO_PE1                   (1u << 1)  /**< \brief Pin Controlled by PE1 */
#define PIO_PE2                   (1u << 2)  /**< \brief Pin Controlled by PE2 */
#define PIO_PE3                   (1u << 3)  /**< \brief Pin Controlled by PE3 */
#define PIO_PE4                   (1u << 4)  /**< \brief Pin Controlled by PE4 */
#define PIO_PE5                   (1u << 5)  /**< \brief Pin Controlled by PE5 */
/* ========== Pio definition for AFEC0 peripheral ========== */
#define PIO_PD30X1_AFE0_AD0       (1u << 30) /**< \brief Afec0 signal: AFE0_AD0 */
#define PIO_PA21X1_AFE0_AD1       (1u << 21) /**< \brief Afec0 signal: AFE0_AD1/PIODCEN2 */
#define PIO_PA21X1_PIODCEN2       (1u << 21) /**< \brief Afec0 signal: AFE0_AD1/PIODCEN2 */
#define PIO_PB0X1_AFE0_AD10       (1u << 0)  /**< \brief Afec0 signal: AFE0_AD10/RTCOUT0 */
#define PIO_PB0X1_RTCOUT0         (1u << 0)  /**< \brief Afec0 signal: AFE0_AD10/RTCOUT0 */
#define PIO_PB3X1_AFE0_AD2        (1u << 3)  /**< \brief Afec0 signal: AFE0_AD2/WKUP12 */
#define PIO_PB3X1_WKUP12          (1u << 3)  /**< \brief Afec0 signal: AFE0_AD2/WKUP12 */
#define PIO_PE5X1_AFE0_AD3        (1u << 5)  /**< \brief Afec0 signal: AFE0_AD3 */
#define PIO_PE4X1_AFE0_AD4        (1u << 4)  /**< \brief Afec0 signal: AFE0_AD4 */
#define PIO_PB2X1_AFE0_AD5        (1u << 2)  /**< \brief Afec0 signal: AFE0_AD5 */
#define PIO_PA17X1_AFE0_AD6       (1u << 17) /**< \brief Afec0 signal: AFE0_AD6 */
#define PIO_PA18X1_AFE0_AD7       (1u << 18) /**< \brief Afec0 signal: AFE0_AD7 */
#define PIO_PA19X1_AFE0_AD8       (1u << 19) /**< \brief Afec0 signal: AFE0_AD8/WKUP9 */
#define PIO_PA19X1_WKUP9          (1u << 19) /**< \brief Afec0 signal: AFE0_AD8/WKUP9 */
#define PIO_PA20X1_AFE0_AD9       (1u << 20) /**< \brief Afec0 signal: AFE0_AD9/WKUP10 */
#define PIO_PA20X1_WKUP10         (1u << 20) /**< \brief Afec0 signal: AFE0_AD9/WKUP10 */
#define PIO_PA8B_AFE0_ADTRG       (1u << 8)  /**< \brief Afec0 signal: AFE0_ADTRG */
/* ========== Pio definition for AFEC1 peripheral ========== */
#define PIO_PB1X1_AFE1_AD0        (1u << 1)  /**< \brief Afec1 signal: AFE1_AD0/RTCOUT1 */
#define PIO_PB1X1_RTCOUT1         (1u << 1)  /**< \brief Afec1 signal: AFE1_AD0/RTCOUT1 */
#define PIO_PC13X1_AFE1_AD1       (1u << 13) /**< \brief Afec1 signal: AFE1_AD1 */
#define PIO_PE3X1_AFE1_AD10       (1u << 3)  /**< \brief Afec1 signal: AFE1_AD10 */
#define PIO_PE0X1_AFE1_AD11       (1u << 0)  /**< \brief Afec1 signal: AFE1_AD11 */
#define PIO_PC15X1_AFE1_AD2       (1u << 15) /**< \brief Afec1 signal: AFE1_AD2 */
#define PIO_PC12X1_AFE1_AD3       (1u << 12) /**< \brief Afec1 signal: AFE1_AD3 */
#define PIO_PC29X1_AFE1_AD4       (1u << 29) /**< \brief Afec1 signal: AFE1_AD4 */
#define PIO_PC30X1_AFE1_AD5       (1u << 30) /**< \brief Afec1 signal: AFE1_AD5 */
#define PIO_PC31X1_AFE1_AD6       (1u << 31) /**< \brief Afec1 signal: AFE1_AD6 */
#define PIO_PC26X1_AFE1_AD7       (1u << 26) /**< \brief Afec1 signal: AFE1_AD7 */
#define PIO_PC27X1_AFE1_AD8       (1u << 27) /**< \brief Afec1 signal: AFE1_AD8 */
#define PIO_PC0X1_AFE1_AD9        (1u << 0)  /**< \brief Afec1 signal: AFE1_AD9 */
#define PIO_PD9C_AFE1_ADTRG       (1u << 9)  /**< \brief Afec1 signal: AFE1_ADTRG */
/* ========== Pio definition for ARM peripheral ========== */
#define PIO_PB7X1_SWCLK           (1u << 7)  /**< \brief Arm signal: SWCLK/TCK */
#define PIO_PB7X1_TCK             (1u << 7)  /**< \brief Arm signal: SWCLK/TCK */
#define PIO_PB6X1_SWDIO           (1u << 6)  /**< \brief Arm signal: SWDIO/TMS */
#define PIO_PB6X1_TMS             (1u << 6)  /**< \brief Arm signal: SWDIO/TMS */
#define PIO_PB4X1_TDI             (1u << 4)  /**< \brief Arm signal: TDI */
#define PIO_PB5X1_TDO             (1u << 5)  /**< \brief Arm signal: TDO/TRACESWO/WKUP13 */
#define PIO_PB5X1_TRACESWO        (1u << 5)  /**< \brief Arm signal: TDO/TRACESWO/WKUP13 */
#define PIO_PB5X1_WKUP13          (1u << 5)  /**< \brief Arm signal: TDO/TRACESWO/WKUP13 */
/* ========== Pio definition for DACC peripheral ========== */
#define PIO_PB13X1_DAC0           (1u << 13) /**< \brief Dacc signal: DAC0 */
#define PIO_PD0X1_DAC1            (1u << 0)  /**< \brief Dacc signal: DAC1 */
#define PIO_PA2C_DATRG            (1u << 2)  /**< \brief Dacc signal: DATRG */
/* ========== Pio definition for EBI peripheral ========== */
#define PIO_PC18A_A0              (1u << 18) /**< \brief Ebi signal: A0/NBS0 */
#define PIO_PC18A_NBS0            (1u << 18) /**< \brief Ebi signal: A0/NBS0 */
#define PIO_PC19A_A1              (1u << 19) /**< \brief Ebi signal: A1 */
#define PIO_PC28A_A10             (1u << 28) /**< \brief Ebi signal: A10 */
#define PIO_PC29A_A11             (1u << 29) /**< \brief Ebi signal: A11 */
#define PIO_PC30A_A12             (1u << 30) /**< \brief Ebi signal: A12 */
#define PIO_PC31A_A13             (1u << 31) /**< \brief Ebi signal: A13 */
#define PIO_PA18C_A14             (1u << 18) /**< \brief Ebi signal: A14 */
#define PIO_PA19C_A15             (1u << 19) /**< \brief Ebi signal: A15 */
#define PIO_PA20C_A16             (1u << 20) /**< \brief Ebi signal: A16/BA0 */
#define PIO_PA20C_BA0             (1u << 20) /**< \brief Ebi signal: A16/BA0 */
#define PIO_PA0C_A17              (1u << 0)  /**< \brief Ebi signal: A17/BA1 */
#define PIO_PA0C_BA1              (1u << 0)  /**< \brief Ebi signal: A17/BA1 */
#define PIO_PA1C_A18              (1u << 1)  /**< \brief Ebi signal: A18 */
#define PIO_PA23C_A19             (1u << 23) /**< \brief Ebi signal: A19 */
#define PIO_PC20A_A2              (1u << 20) /**< \brief Ebi signal: A2 */
#define PIO_PA24C_A20             (1u << 24) /**< \brief Ebi signal: A20 */
#define PIO_PC16A_A21             (1u << 16) /**< \brief Ebi signal: A21/NANDALE */
#define PIO_PC16A_NANDALE         (1u << 16) /**< \brief Ebi signal: A21/NANDALE */
#define PIO_PC17A_A22             (1u << 17) /**< \brief Ebi signal: A22/NANDCLE */
#define PIO_PC17A_NANDCLE         (1u << 17) /**< \brief Ebi signal: A22/NANDCLE */
#define PIO_PA25C_A23             (1u << 25) /**< \brief Ebi signal: A23 */
#define PIO_PC21A_A3              (1u << 21) /**< \brief Ebi signal: A3 */
#define PIO_PC22A_A4              (1u << 22) /**< \brief Ebi signal: A4 */
#define PIO_PC23A_A5              (1u << 23) /**< \brief Ebi signal: A5 */
#define PIO_PC24A_A6              (1u << 24) /**< \brief Ebi signal: A6 */
#define PIO_PC25A_A7              (1u << 25) /**< \brief Ebi signal: A7 */
#define PIO_PC26A_A8              (1u << 26) /**< \brief Ebi signal: A8 */
#define PIO_PC27A_A9              (1u << 27) /**< \brief Ebi signal: A9 */
#define PIO_PD17C_CAS             (1u << 17) /**< \brief Ebi signal: CAS */
#define PIO_PC0A_D0               (1u << 0)  /**< \brief Ebi signal: D0 */
#define PIO_PC1A_D1               (1u << 1)  /**< \brief Ebi signal: D1 */
#define PIO_PE2A_D10              (1u << 2)  /**< \brief Ebi signal: D10 */
#define PIO_PE3A_D11              (1u << 3)  /**< \brief Ebi signal: D11 */
#define PIO_PE4A_D12              (1u << 4)  /**< \brief Ebi signal: D12 */
#define PIO_PE5A_D13              (1u << 5)  /**< \brief Ebi signal: D13 */
#define PIO_PA15A_D14             (1u << 15) /**< \brief Ebi signal: D14 */
#define PIO_PA16A_D15             (1u << 16) /**< \brief Ebi signal: D15 */
#define PIO_PC2A_D2               (1u << 2)  /**< \brief Ebi signal: D2 */
#define PIO_PC3A_D3               (1u << 3)  /**< \brief Ebi signal: D3 */
#define PIO_PC4A_D4               (1u << 4)  /**< \brief Ebi signal: D4 */
#define PIO_PC5A_D5               (1u << 5)  /**< \brief Ebi signal: D5 */
#define PIO_PC6A_D6               (1u << 6)  /**< \brief Ebi signal: D6 */
#define PIO_PC7A_D7               (1u << 7)  /**< \brief Ebi signal: D7 */
#define PIO_PE0A_D8               (1u << 0)  /**< \brief Ebi signal: D8 */
#define PIO_PE1A_D9               (1u << 1)  /**< \brief Ebi signal: D9 */
#define PIO_PC9A_NANDOE           (1u << 9)  /**< \brief Ebi signal: NANDOE */
#define PIO_PC10A_NANDWE          (1u << 10) /**< \brief Ebi signal: NANDWE */
#define PIO_PC14A_NCS0            (1u << 14) /**< \brief Ebi signal: NCS0 */
#define PIO_PC15A_NCS1            (1u << 15) /**< \brief Ebi signal: NCS1/SDCS */
#define PIO_PC15A_SDCS            (1u << 15) /**< \brief Ebi signal: NCS1/SDCS */
#define PIO_PD18A_NCS1            (1u << 18) /**< \brief Ebi signal: NCS1/SDCS */
#define PIO_PD18A_SDCS            (1u << 18) /**< \brief Ebi signal: NCS1/SDCS */
#define PIO_PA22C_NCS2            (1u << 22) /**< \brief Ebi signal: NCS2 */
#define PIO_PC12A_NCS3            (1u << 12) /**< \brief Ebi signal: NCS3 */
#define PIO_PD19A_NCS3            (1u << 19) /**< \brief Ebi signal: NCS3 */
#define PIO_PC11A_NRD             (1u << 11) /**< \brief Ebi signal: NRD */
#define PIO_PC13A_NWAIT           (1u << 13) /**< \brief Ebi signal: NWAIT */
#define PIO_PC8A_NWR0             (1u << 8)  /**< \brief Ebi signal: NWR0/NWE */
#define PIO_PC8A_NWE              (1u << 8)  /**< \brief Ebi signal: NWR0/NWE */
#define PIO_PD15C_NWR1            (1u << 15) /**< \brief Ebi signal: NWR1/NBS1 */
#define PIO_PD15C_NBS1            (1u << 15) /**< \brief Ebi signal: NWR1/NBS1 */
#define PIO_PD16C_RAS             (1u << 16) /**< \brief Ebi signal: RAS */
#define PIO_PC13C_SDA10           (1u << 13) /**< \brief Ebi signal: SDA10 */
#define PIO_PD13C_SDA10           (1u << 13) /**< \brief Ebi signal: SDA10 */
#define PIO_PD23C_SDCK            (1u << 23) /**< \brief Ebi signal: SDCK */
#define PIO_PD14C_SDCKE           (1u << 14) /**< \brief Ebi signal: SDCKE */
#define PIO_PD29C_SDWE            (1u << 29) /**< \brief Ebi signal: SDWE */
/* ========== Pio definition for GMAC peripheral ========== */
#define PIO_PD13A_GCOL            (1u << 13) /**< \brief Gmac signal: GCOL */
#define PIO_PD10A_GCRS            (1u << 10) /**< \brief Gmac signal: GCRS */
#define PIO_PD8A_GMDC             (1u << 8)  /**< \brief Gmac signal: GMDC */
#define PIO_PD9A_GMDIO            (1u << 9)  /**< \brief Gmac signal: GMDIO */
#define PIO_PD5A_GRX0             (1u << 5)  /**< \brief Gmac signal: GRX0 */
#define PIO_PD6A_GRX1             (1u << 6)  /**< \brief Gmac signal: GRX1 */
#define PIO_PD11A_GRX2            (1u << 11) /**< \brief Gmac signal: GRX2 */
#define PIO_PD12A_GRX3            (1u << 12) /**< \brief Gmac signal: GRX3 */
#define PIO_PD14A_GRXCK           (1u << 14) /**< \brief Gmac signal: GRXCK */
#define PIO_PD4A_GRXDV            (1u << 4)  /**< \brief Gmac signal: GRXDV */
#define PIO_PD7A_GRXER            (1u << 7)  /**< \brief Gmac signal: GRXER */
#define PIO_PB1B_GTSUCOMP         (1u << 1)  /**< \brief Gmac signal: GTSUCOMP */
#define PIO_PB12B_GTSUCOMP        (1u << 12) /**< \brief Gmac signal: GTSUCOMP */
#define PIO_PD11C_GTSUCOMP        (1u << 11) /**< \brief Gmac signal: GTSUCOMP */
#define PIO_PD20C_GTSUCOMP        (1u << 20) /**< \brief Gmac signal: GTSUCOMP */
#define PIO_PD2A_GTX0             (1u << 2)  /**< \brief Gmac signal: GTX0 */
#define PIO_PD3A_GTX1             (1u << 3)  /**< \brief Gmac signal: GTX1 */
#define PIO_PD15A_GTX2            (1u << 15) /**< \brief Gmac signal: GTX2 */
#define PIO_PD16A_GTX3            (1u << 16) /**< \brief Gmac signal: GTX3 */
#define PIO_PD0A_GTXCK            (1u << 0)  /**< \brief Gmac signal: GTXCK */
#define PIO_PD1A_GTXEN            (1u << 1)  /**< \brief Gmac signal: GTXEN */
#define PIO_PD17A_GTXER           (1u << 17) /**< \brief Gmac signal: GTXER */
/* ========== Pio definition for HSMCI peripheral ========== */
#define PIO_PA28C_MCCDA           (1u << 28) /**< \brief Hsmci signal: MCCDA */
#define PIO_PA25D_MCCK            (1u << 25) /**< \brief Hsmci signal: MCCK */
#define PIO_PA30C_MCDA0           (1u << 30) /**< \brief Hsmci signal: MCDA0 */
#define PIO_PA31C_MCDA1           (1u << 31) /**< \brief Hsmci signal: MCDA1 */
#define PIO_PA26C_MCDA2           (1u << 26) /**< \brief Hsmci signal: MCDA2 */
#define PIO_PA27C_MCDA3           (1u << 27) /**< \brief Hsmci signal: MCDA3 */
/* ========== Pio definition for ISI peripheral ========== */
#define PIO_PD22D_ISI_D0          (1u << 22) /**< \brief Isi signal: ISI_D0 */
#define PIO_PD21D_ISI_D1          (1u << 21) /**< \brief Isi signal: ISI_D1 */
#define PIO_PD30D_ISI_D10         (1u << 30) /**< \brief Isi signal: ISI_D10 */
#define PIO_PD31D_ISI_D11         (1u << 31) /**< \brief Isi signal: ISI_D11 */
#define PIO_PB3D_ISI_D2           (1u << 3)  /**< \brief Isi signal: ISI_D2 */
#define PIO_PA9B_ISI_D3           (1u << 9)  /**< \brief Isi signal: ISI_D3 */
#define PIO_PA5B_ISI_D4           (1u << 5)  /**< \brief Isi signal: ISI_D4 */
#define PIO_PD11D_ISI_D5          (1u << 11) /**< \brief Isi signal: ISI_D5 */
#define PIO_PD12D_ISI_D6          (1u << 12) /**< \brief Isi signal: ISI_D6 */
#define PIO_PA27D_ISI_D7          (1u << 27) /**< \brief Isi signal: ISI_D7 */
#define PIO_PD27D_ISI_D8          (1u << 27) /**< \brief Isi signal: ISI_D8 */
#define PIO_PD28D_ISI_D9          (1u << 28) /**< \brief Isi signal: ISI_D9 */
#define PIO_PD24D_ISI_HSYNC       (1u << 24) /**< \brief Isi signal: ISI_HSYNC */
#define PIO_PA24D_ISI_PCK         (1u << 24) /**< \brief Isi signal: ISI_PCK */
#define PIO_PD25D_ISI_VSYNC       (1u << 25) /**< \brief Isi signal: ISI_VSYNC */
/* ========== Pio definition for MCAN0 peripheral ========== */
#define PIO_PB3A_CANRX0           (1u << 3)  /**< \brief Mcan0 signal: CANRX0 */
#define PIO_PB2A_CANTX0           (1u << 2)  /**< \brief Mcan0 signal: CANTX0 */
/* ========== Pio definition for MCAN1 peripheral ========== */
#define PIO_PC12C_CANRX1          (1u << 12) /**< \brief Mcan1 signal: CANRX1 */
#define PIO_PD28B_CANRX1          (1u << 28) /**< \brief Mcan1 signal: CANRX1 */
#define PIO_PC14C_CANTX1          (1u << 14) /**< \brief Mcan1 signal: CANTX1 */
#define PIO_PD12B_CANTX1          (1u << 12) /**< \brief Mcan1 signal: CANTX1 */
/* ========== Pio definition for PIOA peripheral ========== */
#define PIO_PA21X1_AFE0_AD1       (1u << 21) /**< \brief Pioa signal: AFE0_AD1/PIODCEN2 */
#define PIO_PA21X1_PIODCEN2       (1u << 21) /**< \brief Pioa signal: AFE0_AD1/PIODCEN2 */
#define PIO_PA3X1_PIODC0          (1u << 3)  /**< \brief Pioa signal: PIODC0 */
#define PIO_PA10X1_PIODC4         (1u << 10) /**< \brief Pioa signal: PIODC4 */
#define PIO_PA12X1_PIODC6         (1u << 12) /**< \brief Pioa signal: PIODC6 */
#define PIO_PA13X1_PIODC7         (1u << 13) /**< \brief Pioa signal: PIODC7 */
#define PIO_PA22X1_PIODCCLK       (1u << 22) /**< \brief Pioa signal: PIODCCLK */
#define PIO_PA4X1_WKUP3           (1u << 4)  /**< \brief Pioa signal: WKUP3/PIODC1 */
#define PIO_PA4X1_PIODC1          (1u << 4)  /**< \brief Pioa signal: WKUP3/PIODC1 */
#define PIO_PA5X1_WKUP4           (1u << 5)  /**< \brief Pioa signal: WKUP4/PIODC2 */
#define PIO_PA5X1_PIODC2          (1u << 5)  /**< \brief Pioa signal: WKUP4/PIODC2 */
#define PIO_PA9X1_WKUP6           (1u << 9)  /**< \brief Pioa signal: WKUP6/PIODC3 */
#define PIO_PA9X1_PIODC3          (1u << 9)  /**< \brief Pioa signal: WKUP6/PIODC3 */
#define PIO_PA11X1_WKUP7          (1u << 11) /**< \brief Pioa signal: WKUP7/PIODC5 */
#define PIO_PA11X1_PIODC5         (1u << 11) /**< \brief Pioa signal: WKUP7/PIODC5 */
#define PIO_PA14X1_WKUP8          (1u << 14) /**< \brief Pioa signal: WKUP8/PIODCEN1 */
#define PIO_PA14X1_PIODCEN1       (1u << 14) /**< \brief Pioa signal: WKUP8/PIODCEN1 */
/* ========== Pio definition for PMC peripheral ========== */
#define PIO_PA6B_PCK0             (1u << 6)  /**< \brief Pmc signal: PCK0 */
#define PIO_PB12D_PCK0            (1u << 12) /**< \brief Pmc signal: PCK0 */
#define PIO_PB13B_PCK0            (1u << 13) /**< \brief Pmc signal: PCK0 */
#define PIO_PA17B_PCK1            (1u << 17) /**< \brief Pmc signal: PCK1 */
#define PIO_PA21B_PCK1            (1u << 21) /**< \brief Pmc signal: PCK1 */
#define PIO_PA3C_PCK2             (1u << 3)  /**< \brief Pmc signal: PCK2 */
#define PIO_PA18B_PCK2            (1u << 18) /**< \brief Pmc signal: PCK2 */
#define PIO_PA31B_PCK2            (1u << 31) /**< \brief Pmc signal: PCK2 */
#define PIO_PB3B_PCK2             (1u << 3)  /**< \brief Pmc signal: PCK2 */
#define PIO_PD31C_PCK2            (1u << 31) /**< \brief Pmc signal: PCK2 */
/* ========== Pio definition for PWM0 peripheral ========== */
#define PIO_PA10B_PWMC0_PWMEXTRG0 (1u << 10) /**< \brief Pwm0 signal: PWMC0_PWMEXTRG0 */
#define PIO_PA22B_PWMC0_PWMEXTRG1 (1u << 22) /**< \brief Pwm0 signal: PWMC0_PWMEXTRG1 */
#define PIO_PA9C_PWMC0_PWMFI0     (1u << 9)  /**< \brief Pwm0 signal: PWMC0_PWMFI0 */
#define PIO_PD8B_PWMC0_PWMFI1     (1u << 8)  /**< \brief Pwm0 signal: PWMC0_PWMFI1 */
#define PIO_PD9B_PWMC0_PWMFI2     (1u << 9)  /**< \brief Pwm0 signal: PWMC0_PWMFI2 */
#define PIO_PA0A_PWMC0_PWMH0      (1u << 0)  /**< \brief Pwm0 signal: PWMC0_PWMH0 */
#define PIO_PA11B_PWMC0_PWMH0     (1u << 11) /**< \brief Pwm0 signal: PWMC0_PWMH0 */
#define PIO_PA23B_PWMC0_PWMH0     (1u << 23) /**< \brief Pwm0 signal: PWMC0_PWMH0 */
#define PIO_PB0A_PWMC0_PWMH0      (1u << 0)  /**< \brief Pwm0 signal: PWMC0_PWMH0 */
#define PIO_PD11B_PWMC0_PWMH0     (1u << 11) /**< \brief Pwm0 signal: PWMC0_PWMH0 */
#define PIO_PD20A_PWMC0_PWMH0     (1u << 20) /**< \brief Pwm0 signal: PWMC0_PWMH0 */
#define PIO_PA2A_PWMC0_PWMH1      (1u << 2)  /**< \brief Pwm0 signal: PWMC0_PWMH1 */
#define PIO_PA12B_PWMC0_PWMH1     (1u << 12) /**< \brief Pwm0 signal: PWMC0_PWMH1 */
#define PIO_PA24B_PWMC0_PWMH1     (1u << 24) /**< \brief Pwm0 signal: PWMC0_PWMH1 */
#define PIO_PB1A_PWMC0_PWMH1      (1u << 1)  /**< \brief Pwm0 signal: PWMC0_PWMH1 */
#define PIO_PD21A_PWMC0_PWMH1     (1u << 21) /**< \brief Pwm0 signal: PWMC0_PWMH1 */
#define PIO_PA13B_PWMC0_PWMH2     (1u << 13) /**< \brief Pwm0 signal: PWMC0_PWMH2 */
#define PIO_PA25B_PWMC0_PWMH2     (1u << 25) /**< \brief Pwm0 signal: PWMC0_PWMH2 */
#define PIO_PB4B_PWMC0_PWMH2      (1u << 4)  /**< \brief Pwm0 signal: PWMC0_PWMH2 */
#define PIO_PC19B_PWMC0_PWMH2     (1u << 19) /**< \brief Pwm0 signal: PWMC0_PWMH2 */
#define PIO_PD22A_PWMC0_PWMH2     (1u << 22) /**< \brief Pwm0 signal: PWMC0_PWMH2 */
#define PIO_PA7B_PWMC0_PWMH3      (1u << 7)  /**< \brief Pwm0 signal: PWMC0_PWMH3 */
#define PIO_PA14B_PWMC0_PWMH3     (1u << 14) /**< \brief Pwm0 signal: PWMC0_PWMH3 */
#define PIO_PA17C_PWMC0_PWMH3     (1u << 17) /**< \brief Pwm0 signal: PWMC0_PWMH3 */
#define PIO_PC13B_PWMC0_PWMH3     (1u << 13) /**< \brief Pwm0 signal: PWMC0_PWMH3 */
#define PIO_PC21B_PWMC0_PWMH3     (1u << 21) /**< \brief Pwm0 signal: PWMC0_PWMH3 */
#define PIO_PD23A_PWMC0_PWMH3     (1u << 23) /**< \brief Pwm0 signal: PWMC0_PWMH3 */
#define PIO_PA1A_PWMC0_PWML0      (1u << 1)  /**< \brief Pwm0 signal: PWMC0_PWML0 */
#define PIO_PA19B_PWMC0_PWML0     (1u << 19) /**< \brief Pwm0 signal: PWMC0_PWML0 */
#define PIO_PB5B_PWMC0_PWML0      (1u << 5)  /**< \brief Pwm0 signal: PWMC0_PWML0 */
#define PIO_PC0B_PWMC0_PWML0      (1u << 0)  /**< \brief Pwm0 signal: PWMC0_PWML0 */
#define PIO_PD10B_PWMC0_PWML0     (1u << 10) /**< \brief Pwm0 signal: PWMC0_PWML0 */
#define PIO_PD24A_PWMC0_PWML0     (1u << 24) /**< \brief Pwm0 signal: PWMC0_PWML0 */
#define PIO_PA20B_PWMC0_PWML1     (1u << 20) /**< \brief Pwm0 signal: PWMC0_PWML1 */
#define PIO_PB12A_PWMC0_PWML1     (1u << 12) /**< \brief Pwm0 signal: PWMC0_PWML1 */
#define PIO_PC1B_PWMC0_PWML1      (1u << 1)  /**< \brief Pwm0 signal: PWMC0_PWML1 */
#define PIO_PC18B_PWMC0_PWML1     (1u << 18) /**< \brief Pwm0 signal: PWMC0_PWML1 */
#define PIO_PD25A_PWMC0_PWML1     (1u << 25) /**< \brief Pwm0 signal: PWMC0_PWML1 */
#define PIO_PA16C_PWMC0_PWML2     (1u << 16) /**< \brief Pwm0 signal: PWMC0_PWML2 */
#define PIO_PA30A_PWMC0_PWML2     (1u << 30) /**< \brief Pwm0 signal: PWMC0_PWML2 */
#define PIO_PB13A_PWMC0_PWML2     (1u << 13) /**< \brief Pwm0 signal: PWMC0_PWML2 */
#define PIO_PC2B_PWMC0_PWML2      (1u << 2)  /**< \brief Pwm0 signal: PWMC0_PWML2 */
#define PIO_PC20B_PWMC0_PWML2     (1u << 20) /**< \brief Pwm0 signal: PWMC0_PWML2 */
#define PIO_PD26A_PWMC0_PWML2     (1u << 26) /**< \brief Pwm0 signal: PWMC0_PWML2 */
#define PIO_PA15C_PWMC0_PWML3     (1u << 15) /**< \brief Pwm0 signal: PWMC0_PWML3 */
#define PIO_PC3B_PWMC0_PWML3      (1u << 3)  /**< \brief Pwm0 signal: PWMC0_PWML3 */
#define PIO_PC15B_PWMC0_PWML3     (1u << 15) /**< \brief Pwm0 signal: PWMC0_PWML3 */
#define PIO_PC22B_PWMC0_PWML3     (1u << 22) /**< \brief Pwm0 signal: PWMC0_PWML3 */
#define PIO_PD27A_PWMC0_PWML3     (1u << 27) /**< \brief Pwm0 signal: PWMC0_PWML3 */
/* ========== Pio definition for PWM1 peripheral ========== */
#define PIO_PA30B_PWMC1_PWMEXTRG0 (1u << 30) /**< \brief Pwm1 signal: PWMC1_PWMEXTRG0 */
#define PIO_PA18A_PWMC1_PWMEXTRG1 (1u << 18) /**< \brief Pwm1 signal: PWMC1_PWMEXTRG1 */
#define PIO_PA21C_PWMC1_PWMFI0    (1u << 21) /**< \brief Pwm1 signal: PWMC1_PWMFI0 */
#define PIO_PA26D_PWMC1_PWMFI1    (1u << 26) /**< \brief Pwm1 signal: PWMC1_PWMFI1 */
#define PIO_PA28D_PWMC1_PWMFI2    (1u << 28) /**< \brief Pwm1 signal: PWMC1_PWMFI2 */
#define PIO_PA12C_PWMC1_PWMH0     (1u << 12) /**< \brief Pwm1 signal: PWMC1_PWMH0 */
#define PIO_PD1B_PWMC1_PWMH0      (1u << 1)  /**< \brief Pwm1 signal: PWMC1_PWMH0 */
#define PIO_PA14C_PWMC1_PWMH1     (1u << 14) /**< \brief Pwm1 signal: PWMC1_PWMH1 */
#define PIO_PD3B_PWMC1_PWMH1      (1u << 3)  /**< \brief Pwm1 signal: PWMC1_PWMH1 */
#define PIO_PA31D_PWMC1_PWMH2     (1u << 31) /**< \brief Pwm1 signal: PWMC1_PWMH2 */
#define PIO_PD5B_PWMC1_PWMH2      (1u << 5)  /**< \brief Pwm1 signal: PWMC1_PWMH2 */
#define PIO_PA8A_PWMC1_PWMH3      (1u << 8)  /**< \brief Pwm1 signal: PWMC1_PWMH3 */
#define PIO_PD7B_PWMC1_PWMH3      (1u << 7)  /**< \brief Pwm1 signal: PWMC1_PWMH3 */
#define PIO_PA11C_PWMC1_PWML0     (1u << 11) /**< \brief Pwm1 signal: PWMC1_PWML0 */
#define PIO_PD0B_PWMC1_PWML0      (1u << 0)  /**< \brief Pwm1 signal: PWMC1_PWML0 */
#define PIO_PA13C_PWMC1_PWML1     (1u << 13) /**< \brief Pwm1 signal: PWMC1_PWML1 */
#define PIO_PD2B_PWMC1_PWML1      (1u << 2)  /**< \brief Pwm1 signal: PWMC1_PWML1 */
#define PIO_PA23D_PWMC1_PWML2     (1u << 23) /**< \brief Pwm1 signal: PWMC1_PWML2 */
#define PIO_PD4B_PWMC1_PWML2      (1u << 4)  /**< \brief Pwm1 signal: PWMC1_PWML2 */
#define PIO_PA5A_PWMC1_PWML3      (1u << 5)  /**< \brief Pwm1 signal: PWMC1_PWML3 */
#define PIO_PD6B_PWMC1_PWML3      (1u << 6)  /**< \brief Pwm1 signal: PWMC1_PWML3 */
/* ========== Pio definition for QSPI peripheral ========== */
#define PIO_PA11A_QCS             (1u << 11) /**< \brief Qspi signal: QCS */
#define PIO_PA13A_QIO0            (1u << 13) /**< \brief Qspi signal: QIO0 */
#define PIO_PA12A_QIO1            (1u << 12) /**< \brief Qspi signal: QIO1 */
#define PIO_PA17A_QIO2            (1u << 17) /**< \brief Qspi signal: QIO2 */
#define PIO_PD31A_QIO3            (1u << 31) /**< \brief Qspi signal: QIO3 */
#define PIO_PA14A_QSCK            (1u << 14) /**< \brief Qspi signal: QSCK */
/* ========== Pio definition for SPI0 peripheral ========== */
#define PIO_PD20B_SPI0_MISO       (1u << 20) /**< \brief Spi0 signal: SPI0_MISO */
#define PIO_PD21B_SPI0_MOSI       (1u << 21) /**< \brief Spi0 signal: SPI0_MOSI */
#define PIO_PB2D_SPI0_NPCS0       (1u << 2)  /**< \brief Spi0 signal: SPI0_NPCS0 */
#define PIO_PA31A_SPI0_NPCS1      (1u << 31) /**< \brief Spi0 signal: SPI0_NPCS1 */
#define PIO_PD25B_SPI0_NPCS1      (1u << 25) /**< \brief Spi0 signal: SPI0_NPCS1 */
#define PIO_PD12C_SPI0_NPCS2      (1u << 12) /**< \brief Spi0 signal: SPI0_NPCS2 */
#define PIO_PD27B_SPI0_NPCS3      (1u << 27) /**< \brief Spi0 signal: SPI0_NPCS3 */
#define PIO_PD22B_SPI0_SPCK       (1u << 22) /**< \brief Spi0 signal: SPI0_SPCK */
/* ========== Pio definition for SPI1 peripheral ========== */
#define PIO_PC26C_SPI1_MISO       (1u << 26) /**< \brief Spi1 signal: SPI1_MISO */
#define PIO_PC27C_SPI1_MOSI       (1u << 27) /**< \brief Spi1 signal: SPI1_MOSI */
#define PIO_PC25C_SPI1_NPCS0      (1u << 25) /**< \brief Spi1 signal: SPI1_NPCS0 */
#define PIO_PC28C_SPI1_NPCS1      (1u << 28) /**< \brief Spi1 signal: SPI1_NPCS1 */
#define PIO_PD0C_SPI1_NPCS1       (1u << 0)  /**< \brief Spi1 signal: SPI1_NPCS1 */
#define PIO_PC29C_SPI1_NPCS2      (1u << 29) /**< \brief Spi1 signal: SPI1_NPCS2 */
#define PIO_PD1C_SPI1_NPCS2       (1u << 1)  /**< \brief Spi1 signal: SPI1_NPCS2 */
#define PIO_PC30C_SPI1_NPCS3      (1u << 30) /**< \brief Spi1 signal: SPI1_NPCS3 */
#define PIO_PD2C_SPI1_NPCS3       (1u << 2)  /**< \brief Spi1 signal: SPI1_NPCS3 */
#define PIO_PC24C_SPI1_SPCK       (1u << 24) /**< \brief Spi1 signal: SPI1_SPCK */
/* ========== Pio definition for SSC peripheral ========== */
#define PIO_PA10C_RD              (1u << 10) /**< \brief Ssc signal: RD */
#define PIO_PD24B_RF              (1u << 24) /**< \brief Ssc signal: RF */
#define PIO_PA22A_RK              (1u << 22) /**< \brief Ssc signal: RK */
#define PIO_PB5D_TD               (1u << 5)  /**< \brief Ssc signal: TD */
#define PIO_PD10C_TD              (1u << 10) /**< \brief Ssc signal: TD */
#define PIO_PD26B_TD              (1u << 26) /**< \brief Ssc signal: TD */
#define PIO_PB0D_TF               (1u << 0)  /**< \brief Ssc signal: TF */
#define PIO_PB1D_TK               (1u << 1)  /**< \brief Ssc signal: TK */
/* ========== Pio definition for TC0 peripheral ========== */
#define PIO_PA4B_TCLK0            (1u << 4)  /**< \brief Tc0 signal: TCLK0 */
#define PIO_PA28B_TCLK1           (1u << 28) /**< \brief Tc0 signal: TCLK1 */
#define PIO_PA29B_TCLK2           (1u << 29) /**< \brief Tc0 signal: TCLK2 */
#define PIO_PA0B_TIOA0            (1u << 0)  /**< \brief Tc0 signal: TIOA0 */
#define PIO_PA15B_TIOA1           (1u << 15) /**< \brief Tc0 signal: TIOA1 */
#define PIO_PA26B_TIOA2           (1u << 26) /**< \brief Tc0 signal: TIOA2 */
#define PIO_PA1B_TIOB0            (1u << 1)  /**< \brief Tc0 signal: TIOB0 */
#define PIO_PA16B_TIOB1           (1u << 16) /**< \brief Tc0 signal: TIOB1 */
#define PIO_PA27B_TIOB2           (1u << 27) /**< \brief Tc0 signal: TIOB2 */
/* ========== Pio definition for TC1 peripheral ========== */
#define PIO_PC25B_TCLK3           (1u << 25) /**< \brief Tc1 signal: TCLK3 */
#define PIO_PC28B_TCLK4           (1u << 28) /**< \brief Tc1 signal: TCLK4 */
#define PIO_PC31B_TCLK5           (1u << 31) /**< \brief Tc1 signal: TCLK5 */
#define PIO_PC23B_TIOA3           (1u << 23) /**< \brief Tc1 signal: TIOA3 */
#define PIO_PC26B_TIOA4           (1u << 26) /**< \brief Tc1 signal: TIOA4 */
#define PIO_PC29B_TIOA5           (1u << 29) /**< \brief Tc1 signal: TIOA5 */
#define PIO_PC24B_TIOB3           (1u << 24) /**< \brief Tc1 signal: TIOB3 */
#define PIO_PC27B_TIOB4           (1u << 27) /**< \brief Tc1 signal: TIOB4 */
#define PIO_PC30B_TIOB5           (1u << 30) /**< \brief Tc1 signal: TIOB5 */
/* ========== Pio definition for TC2 peripheral ========== */
#define PIO_PC7B_TCLK6            (1u << 7)  /**< \brief Tc2 signal: TCLK6 */
#define PIO_PC10B_TCLK7           (1u << 10) /**< \brief Tc2 signal: TCLK7 */
#define PIO_PC14B_TCLK8           (1u << 14) /**< \brief Tc2 signal: TCLK8 */
#define PIO_PC5B_TIOA6            (1u << 5)  /**< \brief Tc2 signal: TIOA6 */
#define PIO_PC8B_TIOA7            (1u << 8)  /**< \brief Tc2 signal: TIOA7 */
#define PIO_PC11B_TIOA8           (1u << 11) /**< \brief Tc2 signal: TIOA8 */
#define PIO_PC6B_TIOB6            (1u << 6)  /**< \brief Tc2 signal: TIOB6 */
#define PIO_PC9B_TIOB7            (1u << 9)  /**< \brief Tc2 signal: TIOB7 */
#define PIO_PC12B_TIOB8           (1u << 12) /**< \brief Tc2 signal: TIOB8 */
/* ========== Pio definition for TC3 peripheral ========== */
#define PIO_PE5B_TCLK10           (1u << 5)  /**< \brief Tc3 signal: TCLK10 */
#define PIO_PD24C_TCLK11          (1u << 24) /**< \brief Tc3 signal: TCLK11 */
#define PIO_PE2B_TCLK9            (1u << 2)  /**< \brief Tc3 signal: TCLK9 */
#define PIO_PE3B_TIOA10           (1u << 3)  /**< \brief Tc3 signal: TIOA10 */
#define PIO_PD21C_TIOA11          (1u << 21) /**< \brief Tc3 signal: TIOA11 */
#define PIO_PE0B_TIOA9            (1u << 0)  /**< \brief Tc3 signal: TIOA9 */
#define PIO_PE4B_TIOB10           (1u << 4)  /**< \brief Tc3 signal: TIOB10 */
#define PIO_PD22C_TIOB11          (1u << 22) /**< \brief Tc3 signal: TIOB11 */
#define PIO_PE1B_TIOB9            (1u << 1)  /**< \brief Tc3 signal: TIOB9 */
/* ========== Pio definition for TWIHS0 peripheral ========== */
#define PIO_PA4A_TWCK0            (1u << 4)  /**< \brief Twihs0 signal: TWCK0 */
#define PIO_PA3A_TWD0             (1u << 3)  /**< \brief Twihs0 signal: TWD0 */
/* ========== Pio definition for TWIHS1 peripheral ========== */
#define PIO_PB5A_TWCK1            (1u << 5)  /**< \brief Twihs1 signal: TWCK1 */
#define PIO_PB4A_TWD1             (1u << 4)  /**< \brief Twihs1 signal: TWD1 */
/* ========== Pio definition for TWIHS2 peripheral ========== */
#define PIO_PD28C_TWCK2           (1u << 28) /**< \brief Twihs2 signal: TWCK2 */
#define PIO_PD27C_TWD2            (1u << 27) /**< \brief Twihs2 signal: TWD2 */
/* ========== Pio definition for UART0 peripheral ========== */
#define PIO_PA9A_URXD0            (1u << 9)  /**< \brief Uart0 signal: URXD0 */
#define PIO_PA10A_UTXD0           (1u << 10) /**< \brief Uart0 signal: UTXD0 */
/* ========== Pio definition for UART1 peripheral ========== */
#define PIO_PA5C_URXD1            (1u << 5)  /**< \brief Uart1 signal: URXD1 */
#define PIO_PA4C_UTXD1            (1u << 4)  /**< \brief Uart1 signal: UTXD1 */
#define PIO_PA6C_UTXD1            (1u << 6)  /**< \brief Uart1 signal: UTXD1 */
#define PIO_PD26D_UTXD1           (1u << 26) /**< \brief Uart1 signal: UTXD1 */
/* ========== Pio definition for UART2 peripheral ========== */
#define PIO_PD25C_URXD2           (1u << 25) /**< \brief Uart2 signal: URXD2 */
#define PIO_PD26C_UTXD2           (1u << 26) /**< \brief Uart2 signal: UTXD2 */
/* ========== Pio definition for UART3 peripheral ========== */
#define PIO_PD28A_URXD3           (1u << 28) /**< \brief Uart3 signal: URXD3 */
#define PIO_PD30A_UTXD3           (1u << 30) /**< \brief Uart3 signal: UTXD3 */
#define PIO_PD31B_UTXD3           (1u << 31) /**< \brief Uart3 signal: UTXD3 */
/* ========== Pio definition for UART4 peripheral ========== */
#define PIO_PD18C_URXD4           (1u << 18) /**< \brief Uart4 signal: URXD4 */
#define PIO_PD3C_UTXD4            (1u << 3)  /**< \brief Uart4 signal: UTXD4 */
#define PIO_PD19C_UTXD4           (1u << 19) /**< \brief Uart4 signal: UTXD4 */
/* ========== Pio definition for USART0 peripheral ========== */
#define PIO_PB2C_CTS0             (1u << 2)  /**< \brief Usart0 signal: CTS0 */
#define PIO_PD0D_DCD0             (1u << 0)  /**< \brief Usart0 signal: DCD0 */
#define PIO_PD2D_DSR0             (1u << 2)  /**< \brief Usart0 signal: DSR0 */
#define PIO_PD1D_DTR0             (1u << 1)  /**< \brief Usart0 signal: DTR0 */
#define PIO_PD3D_RI0              (1u << 3)  /**< \brief Usart0 signal: RI0 */
#define PIO_PB3C_RTS0             (1u << 3)  /**< \brief Usart0 signal: RTS0 */
#define PIO_PB0C_RXD0             (1u << 0)  /**< \brief Usart0 signal: RXD0 */
#define PIO_PB13C_SCK0            (1u << 13) /**< \brief Usart0 signal: SCK0 */
#define PIO_PB1C_TXD0             (1u << 1)  /**< \brief Usart0 signal: TXD0 */
/* ========== Pio definition for USART1 peripheral ========== */
#define PIO_PA25A_CTS1            (1u << 25) /**< \brief Usart1 signal: CTS1 */
#define PIO_PA26A_DCD1            (1u << 26) /**< \brief Usart1 signal: DCD1 */
#define PIO_PA28A_DSR1            (1u << 28) /**< \brief Usart1 signal: DSR1 */
#define PIO_PA27A_DTR1            (1u << 27) /**< \brief Usart1 signal: DTR1 */
#define PIO_PA3B_LONCOL1          (1u << 3)  /**< \brief Usart1 signal: LONCOL1 */
#define PIO_PA29A_RI1             (1u << 29) /**< \brief Usart1 signal: RI1 */
#define PIO_PA24A_RTS1            (1u << 24) /**< \brief Usart1 signal: RTS1 */
#define PIO_PA21A_RXD1            (1u << 21) /**< \brief Usart1 signal: RXD1 */
#define PIO_PA23A_SCK1            (1u << 23) /**< \brief Usart1 signal: SCK1 */
#define PIO_PB4D_TXD1             (1u << 4)  /**< \brief Usart1 signal: TXD1 */
/* ========== Pio definition for USART2 peripheral ========== */
#define PIO_PD19B_CTS2            (1u << 19) /**< \brief Usart2 signal: CTS2 */
#define PIO_PD4D_DCD2             (1u << 4)  /**< \brief Usart2 signal: DCD2 */
#define PIO_PD6D_DSR2             (1u << 6)  /**< \brief Usart2 signal: DSR2 */
#define PIO_PD5D_DTR2             (1u << 5)  /**< \brief Usart2 signal: DTR2 */
#define PIO_PD7D_RI2              (1u << 7)  /**< \brief Usart2 signal: RI2 */
#define PIO_PD18B_RTS2            (1u << 18) /**< \brief Usart2 signal: RTS2 */
#define PIO_PD15B_RXD2            (1u << 15) /**< \brief Usart2 signal: RXD2 */
#define PIO_PD17B_SCK2            (1u << 17) /**< \brief Usart2 signal: SCK2 */
#define PIO_PD16B_TXD2            (1u << 16) /**< \brief Usart2 signal: TXD2 */
/* ========== Pio definition for UTMI peripheral ========== */
#define PIO_PC12D_UTMI_CDRBISTEN   (1u << 12) /**< \brief Utmi signal: UTMI_CDRBISTEN */
#define PIO_PC29D_UTMI_CDRCPDIVEN  (1u << 29) /**< \brief Utmi signal: UTMI_CDRCPDIVEN */
#define PIO_PC31D_UTMI_CDRCPSEL0   (1u << 31) /**< \brief Utmi signal: UTMI_CDRCPSEL0 */
#define PIO_PC30D_UTMI_CDRCPSEL1   (1u << 30) /**< \brief Utmi signal: UTMI_CDRCPSEL1 */
#define PIO_PC15D_UTMI_CDRCPSELDIV (1u << 15) /**< \brief Utmi signal: UTMI_CDRCPSELDIV */
#define PIO_PC0D_UTMI_HDIS         (1u << 0)  /**< \brief Utmi signal: UTMI_HDIS */
#define PIO_PC27D_UTMI_LS0         (1u << 27) /**< \brief Utmi signal: UTMI_LS0 */
#define PIO_PC26D_UTMI_LS1         (1u << 26) /**< \brief Utmi signal: UTMI_LS1 */
#define PIO_PE3D_UTMI_RXACT        (1u << 3)  /**< \brief Utmi signal: UTMI_RXACT */
#define PIO_PE2D_UTMI_RXERR        (1u << 2)  /**< \brief Utmi signal: UTMI_RXERR */
#define PIO_PE1D_UTMI_RXVAL        (1u << 1)  /**< \brief Utmi signal: UTMI_RXVAL */
/* ========== Pio indexes ========== */
#define PIO_PA0_IDX               0
#define PIO_PA1_IDX               1
#define PIO_PA2_IDX               2
#define PIO_PA3_IDX               3
#define PIO_PA4_IDX               4
#define PIO_PA5_IDX               5
#define PIO_PA6_IDX               6
#define PIO_PA7_IDX               7
#define PIO_PA8_IDX               8
#define PIO_PA9_IDX               9
#define PIO_PA10_IDX              10
#define PIO_PA11_IDX              11
#define PIO_PA12_IDX              12
#define PIO_PA13_IDX              13
#define PIO_PA14_IDX              14
#define PIO_PA15_IDX              15
#define PIO_PA16_IDX              16
#define PIO_PA17_IDX              17
#define PIO_PA18_IDX              18
#define PIO_PA19_IDX              19
#define PIO_PA20_IDX              20
#define PIO_PA21_IDX              21
#define PIO_PA22_IDX              22
#define PIO_PA23_IDX              23
#define PIO_PA24_IDX              24
#define PIO_PA25_IDX              25
#define PIO_PA26_IDX              26
#define PIO_PA27_IDX              27
#define PIO_PA28_IDX              28
#define PIO_PA29_IDX              29
#define PIO_PA30_IDX              30
#define PIO_PA31_IDX              31
#define PIO_PB0_IDX               32
#define PIO_PB1_IDX               33
#define PIO_PB2_IDX               34
#define PIO_PB3_IDX               35
#define PIO_PB4_IDX               36
#define PIO_PB5_IDX               37
#define PIO_PB6_IDX               38
#define PIO_PB7_IDX               39
#define PIO_PB8_IDX               40
#define PIO_PB9_IDX               41
#define PIO_PB12_IDX              44
#define PIO_PB13_IDX              45
#define PIO_PC0_IDX               64
#define PIO_PC1_IDX               65
#define PIO_PC2_IDX               66
#define PIO_PC3_IDX               67
#define PIO_PC4_IDX               68
#define PIO_PC5_IDX               69
#define PIO_PC6_IDX               70
#define PIO_PC7_IDX               71
#define PIO_PC8_IDX               72
#define PIO_PC9_IDX               73
#define PIO_PC10_IDX              74
#define PIO_PC11_IDX              75
#define PIO_PC12_IDX              76
#define PIO_PC13_IDX              77
#define PIO_PC14_IDX              78
#define PIO_PC15_IDX              79
#define PIO_PC16_IDX              80
#define PIO_PC17_IDX              81
#define PIO_PC18_IDX              82
#define PIO_PC19_IDX              83
#define PIO_PC20_IDX              84
#define PIO_PC21_IDX              85
#define PIO_PC22_IDX              86
#define PIO_PC23_IDX              87
#define PIO_PC24_IDX              88
#define PIO_PC25_IDX              89
#define PIO_PC26_IDX              90
#define PIO_PC27_IDX              91
#define PIO_PC28_IDX              92
#define PIO_PC29_IDX              93
#define PIO_PC30_IDX              94
#define PIO_PC31_IDX              95
#define PIO_PD0_IDX               96
#define PIO_PD1_IDX               97
#define PIO_PD2_IDX               98
#define PIO_PD3_IDX               99
#define PIO_PD4_IDX               100
#define PIO_PD5_IDX               101
#define PIO_PD6_IDX               102
#define PIO_PD7_IDX               103
#define PIO_PD8_IDX               104
#define PIO_PD9_IDX               105
#define PIO_PD10_IDX              106
#define PIO_PD11_IDX              107
#define PIO_PD12_IDX              108
#define PIO_PD13_IDX              109
#define PIO_PD14_IDX              110
#define PIO_PD15_IDX              111
#define PIO_PD16_IDX              112
#define PIO_PD17_IDX              113
#define PIO_PD18_IDX              114
#define PIO_PD19_IDX              115
#define PIO_PD20_IDX              116
#define PIO_PD21_IDX              117
#define PIO_PD22_IDX              118
#define PIO_PD23_IDX              119
#define PIO_PD24_IDX              120
#define PIO_PD25_IDX              121
#define PIO_PD26_IDX              122
#define PIO_PD27_IDX              123
#define PIO_PD28_IDX              124
#define PIO_PD29_IDX              125
#define PIO_PD30_IDX              126
#define PIO_PD31_IDX              127
#define PIO_PE0_IDX               128
#define PIO_PE1_IDX               129
#define PIO_PE2_IDX               130
#define PIO_PE3_IDX               131
#define PIO_PE4_IDX               132
#define PIO_PE5_IDX               133

#endif /* _SAME70Q20_PIO_ */

/*@}*/

/* ************************************************************************** */
/*   MEMORY MAPPING DEFINITIONS FOR SAME70Q20 */
/* ************************************************************************** */

#define IFLASH_SIZE             (0x100000u)
#define IFLASH_PAGE_SIZE        (512u)
#define IFLASH_LOCK_REGION_SIZE (16384u)
#define IFLASH_NB_OF_PAGES      (2048u)
#define IFLASH_NB_OF_LOCK_BITS  (64u)
#define IRAM_SIZE               (0x60000u)

#define QSPIMEM_ADDR  (0x80000000u) /**< QSPI Memory base address */
#define AXIMX_ADDR    (0xA0000000u) /**< AXI Bus Matrix base address */
#define ITCM_ADDR     (0x00000000u) /**< Instruction Tightly Coupled Memory base address */
#define IFLASH_ADDR   (0x00400000u) /**< Internal Flash base address */
#define IROM_ADDR     (0x00800000u) /**< Internal ROM base address */
#define DTCM_ADDR     (0x20000000u) /**< Data Tightly Coupled Memory base address */
#define IRAM_ADDR     (0x20400000u) /**< Internal RAM base address */
#define EBI_CS0_ADDR  (0x60000000u) /**< EBI Chip Select 0 base address */
#define EBI_CS1_ADDR  (0x61000000u) /**< EBI Chip Select 1 base address */
#define EBI_CS2_ADDR  (0x62000000u) /**< EBI Chip Select 2 base address */
#define EBI_CS3_ADDR  (0x63000000u) /**< EBI Chip Select 3 base address */
#define SDRAM_CS_ADDR (0x70000000u) /**< SDRAM Chip Select base address */

/* ************************************************************************** */
/*   MISCELLANEOUS DEFINITIONS FOR SAME70Q20 */
/* ************************************************************************** */

#define CHIP_JTAGID (0x05B3D03FUL)
#define CHIP_CIDR   (0xA1020C01UL)
#define CHIP_EXID   (0x00000002UL)

/* ************************************************************************** */
/*   ELECTRICAL DEFINITIONS FOR SAME70Q20 */
/* ************************************************************************** */

/* %ATMEL_ELECTRICAL% */

/* Device characteristics */
#define CHIP_FREQ_SLCK_RC_MIN           (20000UL)
#define CHIP_FREQ_SLCK_RC               (32000UL)
#define CHIP_FREQ_SLCK_RC_MAX           (44000UL)
#define CHIP_FREQ_MAINCK_RC_4MHZ        (4000000UL)
#define CHIP_FREQ_MAINCK_RC_8MHZ        (8000000UL)
#define CHIP_FREQ_MAINCK_RC_12MHZ       (12000000UL)
#define CHIP_FREQ_CPU_MAX               (300000000UL)
#define CHIP_FREQ_XTAL_32K              (32768UL)
#define CHIP_FREQ_XTAL_12M              (12000000UL)

/* Embedded Flash Read Wait State (for Worst-Case Conditions) */
#define CHIP_FREQ_FWS_0                 (23000000UL)  /**< \brief Maximum operating frequency when FWS is 0 */
#define CHIP_FREQ_FWS_1                 (46000000UL)  /**< \brief Maximum operating frequency when FWS is 1 */
#define CHIP_FREQ_FWS_2                 (69000000UL)  /**< \brief Maximum operating frequency when FWS is 2 */
#define CHIP_FREQ_FWS_3                 (92000000UL)  /**< \brief Maximum operating frequency when FWS is 3 */
#define CHIP_FREQ_FWS_4                 (115000000UL) /**< \brief Maximum operating frequency when FWS is 4 */
#define CHIP_FREQ_FWS_5                 (138000000UL) /**< \brief Maximum operating frequency when FWS is 5 */
#define CHIP_FREQ_FWS_6                 (150000000UL) /**< \brief Maximum operating frequency when FWS is 6 */

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* _SAME70Q20_ */

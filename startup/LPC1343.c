//*****************************************************************************
//   +--+       
//   | ++----+   
//   +-++    |  
//     |     |  
//   +-+--+  |   
//   | +--+--+  
//   +----+    Copyright (c) 2009 Code Red Technologies Ltd. 
//
// Microcontroller Startup code for use with Red Suite
//
// Software License Agreement
// 
// The software is owned by Code Red Technologies and/or its suppliers, and is 
// protected under applicable copyright laws.  All rights are reserved.  Any 
// use in violation of the foregoing restrictions may subject the user to criminal 
// sanctions under applicable laws, as well as to civil liability for the breach 
// of the terms and conditions of this license.
// 
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// USE OF THIS SOFTWARE FOR COMMERCIAL DEVELOPMENT AND/OR EDUCATION IS SUBJECT
// TO A CURRENT END USER LICENSE AGREEMENT (COMMERCIAL OR EDUCATIONAL) WITH
// CODE RED TECHNOLOGIES LTD. 
//
//*****************************************************************************
#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

//*****************************************************************************
//
// Forward declaration of the default handlers. These are aliased.
// When the application defines a handler (with the same name), this will 
// automatically take precedence over these weak definitions
//
//*****************************************************************************
void reset_handler(void);
WEAK void dummy_handler(void);

//*****************************************************************************
//
// Forward declaration of the specific IRQ handlers. These are aliased
// to the intdefaulthandler, which is a 'forever' loop. When the application
// defines a handler (with the same name), this will automatically take 
// precedence over these weak definitions
//
//*****************************************************************************

void i2c_isr(void) ALIAS(intdefaulthandler);
void timer16_0_isr(void) ALIAS(intdefaulthandler);
void timer16_1_isr(void) ALIAS(intdefaulthandler);
void timer32_0_isr(void) ALIAS(intdefaulthandler);
void timer32_1_isr(void) ALIAS(intdefaulthandler);
void ssp_isr(void) ALIAS(intdefaulthandler);
void uart_isr(void) ALIAS(intdefaulthandler);
void usb_isr(void) ALIAS(intdefaulthandler);
void usb_fiq_handler(void) ALIAS(intdefaulthandler);
void adc_isr(void) ALIAS(intdefaulthandler);
void wdt_isr(void) ALIAS(intdefaulthandler);
void bod_isr(void) ALIAS(intdefaulthandler);
void fmc_isr(void) ALIAS(intdefaulthandler);
void pioint3_isr(void) ALIAS(intdefaulthandler);
void pioint2_isr(void) ALIAS(intdefaulthandler);
void pioint1_isr(void) ALIAS(intdefaulthandler);
void pioint0_isr(void) ALIAS(intdefaulthandler);

void wkup_isr_pio0_0 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_1 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_2 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_3 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_4 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_5 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_6 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_7 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_8 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_9 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_10(void) ALIAS(intdefaulthandler);
void wkup_isr_pio0_11(void) ALIAS(intdefaulthandler);

void wkup_isr_pio1_0 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_1 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_2 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_3 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_4 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_5 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_6 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_7 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_8 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_9 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_10(void) ALIAS(intdefaulthandler);
void wkup_isr_pio1_11(void) ALIAS(intdefaulthandler);

void wkup_isr_pio2_0 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_1 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_2 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_3 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_4 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_5 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_6 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_7 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_8 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_9 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_10(void) ALIAS(intdefaulthandler);
void wkup_isr_pio2_11(void) ALIAS(intdefaulthandler);

void wkup_isr_pio3_0 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio3_1 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio3_2 (void) ALIAS(intdefaulthandler);
void wkup_isr_pio3_3 (void) ALIAS(intdefaulthandler);

/* The entry point for the application. */
extern WEAK void main(void);
//*****************************************************************************
//
// External declaration for the pointer to the stack top from the Linker Script
//
//*****************************************************************************
extern void __stack_end__(void);

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__attribute__ ((section(".isr_vector")))
void (*const isr_vectors[]) (void) =
{
	// Core Level - CM3
	__stack_end__,		// The initial stack pointer
	reset_handler,		// The reset handler
	dummy_handler,		// The NMI handler
	dummy_handler,		// The hard fault handler
	dummy_handler,		// The MPU fault handler
	dummy_handler,		// The bus fault handler
	dummy_handler,		// The usage fault handler
	0,			// Reserved
	0,			// Reserved
	0,			// Reserved
	0,			// Reserved
	dummy_handler,		// SVCall handler
	dummy_handler,		// Debug monitor handler
	0,			// Reserved
	dummy_handler,		// The PendSV handler
	dummy_handler,		// The SysTick handler
	// Wakeup sources (40 ea.) for the I/O pins:
	//   PIO0 (0:11)
	//   PIO1 (0:11)
	//   PIO2 (0:11)
	//   PIO3 (0:3)
	wkup_isr_pio0_0,	// PIO0_0  Wakeup
	wkup_isr_pio0_1,	// PIO0_1  Wakeup
	wkup_isr_pio0_2,	// PIO0_2  Wakeup
	wkup_isr_pio0_3,	// PIO0_3  Wakeup
	wkup_isr_pio0_4,	// PIO0_4  Wakeup
	wkup_isr_pio0_5,	// PIO0_5  Wakeup
	wkup_isr_pio0_6,	// PIO0_6  Wakeup
	wkup_isr_pio0_7,	// PIO0_7  Wakeup
	wkup_isr_pio0_8,	// PIO0_8  Wakeup
	wkup_isr_pio0_9,	// PIO0_9  Wakeup
	wkup_isr_pio0_10,	// PIO0_10 Wakeup
	wkup_isr_pio0_11,	// PIO0_11 Wakeup
	wkup_isr_pio1_0,	// PIO1_0  Wakeup
	wkup_isr_pio1_1,	// PIO1_1  Wakeup
	wkup_isr_pio1_2,	// PIO1_2  Wakeup
	wkup_isr_pio1_3,	// PIO1_3  Wakeup
	wkup_isr_pio1_4,	// PIO1_4  Wakeup
	wkup_isr_pio1_5,	// PIO1_5  Wakeup
	wkup_isr_pio1_6,	// PIO1_6  Wakeup
	wkup_isr_pio1_7,	// PIO1_7  Wakeup
	wkup_isr_pio1_8,	// PIO1_8  Wakeup
	wkup_isr_pio1_9,	// PIO1_9  Wakeup
	wkup_isr_pio1_10,	// PIO1_10 Wakeup
	wkup_isr_pio1_11,	// PIO1_11 Wakeup
	wkup_isr_pio2_0,	// PIO2_0  Wakeup
	wkup_isr_pio2_1,	// PIO2_1  Wakeup
	wkup_isr_pio2_2,	// PIO2_2  Wakeup
	wkup_isr_pio2_3,	// PIO2_3  Wakeup
	wkup_isr_pio2_4,	// PIO2_4  Wakeup
	wkup_isr_pio2_5,	// PIO2_5  Wakeup
	wkup_isr_pio2_6,	// PIO2_6  Wakeup
	wkup_isr_pio2_7,	// PIO2_7  Wakeup
	wkup_isr_pio2_8,	// PIO2_8  Wakeup
	wkup_isr_pio2_9,	// PIO2_9  Wakeup
	wkup_isr_pio2_10,	// PIO2_10 Wakeup
	wkup_isr_pio2_11,	// PIO2_11 Wakeup
	wkup_isr_pio3_0,	// PIO3_0  Wakeup
	wkup_isr_pio3_1,	// PIO3_1  Wakeup
	wkup_isr_pio3_2,	// PIO3_2  Wakeup
	wkup_isr_pio3_3,	// PIO3_3  Wakeup
	i2c_isr,		// I2C0
	timer16_0_isr,		// CT16B0 (16-bit Timer 0)
	timer16_1_isr,		// CT16B1 (16-bit Timer 1)
	timer32_0_isr,		// CT32B0 (32-bit Timer 0)
	timer32_1_isr,		// CT32B1 (32-bit Timer 1)
	ssp_isr,		// SSP0
	uart_isr,		// UART0
	usb_isr,		// USB IRQ
	usb_fiq_handler,	// USB FIQ
	adc_isr,		// ADC   (A/D Converter)
	wdt_isr,		// WDT   (Watchdog Timer)
	bod_isr,		// BOD   (Brownout Detect)
	fmc_isr,		// Flash (IP2111 Flash Memory Controller)
	pioint3_isr,		// PIO INT3
	pioint2_isr,		// PIO INT2
	pioint1_isr,		// PIO INT1
	pioint0_isr,		// PIO INT0
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern unsigned long __end_of_text__;
extern unsigned long __data_beg__;
extern unsigned long __data_end__;
extern unsigned long __bss_beg__;
extern unsigned long __bss_end__;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void reset_handler(void) {
    unsigned long *pulSrc, *pulDest;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pulSrc = &__end_of_text__;
    for (pulDest = &__data_beg__; pulDest < &__data_end__;) {
	*pulDest++ = *pulSrc++;
    }

    //
    // Zero fill the bss segment.  This is done with inline assembly since this
    // will clear the value of pulDest if it is not kept in a register.
    //
    __asm("    ldr     r0, =__bss_beg__\n" "    ldr     r1, =__bss_end__\n"
	  "    mov     r2, #0\n" "    .thumb_func\n" "zero_loop:\n"
	  "        cmp     r0, r1\n" "        it      lt\n"
	  "        strlt   r2, [r0], #4\n" "        blt     zero_loop");

    while (1)
    main();
}

/*
 * This is the code that gets called when the processor receives a NMI.  This
 * simply enters an infinite loop, preserving the system state for examination
 * by a debugger.
 */
void serial_write_str(const char *str);
void dummy_handler(void) {
	/* TODO: check if UART is enabled */
	serial_write_str("Got some kind of fault\r\n");
	while (1);
}

/*
 * Processor ends up here if an unexpected interrupt occurs or a handler
 * is not present in the application code.
 */
static void intdefaulthandler(void) {
	/* Do nothing */
}

/* Libc replacement */
int __errno = 0;

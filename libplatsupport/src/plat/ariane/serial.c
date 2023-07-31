/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <string.h>
#include <stdlib.h>
#include <platsupport/serial.h>
#include "../../chardev.h"

static void uart_handle_irq(ps_chardevice_t *dev)
{
    /* nothing to do, interrupts are not used */
}

#define UART_RBR_OFFSET     0   /* In:  Recieve Buffer Register */
#define UART_THR_OFFSET     0   /* Out: Transmitter Holding Register */
#define UART_DLL_OFFSET     0   /* Out: Divisor Latch Low */
#define UART_IER_OFFSET     1   /* I/O: Interrupt Enable Register */
#define UART_DLM_OFFSET     1   /* Out: Divisor Latch High */
#define UART_FCR_OFFSET     2   /* Out: FIFO Control Register */
#define UART_IIR_OFFSET     2   /* I/O: Interrupt Identification Register */
#define UART_LCR_OFFSET     3   /* Out: Line Control Register */
#define UART_MCR_OFFSET     4   /* Out: Modem Control Register */
#define UART_LSR_OFFSET     5   /* In:  Line Status Register */
#define UART_MSR_OFFSET     6   /* In:  Modem Status Register */
#define UART_SCR_OFFSET     7   /* I/O: Scratch Register */
#define UART_MDR1_OFFSET    8   /* I/O:  Mode Register */

#define UART_LSR_FIFOE      0x80    /* Fifo error */
#define UART_LSR_TEMT       0x40    /* Transmitter empty */
#define UART_LSR_THRE       0x20    /* Transmit-hold-register empty */
#define UART_LSR_BI     0x10    /* Break interrupt indicator */
#define UART_LSR_FE     0x08    /* Frame error indicator */
#define UART_LSR_PE     0x04    /* Parity error indicator */
#define UART_LSR_OE     0x02    /* Overrun error indicator */
#define UART_LSR_DR     0x01    /* Receiver data ready */
#define UART_LSR_BRK_ERROR_BITS 0x1E    /* BI, FE, PE, OE bits */

#define ARIANE_UART_ADDR			0x10000000
#define ARIANE_UART_FREQ			50000000
#define ARIANE_UART_BAUDRATE		115200
// #define ARIANE_UART_BAUDRATE		25000
#define ARIANE_UART_REG_SHIFT		2
#define ARIANE_UART_REG_WIDTH		4

static volatile void *uart8250_base;
static uint32_t uart8250_in_freq;
static uint32_t uart8250_baudrate;
static uint32_t uart8250_reg_width;
static uint32_t uart8250_reg_shift;

static inline void __raw_writeb(uint8_t val, volatile void *addr)
{
	asm volatile("sb %0, 0(%1)" : : "r"(val), "r"(addr));
}

static inline void __raw_writew(uint16_t val, volatile void *addr)
{
	asm volatile("sh %0, 0(%1)" : : "r"(val), "r"(addr));
}

static inline void __raw_writel(uint32_t val, volatile void *addr)
{
	asm volatile("sw %0, 0(%1)" : : "r"(val), "r"(addr));
}

static inline uint8_t __raw_readb(const volatile void *addr)
{
	uint8_t val;

	asm volatile("lb %0, 0(%1)" : "=r"(val) : "r"(addr));
	return val;
}

static inline uint16_t __raw_readw(const volatile void *addr)
{
	uint16_t val;

	asm volatile("lh %0, 0(%1)" : "=r"(val) : "r"(addr));
	return val;
}

static inline uint32_t __raw_readl(const volatile void *addr)
{
	uint32_t val;

	asm volatile("lw %0, 0(%1)" : "=r"(val) : "r"(addr));
	return val;
}


#define __io_br()	do {} while (0)
#define __io_ar()	__asm__ __volatile__ ("fence i,r" : : : "memory");
#define __io_bw()	__asm__ __volatile__ ("fence w,o" : : : "memory");
#define __io_aw()	do {} while (0)

#define readb(c)	({ uint8_t  __v; __io_br(); __v = __raw_readb(c); __io_ar(); __v; })
#define readw(c)	({ uint16_t __v; __io_br(); __v = __raw_readw(c); __io_ar(); __v; })
#define readl(c)	({ uint32_t __v; __io_br(); __v = __raw_readl(c); __io_ar(); __v; })

#define writeb(v,c)	({ __io_bw(); __raw_writeb((v),(c)); __io_aw(); })
#define writew(v,c)	({ __io_bw(); __raw_writew((v),(c)); __io_aw(); })
#define writel(v,c)	({ __io_bw(); __raw_writel((v),(c)); __io_aw(); })

static uint32_t get_reg(uint32_t num)
{
	uint32_t offset = num << uart8250_reg_shift;

	if (uart8250_reg_width == 1)
		return readb(uart8250_base + offset);
	else if (uart8250_reg_width == 2)
		return readw(uart8250_base + offset);
	else
		return readl(uart8250_base + offset);
}

static void set_reg(uint32_t num, uint32_t val)
{
	uint32_t offset = num << uart8250_reg_shift;

	if (uart8250_reg_width == 1)
		writeb(val, uart8250_base + offset);
	else if (uart8250_reg_width == 2)
		writew(val, uart8250_base + offset);
	else
		writel(val, uart8250_base + offset);
}

int uart_putchar(ps_chardevice_t *dev, int c)
{
    while ((get_reg(UART_LSR_OFFSET) & UART_LSR_THRE) == 0)
		;

	set_reg(UART_THR_OFFSET, c);

    // volatile int loop = 0;
    // while (loop < 100000) {
    //     loop = loop + 1;
    // }

    return c;
}

int uart_getchar(ps_chardevice_t *dev)
{
    return -1;
}

int uart_init(const struct dev_defn *defn,
              const ps_io_ops_t *ops,
              ps_chardevice_t *dev)
{
    memset(dev, 0, sizeof(*dev));

    /* Map device. */
    void *vaddr = chardev_map(defn, ops);
    if (vaddr == NULL) {
        return -1;
    }

    uint16_t bdiv;

	uart8250_base      = (volatile void *)vaddr;
	uart8250_reg_shift = ARIANE_UART_REG_SHIFT;
	uart8250_reg_width = ARIANE_UART_REG_WIDTH;
	uart8250_in_freq   = ARIANE_UART_FREQ;
	uart8250_baudrate  = ARIANE_UART_BAUDRATE;

	bdiv = uart8250_in_freq / (16 * uart8250_baudrate);

    /* Disable all interrupts */
	set_reg(UART_IER_OFFSET, 0x00);
	/* Enable DLAB */
	set_reg(UART_LCR_OFFSET, 0x80);

	if (bdiv) {
		/* Set divisor low byte */
		set_reg(UART_DLL_OFFSET, bdiv & 0xff);
		/* Set divisor high byte */
		set_reg(UART_DLM_OFFSET, (bdiv >> 8) & 0xff);
	}

	/* 8 bits, no parity, two stop bit */
	set_reg(UART_LCR_OFFSET, 0x07);
	/* Enable FIFO */
	set_reg(UART_FCR_OFFSET, 0x01);
	/* No modem control DTR RTS */
	set_reg(UART_MCR_OFFSET, 0x00);
	/* Clear line status */
	get_reg(UART_LSR_OFFSET);
	/* Read receive buffer */
	get_reg(UART_RBR_OFFSET);
	/* Set scratchpad */
	set_reg(UART_SCR_OFFSET, 0x00);

    /* Set up all the device properties. */
    dev->id         = defn->id;
    dev->vaddr      = (void *)vaddr;
    dev->read       = &uart_read;
    dev->write      = &uart_write;
    dev->handle_irq = &uart_handle_irq;
    dev->irqs       = defn->irqs;
    dev->ioops      = *ops;
    dev->flags      = SERIAL_AUTO_CR;

    return 0;
}

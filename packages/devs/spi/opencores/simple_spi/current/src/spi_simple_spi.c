//=============================================================================
//
//      spi_simple_spi.c
//
//      SPI driver implementation for simple_spi
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2008, 2009 Free Software Foundation, Inc.
//
// eCos is free software; you can redistribute it and/or modify it under    
// the terms of the GNU General Public License as published by the Free     
// Software Foundation; either version 2 or (at your option) any later      
// version.                                                                 
//
// eCos is distributed in the hope that it will be useful, but WITHOUT      
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or    
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License    
// for more details.                                                        
//
// You should have received a copy of the GNU General Public License        
// along with eCos; if not, write to the Free Software Foundation, Inc.,    
// 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.            
//
// As a special exception, if other files instantiate templates or use      
// macros or inline functions from this file, or you compile this file      
// and link it with other works to produce a work based on this file,       
// this file does not by itself cause the resulting work to be covered by   
// the GNU General Public License. However the source code for this file    
// must still be made available in accordance with section (3) of the GNU   
// General Public License v2.                                               
//
// This exception does not invalidate any other reasons why a work based    
// on this file might be covered by the GNU General Public License.         
// -------------------------------------------                              
// ####ECOSGPLCOPYRIGHTEND####                                              
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):   Piotr Skrzypek
// Date:        2012-05-14
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

#include <cyg/io/spi.h>
#include <cyg/io/spi_simple_spi.h>

#include <pkgconf/devs_spi_opencores_simple_spi.h>
#include <pkgconf/hal.h>
#include <pkgconf/io_spi.h>

#define SIMPLE_SPI_BASE 0xB0000000
#define SIMPLE_SPI_SPACE 0x01000000

// Register space
#define SIMPLE_SPI_SPCR 0x00
#define SIMPLE_SPI_SPSR 0x01
#define SIMPLE_SPI_SPDR 0x02
#define SIMPLE_SPI_SPER 0x03
#define SIMPLE_SPI_SPSS 0x04

// SIMPLE_SPI_SPCR bits
#define SIMPLE_SPI_SPCR_SPIE 0x80
#define SIMPLE_SPI_SPCR_SPE  0x40
#define SIMPLE_SPI_SPCR_MSTR 0x10
#define SIMPLE_SPI_SPCR_CPOL 0x08
#define SIMPLE_SPI_SPCR_CPHA 0x04
#define SIMPLE_SPI_SPCR_SPR  0x03

// SIMPLE_SPI_SPSR bits
#define SIMPLE_SPI_SPSR_SPIF    0x80
#define SIMPLE_SPI_SPSR_WCOL    0x40
#define SIMPLE_SPI_SPSR_WFFULL  0x08
#define SIMPLE_SPI_SPSR_WFEMPTY 0x04
#define SIMPLE_SPI_SPSR_RFFULL  0x02
#define SIMPLE_SPI_SPSR_RFEMPTY 0x01

// SIMPLE_SPI_SPER bits
#define SIMPLE_SPI_SPER_ICNT(x) ((x-1) << 6)

// Divider table
cyg_uint8 simple_spi_divflags[] = {0x0, 0x1, 0x4, 0x2, 0x3, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB};

cyg_vector_t simple_spi_int_vectors[] = { CYGNUM_DEVS_SPI_OPENCORES_SIMPLE_SPI_INTS };

static cyg_uint32 simple_spi_isr(cyg_vector_t vector, cyg_addrword_t data) {

	cyg_drv_interrupt_mask(vector);
	cyg_drv_interrupt_acknowledge(vector);

	return(CYG_ISR_CALL_DSR | CYG_ISR_HANDLED);

}

static void simple_spi_dsr(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data) {

	cyg_spi_opencores_simple_spi_bus_t *bus = (cyg_spi_opencores_simple_spi_bus_t *) data;

	// Clear interrupt
	HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPSR, SIMPLE_SPI_SPSR_SPIF);
	cyg_drv_interrupt_unmask(vector);

	// Unlock waiting thread
	cyg_drv_cond_signal(&bus->condvar);

}


static void simple_spi_transaction_begin(cyg_spi_device* device) {

	cyg_spi_opencores_simple_spi_bus_t* bus = (cyg_spi_opencores_simple_spi_bus_t*) device->spi_bus;
	cyg_spi_opencores_simple_spi_device_t* dev = (cyg_spi_opencores_simple_spi_device_t*) device;

	// Temporarily disable peripheral and clean up fifos.
	HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPCR, 0);

	// Determine divider flags to meet required SCK speed. Find highest speed that is
	// lower or equal the provided threshold.
	cyg_uint32 freq = CYGNUM_DEVS_SPI_OPENCORES_SIMPLE_SPI_BUS_SPEED * 1000000;
	int i = 0;
	while((freq / (1 << (i + 1))) > (dev->freq) && (i) < (sizeof(simple_spi_divflags)-1)) {
		i++;
	}

	cyg_uint8 reg;

	// Configure extensions register with speed and interrupt granularity
	reg = SIMPLE_SPI_SPER_ICNT(1) |
	      (simple_spi_divflags[i] >> 2);
	HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPER, reg);

	// Configure control register with speed, phase and polarity
	reg = SIMPLE_SPI_SPCR_SPE | 
	      SIMPLE_SPI_SPCR_MSTR |
	      (dev->polarity ? SIMPLE_SPI_SPCR_CPOL : 0) |
	      (dev->phase ? SIMPLE_SPI_SPCR_CPHA : 0) |
	      (simple_spi_divflags[i] & SIMPLE_SPI_SPCR_SPR);
	HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPCR, reg);

}

static void simple_spi_transaction_transfer_tick(cyg_spi_device* device, 
						 cyg_bool polled, 
						 cyg_uint32 count, 
						 const cyg_uint8* tx_data, 
						 cyg_uint8* rx_data, 
						 cyg_bool drop_cs,
						 cyg_bool is_tick) {

	cyg_spi_opencores_simple_spi_bus_t* bus = (cyg_spi_opencores_simple_spi_bus_t*) device->spi_bus;
	cyg_spi_opencores_simple_spi_device_t* dev = (cyg_spi_opencores_simple_spi_device_t*) device;

	// Enable interrupts if using interrupt mode
	cyg_uint8 reg;
	if(!polled) {
		HAL_READ_UINT8(bus->base_addr + SIMPLE_SPI_SPCR, reg);
		reg |= SIMPLE_SPI_SPCR_SPIE;
		HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPCR, reg);
	}

	// Assert CS if it is not asserted yet. Note that tick command is not supposed
	// to use CS
	if(!bus->cs_asserted && !is_tick) {
		HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPSS, (1 << dev->cs));
		CYGACC_CALL_IF_DELAY_US(dev->cs_to_tran);
		bus->cs_asserted = true;
	}

	// Transfer the data
	int i;
	for(i = 0; i < count; i++) {

		// If multiple bytes are transfered, then wait tran_to_tran
		if(i > 0) {
			CYGACC_CALL_IF_DELAY_US(dev->tran_to_tran);
		}

		// Send data or NULL (tick uses NULL)
		reg = tx_data ? tx_data[i] : 0;
		HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPDR, reg);

		// Wait for transmission
		if(polled) {
			do {
				HAL_READ_UINT8(bus->base_addr + SIMPLE_SPI_SPSR, reg);
			} while(!(reg & SIMPLE_SPI_SPSR_SPIF));
			HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPSR, SIMPLE_SPI_SPSR_SPIF);
		}
		else {
			//FIXME at high SCK speeds the interrupt happens so quickly that
			//this thread can't make to cond_wait. Signal from DSR is posted 
			//to nowhere and the thread hangs.
			cyg_drv_mutex_lock(&bus->mutex);
			cyg_drv_cond_wait(&bus->condvar);
			cyg_drv_mutex_unlock(&bus->mutex);
		}

		// Read received byte and store if required
		HAL_READ_UINT8(bus->base_addr + SIMPLE_SPI_SPDR, reg);
		if(rx_data) {
			rx_data[i] = reg;
		}
	}

	// Drop CS if required
	if(drop_cs && !is_tick) {
		bus->cs_asserted = false;
		CYGACC_CALL_IF_DELAY_US(dev->tran_to_cs);
		HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPSS, 0);
	}

	// Disable interrupts if using interrupt mode
	if(!polled) {
		HAL_READ_UINT8(bus->base_addr + SIMPLE_SPI_SPCR, reg);
		reg &= ~SIMPLE_SPI_SPCR_SPIE;
		HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPCR, reg);
	}

}

static void simple_spi_transaction_transfer(cyg_spi_device* device, 
				       cyg_bool polled, 
				       cyg_uint32 count, 
				       const cyg_uint8* tx_data, 
				       cyg_uint8* rx_data, 
				       cyg_bool drop_cs) {

	simple_spi_transaction_transfer_tick(device, polled, count, tx_data, rx_data, drop_cs, false);

}

static void simple_spi_transaction_tick(cyg_spi_device* device, 
				   cyg_bool polled, 
				   cyg_uint32 count) {

	simple_spi_transaction_transfer_tick(device, polled, count, NULL, NULL, true, true);

}

static void simple_spi_transaction_end(cyg_spi_device* device) {

	cyg_spi_opencores_simple_spi_bus_t* bus = (cyg_spi_opencores_simple_spi_bus_t*) device->spi_bus;

	// Disable the peripheral and clean up buffers.
	HAL_WRITE_UINT8(bus->base_addr + SIMPLE_SPI_SPCR, 0);

}

static int simple_spi_get_config(cyg_spi_device* device, 
			    cyg_uint32 key, 
			    void* buf, 
			    cyg_uint32* len) {

	cyg_spi_opencores_simple_spi_device_t* dev = (cyg_spi_opencores_simple_spi_device_t*) device;

	if(key == CYG_IO_GET_CONFIG_SPI_CLOCKRATE) {
		cyg_uint32 *freq = (cyg_uint32*) buf;
		*freq = dev->freq;
		return 0;
	}
	else {
		return -1;
	}
}

static int simple_spi_set_config(cyg_spi_device* device, 
			    cyg_uint32 key, 
			    const void* buf, 
			    cyg_uint32* len) {

	cyg_spi_opencores_simple_spi_device_t* dev = (cyg_spi_opencores_simple_spi_device_t*) device;

	if(key == CYG_IO_SET_CONFIG_SPI_CLOCKRATE) {
		dev->freq = *((cyg_uint32*)buf);
		return 0;
	}
	else {
		return -1;
	}

}

// Declare number of SPI buses
cyg_spi_opencores_simple_spi_bus_t cyg_spi_simple_spi_bus[CYGNUM_DEVS_SPI_OPENCORES_SIMPLE_SPI_COUNT];

// This is a low level constructor of the driver. It is called very early
static void CYGBLD_ATTRIB_C_INIT_PRI(CYG_INIT_BUS_SPI) simple_spi_init(void) {

	int i;
	cyg_uint32 base = SIMPLE_SPI_BASE;
	for(i = 0; i < CYGNUM_DEVS_SPI_OPENCORES_SIMPLE_SPI_COUNT; i++) {

		// Connect functions
		cyg_spi_simple_spi_bus[i].spi_bus.spi_transaction_begin = simple_spi_transaction_begin;
		cyg_spi_simple_spi_bus[i].spi_bus.spi_transaction_transfer = simple_spi_transaction_transfer;
		cyg_spi_simple_spi_bus[i].spi_bus.spi_transaction_tick = simple_spi_transaction_tick;
		cyg_spi_simple_spi_bus[i].spi_bus.spi_transaction_end = simple_spi_transaction_end;
		cyg_spi_simple_spi_bus[i].spi_bus.spi_get_config = simple_spi_get_config;
		cyg_spi_simple_spi_bus[i].spi_bus.spi_set_config = simple_spi_set_config;

		// Fill the base address
		cyg_spi_simple_spi_bus[i].base_addr = base;
		base += SIMPLE_SPI_SPACE;

		// Initialize interrupts
		cyg_drv_interrupt_create(simple_spi_int_vectors[i],
					 CYGNUM_DEVS_SPI_OPENCORES_SIMPLE_SPI_INT_PRI,
					 (cyg_addrword_t) &cyg_spi_simple_spi_bus[i],
					 simple_spi_isr,
					 simple_spi_dsr,
					 &cyg_spi_simple_spi_bus[i].int_handle,
					 &cyg_spi_simple_spi_bus[i].int_data);
		cyg_drv_interrupt_attach(cyg_spi_simple_spi_bus[i].int_handle);
		cyg_drv_interrupt_unmask(simple_spi_int_vectors[i]);

		// Initialize synchronization
		cyg_drv_mutex_init(&cyg_spi_simple_spi_bus[i].mutex);
		cyg_drv_cond_init(&cyg_spi_simple_spi_bus[i].condvar, 
				  &cyg_spi_simple_spi_bus[i].mutex);

		// Private status
		cyg_spi_simple_spi_bus[i].cs_asserted = false;

		// Initialize upper layer
		CYG_SPI_BUS_COMMON_INIT(&cyg_spi_simple_spi_bus[i].spi_bus);
	}
}



#ifndef CYGONCE_DEVS_SPI_OPENCORES_SIMPLE_SPI_H
#define CYGONCE_DEVS_SPI_OPENCORES_SIMPLE_SPI_H
//=============================================================================
//
//      spi_simple_spi.h
//
//      Header definitions for simple_spi driver.
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

#include <pkgconf/hal.h>
#include <pkgconf/io_spi.h>
#include <pkgconf/devs_spi_opencores_simple_spi.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>
#include <cyg/io/spi.h>

// Use this macro in your code to declare SPI devices.
//
// _name_ name of the handler variable that must be passed on each call 
//        to eCos SPI API
//
// _bus_ SPI bus number to which the device is attached;
//       numbers start from zero
//
// _cs_ number of the chip select line on the bus;
//      numbers start from zero
//
// _polarity_ SPI polarity
//
// _phase_ SPI phase
//
// _freq_ SCK frequency in Hertz
//
// _cs_to_tran_ minimum delay between chip select and transfer, in us
//
// _tran_to_cs_ minimum delay between transfer and chip deselect, in us
//
// _tran_to_tran_ minimum delay between transfers, in us

#define CYG_DEVS_SPI_OPENCORES_SIMPLE_SPI_DEVICE(_name_,         \
						 _bus_,          \
						 _cs_,           \
						 _polarity_,     \
						 _phase_,        \
						 _freq_,         \
						 _cs_to_tran_,   \
						 _tran_to_cs_,   \
						 _tran_to_tran_) \
                                                                 \
	cyg_spi_opencores_simple_spi_device_t _name_ ##_simple_spi CYG_SPI_DEVICE_ON_BUS(_bus_) = { \
	.spi_device = {                                                                             \
		.spi_bus = (cyg_spi_bus*) &cyg_spi_simple_spi_bus[_bus_]                            \
	},                                                                                          \
	.cs = _cs_,                                                                                 \
	.polarity = _polarity_,                                                                     \
	.phase = _phase_,                                                                           \
	.freq = _freq_,                                                                             \
	.cs_to_tran = _cs_to_tran_,                                                                 \
	.tran_to_cs = _tran_to_cs_,                                                                 \
	.tran_to_tran = _tran_to_tran_,                                                             \
	};                                                                                          \
	extern cyg_spi_device _name_ __attribute__((alias ( #_name_ "_simple_spi" )));

typedef struct {
	// Upper layer
	cyg_spi_device spi_device;

	// Private data
	cyg_uint8 cs;
	cyg_uint8 polarity;
	cyg_uint8 phase;
	cyg_uint32 freq;
	cyg_uint16 cs_to_tran;
	cyg_uint16 tran_to_cs;
	cyg_uint16 tran_to_tran;

} cyg_spi_opencores_simple_spi_device_t;

typedef struct {
	// Upper layer callbacks
	cyg_spi_bus spi_bus;

	// Private data
	cyg_uint32 base_addr;

	cyg_interrupt int_data;
	cyg_handle_t int_handle;

	cyg_drv_mutex_t mutex;
	cyg_drv_cond_t condvar;

	// State data
	cyg_bool cs_asserted;

} cyg_spi_opencores_simple_spi_bus_t;

externC cyg_spi_opencores_simple_spi_bus_t cyg_spi_simple_spi_bus[];

//=============================================================================
#endif // CYGONCE_DEVS_SPI_OPENCORES_SIMPLE_SPI_H

//==========================================================================
//
//      if_sdcmsc.c
//
//      Provide a disk device driver for SDCard Mass Storage Controller
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2004, 2006 Free Software Foundation, Inc.                  
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
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author:       Piotr Skrzypek
// Date:         2012-05-01
//
//####DESCRIPTIONEND####
//==========================================================================

#include <pkgconf/system.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <string.h>
#include <errno.h>
#include <cyg/io/io.h>
#include <cyg/io/devtab.h>
#include <cyg/io/disk.h>

// Settings exported from CDL
#include <pkgconf/devs_disk_opencores_sdcmsc.h>

// SDCMSC address space
#define SDCMSC_BASE 0x9e000000

// Register space
#define SDCMSC_ARGUMENT           0x00
#define SDCMSC_COMMAND            0x04
#define SDCMSC_CARD_STATUS        0x08
#define SDCMSC_RESPONSE           0x0C
#define SDCMSC_CONTROLLER_SETTING 0x1C
#define SDCMSC_BLOCK_SIZE         0x20
#define SDCMSC_POWER_CONTROL      0x24
#define SDCMSC_SOFTWARE_RESET     0x28
#define SDCMSC_TIMEOUT            0x2C
#define SDCMSC_NORMAL_INT_STATUS  0x30
#define SDCMSC_ERROR_INT_STATUS   0x34
#define SDCMSC_NORMAL_INT_ENABLE  0x38
#define SDCMSC_ERROR_INT_ENABLE   0x3C
#define SDCMSC_CAPABILITY         0x48
#define SDCMSC_CLOCK_DIVIDER      0x4C
#define SDCMSC_BD_BUFFER_STATUS   0x50
#define SDCMSC_DAT_INT_STATUS     0x54
#define SDCMSC_DAT_INT_ENABLE     0x58
#define SDCMSC_BD_RX              0x60
#define SDCMSC_BD_TX              0x80

// SDCMSC_COMMAND bits
#define SDCMSC_COMMAND_CMDI(x) (x << 8)
#define SDCMSC_COMMAND_CMDW(x) (x << 6)
#define SDCMSC_COMMAND_CICE    0x10
#define SDCMSC_COMMAND_CIRC    0x08
#define SDCMSC_COMMAND_RTS_48  0x02
#define SDCMSC_COMMAND_RTS_136 0x01

//SDCMSC_CARD_STATUS bits
#define SDCMSC_CARD_STATUS_CICMD 0x01

// SDCMSC_NORMAL_INT_STATUS bits
#define SDCMSC_NORMAL_INT_STATUS_EI 0x8000
#define SDCMSC_NORMAL_INT_STATUS_CC 0x0001

// SDCMSC_DAT_INT_STATUS
#define SDCMSC_DAT_INT_STATUS_TRS 0x01

typedef struct cyg_sdcmsc_disk_info_t {
	int is_v20;
	int is_sdhc;
	cyg_uint32 rca;
	int connected;
} cyg_sdcmsc_disk_info_t;

static int sdcmsc_card_cmd(cyg_uint32 cmd, 
			   cyg_uint32 arg, 
			   cyg_uint32 *response) {

	// Send command to card
	HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_COMMAND, cmd);
	HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_ARGUMENT, arg);

	// Wait for response
	cyg_uint32 reg;
	cyg_uint32 mask = SDCMSC_NORMAL_INT_STATUS_EI | 
			  SDCMSC_NORMAL_INT_STATUS_CC;

	do {
		HAL_READ_UINT32(SDCMSC_BASE + SDCMSC_NORMAL_INT_STATUS, reg);
	} while(!(reg & mask));
	HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_NORMAL_INT_STATUS, 0);

	// Optionally read response register
	if(response) {
		HAL_READ_UINT32(SDCMSC_BASE + SDCMSC_RESPONSE, *response);
	}

	// Check for errors
	if(reg & SDCMSC_NORMAL_INT_STATUS_EI) {
		HAL_READ_UINT32(SDCMSC_BASE + SDCMSC_ERROR_INT_STATUS, reg);
		if(reg & (1 << 3)) diag_printf("Command index error\n");
		if(reg & (1 << 1)) diag_printf("Command CRC error\n");
		if(reg & (1 << 0)) diag_printf("Command timeout\n");
		HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_ERROR_INT_STATUS, 0);
		return 0;
	}
	else {
		return 1;
	}
}

// Card initialization and identification implemented according to
// Physical Layer Simplified Specification Version 3.01
static int sdcmsc_card_init(cyg_sdcmsc_disk_info_t *data,
			    char *serial,
			    char *firmware_rev,
			    char *model_num,
			    cyg_uint32 *capacity) {

	cyg_uint32 reg;
	cyg_uint32 cmd;
	cyg_uint32 arg;

	// Send CMD0 to switch the card to idle state
	cmd = SDCMSC_COMMAND_CMDI(0);
	if(!sdcmsc_card_cmd(cmd, 0, NULL)) return 0;

	// Send CMD8 offering 2.7V to 3.6V range
	// If the card doesn't responde it means either:
	// 1. Card supports v2.0 but can't communicate using
	//    current voltage levels
	// 2. Card does not support v2.0
	cmd = SDCMSC_COMMAND_CMDI(8) | 
	      SDCMSC_COMMAND_CICE |
	      SDCMSC_COMMAND_CIRC |
	      SDCMSC_COMMAND_RTS_48;
	data->is_v20 = sdcmsc_card_cmd(cmd, 0x1AA, NULL);

	do {
		HAL_READ_UINT32(SDCMSC_BASE + SDCMSC_CARD_STATUS, reg);
	} while(reg & SDCMSC_CARD_STATUS_CICMD);

	// Repeat ACMD41 until card set the busy bit to 1
	// Since ACMD is an extended command, it must be preceded
	// by CMD55
	do {
		cmd = SDCMSC_COMMAND_CMDI(55) | 
		      SDCMSC_COMMAND_CICE |
		      SDCMSC_COMMAND_CIRC |
		      SDCMSC_COMMAND_RTS_48;
		if(!sdcmsc_card_cmd(cmd, 0, NULL)) return 0;

		cmd = SDCMSC_COMMAND_CMDI(41) |
		      SDCMSC_COMMAND_RTS_48;
		arg = data->is_v20 ? 
		      0x40FF8000 : 
		      0x00FF8000;
		if(!sdcmsc_card_cmd(cmd, arg, &reg)) return 0;

	} while(!(reg & 0x80000000));

	data->is_sdhc = !!(reg & 0x40000000);

	// Issue CMD2 to switch from ready state to ident. Unfortunately, it is
	// not possible to read whole CID because the command can be issued only
	// once, and the peripheral can store only 32bit of the command at once.
	cmd = SDCMSC_COMMAND_CMDI(2) |
	      SDCMSC_COMMAND_RTS_136;
	if(!sdcmsc_card_cmd(cmd, 0, NULL)) return 0;

	// Issue CMD3 to get RCA and switch from ident state to stby.
	cmd = SDCMSC_COMMAND_CMDI(3) |
	      SDCMSC_COMMAND_CICE |
	      SDCMSC_COMMAND_CIRC |
	      SDCMSC_COMMAND_RTS_48;
	if(!sdcmsc_card_cmd(cmd, 0, &reg)) return 0;
	data->rca = reg & 0xFFFF0000;

	// Calculate card capacity. Use information stored in CSD register.
	cyg_uint32 card_capacity;
	if(data->is_sdhc) {
		cmd = SDCMSC_COMMAND_CMDI(9) |
		      SDCMSC_COMMAND_CMDW(1) |
		      SDCMSC_COMMAND_RTS_136;
		if(!sdcmsc_card_cmd(cmd, data->rca, &reg)) return 0;
		card_capacity = reg & 0x3F;
		card_capacity <<= 16;

		cmd = SDCMSC_COMMAND_CMDI(9) |
		      SDCMSC_COMMAND_CMDW(2) |
		      SDCMSC_COMMAND_RTS_136;
		if(!sdcmsc_card_cmd(cmd, data->rca, &reg)) return 0;
		reg >>= 16;
		card_capacity |= reg;
		card_capacity += 1;
		card_capacity *= 1000;
	}
	else {
		cmd = SDCMSC_COMMAND_CMDI(9) |
		      SDCMSC_COMMAND_CMDW(1) |
		      SDCMSC_COMMAND_RTS_136;
		if(!sdcmsc_card_cmd(cmd, data->rca, &reg)) return 0;
		cyg_uint32 read_bl_len = (reg >> 16) & 0x0F;
		cyg_uint32 c_size = reg & 0x3FF;
		c_size <<= 2;

		cmd = SDCMSC_COMMAND_CMDI(9) |
		      SDCMSC_COMMAND_CMDW(2) |
		      SDCMSC_COMMAND_RTS_136;
		if(!sdcmsc_card_cmd(cmd, data->rca, &reg)) return 0;
		c_size |= (reg >> 30) & 0x03;
		cyg_uint32 c_size_mult = (reg >> 15) & 0x07;
		card_capacity = c_size + 1;
		card_capacity *= 1 << (c_size_mult + 2);
		card_capacity *= 1 << (read_bl_len);
		card_capacity >>= 9;
	}

	// Fill disk identification struct using information in CID register
	// use OEM/APPlication ID field to fill model_num,
	// Product revision field to fill firmware_rev,
	// and Product serial number to field to fill serial
	cmd = SDCMSC_COMMAND_CMDI(10) |
	      SDCMSC_COMMAND_CMDW(0) |
	      SDCMSC_COMMAND_RTS_136;
	if(!sdcmsc_card_cmd(cmd, data->rca, &reg)) return 0;
	model_num[0] = (reg >> 16) & 0xFF;
	model_num[1] = (reg >> 8) & 0xFF;
	model_num[2] = 0;

	cmd = SDCMSC_COMMAND_CMDI(10) |
	      SDCMSC_COMMAND_CMDW(2) |
	      SDCMSC_COMMAND_RTS_136;
	if(!sdcmsc_card_cmd(cmd, data->rca, &reg)) return 0;
	firmware_rev[0] = (reg >> 24) & 0xFF;
	firmware_rev[1] = 0;
	serial[0] = (reg >> 16) & 0xFF;
	serial[1] = (reg >> 8) & 0xFF;
	serial[2] = reg & 0xFF;

	cmd = SDCMSC_COMMAND_CMDI(10) |
	      SDCMSC_COMMAND_CMDW(3) |
	      SDCMSC_COMMAND_RTS_136;
	if(!sdcmsc_card_cmd(cmd, data->rca, &reg)) return 0;
	serial[3] = (reg >> 24) & 0xFF;

	// Put card in transfer state 
	cmd = SDCMSC_COMMAND_CMDI(7) |
	      SDCMSC_COMMAND_CICE |
	      SDCMSC_COMMAND_CIRC |
	      SDCMSC_COMMAND_RTS_48;
	if(!sdcmsc_card_cmd(cmd, data->rca, &reg)) return 0;
	if(reg != 0x700) return 0;

	// Set block size to 512
	cmd = SDCMSC_COMMAND_CMDI(16) |
	      SDCMSC_COMMAND_CICE |
	      SDCMSC_COMMAND_CIRC |
	      SDCMSC_COMMAND_RTS_48;
	if(!sdcmsc_card_cmd(cmd, 512, NULL)) return 0;

	// Set 4-bits bus mode
	cmd = SDCMSC_COMMAND_CMDI(55) |
	      SDCMSC_COMMAND_CICE |
	      SDCMSC_COMMAND_CIRC |
	      SDCMSC_COMMAND_RTS_48;
	if(!sdcmsc_card_cmd(cmd, data->rca, NULL)) return 0;

	cmd = SDCMSC_COMMAND_CMDI(6) |
	      SDCMSC_COMMAND_CICE |
	      SDCMSC_COMMAND_CIRC |
	      SDCMSC_COMMAND_RTS_48;
	if(!sdcmsc_card_cmd(cmd, 0x02, NULL)) return 0;

	return 1;
}

static int sdcmsc_card_queue(cyg_sdcmsc_disk_info_t *data, 
			int direction_transmit,
			int block_addr,
			cyg_uint32 buffer_addr) {

        // SDSC cards use byte addressing, while SDHC use block addressing.
        // It is therefore required to multiply the address by 512 if
        // we are dealing with SDSC card, to remain compatible with the API.
	if(!data->is_sdhc) {
		block_addr <<= 9;
	}

	if(direction_transmit) {
		HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_BD_TX, buffer_addr);
		HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_BD_TX, block_addr);
	}
	else {
		HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_BD_RX, buffer_addr);
		HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_BD_RX, block_addr);
	}

	// Now wait for the response
	cyg_uint32 reg;
	do {
		HAL_READ_UINT32(SDCMSC_BASE + SDCMSC_DAT_INT_STATUS, reg);
	} while(!reg);
	HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_DAT_INT_STATUS, 0);

	// Check for errors
	if(reg == SDCMSC_DAT_INT_STATUS_TRS) {
		return 1;
	}
	else {
		if(reg & (1 << 5)) diag_printf("Transmission error\n");
		if(reg & (1 << 4)) diag_printf("Command error\n");
		if(reg & (1 << 2)) diag_printf("FIFO error\n");
		if(reg & (1 << 1)) diag_printf("Retry error\n");
		return 0;
	}
}

// This is an API function. Is is called once, in the beginning
static cyg_bool sdcmsc_disk_init(struct cyg_devtab_entry* tab) {

	// Set highest possible timeout
	HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_TIMEOUT, 0xFFFE);

	// Reset the peripheral
	HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_SOFTWARE_RESET, 1);
	HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_CLOCK_DIVIDER, 2);
	HAL_WRITE_UINT32(SDCMSC_BASE + SDCMSC_SOFTWARE_RESET, 0);

	// Call upper level
	disk_channel* ch = (disk_channel*) tab->priv;
	return (*ch->callbacks->disk_init)(tab);
}

// This function is called when user mounts the disk
static Cyg_ErrNo sdcmsc_disk_lookup(struct cyg_devtab_entry** tab, 
				    struct cyg_devtab_entry *sub_tab, 
				    const char* name) {

	disk_channel *ch = (disk_channel*) (*tab)->priv;
	cyg_sdcmsc_disk_info_t *data = (cyg_sdcmsc_disk_info_t*) ch->dev_priv;

	// If the card was not initialized yet, it's time to do it
	// and call disk_connected callback
	if(!data->connected) {

		cyg_disk_identify_t id;

		// Pass dummy CHS geometry and hope the upper level 
		// will use LBA mode. To guess CHS we would need to
		// analyze partition table and confront LBA and CHS
		// addresses. And it would work only if proper LBA
		// field is stored in MBR. Is is definitely something
		// that should be done by upper level.
		id.cylinders_num = 1;
		id.heads_num = 1;
		id.sectors_num = 1;

		id.phys_block_size = 1;
		id.max_transfer = 512;

		// Initialize the card
		data->connected = sdcmsc_card_init(data,
						   id.serial,
						   id.firmware_rev,
						   id.model_num,
						   &id.lba_sectors_num);

		if(data->connected) {
			// Let upper level know there is a new disk
			(*ch->callbacks->disk_connected)(*tab, &id);
		}
	}

	// Call upper level
	return (*ch->callbacks->disk_lookup)(tab, sub_tab, name);
}

// API function to read block from the disk
static Cyg_ErrNo sdcmsc_disk_read(disk_channel* ch, 
				  void* buf, 
				  cyg_uint32 blocks, 
				  cyg_uint32 first_block) {

	cyg_sdcmsc_disk_info_t *data = (cyg_sdcmsc_disk_info_t*) ch->dev_priv;

	int i;
	int result;
	cyg_uint32 reg;
	for(i = 0; i < blocks; i++) {

		// Check for free receive buffers
		HAL_READ_UINT32(SDCMSC_BASE + SDCMSC_BD_BUFFER_STATUS, reg);
		reg >>= 8;
		reg &= 0xFF;
		if(reg == 0) {
			return -EIO;
		}

		result = sdcmsc_card_queue(data, 0, first_block, (cyg_uint32) buf);
		if(!result) {
			return -EIO;
		}
	}

	return ENOERR;

}

// API function to write block to disk
static Cyg_ErrNo sdcmsc_disk_write(disk_channel* ch, 
				   const void* buf, 
				   cyg_uint32 blocks, 
				   cyg_uint32 first_block) {

	cyg_sdcmsc_disk_info_t *data = (cyg_sdcmsc_disk_info_t*) ch->dev_priv;

	int i;
	int result;
	cyg_uint32 reg;
	for(i = 0; i < blocks; i++) {

		// Check for free transmit buffers
		HAL_READ_UINT32(SDCMSC_BASE + SDCMSC_BD_BUFFER_STATUS, reg);
		reg &= 0xFF;
		if(reg == 0) {
			return -EIO;
		}

		result = sdcmsc_card_queue(data, 1, first_block, (cyg_uint32) buf);
		if(!result) {
			return -EIO;
		}
	}

	return ENOERR;

}

// API function to fetch driver configuration and disk info.
static Cyg_ErrNo sdcmsc_disk_get_config(disk_channel* ch, 
					cyg_uint32 key, 
					const void* buf, 
					cyg_uint32* len) {

	CYG_UNUSED_PARAM(disk_channel*, ch);
	CYG_UNUSED_PARAM(cyg_uint32, key);
	CYG_UNUSED_PARAM(const void*, buf);
	CYG_UNUSED_PARAM(cyg_uint32*, len);

	return -EINVAL;
}

// API function to update driver status information.
static Cyg_ErrNo sdcmsc_disk_set_config(disk_channel* ch, 
					cyg_uint32 key, 
					const void* buf, 
					cyg_uint32* len) {

	cyg_sdcmsc_disk_info_t *data = (cyg_sdcmsc_disk_info_t*) ch->dev_priv;

	if(key == CYG_IO_SET_CONFIG_DISK_UMOUNT) {
	        if(ch->info->mounts == 0) {
			data->connected = false;
			return (ch->callbacks->disk_disconnected)(ch);
	        }
		else {
			return ENOERR;
		}
	}
	else {
		return -EINVAL;
	}

}

// Register the driver in the system

static cyg_sdcmsc_disk_info_t cyg_sdcmsc_disk0_hwinfo = {
	.connected = 0
};

DISK_FUNS(cyg_sdcmsc_disk_funs,
	  sdcmsc_disk_read,
	  sdcmsc_disk_write,
	  sdcmsc_disk_get_config,
	  sdcmsc_disk_set_config
);


DISK_CONTROLLER(cyg_sdcmsc_disk_controller_0, cyg_sdcmsc_disk0_hwinfo);

DISK_CHANNEL(cyg_sdcmsc_disk0_channel,
             cyg_sdcmsc_disk_funs,
             cyg_sdcmsc_disk0_hwinfo,
             cyg_sdcmsc_disk_controller_0,
             true, //mbr supported
             4 //partitions
);

BLOCK_DEVTAB_ENTRY(cyg_sdcmsc_disk0_devtab_entry,
		   CYGDAT_DEVS_DISK_OPENCORES_SDCMSC_DISK0_NAME,
		   0,
		   &cyg_io_disk_devio,
		   &sdcmsc_disk_init,
		   &sdcmsc_disk_lookup,
		   &cyg_sdcmsc_disk0_channel);

// EOF if_sdcmsc.c

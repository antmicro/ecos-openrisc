//==========================================================================
//
//      if_ethmac.c
//
//	OpenCores ETHMAC controller driver
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2003, 2004 Free Software Foundation, Inc.                  
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
//#####DESCRIPTIONBEGIN####
//
// Author(s):    Piotr Skrzypek (pskrzypek@antmicro.com)
// Date:         2012-04-10
// Purpose:      
// Description:  
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/drv_api.h>
#include <cyg/io/eth/netdev.h>
#include <cyg/io/eth/eth_drv.h>

// Settings exported from CDL
#include <pkgconf/devs_eth_opencores_ethmac.h>

// ETHMAC address space
#define ETHMAC_BASE 0x92000000
#define ETHMAC_REG_BASE   (ETHMAC_BASE)
#define ETHMAC_BD_BASE    (ETHMAC_BASE + 0x400)
#define ETHMAC_TX_BD_BASE (ETHMAC_BD_BASE)
#define ETHMAC_RX_BD_BASE (ETHMAC_BD_BASE + 0x200)

// Register space
#define ETHMAC_MODER         0x000
#define ETHMAC_INT_SOURCE    0x004
#define ETHMAC_INT_MASK      0x008
#define ETHMAC_IPGT          0x00C
#define ETHMAC_IPGR1         0x010
#define ETHMAC_IPGR2         0x014
#define ETHMAC_PACKETLEN     0x018
#define ETHMAC_COLLCONF      0x01C
#define ETHMAC_TX_BD_NUM     0x020
#define ETHMAC_CTRLMODER     0x024
#define ETHMAC_MIIMODER      0x028
#define ETHMAC_MIICOMMAND    0x02C
#define ETHMAC_MIIADDRESS    0x030
#define ETHMAC_MIITX_DATA    0x034
#define ETHMAC_MIIRX_DATA    0x038
#define ETHMAC_MIISTATUS     0x03C
#define ETHMAC_MAC_ADDR0     0x040
#define ETHMAC_MAC_ADDR1     0x044
#define ETHMAC_ETH_HASH0_ADR 0x048
#define ETHMAC_ETH_HASH1_ADR 0x04C
#define ETHMAC_ETH_TXCTRL    0x050

// MODER bits
#define ETHMAC_MODER_RECSMALL 0x00010000
#define ETHMAC_MODER_PAD      0x00008000
#define ETHMAC_MODER_HUGEN    0x00004000
#define ETHMAC_MODER_CRCEN    0x00002000
#define ETHMAC_MODER_DLYCRCEN 0x00001000
#define ETHMAC_MODER_FULLD    0x00000400
#define ETHMAC_MODER_EXDFREN  0x00000200
#define ETHMAC_MODER_NOBCKOF  0x00000100
#define ETHMAC_MODER_LOOPBCK  0x00000080
#define ETHMAC_MODER_IFG      0x00000040
#define ETHMAC_MODER_PRO      0x00000020
#define ETHMAC_MODER_IAM      0x00000010
#define ETHMAC_MODER_BRO      0x00000008
#define ETHMAC_MODER_NOPRE    0x00000004
#define ETHMAC_MODER_TXEN     0x00000002
#define ETHMAC_MODER_RXEN     0x00000001

// INT_SOURCE bits
#define ETHMAC_INT_SOURCE_RXC  0x00000040
#define ETHMAC_INT_SOURCE_TXC  0x00000020
#define ETHMAC_INT_SOURCE_BUSY 0x00000010
#define ETHMAC_INT_SOURCE_RXE  0x00000008
#define ETHMAC_INT_SOURCE_RXB  0x00000004
#define ETHMAC_INT_SOURCE_TXE  0x00000002
#define ETHMAC_INT_SOURCE_TXB  0x00000001

// INT_MASK bits
#define ETHMAC_INT_MASK_RXC_M  0x00000040
#define ETHMAC_INT_MASK_TXC_M  0x00000020
#define ETHMAC_INT_MASK_BUSY_M 0x00000010
#define ETHMAC_INT_MASK_RXE_M  0x00000008
#define ETHMAC_INT_MASK_RXF_M  0x00000004
#define ETHMAC_INT_MASK_TXE_M  0x00000002
#define ETHMAC_INT_MASK_TXB_M  0x00000001

// CTRLMODER bits
#define ETHMAC_CTRLMODER_TXFLOW  0x00000004
#define ETHMAC_CTRLMODER_RXFLOW  0x00000002
#define ETHMAC_CTRLMODER_PASSALL 0x00000001

// MIIMODER bits
#define ETHMAC_MIIMODER_MIINOPRE 0x00000100

// MIICOMMAND bits
#define ETHMAC_MIICOMMAND_WCTRLDATA 0x00000004
#define ETHMAC_MIICOMMAND_RSTAT     0x00000002
#define ETHMAC_MIICOMMAND_SCANSTAT  0x00000001

// MIISTATUS bits
#define ETHMAC_MIISTATUS_NVALID   0x00000004
#define ETHMAC_MIISTATUS_BUSY     0x00000002
#define ETHMAC_MIISTATUS_LINKFAIL 0x00000001

// TXCTRL bits
#define ETHMAC_TXCTRL_TXPAUSERQ 0x00010000

// TX BD bits
#define ETHMAC_TX_BD_RD   0x8000
#define ETHMAC_TX_BD_IRQ  0x4000
#define ETHMAC_TX_BD_WR   0x2000
#define ETHMAC_TX_BD_PAD  0x1000
#define ETHMAC_TX_BD_CRC  0x0800
#define ETHMAC_TX_BD_UR   0x0100
#define ETHMAC_TX_BD_RTRY 0x00F0
#define ETHMAC_TX_BD_RL   0x0008
#define ETHMAC_TX_BD_LC   0x0004
#define ETHMAC_TX_BD_DF   0x0002
#define ETHMAC_TX_BD_CS   0x0001

// RX BD bits
#define ETHMAC_RX_BD_E   0x8000
#define ETHMAC_RX_BD_IRQ 0x4000
#define ETHMAC_RX_BD_WR  0x2000
#define ETHMAC_RX_BD_CF  0x0100
#define ETHMAC_RX_BD_M   0x0080
#define ETHMAC_RX_BD_OR  0x0040
#define ETHMAC_RX_BD_IS  0x0020
#define ETHMAC_RX_BD_DN  0x0010
#define ETHMAC_RX_BD_TL  0x0008
#define ETHMAC_RX_BD_SF  0x0004
#define ETHMAC_RX_BD_CRC 0x0002
#define ETHMAC_RX_BD_LC  0x0001

typedef struct {

	// Buffer to store received frames
	char rx_buffer[CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_RXBUF_COUNT]
                      [CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_PACKETLEN];

	// Number of rx buffer descriptor that ETHMAC is pointing to
	int rx_head;

	// Array of keys (handlers given by the stack) 
	unsigned long tx_key[CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_TXBUF_COUNT];

	// Buffer to store frames for transmission
	char tx_buffer[CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_TXBUF_COUNT]
                      [CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_PACKETLEN];

	// Number of tx buffer descriptor that ETHMAC is pointing to
	int tx_head;

	// Number of descriptors waiting to be transmitted by ETHMAC
	int tx_count;

	// Interrupt resources
	cyg_handle_t int_handle;
	cyg_interrupt int_object;

} ethmac_eth_t;

ethmac_eth_t eth0_data;

ETH_DRV_SC(ethmac_eth0_sc,
           (void*) &eth0_data,
           "eth0",
           ethmac_eth_start,
           ethmac_eth_stop,
           ethmac_eth_control,
           ethmac_eth_can_send,
           ethmac_eth_send,
           ethmac_eth_recv,
           ethmac_eth_deliver,
           ethmac_eth_poll,
           ethmac_eth_int_vector);

NETDEVTAB_ENTRY(ethmac_netdev,
                "ethmac_eth0",
                ethmac_eth_init,
                &ethmac_eth0_sc);

static void ethmac_eth_debug(char *msg) {
	//diag_printf("\033[31;1m%s\033[0m\n", msg);
}

static int ethmac_eth_isr(cyg_vector_t vector, cyg_addrword_t data) {

	// Mask the interrupt in PIC, and call generic network DSR.
	// This DSR will call ethmac_eth_deliver. The interrupt will be
	// unmasked there.
	cyg_drv_interrupt_mask(CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_INT);
	cyg_drv_interrupt_acknowledge(CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_INT);

	return(CYG_ISR_HANDLED | CYG_ISR_CALL_DSR);
}

static bool ethmac_eth_init(struct cyg_netdevtab_entry *tab) {

	struct eth_drv_sc *sc = tab->device_instance;
	ethmac_eth_t *data = sc->driver_private;

	// ETHMAC uses buffer descriptors that store a pointer to
	// RAM memory. The transmit / receive queue is made from
	// multiple buffer descriptors, serviced in a round robin
	// manner. Following is the initialization of those descriptors.
	int i;
	cyg_uint32 reg;

	// TX buffer descriptors
	reg = ETHMAC_TX_BD_IRQ | ETHMAC_TX_BD_PAD | ETHMAC_TX_BD_CRC;
	for(i = 0; i < CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_TXBUF_COUNT; i++) {
		HAL_WRITE_UINT32(ETHMAC_TX_BD_BASE + (i * 8), reg);
		HAL_WRITE_UINT32(ETHMAC_TX_BD_BASE + (i * 8) + 4, 
		                 (cyg_uint32)&data->tx_buffer[i][0]);
	}
	reg |= ETHMAC_TX_BD_WR;
	i--;
	HAL_WRITE_UINT32(ETHMAC_TX_BD_BASE + (i * 8), reg);
	
	data->tx_head = 0;
	data->tx_count = 0;

	// RX buffer descriptors
	reg = CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_PACKETLEN << 16 | 
	      ETHMAC_RX_BD_E | ETHMAC_RX_BD_IRQ;
	for(i = 0; i < CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_RXBUF_COUNT; i++) {
		HAL_WRITE_UINT32(ETHMAC_RX_BD_BASE + (i * 8), reg);
		HAL_WRITE_UINT32(ETHMAC_RX_BD_BASE + (i * 8) + 4,
		                 (cyg_uint32)&data->rx_buffer[i][0]);
	}
	reg |= ETHMAC_RX_BD_WR;
	i--;
	HAL_WRITE_UINT32(ETHMAC_RX_BD_BASE + (i * 8), reg);

	data->rx_head = 0;

	// Below is a configuration of ETHMAC peripheral. The stack
	// sends here frames without the preamble and CRC, so we need
	// to enable them in the peripheral.

	// Disable MAC (just in case)
	HAL_READ_UINT32(ETHMAC_REG_BASE + ETHMAC_MODER, reg);
	reg &= ~(ETHMAC_MODER_TXEN | ETHMAC_MODER_RXEN);
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_MODER, reg);

	// Enable Full Duplex mode, CRC and PADDING
	HAL_READ_UINT32(ETHMAC_REG_BASE + ETHMAC_MODER, reg);
	#ifdef CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_FULLDUPLEX
	reg |= ETHMAC_MODER_FULLD;
	#endif
	reg |= ETHMAC_MODER_CRCEN | ETHMAC_MODER_PAD;
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_MODER, reg);

	#ifdef CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_FULLDUPLEX
	// Reconfigure timing if full duplex mode was selected
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_IPGT, 0x15);

	// Enable PAUSE frames
	reg = ETHMAC_CTRLMODER_RXFLOW | ETHMAC_CTRLMODER_TXFLOW;
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_CTRLMODER, reg);
	#endif

	// Configure packet size
	reg = (0x40 << 16) | CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_PACKETLEN;
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_PACKETLEN, reg);

	// Clear possible interrupts
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_INT_SOURCE, 0x7F);

	// Set MAC address
	unsigned char mac_addr[6] = { CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_MACADDR };
	reg = mac_addr[0] << 8 | mac_addr[1];
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_MAC_ADDR1, reg);
	reg = mac_addr[2] << 24 | mac_addr[3] << 16 | mac_addr[4] << 8 | mac_addr[5];
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_MAC_ADDR0, reg);

	// Attach an interrupt
	cyg_drv_interrupt_create((cyg_vector_t)CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_INT,
				 (cyg_priority_t)CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_INT_PRIO,
	                         (cyg_addrword_t) sc,
	                         (cyg_ISR_t*)ethmac_eth_isr,
	                         (cyg_DSR_t*)eth_drv_dsr,
	                         (cyg_handle_t*)&data->int_handle,
	                         (cyg_interrupt*)&data->int_object);
	cyg_drv_interrupt_attach(data->int_handle);
	cyg_drv_interrupt_acknowledge(CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_INT);
	cyg_drv_interrupt_unmask(CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_INT);

	// Initialize upper level driver by sending MAC address
	(sc->funs->eth_drv->init)(sc, mac_addr);

	return 1;
}

static void ethmac_eth_start(struct eth_drv_sc *sc, 
                             unsigned char *enaddr, 
                             int flags) {
	cyg_uint32 reg;

	// Enable transceiver
	HAL_READ_UINT32(ETHMAC_REG_BASE + ETHMAC_MODER, reg);
	reg |= ETHMAC_MODER_TXEN | ETHMAC_MODER_RXEN;
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_MODER, reg);

	// Enable interrutps. We want to track transmission and
	// reception of data frames only. Control frames are ignored.
	// They are not passed to the host.
	reg = ETHMAC_INT_MASK_RXE_M | ETHMAC_INT_MASK_RXF_M |
	      ETHMAC_INT_MASK_TXE_M | ETHMAC_INT_MASK_TXB_M;
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_INT_MASK, reg);

}

static void ethmac_eth_stop(struct eth_drv_sc *sc) {
	cyg_uint32 reg;

	// Disable transceiver
	HAL_READ_UINT32(ETHMAC_REG_BASE + ETHMAC_MODER, reg);
	reg &= ~(ETHMAC_MODER_TXEN | ETHMAC_MODER_RXEN);
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_MODER, reg);

	// Disable interrupts
	reg = 0;
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_INT_MASK, reg);

}

static int ethmac_eth_control(struct eth_drv_sc *sc,
                          unsigned long key,
                          void *data,
                          int len) {
	//TODO implement following keys:
	//ETH_DRV_GET_MAC_ADDRESS
	//ETH_DRV_SET_MAC_ADDRESS
	//ETH_DRV_GET_IF_STATS_UD
	//ETH_DRV_GET_IF_STATS
	//ETH_DRV_SET_MC_LIST
	//ETH_DRV_SET_MC_ALL
	return -1;
}

static int ethmac_eth_can_send(struct eth_drv_sc *sc) {

	ethmac_eth_t *data = sc->driver_private;

	// This function should return the number of free transmission
	// slots.

	int slots;

	cyg_drv_dsr_lock();
	slots = CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_TXBUF_COUNT - data->tx_count;
	cyg_drv_dsr_unlock();

	return slots;
}

static void ethmac_eth_send(struct eth_drv_sc *sc,
                           struct eth_drv_sg *sg_list,
                           int sg_len,
                           int total_len,
                           unsigned long key) {

	// Reject too big frames
	if(total_len > CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_PACKETLEN) {

		// Immediately notify the stack, that transmission failed
		sc->funs->eth_drv->tx_done(sc, key, -1);
		return;
	}

	ethmac_eth_t *data = sc->driver_private;

	// Determine the next free buffer descriptor
	int free_bd;
	cyg_drv_dsr_lock();
	free_bd = (data->tx_head + data->tx_count) % CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_TXBUF_COUNT;
	cyg_drv_dsr_unlock();

	// Upper stack gives us a scatter-gather list. We need to assemble that into one
	// single packet. We store the packet directly in the transmission buffer.
	int i, j;
	j = 0;
	for(i = 0; i < sg_len; i++) {
		memcpy(&data->tx_buffer[free_bd][j], (char*)sg_list[i].buf, sg_list[i].len);
		j += sg_list[i].len;
	}

	// Store key (handler) so it will be possible to notify the stack later
	data->tx_key[free_bd] = key;

	// Reconfigure the buffer descriptor so notify ETHMAC it is ready for transmission
	cyg_uint32 reg;
	reg = (total_len << 16) | ETHMAC_TX_BD_RD | ETHMAC_TX_BD_IRQ | ETHMAC_TX_BD_PAD | ETHMAC_TX_BD_CRC;
	if(free_bd == CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_TXBUF_COUNT - 1) {
		reg |= ETHMAC_TX_BD_WR;
	}
	HAL_WRITE_UINT32(ETHMAC_TX_BD_BASE + (8 * free_bd), reg);

	// Update queue control variables
	cyg_drv_dsr_lock();
	data->tx_count++;
	cyg_drv_dsr_unlock();

}

static void ethmac_eth_recv(struct eth_drv_sc *sc,
                            struct eth_drv_sg *sg_list,
                            int sg_len) {

	ethmac_eth_t *data = sc->driver_private;

	// This function is called by the upper layer as a result of calling
	// _recv callback. We need to move data from reception buffer
	// to given scatter-gather list.

	if(sg_list == NULL) {
		return;
	}

	// Copy..
	int i, j;
	j = 0;
	for(i = 0; i < sg_len; i++) {
		if(sg_list[i].buf) {
			memcpy((char*)sg_list[i].buf, data->rx_buffer[data->rx_head] + j, sg_list[i].len);
			j += sg_list[i].len;
		}
	}

}

static void ethmac_eth_deliver(struct eth_drv_sc *sc) {

	// This function is called from default network DSR provided
	// by common eCos package. Is is called when network
	// interrupt occurs. It simply calls _poll to analyze status
	// registers. Interrupt flags in MAC registers are cleared in 
	// _poll function.
	ethmac_eth_poll(sc);

	// Unmask the interrupt
	cyg_drv_interrupt_unmask(CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_INT);

}

static void ethmac_eth_poll(struct eth_drv_sc *sc) {

	cyg_uint32 reg;
	ethmac_eth_t *data = sc->driver_private;

	// This function is designed to be idempotent. It can be called multiple
	// times. We don't need to count how many interrupts occured.
	// This function analyzes ETHMAC status registers to find out if any
	// packets were received or transmitted. 

	// Check if any packets were received

	// Clear flags associated with reception
	reg = ETHMAC_INT_SOURCE_RXC | ETHMAC_INT_SOURCE_BUSY |
	      ETHMAC_INT_SOURCE_RXE | ETHMAC_INT_SOURCE_RXB;
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_INT_SOURCE, reg);

	// Traverse receive buffer descriptors to see if any frames were received

	cyg_uint32 errors = ETHMAC_RX_BD_CF | ETHMAC_RX_BD_M |
	                    ETHMAC_RX_BD_OR | ETHMAC_RX_BD_IS |
		            ETHMAC_RX_BD_DN | ETHMAC_RX_BD_TL |
		            ETHMAC_RX_BD_SF | ETHMAC_RX_BD_CRC |
		            ETHMAC_RX_BD_LC;

	cyg_drv_dsr_lock();

	while(1) {

		HAL_READ_UINT32(ETHMAC_RX_BD_BASE + (8*data->rx_head), reg);

		if(reg & ETHMAC_RX_BD_E) {
			// This buffer descriptor is not filled yet
			break;
		}

		if(reg & errors) {
			if(reg & ETHMAC_RX_BD_CF) 
				ethmac_eth_debug("RX Control frame");
			if(reg & ETHMAC_RX_BD_M) 
				ethmac_eth_debug("RX Miss");
			if(reg & ETHMAC_RX_BD_OR)
				ethmac_eth_debug("RX Overrun");
			if(reg & ETHMAC_RX_BD_IS) 
				ethmac_eth_debug("RX Invalid symbol");
			if(reg & ETHMAC_RX_BD_DN) 
				ethmac_eth_debug("RX Dribble nibble");
			if(reg & ETHMAC_RX_BD_TL) 
				ethmac_eth_debug("RX Too long");
			if(reg & ETHMAC_RX_BD_SF) 
				ethmac_eth_debug("RX Short frame");
			if(reg & ETHMAC_RX_BD_CRC) 
				ethmac_eth_debug("RX CRC error");
			if(reg & ETHMAC_RX_BD_LC)
				ethmac_eth_debug("RX Late collision");
		}
		else {
			// We need to notify the stack to prepare buffers for this frame.
			// Last 4 bytes need to be cut as ETHMAC attaches CRC behind
			// frames.
			int size = reg >> 16;
			size -= 4;
			sc->funs->eth_drv->recv(sc, size);
		}

		// Reconfigure buffer descriptor so it can accept new frame
		reg = CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_PACKETLEN << 16 |
	              ETHMAC_RX_BD_E | ETHMAC_RX_BD_IRQ;
		if(data->rx_head == CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_RXBUF_COUNT-1) {
			reg |= ETHMAC_RX_BD_WR;
		}
		HAL_WRITE_UINT32(ETHMAC_RX_BD_BASE + (8*data->rx_head), reg);

		// Update queue control variables
		data->rx_head = (data->rx_head + 1) % CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_RXBUF_COUNT;

	}

	cyg_drv_dsr_unlock();

	// Check if any packets were sent

	// Clear flags associated with transmission
	reg = ETHMAC_INT_SOURCE_TXC | ETHMAC_INT_SOURCE_TXE |
	      ETHMAC_INT_SOURCE_TXB;
	HAL_WRITE_UINT32(ETHMAC_REG_BASE + ETHMAC_INT_SOURCE, reg);

	// Traverse transmit buffer descriptors to see if any frames were transmitted

	errors = ETHMAC_TX_BD_UR | ETHMAC_TX_BD_RL |
	         ETHMAC_TX_BD_LC | ETHMAC_TX_BD_CS;

	cyg_drv_dsr_lock();

	while(1) {

		HAL_READ_UINT32(ETHMAC_TX_BD_BASE + (8*data->tx_head), reg);

		if((reg & ETHMAC_TX_BD_RD) || (data->tx_count == 0)) {
			// This buffer descriptor is not yet transmitted or
			// there are no buffer descriptors to transmit.
			break;
		}

		if(reg & errors) {
			if(reg & ETHMAC_TX_BD_UR)
				ethmac_eth_debug("TX Underrun");
			if(reg & ETHMAC_TX_BD_RL)
				ethmac_eth_debug("TX Retransmission limit");
			if(reg & ETHMAC_TX_BD_LC)
				ethmac_eth_debug("TX Late collision");
			if(reg & ETHMAC_TX_BD_CS)
				ethmac_eth_debug("TX Carrier lost");

			// Notify the stack that transmission failed.
			sc->funs->eth_drv->tx_done(sc, data->tx_key[data->tx_head], -1);
		}
		else {
			// Notify the stack that transmission succeeded.
			sc->funs->eth_drv->tx_done(sc, data->tx_key[data->tx_head], 0);
		}

		// Update queue control variables.
		data->tx_count--;
		data->tx_head = (data->tx_head + 1) % CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_TXBUF_COUNT;
	}

	cyg_drv_dsr_unlock();

}

static int ethmac_eth_int_vector(struct eth_drv_sc *sc) {

	return CYGPKG_DEVS_ETH_OPENCORES_ETHMAC_INT;

}


// ------------------------------------------------------------------------
// EOF if_ethmac.c

#include <cyg/hal/hal_intr.h>


// Baud rate values, based on internal system (50MHz) clock
#define BAUD_DIVISOR(baud) ((50000000)/(16*baud))
static unsigned short select_baud[] = {
    0,                    // Unused
	BAUD_DIVISOR(50),				//50
	BAUD_DIVISOR(75),				//75
	BAUD_DIVISOR(110),				//110
    0,                    // 134.5
	BAUD_DIVISOR(150),				//150
	BAUD_DIVISOR(200),				//200
	BAUD_DIVISOR(300),				//300
	BAUD_DIVISOR(600),				//600
	BAUD_DIVISOR(1200),				//1200
	BAUD_DIVISOR(1800),				//1800
	BAUD_DIVISOR(2400),				//2400
	BAUD_DIVISOR(3600),				//3600
	BAUD_DIVISOR(4800),				//4800
	BAUD_DIVISOR(7200),				//7200
	BAUD_DIVISOR(9600),				//9600
	BAUD_DIVISOR(14400),				//14400
	BAUD_DIVISOR(19200),				//19200
	BAUD_DIVISOR(38400),				//38400
	BAUD_DIVISOR(57600),				//57600
	BAUD_DIVISOR(115200),				//115200
	BAUD_DIVISOR(230400),				//230400
};

#ifdef CYGPKG_IO_SERIAL_OPENCORES_SERIAL0

static pc_serial_info pc_serial_info0 = {CYGNUM_IO_SERIAL_OPENCORES_SERIAL0_IOBASE,
                                         CYGNUM_IO_SERIAL_OPENCORES_SERIAL0_INT};


#if CYGNUM_IO_SERIAL_OPENCORES_SERIAL0_BUFSIZE > 0
static unsigned char pc_serial_out_buf0[CYGNUM_IO_SERIAL_OPENCORES_SERIAL0_BUFSIZE];
static unsigned char pc_serial_in_buf0[CYGNUM_IO_SERIAL_OPENCORES_SERIAL0_BUFSIZE];

static SERIAL_CHANNEL_USING_INTERRUPTS(pc_serial_channel0,
                                       pc_serial_funs,
                                       pc_serial_info0,
                                       CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_OPENCORES_SERIAL0_BAUD),
                                       CYG_SERIAL_STOP_DEFAULT,
                                       CYG_SERIAL_PARITY_DEFAULT,
                                       CYG_SERIAL_WORD_LENGTH_DEFAULT,
                                       CYG_SERIAL_FLAGS_DEFAULT,
                                       &pc_serial_out_buf0[0], sizeof(pc_serial_out_buf0),
                                       &pc_serial_in_buf0[0], sizeof(pc_serial_in_buf0)
    );
#else
static SERIAL_CHANNEL(pc_serial_channel0,
                      pc_serial_funs,
                      pc_serial_info0,
                      CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_OPENCORES_SERIAL0_BAUD),
                      CYG_SERIAL_STOP_DEFAULT,
                      CYG_SERIAL_PARITY_DEFAULT,
                      CYG_SERIAL_WORD_LENGTH_DEFAULT,
                      CYG_SERIAL_FLAGS_DEFAULT
    );
#endif

DEVTAB_ENTRY(pc_serial_io0,
             CYGDAT_IO_SERIAL_OPENCORES_SERIAL0_NAME,
             0,                     // Does not depend on a lower level interface
             &cyg_io_serial_devio,
             pc_serial_init,
             pc_serial_lookup,     // Serial driver may need initializing
             &pc_serial_channel0
    );
#endif //  CYGPKG_IO_SERIAL_OPENCORES_SERIAL0

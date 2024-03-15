/* ioctl interface */
#define __JTAG_IOCTL_MAGIC	0xb2
#define JTAG_IOCBITBANG	_IOW(__JTAG_IOCTL_MAGIC, 6, unsigned int)

/**
 * struct jtag_bitbang - jtag bitbang:
 *
 * @tms: JTAG TMS
 * @tdi: JTAG TDI (input)
 * @tdo: JTAG TDO (output)
 *
 * Structure provide interface to JTAG device for JTAG bitbang execution.
 */
struct tck_bitbang {
	uint8_t	tms;
	uint8_t	tdi;
	uint8_t	tdo;
} __attribute__((__packed__));

/**
 * struct bitbang_packet - jtag bitbang array packet:
 *
 * @data:   JTAG Bitbang struct array pointer(input/output)
 * @length: array size (input)
 *
 * Structure provide interface to JTAG device for JTAG bitbang bundle execution
 */
struct bitbang_packet {
	struct tck_bitbang *data;
	uint32_t	length;
} __attribute__((__packed__));

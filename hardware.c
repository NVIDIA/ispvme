/**************************************************************************************************************************************************************************************************************

Copyright 2023 Lattice Semiconductor Corp.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
�AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************************************************************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "jtag.h"

/********************************************************************************
* Return codes
*********************************************************************************/
#define INVALID_JTAG_HANDLE -7
#define KERNEL_WRITE_FAIL -8

/********************************************************************************
* Declaration of global variables 
*
*********************************************************************************/

unsigned char  g_siIspPins        = 0x00;   /*Keeper of JTAG pin state*/
unsigned char  g_siIspPinsObmc        = 0x00;   /*Keeper of JTAG pin state for OBMC*/
unsigned short g_usInPort         = 0x379;  /*Address of the TDO pin*/
unsigned short g_usOutPort		  = 0x378;  /*Address of TDI, TMS, TCK pin*/
unsigned short g_usCpu_Frequency  = 1000;   /*Enter your CPU frequency here, unit in MHz.*/
unsigned short g_readTDO		  = 0;
unsigned short g_usHardwareInitialized = 0;
extern int g_iDeviceFd;
extern char *g_devicePath;


/*********************************************************************************
* This is the definition of the bit locations of each respective
* signal in the global variable g_siIspPins.
*
* NOTE: Users must add their own implementation here to define
*       the bit location of the signal to target their hardware.
*       The example below is for the Lattice download cable on
*       on the parallel port.
*
*********************************************************************************/

const unsigned char g_ucPinTDI          = 0x01;    /* Bit address of TDI */
const unsigned char g_ucPinTCK          = 0x02;    /* Bit address of TCK */
const unsigned char g_ucPinTMS          = 0x04;    /* Bit address of TMS */
const unsigned char g_ucPinENABLE       = 0x08;    /* Bit address of ENABLE */
const unsigned char g_ucPinTRST         = 0x10;    /* Bit address of TRST */
const unsigned char g_ucPinTDO          = 0x40;    /* Bit address of TDO*/

/***************************************************************
*
* Functions declared in hardware.c module.
*
***************************************************************/
void writePort( unsigned char a_ucPins, unsigned char a_ucValue );
unsigned char readPort();
int sclock();
void ispVMDelay( unsigned short a_usTimeDelay );
void calibration(void);
uint8_t obmcHwInitialize(void);

/********************************************************************************
* writePort
* To apply the specified value to the pins indicated. This routine will
* be modified for specific systems. 
* As an example, this code uses the IBM-PC standard Parallel port, along with the
* schematic shown in Lattice documentation, to apply the signals to the
* JTAG pins.
*
* PC Parallel port pin    Signal name           Port bit address
*        2                g_ucPinTDI             1
*        3                g_ucPinTCK             2
*        4                g_ucPinTMS             4
*        5                g_ucPinENABLE          8
*        6                g_ucPinTRST            16
*        10               g_ucPinTDO             64
*                     
*  Parameters:
*   - a_ucPins, which is actually a set of bit flags (defined above)
*     that correspond to the bits of the data port. Each of the I/O port
*     bits that drives an isp programming pin is assigned a flag 
*     (through a #define) corresponding to the signal it drives. To 
*     change the value of more than one pin at once, the flags are added 
*     together, much like file access flags are.
*
*     The bit flags are only set if the pin is to be changed. Bits that 
*     do not have their flags set do not have their levels changed. The 
*     state of the port is always manintained in the static global 
*     variable g_siIspPins, so that each pin can be addressed individually 
*     without disturbing the others.
*
*   - a_ucValue, which is either HIGH (0x01 ) or LOW (0x00 ). Only these two
*     values are valid. Any non-zero number sets the pin(s) high.
*
*********************************************************************************/

void writePort( unsigned char a_ucPins, unsigned char a_ucValue )
{
	if ( a_ucValue ) {
		g_siIspPinsObmc = (unsigned char) (a_ucPins | g_siIspPinsObmc);
	}
	else {
		g_siIspPinsObmc = (unsigned char) (~a_ucPins & g_siIspPinsObmc);
	}
	// printf("setting %x to %x; global state: %x\n", a_ucPins, a_ucValue, g_siIspPinsObmc);

	/* This is a sample code for Windows/DOS without Windows Driver.
	_outp( g_usOutPort, g_siIspPins );
	*/
}

/*********************************************************************************
*
* readPort
*
* Returns the value of the TDO from the device.
*
**********************************************************************************/
unsigned char readPort()
{
	// unsigned char ucRet = 0;

	/* This is a sample code for Windows/DOS
	if ( _inp( g_usInPort ) & g_ucPinTDO ) {
		ucRet = 0x01;
    }
	else {
       ucRet = 0x00;
    }
	*/
#ifdef OBMC_AST
	
	// printf("setting %x to %x; global state: %x\n", a_ucPins, a_ucValue, g_siIspPinsObmc);
	// printf("read port\n");
	return (unsigned char) g_readTDO;

#endif //OBMC_AST
	//return ( ucRet );
} 

/*********************************************************************************
* sclock
*
* Apply a pulse to TCK.
*
*
*********************************************************************************/
int sclock()
{	

#ifdef OBMC_AST

	struct tck_bitbang data;
	struct bitbang_packet bb_packet;

	if (!g_usHardwareInitialized)
	{
		uint8_t rc = obmcHwInitialize();
		if (rc) {
			fprintf(stderr, "Error:  OBMC JTAG handle not found\n");
			return rc;
		}
	}

	data.tms = (uint8_t) ((g_siIspPinsObmc >> 2) & 0x1);
	data.tdi = (uint8_t) ((g_siIspPinsObmc) & 0x1);

	bb_packet.length = 1;
	bb_packet.data = &data;

	if (g_iDeviceFd > 0) {
		if( ioctl(g_iDeviceFd, JTAG_IOCBITBANG, &bb_packet) <0 ){
			 printf("IOCTL failure: %d \n", errno);
			 return KERNEL_WRITE_FAIL;
		}
	}

	else {
		printf("Device fd not valid");
		return INVALID_JTAG_HANDLE;
	}

	g_readTDO = data.tdo & 0x1;
	sleep(0.00000001);
	// printf("tdo: %u, tdi: %u, tms: %u\n", g_readTDO, data.tdi, data.tms);

#endif //OBMC_AST

return 0;

}
/********************************************************************************
*
* ispVMDelay
*
*
* Users must implement a delay to observe a_usTimeDelay, where
* bit 15 of the a_usTimeDelay defines the unit.
*      1 = milliseconds
*      0 = microseconds
* Example:
*      a_usTimeDelay = 0x0001 = 1 microsecond delay.
*      a_usTimeDelay = 0x8001 = 1 millisecond delay.
*
* This subroutine is called upon to provide a delay from 1 millisecond to a few 
* hundreds milliseconds each time. 
* It is understood that due to a_usTimeDelay is defined as unsigned short, a 16 bits
* integer, this function is restricted to produce a delay to 64000 micro-seconds 
* or 32000 milli-second maximum. The VME file will never pass on to this function
* a delay time > those maximum number. If it needs more than those maximum, the VME
* file will launch the delay function several times to realize a larger delay time
* cummulatively.
* It is perfectly alright to provide a longer delay than required. It is not 
* acceptable if the delay is shorter.
*
* Delay function example--using the machine clock signal of the native CPU------
* When porting ispVME to a native CPU environment, the speed of CPU or 
* the system clock that drives the CPU is usually known. 
* The speed or the time it takes for the native CPU to execute one for loop 
* then can be calculated as follows:
*       The for loop usually is compiled into the ASSEMBLY code as shown below:
*       LOOP: DEC RA;
*             JNZ LOOP;
*       If each line of assembly code needs 4 machine cycles to execute, 
*       the total number of machine cycles to execute the loop is 2 x 4 = 8.
*       Usually system clock = machine clock (the internal CPU clock). 
*       Note: Some CPU has a clock multiplier to double the system clock for 
              the machine clock.
*
*       Let the machine clock frequency of the CPU be F, or 1 machine cycle = 1/F.
*       The time it takes to execute one for loop = (1/F ) x 8.
*       Or one micro-second = F(MHz)/8;
*
* Example: The CPU internal clock is set to 100Mhz, then one micro-second = 100/8 = 12
*
* The C code shown below can be used to create the milli-second accuracy. 
* Users only need to enter the speed of the cpu.
*
**********************************************************************************/
void ispVMDelay( unsigned short a_usTimeDelay)
{
    uint16_t delay_time = a_usTimeDelay & 0x7FFF; // Remove the flag bit
    uint16_t unit_flag_mask = 0x8000; // Mask for extracting the unit flag

    if (a_usTimeDelay & unit_flag_mask) {
        // Millisecond delay
        usleep(delay_time * 1000);
    } else {
        // Microsecond delay
        usleep(delay_time);
    }
}

/*********************************************************************************
*
* calibration
*
* It is important to confirm if the delay function is indeed providing 
* the accuracy required. Also one other important parameter needed 
* checking is the clock frequency. 
* Calibration will help to determine the system clock frequency 
* and the loop_per_micro value for one micro-second delay of the target 
* specific hardware.
*              
**********************************************************************************/
void calibration(void)
{
	/*Apply 2 pulses to TCK.*/
	writePort( g_ucPinTCK, 0x00 );
	writePort( g_ucPinTCK, 0x01 );
	writePort( g_ucPinTCK, 0x00 );
	writePort( g_ucPinTCK, 0x01 );
	writePort( g_ucPinTCK, 0x00 );

	/*Delay for 1 millisecond. Pass on 1000 or 0x8001 both = 1ms delay.*/
	ispVMDelay(0x8001);

	/*Apply 2 pulses to TCK*/
	writePort( g_ucPinTCK, 0x01 );
	writePort( g_ucPinTCK, 0x00 );
	writePort( g_ucPinTCK, 0x01 );
	writePort( g_ucPinTCK, 0x00 );
}

uint8_t obmcHwInitialize() {
	g_iDeviceFd = open(g_devicePath, O_RDWR);
	if (g_iDeviceFd < 0) {
		return INVALID_JTAG_HANDLE;
	}
	g_usHardwareInitialized = 1;
	return 0;
}

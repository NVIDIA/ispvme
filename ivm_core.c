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
#include <string.h>
#include "vmopcode.h"

/***************************************************************
*
* Global variables used to specify the flow control and data type.
*
*	g_usFlowControl:	flow control register. Each bit in the
*                               register can potentially change the
*                               personality of the embedded engine.
*	g_usDataType:		holds the data type of the current row.
*
***************************************************************/

unsigned short g_usFlowControl	= 0x0000;
unsigned short g_usDataType		= 0x0000;

/***************************************************************
*
* Global variables used to specify the ENDDR and ENDIR.
*
*	g_ucEndDR:		the state that the device goes to after SDR.
*	g_ucEndIR:		the state that the device goes to after SIR.
*
***************************************************************/

unsigned char g_ucEndDR = DRPAUSE;
unsigned char g_ucEndIR = IRPAUSE;

/***************************************************************
*
* Global variables used to support header/trailer.
*
*	g_usHeadDR:		the number of lead devices in bypass.
*	g_usHeadIR:		the sum of IR length of lead devices.
*	g_usTailDR:		the number of tail devices in bypass.
*	g_usTailIR:		the sum of IR length of tail devices.
*
***************************************************************/

unsigned short g_usHeadDR = 0;
unsigned short g_usHeadIR = 0;
unsigned short g_usTailDR = 0;
unsigned short g_usTailIR = 0;

/***************************************************************
*
* Global variable to store the number of bits of data or instruction
* to be shifted into or out from the device.
*
***************************************************************/

unsigned short g_usiDataSize = 0;

/***************************************************************
*
* Stores the frequency. Default to 1 MHz.
*
***************************************************************/

int g_iFrequency = 1000;

/***************************************************************
*
* Stores the maximum amount of ram needed to hold a row of data.
*
***************************************************************/

unsigned short g_usMaxSize = 0;

/***************************************************************
*
* Stores the LSH or RSH value.
*
***************************************************************/

unsigned short g_usShiftValue = 0;

/***************************************************************
*
* Stores the current repeat loop value.
*
***************************************************************/

unsigned short g_usRepeatLoops = 0;

/***************************************************************
*
* Stores the current vendor.
*
***************************************************************/

signed char g_cVendor = LATTICE;

/***************************************************************
*
* Stores the VME file CRC.
*
***************************************************************/

unsigned short g_usCalculatedCRC = 0;

/***************************************************************
*
* Stores the Device Checksum.
*
***************************************************************/
unsigned long g_usChecksum = 0;
unsigned int g_uiChecksumIndex = 0;

/***************************************************************
*
* Stores the current state of the JTAG state machine.
*
***************************************************************/

signed char g_cCurrentJTAGState = 0;

/***************************************************************
*
* Global variables used to support looping.
*
*	g_pucHeapMemory:	holds the entire repeat loop.
*	g_iHeapCounter:		points to the current byte in the repeat loop.
*	g_iHEAPSize:		the current size of the repeat in bytes.
*
***************************************************************/

unsigned char * g_pucHeapMemory		= NULL;
unsigned short g_iHeapCounter		= 0;
unsigned short g_iHEAPSize          = 0;

/***************************************************************
*
* Global variables used to support intelligent programming.
*
*	g_usIntelDataIndex:     points to the current byte of the
*                               intelligent buffer.
*	g_usIntelBufferSize:	holds the size of the intelligent
*                               buffer.
*
***************************************************************/

unsigned short g_usIntelDataIndex	= 0;
unsigned short g_usIntelBufferSize	= 0;

/****************************************************************************
*
* Holds the maximum size of each respective buffer. These variables are used
* to write the HEX files when converting VME to HEX.
*
*****************************************************************************/

unsigned short g_usTDOSize      = 0;
unsigned short g_usMASKSize		= 0;
unsigned short g_usTDISize		= 0;
unsigned short g_usDMASKSize    = 0;
unsigned short g_usLCOUNTSize   = 0;
unsigned short g_usHDRSize		= 0;
unsigned short g_usTDRSize		= 0;
unsigned short g_usHIRSize		= 0;
unsigned short g_usTIRSize		= 0;
unsigned short g_usHeapSize		= 0;

/***************************************************************
*
* Global variables used to store data.
*
*   g_pucOutMaskData:		local RAM to hold one row of MASK data.
*   g_pucInData:			local RAM to hold one row of TDI data.
*	g_pucOutData:			local RAM to hold one row of TDO data.
*	g_pucHIRData:			local RAM to hold the current SIR header.
*	g_pucTIRData:			local RAM to hold the current SIR trailer.
*	g_pucHDRData:			local RAM to hold the current SDR header.
*	g_pucTDRData:			local RAM to hold the current SDR trailer.
*	g_pucIntelBuffer:		local RAM to hold the current intelligent buffer.
*   g_pucOutDMaskData:		local RAM to hold one row of DMASK data.
*
***************************************************************/

unsigned char * g_pucOutMaskData	= NULL,
              * g_pucInData         = NULL,
              * g_pucOutData		= NULL,
              * g_pucHIRData		= NULL,
              * g_pucTIRData		= NULL,
              * g_pucHDRData		= NULL,
              * g_pucTDRData		= NULL,
              * g_pucIntelBuffer	= NULL,
              * g_pucOutDMaskData	= NULL;

/***************************************************************
*
* JTAG state machine transition table.
*
***************************************************************/

struct {
	 unsigned char  CurState;  /* From this state */
	 unsigned char  NextState; /* Step to this state */
	 unsigned char  Pattern;   /* The tragetory of TMS */
	 unsigned char  Pulses;    /* The number of steps */
} g_JTAGTransistions[ 25 ] = {
{ RESET,	RESET,		0xFC, 6 },	/* Transitions from RESET */
{ RESET,	IDLE,		0x00, 1 },
{ RESET,	DRPAUSE,	0x50, 5 },   
{ RESET,	IRPAUSE,	0x68, 6 },   
{ IDLE,		RESET,		0xE0, 3 },	/* Transitions from IDLE */
{ IDLE,		DRPAUSE,	0xA0, 4 },
{ IDLE,		IRPAUSE,	0xD0, 5 },
{ DRPAUSE,	RESET,		0xF8, 5 },	/* Transitions from DRPAUSE */
{ DRPAUSE,	IDLE,		0xC0, 3 },
{ DRPAUSE,	IRPAUSE,	0xF4, 7 },
{ DRPAUSE,	DRPAUSE,	0xE8, 6 },  /* 06/14/06 Support POLING STATUS LOOP*/
{ IRPAUSE,	RESET,		0xF8, 5 },	/* Transitions from IRPAUSE */
{ IRPAUSE,	IDLE,		0xC0, 3 },
{ IRPAUSE,	DRPAUSE,	0xE8, 6 },
{ DRPAUSE,	SHIFTDR,	0x80, 2 },  /* Extra transitions using SHIFTDR */
{ IRPAUSE,	SHIFTDR,	0xE0, 5 },
{ SHIFTDR,	DRPAUSE,	0x80, 2 },
{ SHIFTDR,	IDLE,		0xC0, 3 },
{ IRPAUSE,	SHIFTIR,	0x80, 2 },	/* Extra transitions using SHIFTIR */
{ SHIFTIR,	IRPAUSE,	0x80, 2 },
{ SHIFTIR,	IDLE,		0xC0, 3 },
{ DRPAUSE,	DRCAPTURE,	0xE0, 4 }, /* 11/15/05 Support DRCAPTURE*/
{ DRCAPTURE, DRPAUSE,	0x80, 2 },
{ IDLE,     DRCAPTURE,	0x80, 2 },
{ IRPAUSE,  DRCAPTURE, 	0xE0, 4 }
};

/***************************************************************
*
* List to hold all LVDS pairs.
*
***************************************************************/

LVDSPair * g_pLVDSList           = NULL;
unsigned short g_usLVDSPairCount = 0;

/***************************************************************
*
* Function prototypes.
*
***************************************************************/

signed char ispVMCode();
signed char ispVMDataCode();
long int ispVMDataSize();
void ispVMData( unsigned char * Data );
signed char ispVMShift( signed char Code );
signed char ispVMAmble( signed char Code );
signed char ispVMLoop( unsigned short a_usLoopCount );
signed char ispVMBitShift( signed char mode, unsigned short bits );
void ispVMComment( unsigned short a_usCommentSize );
void ispVMHeader( unsigned short a_usHeaderSize );
signed char ispVMLCOUNT( unsigned short a_usCountSize );
void ispVMClocks( unsigned short Clocks );
void ispVMBypass( signed char ScanType, unsigned short Bits );
void ispVMStateMachine( signed char NextState );
void ispVMStart();
void ispVMEnd();
signed char ispVMSend(unsigned short int);
signed char ispVMRead(unsigned short int);
signed char ispVMReadandSave(unsigned short int);
signed char ispVMProcessLVDS( unsigned short a_usLVDSCount );


/***************************************************************
*
* External variables and functions in ispvm_ui.c module
*
***************************************************************/
extern void vme_out_char(unsigned char charOut);
extern void vme_out_hex(unsigned char hexOut);
extern void vme_out_string(char *stringOut);
extern unsigned char GetByte();
extern void ispVMMemManager( signed char types, unsigned short size );

/***************************************************************
*
* External variables and functions in hardware.c module
*
***************************************************************/
extern void ispVMDelay( unsigned short int a_usMicroSecondDelay );
extern unsigned char readPort();
extern void writePort( unsigned char pins, unsigned char value );
extern void sclock();
extern signed char g_cCurrentJTAGState;
extern const unsigned char g_ucPinTDI;
extern const unsigned char g_ucPinTCK;
extern const unsigned char g_ucPinTMS;
extern const unsigned char g_ucPinENABLE;
extern const unsigned char g_ucPinTRST;
extern const unsigned char g_ucPinTDO;

#ifdef VME_DEBUG

/***************************************************************
*
* GetState
*
* Returns the state as a string based on the opcode. Only used
* for debugging purposes.
*
***************************************************************/

const char * GetState( unsigned char a_ucState )
{
	switch( a_ucState ) {
	case RESET:
		return( "RESET" );
	case IDLE:
		return( "IDLE" );
	case IRPAUSE:
		return( "IRPAUSE" );
	case DRPAUSE:
		return( "DRPAUSE" );
	case SHIFTIR:
		return( "SHIFTIR" );
	case SHIFTDR:
		return( "SHIFTDR" );
	case DRCAPTURE:/* 11/15/05 support DRCAPTURE*/
		return( "DRCAPTURE" );
	default:
		break;
	}

	return 0;
}

/***************************************************************
*
* PrintData
*
* Prints the data. Only used for debugging purposes.
*
***************************************************************/

void PrintData( unsigned short a_iDataSize, unsigned char * a_pucData )
{
	unsigned short usByteSize  = 0;
	unsigned short usBitIndex  = 0;
	signed short usByteIndex   = 0;
	unsigned char ucByte       = 0;
	unsigned char ucFlipByte   = 0;

	if ( a_iDataSize % 8 ) {
		usByteSize = (unsigned short)(a_iDataSize / 8 + 1);
	}
	else {
		usByteSize = (unsigned short)(a_iDataSize / 8);
	}
	printf( "(" );
	for ( usByteIndex = (signed short)(usByteSize - 1); usByteIndex >= 0; usByteIndex-- ) {
		ucByte = a_pucData[ usByteIndex ];
		ucFlipByte = 0x00;

		/***************************************************************
		*
		* Flip each byte.
		*
		***************************************************************/

		for ( usBitIndex = 0; usBitIndex < 8; usBitIndex++ ) {
			ucFlipByte <<= 1;
			if ( ucByte & 0x1) {
				ucFlipByte |= 0x1;
			}

			ucByte >>= 1;
		}

		/***************************************************************
		*
		* Print the flipped byte.
		*
		***************************************************************/

		printf( "%.02X", ucFlipByte );
		if ( ( usByteSize - usByteIndex ) % 40 == 39 ) {
			printf( "\n\t\t" );
		}
		if(usByteIndex < 0)
			break;
	}
	printf( ")" );
}
#endif //VME_DEBUG

/***************************************************************
*
* ispVMDataSize
*
* Returns a VME-encoded number, usually used to indicate the
* bit length of an SIR/SDR command.
*
***************************************************************/

long int ispVMDataSize()
{
	long int iSize           = 0;
	signed char cCurrentByte = 0;
	signed char cIndex       = 0;
	cIndex = 0;
	while ( ( cCurrentByte = GetByte() ) & 0x80 ) {
		iSize |= ( ( long int ) ( cCurrentByte & 0x7F ) ) << cIndex;
		cIndex += 7;
	}
	iSize |= ( ( long int ) ( cCurrentByte & 0x7F ) ) << cIndex;
	return iSize;
}

/***************************************************************
*
* ispVMCode
*
* This is the heart of the embedded engine. All the high-level opcodes
* are extracted here. Once they have been identified, then it
* will call other functions to handle the processing.
*
***************************************************************/

signed char ispVMCode()
{
	unsigned short iRepeatSize = 0;
	signed char cOpcode		   = 0;
	signed char cRetCode       = 0;
	unsigned char ucState      = 0;
	unsigned short usDelay     = 0;
	unsigned short usToggle    = 0;
	unsigned char usByte       = 0;

	/***************************************************************
	*
	* Check the compression flag only if this is the first time
	* this function is entered. Do not check the compression flag if
	* it is being called recursively from other functions within
	* the embedded engine.
	*
	***************************************************************/

	if ( !( g_usDataType & LHEAP_IN ) && !( g_usDataType & HEAP_IN ) ) {
		usByte = GetByte();
		if ( usByte == 0xf1 ) {
			g_usDataType |= COMPRESS;
		}
		else if ( usByte == 0xf2 ) {
			g_usDataType &= ~COMPRESS;
		}
		else {
			return VME_INVALID_FILE;
		}
	}
	
	/***************************************************************
	*
	* Begin looping through all the VME opcodes.
	*
	***************************************************************/

	while ( ( cOpcode = GetByte() ) >= 0 ) {

		switch ( cOpcode ) {
		case STATE:

			/***************************************************************
			*
			* Step the JTAG state machine.
			*
			***************************************************************/

			ucState = GetByte();

			/***************************************************************
			*
			* Step the JTAG state machine to DRCAPTURE to support Looping.
			*
			***************************************************************/

			if ( (g_usDataType & LHEAP_IN) &&
				 (ucState == DRPAUSE ) &&
				 ( g_cCurrentJTAGState == ucState ))
			{
				ispVMStateMachine( DRCAPTURE );
			}

			ispVMStateMachine( ucState );

#ifdef VME_DEBUG
			if ( g_usDataType & LHEAP_IN ) {
				printf( "LDELAY %s ", GetState( ucState ) );
			}
			else {
				printf( "STATE %s;\n", GetState( ucState ) );
			}
#endif //VME_DEBUG
			break;
		case SIR:
		case SDR:
		case XSDR:

#ifdef VME_DEBUG
			switch( cOpcode ) {
			case SIR:
				printf( "SIR " );
				break;
			case SDR:
			case XSDR:
				if ( g_usDataType & LHEAP_IN ) {
					printf( "LSDR " );
				}
				else {
					printf( "SDR " );
				}
				break;
			}
#endif //VME_DEBUG
			/***************************************************************
			*
			* Shift in data into the device.
			*
			***************************************************************/

			cRetCode = ispVMShift( cOpcode );
			if ( cRetCode != 0 ) {
				return ( cRetCode );
			}
			break;
		case WAIT:

			/***************************************************************
			*
			* Observe delay.
			*
			***************************************************************/

			usDelay = (unsigned short) ispVMDataSize();
			ispVMDelay( usDelay );

#ifdef VME_DEBUG
			if ( usDelay & 0x8000 ) {

				/***************************************************************
				*
				* Since MSB is set, the delay time must be decoded to 
				* millisecond. The SVF2VME encodes the MSB to represent 
				* millisecond.
				*
				***************************************************************/

				usDelay &= ~0x8000;
				if ( g_usDataType & LHEAP_IN ) {
					printf( "%.2E SEC;\n", ( float ) usDelay / 1000 );
				}
				else {
					printf( "RUNTEST %.2E SEC;\n", ( float ) usDelay / 1000 );
				}
			}
			else {                       
				
				/***************************************************************
				*
				* Since MSB is not set, the delay time is given as microseconds.
				*
				***************************************************************/

				if ( g_usDataType & LHEAP_IN ) {
					printf( "%.2E SEC;\n", ( float ) usDelay / 1000000 );
				}
				else {
					printf( "RUNTEST %.2E SEC;\n", ( float ) usDelay / 1000000 );
				}
			}
#endif //VME_DEBUG
			break;
		case TCK:

			/***************************************************************
			*
			* Issue clock toggles.
			*
			***************************************************************/

			usToggle = (unsigned short) ispVMDataSize();
			ispVMClocks( usToggle );

#ifdef VME_DEBUG
			printf( "RUNTEST %d TCK;\n", usToggle );
#endif //VME_DEBUG
			break;
		case ENDDR:

			/***************************************************************
			*
			* Set the ENDDR.
			*
			***************************************************************/

			g_ucEndDR = GetByte();

#ifdef VME_DEBUG
			printf( "ENDDR %s;\n", GetState( g_ucEndDR ) );
#endif //VME_DEBUG
			break;
		case ENDIR:

			/***************************************************************
			*
			* Set the ENDIR.
			*
			***************************************************************/

			g_ucEndIR = GetByte();

#ifdef VME_DEBUG
			printf( "ENDIR %s;\n", GetState( g_ucEndIR ) );
#endif //VME_DEBUG
			break;
		case HIR:
		case TIR:
		case HDR:
		case TDR:

#ifdef VME_DEBUG
			switch( cOpcode ) {
			case HIR:
				printf( "HIR " );
				break;
			case TIR:
				printf( "TIR " );
				break;
			case HDR:
				printf( "HDR " );
				break;
			case TDR:
				printf( "TDR " );
				break;
			}
#endif //VME_DEBUG
			
			/***************************************************************
			*
			* Set the header/trailer of the device in order to bypass
			* successfully.
			*
			***************************************************************/

			cRetCode = ispVMAmble( cOpcode );
			if ( cRetCode != 0 ) {
				return ( cRetCode );
			}

#ifdef VME_DEBUG
			printf( ";\n" );
#endif //VME_DEBUG
			break;
		case MEM:

			/***************************************************************
			*
			* The maximum RAM required to support processing one row of the
			* VME file.
			*
			***************************************************************/

			g_usMaxSize = (unsigned short) ispVMDataSize();

#ifdef VME_DEBUG
			printf( "// MEMSIZE %d\n", g_usMaxSize );
#endif //VME_DEBUG
			break;
		case VENDOR:

			/***************************************************************
			*
			* Set the VENDOR type.
			*
			***************************************************************/

			cOpcode = GetByte();
			switch ( cOpcode ) {
			case LATTICE:
#ifdef VME_DEBUG
				printf( "// VENDOR LATTICE\n" );
#endif //VME_DEBUG
				g_cVendor = LATTICE;
				break;
			case ALTERA:
#ifdef VME_DEBUG
				printf( "// VENDOR ALTERA\n" );
#endif //VME_DEBUG
				g_cVendor = ALTERA;
				break;
			case XILINX:
#ifdef VME_DEBUG
				printf( "// VENDOR XILINX\n" );
#endif //VME_DEBUG
				g_cVendor = XILINX;
				break;
			default:
				break;
			}
			break;
		case SETFLOW:

			/***************************************************************
			*
			* Set the flow control. Flow control determines the personality
			* of the embedded engine.
			*
			***************************************************************/

			g_usFlowControl |= (unsigned short) ispVMDataSize();
			break;
		case RESETFLOW:

			/***************************************************************
			*
			* Unset the flow control.
			*
			***************************************************************/

			g_usFlowControl &= (unsigned short) ~( ispVMDataSize() );
			break;
		case HEAP:

			/***************************************************************
			*
			* Allocate heap size to store loops.
			*
			***************************************************************/

			cRetCode = GetByte();
			if ( cRetCode != SECUREHEAP ) {
				return VME_INVALID_FILE;
			}
			g_iHEAPSize = (unsigned short) ispVMDataSize();

			/****************************************************************************
			*
			* Store the maximum size of the HEAP buffer. Used to convert VME to HEX.
			*
			*****************************************************************************/

			if ( g_iHEAPSize > g_usHeapSize ) {
				g_usHeapSize = g_iHEAPSize;
			}

			ispVMMemManager( HEAP, ( unsigned short ) g_iHEAPSize );
			break;
		case REPEAT:

			/***************************************************************
			*
			* Execute loops.
			*
			***************************************************************/

			g_usRepeatLoops = 0;

			iRepeatSize = (unsigned short) ispVMDataSize();
			
			cRetCode = ispVMLoop( ( unsigned short ) iRepeatSize );
			if ( cRetCode != 0 ) {
				return ( cRetCode );
			}
			break;
		case ENDLOOP:

			/***************************************************************
			*
			* Exit point from processing loops.
			*
			***************************************************************/

			return ( cRetCode );
		case ENDVME:

			/***************************************************************
			*
			* The only valid exit point that indicates end of programming.
			*
			***************************************************************/

			return ( cRetCode );
		case SHR:

			/***************************************************************
			*
			* Right-shift address.
			*
			***************************************************************/

			g_usFlowControl |= SHIFTRIGHT;

			g_usShiftValue = (unsigned short) (g_usRepeatLoops * (unsigned short)GetByte());
			break;
		case SHL:

			/***************************************************************
			*
			* Left-shift address.
			*
			***************************************************************/

			g_usFlowControl |= SHIFTLEFT;

			g_usShiftValue = (unsigned short)(g_usRepeatLoops * (unsigned short)GetByte());
			break;
		case FREQUENCY:

			/***************************************************************
			*
			* Set the frequency.
			*
			***************************************************************/

			g_iFrequency = (int) (ispVMDataSize() );
			if(g_iFrequency >= 1000)
			{
				g_iFrequency = g_iFrequency / 1000;
				if(g_iFrequency == 1)
					g_iFrequency = 1000;
#ifdef VME_DEBUG
			printf( "FREQUENCY %.2E HZ;\n", ( float ) g_iFrequency * 1000 );
#endif //VME_DEBUG
			}
			else
			{
				if(g_iFrequency == 0)
					g_iFrequency = 1000;
#ifdef VME_DEBUG
			printf( "FREQUENCY %.2E HZ;\n", ( float ) g_iFrequency );
#endif //VME_DEBUG
			}
			break;
		case LCOUNT:

			/***************************************************************
			*
			* Process LCOUNT command.
			*
			***************************************************************/

			cRetCode = ispVMLCOUNT( ( unsigned short ) ispVMDataSize() );
			if ( cRetCode != 0 ) {
				return ( cRetCode );
			}
			break;
		case VUES:

			/***************************************************************
			*
			* Set the flow control to verify USERCODE.
			*
			***************************************************************/

			g_usFlowControl |= VERIFYUES;
			break;
		case COMMENT:

			/***************************************************************
			*
			* Display comment.
			*
			***************************************************************/

			ispVMComment( ( unsigned short ) ispVMDataSize() );
			break;
		case LVDS:

			/***************************************************************
			*
			* Process LVDS command.
			*
			***************************************************************/

			ispVMProcessLVDS( ( unsigned short ) ispVMDataSize() );
			break;
		case HEADER:

			/***************************************************************
			*
			* Discard header.
			*
			***************************************************************/

			ispVMHeader( ( unsigned short ) ispVMDataSize() );
			break;
		/* 03/14/06 Support Toggle ispENABLE signal*/
		case ispEN:
			ucState = GetByte();
			if((ucState == ON)||(ucState == 0x01))
				writePort( g_ucPinENABLE, 0x01 );
			else
				writePort( g_ucPinENABLE, 0x00 );
			ispVMDelay( 1 );
			break;	
        /* 05/24/06 support Toggle TRST pin*/
		case TRST:
			ucState = GetByte();
			if(ucState == 0x01)
				writePort( g_ucPinTRST, 0x01 );
			else
				writePort( g_ucPinTRST, 0x00 );
			ispVMDelay( 1 );
			break;	                
		default:

			/***************************************************************
			*
			* Invalid opcode encountered.
			*
			***************************************************************/

#ifdef VME_DEBUG
			printf( "\nINVALID OPCODE: 0x%.2X\n", cOpcode );
#endif //VME_DEBUG

			return VME_INVALID_FILE;
		}
	}
	
	/***************************************************************
	*
	* Invalid exit point. Processing the token 'ENDVME' is the only
	* valid way to exit the embedded engine.
	*
	***************************************************************/

	return ( VME_INVALID_FILE ); 
}

/***************************************************************
*
* ispVMDataCode
*
* Processes the TDI/TDO/MASK/DMASK etc of an SIR/SDR command.
*
***************************************************************/

signed char ispVMDataCode()
{
	signed char cDataByte    = 0;
	signed char siDataSource = 0;  /*source of data from file by default*/
	
	if ( g_usDataType & HEAP_IN ) {
		siDataSource = 1;  /*the source of data from memory*/
	}
	
	/****************************************************************************
	*
	* Clear the data type register.
	*
	*****************************************************************************/

	g_usDataType &= ~( MASK_DATA + TDI_DATA + TDO_DATA + DMASK_DATA + CMASK_DATA );
	
	/****************************************************************************
	*
	* Iterate through SIR/SDR command and look for TDI, TDO, MASK, etc.
	*
	*****************************************************************************/

	while ( ( cDataByte = GetByte() ) >= 0 ) { 

			ispVMMemManager( cDataByte, g_usMaxSize );
			switch ( cDataByte ) {
			case TDI:

				/****************************************************************************
				*
				* Store the maximum size of the TDI buffer. Used to convert VME to HEX.
				*
				*****************************************************************************/

				if ( g_usiDataSize > g_usTDISize ) {
					g_usTDISize = g_usiDataSize;
				}
				/****************************************************************************
				*
				* Updated data type register to indicate that TDI data is currently being
				* used. Process the data in the VME file into the TDI buffer.
				*
				*****************************************************************************/

				g_usDataType |= TDI_DATA;
				ispVMData( g_pucInData );
				break;
			case XTDO:

				/****************************************************************************
				*
				* Store the maximum size of the TDO buffer. Used to convert VME to HEX.
				*
				*****************************************************************************/

				if ( g_usiDataSize > g_usTDOSize ) {
					g_usTDOSize = g_usiDataSize;
				}

				/****************************************************************************
				*
				* Updated data type register to indicate that TDO data is currently being
				* used.
				*
				*****************************************************************************/

				g_usDataType |= TDO_DATA;
				break;
            case TDO:

				/****************************************************************************
				*
				* Store the maximum size of the TDO buffer. Used to convert VME to HEX.
				*
				*****************************************************************************/

				if ( g_usiDataSize > g_usTDOSize ) {
					g_usTDOSize = g_usiDataSize;
				}

				/****************************************************************************
				*
				* Updated data type register to indicate that TDO data is currently being
				* used. Process the data in the VME file into the TDO buffer.
				*
				*****************************************************************************/

				g_usDataType |= TDO_DATA;
				ispVMData( g_pucOutData );
				break;
            case MASK:

				/****************************************************************************
				*
				* Store the maximum size of the MASK buffer. Used to convert VME to HEX.
				*
				*****************************************************************************/

				if ( g_usiDataSize > g_usMASKSize ) {
					g_usMASKSize = g_usiDataSize;
				}

				/****************************************************************************
				*
				* Updated data type register to indicate that MASK data is currently being
				* used. Process the data in the VME file into the MASK buffer.
				*
				*****************************************************************************/

				g_usDataType |= MASK_DATA;
				ispVMData( g_pucOutMaskData );
				break;
			case DMASK:

				/****************************************************************************
				*
				* Store the maximum size of the DMASK buffer. Used to convert VME to HEX.
				*
				*****************************************************************************/

				if ( g_usiDataSize > g_usDMASKSize ) {
					g_usDMASKSize = g_usiDataSize;
				}

				/****************************************************************************
				*
				* Updated data type register to indicate that DMASK data is currently being
				* used. Process the data in the VME file into the DMASK buffer.
				*
				*****************************************************************************/

				g_usDataType |= DMASK_DATA;
				ispVMData( g_pucOutDMaskData );
				break;
			case CMASK:

				/****************************************************************************
				*
				* Updated data type register to indicate that CMASK data is currently being
				* used. Process the data in the VME file into the CMASK buffer.
				*
				*****************************************************************************/

				g_usDataType |= CMASK_DATA;
				ispVMData( g_pucOutMaskData );
				break;
			case CONTINUE:
				return ( 0 );
			default:      
				
				/****************************************************************************
				*
				* Encountered invalid opcode.
				*
				*****************************************************************************/

				return ( VME_INVALID_FILE );
			}
			
			switch ( cDataByte ) {
			case TDI:

				/****************************************************************************
				*
				* Left bit shift. Used when performing algorithm looping.
				*
				*****************************************************************************/

				if ( g_usFlowControl & SHIFTLEFT ) {
					ispVMBitShift( SHL, g_usShiftValue );
					g_usFlowControl &= ~SHIFTLEFT;
				}

				/****************************************************************************
				*
				* Right bit shift. Used when performing algorithm looping.
				*
				*****************************************************************************/

				if ( g_usFlowControl & SHIFTRIGHT ) {
					ispVMBitShift( SHR, g_usShiftValue );
					g_usFlowControl &= ~SHIFTRIGHT;
				}               
			default:
				break;
			}
			
			if ( siDataSource ) {
				g_usDataType |= HEAP_IN;  /*restore data from memory*/
			}
	}
	
	if ( siDataSource ) {  /*fetch data from heap memory upon return*/
		g_usDataType |= HEAP_IN;
	}

	if ( cDataByte < 0 ) {

		/****************************************************************************
		*
		* Encountered invalid opcode.
		*
		*****************************************************************************/

		return ( VME_INVALID_FILE );
	}
	else {
		return ( 0 );
	}
}

/***************************************************************
*
* ispVMData
* Extract one row of data operand from the current data type opcode. Perform
* the decompression if necessary. Extra RAM is not required for the
* decompression process. The decompression scheme employed in this module
* is on row by row basis. The format of the data stream:
* [compression code][compressed data stream]
* 0x00    --No compression
* 0x01    --Compress by 0x00.
*           Example:
*           Original stream:   0x000000000000000000000001
*           Compressed stream: 0x01000901
*           Detail:            0x01 is the code, 0x00 is the key, 
*                              0x09 is the count of 0x00 bytes,
*                              0x01 is the uncompressed byte.
* 0x02    --Compress by 0xFF.
*           Example:
*           Original stream:   0xFFFFFFFFFFFFFFFFFFFFFF01
*           Compressed stream: 0x02FF0901
*           Detail:            0x02 is the code, 0xFF is the key, 
*                              0x09 is the count of 0xFF bytes,
*                              0x01 is the uncompressed byte.
* 0x03
* : :
* 0xFE   -- Compress by nibble blocks.
*           Example:
*           Original stream:   0x84210842108421084210
*           Compressed stream: 0x0584210
*           Detail:            0x05 is the code, means 5 nibbles block.
*                              0x84210 is the 5 nibble blocks.
*                              The whole row is 80 bits given by g_usiDataSize.
*                              The number of times the block repeat itself
*                              is found by g_usiDataSize/(4*0x05) which is 4.
* 0xFF   -- Compress by the most frequently happen byte.
*           Example:
*           Original stream:   0x04020401030904040404
*           Compressed stream: 0xFF04(0,1,0x02,0,1,0x01,1,0x03,1,0x09,0,0,0)
*                          or: 0xFF044090181C240
*           Detail:            0xFF is the code, 0x04 is the key.
*                              a bit of 0 represent the key shall be put into
*                              the current bit position and a bit of 1
*                              represent copying the next of 8 bits of data
*                              in.
*
***************************************************************/

void ispVMData( unsigned char * ByteData )
{
	unsigned short size               = 0;
	unsigned short i, j, m, getData   = 0;
	unsigned char cDataByte           = 0;
	unsigned char compress            = 0;
	unsigned short FFcount            = 0;
	unsigned char compr_char          = 0xFF; 
	unsigned short index              = 0;
	signed char compression           = 0;
	
	/*convert number in bits to bytes*/
	if (g_usiDataSize%8>0) {
		size = (unsigned short)(g_usiDataSize/8 + 1);
	}
	else {
		size = (unsigned short)(g_usiDataSize/8);
	}
	
	/* If there is compression, then check if compress by key of 0x00 or 0xFF
	   or by other keys or by nibble blocks*/
	
	if ( g_usDataType & COMPRESS ) {
		compression = 1;
		if ( ( ( compress = GetByte() ) == VAR ) && ( g_usDataType & HEAP_IN ) ) {
			getData = 1;
			g_usDataType &= ~(HEAP_IN);
			compress = GetByte();
		}
		
		switch (compress){
		case 0x00:  
			/* No compression */
			compression = 0; 
			break;            
		case 0x01:  
			/* Compress by byte 0x00 */
			compr_char = 0x00;
			break;
		case 0x02: 
			/* Compress by byte 0xFF */
			compr_char = 0xFF;  
			break;
		case 0xFF: 
			/* Huffman encoding */
			compr_char = GetByte();
			i = 8;
			for ( index = 0; index < size; index++ ) {
				ByteData[ index ] = 0x00;
				if ( i > 7 ) {
					cDataByte = GetByte(); 
					i = 0;
				}
				if ((cDataByte << i++) & 0x80) 
					m = 8;
				else {
					ByteData[index] = compr_char;
					m = 0;
				}
				
				for (j = 0; j < m; j++) {
					if (i > 7) {
						cDataByte = GetByte(); 
						i = 0;
					}
					ByteData[index] |=((cDataByte << i++)&0x80) >> j;
				} 
			}     
			size = 0;
			break;
		default:   
			for (index = 0; index < size; index++) 
				ByteData[index] = 0x00;
			for (index = 0; index < compress; index++) {
				if (index%2 == 0) 
					cDataByte = GetByte();
				for (i = 0; i < size*2/compress; i++){
					j = (unsigned short)(index + (i*(unsigned short)compress));
					/*clear the nibble to zero first*/
					if (j%2) {
						if (index%2) 
							ByteData[j/2] |= cDataByte & 0x0F;
						else 
							ByteData[j/2] |= cDataByte >> 4;
					}
					else {
						if (index%2) 
							ByteData[j/2] |= cDataByte << 4;
						else 
							ByteData[j/2] |= cDataByte & 0xF0;
					}
				}
			}
			size = 0;
			break;
		}
	}
	
	FFcount = 0;

	/* Decompress by byte 0x00 or 0xFF */
	for (index = 0; index < size; index++) {
		if (FFcount <= 0) {
			cDataByte = GetByte();
			if ((cDataByte == VAR) && (g_usDataType&HEAP_IN) && !getData && !(g_usDataType&COMPRESS)) {
				getData = 1;
				g_usDataType &= ~(HEAP_IN);
				cDataByte = GetByte();
			}
			ByteData[index] = cDataByte; 
			if ((compression) &&(cDataByte == compr_char)) /*decompression is on*/
				FFcount = (unsigned short) ispVMDataSize();     /*The number of 0xFF or 0x00 bytes*/
		}
		else {
			FFcount--; /*Use up the 0xFF chain first*/
			ByteData[index] = compr_char;
		}
	}
	
	if (getData) {
		g_usDataType |= HEAP_IN;
		getData = 0;
	}
}

/***************************************************************
*
* ispVMShift
*
* Processes the SDR/XSDR/SIR commands.
*
***************************************************************/

signed char ispVMShift( signed char a_cCode )
{
	unsigned short iDataIndex  = 0;
	unsigned short iReadLoop   = 0;
	signed char cRetCode       = 0;
	
	cRetCode=0;
	g_usiDataSize = (unsigned short) ispVMDataSize();

	g_usDataType &= ~( SIR_DATA + EXPRESS + SDR_DATA );   /*clear the flags first*/
	switch ( a_cCode ) {
	case SIR:
		g_usDataType |= SIR_DATA;
		/* 1/15/04 If performing cascading, then go directly to SHIFTIR.  Else, 
		   go to IRPAUSE before going to SHIFTIR */
		if ( g_usFlowControl & CASCADE ) {
			ispVMStateMachine( SHIFTIR );
		}
		else {
			ispVMStateMachine( IRPAUSE );
			ispVMStateMachine( SHIFTIR );
			if ( g_usHeadIR > 0 ){ 
				ispVMBypass( HIR, g_usHeadIR );
				sclock();
			}
		}
		break;
	case XSDR:  
		g_usDataType |= EXPRESS; /*mark simultaneous in and out*/
		// Intentional fall through
    case SDR:
		g_usDataType |= SDR_DATA;
		/* 1/15/04 If already in SHIFTDR, then do not move state or shift in header.  
		   This would imply that the previously shifted frame was a cascaded frame.  */
		if ( g_cCurrentJTAGState != SHIFTDR ) {
			/* 1/15/04 If performing cascading, then go directly to SHIFTDR.  Else, 
		       go to DRPAUSE before going to SHIFTDR */
			if ( g_usFlowControl & CASCADE ) {
				if ( g_cCurrentJTAGState == DRPAUSE ) {
					ispVMStateMachine( SHIFTDR );
					/* 1/15/04 If cascade flag has been set and the current state is 
					   DRPAUSE, this implies that the first cascaded frame is about to
					   be shifted in.  The header must be shifted prior to shifting
					   the first cascaded frame. */
					if ( g_usHeadDR > 0 ) {
						ispVMBypass( HDR, g_usHeadDR );
						sclock();
					}
				}
				else {
					ispVMStateMachine( SHIFTDR );
				}
			}
			else {
				ispVMStateMachine( DRPAUSE );
				ispVMStateMachine( SHIFTDR );
				if ( g_usHeadDR > 0 ) {
					ispVMBypass( HDR, g_usHeadDR );
					sclock();
				}
			}
		}
		break;
	default:
		return ( VME_INVALID_FILE );
	}

	cRetCode = ispVMDataCode();
	
	if ( cRetCode != 0 ) {
		return ( VME_INVALID_FILE );
	}

#ifdef VME_DEBUG
	printf( "%d ", g_usiDataSize );

	if ( g_usDataType & TDI_DATA ) {
		printf( "TDI " );
		PrintData( g_usiDataSize, g_pucInData );
	}

	if ( g_usDataType & TDO_DATA ) {
		printf( "\n\t\tTDO " );
		PrintData( g_usiDataSize, g_pucOutData );
	}

	if ( g_usDataType & MASK_DATA ) {
		printf( "\n\t\tMASK " );
		PrintData( g_usiDataSize, g_pucOutMaskData );
	}

	if ( g_usDataType & DMASK_DATA ) {
		printf( "\n\t\tDMASK " );
		PrintData( g_usiDataSize, g_pucOutDMaskData );
	}

	printf( ";\n" );
#endif //VME_DEBUG
	
	if ( g_usDataType & TDO_DATA || g_usDataType & DMASK_DATA ) {
		if(g_usDataType & DMASK_DATA){
			cRetCode = ispVMReadandSave( g_usiDataSize );
			if(!cRetCode){
				if ( g_usTailDR > 0 ) {
					sclock();
					ispVMBypass( TDR, g_usTailDR ); 
				}
				ispVMStateMachine( DRPAUSE );
				ispVMStateMachine( SHIFTDR );
				if( g_usHeadDR > 0 ){
					ispVMBypass( HDR, g_usHeadDR );
					sclock();
				}
				for ( iDataIndex=0; iDataIndex < g_usiDataSize / 8 + 1; iDataIndex++ )
					g_pucInData[ iDataIndex ] = g_pucOutData[ iDataIndex ];
				g_usDataType &= ~( TDO_DATA+ DMASK_DATA );	
				cRetCode = ispVMSend( g_usiDataSize );
			}
		}
		else{
			cRetCode = ispVMRead( g_usiDataSize );
			if ( cRetCode == -1 && g_cVendor == XILINX ) {
				for( iReadLoop = 0; iReadLoop < 30; iReadLoop++ ){
					cRetCode = ispVMRead( g_usiDataSize );
					if( !cRetCode ) {
						break;
					}
					else {
						ispVMStateMachine( DRPAUSE ); /*Always DRPAUSE*/
						/*Bypass other devices when appropriate*/
						ispVMBypass( TDR, g_usTailDR );
						ispVMStateMachine( g_ucEndDR );
						ispVMStateMachine( IDLE );
						ispVMDelay( 1000 );
					}
				}
			}
		}
	}
	else { /*TDI only*/
		cRetCode = ispVMSend( g_usiDataSize );
	}
	
	/*transfer the input data to the output buffer for the next verify*/
	if ( ( g_usDataType & EXPRESS ) || ( a_cCode == SDR ) ) {
		if ( g_pucOutData ) {
			for ( iDataIndex=0; iDataIndex < g_usiDataSize / 8 + 1; iDataIndex++ )
				g_pucOutData[ iDataIndex ] = g_pucInData[ iDataIndex ];
		}
	}
	
	switch( a_cCode ) {
	case SIR:
		/* 1/15/04 If not performing cascading, then shift ENDIR */
		if ( !( g_usFlowControl & CASCADE ) ) {
			if ( g_usTailIR > 0 ) {
				sclock();
				ispVMBypass( TIR, g_usTailIR );
			}
			ispVMStateMachine( g_ucEndIR );
		}
		break;
    case XSDR:
    case SDR: 
		/* 1/15/04 If not performing cascading, then shift ENDDR */
		if ( !( g_usFlowControl & CASCADE ) ) {
			if ( g_usTailDR > 0 ) {
				sclock();
				ispVMBypass( TDR, g_usTailDR );
			}
			ispVMStateMachine( g_ucEndDR );
		}
		break;
    default:
		break;
	}
	
	return ( cRetCode );
}

/***************************************************************
*
* ispVMAmble
*
* This routine is to extract Header and Trailer parameter for SIR and 
* SDR operations.
*
* The Header and Trailer parameter are the pre-amble and post-amble bit
* stream need to be shifted into TDI or out of TDO of the devices. Mostly
* is for the purpose of bypassing the leading or trailing devices. ispVM
* supports only shifting data into TDI to bypass the devices. 
*
* For a single device, the header and trailer parameters are all set to 0
* as default by ispVM. If it is for multiple devices, the header and trailer
* value will change as specified by the VME file.
*
***************************************************************/

signed char ispVMAmble( signed char Code )
{
	signed char compress = 0;
	g_usiDataSize = (unsigned short)ispVMDataSize();
	
#ifdef VME_DEBUG
	printf( "%d", g_usiDataSize );
#endif //VME_DEBUG

	if ( g_usiDataSize ) {

		/****************************************************************************
		*
		* Discard the TDI byte and set the compression bit in the data type register
		* to false if compression is set because TDI data after HIR/HDR/TIR/TDR is not
		* compressed.
		*
		*****************************************************************************/

		GetByte();
		if ( g_usDataType & COMPRESS ) {
			g_usDataType &= ~( COMPRESS );
			compress = 1;
		}
	}
	
	switch ( Code ) {
	case HIR:

		/****************************************************************************
		*
		* Store the maximum size of the HIR buffer. Used to convert VME to HEX.
		*
		*****************************************************************************/

		if ( g_usiDataSize > g_usHIRSize ) {
			g_usHIRSize = g_usiDataSize;
		}
		
		/****************************************************************************
		*
		* Assign the HIR value and allocate memory.
		*
		*****************************************************************************/

		g_usHeadIR = g_usiDataSize;
		if ( g_usHeadIR ) {
			ispVMMemManager( HIR, g_usHeadIR );
			ispVMData( g_pucHIRData );

#ifdef VME_DEBUG
			printf( " TDI " );
			PrintData( g_usHeadIR, g_pucHIRData );
#endif //VME_DEBUG
		}
		break;
	case TIR:

		/****************************************************************************
		*
		* Store the maximum size of the TIR buffer. Used to convert VME to HEX.
		*
		*****************************************************************************/

		if ( g_usiDataSize > g_usTIRSize ) {
			g_usTIRSize = g_usiDataSize;
		}
		
		/****************************************************************************
		*
		* Assign the TIR value and allocate memory.
		*
		*****************************************************************************/

		g_usTailIR = g_usiDataSize;
		if ( g_usTailIR ) {
			ispVMMemManager( TIR, g_usTailIR );
			ispVMData( g_pucTIRData );

#ifdef VME_DEBUG
			printf( " TDI " );
			PrintData( g_usTailIR, g_pucTIRData );
#endif //VME_DEBUG
		}
		break;
	case HDR:

		/****************************************************************************
		*
		* Store the maximum size of the HDR buffer. Used to convert VME to HEX.
		*
		*****************************************************************************/

		if ( g_usiDataSize > g_usHDRSize ) {
			g_usHDRSize = g_usiDataSize;
		}
		
		/****************************************************************************
		*
		* Assign the HDR value and allocate memory.
		*
		*****************************************************************************/

		g_usHeadDR = g_usiDataSize;
		if ( g_usHeadDR ) {
			ispVMMemManager( HDR, g_usHeadDR );
			ispVMData( g_pucHDRData );

#ifdef VME_DEBUG
			printf( " TDI " );
			PrintData( g_usHeadDR, g_pucHDRData );
#endif //VME_DEBUG
		}
		break;
	case TDR:

		/****************************************************************************
		*
		* Store the maximum size of the TDR buffer. Used to convert VME to HEX.
		*
		*****************************************************************************/

		if ( g_usiDataSize > g_usTDRSize ) {
			g_usTDRSize = g_usiDataSize;
		}
		
		/****************************************************************************
		*
		* Assign the TDR value and allocate memory.
		*
		*****************************************************************************/

		g_usTailDR = g_usiDataSize;
		if ( g_usTailDR ) {
			ispVMMemManager( TDR, g_usTailDR );
			ispVMData( g_pucTDRData );

#ifdef VME_DEBUG
			printf( " TDI " );
			PrintData( g_usTailDR, g_pucTDRData );
#endif //VME_DEBUG
		}
		break;
	default:
		break;
	}

	/****************************************************************************
	*
	* Re-enable compression if it was previously set.
	*
	*****************************************************************************/

	if ( compress ) {
		g_usDataType |= COMPRESS;
	}

	if ( g_usiDataSize ) {
		Code = GetByte();
		if ( Code == CONTINUE ) {
			return 0;
		}
		else {

			/****************************************************************************
			*
			* Encountered invalid opcode.
			*
			*****************************************************************************/

			return VME_INVALID_FILE; 
		}
	}

	return 0;
}

/***************************************************************
*
* ispVMLoop
*
* Perform the function call upon by the REPEAT opcode.
* Memory is to be allocated to store the entire loop from REPEAT to ENDLOOP.
* After the loop is stored then execution begin. The REPEATLOOP flag is set
* on the g_usFlowControl register to indicate the repeat loop is in session
* and therefore fetch opcode from the memory instead of from the file.
*
***************************************************************/

signed char ispVMLoop(unsigned short a_usLoopCount)
{
	signed char cRetCode      = 0;
	unsigned short iHeapIndex = 0;
	unsigned short iLoopIndex = 0;
	
	g_usShiftValue = 0;
	for ( iHeapIndex = 0; iHeapIndex < g_iHEAPSize; iHeapIndex++ ) {
		g_pucHeapMemory[ iHeapIndex ] = GetByte();
	}
	
	if ( g_pucHeapMemory[ iHeapIndex - 1 ] != ENDLOOP ) {
		return( VME_INVALID_FILE );
	}
	
	g_usFlowControl |= REPEATLOOP;
	g_usDataType |= HEAP_IN; 
	
	for ( iLoopIndex = 0; iLoopIndex < a_usLoopCount; iLoopIndex++ ) {
		g_iHeapCounter = 0;
		cRetCode = ispVMCode();
		g_usRepeatLoops++;
		if ( cRetCode < 0 ) {
			break;
		}
	}

	g_usDataType &= ~( HEAP_IN );
	g_usFlowControl &= ~( REPEATLOOP );
	return ( cRetCode );
}

/***************************************************************
*
* ispVMBitShift
*
* Shift the TDI stream left or right by the number of bits. The data in 
* *g_pucInData is of the VME format, so the actual shifting is the reverse of
* IEEE 1532 or SVF format.                 
*
***************************************************************/

signed char ispVMBitShift(signed char mode, unsigned short bits)
{
	unsigned short i       = 0;
	unsigned short size    = 0;
	unsigned short tmpbits = 0;

	if (g_usiDataSize%8>0) {
		size = (unsigned short)(g_usiDataSize/8 + 1);
	}
	else {
		size = (unsigned short)(g_usiDataSize/8);
	}

	switch(mode) {
	case SHR:
		for (i = 0; i < size; i++) {
			if (g_pucInData[i] != 0) {			
				tmpbits = bits;
				while (tmpbits > 0) {
					g_pucInData[i] <<= 1;
					if (g_pucInData[i] == 0) {
						i--;
						g_pucInData[i] = 1;
					}
					tmpbits--;
				}
			}
		}
		break;
	case SHL:
		for (i = 0; i < size; i++) {
			if (g_pucInData[i] != 0) {			
				tmpbits = bits;
				while (tmpbits > 0) {
					g_pucInData[i] >>= 1;
					if (g_pucInData[i] == 0) {
						i--;
						g_pucInData[i] = 8;
					}
					tmpbits--;
				}
			}
		}
		break;
	default: 
		return ( VME_INVALID_FILE );
	}

	return (0);
}

/***************************************************************
*
* ispVMComment
*
* Displays the SVF comments.
*
***************************************************************/

void ispVMComment( unsigned short a_usCommentSize )
{
	char cCurByte = 0; 
	for ( ; a_usCommentSize > 0; a_usCommentSize-- ) {
		/****************************************************************************
		*
		* Print character to the terminal.
		*
		*****************************************************************************/
		cCurByte = GetByte();
		vme_out_char( cCurByte );
	}
	cCurByte = '\n';
	vme_out_char(cCurByte );
}

/***************************************************************
*
* ispVMHeader
*
* Iterate the length of the header and discard it.
*
***************************************************************/

void ispVMHeader( unsigned short a_usHeaderSize )
{
	for ( ; a_usHeaderSize > 0; a_usHeaderSize-- ) {
		GetByte();
	}
}

/***************************************************************
*
* ispVMCalculateCRC32
*
* Calculate the 32-bit CRC.
*
***************************************************************/

void ispVMCalculateCRC32( unsigned char a_ucData ) 
{
	unsigned char ucIndex          = 0;
	unsigned char ucFlipData       = 0;
	unsigned short usCRCTableEntry = 0;
	unsigned int crc_table[ 16 ] = {
		0x0000, 0xCC01, 0xD801, 
		0x1400, 0xF001, 0x3C00, 
		0x2800, 0xE401, 0xA001, 
		0x6C00, 0x7800, 0xB401, 
		0x5000, 0x9C01, 0x8801, 
		0x4400
	};

	for ( ucIndex = 0; ucIndex < 8; ucIndex++ ) {
		ucFlipData <<= 1;
		if ( a_ucData & 0x01 ) {
			ucFlipData |= 0x01;
		}
		a_ucData >>= 1;
	}

	usCRCTableEntry = (unsigned short)(crc_table[ g_usCalculatedCRC & 0xF ]);
	g_usCalculatedCRC = (unsigned short)(( g_usCalculatedCRC >> 4 ) & 0x0FFF);
	g_usCalculatedCRC = (unsigned short)(g_usCalculatedCRC ^ usCRCTableEntry ^ crc_table[ ucFlipData & 0xF ]);
	usCRCTableEntry = (unsigned short)(crc_table[ g_usCalculatedCRC & 0xF ]);
	g_usCalculatedCRC = (unsigned short)(( g_usCalculatedCRC >> 4 ) & 0x0FFF);
	g_usCalculatedCRC = (unsigned short)(g_usCalculatedCRC ^ usCRCTableEntry ^ crc_table[ ( ucFlipData >> 4 ) & 0xF ]);
}

/***************************************************************
*
* ispVMLCOUNT
*
* Process the intelligent programming loops.
*
***************************************************************/

signed char ispVMLCOUNT( unsigned short a_usCountSize )
{
	unsigned short usContinue		  = 1;
	unsigned short usIntelBufferIndex = 0;
	unsigned short usCountIndex       = 0;
	signed char cRetCode              = 0;
	signed char cRepeatHeap           = 0;
	signed char cOpcode               = 0;
	unsigned char ucState             = 0;
	unsigned short usDelay            = 0;
	unsigned short usToggle           = 0;
	unsigned char usByte __attribute__((unused))             = 0;

	g_usIntelBufferSize = (unsigned short)ispVMDataSize();

	/****************************************************************************
	*
	* Allocate memory for intel buffer.
	*
	*****************************************************************************/

	ispVMMemManager( LHEAP, g_usIntelBufferSize );

	/****************************************************************************
	*
	* Store the maximum size of the intelligent buffer. Used to convert VME to HEX.
	*
	*****************************************************************************/

	if ( g_usIntelBufferSize > g_usLCOUNTSize ) {
		g_usLCOUNTSize = g_usIntelBufferSize;
	}

	/****************************************************************************
	*
	* Copy intel data to the buffer.
	*
	*****************************************************************************/

	for ( usIntelBufferIndex = 0; usIntelBufferIndex < g_usIntelBufferSize; usIntelBufferIndex++ ) {
		g_pucIntelBuffer[ usIntelBufferIndex ] = GetByte();
	}

	/****************************************************************************
	*
	* Set the data type register to get data from the intelligent data buffer.
	*
	*****************************************************************************/

	g_usDataType |= LHEAP_IN;

	/****************************************************************************
	*
	* If the HEAP_IN flag is set, temporarily unset the flag so data will be
	* retrieved from the status buffer.
	*
	*****************************************************************************/

	if ( g_usDataType & HEAP_IN ) {
		g_usDataType &= ~HEAP_IN;
		cRepeatHeap = 1;
	}

#ifdef VME_DEBUG
	printf( "LCOUNT %d;\n", a_usCountSize );
#endif //VME_DEBUG

	/****************************************************************************
	*
	* Iterate through the intelligent programming command.
	*
	*****************************************************************************/
	
	for ( usCountIndex = 0; usCountIndex < a_usCountSize; usCountIndex++ ) {

		/****************************************************************************
		*
		* Initialize the intel data index to 0 before each iteration.
		*
		*****************************************************************************/

		g_usIntelDataIndex = 0;
		cOpcode            = 0;
		ucState            = 0;
		usDelay            = 0;
		usToggle           = 0;
		usByte             = 0;
		usContinue		   = 1;

		/***************************************************************
		*
		* Begin looping through all the VME opcodes.
		*
		***************************************************************/
		/***************************************************************
		* 4/1/09 Nguyen replaced the recursive function call codes on 
		*        the ispVMLCOUNT function
		*
		***************************************************************/
		while ( usContinue ) 
		{
			cOpcode = GetByte();
			switch ( cOpcode ) {
			case HIR:
			case TIR:
			case HDR:
			case TDR:
				/***************************************************************
				*
				* Set the header/trailer of the device in order to bypass
				* successfully.
				*
				***************************************************************/

				ispVMAmble( cOpcode );
			break;
			case STATE:

				/***************************************************************
				*
				* Step the JTAG state machine.
				*
				***************************************************************/

				ucState = GetByte();
				/***************************************************************
				*
				* Step the JTAG state machine to DRCAPTURE to support Looping.
				*
				***************************************************************/

				if ( (g_usDataType & LHEAP_IN) &&
					 (ucState == DRPAUSE ) &&
					 ( g_cCurrentJTAGState == ucState ))
				{
					ispVMStateMachine( DRCAPTURE );
				}
				ispVMStateMachine( ucState );
#ifdef VME_DEBUG
				printf( "LDELAY %s ", GetState( ucState ) );
#endif //VME_DEBUG
				break;
			case SIR:
#ifdef VME_DEBUG
				printf( "SIR " );					
#endif //VME_DEBUG
				/***************************************************************
				*
				* Shift in data into the device.
				*
				***************************************************************/

				cRetCode = ispVMShift( cOpcode );
				break;
			case SDR:

#ifdef VME_DEBUG
				printf( "LSDR " );
#endif //VME_DEBUG
				/***************************************************************
				*
				* Shift in data into the device.
				*
				***************************************************************/

				cRetCode = ispVMShift( cOpcode );
				break;
			case WAIT:

				/***************************************************************
				*
				* Observe delay.
				*
				***************************************************************/

				usDelay = (unsigned short)ispVMDataSize();
				ispVMDelay( usDelay );

#ifdef VME_DEBUG
				if ( usDelay & 0x8000 ) {

					/***************************************************************
					*
					* Since MSB is set, the delay time must be decoded to 
					* millisecond. The SVF2VME encodes the MSB to represent 
					* millisecond.
					*
					***************************************************************/

					usDelay &= ~0x8000;
					printf( "%.2E SEC;\n", ( float ) usDelay / 1000 );
				}
				else {                       
					
					/***************************************************************
					*
					* Since MSB is not set, the delay time is given as microseconds.
					*
					***************************************************************/

					printf( "%.2E SEC;\n", ( float ) usDelay / 1000000 );
				}
#endif //VME_DEBUG
				break;
			case TCK:

				/***************************************************************
				*
				* Issue clock toggles.
				*
				***************************************************************/

				usToggle = (unsigned short)ispVMDataSize();
				ispVMClocks( usToggle );

#ifdef VME_DEBUG
				printf( "RUNTEST %d TCK;\n", usToggle );
#endif //VME_DEBUG
				break;
			case ENDLOOP:

				/***************************************************************
				*
				* Exit point from processing loops.
				*
				***************************************************************/
				usContinue = 0;
				break;

			case COMMENT:

				/***************************************************************
				*
				* Display comment.
				*
				***************************************************************/

				ispVMComment( ( unsigned short ) ispVMDataSize() );
				break;
			case ispEN:
				ucState = GetByte();
				if((ucState == ON)||(ucState == 0x01))
					writePort( g_ucPinENABLE, 0x01 );
				else
					writePort( g_ucPinENABLE, 0x00 );
				ispVMDelay( 1 );
				break;	
			case TRST:
				if(GetByte() == 0x01)
					writePort( g_ucPinTRST, 0x01 );
				else
					writePort( g_ucPinTRST, 0x00 );
				ispVMDelay( 1 );
				break;
			default:

				/***************************************************************
				*
				* Invalid opcode encountered.
				*
				***************************************************************/

#ifdef VME_DEBUG
				printf( "\nINVALID OPCODE: 0x%.2X\n", cOpcode );
#endif //VME_DEBUG

				return VME_INVALID_FILE;
			}
		}
		if ( cRetCode >= 0 ) {
			/****************************************************************************
			*
			* Break if intelligent programming is successful.
			*
			*****************************************************************************/

			break;
		}

	}
	/****************************************************************************
	*
	* If HEAP_IN flag was temporarily disabled, re-enable it before exiting.
	*
	*****************************************************************************/

	if ( cRepeatHeap ) {
		g_usDataType |= HEAP_IN;
	}

	/****************************************************************************
	*
	* Set the data type register to not get data from the intelligent data buffer.
	*
	*****************************************************************************/

	g_usDataType &= ~LHEAP_IN;
	return cRetCode;
}

/***************************************************************
*
* ispVMClocks
*
* Applies the specified number of pulses to TCK.
*
***************************************************************/

void ispVMClocks( unsigned short Clocks )
{
	unsigned short iClockIndex = 0;
	for ( iClockIndex = 0; iClockIndex < Clocks; iClockIndex++ ) {
		sclock();
	}
}

/***************************************************************
*
* ispVMBypass
*
* This procedure takes care of the HIR, HDR, TIR, TDR for the
* purpose of putting the other devices into Bypass mode. The 
* current state is checked to find out if it is at DRPAUSE or 
* IRPAUSE. If it is at DRPAUSE, perform bypass register scan.
* If it is at IRPAUSE, scan into instruction registers the bypass
* instruction.
*
***************************************************************/

void ispVMBypass( signed char ScanType, unsigned short Bits )
{
	unsigned short iIndex       = 0;
	unsigned short iSourceIndex = 0;
	unsigned char cBitState     = 0;
	unsigned char cCurByte      = 0;   
	unsigned char * pcSource    = NULL;
	
	if ( Bits <= 0 ) {
		return;
	}

	switch ( ScanType ) {
	case HIR:
		pcSource = g_pucHIRData;
		break;
	case TIR:
		pcSource = g_pucTIRData;
		break;
	case HDR:
		pcSource = g_pucHDRData;
		break;
	case TDR:
		pcSource = g_pucTDRData;
		break;
	default:
		break;
	}
	if(pcSource)
	{
		iSourceIndex = 0;
		cBitState = 0;
		for ( iIndex = 0; iIndex < Bits - 1; iIndex++ ) {
			/* Scan instruction or bypass register */
			if ( iIndex % 8 == 0 ) {
				cCurByte = pcSource[ iSourceIndex++ ];
			}
			cBitState = ( unsigned char ) ( ( ( cCurByte << iIndex % 8 ) & 0x80 ) ? 0x01 : 0x00 );
			writePort( g_ucPinTDI, cBitState );
			sclock();
		}   

		if ( iIndex % 8 == 0 )  {
			cCurByte = pcSource[ iSourceIndex++ ];
		}

		cBitState = ( unsigned char ) ( ( ( cCurByte << iIndex % 8 ) & 0x80 ) ? 0x01 : 0x00 );
		writePort( g_ucPinTDI, cBitState );
	}
}

/***************************************************************
*
* ispVMStateMachine
*
* This procedure steps all devices in the daisy chain from a given
* JTAG state to the next desirable state. If the next state is TLR,
* the JTAG state machine is brute forced into TLR by driving TMS
* high and pulse TCK 6 times.
*
***************************************************************/

void ispVMStateMachine( signed char cNextJTAGState )
{
	signed char cPathIndex  = 0;
	signed char cStateIndex = 0;
	short int	found       = 0;
	
	if ( ( g_cCurrentJTAGState == cNextJTAGState ) && ( cNextJTAGState != RESET ) ) {
		return;
	}
	
	for ( cStateIndex = 0; cStateIndex < 25; cStateIndex++ ) {
		if ( ( g_cCurrentJTAGState == g_JTAGTransistions[ cStateIndex ].CurState ) && ( cNextJTAGState == g_JTAGTransistions[cStateIndex].NextState ) ) {
			found = 1;
			break;
		}
	}
	if(found)
	{
		g_cCurrentJTAGState = cNextJTAGState;
		for ( cPathIndex = 0; cPathIndex < g_JTAGTransistions[ cStateIndex ].Pulses; cPathIndex++ ) {
			if ( ( g_JTAGTransistions[ cStateIndex ].Pattern << cPathIndex ) & 0x80 ) {
				writePort( g_ucPinTMS, ( unsigned char ) 0x01 );
			}
			else {
				writePort( g_ucPinTMS, ( unsigned char ) 0x00 );
			}
			sclock();
		}
		
		writePort( g_ucPinTDI, 0x00 );
		writePort( g_ucPinTMS, 0x00 );
	}
}

/***************************************************************
*
* ispVMStart
*
* Enable the port to the device and set the state to RESET (TLR).
*
***************************************************************/

void ispVMStart()
{
#ifdef VME_DEBUG
	printf( "// ISPVM EMBEDDED ADDED\n" );
	printf( "STATE RESET;\n" );
#endif

	ispVMStateMachine( RESET );    /*step devices to RESET state*/
}

/***************************************************************
*
* ispVMEnd
*
* Set the state of devices to RESET to enable the devices and disable 
* the port.
*
***************************************************************/

void ispVMEnd()
{
#ifdef VME_DEBUG
	printf( "// ISPVM EMBEDDED ADDED\n" );
	printf( "STATE RESET;\n" );
	printf( "RUNTEST 1.00E-001 SEC;\n" );
#endif

	ispVMStateMachine( RESET );   /*step devices to RESET state */
	ispVMDelay( 1000 );              /*wake up devices*/
}

/***************************************************************
*
* ispVMSend
*
* Send the TDI data stream to devices. The data stream can be
* instructions or data.
*
***************************************************************/

signed char ispVMSend( unsigned short a_usiDataSize )
{
	unsigned short iIndex       = 0;
	unsigned short iInDataIndex = 0;
	unsigned char cCurByte      = 0;
	unsigned char cBitState     = 0;
	
	for ( iIndex = 0; iIndex < a_usiDataSize - 1; iIndex++ ) { 
		if ( iIndex % 8 == 0 ) { 
			cCurByte = g_pucInData[ iInDataIndex++ ];
		}
		cBitState = ( unsigned char ) ( ( ( cCurByte << iIndex % 8 ) & 0x80 ) ? 0x01 : 0x00 );
		writePort( g_ucPinTDI, cBitState );
		sclock();
	}

	if ( iIndex % 8 == 0 ) {
		/* Take care of the last bit */
		cCurByte = g_pucInData[ iInDataIndex ]; 
	}

	cBitState = ( unsigned char ) ( ( ( cCurByte << iIndex % 8 ) & 0x80 ) ? 0x01 : 0x00 );

	writePort( g_ucPinTDI, cBitState );
	if ( g_usFlowControl & CASCADE ) {
		/* 1/15/04 Clock in last bit for the first n-1 cascaded frames */
		sclock();
	}

	return 0;
}

/***************************************************************
*
* ispVMRead
*
* Read the data stream from devices and verify.
*
***************************************************************/

signed char ispVMRead( unsigned short a_usiDataSize )
{
	unsigned short usDataSizeIndex    = 0;
	unsigned short usErrorCount       = 0;
	unsigned short usLastBitIndex     = 0;
	unsigned char cDataByte           = 0;
	unsigned char cMaskByte           = 0;
	unsigned char cInDataByte         = 0;
	unsigned char cCurBit             = 0;
	unsigned char cByteIndex          = 0;
	unsigned short usBufferIndex      = 0;
	unsigned char ucDisplayByte       = 0x00;
	unsigned char ucDisplayFlag       = 0x01;
	char StrChecksum[256]            = {0};
	unsigned char g_usCalculateChecksum = 0x00;

	usLastBitIndex = (unsigned short)(a_usiDataSize - 1);
	
#ifndef VME_DEBUG
	/****************************************************************************
	*
	* If mask is not all zeros, then set the display flag to 0x00, otherwise
	* it shall be set to 0x01 to indicate that data read from the device shall
	* be displayed. If VME_DEBUG is defined, always display data.
	*
	*****************************************************************************/

	for ( usDataSizeIndex = 0; usDataSizeIndex < ( a_usiDataSize + 7 ) / 8; usDataSizeIndex++ ) {
		if ( g_usDataType & MASK_DATA ) {
			if ( g_pucOutMaskData[ usDataSizeIndex ] != 0x00 ) {
				ucDisplayFlag = 0x00;
				break;
			}
		}
		else if ( g_usDataType & CMASK_DATA ) {
			g_usCalculateChecksum = 0x01;
			ucDisplayFlag = 0x00;
			break;
		}
		else {
			ucDisplayFlag = 0x00;
			break;
		}
	}
#endif //VME_DEBUG
	
	/****************************************************************************
	*
	* Begin shifting data in and out of the device.
	*
	*****************************************************************************/

	for ( usDataSizeIndex = 0; usDataSizeIndex < a_usiDataSize; usDataSizeIndex++ ) { 
		if ( cByteIndex == 0 ) {

			/***************************************************************
			*
			* Grab byte from TDO buffer.
			*
			***************************************************************/

			if ( g_usDataType & TDO_DATA ) {
				cDataByte = g_pucOutData[ usBufferIndex ];
			}
			
			/***************************************************************
			*
			* Grab byte from MASK buffer.
			*
			***************************************************************/

			if ( g_usDataType & MASK_DATA ) {
				cMaskByte = g_pucOutMaskData[ usBufferIndex ];
			}
			else { 
				cMaskByte = 0xFF;
			}
          
			/***************************************************************
			*
			* Grab byte from CMASK buffer.
			*
			***************************************************************/

			if ( g_usDataType & CMASK_DATA ) {
				cMaskByte = 0x00;
				g_usCalculateChecksum = 0x01;
			}
			
			/***************************************************************
			*
			* Grab byte from TDI buffer.
			*
			***************************************************************/

			if ( g_usDataType & TDI_DATA ) {
				cInDataByte = g_pucInData[ usBufferIndex ];
			}
			
			usBufferIndex++;
		}
		
		cCurBit = readPort();
		
		if ( ucDisplayFlag ) {
			ucDisplayByte <<= 1;
			ucDisplayByte |= cCurBit;
		}

		/****************************************************************************
		*
		* Check if data read from port matches with expected TDO.
		*
		*****************************************************************************/

		if ( g_usDataType & TDO_DATA ) {
			if( g_usCalculateChecksum )
			{
				if(cCurBit == 0x01)
					g_usChecksum += (1 << (g_uiChecksumIndex % 8));	
				g_uiChecksumIndex++;
			}
			else
			{
				if ( ( ( ( cMaskByte << cByteIndex ) & 0x80 ) ? 0x01 : 0x00 ) ) {	
					if ( cCurBit != ( unsigned char ) ( ( ( cDataByte << cByteIndex ) & 0x80 ) ? 0x01 : 0x00 ) ) {
						usErrorCount++;  
					}
				}
			}
		}

		/****************************************************************************
		*
		* Write TDI data to the port.
		*
		*****************************************************************************/
		
		writePort( g_ucPinTDI, ( unsigned char ) ( ( ( cInDataByte << cByteIndex ) & 0x80 ) ? 0x01 : 0x00 ) );
		
		if ( usDataSizeIndex < usLastBitIndex ) {

			/****************************************************************************
			*
			* Clock data out from the data shift register.
			*
			*****************************************************************************/

			sclock();
		}
		else if ( g_usFlowControl & CASCADE ) {
			
			/****************************************************************************
			*
			* Clock in last bit for the first N - 1 cascaded frames.
			*
			*****************************************************************************/

			sclock();
		}

		/***************************************************************
		*
		* Increment the byte index. If it exceeds 7, then reset it back
		* to zero.
		*
		***************************************************************/

		cByteIndex++;
		if ( cByteIndex >= 8 ) {
			if ( ucDisplayFlag ) {

				/***************************************************************
				*
				* Store displayed data in the TDO buffer. By reusing the TDO
				* buffer to store displayed data, there is no need to allocate
				* a buffer simply to hold display data. This will not cause any
				* false verification errors because the true TDO byte has already
				* been consumed.
				*
				***************************************************************/

				g_pucOutData[ usBufferIndex - 1 ] = ucDisplayByte;
				ucDisplayByte = 0;
			}

			cByteIndex = 0;
		}
		//09/12/07 Nguyen changed to display the 1 bit expected data
		else if(a_usiDataSize == 1)
		{
			if ( ucDisplayFlag ) {

				/***************************************************************
				*
				* Store displayed data in the TDO buffer. By reusing the TDO
				* buffer to store displayed data, there is no need to allocate
				* a buffer simply to hold display data. This will not cause any
				* false verification errors because the true TDO byte has already
				* been consumed.
				*
				***************************************************************/

				/****************************************************************************
				*
				* Flip ucDisplayByte and store it in cDataByte.
				*
				*****************************************************************************/
				cDataByte = 0x00;
				for ( usBufferIndex = 0; usBufferIndex < 8; usBufferIndex++ ) {
					cDataByte <<= 1;
					if ( ucDisplayByte & 0x01 ) {
						cDataByte |= 0x01;
					}
					ucDisplayByte >>= 1;
				}
				g_pucOutData[ 0 ] = cDataByte;
				ucDisplayByte = 0;
			}

			cByteIndex = 0;
		}
	}
	if ( ucDisplayFlag ) {

		/****************************************************************************
		*
		* Display data read from the device.
		*
		*****************************************************************************/

#ifdef VME_DEBUG
		printf( "RECEIVED TDO (" );
#else
		vme_out_string( "Display Data: 0x" );
#endif //VME_DEBUG

		for ( usDataSizeIndex = (unsigned short)( ( a_usiDataSize + 7 ) / 8 ); usDataSizeIndex > 0 ; usDataSizeIndex-- ) {
			cMaskByte = g_pucOutData[ usDataSizeIndex - 1 ];
			cDataByte = 0x00;

			/****************************************************************************
			*
			* Flip cMaskByte and store it in cDataByte.
			*
			*****************************************************************************/

			for ( usBufferIndex = 0; usBufferIndex < 8; usBufferIndex++ ) {
				cDataByte <<= 1;
				if ( cMaskByte & 0x01 ) {
					cDataByte |= 0x01;
				}
				cMaskByte >>= 1;
			}
#ifdef VME_DEBUG
			printf( "%.2X", cDataByte );
			if ( ( ( ( a_usiDataSize + 7 ) / 8 ) - usDataSizeIndex ) % 40 == 39 ) {
				printf( "\n\t\t" );
			}
#else
			vme_out_hex( cDataByte );
#endif //VME_DEBUG
		}

#ifdef VME_DEBUG
		printf( ")\n\n" );
#else
		vme_out_string( "\n\n" );
#endif //VME_DEBUG
		//09/02/08 Nguyen changed to display the data Checksum		
		if(g_usChecksum != 0)
		{
			g_usChecksum &= 0xFFFF;
			sprintf(StrChecksum,"Data Checksum: %.4lX\n\n",g_usChecksum);
			vme_out_string(StrChecksum);
			g_usChecksum = 0;
		}
	}

	if ( usErrorCount > 0 ) {
		if ( g_usFlowControl & VERIFYUES ) {
			vme_out_string( "USERCODE verification failed.  Continue programming......\n\n" );
			g_usFlowControl &= ~( VERIFYUES );
			return 0;
		}
		else {

#ifdef VME_DEBUG
			printf( "TOTAL ERRORS: %d\n", usErrorCount );
#endif //VME_DEBUG

			return VME_VERIFICATION_FAILURE;
		}
	}
	else {
		if ( g_usFlowControl & VERIFYUES ) {
			vme_out_string( "USERCODE verification passed.  Programming aborted. \n\n" );
			g_usFlowControl &= ~( VERIFYUES );
			return 1;
		}
		else {
			return 0;
		}
	}
}

/***************************************************************
*
* ispVMReadandSave
*
* Support dynamic I/O.
*
***************************************************************/

signed char ispVMReadandSave( unsigned short int a_usiDataSize )
{
	unsigned short int usDataSizeIndex = 0;
	unsigned short int usLastBitIndex  = 0;
	unsigned short int usBufferIndex   = 0;
	unsigned short int usOutBitIndex   = 0;
	unsigned short int usLVDSIndex     = 0;
	unsigned char cDataByte            = 0;
	unsigned char cDMASKByte           = 0;
	unsigned char cInDataByte          = 0;
	unsigned char cCurBit              = 0;
	unsigned char cByteIndex           = 0;
	signed char cLVDSByteIndex         = 0;

	usLastBitIndex = (unsigned short) (a_usiDataSize - 1);
	
	/***************************************************************
	*
	* Iterate through the data bits.
	*
	***************************************************************/

	for ( usDataSizeIndex = 0; usDataSizeIndex < a_usiDataSize; usDataSizeIndex++ ) {
		if ( cByteIndex == 0 ) {

			/***************************************************************
			*
			* Grab byte from DMASK buffer.
			*
			***************************************************************/

			if ( g_usDataType & DMASK_DATA ) {
				cDMASKByte = g_pucOutDMaskData[ usBufferIndex ];
			}
			else { 
				cDMASKByte = 0x00;
			}
          
			/***************************************************************
			*
			* Grab byte from TDI buffer.
			*
			***************************************************************/

			if ( g_usDataType & TDI_DATA ) {
				cInDataByte = g_pucInData[ usBufferIndex ];
			}

			usBufferIndex++;
		}
		
		cCurBit = readPort();
		cDataByte = ( unsigned char ) ( ( ( cInDataByte << cByteIndex ) & 0x80 ) ? 0x01 : 0x00 );

		/***************************************************************
		*
		* Initialize the byte to be zero.
		*
		***************************************************************/

		if ( usOutBitIndex % 8 == 0 ) {
			g_pucOutData[ usOutBitIndex / 8 ] = 0x00;
		}

		/***************************************************************
		*
		* Use TDI, DMASK, and device TDO to create new TDI (actually
		* stored in g_pucOutData).
		*
		***************************************************************/

		if ( ( ( ( cDMASKByte << cByteIndex ) & 0x80 ) ? 0x01 : 0x00 ) ) {

			if ( g_pLVDSList ) {
				for ( usLVDSIndex = 0; usLVDSIndex < g_usLVDSPairCount; usLVDSIndex++ ) {
					if ( g_pLVDSList[ usLVDSIndex ].usNegativeIndex == usDataSizeIndex ) {
						g_pLVDSList[ usLVDSIndex ].ucUpdate = 0x01;
						break;
					}
				}
			}

			/***************************************************************
			*
			* DMASK bit is 1, use TDI.
			*
			***************************************************************/

			g_pucOutData[ usOutBitIndex / 8 ] |= ( unsigned char ) ( ( ( cDataByte & 0x1 ) ? 0x01 : 0x00 ) << ( 7 - usOutBitIndex % 8 ) );
		}
		else {

			/***************************************************************
			*
			* DMASK bit is 0, use device TDO.
			*
			***************************************************************/

			g_pucOutData[ usOutBitIndex / 8 ] |= ( unsigned char ) ( ( ( cCurBit & 0x1 ) ? 0x01 : 0x00 ) << ( 7 - usOutBitIndex % 8 ) );
		}

		/***************************************************************
		*
		* Shift in TDI in order to get TDO out.
		*
		***************************************************************/

		usOutBitIndex++;
		writePort( g_ucPinTDI, cDataByte );
		if ( usDataSizeIndex < usLastBitIndex ) {
			sclock();
		}

		/***************************************************************
		*
		* Increment the byte index. If it exceeds 7, then reset it back
		* to zero.
		*
		***************************************************************/

		cByteIndex++;
		if ( cByteIndex >= 8 ) {
			cByteIndex = 0;
		}
	}

	/***************************************************************
	*
	* If g_pLVDSList exists and pairs need updating, then update
	* the negative-pair to receive the flipped positive-pair value.
	*
	***************************************************************/

	if ( g_pLVDSList ) {
		for ( usLVDSIndex = 0; usLVDSIndex < g_usLVDSPairCount; usLVDSIndex++ ) {
			if ( g_pLVDSList[ usLVDSIndex ].ucUpdate ) {
				
				/***************************************************************
				*
				* Read the positive value and flip it.
				*
				***************************************************************/

				cDataByte = ( unsigned char ) ( ( ( g_pucOutData[ g_pLVDSList[ usLVDSIndex ].usPositiveIndex / 8 ] << ( g_pLVDSList[ usLVDSIndex ].usPositiveIndex % 8 ) ) & 0x80 ) ? 0x01 : 0x00 );
				cDataByte = ( unsigned char ) (!cDataByte);

				/***************************************************************
				*
				* Get the byte that needs modification.
				*
				***************************************************************/

				cInDataByte = g_pucOutData[ g_pLVDSList[ usLVDSIndex ].usNegativeIndex / 8 ];

				if ( cDataByte ) {

					/***************************************************************
					*
					* Copy over the current byte and set the negative bit to 1.
					*
					***************************************************************/

					cDataByte = 0x00;
					for ( cLVDSByteIndex = 7; cLVDSByteIndex >= 0; cLVDSByteIndex-- ) {
						cDataByte <<= 1;
						if ( 7 - ( g_pLVDSList[ usLVDSIndex ].usNegativeIndex % 8 ) == cLVDSByteIndex ) {

							/***************************************************************
							*
							* Set negative bit to 1.
							*
							***************************************************************/

							cDataByte |= 0x01;
						}
						else if ( cInDataByte & 0x80 ) {
							cDataByte |= 0x01;
						}

						cInDataByte <<= 1;
					}

					/***************************************************************
					*
					* Store the modified byte.
					*
					***************************************************************/

					g_pucOutData[ g_pLVDSList[ usLVDSIndex ].usNegativeIndex / 8 ] = cDataByte;
				}
				else {

					/***************************************************************
					*
					* Copy over the current byte and set the negative bit to 0.
					*
					***************************************************************/

					cDataByte = 0x00;
					for ( cLVDSByteIndex = 7; cLVDSByteIndex >= 0; cLVDSByteIndex-- ) {
						cDataByte <<= 1;
						if ( 7 - ( g_pLVDSList[ usLVDSIndex ].usNegativeIndex % 8 ) == cLVDSByteIndex ) {

							/***************************************************************
							*
							* Set negative bit to 0.
							*
							***************************************************************/

							cDataByte |= 0x00;
						}
						else if ( cInDataByte & 0x80 ) {
							cDataByte |= 0x01;
						}

						cInDataByte <<= 1;
					}

					/***************************************************************
					*
					* Store the modified byte.
					*
					***************************************************************/

					g_pucOutData[ g_pLVDSList[ usLVDSIndex ].usNegativeIndex / 8 ] = cDataByte;
				}
				
				break;
			}
		}
	}

	return( 0 );
}

signed char ispVMProcessLVDS( unsigned short a_usLVDSCount )
{
	unsigned short usLVDSIndex = 0;
	
	/***************************************************************
	*
	* Allocate memory to hold LVDS pairs.
	*
	***************************************************************/

	ispVMMemManager( LVDS, a_usLVDSCount );
	g_usLVDSPairCount = a_usLVDSCount;

#ifdef VME_DEBUG
	printf( "LVDS %d (", a_usLVDSCount );
#endif //VME_DEBUG

	/***************************************************************
	*
	* Iterate through each given LVDS pair.
	*
	***************************************************************/

	for ( usLVDSIndex = 0; usLVDSIndex < g_usLVDSPairCount; usLVDSIndex++ ) {

		/***************************************************************
		*
		* Assign the positive and negative indices of the LVDS pair.
		*
		***************************************************************/

		g_pLVDSList[ usLVDSIndex ].usPositiveIndex = (unsigned short) ispVMDataSize();
		g_pLVDSList[ usLVDSIndex ].usNegativeIndex = (unsigned short)ispVMDataSize();

#ifdef VME_DEBUG
		if ( usLVDSIndex < g_usLVDSPairCount - 1 ) {
			printf( "%d:%d, ", g_pLVDSList[ usLVDSIndex ].usPositiveIndex, g_pLVDSList[ usLVDSIndex ].usNegativeIndex );
		}
		else {
			printf( "%d:%d", g_pLVDSList[ usLVDSIndex ].usPositiveIndex, g_pLVDSList[ usLVDSIndex ].usNegativeIndex );
		}
#endif //VME_DEBUG

	}

#ifdef VME_DEBUG
	printf( ");\n", a_usLVDSCount );
#endif //VME_DEBUG

	return( 0 );
}

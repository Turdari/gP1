/*
 * mcc.h - mcc.c header file
 *
 * (C) Copyright 2002
 * Gianfranco Morandi, Eurostudio s.r.l., gianfranco.morandi@euro-studio.it
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA        
 *
*/

#define TRUE 					1
#define FALSE 					0
#define ON						1
#define OFF						0

#define CHNSTART				64

#define NUMBER_OF_CHANNELS 		64  /* logical channels (MCNs) */
#define NUM_RXBDS 				2
#define NUM_TXBDS 				NUM_RXBDS
#define BUFFER_SIZE 			64		/* size of buffers */
#define MCCXTRABASE 			0x2000  /* offset from BASE_ADDR that marks the
							   			   beginning of the extra channel-specific
							   			   parameters for the MCC 
							   			   put it int the mcc2 chn specific parameter area*/
							   			 
#define TxBDSOFFSET				NUMBER_OF_CHANNELS*NUM_RXBDS
#define MCC_MRBLR 				BUFFER_SIZE +32					/* arbitrarily set the max receive 
									   							buffer length to be 32 bytes more
									   							than that of the expected buffer 
									   							length */
#define CHNINT2					CHNSTART+31
#define NUM_CHN_INT				1			/* number of channel abled to give interrupt*/

#define CRC16 					0xf0b8		/* mask for 16 bit crc */
#define CRC32 					0xdebb20e3	/* mask for 32 bit crc */
#define SUPERCHAN_TABLE 		0x3000	 	/* address offset for superchannel table */

#define CH64BDOFFSET 			0   		/* offsets used to index into correct areas of each */
#define CH65BDOFFSET 			8   		/* channel's BD and data areas */
#define CH66BDOFFSET 			16
#define CH67BDOFFSET 			24

#define HDLC_NOTSUPERCHAN 		0   		/* channel mode combinations */
#define HDLC_SUPERCHAN    		1
#define TRAN_NOTSUPERCHAN 		2
#define TRAN_SUPERCHAN    		3 

#define CPCR_RST                0x80000000
#define CPCR_FLG                0x00010000
#define CPCR_MCC1_PSBC          0x1f800000
#define CPCR_MCC2_PSBC          0x23A00000
#define CPCR_MCC_STOP_TX        0x00000004
#define CPCR_GRSTOP_TX          0x00000005
#define CPCR_MCC_STOP_RX        0x00000009
#define CPCR_INIT_TX_RX_PARAMS  0x00000000
#define CPCR_MCN_MCH0           0x00000000
#define READY_TO_RX_CMD         0  			/* Ready to receive a command */


/* INTERRUPT CONTROL */
#define SIMR_L_MCC1     		0x08000000
#define SIMR_L_MCC2     		0x04000000
#define SIVEC_MCC1	    		36
#define SIVEC_MCC2	    		37
#define VALID_ENTRY				0x80000000
#define WRAP_ENTRY				0x40000000
#define CHN_ENTRY				0x00003fc0
#define RINT0					0x4000
#define QOV0					0x8000
#define GUN						0x0002
#define GOV						0x0001
#define TINT					0x0004

/* GENERAL PURPOSE */
#define  ALL_ONES    			0xFFFFFFFF
#define  ALL_ZEROS   			0x00000000
#define BD_RX_ERROR 			0x002C    /* Mask for set of Receive Buffer Errors */

/*---------------------------------------------------------------------------------
 *								Global varible definition 
 *---------------------------------------------------------------------------------
 */

typedef	char		   	BYTE;
typedef unsigned char 	UBYTE;	
typedef	short		   	HWORD;	
typedef	unsigned short	UHWORD;	
typedef	long		   	WORD;	
typedef unsigned long 	UWORD;	
typedef unsigned char 	BOOL;	


typedef	volatile char		   	VBYTE;
typedef volatile unsigned char 	VUBYTE;	
typedef	volatile short		   	VHWORD;	
typedef	volatile unsigned short	VUHWORD;	
typedef	volatile long		   	VWORD;	
typedef volatile unsigned long 	VUWORD;	
typedef volatile unsigned char 	VBOOL;	


/* Buffer Descriptor structure
*/
typedef struct BufferDescriptor {
   UHWORD bd_cstatus;     		/* control and status */
   UHWORD bd_length;      		/* transfer length */
   unsigned int bd_addr;      	/* buffer address */
} BD;

/* Buffer descriptor ring
*/
typedef struct BufferDescRings {
    BD RxBD[NUMBER_OF_CHANNELS*NUM_RXBDS];    /* Rx BD ring */
    BD TxBD[NUMBER_OF_CHANNELS*NUM_TXBDS];    /* Tx BD ring */

} BDRINGS;

/* read & write system call data structure
*/
typedef struct _FrameBuffer {
    unsigned char * wp;   // write pointer
    unsigned char * rp;   // read pointer
    unsigned char  data[BUFFER_SIZE];
} FrameBuffer ;

/* constant output pattern
*/
#define SILENCE_PATTERN				0xD5
#define NULL_PATTERN				0xFF
#define SYNC_PATTERN				0x7E

/* memmory bus definition
*/
#define M_PPC 0
#define LOCAL 1

/* loal or 60x bus selection
*/
#define BD_BUS     M_PPC     /* IMPORTANT: Select what bus you will be placing */
#define BUFFER_BUS M_PPC    /* your BDs and data on for this example */

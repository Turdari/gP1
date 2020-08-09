/*
 * mccIOC.c - ioctl definition 
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
 
/*
 * -------------------------------------------------------------------------------- 
 * 			 					UTILITY                                                
 * --------------------------------------------------------------------------------
 *  
*/
#define MCC_IOC_MAGIC			            'M'
#define TRUE								1
#define FALSE								0
#define TX									1
#define RX									0

/*
 * -------------------------------------------------------------------------------- 
 * 			 					MCC BDS STATUS                                                
 * --------------------------------------------------------------------------------
 *  
*/
#define MCC_IOC_RXBDS		       		 	_IO(MCC_IOC_MAGIC,0)
#define MCC_IOC_TXBDS		       		 	_IO(MCC_IOC_MAGIC,1)
typedef struct {
   unsigned short bd_cstatus[NUM_RXBDS];
   unsigned short bd_length[NUM_RXBDS]; 
   unsigned int bd_addr[NUM_RXBDS];
   unsigned char chn;
} BDS_IOC;
/*
 * -------------------------------------------------------------------------------- 
 * 			 					SIxRAM STATUS                                               
 * --------------------------------------------------------------------------------
 *  
*/
#define SIXRAM_IOC_REGS						_IO(MCC_IOC_MAGIC,2)
typedef struct  {
	unsigned char  chn;					/* chn number */
	unsigned short tx_siram;			/* tx si ram entry */
	unsigned short rx_siram;			/* rx si ram entry */
	unsigned short sixmr;				/* SI TDM Mode Registers A-D */
    unsigned char  sigmr;       		/* SI Global Mode Register */
    unsigned char  sicmdr;      		/* SI Command Register */
    unsigned char  sistr;       		/* SI Status Register */
    unsigned short sirsr;       		/* SI RAM Shadow Address Register */
    unsigned short mcce;        		/* MCC Event Register */
    unsigned short mccm;        		/* MCC Mask Register */
    unsigned char  mccr;        		/* MCC Configuration Register */
} SI_REGS_IOC;
/*
 * -------------------------------------------------------------------------------- 
 * 			 					SIxRAM STATUS                                               
 * --------------------------------------------------------------------------------
 *  
*/
#define CPM_IOC_REGS						_IO(MCC_IOC_MAGIC,3)
typedef struct {
    unsigned int  cpm_cpcr;           	/* Communication Processor Command Register */
    unsigned int  cpm_rccr;           	/* RISC Configuration Register */
    unsigned int  cpm_rmdr;           	/* RISC Microcode Dev. Support Control Reg. */
    unsigned short cpm_rctr1;			/* RISC Controller Trap Register */
    unsigned short cpm_rctr2;          	/* RISC Controller Trap Register */
    unsigned short cpm_rctr3;          	/* RISC Controller Trap Register */
    unsigned short cpm_rctr4;          	/* RISC Controller Trap Register */
    unsigned short cpm_rter;           	/* RISC Timers Event Register */
    unsigned short cpm_rtmr;           	/* RISC Timers Mask Register */
    unsigned short cpm_rtscr;          	/* RISC Time-Stamp Timer Control Register */
    unsigned short cpm_rmds;           	/* RISC Development Support Status Register */
    unsigned int cpm_rtsr;           	/* RISC Time-Stamp Register */
} CPM_STATUS_IOC;
/*
 * -------------------------------------------------------------------------------- 
 * 			 					MCC TxRx BUFFER STATUS                                                
 * --------------------------------------------------------------------------------
 *  
*/
#define TXRX_IOC_BUFF						_IO(MCC_IOC_MAGIC,4)
typedef struct {
	unsigned char offset;
	unsigned char index;
	unsigned char TxBuff[BUFFER_SIZE];
	unsigned char RxBuff[BUFFER_SIZE];
} TXRX_BUFF_IOC;

/*
 * -------------------------------------------------------------------------------- 
 * 			 					FASYNC METHOD ACTIVATION                                             
 * --------------------------------------------------------------------------------
 *  
*/
#define MCC_FASYNC_IOC_DEAMON				_IO(MCC_IOC_MAGIC,5)
/*
 * -------------------------------------------------------------------------------- 
 * 			 					MCC RESTART COMMAND                                             
 * --------------------------------------------------------------------------------
 *  
*/
#define MCC_RESTART_CMD				_IO(MCC_IOC_MAGIC,6)
/*
 * -------------------------------------------------------------------------------- 
 * 			 					END                                                
 * --------------------------------------------------------------------------------
 *  
*/
#define MCC_IOC_MAXNR						6

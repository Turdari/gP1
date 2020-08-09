/*
 * device.c - mcc driver: bottom half side 
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
 * 
 * 			   
*/

#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif

#include <linux/kernel.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/param.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <asm/semaphore.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/time.h>
#include <linux/malloc.h>

#include "mcc.h"           /* Local header file */    
#include "mpc8260.h"       /* IMM definitions and declarations */
#include "mccIOC.h"


/*---------------------------------------------------------------------------------
 *								Global variables
 *---------------------------------------------------------------------------------
 */

volatile t_PQ2IMM *IMM;          /* Internal Memory Map base pointer */
VUBYTE SI;              /* Which SI is in use for this example */
VUBYTE MCC;             /* Which MCC is in use for this example */


/* read & write handshaking (non volevo farlo cosi' !)
*/
extern int writeNow[BUFFER_SIZE];
extern int valid[BUFFER_SIZE];
extern int readNow[BUFFER_SIZE];

/* buffer descriptors base pointer
*/
extern BDRINGS *RxTxBD;

/*	TxRx data buffer 
*/
extern unsigned char *BufferPool;

/* TxRx interrupt queue
*/
extern  unsigned int *txIntCQ;
extern  unsigned int *rxIntCQ;

/* spin lock variable
*/
extern spinlock_t lock;

/* data stream read and write buffer
*/
extern FrameBuffer *ReadBuffer;
extern FrameBuffer *WriteBuffer;

/*---------------------------------------------------------------------------------
 *								Function prototypes 
 *---------------------------------------------------------------------------------
 */
void    InitBDs(unsigned int,UBYTE);
void    MCCGlobalInit(void);
void    MCCChanSpecInit(UHWORD,UBYTE);
void    MCCExChanSpecInit(unsigned int,UHWORD,UHWORD);
void    InitTxRxParams(UHWORD);
void    SIinit(void);
void    startTxRx(void);
void    stopTxRx(void);
void 	mainInitMCC(unsigned int );
void 	mcc_ISR(int , void *, struct pt_regs *);
void 	BDsStatus(unsigned char, unsigned char, unsigned short *, unsigned short *, unsigned int *);
void 	SiRamStatus(unsigned char,unsigned char*,unsigned char*,unsigned char*,unsigned short*,
			unsigned short*,unsigned short*,unsigned char*,unsigned short*,unsigned short*,unsigned short*);
void 	CpmReadRegisters(CPM_STATUS_IOC *);
void 	getFrame(unsigned int, unsigned short);
void 	sendFrame(unsigned int, unsigned short);
void 	restoreBDs(unsigned short);
void 	RestartServiceRoutine(void);
void 	stopCpmMcc(unsigned char mcn);
extern void mccFasync(void);
/*---------------------------------------------------------------------------------
 *								Functions 
 *---------------------------------------------------------------------------------
 */
 
/*
 * Name:
 *	mainInitMCC
 * Description:
 *	main MCC and SI init 
*/
void mainInitMCC(unsigned int BASE_ADDR){
	unsigned int chnOffset,chnNum;
		
	/* work with MCC1,SI1 and TDMC
	*/
    SI = SI1;
    MCC = MCC1;               
    //TDM = TDMC;


	/* pointer to MPC8260 internal memory map.
	*/
    IMM = (t_PQ2IMM *)(BASE_ADDR);



	/*Initialize SI RAM : TDM1C and TDM1D routing
	*/
    SIinit();


	/*Initialize TxRxBDs
	*/
	chnOffset = 0;
	for(chnNum = CHNSTART;chnNum < (CHNSTART+NUMBER_OF_CHANNELS);chnNum++){
		InitBDs(chnOffset,TRAN_NOTSUPERCHAN);
		chnOffset+=NUM_RXBDS;	
	}
	
	
	/*Initialize Global parameters
	*/
    MCCGlobalInit();

 	
	/*Initialize Extra Channel Specific parameters
	*/
	chnOffset = 0;
	for(chnNum = CHNSTART;chnNum < (CHNSTART+NUMBER_OF_CHANNELS);chnNum++){
		MCCExChanSpecInit(chnNum,chnOffset+TxBDSOFFSET,chnOffset);
		chnOffset+=NUM_RXBDS;	
	}

    /* Initialize Channel Specific parameters
    */
     for(chnNum = CHNSTART;chnNum < (CHNSTART+NUMBER_OF_CHANNELS);chnNum++){
    	MCCChanSpecInit(chnNum,TRAN_NOTSUPERCHAN);
	}
} /* mainInitMCC */

/*
 * Name:
 *	startTxRx
 * Description:
 *	start TDM 
*/
void startTxRx(){
	/* Issue Init RX & TX Parameters Command for MCC1
	*/
    InitTxRxParams(64);
    InitTxRxParams(96);
 

    /* Enable TDM1C and TDM1D
    */
	IMM->si_regs[SI].sigmr = 0x0c;
 

}

/*
 * Name:
 *	stopTxRx
 * Description:
 *	stop TDM 
*/
void stopTxRx(){
	unsigned char mcn;
	/* Disable TDM1C and TDM1D
    */
	IMM->si_regs[SI].sigmr = 0x00;
	/* stop CPM
	*/
	for(mcn = CHNSTART; mcn<(CHNSTART+NUMBER_OF_CHANNELS);mcn++)
 		stopCpmMcc(mcn);
} /* end of stopTxRx */

/*
 * Name:
 *	stopCpmMcc
 * Description:
 *	stop MCC  
*/
void stopCpmMcc(unsigned char mcn){
    volatile unsigned int psbc;
    
    if (MCC == MCC1) 
    	psbc = CPCR_MCC1_PSBC;
    else
    psbc = CPCR_MCC2_PSBC;
   	while ((IMM->cpm_cpcr & CPCR_FLG) != READY_TO_RX_CMD); 
    IMM->cpm_cpcr = CPCR_MCC_STOP_TX |
                 psbc |  
                 (mcn<<6)|
                 CPCR_FLG;              /* ISSUE COMMAND */

    while ((IMM->cpm_cpcr & CPCR_FLG) != READY_TO_RX_CMD);
	 	
    while ((IMM->cpm_cpcr & CPCR_FLG) != READY_TO_RX_CMD); 
    IMM->cpm_cpcr = CPCR_MCC_STOP_RX |
                 psbc |  
                 (mcn<<6)|
                 CPCR_FLG;              /* ISSUE COMMAND */

    while ((IMM->cpm_cpcr & CPCR_FLG) != READY_TO_RX_CMD);
}
/*
 * Name:
 *	SIinit
 * Description:
 *	SI ram entries configuration 
*/
void SIinit(){
	unsigned short chnNum;
	unsigned int count;
	
	chnNum = CHNSTART; /*chnannel 64 */
	
	/* SI1 entries for TDM1C
	*/
	for(count=0;count < 32;count++){
		IMM->si_ram[SI].tx_siram[count] = 0x8002 | (chnNum << 5); /* see page 14-13*/
		IMM->si_ram[SI].rx_siram[count] = 0x8002 | (chnNum << 5);
		if(count == 31){
			IMM->si_ram[SI].tx_siram[count] = 0x8003 | (chnNum << 5); /* see page 14-13*/
			IMM->si_ram[SI].rx_siram[count] = 0x8003 | (chnNum << 5);
		}
		chnNum++;		
    }
	/* SI1 entries for TDM1D
	*/
	for(count=0;count < 32;count++){
		IMM->si_ram[SI].tx_siram[count+64] = 0x8002 | (chnNum << 5); /* see page 14-13*/
		IMM->si_ram[SI].rx_siram[count+64] = 0x8002 | (chnNum << 5);
		if(count == 31){
			IMM->si_ram[SI].tx_siram[count+64] = 0x8003 | (chnNum << 5); /* see page 14-13*/
			IMM->si_ram[SI].rx_siram[count+64] = 0x8003 | (chnNum << 5);
		}
		chnNum++;		
    }


 
    IMM->si_regs[SI].sixmr[TDMC] = 0x0040;  /* 0 offset for SIRAM 1cond bank 1st 32 entries, CRT */
    IMM->si_regs[SI].sixmr[TDMD] = 0x2040;  /* 64 offset for SIRAM 2cond bank 1st 32 entries, CRT */

} /* SIinit */

/*
 * Name:
 *	InitBDs
 * Description:
 *	Init Tx and Rx BDs 
*/
void InitBDs(unsigned int offset, UBYTE mode){
   
unsigned int index;
unsigned int addr;

   	/* Initialize RxBDs
   	*/
	for (index = 0; index < NUM_RXBDS; index++){
		if( index != (NUM_RXBDS-1) ){/* If not the last RxBD for this channel */
    		RxTxBD->RxBD[index+offset].bd_cstatus = 0x9000; /* Empty, Interrupt */ /* old 0x9000 */
		} else {/* if last RxBD for this channel */
        	RxTxBD->RxBD[index+offset].bd_cstatus = 0xB000; /* Empty, Interrupt,Wrap */
    	}
      	RxTxBD->RxBD[index+offset].bd_length = 0;/* clear the buffer length */
      	/*address of receive area */
      	addr = (unsigned int )BufferPool + BUFFER_SIZE*NUMBER_OF_CHANNELS*NUM_TXBDS + (offset+index)*BUFFER_SIZE;
      	RxTxBD->RxBD[index+offset].bd_addr =(unsigned int )( addr - (unsigned int )PAGE_OFFSET );
	}/* end for loop initializing RxBDs */
 
	/* Initialize TxBDs
	*/
	for (index=0; index < NUM_TXBDS; index++){
		if( index != (NUM_TXBDS-1) ) { /* If not the last TxBD for this channel */
        	RxTxBD->TxBD[index+offset].bd_cstatus = 0x8000; /* Set Ready bit */
         
         	/*----------------------------------------------------*/
         	/* If this channel is HDLC, also set Last and TC bits */
         	/*----------------------------------------------------*/
         
         	/*if ( (mode == HDLC_NOTSUPERCHAN) || (mode == HDLC_SUPERCHAN) )
            	RxTxBD->TxBD[index+offset].bd_cstatus |= 0x0C00;     */         
	
		} else { /* if last TxBD for this channel */
			RxTxBD->TxBD[index+offset].bd_cstatus = 0xA000;/* Set Ready, Wrap bits */

         /*----------------------------------------------------*/
         /* If this channel is HDLC, also set Last and TC bits */
         /*----------------------------------------------------*/
         
         /*if ( (mode == HDLC_NOTSUPERCHAN) || (mode == HDLC_SUPERCHAN) )
            RxTxBD->TxBD[index+offset].bd_cstatus |= 0x0C00;*/     
      	}
      
      /* load the buffer length 
      */
	  	
      //if ( (mode == HDLC_NOTSUPERCHAN) || (mode == HDLC_SUPERCHAN) )
            /* if this channel is HDLC */
      //{
      //   RxTxBD->TxBD[index+offset].bd_length = (BUFFER_SIZE-4);  
      //}
      
     // else  /* Transparent */
      
      RxTxBD->TxBD[index+offset].bd_length = (BUFFER_SIZE);/* set buffer length */
      addr = ( (unsigned int ) BufferPool+(offset+index)*BUFFER_SIZE);
      RxTxBD->TxBD[index+offset].bd_addr = (unsigned int )( addr - (unsigned int )PAGE_OFFSET);/* set address to point tx area */
      
   }
} /* end InitBDs */

/*
 * Name:
 *	MCCGlobalInit
 * Description:
 *	configure MCC global parameter 
*/
void MCCGlobalInit(){

	unsigned int addr;
/* set up abbreviated pointer */
t_Mcc_Pram* MCCglobal;


	MCCglobal = (t_Mcc_Pram * ) &(IMM->pram.serials.mcc_pram[MCC]); 
      

  	addr = (unsigned int ) RxTxBD;
  
    MCCglobal->mccbase  = addr - PAGE_OFFSET;/* base of BDs used by this MCC */
    MCCglobal->mccstate = ALL_ZEROS; 
    MCCglobal->mrblr    = MCC_MRBLR;   /* max Rx buffer length */
    MCCglobal->grfthr   = NUMBER_OF_CHANNELS*NUM_RXBDS;
                                        
    MCCglobal->grfcnt   = MCCglobal->grfthr; /* grfthr decrementer */
    MCCglobal->rinttmp  = ALL_ZEROS;
    MCCglobal->data0    = ALL_ZEROS;
    MCCglobal->data1    = ALL_ZEROS;
    
    addr = (unsigned int) txIntCQ;
    
    MCCglobal->tintbase = addr - PAGE_OFFSET;    /* base of Tx int. circular queue */
    MCCglobal->tintptr  = MCCglobal->tintbase;  /* temp ptr for tx interrupts*/
    MCCglobal->tinttmp  = ALL_ZEROS;    /* must be cleared by user */    
    MCCglobal->sctpbase = (unsigned int )NULL; 	  /*base of superchannel table*/ 
    MCCglobal->c_mask32 = CRC32;        /* 32 bit crc constant */
    MCCglobal->xtrabase = MCCXTRABASE; /*base of chan-specific xtra params */
    MCCglobal->c_mask16 = CRC16;        /* 16 bit crc constant */

    /* Note that we are using only 1 one of the four possible Rx int. queues */
    MCCglobal->rinttmp0 = ALL_ZEROS;    /* must be cleared by user */
    MCCglobal->rinttmp1 = ALL_ZEROS;    /* must be cleared by user */
    MCCglobal->rinttmp2 = ALL_ZEROS;    /* must be cleared by user */
    MCCglobal->rinttmp3 = ALL_ZEROS;    /* must be cleared by user */
    
    addr = (unsigned int )rxIntCQ; 
    
    MCCglobal->rintbase0 = addr - PAGE_OFFSET;  /* base of Rx int. circular queue #0*/
    MCCglobal->rintptr0 = MCCglobal->rintbase0;   /* current RxINTCQ0 ptr */
    MCCglobal->rintbase1 = ALL_ZEROS;   /* base of Rx int. circular queue #1*/
    MCCglobal->rintptr1 = MCCglobal->rintbase1;   /* current RxINTCQ1 ptr */
    MCCglobal->rintbase2 = ALL_ZEROS;   /* base of Rx int. circular queue #2*/
    MCCglobal->rintptr2 = MCCglobal->rintbase2;   /* current RxINTCQ2 ptr */
    MCCglobal->rintbase3 = ALL_ZEROS;   /* base of Rx int. circular queue #3*/
    MCCglobal->rintptr3 = MCCglobal->rintbase3;   /* current RxINTCQ3 ptr */
    
    

	IMM->si_regs[MCC].mccr = 0x1b; /* gruppo 1 a TDMA1,
									  gruppo 2 a TDMB1
									  gruppo 3 a TDMC1
									  gruppo 4 a TDMD1	*/ 
    
       
    /* Enable all interrupts 
    */
    IMM->si_regs[MCC].mccm = 0xff0f ;
    

    /* Clear MCCE Register by writing all 1's 
    */
    IMM->si_regs[MCC].mcce = (unsigned short )ALL_ONES;

} /* end MCCGlobalInit() */

/*
 * Name:
 *	MCCChanSpecInit
 * Description:
 *	configure MCC channel specific parameter 
*/
void MCCChanSpecInit(unsigned short mcn, unsigned char   mode){

	t_Mch_Pram* MCCmch;

	MCCmch =  (t_Mch_Pram*) ((unsigned int )IMM + (64 * ((unsigned int )  mcn)));

        
    /* Channel-Specific Parameter RAM Initialization 
    */
	MCCmch->zistate = 0x10000207;	/* regular channel */
	MCCmch->zidata0 = ALL_ONES;
	MCCmch->zidata1 = ALL_ONES;
    MCCmch->tbdflags = ALL_ZEROS;
    MCCmch->tbdcnt  = ALL_ZEROS;
    MCCmch->tbdptr = ALL_ZEROS;

	if (mcn == CHNINT2 )
		MCCmch->intmask = 0x0001;		/* enable interrupts for RXB */
    else
		MCCmch->intmask = 0x0000;		/* disable interrupts for RXB */
    
    if ((mode==HDLC_SUPERCHAN) || (mode==HDLC_NOTSUPERCHAN)){  /* HDLC*/
        MCCmch->chamr = 0xE480;     /* set MODE to HDLC, POL, CRC, Reversed bit ordering*/
        MCCmch->maxrlen = MCC_MRBLR;	/* HDLC max frame receive length */
        MCCmch->zdstate = 0x00FFFFE0;
    } else { /* TRANSPARENT */    
        MCCmch->chamr = 0x7400;     /* set MODE to TRAN, POL */
        MCCmch->maxrlen = BUFFER_SIZE;  /* in transparent, this should be
                                           to exactly the amount of data
                                           desired to fill the BD */
        MCCmch->zdstate = 0x50ffffe0;	/* changed for ucode patch for non-sync tran. */
    };
    
	MCCmch->tcrc = ALL_ZEROS;
	MCCmch->zddata0 = ALL_ONES;
	MCCmch->zddata1 = ALL_ONES;
    MCCmch->rbdflags = ALL_ZEROS;
    MCCmch->rbdcnt  = ALL_ZEROS;
    MCCmch->rbdptr = ALL_ZEROS;
    MCCmch->sync_maxcnt = ALL_ZEROS;
    MCCmch->rcrc  = ALL_ZEROS;
    
    if ((mode==HDLC_NOTSUPERCHAN) || (mode==TRAN_NOTSUPERCHAN)){
        MCCmch->tstate = 0x38800000;	/* global, big endian*/
        MCCmch->rstate = 0x38800000;	/* global, big endian*/
        if (BD_BUS == LOCAL) {
            MCCmch->tstate |= 0x01000000;	/* set BDB */
            MCCmch->rstate |= 0x01000000;	/* set BDB */
        }
    
        if (BUFFER_BUS == LOCAL) {
            MCCmch->tstate |= 0x02000000;	/* set DTB */
            MCCmch->rstate |= 0x02000000;	/* set DTB */
        }
    } else {     /* initialize states for superchannelling (to be set in StartChannel) */
    
        MCCmch->tstate = ALL_ZEROS;	
        MCCmch->rstate = ALL_ZEROS;	
    };
    
} /* end MCCChanSpecInit() */

/*
 * Name:
 *	MCCExChanSpecInit
 * Description:
 *	configure MCC channel extra specific parameter 
*/
void MCCExChanSpecInit(unsigned int mcn, unsigned short mctbase, unsigned short mcrbase) {


	t_MchXtra_Pram* MCCmchx;

	MCCmchx=(t_MchXtra_Pram * ) ((unsigned int )IMM  + MCCXTRABASE + (8 * mcn));


    /*-----------------------------------------------------------------*/
    /* Let MCC1 know where base of TxBDs are for this logical channel. */
    /* Note that tbase is an offset from MCCBASE in the global params. */
    /*-----------------------------------------------------------------*/
    
    MCCmchx->tbase = mctbase;
	MCCmchx->tbptr = MCCmchx->tbase;		/* init "current TxBD ptr" */


    /*-----------------------------------------------------------------*/
    /* Let MCC1 know where base of RxBDs are for this logical channel  */
    /* Note that rbase is an offset from MCCBASE in the global params. */
    /*-----------------------------------------------------------------*/
    
    MCCmchx->rbase = mcrbase;
	MCCmchx->rbptr = MCCmchx->rbase;		/* init "current RxBD ptr" */

	
} /* MCCExChanSpecInit */

/*
 * Name:
 *	InitTxRxParams
 * Description:
 *	Issues the INIT_TX_RX_PARAMS command to the CPCR for the given mcn
 * NOTE:
 *	issuing an init params command to an MCC channel  
 * 	actually initializes the 32 consecutive channels starting with  
 * 	channel number specified.
*/
void InitTxRxParams (unsigned short mcn) {
    volatile unsigned int psbc;

    if (MCC == MCC1) 
    	psbc = CPCR_MCC1_PSBC;
    else
    psbc = CPCR_MCC2_PSBC;

    while ((IMM->cpm_cpcr & CPCR_FLG) != READY_TO_RX_CMD); 

    IMM->cpm_cpcr = CPCR_INIT_TX_RX_PARAMS |
                 psbc |  
                 (mcn<<6)|
                 CPCR_FLG;              /* ISSUE COMMAND */

    while ((IMM->cpm_cpcr & CPCR_FLG) != READY_TO_RX_CMD);
    
} /* InitTxRxParams */

/*
 * Name:
 *	getFrame
 * Description:
 *	copy read data from RxBDs into the read buffer
 * Input parameter:
 *	chn:channel number, 
 *	offset:chn offset in BufferPool area, 
 *	index: BD identifier
 */
void getFrame(unsigned int chn, unsigned short offset) {
	unsigned int index=0;
    unsigned char *rx;
    FrameBuffer *read = (FrameBuffer *) ((unsigned int )ReadBuffer + (sizeof(FrameBuffer))*chn);
    unsigned int i;
	while(index<NUM_RXBDS){
		if( (RxTxBD->RxBD[index+offset].bd_cstatus & 0x8000 )== 0) /* BD filled */
    		rx = BufferPool + BUFFER_SIZE*NUMBER_OF_CHANNELS*NUM_TXBDS + (offset+index)*BUFFER_SIZE;
    	index++;
    }
	for(i = 0; i < BUFFER_SIZE; i++){
	    read->data[i] = (unsigned char) *rx++; 
    }
    readNow[chn] = 1;
} /* getFrame */


/*
 * Name:
 *	sendFrame
 * Description:
 *	copy from write buffer into the TxBDs 
 * Input parameter:
 *	chn:channel number, 
 *	offset:chn offset in BufferPool area, 
 *	index: BD identifier
 */
void sendFrame(unsigned int chn,unsigned short offset) {
	unsigned int index=0;
    unsigned char *tx;
    unsigned short i;
    FrameBuffer *write = (FrameBuffer *) ( (unsigned int ) WriteBuffer + (sizeof(FrameBuffer))*chn);
	while (index<NUM_TXBDS){
		if( (RxTxBD->TxBD[index+offset].bd_cstatus & 0x8000 )== 0) /* BD serviced */
    		tx = BufferPool+((offset+index)*BUFFER_SIZE);
    	index++;
    }
	if ( valid[chn] == 0){
        for (i = 0; i < BUFFER_SIZE; i++) {
            *tx++ = ((chn == 0) || (chn == 32) )? SYNC_PATTERN : SILENCE_PATTERN;
        }
	} else {
	    for (i = 0; i < BUFFER_SIZE; i++) {
    	    *tx++ = (unsigned char) write->data[i];
    	}
    	valid[chn] = 0;
	}
	writeNow[chn] = 1;

} /* end of sendFrame */

/*
 * Name:
 *	restoreBDs
 * Description:
 * 	reinitilize BDs	
 */
void restoreBDs(unsigned short offset){
	unsigned int index=0;
	while (index<NUM_RXBDS){
		if( index != (NUM_RXBDS-1) ){/* If not the last RxBD for this channel */
			if( (RxTxBD->RxBD[index+offset].bd_cstatus & 0x8000 )== 0) /* BD filled */
 		   		RxTxBD->RxBD[index+offset].bd_cstatus = 0x9000; /* Empty, No Interrupt */ /* old 0x9000 */
        		RxTxBD->TxBD[index+offset].bd_cstatus = 0x8000; /* Set Ready bit */

		} else {/* if last RxBD for this channel */
			if( (RxTxBD->RxBD[index+offset].bd_cstatus & 0x8000 )== 0) /* BD filled */
	        	RxTxBD->RxBD[index+offset].bd_cstatus = 0xB000; /* Empty, Interrupt,Wrap */
				RxTxBD->TxBD[index+offset].bd_cstatus = 0xA000;/* Set Ready, Wrap bits */
    	}
		index++;
	}
}
/*
 * Name:
 *	mcc_ISR
 * Description:
 *	MCC Interrupt handler
 *	a new interrupt comes when 32°channel buffer has been recieved
 */
void mcc_ISR(int irq, void *dev_id, struct pt_regs *regs) {
	unsigned int mcce;
	unsigned int chnOffset;
	unsigned int chnNum;
	/* lock intetruupt handler
	*/
	spin_lock_irq(&lock);
   	/* Copy the MCC event register 
   	*/
	mcce = IMM->si_regs[MCC].mcce; /* Save off scce */
   	/* Clear MCC1 Event Register (by writing 1s) 
   	*/
   	IMM->si_regs[MCC].mcce = (unsigned short )ALL_ONES;
		
	if (mcce & RINT0){/* at last one new entry was added in the Rx interrupt table */
			chnOffset = 0;
			for(chnNum = 0; chnNum < (NUMBER_OF_CHANNELS); chnNum++){
				getFrame(chnNum,chnOffset);
				sendFrame(chnNum,chnOffset);
				restoreBDs(chnOffset);
				chnOffset+=NUM_RXBDS;	
			}
    		memset((unsigned char  *)&rxIntCQ[0], 0,NUM_CHN_INT*NUM_RXBDS*4);
    		*((unsigned int  *)&rxIntCQ[0] + NUM_CHN_INT*NUM_RXBDS - 1) = 0x40000000; /* set wrap bit at end of RX intcq */
	} else {
			/* may be underrun, fatal error
			*/
			printk("MCC:interrupt error:0x%x\n",mcce);

			/* Restart the driver
			RestartServiceRoutine();*/

			/* asyncronous notification 
			*/
			mccFasync();
	}
	/* unlock intetruupt handler
	*/
	spin_unlock_irq(&lock);
	/* Enable RINT0 interrupts 
	*/
	IMM->si_regs[MCC].mccm =0xff0f; /* all source interrupt abled, see page 27-19*/

} /* end of mcc_ISR */


/*
 * Name:
 *	BDsStatus
 * Description:
 *	read TxRxBDs satus 
 */
void BDsStatus(unsigned char TxRx, unsigned char offset, unsigned short *bd_status, unsigned short *bd_length, unsigned int  *bd_addr){
	unsigned char index;
	if (TxRx == TX){
		for (index=0;index<NUM_TXBDS;index++){
        	bd_status[index]=RxTxBD->TxBD[index+offset].bd_cstatus;
        	bd_length[index]=RxTxBD->TxBD[index+offset].bd_length;
        	bd_addr[index]=RxTxBD->TxBD[index+offset].bd_addr;
        }
	} else {
		for (index=0;index<NUM_RXBDS;index++){
        	bd_status[index]=RxTxBD->RxBD[index+offset].bd_cstatus;
        	bd_length[index]=RxTxBD->RxBD[index+offset].bd_length;
        	bd_addr[index]=RxTxBD->RxBD[index+offset].bd_addr;
        }
	}
}/* BDsStatus end */
/*
 * Name:
 *	SiRamStatus
 * Description:
 *	read Si ram satus 
 */
void SiRamStatus(unsigned char chn,unsigned char *sigmr,unsigned char *sicmdr,
					unsigned char *sistr,unsigned short *sirsr,unsigned short *mcce,
					unsigned short *mccm,unsigned char *mccr,unsigned short *sixmr,
					unsigned short *tx_siram,unsigned short *rx_siram){
	*sigmr=IMM->si_regs[SI].sigmr;
	*sicmdr=IMM->si_regs[SI].sicmdr;
	*sistr=IMM->si_regs[SI].sistr;
	*sirsr=IMM->si_regs[SI].sirsr;
	*mcce=IMM->si_regs[SI].mcce;
	*mccm=IMM->si_regs[SI].mccm;
	*mccr=IMM->si_regs[SI].mccr;
	*sixmr=IMM->si_regs[SI].sixmr[(chn<32)?TDMC:TDMD];
	*tx_siram=IMM->si_ram[SI].tx_siram[(chn<32)?chn:(chn+32)];
	*rx_siram=IMM->si_ram[SI].rx_siram[(chn<32)?chn:(chn+32)];

}/* SiRamStatus end*/
   
/*
 * Name:
 *	CpmReadRegisters
 * Description:
 *	read CPM register 
 */
void CpmReadRegisters(CPM_STATUS_IOC *CpmStatusRegisterIOCTL){
	CpmStatusRegisterIOCTL->cpm_cpcr=IMM->cpm_cpcr;
	CpmStatusRegisterIOCTL->cpm_rccr=IMM->cpm_rccr;
	CpmStatusRegisterIOCTL->cpm_rmdr=IMM->cpm_rmdr;
	CpmStatusRegisterIOCTL->cpm_rctr1=IMM->cpm_rctr1;
	CpmStatusRegisterIOCTL->cpm_rctr2=IMM->cpm_rctr2;
	CpmStatusRegisterIOCTL->cpm_rctr3=IMM->cpm_rctr3;
	CpmStatusRegisterIOCTL->cpm_rctr3=IMM->cpm_rctr3;
	CpmStatusRegisterIOCTL->cpm_rter=IMM->cpm_rter;
	CpmStatusRegisterIOCTL->cpm_rtmr=IMM->cpm_rtmr;
	CpmStatusRegisterIOCTL->cpm_rtscr=IMM->cpm_rtscr;
	CpmStatusRegisterIOCTL->cpm_rmds=IMM->cpm_rmds;
	CpmStatusRegisterIOCTL->cpm_rtsr=IMM->cpm_rtsr;
} /* CpmReadRegisters end  */
/*
 * Name:
 *	RestartServiceRoutine
 * Description:
 *	restart of mcc after a fatal error
 */
void RestartServiceRoutine(void){
	unsigned char chnOffset,chnNum;
	/* reinizilize the mcc.
	*/
	stopTxRx();
	/*Reinitialize TxRxBDs
	*/
	chnOffset = 0;
	for(chnNum = CHNSTART;chnNum < (CHNSTART+NUMBER_OF_CHANNELS);chnNum++){
		InitBDs(chnOffset,TRAN_NOTSUPERCHAN);
		chnOffset+=NUM_RXBDS;	
	}	
	startTxRx();		
	return;
} /* end of RestartServiceRoutine */

/*    
 *
 * File:  MPC8260.H
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
 * Description:  
 *
 * Main Internal Memory Maps for the MPC8260 PowerQUICC 2.
 *
 * Note that different structures can be overlaid at the same offsets for the 
 * different modes of operation.
 *
 *
 * History:
 *
 * 13 JUL 98    ggh    initial version
 * 15 JAN 99    ggh    updated MCC structures
 *
 *
*/

#ifndef _MPC8260_H 
#define _MPC8260_H

//#define _Packed __packed__
#define _PackedType
 

/******************************************************************************
 *
 *  PARAMETER RAM (PRAM) FOR EACH PERIPHERAL
 *  
 *  Each subsection contains protocol-specific PRAM for each peripheral,
 *  followed by the PRAM common to all protocols for that peripheral.  These 
 *  structs are used as needed in the main MPC8260 memory map structure.  Note 
 *  that different modes of operation will require the use of different PRAM 
 *  structs, and that certain structs may overlay and conflict with the use of 
 *  other PRAM areas.  Consult the MPC8260 VUser Manual for details as to what
 *  is unavailable when certain protocols are run on certain peripherals.
 *
 ******************************************************************************/
                                  

/*---------------------------------------------------------------------------*/
/*                   SERIAL COMMUNICATION CONTROLLER (SCC)                   */
/*---------------------------------------------------------------------------*/

/*----------*/
/* SCC HDLC */
/*----------*/

typedef  struct           /* 44 bytes */ 
{
  VUBYTE  reserved1[4]; /* Reserved area */
  VUWORD  c_mask;       /* CRC constant */
  VUWORD  c_pres;       /* CRC preset */
  VUHWORD disfc;        /* discarded frame counter */
  VUHWORD crcec;        /* CRC error counter */
  VUHWORD abtsc;        /* abort sequence counter */
  VUHWORD nmarc;        /* nonmatching address rx cnt */
  VUHWORD retrc;        /* frame transmission counter. */
  /* For FCC this area is reserved.*/
  VUHWORD mflr;         /* maximum frame length reg */
  VUHWORD max_cnt;      /* maximum length counter */
  VUHWORD rfthr;        /* received frames threshold */
  VUHWORD rfcnt;        /* received frames count */
  VUHWORD hmask;        /* user defined frm addr mask */
  VUHWORD haddr1;       /* user defined frm address 1 */
  VUHWORD haddr2;       /* user defined frm address 2 */
  VUHWORD haddr3;       /* user defined frm address 3 */
  VUHWORD haddr4;       /* user defined frm address 4 */
  VUHWORD tmp;          /* temp */
  VUHWORD tmp_mb;       /* temp */
}/*_PackedType */ t_HdlcScc_Pram;

 
/*--------------*/
/* SCC Ethernet */
/*--------------*/

typedef  struct          /* 116 bytes */ 
{
  VUWORD  c_pres;      /* CRC preset */
  VUWORD  c_mask;      /* CRC constant mask*/
  VUWORD  crcec;       /* CRC error counter */
  VUWORD  alec;        /* alignment error counter */
  VUWORD  disfc;       /* discarded frame counter */
  VUHWORD pads;        /* Short frame pad character. */
  VUHWORD ret_lim;     /* Retry limit threshold. */
  VUHWORD ret_cnt;     /* Retry limit counter. */
  VUHWORD mflr;        /* maximum frame length reg */
  VUHWORD minflr;      /* minimum frame length reg */
  VUHWORD maxd1;       /* max DMA1 length register. */
  VUHWORD maxd2;       /* max DMA2 length register. */
  VUHWORD maxd;        /* Rx max DMA. */
  VUHWORD dma_cnt;     /* Rx DMA counter. */
  VUHWORD max_b;       /* max buffer descriptor byte count. */
  VUHWORD gaddr1;      /* group address filter */
  VUHWORD gaddr2;      /* group address filter */
  VUHWORD gaddr3;      /* group address filter */
  VUHWORD gaddr4;      /* group address filter */
  VUWORD  tbuf0_data0; /* Saved area 0, current frame. */
  VUWORD  tbuf0_data1; /* Saved area 1, current frame. */
  VUWORD  tbuf0_rba0;
  VUWORD  tbuf0_crc;
  VUHWORD tbuf0_bcnt;
  VUHWORD paddr1_h;    /* physical address (MSB) */
  VUHWORD paddr1_m;    /* physical address */
  VUHWORD paddr1_l;    /* physical address (LSB) */
  VUHWORD p_per;       /* persistence */
  VUHWORD rfbd_ptr;    /* Rx first BD pointer. */
  VUHWORD tfbd_ptr;    /* Tx first BD pointer. */
  VUHWORD tlbd_ptr;    /* Tx last BD pointer. */
  VUWORD  tbuf1_data0; /* Saved area 0, next frame. */
  VUWORD  tbuf1_data1; /* Saved area 1, next frame. */
  VUWORD  tbuf1_rba0;
  VUWORD  tbuf1_crc;
  VUHWORD tbuf1_bcnt;
  VUHWORD tx_len;      /* tx frame length counter */
  VUHWORD iaddr1;      /* individual address filter. */
  VUHWORD iaddr2;      /* individual address filter.  */
  VUHWORD iaddr3;      /* individual address filter. */
  VUHWORD iaddr4;      /* individual address filter.  */
  VUHWORD boff_cnt;    /* back-off counter */
  VUHWORD taddr_h;     /* temp address (MSB) */
  VUHWORD taddr_m;     /* temp address */
  VUHWORD taddr_l;     /* temp address (LSB) */
}  /* _PackedType */ t_EnetScc_Pram;

/*----------*/
/* SCC VUART */
/*----------*/

typedef  struct             /* 54 bytes */ 
{
  VUBYTE  reserved1[8];   /* Reserved area */
  VUHWORD max_idl;        /* maximum idle characters */
  VUHWORD idlc;           /* rx idle counter (internal) */
  VUHWORD brkcr;          /* break count register */
  VUHWORD parec;          /* Rx parity error counter */
  VUHWORD frmec;          /* Rx framing error counter */
  VUHWORD nosec;          /* Rx noise counter */
  VUHWORD brkec;          /* Rx break character counter */
  VUHWORD brkln;          /* Receive break length */
  VUHWORD uaddr1;         /* address character 1 */
  VUHWORD uaddr2;         /* address character 2 */
  VUHWORD rtemp;          /* temp storage */
  VUHWORD toseq;          /* Tx out of sequence char */
  VUHWORD cc[8];          /* Rx control characters */
  VUHWORD rccm;           /* Rx control char mask */
  VUHWORD rccr;           /* Rx control char register */
  VUHWORD rlbc;           /* Receive last break char */
}  /* _PackedType */ t_UartScc_Pram;


/*-----------------*/
/* SCC Transparent */
/*-----------------*/

typedef  struct             /* 8 bytes */
{
  VUWORD  c_mask;         /* CRC constant */
  VUWORD  c_pres;         /* CRC preset */
}  /* _PackedType */ t_TransScc_Pram;


/*------------*/
/* SCC Bisync */
/*------------*/

typedef  struct             /* 36 bytes */
{
  VUBYTE  reserved1[4];   /* Reserved area */
  VUWORD  crcc;           /* CRC Constant Temp Value */
  VUHWORD prcrc;          /* Preset Receiver CRC-16/LRC */
  VUHWORD ptcrc;          /* Preset Transmitter CRC-16/LRC */
  VUHWORD parec;          /* Receive Parity Error Counter */
  VUHWORD bsync;          /* BISYNC SYNC Character */
  VUHWORD bdle;           /* BISYNC DLE Character */
  VUHWORD cc[8];          /* Rx control characters */
  VUHWORD rccm;           /* Receive Control Character Mask */
}  /* _PackedType */ t_BisyncScc_Pram;


/*-----------------*/
/* SCC Common PRAM */
/*-----------------*/

typedef  struct         /* 398 bytes */
{
  VUHWORD rbase;      /* RX BD base address */
  VUHWORD tbase;      /* TX BD base address */
  VUBYTE  rfcr;       /* Rx function code */
  VUBYTE  tfcr;       /* Tx function code */
  VUHWORD mrblr;      /* Rx buffer length */
  VUWORD  rstate;     /* Rx internal state */
  VUWORD  rptr;       /* Rx internal data pointer */
  VUHWORD rbptr;      /* rb BD Pointer */
  VUHWORD rcount;     /* Rx internal byte count */
  VUWORD  rtemp;      /* Rx temp */
  VUWORD  tstate;     /* Tx internal state */
  VUWORD  tptr;       /* Tx internal data pointer */
  VUHWORD tbptr;      /* Tx BD pointer */
  VUHWORD tcount;     /* Tx byte count */
  VUWORD  ttemp;      /* Tx temp */
  VUWORD  rcrc;       /* temp receive CRC */
  VUWORD  tcrc;       /* temp transmit CRC */
  union 
  {
    t_HdlcScc_Pram    h;
    t_EnetScc_Pram    e;
    t_UartScc_Pram    u;
    t_TransScc_Pram   t;
    t_BisyncScc_Pram  b;
  } SpecificProtocol;
  VUBYTE COMPLETE_SIZE_OF_DPRAM_PAGE[0x5c];
}  /* _PackedType */ t_Scc_Pram;



/*---------------------------------------------------------------------------*/
/*                      FAST COMMUNICATION CONTROLLER (FCC)                  */
/*---------------------------------------------------------------------------*/

/*----------*/
/* FCC HDLC */
/*----------*/

typedef  struct             /* 48 bytes */
{
  VUBYTE  reserved1[8];   /* Reserved area */
  VUWORD  c_mask;         /* CRC constant */
  VUWORD  c_pres;         /* CRC preset */
  VUHWORD disfc;          /* discarded frame counter */
  VUHWORD crcec;          /* CRC error counter */
  VUHWORD abtsc;          /* abort sequence counter */
  VUHWORD nmarc;          /* nonmatching address rx cnt */
  VUWORD  max_cnt;        /* maximum length counter */
  VUHWORD mflr;           /* maximum frame length reg */
  VUHWORD rfthr;          /* received frames threshold */
  VUHWORD rfcnt;          /* received frames count */
  VUHWORD hmask;          /* user defined frm addr mask */
  VUHWORD haddr1;         /* user defined frm address 1 */
  VUHWORD haddr2;         /* user defined frm address 2 */
  VUHWORD haddr3;         /* user defined frm address 3 */
  VUHWORD haddr4;         /* user defined frm address 4 */
  VUHWORD tmp;            /* temp */
  VUHWORD tmp_mb;         /* temp */
}  /* _PackedType */ t_HdlcFcc_Pram;


/*--------------*/
/* FCC Ethernet */
/*--------------*/

typedef  struct             /* 198 bytes */
{
  VUWORD  stat_bus;       /* Internal use buffer. */
  VUWORD  cam_ptr;        /* CAM address. */
  VUWORD  c_mask;         /* CRC constant mask*/
  VUWORD  c_pres;         /* CRC preset */
  VUWORD  crcec;          /* CRC error counter */
  VUWORD  alec;           /* alignment error counter */
  VUWORD  disfc;          /* discarded frame counter */
  VUHWORD ret_lim;        /* Retry limit threshold. */
  VUHWORD ret_cnt;        /* Retry limit counter. */
  VUHWORD p_per;          /* persistence */
  VUHWORD boff_cnt;       /* back-off counter */
  VUWORD  gaddr_h;        /* group address filter, high */
  VUWORD  gaddr_l;        /* group address filter, low */
  VUHWORD tfcstat;        /* out of sequece Tx BD staus. */
  VUHWORD tfclen;         /* out of sequece Tx BD length. */
  VUWORD  tfcptr;         /* out of sequece Tx BD data pointer. */
  VUHWORD mflr;           /* maximum frame length reg */
  VUHWORD paddr1_h;       /* physical address (MSB) */
  VUHWORD paddr1_m;       /* physical address */
  VUHWORD paddr1_l;       /* physical address (LSB) */
  VUHWORD ibd_cnt;        /* internal BD counter. */
  VUHWORD ibd_start;      /* internal BD start pointer. */
  VUHWORD ibd_end;        /* internal BD end pointer. */
  VUHWORD tx_len;         /* tx frame length counter */
  VUBYTE  ibd_base[0x20]; /* internal micro code usage. */
  VUWORD  iaddr_h;        /* individual address filter, high */
  VUWORD  iaddr_l;        /* individual address filter, low */
  VUHWORD minflr;         /* minimum frame length reg */
  VUHWORD taddr_h;        /* temp address (MSB) */
  VUHWORD taddr_m;        /* temp address */
  VUHWORD taddr_l;        /* temp address (LSB) */
  VUHWORD pad_ptr;        /* pad_ptr. */
  VUHWORD cf_type;        /* RESERVED (flow control frame type coding) */
  VUHWORD cf_range;       /* flow control frame range. */
  VUHWORD max_b;          /* max buffer descriptor byte count. */
  VUHWORD maxd1;          /* max DMA1 length register. */
  VUHWORD maxd2;          /* max DMA2 length register. */
  VUHWORD maxd;           /* Rx max DMA. */
  VUHWORD dma_cnt;        /* Rx DMA counter. */
    
  /* counter: */
  VUWORD  octc;           /* received octets counter. */
  VUWORD  colc;           /* estimated number of collisions */
  VUWORD  broc;           /* received good packets of broadcast address */
  VUWORD  mulc;           /* received good packets of multicast address */
  VUWORD  uspc;           /* received packets shorter then 64 octets. */
  VUWORD  frgc;           /* as uspc + bad packets */
  VUWORD  ospc;           /* received packets longer then 1518 octets. */
  VUWORD  jbrc;           /* as ospc + bad packets  */
  VUWORD  p64c;           /* received packets of 64 octets. */
  VUWORD  p65c;           /* received packets of 65-128 octets. */
  VUWORD  p128c;          /* received packets of 128-255 octets. */
  VUWORD  p256c;          /* received packets of 256-511 octets. */
  VUWORD  p512c;          /* received packets of 512-1023 octets. */
  VUWORD  p1024c;         /* received packets of 1024-1518 octets. */
  VUWORD  cam_buf;        /* cam respond internal buffer. */
  VUHWORD rfthr;          /* received frames threshold */
  VUHWORD rfcnt;          /* received frames count */
}  /* _PackedType */ t_EnetFcc_Pram;


/*-----------------*/
/* FCC Common PRAM */
/*-----------------*/

typedef  struct             /* 306 bytes */
{
  VUHWORD riptr;          /* Rx internal temporary data pointer. */
  VUHWORD tiptr;          /* Tx internal temporary data pointer. */
  VUHWORD reserved0;      /* Reserved */
  VUHWORD mrblr;          /* Rx buffer length */
  VUWORD  rstate;         /* Rx internal state */
  VUWORD  rbase;          /* RX BD base address */
  VUHWORD rbdstat;        /* Rx BD status and control */
  VUHWORD rbdlen;         /* Rx BD data length */
  VUWORD  rdptr;          /* rx BD data pointer */
  VUWORD  tstate;         /* Tx internal state */
  VUWORD  tbase;          /* TX BD base address */
  VUHWORD tbdstat;        /* Tx BD status and control */
  VUHWORD tbdlen;         /* Tx BD data length */
  VUWORD  tdptr;          /* Tx  data pointer */
  VUWORD  rbptr;          /* rx BD pointer */
  VUWORD  tbptr;          /* Tx BD pointer */
  VUWORD  rcrc;           /* Temp receive CRC */
  VUWORD  reserved_1[0x1];
  VUWORD  tcrc;           /* Temp transmit CRC */
  union                   /* Protocol-Specific parameter ram */
  {
    t_HdlcFcc_Pram    h;
    t_EnetFcc_Pram    e;
  } SpecificProtocol;      
}  /* _PackedType */ t_Fcc_Pram;



/*---------------------------------------------------------------------------*/
/*                  MULTICHANNEL COMMUNICATION CONTROLLER (MCC)              */
/*---------------------------------------------------------------------------*/

/******************************************************************************
 * Note that each MCC uses multiple logical channels.  We first define the     *
 * PRAM for a logical channel (which can be used in either HDLC or Transparent *
 * mode;  wherever there are differences, it is specified), followed by the    *
 * PRAM for an MCC itself.                                                     *
 ******************************************************************************/

/*---------------------*/
/* MCC Logical Channel */
/*---------------------*/

typedef  struct             /* 64 bytes */
{
  VUWORD  tstate;         /* Tx internal state. */ 
  VUWORD  zistate;        /* Zero insertion machine state. */
  VUWORD  zidata0;        /* Zero insertion high. */
  VUWORD  zidata1;        /* Zero insertion low. */
  VUHWORD tbdflags;       /* Tx internal BD flags. */
  VUHWORD tbdcnt;         /* Tx internal byte count . */
  VUWORD  tbdptr;         /* Tx internal data pointer. */
  VUHWORD intmask;        /* Interrupt mask flags. */
  VUHWORD chamr;          /* channel mode register. */
  VUWORD  tcrc;           /* Transparent: reserved. */
  /* Hdlc: Temp receive CRC.*/
  VUWORD  rstate;         /* Rx internal state. */ 
  VUWORD  zdstate;        /* Zero deletion machine state. */
  VUWORD  zddata0;        /* Zero deletion high. */
  VUWORD  zddata1;        /* Zero deletion low. */
  VUHWORD rbdflags;       /* Rx internal BD flags. */
  VUHWORD rbdcnt;         /* Rx internal byte count . */
  VUWORD  rbdptr;         /* Rx internal data pointer. */
  VUHWORD maxrlen;        /* Transparent: Max receive buffer length. */
  /* Hdlc: Max receive frame length. */
  VUHWORD sync_maxcnt;    /* Transparent: Receive synchronization pattern*/
  /* Hdlc: Max length counter. */
  VUWORD  rcrc;           /* Transparent: reserved. */
  /* Hdlc: Temp receive CRC.*/
}  /* _PackedType */ t_Mch_Pram;


/*--------------------------------------*/
/* MCC Logical Channel Extra Parameters */
/*--------------------------------------*/

typedef  struct             /* 8 bytes */
{
  VUHWORD tbase;          /* TxBD base address. */
  VUHWORD tbptr;          /* Current TxBD pointer. */
  VUHWORD rbase;          /* RxBD base address. */
  VUHWORD rbptr;          /* Current RxBD pointer. */
}  /* _PackedType */ t_MchXtra_Pram;


/*----------*/
/* MCC PRAM */
/*----------*/

typedef  struct             /* 256 bytes */
{
  VUWORD  mccbase;        /* A pointer to starting address of BD rings. */
  VUHWORD mccstate;       /* Controller state. */
  VUHWORD mrblr;          /* Maximum receive buffer length. */
  VUHWORD grfthr;         /* Global receive frame threshold. */
  VUHWORD grfcnt;         /* Global receive frame counter. */
  VUWORD  rinttmp;        /* Temp location for rx interrupt table entry. */                            
  VUWORD  data0;          /* Temporary location for holding data. */
  VUWORD  data1;          /* Temporary location for holding data. */
  VUWORD  tintbase;       /* Transmit interrupt table base address. */
  VUWORD  tintptr;        /* Transmit interrupt table pointer. */ 
  VUWORD  tinttmp;        /* Temp location for rx interrupt table entry. */
  VUHWORD sctpbase;       /* Pointer to the super channel transmit table*/
  VUBYTE  res0[0x2];      /* Reserved area */
  VUWORD  c_mask32;       /* CRC constant. */
  VUHWORD xtrabase;       /* Pointer to the beginning of extra parameters */                            
  VUHWORD c_mask16;       /* CRC constant. */
  VUWORD  rinttmp0;       /* Temp location for rx interrupt table entry. */                            
  VUWORD  rinttmp1;       /* Temp location for rx interrupt table entry. */
  VUWORD  rinttmp2;       /* Temp location for rx interrupt table entry. */
  VUWORD  rinttmp3;       /* Temp location for rx interrupt table entry. */
  VUWORD  rintbase0;      /* Rx interrupt table base address. */
  VUWORD  rintptr0;       /* Rx interrupt table pointer. */
  VUWORD  rintbase1;      /* Rx interrupt table base address. */
  VUWORD  rintptr1;       /* Rx interrupt table pointer. */
  VUWORD  rintbase2;      /* Rx interrupt table base address. */
  VUWORD  rintptr2;       /* Rx interrupt table pointer. */
  VUWORD  rintbase3;      /* Rx interrupt table base address. */
  VUWORD  rintptr3;       /* Rx interrupt table pointer. */
  VUBYTE  pad[0xa0];
}  /* _PackedType */ t_Mcc_Pram;



/*---------------------------------------------------------------------------*/
/*                              ATM PARAMETER RAM                            */
/*---------------------------------------------------------------------------*/


/*--------------------------------------*/
/* Address Compression parameters table */
/*--------------------------------------*/

struct AddressCompressionPram /* 18 bytes */
{
  VUWORD  VptBase;        /* VP-level addressing table base address */
  VUWORD  VctBase;        /* VC-level addressing table base address */
  VUWORD  Vpt1Base;       /* VP1-level addressing table base address */ 
  VUWORD  Vct1Base;       /* VC1-level addressing table base address */
  VUHWORD VpMask;         /* VP mask for address compression look-up */
}  /* _PackedType */;


/*-------------------------------*/
/* External CAM parameters table */
/*-------------------------------*/

struct ExtCamPram          /* 18 bytes */
{
  VUWORD  ExtCamBase;     /* Base address of the external CAM */
  VUBYTE  reserved00[4];  /* Reserved */
  VUWORD  ExtCam1Base;    /* Base address of the external CAM1 */
  VUBYTE  reserved01[6];  /* Reserved */
}  /* _PackedType */;


/*---------------------------*/
/* ATM mode parameters table */
/*---------------------------*/

typedef  struct AtmPram     /* 274 bytes */
{
  VUBYTE  reserved0[64];  /* Reserved */
  VUHWORD RxCellTmpBase;  /* Rx cell temporary base address */
  VUHWORD TxCellTmpBase;  /* Tx cell temporary base address */
  VUHWORD VUdcTmpBase;    /* VUDC temp base address (in VUDC mode only) */
  VUHWORD IntRctBase;     /* Internal RTC base address */
  VUHWORD IntTctBase;     /* Internal TCT base address */
  VUHWORD IntTcteBase;    /* Internal ACT base address */
  VUBYTE  reserved1[4];   /* reserved four bytes */
  VUWORD  ExtRctBase;     /* Extrnal RTC base address */
  VUWORD  ExtTctBase;     /* Extrnal TCT base address */
  VUWORD  ExtTcteBase;    /* Extrnal ACT base address */
  VUHWORD VUeadOffset;    /* The offset in half-wordunits of the VUEAD
			     entry in the VUDC extra header. Should be
			     even address. If little-endian format is
			     used, the VUeadOffset is of the little-endian
			     format. */
  VUBYTE  reserved2[2];   /* Reserved */
  VUHWORD PmtBase;        /* Performance monitoring table base address */
  VUHWORD ApcParamBase;   /* APC Parameters table base address */
  VUHWORD FbpParamBase;   /* Free buffer pool parameters base address */
  VUHWORD IntQParamBase;  /* Interrupt queue parameters table base */
  VUBYTE  reserved3[2];
  VUHWORD VUniStatTableBase; /* VUNI statistics table base */
  VUWORD  BdBaseExt;      /* BD ring base address extension */
  union 
  {
    struct AddressCompressionPram   AddrCompression;
    struct ExtCamPram               ExtCam;
  } AddrMapping;          /* Address look-up mechanism */
  VUHWORD VciFiltering;   /* VCI filtering enable bits. If bit i is set,
			     the cell with VCI=i will be sent to the
			     raw cell queue. The bits 0-2 and 5 should
			     be zero. */
  VUHWORD Gmode;          /* Global mode */
  VUHWORD CommInfo1;      /* The information field associated with the */
  VUWORD  CommInfo2;      /* last host command */
  VUBYTE  reserved4[4];   /* Reserved */
  VUWORD  CRC32Preset;    /* Preset for CRC32 */
  VUWORD  CRC32Mask;      /* Constant mask for CRC32 */
  VUHWORD AAL1SnpTableBase; /* AAl1 SNP protection look-up table base */
  VUHWORD reserved5;      /* Reserved */
  VUWORD  SrtsBase;       /* External SRTS logic base address. For AAL1
			     only. Should be 16 bytes aligned */
  VUHWORD IdleBase;       /* Idle cell base address */
  VUHWORD IdleSize;       /* Idle cell size: 52, 56, 60, 64 */
  VUWORD  EmptyCellPayload; /* Empty cell payload (little-indian) */
    
  /* ABR specific only */
  VUWORD  Trm; /* VUpper bound on time between F-RM cells for active source */                                 
  VUHWORD Nrm; /* Controls the maximum data cells sent for each F-RM cell. */                           
  VUHWORD Mrm; /* Controls bandwidth between F-RM, B-RM and user data cell */
  VUHWORD Tcr;            /* Tag cell rate */
  VUHWORD AbrRxTcte;      /* ABR reserved area address (2-UHWORD aligned)*/
  VUBYTE  reserved7[76];  /* Reserved */
}  /* _PackedType */ t_Atm_Pram;



/*---------------------------------------------------------------------------*/
/*                      SERIAL MANAGEMENT CHANNEL  (SMC)                     */
/*---------------------------------------------------------------------------*/

typedef  struct  
{
  VUHWORD rbase;          /* Rx BD Base Address */
  VUHWORD tbase;          /* Tx BD Base Address */
  VUBYTE  rfcr;           /* Rx function code */
  VUBYTE  tfcr;           /* Tx function code */
  VUHWORD mrblr;          /* Rx buffer length */
  VUWORD  rstate;         /* Rx internal state */
  VUWORD  rptr;           /* Rx internal data pointer */
  VUHWORD rbptr;          /* rb BD Pointer */
  VUHWORD rcount;         /* Rx internal byte count */
  VUWORD  rtemp;          /* Rx temp */
  VUWORD  tstate;         /* Tx internal state */
  VUWORD  tptr;           /* Tx internal data pointer */
  VUHWORD tbptr;          /* Tx BD pointer */
  VUHWORD tcount;         /* Tx byte count */
  VUWORD  ttemp;          /* Tx temp */
    
  /* SMC VUART-specific PRAM */
  VUHWORD max_idl;        /* Maximum IDLE Characters */
  VUHWORD idlc;           /* Temporary IDLE Counter */
  VUHWORD brkln;          /* Last Rx Break Length */
  VUHWORD brkec;          /* Rx Break Condition Counter */
  VUHWORD brkcr;          /* Break Count Register (Tx) */
  VUHWORD r_mask;         /* Temporary bit mask */  
}  /* _PackedType */ t_Smc_Pram;



/*---------------------------------------------------------------------------*/
/*                            IDMA PARAMETER RAM                             */
/*---------------------------------------------------------------------------*/

typedef  struct  
{
  VUHWORD ibase;          /* IDMA BD Base Address */
  VUHWORD dcm;            /* DMA channel mode register */
  VUHWORD ibdptr;         /* next bd ptr */
  VUHWORD DPR_buf;        /* ptr to internal 64 byte buffer */
  VUHWORD BUF_inv;        /* The quantity of data in DPR_buf */
  VUHWORD SS_max;         /* Steady State Max. transfer size */
  VUHWORD DPR_in_ptr;     /* write ptr for the internal buffer */
  VUHWORD sts;            /* Source Transfer Size */
  VUHWORD DPR_out_ptr;    /* read ptr for the internal buffer */
  VUHWORD seob;           /* Source end of burst */
  VUHWORD deob;           /* Destination end of burst */
  VUHWORD dts;            /* Destination Transfer Size */
  VUHWORD RetAdd;         /* return address when ERM==1 */
  VUHWORD Reserved;       /* reserved */
  VUWORD  BD_cnt;         /* Internal byte count */
  VUWORD  S_ptr;          /* source internal data ptr */
  VUWORD  D_ptr;          /* destination internal data ptr */
  VUWORD  istate;         /* Internal state */
}  /* _PackedType */ t_Idma_Pram;



/*-------------------------------------------------------------------*/
/*                    INTER-INTEGRATED CIRCUIT  (I2C)                */
/*-------------------------------------------------------------------*/

typedef  struct 
{
  VUHWORD rbase;          /* RX BD base address */
  VUHWORD tbase;          /* TX BD base address */
  VUBYTE  rfcr;           /* Rx function code */
  VUBYTE  tfcr;           /* Tx function code */
  VUHWORD mrblr;          /* Rx buffer length */
  VUWORD  rstate;         /* Rx internal state */
  VUWORD  rptr;           /* Rx internal data pointer */
  VUHWORD rbptr;          /* rb BD Pointer */
  VUHWORD rcount;         /* Rx internal byte count */
  VUWORD  rtemp;          /* Rx temp */
  VUWORD  tstate;         /* Tx internal state */
  VUWORD  tptr;           /* Tx internal data pointer */
  VUHWORD tbptr;          /* Tx BD pointer */
  VUHWORD tcount;         /* Tx byte count */
  VUWORD  ttemp;          /* Tx temp */
}  /* _PackedType */ t_I2c_Pram;



/*---------------------------------------------------------------------------*/
/*                       SERIAL PERIPHERAL INTERFACE  (SPI)                  */
/*---------------------------------------------------------------------------*/

typedef  struct  
{
  VUHWORD rbase;          /* Rx BD Base Address */
  VUHWORD tbase;          /* Tx BD Base Address */
  VUBYTE  rfcr;           /* Rx function code */
  VUBYTE  tfcr;           /* Tx function code */
  VUHWORD mrblr;          /* Rx buffer length */
  VUWORD  rstate;         /* Rx internal state */
  VUWORD  rptr;           /* Rx internal data pointer */
  VUHWORD rbptr;          /* Rx BD Pointer */
  VUHWORD rcount;         /* Rx internal byte count */
  VUWORD  rtemp;          /* Rx temp */
  VUWORD  tstate;         /* Tx internal state */
  VUWORD  tptr;           /* Tx internal data pointer */
  VUHWORD tbptr;          /* Tx BD pointer */
  VUHWORD tcount;         /* Tx byte count */
  VUWORD  ttemp;          /* Tx temp */
  VUBYTE  reserved[8];
}  /* _PackedType */ t_Spi_Pram;



/*---------------------------------------------------------------------------*/
/*                      RISC TIMER PARAMETER RAM                             */
/*---------------------------------------------------------------------------*/

typedef  struct  
{
 
  VUHWORD tm_base;        /* RISC timer table base adr */
  VUHWORD tm_ptr;         /* RISC timer table pointer */
  VUHWORD r_tmr;          /* RISC timer mode register */
  VUHWORD r_tmv;          /* RISC timer valid register */
  VUWORD  tm_cmd;         /* RISC timer cmd register */
  VUWORD  tm_cnt;         /* RISC timer internal cnt */
}  /* _PackedType */ t_timer_pram;



/*--------------------------------------------------------------------------*/
/*                  ROM MICROCODE PARAMETER RAM AREA                        */
/*--------------------------------------------------------------------------*/

typedef  struct  
{
  VUHWORD rev_num;        /* VUcode Revision Number */
  VUHWORD d_ptr;          /* MISC Dump area pointer */
}  /* _PackedType */ t_ucode_pram;








/*--------------------------------------------------------------------------*/
/*                MAIN DEFINITION OF MPC8260 INTERNAL MEMORY MAP            */
/*--------------------------------------------------------------------------*/

typedef  struct 
{

  /* cpm_ram */
  t_Mch_Pram  mch_pram[256];      /* MCC logical channels parameter ram */
  VUBYTE      reserved0[0x4000];  /* Reserved area */
    
  /* DPR_BASE+0x8000*/

  /*---------------------------------------------------------------------*/
  /* A note about the pram union:                                        */
  /* The pram area has been broken out three ways for clean access into  */
  /* certain peripherals' spaces.  This arrangement allows programmers   */
  /* flexibility of usage in terms of being able to change which         */
  /* peripheral is being accessed by simply changing an array value.     */
  /* Given the interweaving of certain peripherals' pram areas, this     */
  /* would not be possible with only one large pram structure.           */
  /*                                                                     */
  /* SERIALS  - For accessing SCC, non-ATM FCC, and MCC pram             */
  /* ATM      - For accessing ATM FCC pram                               */
  /* STANDARD - For accessing timers, revnum, d_ptr, RAND, and the pram  */
  /*            base pointers of the SMCs, IDMAs, SPI, and I2C           */
  /*---------------------------------------------------------------------*/
           
  union 
  {
    
    /*for access to the PRAM structs for SCCs, FCCs, and MCCs */ 
    struct serials 
    {
      t_Scc_Pram scc_pram[4];
      t_Fcc_Pram fcc_pram[3];
      t_Mcc_Pram mcc_pram[2];
      /*           VUBYTE reserved1[0x700];   */
    } serials;
        
    /* for access to ATM PRAM structs */
    struct atm
    {
      VUBYTE reserved2[0x400];
      t_Atm_Pram atm_pram[2];
      /*           VUBYTE reserved3[0xa00]; */
    } atm;
        
    /* for access to the memory locations holding user-defined 
       base addresses of PRAM for SMCs, IDMA, SPI, and I2C. */     
    struct standard
    {
      VUBYTE scc1[0x100];
      VUBYTE scc2[0x100];
      VUBYTE scc3[0x100];
      VUBYTE scc4[0x100];
      VUBYTE fcc1[0x100];
      VUBYTE fcc2[0x100];
      VUBYTE fcc3[0x100];
      VUBYTE mcc1[0x80];
      VUBYTE reserved_0[0x7c];
      VUBYTE smc1[0x2];
      VUBYTE idma1[0x2];
      VUBYTE mcc2[0x80];
      VUBYTE reserved_1[0x7c];
      VUBYTE smc2[0x2];
      VUBYTE idma2[0x2];
      VUBYTE reserved_2[0xfc];
      VUBYTE spi[0x2];
      VUBYTE idma3[0x2];
      VUBYTE reserved_3[0xe0];
      VUBYTE timers[0x10];
      VUBYTE Rev_num[0x2];
      VUBYTE D_ptr[0x2];
      VUBYTE reserved_4[0x4];
      VUBYTE rand[0x4];
      VUBYTE i2c[0x2];
      VUBYTE idma4[0x2];
      VUBYTE reserved_5[0x500];
    } standard;
        
  }  /* _PackedType */ pram;
    
  VUBYTE  reserved11[0x2000];      /* Reserved area */
  VUBYTE  cpm_ram_dpram_2[0x1000]; /* Internal RAM */
  VUBYTE  reserved12[0x4000];      /* Reserved area */

  /* siu */
  VUWORD  siu_siumcr;         /* SIU Module Configuration Register */
  VUWORD  siu_sypcr;          /* System Protection Control Register */
  VUBYTE  reserved13[0x6];    /* Reserved area */
  VUHWORD siu_swsr;           /* Software Service Register */

  /* buses */
  VUBYTE  reserved14[0x14];   /* Reserved area */
  VUWORD  bcr;                /* Bus Configuration Register */
  VUBYTE  ppc_acr;            /* Arbiter Configuration Register */
  VUBYTE  reserved15[0x3];    /* Reserved area */
  VUWORD  ppc_alrh;           /* Arbitration Level Reg. (First clients)*/
  VUWORD  ppc_alrl;           /* Arbitration Level Reg. (Next clients) */
  VUBYTE  lcl_acr;            /* LCL Arbiter Configuration Register */
  VUBYTE  reserved16[0x3];    /* Reserved area */
  VUWORD  lcl_alrh;      /* LCL Arbitration Level Reg. (First clients) */
  VUWORD  lcl_alrl;      /* LCL Arbitration Level Register (Next clients) */
  VUWORD  tescr1;        /* PPC bus transfer error status control reg. 1 */
  VUWORD  tescr2;        /* PPC bus transfer error status control reg. 2 */
  VUWORD  ltescr1;       /* Local bus transfer error status control reg. 1 */
  VUWORD  ltescr2;       /* Local bus transfer error status control reg. 2 */
  VUWORD  pdtea;              /* PPC bus DMA Transfer Error Address */
  VUBYTE  pdtem;              /* PPC bus DMA Transfer Error MSNUM  */
  VUBYTE  reserved17[0x3];    /* Reserved area */
  VUWORD  ldtea;              /* PPC bus DMA Transfer Error Address */
  VUBYTE  ldtem;              /* PPC bus DMA Transfer Error MSNUM  */
  VUBYTE  reserved18[0xa3];   /* Reserved area */

  /* memc */
  struct memc_regs 
  {
    VUWORD br;              /* Base Register */
    VUWORD or;              /* Option Register */
  } memc_regs[12];
  VUBYTE  reserved19[0x8];    /* Reserved area */
  VUWORD  memc_mar;           /* Memory Address Register */
  VUBYTE  reserved20[0x4];    /* Reserved area */
  VUWORD  memc_mamr;          /* Machine A Mode Register */
  VUWORD  memc_mbmr;          /* Machine B Mode Register */
  VUWORD  memc_mcmr;          /* Machine C Mode Register */
  VUWORD  memc_mdmr;          /* Machine D Mode Register */
  VUBYTE  reserved21[0x4];    /* Reserved area */
  VUHWORD memc_mptpr;         /* Memory Periodic Timer Prescaler */
  VUBYTE  reserved22[0x2];    /* Reserved area */
  VUWORD  memc_mdr;           /* Memory Data Register */
  VUBYTE  reserved23[0x4];    /* Reserved area */
  VUWORD  memc_psdmr;         /* PowerPC Bus SDRAM machine Mode Register */
  VUWORD  memc_lsdmr;         /* Local Bus SDRAM machine Mode Register */
  VUBYTE  memc_purt;          /* PowerPC Bus assigned VUPM Refresh Timer */
  VUBYTE  reserved24[0x3];    /* Reserved area */
  VUBYTE  memc_psrt;          /* PowerPC Bus assigned SDRAM Refresh Timer */
  VUBYTE  reserved25[0x3];    /* Reserved area */
  VUBYTE  memc_lurt;          /* Local Bus assigned VUPM Refresh Timer */
  VUBYTE  reserved26[0x3];    /* Reserved area */
  VUBYTE  memc_lsrt;          /* Local Bus assigned SDRAM Refresh Timer */
  VUBYTE  reserved27[0x3];    /* Reserved area */
  VUWORD  memc_immr;          /* Internal Memory Map Register */

  /* pci */
  VUWORD  pcibr0;             /* Base address+valid for PCI window 1 */
  VUWORD  pcibr1;             /* Base address+valid for PCI window 2 */
  VUBYTE  reserved28[0x10];   /* Reserved area */
  VUWORD  pcimsk0;            /* Mask for PCI window 1 */
  VUWORD  pcimsk1;            /* Mask for PCI window 2 */
  VUBYTE  reserved29[0x54];   /* Reserved area */

  /* si_timers */
  VUHWORD si_timers_tmcntsc;  /* Time Counter Status and Control Register */
  VUBYTE  reserved30[0x2];    /* Reserved area */
  VUWORD  si_timers_tmcnt;    /* Time Counter Register */
  VUWORD  si_timers_tmcntsec; /* Time Counter Seconds*/
  VUWORD  si_timers_tmcntal;  /* Time Counter Alarm Register */
  VUBYTE  reserved31[0x10];   /* Reserved area */
  VUHWORD si_timers_piscr;    /* Periodic Interrupt Status and Control Reg. */
  VUBYTE  reserved32[0x2];    /* Reserved area */
  VUWORD  si_timers_pitc;     /* Periodic Interrupt Count Register */
  VUWORD  si_timers_pitr;     /* Periodic Interrupt Timer Register */
  VUBYTE  reserved33[0x54];   /* Reserved area */

  /* test module registers */
  VUWORD  tstmhr;             
  VUWORD  tstmlr;
  VUHWORD tster;
  VUBYTE  reserved34[0x156];  /* Reserved area */
    
  /* pci, part 2 */
  VUWORD  pci_pci;            /* PCI Configuration space */
  VUBYTE  reserved35[0x7fc];  /* Reserved area */
    
  /* ic */
  VUHWORD ic_sicr;            /* Interrupt Configuration Register */
  VUBYTE  reserved36[0x2];    /* Reserved area */
  VUWORD  ic_sivec;           /* CP Interrupt Vector Register */
  VUWORD  ic_sipnr_h;         /* Interrupt Pending Register (HIGH) */
  VUWORD  ic_sipnr_l;         /* Interrupt Pending Register (LOW) */
  VUWORD  ic_siprr;           /* SIU Interrupt Priority Register */
  VUWORD  ic_scprr_h;         /* Interrupt Priority Register (HIGH) */
  VUWORD  ic_scprr_l;         /* Interrupt Priority Register (LOW) */
  VUWORD  ic_simr_h;          /* Interrupt Mask Register (HIGH) */
  VUWORD  ic_simr_l;          /* Interrupt Mask Register (LOW) */
  VUWORD  ic_siexr;           /* External Interrupt Control Register */
  VUBYTE  reserved37[0x58];   /* Reserved area */

  /* clocks */
  VUWORD  clocks_sccr;        /* System Clock Control Register */
  VUBYTE  reserved38[0x4];    /* Reserved area */
  VUWORD  clocks_scmr;        /* System Clock Mode Register */
  VUBYTE  reserved39[0x4];    /* Reserved area */
  VUWORD  clocks_rsr;         /* Reset Status Register */
  VUWORD  clocks_rmr;         /* Reset Moode Register  */
  VUBYTE  reserved40[0x68];   /* Reserved area */

  /* io_ports */
  struct io_regs 
  {
    VUWORD  pdir;           /* Port A-D Data Direction Register */
    VUWORD  ppar;           /* Port A-D Pin Assignment Register */
    VUWORD  psor;           /* Port A-D Special Operation Register */
    VUWORD  podr;           /* Port A-D Open Drain Register */
    VUWORD  pdat;           /* Port A-D Data Register */
    VUBYTE reserved41[0xc]; /* Reserved area */
  } io_regs[4];

  /* cpm_timers */
  VUBYTE  cpm_timers_tgcr1;   /* Timer Global Configuration Register */
  VUBYTE  reserved42[0x3];    /* Reserved area */
  VUBYTE  cpm_timers_tgcr2;   /* Timer Global Configuration Register */
  VUBYTE  reserved43[0xb];    /* Reserved area */
  VUHWORD cpm_timers_tmr1;    /* Timer Mode Register */
  VUHWORD cpm_timers_tmr2;    /* Timer Mode Register */
  VUHWORD cpm_timers_trr1;    /* Timer Reference Register */
  VUHWORD cpm_timers_trr2;    /* Timer Reference Register */
  VUHWORD cpm_timers_tcr1;    /* Timer Capture Register */
  VUHWORD cpm_timers_tcr2;    /* Timer Capture Register */
  VUHWORD cpm_timers_tcn1;    /* Timer Counter */
  VUHWORD cpm_timers_tcn2;    /* Timer Counter */
  VUHWORD cpm_timers_tmr3;    /* Timer Mode Register */
  VUHWORD cpm_timers_tmr4;    /* Timer Mode Register */
  VUHWORD cpm_timers_trr3;    /* Timer Reference Register */
  VUHWORD cpm_timers_trr4;    /* Timer Reference Register */
  VUHWORD cpm_timers_tcr3;    /* Timer Capture Register */
  VUHWORD cpm_timers_tcr4;    /* Timer Capture Register */
  VUHWORD cpm_timers_tcn3;    /* Timer Counter */
  VUHWORD cpm_timers_tcn4;    /* Timer Counter */
  VUHWORD cpm_timers_ter[4];  /* Timer Event Register */
  VUBYTE reserved44[0x260];   /* Reserved area */

  /* sdma general */
  VUBYTE  sdma_sdsr;          /* SDMA Status Register */
  VUBYTE  reserved45[0x3];    /* Reserved area */
  VUBYTE  sdma_sdmr;          /* SDMA Mask Register */
  VUBYTE  reserved46[0x3];    /* Reserved area */

  /* idma */
  VUBYTE  idma_idsr1;         /* IDMA Status Register */
  VUBYTE  reserved47[0x3];    /* Reserved area */
  VUBYTE  idma_idmr1;         /* IDMA Mask Register */
  VUBYTE  reserved48[0x3];    /* Reserved area */
  VUBYTE  idma_idsr2;         /* IDMA Status Register */
  VUBYTE  reserved49[0x3];    /* Reserved area */
  VUBYTE  idma_idmr2;         /* IDMA Mask Register */
  VUBYTE  reserved50[0x3];    /* Reserved area */
  VUBYTE  idma_idsr3;         /* IDMA Status Register */
  VUBYTE  reserved51[0x3];    /* Reserved area */
  VUBYTE  idma_idmr3;         /* IDMA Mask Register */
  VUBYTE  reserved52[0x3];    /* Reserved area */
  VUBYTE  idma_idsr4;         /* IDMA Status Register */
  VUBYTE  reserved53[0x3];    /* Reserved area */
  VUBYTE  idma_idmr4;         /* IDMA Mask Register */
  VUBYTE  reserved54[0x2c3];  /* Reserved area */
    
  /* fcc */
  struct fcc_regs 
  {
    VUWORD  gfmr;           /* FCC General Mode Register */
    VUWORD  psmr;           /* FCC Protocol Specific Mode Register */
    VUHWORD todr;           /* FCC Transmit On Demand Register */
    VUBYTE  reserved55[0x2];/* Reserved area */
    VUHWORD dsr;            /* FCC Data Sync. Register */
    VUBYTE  reserved56[0x2];/* Reserved area */
    VUWORD  fcce;           /* FCC Event Register */
    VUWORD  fccm;           /* FCC Mask Register */
    VUBYTE  fccs;           /* FCC Status Register */
    VUBYTE  reserved57[0x3];/* Reserved area */
    VUWORD  ftprr;          /* FCC Transmit Partial Rate Register */
  } fcc_regs[3];
  VUBYTE reserved58[0x290];   /* Reserved area */
    
  /* brgs 5-8 */
  VUWORD  brgs_brgc5;         /* BRG Configuration Register */
  VUWORD  brgs_brgc6;         /* BRG Configuration Register */
  VUWORD  brgs_brgc7;         /* BRG Configuration Register */
  VUWORD  brgs_brgc8;         /* BRG Configuration Register */
  VUBYTE  reserved59[0x260];  /* Reserved area */
    
  /* i2c */
  VUBYTE  i2c_i2mod;          /* IC Mode Register */
  VUBYTE  reserved60[0x3];    /* Reserved area */
  VUBYTE  i2c_i2add;          /* IC Address Register */
  VUBYTE  reserved61[0x3];    /* Reserved area */
  VUBYTE  i2c_i2brg;          /* IC BRG Register */
  VUBYTE  reserved62[0x3];    /* Reserved area */
  VUBYTE  i2c_i2com;          /* IC Command Register */
  VUBYTE  reserved63[0x3];    /* Reserved area */
  VUBYTE  i2c_i2cer;          /* IC Event Register */
  VUBYTE  reserved64[0x3];    /* Reserved area */
  VUBYTE  i2c_i2cmr;          /* IC Mask Register */
  VUBYTE  reserved65[0x14b];  /* Reserved area */
    
  /* cpm */
  VUWORD  cpm_cpcr;           /* Communication Processor Command Register */
  VUWORD  cpm_rccr;           /* RISC Configuration Register */
  VUWORD  cpm_rmdr;           /* RISC Microcode Dev. Support Control Reg. */
  VUHWORD cpm_rctr1;          /* RISC Controller Trap Register */
  VUHWORD cpm_rctr2;          /* RISC Controller Trap Register */
  VUHWORD cpm_rctr3;          /* RISC Controller Trap Register */
  VUHWORD cpm_rctr4;          /* RISC Controller Trap Register */
  VUBYTE  reserved66[0x2];    /* Reserved area */
  VUHWORD cpm_rter;           /* RISC Timers Event Register */
  VUBYTE  reserved67[0x2];    /* Reserved area */
  VUHWORD cpm_rtmr;           /* RISC Timers Mask Register */
  VUHWORD cpm_rtscr;          /* RISC Time-Stamp Timer Control Register */
  VUHWORD cpm_rmds;           /* RISC Development Support Status Register */
  VUWORD  cpm_rtsr;           /* RISC Time-Stamp Register */
  VUBYTE  reserved68[0xc];    /* Reserved area */

  /* brgs 1-4 */
  VUWORD  brgs_brgc1;         /* BRG Configuration Register */
  VUWORD  brgs_brgc2;         /* BRG Configuration Register */
  VUWORD  brgs_brgc3;         /* BRG Configuration Register */
  VUWORD  brgs_brgc4;         /* BRG Configuration Register */
    
  /* scc */
  struct scc_regs_8260
  {
    VUWORD  gsmr_l;         /* SCC General Mode Register */
    VUWORD  gsmr_h;         /* SCC General Mode Register */
    VUHWORD psmr;           /* SCC Protocol Specific Mode Register */
    VUBYTE  reserved69[0x2];/* Reserved area */
    VUHWORD todr;           /* SCC Transmit-On-Demand Register */
    VUHWORD dsr;            /* SCC Data Synchronization Register */
    VUHWORD scce;           /* SCC Event Register */
    VUBYTE  reserved70[0x2];/* Reserved area */
    VUHWORD sccm;           /* SCC Mask Register */
    VUBYTE  reserved71;     /* Reserved area */
    VUBYTE  sccs;           /* SCC Status Register */
    VUBYTE  reserved72[0x8];/* Reserved area */
  } scc_regs[4];
    
  /* smc */
  struct smc_regs_8260
  {
    VUBYTE  reserved73[0x2];/* Reserved area */
    VUHWORD smcmr;          /* SMC Mode Register */
    VUBYTE  reserved74[0x2];/* Reserved area */
    VUBYTE  smce;           /* SMC Event Register */
    VUBYTE  reserved75[0x3];/* Reserved area */
    VUBYTE  smcm;           /* SMC Mask Register */
    VUBYTE  reserved76[0x5];/* Reserved area */
  } smc_regs[2];
    
  /* spi */
  VUHWORD spi_spmode;         /* SPI Mode Register */
  VUBYTE  reserved77[0x4];    /* Reserved area */
  VUBYTE  spi_spie;           /* SPI Event Register */
  VUBYTE  reserved78[0x3];    /* Reserved area */
  VUBYTE  spi_spim;           /* SPI Mask Register */
  VUBYTE  reserved79[0x2];    /* Reserved area */
  VUBYTE  spi_spcom;          /* SPI Command Register */
  VUBYTE  reserved80[0x52];   /* Reserved area */
    
  /* cpm_mux */
  VUBYTE  cpm_mux_cmxsi1cr;   /* CPM MUX SI Clock Route Register */
  VUBYTE  reserved81;         /* Reserved area */
  VUBYTE  cpm_mux_cmxsi2cr;   /* CPM MUX SI Clock Route Register */
  VUBYTE  reserved82;         /* Reserved area */
  VUWORD  cpm_mux_cmxfcr;     /* CPM MUX FCC Clock Route Register */
  VUWORD  cpm_mux_cmxscr;     /* CPM MUX SCC Clock Route Register */
  VUBYTE  cpm_mux_cmxsmr;     /* CPM MUX SMC Clock Route Register */
  VUBYTE  reserved83;         /* Reserved area */
  VUHWORD cpm_mux_cmxuar;     /* CPM MUX VUTOPIA Address Register */
  VUBYTE  reserved84[0x10];   /* Reserved area */
    
  /* si */
  struct si_regs 
  {
    VUHWORD sixmr[4];       /* SI TDM Mode Registers A-D */
    VUBYTE  sigmr;          /* SI Global Mode Register */
    VUBYTE  reserved85;     /* Reserved area */
    VUBYTE  sicmdr;         /* SI Command Register */
    VUBYTE  reserved86;     /* Reserved area */
    VUBYTE  sistr;          /* SI Status Register */
    VUBYTE  reserved87;     /* Reserved area */
    VUHWORD sirsr;          /* SI RAM Shadow Address Register */
    VUHWORD mcce;           /* MCC Event Register */
    VUBYTE  reserved88[0x2];/* Reserved area */
    VUHWORD mccm;           /* MCC Mask Register */
    VUBYTE  reserved89[0x2];/* Reserved area */
    VUBYTE  mccr;           /* MCC Configuration Register */
    VUBYTE  reserved90[0x7];/* Reserved area */
  } si_regs[2];
  VUBYTE reserved91[0x4a0];   /* Reserved area */
    
  /* si_ram */
  struct si_ram 
  {
    VUHWORD tx_siram[0x100];/* SI Transmit Routing RAM */
    VUBYTE  reserved92[0x200];/* Reserved area */
    VUHWORD rx_siram[0x100];/* SI Receive Routing RAM */
    VUBYTE  reserved93[0x200];/* Reserved area */
  } si_ram[2];
  VUBYTE  reserved94[0x1000]; /* Reserved area */

}  /* _PackedType */ t_PQ2IMM;





/***************************************************************************/
/*                   General Global Definitions                            */
/***************************************************************************/

#define PORTA   0           /* Index into Parallel I/O Regs Array */              
#define PORTB   1           /* Index into Parallel I/O Regs Array */              
#define PORTC   2           /* Index into Parallel I/O Regs Array */              
#define PORTD   3           /* Index into Parallel I/O Regs Array */
                                 
#define PAGE1   0           /* Index 1 into PRAM Array */
#define PAGE2   1           /* Index 2 into PRAM Array */
#define PAGE3   2           /* Index 3 into PRAM Array */
#define PAGE4   3           /* Index 4 into PRAM Array */

#define FCC1    0           /* FCC1 Index into FCC PRAM Array */
#define FCC2    1           /* FCC2 Index into FCC PRAM Array */
#define FCC3    2           /* FCC3 Index into FCC PRAM Array */

#define MCC1    0           /* MCC1 Index into MCC PRAM Array */
#define MCC2    1           /* MCC2 Index into MCC PRAM Array */

#define SCC1    0           /* SCC1 Index into SCC PRAM Array */             
#define SCC2    1           /* SCC2 Index into SCC PRAM Array */             
#define SCC3    2           /* SCC3 Index into SCC PRAM Array */             
#define SCC4    3           /* SCC4 Index into SCC PRAM Array */

#define TDMA    0           /* Index into TDM Mode Register Array */
#define TDMB    1           /* Index into TDM Mode Register Array */
#define TDMC    2           /* Index into TDM Mode Register Array */
#define TDMD    3           /* Index into TDM Mode Register Array */

#define SI1     0           /* Index into SI Register Array */
#define SI2     1           /* Index into SI Register Array */             

#endif

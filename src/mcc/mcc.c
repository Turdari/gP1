/*
 * mcc.c - mcc linux device driver 
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
 * configurated for 64 chns 64..95 to TDM1C and 96..127 to TDM1D
 * clock routing and I/O routing supposed done in fec001a_setup.c.
 * Only 60x bus transations are configurated.
 * Trasmission interrupt not allowed, rx interrupt routed on RXINT0 queue only for 
 * 32nd chn (on TDM1C).
 * 
 * Tx scheme sends WriteBuffer data if it is valid or notone byte, 0xD5 (0x7E for 1st chn), instead   
 * Rx fills ReadBuffer every RxBD recieved. Handshaking is done with  writeNow, redNow and valid global
 * variables.
 * writeNow = 1 : WriteBuffer trasferred by cpm
 * readNow = 1	: ReadBuffer filled by cpm
 * valid = 1 	: WriteBuffder is valid
 * 
 *
 * WriteBuffer, ReadBuffer 	: buffer for read & write system call.
 * RxTxBDs			: Rx and Tx BDs ring. NUM_TXBDS Tx BD and NUM_RXBDS RxBD for chn
 * BufferPool			: Tx and Rx MCC buffer
 * txIntCQ, rxIntCQ		: tx and rx interrupt queue. 
 *
 * ioctl		        : MCC_IOC_TXBDS reads and returns TxBDs status
 *				: MCC_IOC_RXBDS reads and returns RxBDs status
 *				: SIXRAM_IOC_REGS reads and returns SI register and ram entries
 *				: TXRX_IOC_BUFF reads and return Tx and Rx MCC buffer
 *				: MCC_FASYNC_IOC_DEAMON allow fasync notification
 *
 * Rev 1.00 - 20/09/2002
 * Rev 1.01 - 29/10/2002	: add ioctl command.
 * Rev 1.02 - 30/10/2002	: non static definition of the function getFrame,sendFrame,restoreBDs.
 * Rev 1.03 - 04/11/2002	: irq handler locked with spin_lock_irq _irq macro. 
 * Rev 1.04 - 05/11/2002	: new versions of mcc_ISR, getFrame, sendFrame and restoreBDs functions.
 * Rev 1.05 - 06/11/2002	: all MCC interrupt abled to discover FIFO's underun exception
 * Rev 1.06 - 11/11/2002	: fsync notification during underrun interrupt error
 *  	
 *
 */


#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif


#define MCC_DEBUG
#undef MCC_DEBUG
#define MCC_WRITE
#undef MCC_WRITE


#include <linux/param.h>
#include <asm/system.h>

#include <linux/config.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
//#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/malloc.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/tqueue.h>
#include <linux/ioctl.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/immap_8260.h>
#include <asm/pgtable.h>
#include <asm/mpc8260.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/cpm_8260.h>

#include "mcc.h"
#include "sysdep.h"
#include "mccIOC.h"
#include "device.h"

/*-------------------------------------------------------------------------------
 *								Defines
 *--------------------------------------------------------------------------------
 */

#define NUM(dev) 					(MINOR(dev) & 0xf)
#define min(A, B) 					((A<B)?A:B)
#define MCC_IRQ						SIU_INT_MCC1
#define MCC_MAJOR					0   /*	40  */


/*-------------------------------------------------------------------------------
 *								Module parameters
 *--------------------------------------------------------------------------------
 */

static int mcc_major = MCC_MAJOR;/* major number*/

//MODULE_PARM(mcc_major,"i");
MODULE_AUTHOR ("Eurostudio");


/*---------------------------------------------------------------------------------
 *								Function prototypes 
 *---------------------------------------------------------------------------------
 */

#ifdef LINUX_20
int mcc_read(struct inode *,struct file *,char *,int);
#else
ssize_t mcc_read(struct file *,char *,size_t,loff_t *);
#endif

ssize_t do_mcc_read(struct inode *,struct file *,char *,size_t,loff_t *);


#ifdef LINUX_20
int mcc_write(struct inode *,struct file *,char *,int );
#else
ssize_t mcc_write(struct file *,const char *,size_t,loff_t *);
#endif

ssize_t do_mcc_write(struct inode *,struct file *,const char *,size_t,loff_t *);

int mcc_ioctl(struct inode *,struct file *,unsigned int ,unsigned long );

int mcc_open(struct inode *,struct file *);

#ifdef LINUX_20
void mcc_release(struct inode *,struct file *);
#else
int mcc_release(struct inode *,struct file *);
#endif

int mcc_fasync(fasync_file ,struct file *,int );

void mccFasync(void);

/*---------------------------------------------------------------------------------
 *								Global variables
 *---------------------------------------------------------------------------------
 */
/* read & write Handshaking
 */
int writeNow[BUFFER_SIZE];
int valid[BUFFER_SIZE];
int readNow[BUFFER_SIZE];

/* IO register external status variable ### 
 */

extern unsigned int reg0data; 
/* read and write data buffer
 */
FrameBuffer *ReadBuffer, *WriteBuffer;

static struct file_operations mcc_fops=
  {
    read:    mcc_read,
    write:   mcc_write,
    ioctl:   mcc_ioctl,
    open:    mcc_open,
    release: mcc_release,
    fasync:  mcc_fasync,
  };

volatile immap_t *immap;

/*	TxRx data buffer 
 */
unsigned char *BufferPool;

struct fasync_struct *async_queue;

/* buffer descriptors base pointer
 */
BDRINGS *RxTxBD;

/* TxRx interrupt queue
 */
unsigned int *txIntCQ;
unsigned int *rxIntCQ;

/* spin lock
 */
spinlock_t lock;

/* fasync notification
 */
int fasyncActive = FALSE;
/*---------------------------------------------------------------------------------
 *								Function
 *---------------------------------------------------------------------------------
 */
/*
 * Name:
 *	mcc_cleanup
 * Description:
 *	unload mcc driver
 */
void mcc_cleanup (void){
  stopTxRx();
  free_irq( MCC_IRQ,NULL);
  kfree(BufferPool);
  kfree(RxTxBD);
  kfree(txIntCQ);
  kfree(rxIntCQ);
  kfree(WriteBuffer);
  kfree(ReadBuffer);
  unregister_chrdev(mcc_major,"mcc");
}

/* 
 * Name :
 * 	mcc_init
 * Input Parameter:
 * 	None
 * Description:
 * 	load the driver 
 */ 
int  mcc_init(void){
  int mcc_result = 0;
#ifdef MCC_DEBUG
  unsigned int *BoardIO = ioremap(0xF1000000,0x32);
#endif
  int i;
	
  /* pointer to internal register 
   */
  immap = (immap_t *)IMAP_ADDR;

#ifdef MCC_DEBUG
  *BoardIO = 0x30000000 | reg0data;
  reg0data |= 0x30000000;
#endif	
  /* memory allocation for TxRx BD data buffer
   */
  BufferPool = (unsigned char *) kmalloc ((sizeof(unsigned char )*NUMBER_OF_CHANNELS*(NUM_RXBDS+NUM_TXBDS )*BUFFER_SIZE),GFP_KERNEL);												
	
  if (BufferPool == NULL)
    return -ENOMEM;

  for(i=0; i < BUFFER_SIZE; i++){
    readNow[i] = 0;
    writeNow[i] = 0;
    valid[i] = 1;
  }
	
  /* spin lock declaration
   */
  spin_lock_init(&lock);
	

#ifdef MCC_DEBUG
  printk("MCC:Data Buffer at %xH\n",(unsigned int )BufferPool);
#endif 	
  /* clear  buffer 
   */
  memset(BufferPool, NULL_PATTERN, NUMBER_OF_CHANNELS*(NUM_RXBDS+NUM_TXBDS )*BUFFER_SIZE);
  memset(BufferPool, SILENCE_PATTERN, NUMBER_OF_CHANNELS*NUM_TXBDS*BUFFER_SIZE);

#ifdef MCC_DEBUG
  memset(BufferPool, SYNC_PATTERN, NUM_TXBDS*BUFFER_SIZE);	/* fixed pattern for 1st channel  */
#endif 	

    
#ifdef MCC_DEBUG
  printk("MCC:Data Buffer cleared\n");
#endif 	
	
  /* memory allocation for TxRxBDs*/
  RxTxBD = (BDRINGS *) kmalloc ((sizeof(BDRINGS )),GFP_KERNEL);
  if (RxTxBD == NULL)
    return -ENOMEM; 

#ifdef MCC_DEBUG
  printk("RxTxBD = %xH\n",(unsigned int ) RxTxBD);
  printk("MCC:BDs memory allocated.. %xH bytes\n",sizeof(BDRINGS));
#endif 	


  /* memory allocation for txIntCQ
   */
  txIntCQ = (unsigned int *) kmalloc ((sizeof(unsigned int )*NUM_CHN_INT*NUM_TXBDS),GFP_KERNEL);
  if (txIntCQ == NULL)
    return -ENOMEM; 

  /* memory allocation for rxIntCQ
   */
  rxIntCQ = (unsigned int *) kmalloc ((sizeof(unsigned int )*NUM_CHN_INT*NUM_RXBDS),GFP_KERNEL);
  if (rxIntCQ == NULL)
    return -ENOMEM; 
	
  /* txIntCQ and rxIntCQ initialization
   */
  memset((unsigned char  *)txIntCQ, 0,NUM_CHN_INT*NUM_TXBDS*4); /* clear interrupt queue areas */
  memset((unsigned char  *)rxIntCQ, 0,NUM_CHN_INT*NUM_RXBDS*4);
  *((unsigned int  *)txIntCQ + NUM_CHN_INT*NUM_TXBDS - 1) = 0x40000000; /* set wrap bit at end of TX intcq */
  *((unsigned int  *)rxIntCQ + NUM_CHN_INT*NUM_RXBDS - 1) = 0x40000000; /* set wrap bit at end of RX intcq */

#ifdef MCC_DEBUG
  printk("MCC:Init interrupt queue done\n");
#endif 	

  /* memory allocation for read buffer
   */
  ReadBuffer = (FrameBuffer *) kmalloc ( sizeof(FrameBuffer)*NUMBER_OF_CHANNELS, GFP_KERNEL);
  if (ReadBuffer == NULL)
    return -ENOMEM; 

  /* memory allocation for read buffer
   */
  WriteBuffer = (FrameBuffer *) kmalloc ( sizeof(FrameBuffer)*NUMBER_OF_CHANNELS, GFP_KERNEL);	
  if (WriteBuffer == NULL)
    return -ENOMEM;
		
#ifdef MCC_DEBUG
  printk("MCC:Data WriteBuffer pointer= %xH\n" ,(unsigned int )WriteBuffer);
  printk("MCC:Data ReadBuffer pointer= %xH\n" ,(unsigned int )ReadBuffer);
#endif 	


  /* main MCC and SI init
   */	
  mainInitMCC( (unsigned int ) immap);

#ifdef MCC_DEBUG
  printk("MCC:MCC and SI RAM configurated\n");
#endif 	
		
  /* Regist del driver
   */
  mcc_result = register_chrdev(mcc_major, "mcc", &mcc_fops);
  if ( mcc_result < 0 ) {
    printk(KERN_INFO "MCC:Can't get major %d.\n", mcc_major);
    kfree(BufferPool);
    return mcc_result;
  }
  if ( mcc_major == 0) mcc_major = mcc_result;

    
  // register mcc interrupt
  if (request_8xxirq(MCC_IRQ,mcc_ISR,0,"mcc",NULL)) {
    printk("MCC: Could not register interrupt %d.\n", MCC_IRQ);
    return -EBUSY;
  }
    
  /* startup TDM
   */
  startTxRx();

  printk("MCC:driver loaded\n");
	
  return 0;	
}
/*
 * Name:
 *	mcc_open
 * Description:
 *	open system call
 *
 */
int mcc_open(struct inode *inode,struct file *filp){
  if(MINOR(inode->i_rdev)>=NUMBER_OF_CHANNELS)
    return -ENODEV;
#ifdef SINGLE_USER
  if(MOD_IN_USE)
    return -EBUSY;
#endif
  MOD_INC_USE_COUNT;
  readNow[MINOR(inode->i_rdev)] = 0;
  writeNow[MINOR(inode->i_rdev)] = 0;
  valid[MINOR(inode->i_rdev)] = 0;
  return 0;
}/* mcc_open */

/*
 * Name:
 *	mcc_release
 * Description:
 *	close system call
 *
 */
#ifdef LINUX_20
void mcc_release(struct inode *inode,struct file *filp){
  MOD_DEC_USE_COUNT;
  mcc_fasync(-1, filp, 0);
  readNow[MINOR(inode->i_rdev)] = 0;
  writeNow[MINOR(inode->i_rdev)] = 0;
  valid[MINOR(inode->i_rdev)] = 0;
  return;
}
#else
int mcc_release(struct inode *inode,struct file *filp)
{
  MOD_DEC_USE_COUNT;
  mcc_fasync(-1, filp, 0);  
  readNow[MINOR(inode->i_rdev)] = 0;
  writeNow[MINOR(inode->i_rdev)] = 0;
  valid[MINOR(inode->i_rdev)] = 0;
  return 0;
}
#endif


#ifdef LINUX_20
int mcc_read(struct inode *inode,struct file *filp,char *buf,int count){
  return do_mcc_read(inode,filp,buf,count,&filp->f_pos);
}
#else
ssize_t mcc_read(struct file *filp,char *buf,size_t count,loff_t *f_pos){
  return do_mcc_read(filp->f_dentry->d_inode,filp,buf,count,f_pos);
}
#endif

/*
 * Name:
 *	do_mcc_read
 * Description:
 *	char driver read function
 *
 */
ssize_t do_mcc_read(struct inode *inode,struct file *filp,char *buffer,size_t size,loff_t *f_pos) {
  int count = 0;
  int device = MINOR(inode->i_rdev);
  FrameBuffer *read = (FrameBuffer *) ( (unsigned int ) ReadBuffer + (sizeof(FrameBuffer))*device);
	
  if(readNow[device] == 1){
    for(count = 0; count < BUFFER_SIZE; count++){
      put_user(read->data[count], buffer++);		
    }
    readNow[device] = 0;
    return BUFFER_SIZE;
  } 
  return 0;
} /* do_mcc_read */



#ifdef LINUX_20
int mcc_write(struct inode *inode,struct file *filp,const char *buf,int count){
  return do_mcc_write(inode,filp,buf,count,&filp->f_pos);
}
#else
ssize_t mcc_write(struct file *filp,const char *buf,size_t count,loff_t *f_pos){
  return do_mcc_write(filp->f_dentry->d_inode,filp,buf,count,f_pos);
}
#endif
/*
 * Name:
 *	do_mcc_write
 * Description:
 *	char driver write function
 *
 */
ssize_t do_mcc_write(struct inode *inode,struct file *filp,const char *buf,size_t size,loff_t *f_pos) {
  int device = MINOR(inode->i_rdev);
  int count = 0;
  FrameBuffer *write = (FrameBuffer *) ( (unsigned int ) WriteBuffer + (sizeof(FrameBuffer))*device);
#ifdef 	MCC_WRITE
  unsigned int *BoardIO = ioremap(0xF1000000,0x32);
  *BoardIO =~( 0x20000000) & reg0data;
  *BoardIO = 0x20000000 | reg0data;
#endif
  if ( writeNow[device] == 0){
    return 0;
  }
  while (count < BUFFER_SIZE) {
    get_user(write->data[count] ,buf++);
    count++;
  }
  valid[device] = 1;
  writeNow[device] = 0;
  return BUFFER_SIZE;
}/*end of do_mcc_write */



/*
 * Name:
 *	mcc_ioctl
 * Description:
 *	char driver ioctl function
 *
 */
int mcc_ioctl(struct inode *inode,struct file *filp,unsigned int cmd,unsigned long arg){
  int          err=0;

  if(_IOC_TYPE(cmd)!=MCC_IOC_MAGIC)
    return -ENOTTY;
  if(_IOC_NR(cmd)>MCC_IOC_MAXNR)
    return -ENOTTY;

#ifdef LINUX_20
  if(_IOC_DIR(cmd)&_IOC_READ)
    err=verify_area(VERIFY_WRITE,(void *)arg,_IOC_SIZE(cmd));
  else
    if(_IOC_DIR(cmd)&_IOC_WRITE)
      err=verify_area(VERIFY_READ,(void *)arg,_IOC_SIZE(cmd));
  if(err)
    return err;
#else
  if(_IOC_DIR(cmd)&_IOC_READ)
    err=!access_ok(VERIFY_WRITE,(void *)arg,_IOC_SIZE(cmd));
  else
    if(_IOC_DIR(cmd)&_IOC_WRITE)
      err=!access_ok(VERIFY_READ,(void *)arg,_IOC_SIZE(cmd));
  if(err)
    return -EFAULT;
#endif

  switch(cmd)
    {
    case MCC_IOC_TXBDS:
      {
	BDS_IOC *TxBdsIOCTL;
	TxBdsIOCTL = (BDS_IOC *) arg;
	BDsStatus(TX,NUM_TXBDS*TxBdsIOCTL->chn,&TxBdsIOCTL->bd_cstatus[0],&TxBdsIOCTL->bd_length[0],&TxBdsIOCTL->bd_addr[0]);
      }
      break;
    case MCC_IOC_RXBDS:
      {
	BDS_IOC *RxBdsIOCTL;
	RxBdsIOCTL = (BDS_IOC *) arg;
	BDsStatus(RX,NUM_RXBDS*RxBdsIOCTL->chn,&RxBdsIOCTL->bd_cstatus[0],&RxBdsIOCTL->bd_length[0],&RxBdsIOCTL->bd_addr[0]);
      }
      break;
    case SIXRAM_IOC_REGS:
      {
	SI_REGS_IOC *SiRegsIOCTL;
	SiRegsIOCTL = (SI_REGS_IOC *) arg;
	SiRamStatus(SiRegsIOCTL->chn,&SiRegsIOCTL->sigmr,&SiRegsIOCTL->sicmdr,&SiRegsIOCTL->sistr,
		    &SiRegsIOCTL->sirsr,&SiRegsIOCTL->mcce,&SiRegsIOCTL->mccm,&SiRegsIOCTL->mccr,
		    &SiRegsIOCTL->sixmr,&SiRegsIOCTL->tx_siram,&SiRegsIOCTL->rx_siram);
      }
      break;
    case CPM_IOC_REGS:
      {
	CPM_STATUS_IOC *CpmStatusRegisterIOCTL;
	CpmStatusRegisterIOCTL=(CPM_STATUS_IOC *) arg;
	CpmReadRegisters(CpmStatusRegisterIOCTL);
      }
      break;
    case TXRX_IOC_BUFF:
      {
	unsigned char offset;
	unsigned char index;
	unsigned char *txBuffer;
	unsigned char *rxBuffer;
	int count = 0;	
	TXRX_BUFF_IOC *TxRxBufferIOCTL;
	TxRxBufferIOCTL=(TXRX_BUFF_IOC *) arg;
	offset = TxRxBufferIOCTL->offset;															
	index = TxRxBufferIOCTL->index;
	rxBuffer = (unsigned char * )BufferPool + BUFFER_SIZE*NUMBER_OF_CHANNELS*NUM_TXBDS + (offset+index)*BUFFER_SIZE;
	txBuffer = (unsigned char * ) BufferPool+(offset+index)*BUFFER_SIZE;
	for(count = 0; count < BUFFER_SIZE; count++){
	  put_user(txBuffer[count],&TxRxBufferIOCTL->TxBuff[count]);		
	  put_user(rxBuffer[count],&TxRxBufferIOCTL->RxBuff[count]);		
	}
      }
      break;
    case MCC_FASYNC_IOC_DEAMON:
      {
	fasyncActive = TRUE;
      }
      break;
    case MCC_RESTART_CMD:
      {	
	RestartServiceRoutine();
      }
      break;
    default:
      return -ENOTTY;
    }
  return err;
} /* mcc_ioctl */

/*
 * Name:
 *	mccFasync
 * Description:
 *	call MCC fasync method from interrupt hadler.
 */
void mccFasync(void){
  if((async_queue) && (fasyncActive == TRUE) ){
    kill_fasync(&async_queue,SIGIO,POLL_IN);
  }
  	
	
}

/*
 * Name:
 *	mcc_fasync
 * Description:
 *	MCC fasync method
 */
int mcc_fasync(fasync_file fd,struct file *filp,int mode){
  return fasync_helper(fd,filp,mode,&async_queue);
}



/*
 * Name:
 *	mcc_ISR
 * Description:
 *	MCC Interrupt handler
 *
 
 void mcc_ISR(int irq, void *dev_id, struct pt_regs *regs) {
 do_mcc_ISR();
 }
*/

/*
 * linux interface
 *
 */
module_init(mcc_init);
module_exit(mcc_cleanup);


/*
 * device.h - mcc driver header file 
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

extern void     InitBDs(unsigned int, UBYTE);
extern void     MCCGlobalInit(void);
extern void     MCCChanSpecInit(unsigned int,UBYTE);
extern void     MCCExChanSpecInit(UHWORD,UHWORD,UHWORD);
extern void     InitTxRxParams(UHWORD);
extern void     SIinit(void);
extern void    	startTxRx(void);
extern void		stopTxRx(void);
extern void 	mainInitMCC(unsigned int );
extern void 	mcc_ISR(int , void *, struct pt_regs *);
extern void 	BDsStatus(unsigned char, unsigned char, unsigned short *, unsigned short *, unsigned int *);
extern void 	SiRamStatus(unsigned char,unsigned char*,unsigned char*,unsigned char*,unsigned short*,
					unsigned short*,unsigned short*,unsigned char*,unsigned short*,unsigned short*,unsigned short*);
extern void 	CpmReadRegisters(CPM_STATUS_IOC *);
extern void		RestartServiceRoutine(void);

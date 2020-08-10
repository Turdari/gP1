/* Copyright (C) 1999, 2001, 2002 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

#ifdef __ASSEMBLER__

/* Symbolic names for the registers.  The only portable way to write asm
   code is to use number but this produces really unreadable code.
   Therefore these symbolic names.  */

/* Integer registers.  */
#define r0	0
#define r1	1
#define r2	2
#define r3	3
#define r4	4
#define r5	5
#define r6	6
#define r7	7
#define r8	8
#define r9	9
#define r10	10
#define r11	11
#define r12	12
#define r13	13
#define r14	14
#define r15	15
#define r16	16
#define r17	17
#define r18	18
#define r19	19
#define r20	20
#define r21	21
#define r22	22
#define r23	23
#define r24	24
#define r25	25
#define r26	26
#define r27	27
#define r28	28
#define r29	29
#define r30	30
#define r31	31

/* Floating-point registers.  */
#define fp0	0
#define fp1	1
#define fp2	2
#define fp3	3
#define fp4	4
#define fp5	5
#define fp6	6
#define fp7	7
#define fp8	8
#define fp9	9
#define fp10	10
#define fp11	11
#define fp12	12
#define fp13	13
#define fp14	14
#define fp15	15
#define fp16	16
#define fp17	17
#define fp18	18
#define fp19	19
#define fp20	20
#define fp21	21
#define fp22	22
#define fp23	23
#define fp24	24
#define fp25	25
#define fp26	26
#define fp27	27
#define fp28	28
#define fp29	29
#define fp30	30
#define fp31	31

/* Condition code registers.  */
#define cr0	0
#define cr1	1
#define cr2	2
#define cr3	3
#define cr4	4
#define cr5	5
#define cr6	6
#define cr7	7


#ifdef __ELF__

/* This seems to always be the case on PPC.  */
#define ALIGNARG(log2) log2
/* For ELF we need the `.type' directive to make shared libs work right.  */
#define ASM_TYPE_DIRECTIVE(name,typearg) .type name,typearg;
#define ASM_SIZE_DIRECTIVE(name) .size name,.-name

#endif /* __ELF__ */


#endif	/* __ASSEMBLER__ */

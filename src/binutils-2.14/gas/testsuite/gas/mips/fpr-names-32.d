#objdump: -dr --prefix-addresses --show-raw-insn -M gpr-names=numeric,fpr-names=32
#name: MIPS FPR disassembly (32)
#source: fpr-names.s

# Check objdump's handling of -M fpr-names=foo options.

.*: +file format .*mips.*

Disassembly of section .text:
0+0000 <[^>]*> 44800000 	mtc1	\$0,fv0
0+0004 <[^>]*> 44800800 	mtc1	\$0,fv0f
0+0008 <[^>]*> 44801000 	mtc1	\$0,fv1
0+000c <[^>]*> 44801800 	mtc1	\$0,fv1f
0+0010 <[^>]*> 44802000 	mtc1	\$0,ft0
0+0014 <[^>]*> 44802800 	mtc1	\$0,ft0f
0+0018 <[^>]*> 44803000 	mtc1	\$0,ft1
0+001c <[^>]*> 44803800 	mtc1	\$0,ft1f
0+0020 <[^>]*> 44804000 	mtc1	\$0,ft2
0+0024 <[^>]*> 44804800 	mtc1	\$0,ft2f
0+0028 <[^>]*> 44805000 	mtc1	\$0,ft3
0+002c <[^>]*> 44805800 	mtc1	\$0,ft3f
0+0030 <[^>]*> 44806000 	mtc1	\$0,fa0
0+0034 <[^>]*> 44806800 	mtc1	\$0,fa0f
0+0038 <[^>]*> 44807000 	mtc1	\$0,fa1
0+003c <[^>]*> 44807800 	mtc1	\$0,fa1f
0+0040 <[^>]*> 44808000 	mtc1	\$0,ft4
0+0044 <[^>]*> 44808800 	mtc1	\$0,ft4f
0+0048 <[^>]*> 44809000 	mtc1	\$0,ft5
0+004c <[^>]*> 44809800 	mtc1	\$0,ft5f
0+0050 <[^>]*> 4480a000 	mtc1	\$0,fs0
0+0054 <[^>]*> 4480a800 	mtc1	\$0,fs0f
0+0058 <[^>]*> 4480b000 	mtc1	\$0,fs1
0+005c <[^>]*> 4480b800 	mtc1	\$0,fs1f
0+0060 <[^>]*> 4480c000 	mtc1	\$0,fs2
0+0064 <[^>]*> 4480c800 	mtc1	\$0,fs2f
0+0068 <[^>]*> 4480d000 	mtc1	\$0,fs3
0+006c <[^>]*> 4480d800 	mtc1	\$0,fs3f
0+0070 <[^>]*> 4480e000 	mtc1	\$0,fs4
0+0074 <[^>]*> 4480e800 	mtc1	\$0,fs4f
0+0078 <[^>]*> 4480f000 	mtc1	\$0,fs5
0+007c <[^>]*> 4480f800 	mtc1	\$0,fs5f
	\.\.\.

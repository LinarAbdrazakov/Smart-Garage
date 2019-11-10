
build/blank.elf:     file format elf32-littlearm

Sections:
Idx Name              Size      VMA       LMA       File off  Algn  Flags
  0 .isr_vector       000000c0  08000000  08000000  00010000  2**0  CONTENTS, ALLOC, LOAD, READONLY, DATA
  1 .text             00000304  080000c0  080000c0  000100c0  2**2  CONTENTS, ALLOC, LOAD, READONLY, CODE
  2 .rodata           00000004  080003c4  080003c4  000103c4  2**2  CONTENTS, ALLOC, LOAD, READONLY, DATA
  3 .init_array       00000008  080003c8  080003c8  000103c8  2**2  CONTENTS, ALLOC, LOAD, DATA
  4 .fini_array       00000004  080003d0  080003d0  000103d0  2**2  CONTENTS, ALLOC, LOAD, DATA
  5 .data             0000042c  20000000  080003d4  00020000  2**3  CONTENTS, ALLOC, LOAD, DATA
  6 .bss              00000040  2000042c  08000800  0002042c  2**2  ALLOC
  7 ._user_heap_stack 00000604  2000046c  08000800  0002046c  2**0  ALLOC
  8 .ARM.attributes   00000028  00000000  00000000  0002042c  2**0  CONTENTS, READONLY
  9 .debug_info       0000033b  00000000  00000000  00020454  2**0  CONTENTS, READONLY, DEBUGGING
 10 .debug_abbrev     00000168  00000000  00000000  0002078f  2**0  CONTENTS, READONLY, DEBUGGING
 11 .debug_loc        00000140  00000000  00000000  000208f7  2**0  CONTENTS, READONLY, DEBUGGING
 12 .debug_aranges    00000080  00000000  00000000  00020a37  2**0  CONTENTS, READONLY, DEBUGGING
 13 .debug_ranges     00000050  00000000  00000000  00020ab7  2**0  CONTENTS, READONLY, DEBUGGING
 14 .debug_line       000001b2  00000000  00000000  00020b07  2**0  CONTENTS, READONLY, DEBUGGING
 15 .debug_str        00000291  00000000  00000000  00020cb9  2**0  CONTENTS, READONLY, DEBUGGING
 16 .comment          0000007f  00000000  00000000  00020f4a  2**0  CONTENTS, READONLY
 17 .debug_frame      000002b8  00000000  00000000  00020fcc  2**2  CONTENTS, READONLY, DEBUGGING

Disassembly of section .text:

080000c0 <__do_global_dtors_aux>:
 80000c0:	b510      	push	{r4, lr}
 80000c2:	4c06      	ldr	r4, [pc, #24]	; (80000dc <__do_global_dtors_aux+0x1c>)
 80000c4:	7823      	ldrb	r3, [r4, #0]
 80000c6:	2b00      	cmp	r3, #0
 80000c8:	d107      	bne.n	80000da <__do_global_dtors_aux+0x1a>
 80000ca:	4b05      	ldr	r3, [pc, #20]	; (80000e0 <__do_global_dtors_aux+0x20>)
 80000cc:	2b00      	cmp	r3, #0
 80000ce:	d002      	beq.n	80000d6 <__do_global_dtors_aux+0x16>
 80000d0:	4804      	ldr	r0, [pc, #16]	; (80000e4 <__do_global_dtors_aux+0x24>)
 80000d2:	e000      	b.n	80000d6 <__do_global_dtors_aux+0x16>
 80000d4:	bf00      	nop
 80000d6:	2301      	movs	r3, #1
 80000d8:	7023      	strb	r3, [r4, #0]
 80000da:	bd10      	pop	{r4, pc}
 80000dc:	2000042c 	.word	0x2000042c
 80000e0:	00000000 	.word	0x00000000
 80000e4:	080003ac 	.word	0x080003ac

080000e8 <frame_dummy>:
 80000e8:	4b04      	ldr	r3, [pc, #16]	; (80000fc <frame_dummy+0x14>)
 80000ea:	b510      	push	{r4, lr}
 80000ec:	2b00      	cmp	r3, #0
 80000ee:	d003      	beq.n	80000f8 <frame_dummy+0x10>
 80000f0:	4903      	ldr	r1, [pc, #12]	; (8000100 <frame_dummy+0x18>)
 80000f2:	4804      	ldr	r0, [pc, #16]	; (8000104 <frame_dummy+0x1c>)
 80000f4:	e000      	b.n	80000f8 <frame_dummy+0x10>
 80000f6:	bf00      	nop
 80000f8:	bd10      	pop	{r4, pc}
 80000fa:	46c0      	nop			; (mov r8, r8)
 80000fc:	00000000 	.word	0x00000000
 8000100:	20000430 	.word	0x20000430
 8000104:	080003ac 	.word	0x080003ac

08000108 <Reset_Handler>:
 8000108:	480d      	ldr	r0, [pc, #52]	; (8000140 <LoopForever+0x2>)
 800010a:	4685      	mov	sp, r0
 800010c:	480d      	ldr	r0, [pc, #52]	; (8000144 <LoopForever+0x6>)
 800010e:	490e      	ldr	r1, [pc, #56]	; (8000148 <LoopForever+0xa>)
 8000110:	4a0e      	ldr	r2, [pc, #56]	; (800014c <LoopForever+0xe>)
 8000112:	2300      	movs	r3, #0
 8000114:	e002      	b.n	800011c <LoopCopyDataInit>

08000116 <CopyDataInit>:
 8000116:	58d4      	ldr	r4, [r2, r3]
 8000118:	50c4      	str	r4, [r0, r3]
 800011a:	3304      	adds	r3, #4

0800011c <LoopCopyDataInit>:
 800011c:	18c4      	adds	r4, r0, r3
 800011e:	428c      	cmp	r4, r1
 8000120:	d3f9      	bcc.n	8000116 <CopyDataInit>
 8000122:	4a0b      	ldr	r2, [pc, #44]	; (8000150 <LoopForever+0x12>)
 8000124:	4c0b      	ldr	r4, [pc, #44]	; (8000154 <LoopForever+0x16>)
 8000126:	2300      	movs	r3, #0
 8000128:	e001      	b.n	800012e <LoopFillZerobss>

0800012a <FillZerobss>:
 800012a:	6013      	str	r3, [r2, #0]
 800012c:	3204      	adds	r2, #4

0800012e <LoopFillZerobss>:
 800012e:	42a2      	cmp	r2, r4
 8000130:	d3fb      	bcc.n	800012a <FillZerobss>
 8000132:	f000 f815 	bl	8000160 <SystemInit>
 8000136:	f000 f869 	bl	800020c <__libc_init_array>
 800013a:	f000 f80e 	bl	800015a <main>

0800013e <LoopForever>:
 800013e:	e7fe      	b.n	800013e <LoopForever>
 8000140:	20002000 	.word	0x20002000
 8000144:	20000000 	.word	0x20000000
 8000148:	2000042c 	.word	0x2000042c
 800014c:	080003d4 	.word	0x080003d4
 8000150:	2000042c 	.word	0x2000042c
 8000154:	2000046c 	.word	0x2000046c

08000158 <ADC1_COMP_IRQHandler>:
 8000158:	e7fe      	b.n	8000158 <ADC1_COMP_IRQHandler>

0800015a <main>:
 800015a:	b580      	push	{r7, lr}
 800015c:	af00      	add	r7, sp, #0
 800015e:	e7fe      	b.n	800015e <main+0x4>

08000160 <SystemInit>:
 8000160:	b580      	push	{r7, lr}
 8000162:	af00      	add	r7, sp, #0
 8000164:	4b1a      	ldr	r3, [pc, #104]	; (80001d0 <SystemInit+0x70>)
 8000166:	681a      	ldr	r2, [r3, #0]
 8000168:	4b19      	ldr	r3, [pc, #100]	; (80001d0 <SystemInit+0x70>)
 800016a:	2101      	movs	r1, #1
 800016c:	430a      	orrs	r2, r1
 800016e:	601a      	str	r2, [r3, #0]
 8000170:	4b17      	ldr	r3, [pc, #92]	; (80001d0 <SystemInit+0x70>)
 8000172:	685a      	ldr	r2, [r3, #4]
 8000174:	4b16      	ldr	r3, [pc, #88]	; (80001d0 <SystemInit+0x70>)
 8000176:	4917      	ldr	r1, [pc, #92]	; (80001d4 <SystemInit+0x74>)
 8000178:	400a      	ands	r2, r1
 800017a:	605a      	str	r2, [r3, #4]
 800017c:	4b14      	ldr	r3, [pc, #80]	; (80001d0 <SystemInit+0x70>)
 800017e:	681a      	ldr	r2, [r3, #0]
 8000180:	4b13      	ldr	r3, [pc, #76]	; (80001d0 <SystemInit+0x70>)
 8000182:	4915      	ldr	r1, [pc, #84]	; (80001d8 <SystemInit+0x78>)
 8000184:	400a      	ands	r2, r1
 8000186:	601a      	str	r2, [r3, #0]
 8000188:	4b11      	ldr	r3, [pc, #68]	; (80001d0 <SystemInit+0x70>)
 800018a:	681a      	ldr	r2, [r3, #0]
 800018c:	4b10      	ldr	r3, [pc, #64]	; (80001d0 <SystemInit+0x70>)
 800018e:	4913      	ldr	r1, [pc, #76]	; (80001dc <SystemInit+0x7c>)
 8000190:	400a      	ands	r2, r1
 8000192:	601a      	str	r2, [r3, #0]
 8000194:	4b0e      	ldr	r3, [pc, #56]	; (80001d0 <SystemInit+0x70>)
 8000196:	685a      	ldr	r2, [r3, #4]
 8000198:	4b0d      	ldr	r3, [pc, #52]	; (80001d0 <SystemInit+0x70>)
 800019a:	4911      	ldr	r1, [pc, #68]	; (80001e0 <SystemInit+0x80>)
 800019c:	400a      	ands	r2, r1
 800019e:	605a      	str	r2, [r3, #4]
 80001a0:	4b0b      	ldr	r3, [pc, #44]	; (80001d0 <SystemInit+0x70>)
 80001a2:	6ada      	ldr	r2, [r3, #44]	; 0x2c
 80001a4:	4b0a      	ldr	r3, [pc, #40]	; (80001d0 <SystemInit+0x70>)
 80001a6:	210f      	movs	r1, #15
 80001a8:	438a      	bics	r2, r1
 80001aa:	62da      	str	r2, [r3, #44]	; 0x2c
 80001ac:	4b08      	ldr	r3, [pc, #32]	; (80001d0 <SystemInit+0x70>)
 80001ae:	6b1a      	ldr	r2, [r3, #48]	; 0x30
 80001b0:	4b07      	ldr	r3, [pc, #28]	; (80001d0 <SystemInit+0x70>)
 80001b2:	490c      	ldr	r1, [pc, #48]	; (80001e4 <SystemInit+0x84>)
 80001b4:	400a      	ands	r2, r1
 80001b6:	631a      	str	r2, [r3, #48]	; 0x30
 80001b8:	4b05      	ldr	r3, [pc, #20]	; (80001d0 <SystemInit+0x70>)
 80001ba:	6b5a      	ldr	r2, [r3, #52]	; 0x34
 80001bc:	4b04      	ldr	r3, [pc, #16]	; (80001d0 <SystemInit+0x70>)
 80001be:	2101      	movs	r1, #1
 80001c0:	438a      	bics	r2, r1
 80001c2:	635a      	str	r2, [r3, #52]	; 0x34
 80001c4:	4b02      	ldr	r3, [pc, #8]	; (80001d0 <SystemInit+0x70>)
 80001c6:	2200      	movs	r2, #0
 80001c8:	609a      	str	r2, [r3, #8]
 80001ca:	46c0      	nop			; (mov r8, r8)
 80001cc:	46bd      	mov	sp, r7
 80001ce:	bd80      	pop	{r7, pc}
 80001d0:	40021000 	.word	0x40021000
 80001d4:	f8ffb80c 	.word	0xf8ffb80c
 80001d8:	fef6ffff 	.word	0xfef6ffff
 80001dc:	fffbffff 	.word	0xfffbffff
 80001e0:	ffc0ffff 	.word	0xffc0ffff
 80001e4:	fffffeac 	.word	0xfffffeac

080001e8 <NMI_Handler>:
 80001e8:	b580      	push	{r7, lr}
 80001ea:	af00      	add	r7, sp, #0
 80001ec:	46c0      	nop			; (mov r8, r8)
 80001ee:	46bd      	mov	sp, r7
 80001f0:	bd80      	pop	{r7, pc}

080001f2 <HardFault_Handler>:
 80001f2:	b580      	push	{r7, lr}
 80001f4:	af00      	add	r7, sp, #0
 80001f6:	e7fe      	b.n	80001f6 <HardFault_Handler+0x4>

080001f8 <SVC_Handler>:
 80001f8:	b580      	push	{r7, lr}
 80001fa:	af00      	add	r7, sp, #0
 80001fc:	46c0      	nop			; (mov r8, r8)
 80001fe:	46bd      	mov	sp, r7
 8000200:	bd80      	pop	{r7, pc}

08000202 <PendSV_Handler>:
 8000202:	b580      	push	{r7, lr}
 8000204:	af00      	add	r7, sp, #0
 8000206:	46c0      	nop			; (mov r8, r8)
 8000208:	46bd      	mov	sp, r7
 800020a:	bd80      	pop	{r7, pc}

0800020c <__libc_init_array>:
 800020c:	b570      	push	{r4, r5, r6, lr}
 800020e:	4e0d      	ldr	r6, [pc, #52]	; (8000244 <__libc_init_array+0x38>)
 8000210:	4d0d      	ldr	r5, [pc, #52]	; (8000248 <__libc_init_array+0x3c>)
 8000212:	1bad      	subs	r5, r5, r6
 8000214:	10ad      	asrs	r5, r5, #2
 8000216:	d006      	beq.n	8000226 <__libc_init_array+0x1a>
 8000218:	2400      	movs	r4, #0
 800021a:	00a3      	lsls	r3, r4, #2
 800021c:	58f3      	ldr	r3, [r6, r3]
 800021e:	3401      	adds	r4, #1
 8000220:	4798      	blx	r3
 8000222:	42a5      	cmp	r5, r4
 8000224:	d1f9      	bne.n	800021a <__libc_init_array+0xe>
 8000226:	f000 f8c1 	bl	80003ac <_init>
 800022a:	4e08      	ldr	r6, [pc, #32]	; (800024c <__libc_init_array+0x40>)
 800022c:	4d08      	ldr	r5, [pc, #32]	; (8000250 <__libc_init_array+0x44>)
 800022e:	1bad      	subs	r5, r5, r6
 8000230:	10ad      	asrs	r5, r5, #2
 8000232:	d006      	beq.n	8000242 <__libc_init_array+0x36>
 8000234:	2400      	movs	r4, #0
 8000236:	00a3      	lsls	r3, r4, #2
 8000238:	58f3      	ldr	r3, [r6, r3]
 800023a:	3401      	adds	r4, #1
 800023c:	4798      	blx	r3
 800023e:	42a5      	cmp	r5, r4
 8000240:	d1f9      	bne.n	8000236 <__libc_init_array+0x2a>
 8000242:	bd70      	pop	{r4, r5, r6, pc}
 8000244:	080003c8 	.word	0x080003c8
 8000248:	080003c8 	.word	0x080003c8
 800024c:	080003c8 	.word	0x080003c8
 8000250:	080003d0 	.word	0x080003d0

08000254 <register_fini>:
 8000254:	4b03      	ldr	r3, [pc, #12]	; (8000264 <register_fini+0x10>)
 8000256:	b510      	push	{r4, lr}
 8000258:	2b00      	cmp	r3, #0
 800025a:	d002      	beq.n	8000262 <register_fini+0xe>
 800025c:	4802      	ldr	r0, [pc, #8]	; (8000268 <register_fini+0x14>)
 800025e:	f000 f805 	bl	800026c <atexit>
 8000262:	bd10      	pop	{r4, pc}
 8000264:	00000000 	.word	0x00000000
 8000268:	0800027d 	.word	0x0800027d

0800026c <atexit>:
 800026c:	b510      	push	{r4, lr}
 800026e:	0001      	movs	r1, r0
 8000270:	2300      	movs	r3, #0
 8000272:	2200      	movs	r2, #0
 8000274:	2000      	movs	r0, #0
 8000276:	f000 f81f 	bl	80002b8 <__register_exitproc>
 800027a:	bd10      	pop	{r4, pc}

0800027c <__libc_fini_array>:
 800027c:	b570      	push	{r4, r5, r6, lr}
 800027e:	4b09      	ldr	r3, [pc, #36]	; (80002a4 <__libc_fini_array+0x28>)
 8000280:	4c09      	ldr	r4, [pc, #36]	; (80002a8 <__libc_fini_array+0x2c>)
 8000282:	1ae4      	subs	r4, r4, r3
 8000284:	10a4      	asrs	r4, r4, #2
 8000286:	d009      	beq.n	800029c <__libc_fini_array+0x20>
 8000288:	4a08      	ldr	r2, [pc, #32]	; (80002ac <__libc_fini_array+0x30>)
 800028a:	18a5      	adds	r5, r4, r2
 800028c:	00ad      	lsls	r5, r5, #2
 800028e:	18ed      	adds	r5, r5, r3
 8000290:	682b      	ldr	r3, [r5, #0]
 8000292:	3c01      	subs	r4, #1
 8000294:	4798      	blx	r3
 8000296:	3d04      	subs	r5, #4
 8000298:	2c00      	cmp	r4, #0
 800029a:	d1f9      	bne.n	8000290 <__libc_fini_array+0x14>
 800029c:	f000 f88c 	bl	80003b8 <_fini>
 80002a0:	bd70      	pop	{r4, r5, r6, pc}
 80002a2:	46c0      	nop			; (mov r8, r8)
 80002a4:	080003d0 	.word	0x080003d0
 80002a8:	080003d4 	.word	0x080003d4
 80002ac:	3fffffff 	.word	0x3fffffff

080002b0 <__retarget_lock_acquire_recursive>:
 80002b0:	4770      	bx	lr
 80002b2:	46c0      	nop			; (mov r8, r8)

080002b4 <__retarget_lock_release_recursive>:
 80002b4:	4770      	bx	lr
 80002b6:	46c0      	nop			; (mov r8, r8)

080002b8 <__register_exitproc>:
 80002b8:	b5f0      	push	{r4, r5, r6, r7, lr}
 80002ba:	464e      	mov	r6, r9
 80002bc:	4645      	mov	r5, r8
 80002be:	46de      	mov	lr, fp
 80002c0:	4657      	mov	r7, sl
 80002c2:	b5e0      	push	{r5, r6, r7, lr}
 80002c4:	4d36      	ldr	r5, [pc, #216]	; (80003a0 <__register_exitproc+0xe8>)
 80002c6:	b083      	sub	sp, #12
 80002c8:	0006      	movs	r6, r0
 80002ca:	6828      	ldr	r0, [r5, #0]
 80002cc:	4698      	mov	r8, r3
 80002ce:	000f      	movs	r7, r1
 80002d0:	4691      	mov	r9, r2
 80002d2:	f7ff ffed 	bl	80002b0 <__retarget_lock_acquire_recursive>
 80002d6:	4b33      	ldr	r3, [pc, #204]	; (80003a4 <__register_exitproc+0xec>)
 80002d8:	681c      	ldr	r4, [r3, #0]
 80002da:	23a4      	movs	r3, #164	; 0xa4
 80002dc:	005b      	lsls	r3, r3, #1
 80002de:	58e0      	ldr	r0, [r4, r3]
 80002e0:	2800      	cmp	r0, #0
 80002e2:	d052      	beq.n	800038a <__register_exitproc+0xd2>
 80002e4:	6843      	ldr	r3, [r0, #4]
 80002e6:	2b1f      	cmp	r3, #31
 80002e8:	dc13      	bgt.n	8000312 <__register_exitproc+0x5a>
 80002ea:	1c5a      	adds	r2, r3, #1
 80002ec:	9201      	str	r2, [sp, #4]
 80002ee:	2e00      	cmp	r6, #0
 80002f0:	d128      	bne.n	8000344 <__register_exitproc+0x8c>
 80002f2:	9a01      	ldr	r2, [sp, #4]
 80002f4:	3302      	adds	r3, #2
 80002f6:	009b      	lsls	r3, r3, #2
 80002f8:	6042      	str	r2, [r0, #4]
 80002fa:	501f      	str	r7, [r3, r0]
 80002fc:	6828      	ldr	r0, [r5, #0]
 80002fe:	f7ff ffd9 	bl	80002b4 <__retarget_lock_release_recursive>
 8000302:	2000      	movs	r0, #0
 8000304:	b003      	add	sp, #12
 8000306:	bc3c      	pop	{r2, r3, r4, r5}
 8000308:	4690      	mov	r8, r2
 800030a:	4699      	mov	r9, r3
 800030c:	46a2      	mov	sl, r4
 800030e:	46ab      	mov	fp, r5
 8000310:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8000312:	4b25      	ldr	r3, [pc, #148]	; (80003a8 <__register_exitproc+0xf0>)
 8000314:	2b00      	cmp	r3, #0
 8000316:	d03d      	beq.n	8000394 <__register_exitproc+0xdc>
 8000318:	20c8      	movs	r0, #200	; 0xc8
 800031a:	0040      	lsls	r0, r0, #1
 800031c:	e000      	b.n	8000320 <__register_exitproc+0x68>
 800031e:	bf00      	nop
 8000320:	2800      	cmp	r0, #0
 8000322:	d037      	beq.n	8000394 <__register_exitproc+0xdc>
 8000324:	22a4      	movs	r2, #164	; 0xa4
 8000326:	2300      	movs	r3, #0
 8000328:	0052      	lsls	r2, r2, #1
 800032a:	58a1      	ldr	r1, [r4, r2]
 800032c:	6043      	str	r3, [r0, #4]
 800032e:	6001      	str	r1, [r0, #0]
 8000330:	50a0      	str	r0, [r4, r2]
 8000332:	3240      	adds	r2, #64	; 0x40
 8000334:	5083      	str	r3, [r0, r2]
 8000336:	3204      	adds	r2, #4
 8000338:	5083      	str	r3, [r0, r2]
 800033a:	3301      	adds	r3, #1
 800033c:	9301      	str	r3, [sp, #4]
 800033e:	2300      	movs	r3, #0
 8000340:	2e00      	cmp	r6, #0
 8000342:	d0d6      	beq.n	80002f2 <__register_exitproc+0x3a>
 8000344:	009a      	lsls	r2, r3, #2
 8000346:	4692      	mov	sl, r2
 8000348:	4482      	add	sl, r0
 800034a:	464a      	mov	r2, r9
 800034c:	2188      	movs	r1, #136	; 0x88
 800034e:	4654      	mov	r4, sl
 8000350:	5062      	str	r2, [r4, r1]
 8000352:	22c4      	movs	r2, #196	; 0xc4
 8000354:	0052      	lsls	r2, r2, #1
 8000356:	4691      	mov	r9, r2
 8000358:	4481      	add	r9, r0
 800035a:	464a      	mov	r2, r9
 800035c:	3987      	subs	r1, #135	; 0x87
 800035e:	4099      	lsls	r1, r3
 8000360:	6812      	ldr	r2, [r2, #0]
 8000362:	468b      	mov	fp, r1
 8000364:	430a      	orrs	r2, r1
 8000366:	4694      	mov	ip, r2
 8000368:	464a      	mov	r2, r9
 800036a:	4661      	mov	r1, ip
 800036c:	6011      	str	r1, [r2, #0]
 800036e:	2284      	movs	r2, #132	; 0x84
 8000370:	4641      	mov	r1, r8
 8000372:	0052      	lsls	r2, r2, #1
 8000374:	50a1      	str	r1, [r4, r2]
 8000376:	2e02      	cmp	r6, #2
 8000378:	d1bb      	bne.n	80002f2 <__register_exitproc+0x3a>
 800037a:	0002      	movs	r2, r0
 800037c:	465c      	mov	r4, fp
 800037e:	328d      	adds	r2, #141	; 0x8d
 8000380:	32ff      	adds	r2, #255	; 0xff
 8000382:	6811      	ldr	r1, [r2, #0]
 8000384:	430c      	orrs	r4, r1
 8000386:	6014      	str	r4, [r2, #0]
 8000388:	e7b3      	b.n	80002f2 <__register_exitproc+0x3a>
 800038a:	0020      	movs	r0, r4
 800038c:	304d      	adds	r0, #77	; 0x4d
 800038e:	30ff      	adds	r0, #255	; 0xff
 8000390:	50e0      	str	r0, [r4, r3]
 8000392:	e7a7      	b.n	80002e4 <__register_exitproc+0x2c>
 8000394:	6828      	ldr	r0, [r5, #0]
 8000396:	f7ff ff8d 	bl	80002b4 <__retarget_lock_release_recursive>
 800039a:	2001      	movs	r0, #1
 800039c:	4240      	negs	r0, r0
 800039e:	e7b1      	b.n	8000304 <__register_exitproc+0x4c>
 80003a0:	20000428 	.word	0x20000428
 80003a4:	080003c4 	.word	0x080003c4
 80003a8:	00000000 	.word	0x00000000

080003ac <_init>:
 80003ac:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
 80003ae:	46c0      	nop			; (mov r8, r8)
 80003b0:	bcf8      	pop	{r3, r4, r5, r6, r7}
 80003b2:	bc08      	pop	{r3}
 80003b4:	469e      	mov	lr, r3
 80003b6:	4770      	bx	lr

080003b8 <_fini>:
 80003b8:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
 80003ba:	46c0      	nop			; (mov r8, r8)
 80003bc:	bcf8      	pop	{r3, r4, r5, r6, r7}
 80003be:	bc08      	pop	{r3}
 80003c0:	469e      	mov	lr, r3
 80003c2:	4770      	bx	lr

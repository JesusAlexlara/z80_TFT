;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jul 12 2014) (Linux)
; This file was generated Mon Nov 30 13:17:10 2015
;--------------------------------------------------------
	.module main
	.optsdcc -mz80
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _isr_vector38
	.globl _isr_vector66
	.globl _UartReadLine
	.globl _UartPrint
	.globl _UartRead
	.globl _UartWrite
	.globl _UartInit
	.globl _io_read
	.globl _io_write
	.globl _printf
	.globl _isprint
	.globl _corrimiento
	.globl _putchar
	.globl _getchar
	.globl _system_init
	.globl _delay_ms
	.globl _teclado_read
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
_PPI_PORTA	=	0x0040
_PPI_PORTB	=	0x0041
_PPI_PORTC	=	0x0042
_PPI_CTRL	=	0x0043
_URBR	=	0x0020
_UTHR	=	0x0020
_UIER	=	0x0021
_UIIR	=	0x0022
_ULCR	=	0x0023
_ULSR	=	0x0025
_UMCR	=	0x0024
_UMSR	=	0x0026
_UDLL	=	0x0020
_UDLM	=	0x0021
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _DATA
_io_val:
	.ds 1
_io_addr:
	.ds 1
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _INITIALIZED
_corrimiento::
	.ds 1
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area _DABS (ABS)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area _HOME
	.area _GSINIT
	.area _GSFINAL
	.area _GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area _HOME
	.area _HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area _CODE
;z80io.h:272: void io_write(byte addr, byte data){
;	---------------------------------
; Function io_write
; ---------------------------------
_io_write_start::
_io_write:
;z80io.h:274: io_addr=addr;
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	ld	iy,#_io_addr
	ld	0 (iy),a
;z80io.h:275: io_val=data;
	ld	iy,#3
	add	iy,sp
	ld	a,0 (iy)
	ld	iy,#_io_val
	ld	0 (iy),a
;z80io.h:289: __endasm;
	EXX
	EX AF,AF'
	LD A,(_io_val)
	LD BC,(_io_addr)
	OUT (C),A
	EX AF,AF'
	EXX
	ret
_io_write_end::
;z80io.h:292: char io_read(byte addr){
;	---------------------------------
; Function io_read
; ---------------------------------
_io_read_start::
_io_read:
;z80io.h:294: io_addr=addr;
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	ld	iy,#_io_addr
	ld	0 (iy),a
;z80io.h:308: __endasm;
	EXX
	EX AF,AF'
	LD BC,(_io_addr)
	IN A,(C)
	LD (_io_val),A
	EX AF,AF'
	EXX
;z80io.h:310: return io_val;
	ld	iy,#_io_val
	ld	l,0 (iy)
	ret
_io_read_end::
;z80uart.h:64: void UartInit(){
;	---------------------------------
; Function UartInit
; ---------------------------------
_UartInit_start::
_UartInit:
;z80uart.h:67: ULCR = UDLAB;
	ld	a,#0x80
	out	(_ULCR),a
;z80uart.h:69: UDLL = UART_DIV_VALUE();
	ld	a,#0x1A
	out	(_UDLL),a
;z80uart.h:71: UDLM = UART_DIV_VALUE() >> 8;
	ld	a,#0x00
	out	(_UDLM),a
;z80uart.h:73: ULCR = UWLS0 | UWLS1;
	ld	a,#0x03
	out	(_ULCR),a
;z80uart.h:76: UIER = UERBFI;
	ld	a,#0x01
	out	(_UIER),a
	ret
_UartInit_end::
;z80uart.h:80: void UartWrite(char c){
;	---------------------------------
; Function UartWrite
; ---------------------------------
_UartWrite_start::
_UartWrite:
;z80uart.h:84: while( !(io_read(UART_LSR_ADDR) & UTEMT) );
00101$:
	ld	a,#0x25
	push	af
	inc	sp
	call	_io_read
	inc	sp
	bit	6, l
	jr	Z,00101$
;z80uart.h:85: NOP();    
	NOP
;z80uart.h:87: io_write(UART_THR_ADDR,c);
	ld	hl, #2+0
	add	hl, sp
	ld	d, (hl)
	ld	e,#0x20
	push	de
	call	_io_write
	pop	af
;z80uart.h:90: for(i=0;i<0xF0;i++)
	ld	de,#0x00F0
00107$:
;z80uart.h:91: __asm__("nop");
	nop
	dec	de
;z80uart.h:90: for(i=0;i<0xF0;i++)
	ld	a,d
	or	a,e
	jr	NZ,00107$
	ret
_UartWrite_end::
;z80uart.h:94: char UartRead(){
;	---------------------------------
; Function UartRead
; ---------------------------------
_UartRead_start::
_UartRead:
;z80uart.h:97: while( !(io_read(UART_LSR_ADDR) & UDR) );    
00101$:
	ld	a,#0x25
	push	af
	inc	sp
	call	_io_read
	inc	sp
	ld	a,l
	rrca
	jr	NC,00101$
;z80uart.h:98: NOP();
	NOP
;z80uart.h:100: return io_read(UART_RBR_ADDR);
	ld	a,#0x20
	push	af
	inc	sp
	call	_io_read
	inc	sp
	ret
_UartRead_end::
;z80uart.h:104: void UartPrint(const char* str){
;	---------------------------------
; Function UartPrint
; ---------------------------------
_UartPrint_start::
_UartPrint:
;z80uart.h:106: while(*str)       
	pop	bc
	pop	hl
	push	hl
	push	bc
00101$:
	ld	a,(hl)
	or	a, a
	ret	Z
;z80uart.h:107: putchar(*str++);
	inc	hl
	push	hl
	push	af
	inc	sp
	call	_putchar
	inc	sp
	pop	hl
	jr	00101$
	ret
_UartPrint_end::
;z80uart.h:111: void UartReadLine(char* str){
;	---------------------------------
; Function UartReadLine
; ---------------------------------
_UartReadLine_start::
_UartReadLine:
	push	ix
	ld	ix,#0
	add	ix,sp
;z80uart.h:113: int n=0;
	ld	bc,#0x0000
;z80uart.h:115: while(n<MAXREAD-1 && (c=getchar()) != '\n' && c !='\r'){
00111$:
	ld	a,c
	sub	a, #0x63
	ld	a,b
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	NC,00113$
	push	bc
	call	_getchar
	ld	a,l
	pop	bc
	ld	d,a
	sub	a, #0x0A
	jr	Z,00113$
;z80uart.h:117: if(c == 0x7f || c == 0x08){
	ld	a,d
	cp	a,#0x0D
	jr	Z,00113$
	cp	a,#0x7F
	jr	Z,00105$
	sub	a, #0x08
	jr	NZ,00106$
00105$:
;z80uart.h:119: if(n>0){
	xor	a, a
	cp	a, c
	sbc	a, b
	jp	PO, 00149$
	xor	a, #0x80
00149$:
	jp	P,00111$
;z80uart.h:120: str[--n]='\0';
	dec	bc
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	(hl),#0x00
;z80uart.h:121: putchar(c);
	push	bc
	push	de
	push	de
	inc	sp
	call	_putchar
	inc	sp
	ld	a,#0x20
	push	af
	inc	sp
	call	_putchar
	inc	sp
	inc	sp
	call	_putchar
	inc	sp
	pop	bc
	jr	00111$
00106$:
;z80uart.h:127: if(isprint(c))
	ld	a,d
	ld	l,a
	rla
	sbc	a, a
	ld	h,a
	push	bc
	push	de
	push	hl
	call	_isprint
	pop	af
	pop	de
	pop	bc
	ld	a,h
	or	a,l
	jr	Z,00111$
;z80uart.h:129: str[n++]=c;
	ld	h,c
	ld	e,b
	inc	bc
	ld	a,h
	add	a, 4 (ix)
	ld	l,a
	ld	a,e
	adc	a, 5 (ix)
	ld	h,a
	ld	(hl),d
;z80uart.h:130: putchar(c);
	push	bc
	push	de
	inc	sp
	call	_putchar
	inc	sp
	pop	bc
	jp	00111$
00113$:
;z80uart.h:134: putchar('\n');
	ld	a,#0x0A
	push	af
	inc	sp
	call	_putchar
	inc	sp
	pop	ix
	ret
_UartReadLine_end::
;z80uart.h:136: void putchar(char c ){
;	---------------------------------
; Function putchar
; ---------------------------------
_putchar_start::
_putchar:
;z80uart.h:138: if(c=='\n')
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	sub	a, #0x0A
	jr	NZ,00102$
;z80uart.h:139: UartWrite('\r');
	ld	a,#0x0D
	push	af
	inc	sp
	call	_UartWrite
	inc	sp
00102$:
;z80uart.h:141: UartWrite(c);
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	push	af
	inc	sp
	call	_UartWrite
	inc	sp
	ret
_putchar_end::
;z80uart.h:144: char getchar(){
;	---------------------------------
; Function getchar
; ---------------------------------
_getchar_start::
_getchar:
;z80uart.h:145: return UartRead();
	jp	_UartRead
_getchar_end::
;main.c:38: ISR_NMI(){
;	---------------------------------
; Function isr_vector66
; ---------------------------------
_isr_vector66_start::
_isr_vector66:
	push	af
	push	bc
	push	de
	push	hl
	push	iy
;main.c:41: }
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	retn
_isr_vector66_end::
;main.c:43: ISR_INT_38(){
;	---------------------------------
; Function isr_vector38
; ---------------------------------
_isr_vector38_start::
_isr_vector38:
	push	af
	push	bc
	push	de
	push	hl
	push	iy
;main.c:44: teclado_read();
	call	_teclado_read
;main.c:45: EI();
	EI
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	reti
_isr_vector38_end::
;main.c:50: int main(){
;	---------------------------------
; Function main
; ---------------------------------
_main_start::
_main:
	ld	hl,#-16
	add	hl,sp
	ld	sp,hl
;main.c:51: char KEYS[4][4] = 
	ld	hl,#0x0000
	add	hl,sp
	ld	(hl),#0x31
	ld	hl,#0x0000
	add	hl,sp
	ld	e,l
	ld	d,h
	inc	hl
	ld	(hl),#0x32
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	ld	(hl),#0x33
	ld	l,e
	ld	h,d
	inc	hl
	inc	hl
	inc	hl
	ld	(hl),#0x41
	ld	hl,#0x0000
	add	hl,sp
	ex	de,hl
	ld	hl,#0x0004
	add	hl,de
	ld	(hl),#0x34
	ld	hl,#0x0005
	add	hl,de
	ld	(hl),#0x35
	ld	hl,#0x0006
	add	hl,de
	ld	(hl),#0x36
	ld	hl,#0x0007
	add	hl,de
	ld	(hl),#0x42
	ld	hl,#0x0008
	add	hl,de
	ld	(hl),#0x37
	ld	hl,#0x0009
	add	hl,de
	ld	(hl),#0x38
	ld	hl,#0x000A
	add	hl,de
	ld	(hl),#0x39
	ld	hl,#0x000B
	add	hl,de
	ld	(hl),#0x43
	ld	hl,#0x000C
	add	hl,de
	ld	(hl),#0x2A
	ld	hl,#0x000D
	add	hl,de
	ld	(hl),#0x30
	ld	hl,#0x000E
	add	hl,de
	ld	(hl),#0x23
	ld	hl,#0x000F
	add	hl,de
	ld	(hl),#0x44
;main.c:56: corrimiento=0X10;
	ld	hl,#_corrimiento + 0
	ld	(hl), #0x10
;main.c:58: system_init();
	call	_system_init
;main.c:60: while(TRUE)
00105$:
;main.c:62: PPI_PORTC = corrimiento;
	ld	a,(#_corrimiento + 0)
	out	(_PPI_PORTC),a
;main.c:63: if(PPI_PORTC==0x0)
	in	a,(_PPI_PORTC)
	or	a, a
	jr	NZ,00102$
;main.c:65: corrimiento=0x10;
	ld	hl,#_corrimiento + 0
	ld	(hl), #0x10
	jr	00105$
00102$:
;main.c:69: corrimiento<<=1;
	ld	iy,#_corrimiento
	sla	0 (iy)
	jr	00105$
	ld	iy,#16
	add	iy,sp
	ld	sp,iy
	ret
_main_end::
;main.c:78: void system_init(){
;	---------------------------------
; Function system_init
; ---------------------------------
_system_init_start::
_system_init:
;main.c:80: PPI_CTRL = 0x81; //Salida
	ld	a,#0x81
	out	(_PPI_CTRL),a
;main.c:81: PPI_PORTA = 0;
	ld	a,#0x00
	out	(_PPI_PORTA),a
;main.c:82: PPI_PORTB = 0;
	ld	a,#0x00
	out	(_PPI_PORTB),a
;main.c:83: PPI_PORTC = 0;
	ld	a,#0x00
	out	(_PPI_PORTC),a
;main.c:85: UartInit();
	call	_UartInit
;main.c:86: IM(1);
	IM 1 
;main.c:87: EI();
	EI
	ret
_system_init_end::
;main.c:91: void delay_ms(int milis){
;	---------------------------------
; Function delay_ms
; ---------------------------------
_delay_ms_start::
_delay_ms:
;main.c:93: while(milis--){
	pop	bc
	pop	de
	push	de
	push	bc
00102$:
	ld	b,e
	ld	c,d
	dec	de
	ld	a,c
	or	a,b
	ret	Z
;main.c:94: for(i = 0; i<0x10A; i++){
	ld	bc,#0x010A
00107$:
;main.c:95: NOP();
	NOP
	dec	bc
;main.c:94: for(i = 0; i<0x10A; i++){
	ld	a,b
	or	a,c
	jr	NZ,00107$
	jr	00102$
	ret
_delay_ms_end::
;main.c:100: void teclado_read()
;	---------------------------------
; Function teclado_read
; ---------------------------------
_teclado_read_start::
_teclado_read:
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
;main.c:104: RES  =  PPI_PORTC & 0x0F;
	in	a,(_PPI_PORTC)
	and	a, #0x0F
	ld	e,a
	ld	d,#0x00
;main.c:105: RES = RES | corrimiento;
	ld	hl,#_corrimiento + 0
	ld	c, (hl)
	ld	b,#0x00
	ld	a,e
	or	a, c
	ld	h,a
	ld	a,d
	or	a, b
	ld	-2 (ix), h
	ld	-1 (ix), a
;main.c:106: switch(RES)
	ld	a,-2 (ix)
	sub	a, #0x11
	jr	NZ,00184$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00110$
00184$:
	ld	a,-2 (ix)
	sub	a, #0x12
	jr	NZ,00185$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00111$
00185$:
	ld	a,-2 (ix)
	sub	a, #0x14
	jr	NZ,00186$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00112$
00186$:
	ld	a,-2 (ix)
	sub	a, #0x18
	jr	NZ,00187$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00113$
00187$:
	ld	a,-2 (ix)
	sub	a, #0x21
	jr	NZ,00188$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00103$
00188$:
	ld	a,-2 (ix)
	sub	a, #0x22
	jr	NZ,00189$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00106$
00189$:
	ld	a,-2 (ix)
	sub	a, #0x24
	jr	NZ,00190$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00109$
00190$:
	ld	a,-2 (ix)
	sub	a, #0x28
	jr	NZ,00191$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00115$
00191$:
	ld	a,-2 (ix)
	sub	a, #0x41
	jr	NZ,00192$
	ld	a,-1 (ix)
	or	a, a
	jr	Z,00102$
00192$:
	ld	a,-2 (ix)
	sub	a, #0x42
	jr	NZ,00193$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00105$
00193$:
	ld	a,-2 (ix)
	sub	a, #0x44
	jr	NZ,00194$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00108$
00194$:
	ld	a,-2 (ix)
	sub	a, #0x48
	jr	NZ,00195$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00116$
00195$:
	ld	a,-2 (ix)
	sub	a, #0x81
	jr	NZ,00196$
	ld	a,-1 (ix)
	or	a, a
	jr	Z,00101$
00196$:
	ld	a,-2 (ix)
	sub	a, #0x82
	jr	NZ,00197$
	ld	a,-1 (ix)
	or	a, a
	jr	Z,00104$
00197$:
	ld	a,-2 (ix)
	sub	a, #0x84
	jr	NZ,00198$
	ld	a,-1 (ix)
	or	a, a
	jr	Z,00107$
00198$:
	ld	a,-2 (ix)
	sub	a, #0x88
	jp	NZ,00118$
	ld	a,-1 (ix)
	or	a, a
	jp	Z,00114$
	jp	00118$
;main.c:108: case 0x81:
00101$:
;main.c:110: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0031
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:111: break;
	jp	00118$
;main.c:113: case 0x41:		
00102$:
;main.c:115: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0032
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:116: break;
	jp	00118$
;main.c:118: case 0x21:
00103$:
;main.c:120: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0033
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:121: break;
	jp	00118$
;main.c:123: case 0x82:
00104$:
;main.c:125: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0034
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:126: break;
	jp	00118$
;main.c:128: case 0x42:
00105$:
;main.c:130: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0035
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:131: break;
	jp	00118$
;main.c:133: case 0x22:
00106$:
;main.c:135: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0036
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:136: break;
	jp	00118$
;main.c:138: case 0x84:
00107$:
;main.c:140: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0037
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:141: break;
	jp	00118$
;main.c:143: case 0x44:
00108$:
;main.c:145: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0038
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:146: break;
	jp	00118$
;main.c:148: case 0x24:
00109$:
;main.c:150: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0039
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:151: break;
	jr	00118$
;main.c:153: case 0x11:
00110$:
;main.c:155: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0041
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:156: break;
	jr	00118$
;main.c:158: case 0x12:
00111$:
;main.c:160: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0042
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:161: break;
	jr	00118$
;main.c:163: case 0x14:
00112$:
;main.c:165: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0043
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:166: break;
	jr	00118$
;main.c:168: case 0x18:
00113$:
;main.c:170: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0044
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:171: break;
	jr	00118$
;main.c:173: case 0x88:
00114$:
;main.c:175: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0045
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:176: break;
	jr	00118$
;main.c:178: case 0x28:
00115$:
;main.c:180: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0046
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:181: break;
	jr	00118$
;main.c:183: case 0x48:
00116$:
;main.c:185: printf(" %c", letra);
	ld	hl,#___str_0
	ld	bc,#0x0030
	push	bc
	push	hl
	call	_printf
	pop	af
	pop	af
;main.c:188: }
00118$:
	ld	sp, ix
	pop	ix
	ret
_teclado_read_end::
___str_0:
	.ascii " %c"
	.db 0x00
	.area _CODE
	.area _INITIALIZER
__xinit__corrimiento:
	.db #0x00	; 0
	.area _CABS (ABS)

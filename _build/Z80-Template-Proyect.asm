;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jul 12 2014) (Linux)
; This file was generated Tue Dec  1 23:04:36 2015
;--------------------------------------------------------
	.module main
	.optsdcc -mz80
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _isr_vector38
	.globl _isr_vector66
	.globl _isprint
	.globl ___ret_aux
	.globl _io_write
	.globl _io_read
	.globl _io_write_buffer
	.globl _io_read_buffer
	.globl _uart_init
	.globl _uart_set_baudrate
	.globl _uart_write
	.globl _uart_read
	.globl _uart_print
	.globl _uart_read_line
	.globl _uart_disable_interrupts
	.globl _uart_enable_interrupts
	.globl _ppi_init
	.globl _ppi_set_portc_bit
	.globl _ppi_clear_portc_bit
	.globl _delay_10us
	.globl _delay_100us
	.globl _delay_ms
	.globl _putchar
	.globl _getchar
	.globl _system_init
	.globl _lcd_init
	.globl _writeRegister24
	.globl _writeRegister32
	.globl _writeRegister16
	.globl _write8
	.globl _setAddrWindow
	.globl _reset
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
_URRBR	=	0x0010
_URTHR	=	0x0010
_URIER	=	0x0011
_URIIR	=	0x0012
_URLCR	=	0x0013
_URLSR	=	0x0015
_URMCR	=	0x0014
_URMSR	=	0x0016
_URDLL	=	0x0010
_URDLM	=	0x0011
_PPI_PORTA	=	0x0040
_PPI_PORTB	=	0x0041
_PPI_PORTC	=	0x0042
_PPI_CTRL	=	0x0043
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _DATA
___ret_aux::
	.ds 1
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _INITIALIZED
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
;smz80.h:631: void io_write(char port_addr, char data){
;	---------------------------------
; Function io_write
; ---------------------------------
_io_write_start::
_io_write:
;smz80.h:642: __endasm;
	ld ix, #2
	add ix,sp
	ld c, (ix)
	inc ix
	ld a,(ix)
	out (c), a
	ret
_io_write_end::
;smz80.h:649: char io_read(char port_addr){
;	---------------------------------
; Function io_read
; ---------------------------------
_io_read_start::
_io_read:
;smz80.h:661: __endasm;
	LD IX, #2
	ADD IX,SP
	LD C, (IX)
	IN A,(C)
	LD (___ret_aux),A
;smz80.h:663: return __ret_aux;
	ld	iy,#___ret_aux
	ld	l,0 (iy)
	ret
_io_read_end::
;smz80.h:666: void io_write_buffer(char port_addr, char* buffer_out, char count){
;	---------------------------------
; Function io_write_buffer
; ---------------------------------
_io_write_buffer_start::
_io_write_buffer:
;smz80.h:682: __endasm;
	LD IX, #2
	ADD IX,SP
	LD C, (IX)
	INC IX
	LD L,(IX)
	INC IX
	LD H,(IX)
	INC IX
	LD B,(IX)
	OTIR
	ret
_io_write_buffer_end::
;smz80.h:685: void io_read_buffer(char port_addr, char* buffer_in, char count){
;	---------------------------------
; Function io_read_buffer
; ---------------------------------
_io_read_buffer_start::
_io_read_buffer:
;smz80.h:702: __endasm;
	LD IX, #2
	ADD IX,SP
	LD C, (IX)
	INC IX
	LD L,(IX)
	INC IX
	LD H,(IX)
	INC IX
	LD B,(IX)
	INIR
	ret
_io_read_buffer_end::
;smz80.h:705: void uart_init(const uart_cfg_t *uart_config){
;	---------------------------------
; Function uart_init
; ---------------------------------
_uart_init_start::
_uart_init:
	push	ix
	ld	ix,#0
	add	ix,sp
;smz80.h:708: uart_set_baudrate(uart_config->baudrate);
	ld	e,4 (ix)
	ld	d,5 (ix)
	ld	a,(de)
	push	de
	push	af
	inc	sp
	call	_uart_set_baudrate
	inc	sp
	pop	de
;smz80.h:710: URIER = uart_config->interrupt;
	ld	l, e
	ld	h, d
	ld	bc, #0x0004
	add	hl, bc
	ld	a,(hl)
	out	(_URIER),a
;smz80.h:712: URLCR = (uart_config->stop_bits) | (uart_config->parity) | (uart_config->word_length);
	ld	l, e
	ld	h, d
	inc	hl
	ld	b,(hl)
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	ld	a,(hl)
	or	a, b
	ld	h,d
	ld	l, e
	inc	hl
	inc	hl
	inc	hl
	ld	d,(hl)
	or	a, d
	out	(_URLCR),a
	pop	ix
	ret
_uart_init_end::
;smz80.h:715: void uart_set_baudrate(const uart_baudrate_t baudrate){
;	---------------------------------
; Function uart_set_baudrate
; ---------------------------------
_uart_set_baudrate_start::
_uart_set_baudrate:
;smz80.h:718: URLCR |= BV(UDLAB);
	in	a,(_URLCR)
	set	7, a
	out	(_URLCR),a
;smz80.h:720: URDLL = baudrate;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URDLL),a
;smz80.h:722: URDLM = ((uint16_t)baudrate)>>8;
	ld	a, #0x00
	out	(_URDLM),a
;smz80.h:724: URLCR &= ~BV(UDLAB);
	in	a,(_URLCR)
	and	a, #0x7F
	out	(_URLCR),a
	ret
_uart_set_baudrate_end::
;smz80.h:727: void uart_write(char c){
;	---------------------------------
; Function uart_write
; ---------------------------------
_uart_write_start::
_uart_write:
;smz80.h:730: while( !(URLSR & BV(UTHRE)))
00101$:
	in	a,(_URLSR)
	and	a, #0x20
	jr	NZ,00103$
;smz80.h:731: NOP();    
	NOP
	jr	00101$
00103$:
;smz80.h:733: URTHR = c;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URTHR),a
	ret
_uart_write_end::
;smz80.h:736: char uart_read(){
;	---------------------------------
; Function uart_read
; ---------------------------------
_uart_read_start::
_uart_read:
;smz80.h:739: while(!(URLSR & BV(UDR))) 
00101$:
	in	a,(_URLSR)
	rrca
	jr	C,00103$
;smz80.h:740: NOP();
	NOP
	jr	00101$
00103$:
;smz80.h:742: return URRBR;
	in	a,(_URRBR)
	ld	l,a
	ret
_uart_read_end::
;smz80.h:745: void uart_print(const char* str){
;	---------------------------------
; Function uart_print
; ---------------------------------
_uart_print_start::
_uart_print:
;smz80.h:748: while(*str)       
	pop	bc
	pop	hl
	push	hl
	push	bc
00101$:
	ld	a,(hl)
	or	a, a
	ret	Z
;smz80.h:749: putchar(*str++); // envía el siguiente caracter. 
	inc	hl
	push	hl
	push	af
	inc	sp
	call	_putchar
	inc	sp
	pop	hl
	jr	00101$
	ret
_uart_print_end::
;smz80.h:752: int uart_read_line(char* str){
;	---------------------------------
; Function uart_read_line
; ---------------------------------
_uart_read_line_start::
_uart_read_line:
	push	ix
	ld	ix,#0
	add	ix,sp
;smz80.h:754: int n=0;
	ld	bc,#0x0000
;smz80.h:756: while(n<MAXLINE-1 && (c=getchar()) != '\n' && c !='\r'){
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
;smz80.h:758: if(c == 0x7F || c==0x08){
	ld	a,d
	cp	a,#0x0D
	jr	Z,00113$
	cp	a,#0x7F
	jr	Z,00105$
	sub	a, #0x08
	jr	NZ,00106$
00105$:
;smz80.h:760: if(n>0){
	xor	a, a
	cp	a, c
	sbc	a, b
	jp	PO, 00149$
	xor	a, #0x80
00149$:
	jp	P,00111$
;smz80.h:761: str[--n]='\0';
	dec	bc
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	(hl),#0x00
;smz80.h:762: putchar(c);
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
;smz80.h:768: if(isprint(c))
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
;smz80.h:770: str[n++]=c;
	ld	h,c
	ld	e,b
	inc	bc
	ld	a,4 (ix)
	add	a, h
	ld	l,a
	ld	a,5 (ix)
	adc	a, e
	ld	h,a
	ld	(hl),d
;smz80.h:771: putchar(c);
	push	bc
	push	de
	inc	sp
	call	_putchar
	inc	sp
	pop	bc
	jp	00111$
00113$:
;smz80.h:775: str[n]='\0';     
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	(hl),#0x00
;smz80.h:776: putchar('\n');
	push	bc
	ld	a,#0x0A
	push	af
	inc	sp
	call	_putchar
	inc	sp
;smz80.h:777: return n;
	pop	hl
	pop	ix
	ret
_uart_read_line_end::
;smz80.h:780: void uart_disable_interrupts(){
;	---------------------------------
; Function uart_disable_interrupts
; ---------------------------------
_uart_disable_interrupts_start::
_uart_disable_interrupts:
;smz80.h:782: URIER = 0;
	ld	a,#0x00
	out	(_URIER),a
	ret
_uart_disable_interrupts_end::
;smz80.h:785: void uart_enable_interrupts(uart_interrupt_t int_cfg){
;	---------------------------------
; Function uart_enable_interrupts
; ---------------------------------
_uart_enable_interrupts_start::
_uart_enable_interrupts:
;smz80.h:787: URIER = int_cfg;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URIER),a
	ret
_uart_enable_interrupts_end::
;smz80.h:792: void ppi_init(const ppi_cfg_t *ppi_config){
;	---------------------------------
; Function ppi_init
; ---------------------------------
_ppi_init_start::
_ppi_init:
	push	ix
	ld	ix,#0
	add	ix,sp
;smz80.h:794: PPI_CTRL = 0x80 | ppi_config->mode | (ppi_config->pcl_dir << PCPCL) | (ppi_config->pch_dir << PCPCH) | (ppi_config->pa_dir << PCPA) | (ppi_config->pb_dir << PCPB);
	ld	c,4 (ix)
	ld	b,5 (ix)
	ld	a,(bc)
	set	7, a
	ld	e,a
	push	bc
	pop	iy
	ld	a,3 (iy)
	or	a, e
	ld	e,a
	push	bc
	pop	iy
	ld	a,4 (iy)
	rlca
	rlca
	rlca
	and	a,#0xF8
	or	a, e
	ld	e,a
	ld	l, c
	ld	h, b
	inc	hl
	ld	a,(hl)
	rlca
	rlca
	rlca
	rlca
	and	a,#0xF0
	or	a, e
	ld	d,a
	ld	l, c
	ld	h, b
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a, a
	or	a, d
	out	(_PPI_CTRL),a
	pop	ix
	ret
_ppi_init_end::
;smz80.h:797: void ppi_set_portc_bit(const char bit){
;	---------------------------------
; Function ppi_set_portc_bit
; ---------------------------------
_ppi_set_portc_bit_start::
_ppi_set_portc_bit:
;smz80.h:799: PPI_CTRL = 1 | bit << 1;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	add	a, a
	set	0, a
	out	(_PPI_CTRL),a
	ret
_ppi_set_portc_bit_end::
;smz80.h:802: void ppi_clear_portc_bit(const char bit){
;	---------------------------------
; Function ppi_clear_portc_bit
; ---------------------------------
_ppi_clear_portc_bit_start::
_ppi_clear_portc_bit:
;smz80.h:804: PPI_CTRL = bit << 1;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	add	a, a
	out	(_PPI_CTRL),a
	ret
_ppi_clear_portc_bit_end::
;smz80.h:807: void delay_10us(){
;	---------------------------------
; Function delay_10us
; ---------------------------------
_delay_10us_start::
_delay_10us:
;smz80.h:817: __endasm;
	EXX
	EX AF,AF'
	LD B,#0x4
	    LOOP_10:
	DJNZ LOOP_10
	EX AF,AF'
	EXX
	ret
_delay_10us_end::
;smz80.h:821: void delay_100us(){
;	---------------------------------
; Function delay_100us
; ---------------------------------
_delay_100us_start::
_delay_100us:
;smz80.h:831: __endasm;
	EXX
	EX AF,AF'
	LD B,#0x3A
	    LOOP_100:
	DJNZ LOOP_100
	EX AF,AF'
	EXX
	ret
_delay_100us_end::
;smz80.h:835: void delay_ms(int ms){
;	---------------------------------
; Function delay_ms
; ---------------------------------
_delay_ms_start::
_delay_ms:
;smz80.h:839: while(ms--)
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
;smz80.h:840: for(i=0;i<0x10A;i++)
	ld	bc,#0x010A
00107$:
;smz80.h:841: __asm__("nop");
	nop
	dec	bc
;smz80.h:840: for(i=0;i<0x10A;i++)
	ld	a,b
	or	a,c
	jr	NZ,00107$
	jr	00102$
	ret
_delay_ms_end::
;smz80.h:848: void putchar(char c){
;	---------------------------------
; Function putchar
; ---------------------------------
_putchar_start::
_putchar:
;smz80.h:851: if(c=='\n')
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	sub	a, #0x0A
	jr	NZ,00102$
;smz80.h:852: uart_write('\r');
	ld	a,#0x0D
	push	af
	inc	sp
	call	_uart_write
	inc	sp
00102$:
;smz80.h:853: uart_write(c);
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	push	af
	inc	sp
	call	_uart_write
	inc	sp
	ret
_putchar_end::
;smz80.h:858: char getchar(){
;	---------------------------------
; Function getchar
; ---------------------------------
_getchar_start::
_getchar:
;smz80.h:861: return uart_read();
	jp	_uart_read
_getchar_end::
;main.c:112: ISR_NMI(){
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
;main.c:114: }
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	retn
_isr_vector66_end::
_PROGMEM:
	.dw #0x0000
	.dw #0x0001
	.dw #0x00FF
	.dw #0x0032
	.dw #0x0001
	.dw #0x0100
	.dw #0x0002
	.dw #0x0700
	.dw #0x0003
	.dw #0x1030
	.dw #0x0004
	.dw #0x0000
	.dw #0x0008
	.dw #0x0202
	.dw #0x0009
	.dw #0x0000
	.dw #0x000A
	.dw #0x0000
	.dw #0x000C
	.dw #0x0000
	.dw #0x000D
	.dw #0x0000
	.dw #0x000F
	.dw #0x0000
	.dw #0x0010
	.dw #0x0000
	.dw #0x0011
	.dw #0x0007
	.dw #0x0012
	.dw #0x0000
	.dw #0x0013
	.dw #0x0000
	.dw #0x00FF
	.dw #0x00C8
	.dw #0x0010
	.dw #0x1690
	.dw #0x0011
	.dw #0x0227
	.dw #0x00FF
	.dw #0x0032
	.dw #0x0012
	.dw #0x001A
	.dw #0x00FF
	.dw #0x0032
	.dw #0x0013
	.dw #0x1800
	.dw #0x0029
	.dw #0x002A
	.dw #0x00FF
	.dw #0x0032
	.dw #0x0030
	.dw #0x0000
	.dw #0x0031
	.dw #0x0000
	.dw #0x0032
	.dw #0x0000
	.dw #0x0035
	.dw #0x0206
	.dw #0x0036
	.dw #0x0808
	.dw #0x0037
	.dw #0x0007
	.dw #0x0038
	.dw #0x0201
	.dw #0x0039
	.dw #0x0000
	.dw #0x003C
	.dw #0x0000
	.dw #0x003D
	.dw #0x0000
	.dw #0x0020
	.dw #0x0000
	.dw #0x0021
	.dw #0x0000
	.dw #0x0050
	.dw #0x0000
	.dw #0x0051
	.dw #0x00EF
	.dw #0x0052
	.dw #0x0000
	.dw #0x0053
	.dw #0x013F
	.dw #0x0060
	.dw #0xA700
	.dw #0x0061
	.dw #0x0003
	.dw #0x006A
	.dw #0x0000
	.dw #0x0090
	.dw #0x0010
	.dw #0x0092
	.dw #0x0000
	.dw #0x0093
	.dw #0x0003
	.dw #0x0095
	.dw #0x1100
	.dw #0x0097
	.dw #0x0000
	.dw #0x0098
	.dw #0x0000
	.dw #0x0007
	.dw #0x0133
;main.c:116: ISR_INT_38(){
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
;main.c:118: }
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	reti
_isr_vector38_end::
;main.c:121: int main(){
;	---------------------------------
; Function main
; ---------------------------------
_main_start::
_main:
;main.c:124: system_init(); 
	call	_system_init
;main.c:127: while(TRUE){
00102$:
;main.c:128: sleep();    //Entra en modo sleep (HALT)
	HALT
	jr	00102$
_main_end::
;main.c:133: void system_init()
;	---------------------------------
; Function system_init
; ---------------------------------
_system_init_start::
_system_init:
;main.c:135: PPI_CTRL = 0x89; //Palabra de control PA y PB salida PCH entrada PCD salida
	ld	a,#0x89
	out	(_PPI_CTRL),a
;main.c:136: lcd_init();   
	jp	_lcd_init
_system_init_end::
;main.c:139: void lcd_init()
;	---------------------------------
; Function lcd_init
; ---------------------------------
_lcd_init_start::
_lcd_init:
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
;main.c:141: uint8_t i = 0;
	ld	b,#0x00
;main.c:145: delay_ms(200);
	push	bc
	ld	hl,#0x00C8
	push	hl
	call	_delay_ms
	pop	af
	pop	bc
;main.c:147: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xFC
	out	(_PPI_PORTA),a
;main.c:148: while(i < sizeof(PROGMEM) / sizeof(uint16_t)) 
00104$:
	ld	a,b
	sub	a, #0x66
	jr	NC,00106$
;main.c:150: a = PROGMEM[i++];
	ld	l,b
	inc	b
	ld	h,#0x00
	add	hl, hl
	ld	de,#_PROGMEM
	add	hl,de
	ld	a,(hl)
	ld	-2 (ix),a
	inc	hl
	ld	a,(hl)
	ld	-1 (ix),a
;main.c:151: d = PROGMEM[i++];
	ld	l,b
	inc	b
	ld	h,#0x00
	add	hl, hl
	ld	de,#_PROGMEM
	add	hl,de
	ld	e,(hl)
	inc	hl
	ld	d,(hl)
;main.c:153: if(a == TFTLCD_DELAY)
	ld	a,-2 (ix)
	inc	a
	jr	NZ,00102$
	ld	a,-1 (ix)
	or	a, a
	jr	NZ,00102$
;main.c:155: delay_ms(d);
	push	bc
	push	de
	call	_delay_ms
	pop	af
	pop	bc
	jr	00104$
00102$:
;main.c:159: writeRegister16(a, d);
	ld	h,e
	ld	d,-2 (ix)
	push	bc
	push	hl
	inc	sp
	push	de
	inc	sp
	call	_writeRegister16
	pop	af
	pop	bc
	jr	00104$
00106$:
;main.c:162: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xFC
	out	(_PPI_PORTA),a
;main.c:164: writeRegister16(0x0003, a);
	ld	hl,#0x3003
	push	hl
	call	_writeRegister16
;main.c:165: setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);
	ld	hl, #0x013F
	ex	(sp),hl
	ld	hl,#0x00EF
	push	hl
	ld	l, #0x00
	push	hl
	ld	l, #0x00
	push	hl
	call	_setAddrWindow
	ld	hl,#8
	add	hl,sp
	ld	sp,hl
	ld	sp, ix
	pop	ix
	ret
_lcd_init_end::
;main.c:169: void writeRegister24(uint8_t r, uint32_t d) 
;	---------------------------------
; Function writeRegister24
; ---------------------------------
_writeRegister24_start::
_writeRegister24:
;main.c:171: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xFC
	out	(_PPI_PORTA),a
;main.c:172: CD_COMMAND;
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
;main.c:173: write8(r);
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	push	af
	inc	sp
	call	_write8
	inc	sp
;main.c:174: CD_DATA;
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
;main.c:175: delay_10us();
	call	_delay_10us
;main.c:176: write8(d >> 16);
	push	af
	ld	iy,#5
	add	iy,sp
	ld	h,0 (iy)
	ld	l,1 (iy)
	ld	d,2 (iy)
	ld	e,3 (iy)
	pop	af
	ld	b,#0x10
00103$:
	srl	e
	rr	d
	rr	l
	rr	h
	djnz	00103$
	push	hl
	inc	sp
	call	_write8
	inc	sp
;main.c:177: delay_10us();
	call	_delay_10us
;main.c:178: write8(d >> 8);
	push	af
	ld	iy,#5
	add	iy,sp
	ld	h,0 (iy)
	ld	l,1 (iy)
	ld	d,2 (iy)
	ld	e,3 (iy)
	pop	af
	ld	b,#0x08
00105$:
	srl	e
	rr	d
	rr	l
	rr	h
	djnz	00105$
	push	hl
	inc	sp
	call	_write8
	inc	sp
;main.c:179: delay_10us();
	call	_delay_10us
;main.c:180: write8(d);
	ld	iy,#3
	add	iy,sp
	ld	h,0 (iy)
	push	hl
	inc	sp
	call	_write8
	inc	sp
;main.c:181: CS_IDLE;
	in	a,(_PPI_PORTA)
	or	a, #0x03
	out	(_PPI_PORTA),a
	ret
_writeRegister24_end::
;main.c:185: void writeRegister32(uint8_t r, uint32_t d) 
;	---------------------------------
; Function writeRegister32
; ---------------------------------
_writeRegister32_start::
_writeRegister32:
;main.c:187: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xFC
	out	(_PPI_PORTA),a
;main.c:188: CD_COMMAND;
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
;main.c:189: write8(r);
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	push	af
	inc	sp
	call	_write8
	inc	sp
;main.c:190: CD_DATA;
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
;main.c:191: delay_10us();
	call	_delay_10us
;main.c:192: write8(d >> 24);
	push	af
	ld	iy,#5
	add	iy,sp
	ld	h,0 (iy)
	ld	l,1 (iy)
	ld	d,2 (iy)
	ld	e,3 (iy)
	pop	af
	ld	b,#0x18
00103$:
	srl	e
	rr	d
	rr	l
	rr	h
	djnz	00103$
	push	hl
	inc	sp
	call	_write8
	inc	sp
;main.c:193: delay_10us();
	call	_delay_10us
;main.c:194: write8(d >> 16);
	push	af
	ld	iy,#5
	add	iy,sp
	ld	h,0 (iy)
	ld	l,1 (iy)
	ld	d,2 (iy)
	ld	e,3 (iy)
	pop	af
	ld	b,#0x10
00105$:
	srl	e
	rr	d
	rr	l
	rr	h
	djnz	00105$
	push	hl
	inc	sp
	call	_write8
	inc	sp
;main.c:195: delay_10us();
	call	_delay_10us
;main.c:196: write8(d >> 8);
	push	af
	ld	iy,#5
	add	iy,sp
	ld	h,0 (iy)
	ld	l,1 (iy)
	ld	d,2 (iy)
	ld	e,3 (iy)
	pop	af
	ld	b,#0x08
00107$:
	srl	e
	rr	d
	rr	l
	rr	h
	djnz	00107$
	push	hl
	inc	sp
	call	_write8
	inc	sp
;main.c:197: delay_10us();
	call	_delay_10us
;main.c:198: write8(d);
	ld	iy,#3
	add	iy,sp
	ld	h,0 (iy)
	push	hl
	inc	sp
	call	_write8
	inc	sp
;main.c:199: CS_IDLE;
	in	a,(_PPI_PORTA)
	or	a, #0x03
	out	(_PPI_PORTA),a
	ret
_writeRegister32_end::
;main.c:203: void writeRegister16(uint8_t a, uint8_t d) 
;	---------------------------------
; Function writeRegister16
; ---------------------------------
_writeRegister16_start::
_writeRegister16:
;main.c:208: lo = (a); 
	ld	hl, #2+0
	add	hl, sp
	ld	d, (hl)
;main.c:209: CD_COMMAND; 
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
;main.c:210: write8(hi); 
	push	de
	xor	a, a
	push	af
	inc	sp
	call	_write8
	inc	sp
	inc	sp
	call	_write8
	inc	sp
;main.c:213: lo = (d); 
	ld	hl, #3+0
	add	hl, sp
	ld	d, (hl)
;main.c:214: CD_DATA; 
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
;main.c:215: write8(hi); 
	push	de
	xor	a, a
	push	af
	inc	sp
	call	_write8
	inc	sp
	inc	sp
	call	_write8
	inc	sp
	ret
_writeRegister16_end::
;main.c:219: void write8(uint8_t d) 
;	---------------------------------
; Function write8
; ---------------------------------
_write8_start::
_write8:
;main.c:221: PPI_PORTA = d;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_PPI_PORTA),a
;main.c:222: WR_STROBE; 
	in	a,(_PPI_PORTA)
	and	a, #0xFE
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	0, a
	out	(_PPI_PORTA),a
	ret
_write8_end::
;main.c:225: void setAddrWindow(int x1, int y1, int x2, int y2) 
;	---------------------------------
; Function setAddrWindow
; ---------------------------------
_setAddrWindow_start::
_setAddrWindow:
;main.c:228: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xFC
	out	(_PPI_PORTA),a
;main.c:229: x  = x1;
	pop	de
	pop	bc
	push	bc
	push	de
;main.c:230: y  = y1;
	ld	iy,#4
	add	iy,sp
	ld	d,0 (iy)
	ld	l,1 (iy)
;main.c:232: writeRegister16(0x0050, x1); 
	ld	e,c
	push	hl
	push	de
	ld	d, e
	ld	e,#0x50
	push	de
	call	_writeRegister16
	pop	af
	pop	de
	pop	hl
;main.c:233: writeRegister16(0x0051, x2);
	ld	iy,#6
	add	iy,sp
	ld	h,0 (iy)
	push	hl
	push	de
	push	hl
	inc	sp
	ld	a,#0x51
	push	af
	inc	sp
	call	_writeRegister16
	pop	af
	pop	de
	pop	hl
;main.c:234: writeRegister16(0x0052, y1);
	push	de
	ld	e, #0x52
	push	de
	call	_writeRegister16
	pop	af
	pop	de
;main.c:235: writeRegister16(0x0053, y2);
	ld	iy,#8
	add	iy,sp
	ld	h,0 (iy)
	push	de
	push	hl
	inc	sp
	ld	a,#0x53
	push	af
	inc	sp
	call	_writeRegister16
	pop	af
	pop	de
;main.c:236: writeRegister16(0x0020, x );
	push	de
	ld	d, e
	ld	e,#0x20
	push	de
	call	_writeRegister16
	pop	af
	pop	de
;main.c:237: writeRegister16(0x0021, y );
	ld	e, #0x21
	push	de
	call	_writeRegister16
	pop	af
	ret
_setAddrWindow_end::
;main.c:241: void reset() 
;	---------------------------------
; Function reset
; ---------------------------------
_reset_start::
_reset:
;main.c:245: CS_IDLE;
	in	a,(_PPI_PORTA)
	or	a, #0x03
	out	(_PPI_PORTA),a
;main.c:246: WR_IDLE;
	in	a,(_PPI_PORTA)
	set	0, a
	out	(_PPI_PORTA),a
;main.c:247: RD_IDLE;
;main.c:249: RESET_LOW;
	in	a,(_PPI_PORTA)
	and	a, #0xFB
	out	(_PPI_PORTA),a
;main.c:250: delay_ms(2);
	ld	hl,#0x0002
	push	hl
	call	_delay_ms
	pop	af
;main.c:251: RESET_HIGH;
	in	a,(_PPI_PORTA)
	set	2, a
	out	(_PPI_PORTA),a
;main.c:253: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xFC
	out	(_PPI_PORTA),a
;main.c:254: CD_COMMAND;
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
;main.c:255: write8(0x00);
	xor	a, a
	push	af
	inc	sp
	call	_write8
	inc	sp
;main.c:257: for(i=0; i<3; i++)
	ld	d,#0x00
00102$:
;main.c:259: WR_STROBE;
	in	a,(_PPI_PORTA)
	and	a, #0xFE
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	0, a
	out	(_PPI_PORTA),a
;main.c:257: for(i=0; i<3; i++)
	inc	d
	ld	a,d
	sub	a, #0x03
	jr	C,00102$
;main.c:261: CS_IDLE;
	in	a,(_PPI_PORTA)
	or	a, #0x03
	out	(_PPI_PORTA),a
;main.c:263: delay_ms(500);
	ld	hl,#0x01F4
	push	hl
	call	_delay_ms
	pop	af
	ret
_reset_end::
	.area _CODE
	.area _INITIALIZER
	.area _CABS (ABS)

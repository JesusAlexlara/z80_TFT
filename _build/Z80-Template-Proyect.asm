;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jul 12 2014) (Linux)
; This file was generated Sat Dec  5 14:03:21 2015
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
	.globl _jug
	.globl _rana4
	.globl _rana3
	.globl _rana2
	.globl _rana
	.globl _cuadroMorado
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
	.globl _fillScreen
	.globl _flood
	.globl _drawPixel
	.globl _fillRect
	.globl _drawGraf
	.globl _pintaSprite
	.globl _color565
	.globl _init_pantalla
	.globl _pintaAzul
	.globl _pintaNegro
	.globl _pintaBarra
	.globl _repinta_rana
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
_cuadroMorado::
	.ds 800
_rana::
	.ds 800
_rana2::
	.ds 800
_rana3::
	.ds 800
_rana4::
	.ds 800
_jug::
	.ds 6
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _INITIALIZED
_width:
	.ds 2
_height:
	.ds 2
_header_data:
	.ds 2
_width_Rana:
	.ds 2
_height_Rana:
	.ds 2
_header_Rana:
	.ds 2
_width_Rana2:
	.ds 2
_height_Rana2:
	.ds 2
_header_Rana2:
	.ds 2
_width_Rana3:
	.ds 2
_height_Rana3:
	.ds 2
_header_Rana3:
	.ds 2
_width_Rana4:
	.ds 2
_height_Rana4:
	.ds 2
_header_Rana4:
	.ds 2
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
;main.c:51: ISR_NMI()
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
;main.c:54: }
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
;main.c:56: ISR_INT_38()
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
;main.c:58: DI();
	DI
;main.c:61: EI();
	EI
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	reti
_isr_vector38_end::
;main.c:66: int main()
;	---------------------------------
; Function main
; ---------------------------------
_main_start::
_main:
;main.c:70: jug.x = 20;
	ld	hl,#0x0014
	ld	((_jug + 0x0002)), hl
;main.c:71: jug.y = 300;
	ld	hl,#0x012C
	ld	((_jug + 0x0004)), hl
;main.c:72: jug.altura = 15;
	ld	hl,#0x000F
	ld	(_jug), hl
;main.c:73: system_init(); 
	call	_system_init
;main.c:74: PPI_PORTA = 0xff;
	ld	a,#0xFF
	out	(_PPI_PORTA),a
;main.c:76: fillScreen(BLACK);
	ld	hl,#0x0000
	push	hl
	call	_fillScreen
	pop	af
;main.c:77: init_pantalla();
	call	_init_pantalla
;main.c:78: EI();
	EI
;main.c:80: while(TRUE)
00102$:
;main.c:82: SLEEP();
	HALT
	jr	00102$
_main_end::
;main.c:86: void system_init()
;	---------------------------------
; Function system_init
; ---------------------------------
_system_init_start::
_system_init:
;main.c:88: PPI_CTRL = 0x89;
	ld	a,#0x89
	out	(_PPI_CTRL),a
;main.c:89: PPI_PORTA = 0xff;
	ld	a,#0xFF
	out	(_PPI_PORTA),a
;main.c:90: PPI_PORTC = 0x00;
	ld	a,#0x00
	out	(_PPI_PORTC),a
;main.c:91: lcd_init();  
	call	_lcd_init
;main.c:92: IM(1);
	IM 1 
	ret
_system_init_end::
;main.c:95: void lcd_init()
;	---------------------------------
; Function lcd_init
; ---------------------------------
_lcd_init_start::
_lcd_init:
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
;main.c:97: uint8_t i = 0;
	ld	c,#0x00
;main.c:100: CS_IDLE;
	in	a,(_PPI_PORTA)
	set	3, a
	out	(_PPI_PORTA),a
;main.c:101: WR_IDLE;
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
;main.c:102: RD_IDLE;
	in	a,(_PPI_PORTA)
	set	0, a
	out	(_PPI_PORTA),a
;main.c:103: CD_DATA;
	in	a,(_PPI_PORTA)
	set	2, a
	out	(_PPI_PORTA),a
;main.c:105: reset();
	push	bc
	call	_reset
	pop	bc
;main.c:107: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xF7
	out	(_PPI_PORTA),a
;main.c:108: while(i < sizeof(PROGMEM) / sizeof(uint16_t)) 
00104$:
	ld	a,c
	sub	a, #0x66
	jr	NC,00106$
;main.c:110: a = PROGMEM[i++];
	ld	l,c
	inc	c
	ld	h,#0x00
	add	hl, hl
	ld	de,#_PROGMEM
	add	hl,de
	ld	a,(hl)
	ld	-2 (ix),a
	inc	hl
	ld	a,(hl)
	ld	-1 (ix),a
;main.c:111: d = PROGMEM[i++];
	ld	l,c
	inc	c
	ld	h,#0x00
	add	hl, hl
	ld	de,#_PROGMEM
	add	hl,de
	ld	e,(hl)
	inc	hl
	ld	d,(hl)
;main.c:113: if(a == TFTLCD_DELAY)
	ld	a,-2 (ix)
	inc	a
	jr	NZ,00102$
	ld	a,-1 (ix)
	or	a, a
	jr	NZ,00102$
;main.c:115: delay_ms(d);
	push	bc
	push	de
	push	de
	call	_delay_ms
	pop	af
	call	_delay_ms
	pop	af
	pop	bc
	jr	00104$
00102$:
;main.c:120: writeRegister16(a, d);
	push	bc
	push	de
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	call	_writeRegister16
	pop	af
	pop	af
	pop	bc
	jr	00104$
00106$:
;main.c:123: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xF7
	out	(_PPI_PORTA),a
;main.c:125: writeRegister16(0x0003, a);
	ld	hl,#0x1030
	push	hl
	ld	hl,#0x0003
	push	hl
	call	_writeRegister16
	pop	af
;main.c:126: setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);
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
;main.c:130: void writeRegister24(uint8_t r, uint32_t d) 
;	---------------------------------
; Function writeRegister24
; ---------------------------------
_writeRegister24_start::
_writeRegister24:
;main.c:132: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xF7
	out	(_PPI_PORTA),a
;main.c:133: CD_COMMAND;
	in	a,(_PPI_PORTA)
	and	a, #0xFB
	out	(_PPI_PORTA),a
;main.c:134: write8(r);
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	push	af
	inc	sp
	call	_write8
	inc	sp
;main.c:135: CD_DATA;
	in	a,(_PPI_PORTA)
	set	2, a
	out	(_PPI_PORTA),a
;main.c:136: write8(d >> 16);
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
;main.c:137: write8(d >> 8);
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
;main.c:138: write8(d);
	ld	iy,#3
	add	iy,sp
	ld	h,0 (iy)
	push	hl
	inc	sp
	call	_write8
	inc	sp
;main.c:139: CS_IDLE;
	in	a,(_PPI_PORTA)
	set	3, a
	out	(_PPI_PORTA),a
	ret
_writeRegister24_end::
;main.c:143: void writeRegister32(uint8_t r, uint32_t d) 
;	---------------------------------
; Function writeRegister32
; ---------------------------------
_writeRegister32_start::
_writeRegister32:
;main.c:145: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xF7
	out	(_PPI_PORTA),a
;main.c:146: CD_COMMAND;
	in	a,(_PPI_PORTA)
	and	a, #0xFB
	out	(_PPI_PORTA),a
;main.c:147: write8(r);
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	push	af
	inc	sp
	call	_write8
	inc	sp
;main.c:148: CD_DATA;
	in	a,(_PPI_PORTA)
	set	2, a
	out	(_PPI_PORTA),a
;main.c:149: write8(d >> 24);
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
;main.c:150: write8(d >> 16);
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
;main.c:151: write8(d >> 8);
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
;main.c:152: write8(d);
	ld	iy,#3
	add	iy,sp
	ld	h,0 (iy)
	push	hl
	inc	sp
	call	_write8
	inc	sp
;main.c:153: CS_IDLE;
	in	a,(_PPI_PORTA)
	set	3, a
	out	(_PPI_PORTA),a
	ret
_writeRegister32_end::
;main.c:157: void writeRegister16(uint16_t a, uint16_t d) 
;	---------------------------------
; Function writeRegister16
; ---------------------------------
_writeRegister16_start::
_writeRegister16:
;main.c:161: hi = (a) >> 8; 
	ld	iy,#2
	add	iy,sp
	ld	d,1 (iy)
;main.c:162: lo = (a); 
	ld	b,0 (iy)
;main.c:163: CD_COMMAND; 
	in	a,(_PPI_PORTA)
	and	a, #0xFB
	out	(_PPI_PORTA),a
;main.c:164: write8(hi);
	push	bc
	push	de
	inc	sp
	call	_write8
	inc	sp
	inc	sp
	call	_write8
	inc	sp
;main.c:166: hi = (d) >> 8; 
	ld	iy,#4
	add	iy,sp
	ld	d,1 (iy)
;main.c:167: lo = (d); 
	ld	b,0 (iy)
;main.c:168: CD_DATA; 
	in	a,(_PPI_PORTA)
	set	2, a
	out	(_PPI_PORTA),a
;main.c:169: write8(hi);
	push	bc
	push	de
	inc	sp
	call	_write8
	inc	sp
	inc	sp
	call	_write8
	inc	sp
	ret
_writeRegister16_end::
;main.c:175: void write8(uint8_t d) 
;	---------------------------------
; Function write8
; ---------------------------------
_write8_start::
_write8:
;main.c:177: PPI_PORTB = d;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_PPI_PORTB),a
;main.c:178: WR_STROBE; 
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
	ret
_write8_end::
;main.c:181: void setAddrWindow(int x1, int y1, int x2, int y2) 
;	---------------------------------
; Function setAddrWindow
; ---------------------------------
_setAddrWindow_start::
_setAddrWindow:
;main.c:184: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xF7
	out	(_PPI_PORTA),a
;main.c:185: x  = x1;
	pop	bc
	pop	de
	push	de
	push	bc
;main.c:186: y  = y1;
	ld	hl, #4
	add	hl, sp
;main.c:188: writeRegister16(0x0050, x1); 
	ld	c, (hl)
	inc	hl
	ld	b, (hl)
	push	bc
	push	de
	push	de
	ld	bc,#0x0050
	push	bc
	call	_writeRegister16
	pop	af
	pop	af
	ld	hl, #10
	add	hl, sp
	ld	c, (hl)
	inc	hl
	ld	b, (hl)
	push	bc
	ld	bc,#0x0051
	push	bc
	call	_writeRegister16
	pop	af
	pop	af
	pop	de
	pop	hl
;main.c:190: writeRegister16(0x0052, y1);
	push	hl
	push	de
	push	hl
	ld	bc,#0x0052
	push	bc
	call	_writeRegister16
	pop	af
	pop	af
	ld	hl, #12
	add	hl, sp
	ld	c, (hl)
	inc	hl
	ld	b, (hl)
	push	bc
	ld	bc,#0x0053
	push	bc
	call	_writeRegister16
	pop	af
	pop	af
	ld	bc,#0x0020
	push	bc
	call	_writeRegister16
	pop	af
	ld	hl, #0x0021
	ex	(sp),hl
	call	_writeRegister16
	pop	af
	pop	af
	ret
_setAddrWindow_end::
;main.c:196: void reset() 
;	---------------------------------
; Function reset
; ---------------------------------
_reset_start::
_reset:
;main.c:200: CS_IDLE;
	in	a,(_PPI_PORTA)
	set	3, a
	out	(_PPI_PORTA),a
;main.c:201: WR_IDLE;
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
;main.c:202: RD_IDLE;
	in	a,(_PPI_PORTA)
	set	0, a
	out	(_PPI_PORTA),a
;main.c:204: RESET_LOW;
	in	a,(_PPI_PORTA)
	and	a, #0xEF
	out	(_PPI_PORTA),a
;main.c:205: RESET_HIGH;
	in	a,(_PPI_PORTA)
	set	4, a
	out	(_PPI_PORTA),a
;main.c:207: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xF7
	out	(_PPI_PORTA),a
;main.c:208: CD_COMMAND;
	in	a,(_PPI_PORTA)
	and	a, #0xFB
	out	(_PPI_PORTA),a
;main.c:209: write8(0x00);
	xor	a, a
	push	af
	inc	sp
	call	_write8
	inc	sp
;main.c:211: for(i=0; i<3; i++)
	ld	d,#0x00
00102$:
;main.c:213: WR_STROBE;
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
;main.c:211: for(i=0; i<3; i++)
	inc	d
	ld	a,d
	sub	a, #0x03
	jr	C,00102$
;main.c:215: CS_IDLE;
	in	a,(_PPI_PORTA)
	set	3, a
	out	(_PPI_PORTA),a
;main.c:217: delay_ms(500);
	ld	hl,#0x01F4
	push	hl
	call	_delay_ms
	pop	af
	ret
_reset_end::
;main.c:220: void fillScreen(uint16_t color) {
;	---------------------------------
; Function fillScreen
; ---------------------------------
_fillScreen_start::
_fillScreen:
;main.c:227: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xF7
	out	(_PPI_PORTA),a
;main.c:228: writeRegister16(0x0020, x);
	ld	hl,#0x0000
	push	hl
	ld	l, #0x20
	push	hl
	call	_writeRegister16
	pop	af
	pop	af
;main.c:229: writeRegister16(0x0021, y);
	ld	hl,#0x0000
	push	hl
	ld	l, #0x21
	push	hl
	call	_writeRegister16
	pop	af
;main.c:231: flood(color, (long)TFTWIDTH * (long)TFTHEIGHT);
	ld	hl, #0x0001
	ex	(sp),hl
	ld	hl,#0x2C00
	push	hl
	ld	hl, #6
	add	hl, sp
	ld	c, (hl)
	inc	hl
	ld	b, (hl)
	push	bc
	call	_flood
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
	ret
_fillScreen_end::
;main.c:235: void flood(uint16_t color, uint32_t len) 
;	---------------------------------
; Function flood
; ---------------------------------
_flood_start::
_flood:
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-7
	add	hl,sp
	ld	sp,hl
;main.c:239: hi= color >> 8;
	ld	c,5 (ix)
;main.c:240: lo=color;         
	ld	a,4 (ix)
	ld	-7 (ix),a
;main.c:242: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xF7
	out	(_PPI_PORTA),a
;main.c:243: CD_COMMAND;
	in	a,(_PPI_PORTA)
	and	a, #0xFB
	out	(_PPI_PORTA),a
;main.c:245: write8(0x00);
	push	bc
	xor	a, a
	push	af
	inc	sp
	call	_write8
	inc	sp
	ld	a,#0x22
	push	af
	inc	sp
	call	_write8
	inc	sp
	pop	bc
;main.c:248: CD_DATA;
	in	a,(_PPI_PORTA)
	set	2, a
	out	(_PPI_PORTA),a
;main.c:249: write8(hi);
	push	bc
	ld	a,c
	push	af
	inc	sp
	call	_write8
	inc	sp
	ld	a,-7 (ix)
	push	af
	inc	sp
	call	_write8
	inc	sp
	pop	bc
;main.c:251: len--;
	ld	a,6 (ix)
	add	a,#0xFF
	ld	6 (ix),a
	ld	a,7 (ix)
	adc	a,#0xFF
	ld	7 (ix),a
	ld	a,8 (ix)
	adc	a,#0xFF
	ld	8 (ix),a
	ld	a,9 (ix)
	adc	a,#0xFF
	ld	9 (ix),a
;main.c:253: blocks = (uint16_t)(len / 64);
	push	af
	ld	e,6 (ix)
	ld	d,7 (ix)
	ld	h,8 (ix)
	ld	l,9 (ix)
	pop	af
	ld	b,#0x06
00183$:
	srl	l
	rr	h
	rr	d
	rr	e
	djnz	00183$
;main.c:265: for(i = (uint8_t)len & 63; i--; ) 
	ld	a,6 (ix)
	and	a, #0x3F
	ld	-5 (ix),a
;main.c:254: if(hi == lo) 
	ld	a,-7 (ix)
	sub	a, c
	jp	NZ,00135$
;main.c:256: while(blocks--) 
	ld	c, e
	ld	b, d
00104$:
	ld	h,c
	ld	l,b
	dec	bc
	ld	a,l
	or	a,h
	jr	Z,00106$
;main.c:259: do 
	ld	d,#0x10
00101$:
;main.c:261: WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE;
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
;main.c:262: WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; 
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
;main.c:263: }while(--i);
	dec	d
	ld	a,d
	or	a, a
	jr	NZ,00101$
	jr	00104$
00106$:
;main.c:265: for(i = (uint8_t)len & 63; i--; ) 
	ld	d,-5 (ix)
00119$:
	ld	h,d
	dec	d
	ld	a,h
	or	a, a
	jp	Z,00117$
;main.c:267: WR_STROBE;
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
;main.c:268: WR_STROBE;
	in	a,(_PPI_PORTA)
	and	a, #0xFD
	out	(_PPI_PORTA),a
	in	a,(_PPI_PORTA)
	set	1, a
	out	(_PPI_PORTA),a
	jr	00119$
;main.c:273: while(blocks--) 
00135$:
	ld	-4 (ix),e
	ld	-3 (ix),d
00111$:
	ld	a,-4 (ix)
	ld	-2 (ix),a
	ld	a,-3 (ix)
	ld	-1 (ix),a
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	dec	hl
	ld	-4 (ix),l
	ld	-3 (ix),h
	ld	a,-1 (ix)
	or	a,-2 (ix)
	jr	Z,00113$
;main.c:276: do 
	ld	-6 (ix),#0x10
00108$:
;main.c:278: write8(hi); write8(lo); write8(hi); write8(lo);
	push	bc
	ld	a,c
	push	af
	inc	sp
	call	_write8
	inc	sp
	ld	a,-7 (ix)
	push	af
	inc	sp
	call	_write8
	inc	sp
	pop	bc
	push	bc
	ld	a,c
	push	af
	inc	sp
	call	_write8
	inc	sp
	ld	a,-7 (ix)
	push	af
	inc	sp
	call	_write8
	inc	sp
	pop	bc
;main.c:279: write8(hi); write8(lo); write8(hi); write8(lo);
	push	bc
	ld	a,c
	push	af
	inc	sp
	call	_write8
	inc	sp
	ld	a,-7 (ix)
	push	af
	inc	sp
	call	_write8
	inc	sp
	pop	bc
	push	bc
	ld	a,c
	push	af
	inc	sp
	call	_write8
	inc	sp
	ld	a,-7 (ix)
	push	af
	inc	sp
	call	_write8
	inc	sp
	pop	bc
;main.c:280: }while(--i);
	dec	-6 (ix)
	ld	a,-6 (ix)
	or	a, a
	jr	NZ,00108$
	jr	00111$
00113$:
;main.c:282: for(i = (uint8_t)len & 63; i--; ) {
	ld	a,-5 (ix)
	ld	-2 (ix),a
00122$:
	ld	h,-2 (ix)
	dec	-2 (ix)
	ld	a,h
	or	a, a
	jr	Z,00117$
;main.c:283: write8(hi);
	push	bc
	ld	a,c
	push	af
	inc	sp
	call	_write8
	inc	sp
	ld	a,-7 (ix)
	push	af
	inc	sp
	call	_write8
	inc	sp
	pop	bc
	jr	00122$
00117$:
;main.c:287: CS_IDLE;
	in	a,(_PPI_PORTA)
	set	3, a
	out	(_PPI_PORTA),a
	ld	sp, ix
	pop	ix
	ret
_flood_end::
;main.c:290: void drawPixel(int16_t x, int16_t y, uint16_t color) 
;	---------------------------------
; Function drawPixel
; ---------------------------------
_drawPixel_start::
_drawPixel:
	push	ix
	ld	ix,#0
	add	ix,sp
;main.c:292: if((x < 0) || (y < 0) || (x >= TFTWIDTH) || (y >= TFTHEIGHT))
	bit	7, 5 (ix)
	jr	NZ,00106$
	bit	7, 7 (ix)
	jr	NZ,00106$
	ld	a,4 (ix)
	sub	a, #0xF0
	ld	a,5 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	NC,00106$
	ld	a,6 (ix)
	sub	a, #0x40
	ld	a,7 (ix)
	rla
	ccf
	rra
	sbc	a, #0x81
;main.c:294: return;
	jr	NC,00106$
;main.c:296: CS_ACTIVE;
	in	a,(_PPI_PORTA)
	and	a, #0xF7
	out	(_PPI_PORTA),a
;main.c:297: writeRegister16(0x0020, x);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	ld	hl,#0x0020
	push	hl
	call	_writeRegister16
	pop	af
	pop	af
;main.c:298: writeRegister16(0x0021, y);
	ld	l,6 (ix)
	ld	h,7 (ix)
	push	hl
	ld	hl,#0x0021
	push	hl
	call	_writeRegister16
	pop	af
	pop	af
;main.c:299: writeRegister16(0x0022, color);
	ld	l,8 (ix)
	ld	h,9 (ix)
	push	hl
	ld	hl,#0x0022
	push	hl
	call	_writeRegister16
	pop	af
	pop	af
;main.c:300: CS_IDLE;
	in	a,(_PPI_PORTA)
	set	3, a
	out	(_PPI_PORTA),a
00106$:
	pop	ix
	ret
_drawPixel_end::
;main.c:303: void fillRect(int16_t x1, int16_t y1, int16_t w, int16_t h,
;	---------------------------------
; Function fillRect
; ---------------------------------
_fillRect_start::
_fillRect:
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-8
	add	hl,sp
	ld	sp,hl
;main.c:307: if( (w <= 0 ) ||  (h  <= 0 ) || (x1 >= TFTWIDTH) ||  (y1 >= TFTHEIGHT) || ((x2 = x1+w-1) <  0 ) || ((y2  = y1+h-1) <  0 ))
	xor	a, a
	cp	a, 8 (ix)
	sbc	a, 9 (ix)
	jp	PO, 00154$
	xor	a, #0x80
00154$:
	jp	P,00116$
	xor	a, a
	cp	a, 10 (ix)
	sbc	a, 11 (ix)
	jp	PO, 00155$
	xor	a, #0x80
00155$:
	jp	P,00116$
	ld	a,4 (ix)
	sub	a, #0xF0
	ld	a,5 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	NC,00116$
	ld	a,6 (ix)
	sub	a, #0x40
	ld	a,7 (ix)
	rla
	ccf
	rra
	sbc	a, #0x81
	jp	NC,00116$
	ld	a,4 (ix)
	add	a, 8 (ix)
	ld	c,a
	ld	a,5 (ix)
	adc	a, 9 (ix)
	ld	b,a
	ld	e,c
	ld	d,b
	dec	de
	ld	l, e
	ld	h, d
	bit	7, d
	jp	NZ,00116$
	ld	a,6 (ix)
	add	a, 10 (ix)
	ld	e,a
	ld	a,7 (ix)
	adc	a, 11 (ix)
	ld	d,a
	ld	a,e
	add	a,#0xFF
	ld	-8 (ix),a
	ld	a,d
	adc	a,#0xFF
	ld	-7 (ix),a
	ld	a,-8 (ix)
	ld	-6 (ix),a
	ld	a,-7 (ix)
	ld	-5 (ix),a
	bit	7, -7 (ix)
;main.c:308: return;
	jp	NZ,00116$
;main.c:309: if(x1 < 0) 
	bit	7, 5 (ix)
	jr	Z,00109$
;main.c:311: w += x1;
	ld	8 (ix),c
	ld	9 (ix),b
;main.c:312: x1 = 0;
	ld	4 (ix),#0x00
	ld	5 (ix),#0x00
00109$:
;main.c:314: if(y1 < 0) 
	bit	7, 7 (ix)
	jr	Z,00111$
;main.c:316: h += y1;
	ld	10 (ix),e
	ld	11 (ix),d
;main.c:317: y1 = 0;
	ld	6 (ix),#0x00
	ld	7 (ix),#0x00
00111$:
;main.c:319: if(x2 >= TFTWIDTH) {
	ld	a,l
	sub	a, #0xF0
	ld	a,h
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00113$
;main.c:320: x2 = TFTWIDTH - 1;
	ld	hl,#0x00EF
;main.c:321: w  = x2 - x1 + 1;
	ld	a,#0xEF
	sub	a, 4 (ix)
	ld	e,a
	ld	a,#0x00
	sbc	a, 5 (ix)
	ld	d,a
	inc	de
	ld	8 (ix),e
	ld	9 (ix),d
00113$:
;main.c:323: if(y2 >= TFTHEIGHT) {
	ld	a,-6 (ix)
	sub	a, #0x40
	ld	a,-5 (ix)
	rla
	ccf
	rra
	sbc	a, #0x81
	jr	C,00115$
;main.c:324: y2 = TFTHEIGHT - 1;
	ld	-6 (ix),#0x3F
	ld	-5 (ix),#0x01
;main.c:325: h  = y2 - y1 + 1;
	ld	a,#0x3F
	sub	a, 6 (ix)
	ld	e,a
	ld	a,#0x01
	sbc	a, 7 (ix)
	ld	d,a
	inc	de
	ld	10 (ix),e
	ld	11 (ix),d
00115$:
;main.c:327: setAddrWindow(x1, y1, x2, y2);
	pop	de
	pop	bc
	push	bc
	push	de
	push	bc
	push	hl
	ld	l,6 (ix)
	ld	h,7 (ix)
	push	hl
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_setAddrWindow
	ld	hl,#8
	add	hl,sp
	ld	sp,hl
;main.c:328: flood(fillcolor, (uint32_t)w * (uint32_t)h);
	ld	e,8 (ix)
	ld	d,9 (ix)
	ld	a,9 (ix)
	rla
	sbc	a, a
	ld	c,a
	ld	b,a
	ld	a,10 (ix)
	ld	-4 (ix),a
	ld	a,11 (ix)
	ld	-3 (ix),a
	ld	a,11 (ix)
	rla
	sbc	a, a
	ld	-2 (ix),a
	ld	-1 (ix),a
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	push	hl
	push	bc
	push	de
	call	__mullong_rrx_s
	pop	af
	pop	af
	pop	af
	pop	af
	ex	de, hl
	push	hl
	push	de
	ld	l,12 (ix)
	ld	h,13 (ix)
	push	hl
	call	_flood
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
;main.c:329: setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
	ld	hl,#0x013F
	push	hl
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
00116$:
	ld	sp, ix
	pop	ix
	ret
_fillRect_end::
;main.c:332: void drawGraf()
;	---------------------------------
; Function drawGraf
; ---------------------------------
_drawGraf_start::
_drawGraf:
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-21
	add	hl,sp
	ld	sp,hl
;main.c:338: for(i = 0; i < 20; i++)
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	-2 (ix),#0x00
	ld	-1 (ix),#0x00
;main.c:340: for(y = 0; y < 20; y++)
00133$:
	ld	de,#0x0000
00111$:
;main.c:342: HEADER_PIXEL(header_data,pixeles);
	ld	hl,#0x0002
	add	hl,sp
	ld	-8 (ix),l
	ld	-7 (ix),h
	ld	hl,(_header_data)
	ld	a,(hl)
	add	a,#0xDF
	add	a, a
	add	a, a
	ld	c,a
	ld	hl,(_header_data)
	inc	hl
	ld	a,(hl)
	ld	l,a
	rla
	sbc	a, a
	ld	h,a
	ld	a,l
	add	a,#0xDF
	ld	l,a
	ld	a,h
	adc	a,#0xFF
	ld	h,a
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	ld	a,c
	rla
	sbc	a, a
	ld	b,a
	ld	a,c
	or	a, l
	ld	c,a
	ld	a,b
	or	a, h
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),c
	ld	a,-8 (ix)
	add	a, #0x01
	ld	-4 (ix),a
	ld	a,-7 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	ld	hl,(_header_data)
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x0F
	rlca
	rlca
	rlca
	rlca
	and	a,#0xF0
	ld	c,a
	ld	hl,(_header_data)
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	l,a
	rla
	sbc	a, a
	ld	h,a
	ld	a,l
	add	a,#0xDF
	ld	l,a
	ld	a,h
	adc	a,#0xFF
	ld	h,a
	sra	h
	rr	l
	sra	h
	rr	l
	ld	a,c
	rla
	sbc	a, a
	ld	b,a
	ld	a,c
	or	a, l
	ld	c,a
	ld	a,b
	or	a, h
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),c
	ld	a,-8 (ix)
	add	a, #0x02
	ld	-14 (ix),a
	ld	a,-7 (ix)
	adc	a, #0x00
	ld	-13 (ix),a
	ld	hl,(_header_data)
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x03
	rrca
	rrca
	and	a,#0xC0
	ld	c,a
	ld	hl,(_header_data)
	inc	hl
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	or	a, c
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	(hl),a
	ld	hl,#_header_data
	ld	a,(hl)
	add	a, #0x04
	ld	(hl),a
	inc	hl
	ld	a,(hl)
	adc	a, #0x00
	ld	(hl),a
;main.c:343: colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	a,(hl)
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	c,(hl)
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	b,(hl)
	push	de
	push	af
	inc	sp
	ld	a,c
	push	af
	inc	sp
	push	bc
	inc	sp
	call	_color565
	pop	af
	inc	sp
	pop	de
	ld	-12 (ix),l
	ld	-11 (ix),h
;main.c:344: cuadroMorado[i][y] = colorAux;
	ld	a,-2 (ix)
	add	a, #<(_cuadroMorado)
	ld	c,a
	ld	a,-1 (ix)
	adc	a, #>(_cuadroMorado)
	ld	b,a
	ld	l, e
	ld	h, d
	add	hl, hl
	add	hl,bc
	ld	a,-12 (ix)
	ld	(hl),a
	inc	hl
	ld	a,-11 (ix)
	ld	(hl),a
;main.c:340: for(y = 0; y < 20; y++)
	inc	de
	ld	a,e
	sub	a, #0x14
	ld	a,d
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00111$
;main.c:338: for(i = 0; i < 20; i++)
	ld	a,-2 (ix)
	add	a, #0x28
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a, #0x00
	ld	-1 (ix),a
	inc	-16 (ix)
	jr	NZ,00219$
	inc	-15 (ix)
00219$:
	ld	a,-16 (ix)
	sub	a, #0x14
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00133$
;main.c:347: for(i = 0; i < 20; i++)
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	de,#0x0000
;main.c:349: for(y = 0; y < 20; y++)
00137$:
	ld	hl,#0x0000
	ex	(sp), hl
00115$:
;main.c:351: HEADER_PIXEL(header_Rana,pixeles);
	ld	hl,(_header_Rana)
	ld	a,(hl)
	add	a,#0xDF
	add	a, a
	add	a, a
	ld	c,a
	ld	hl,(_header_Rana)
	inc	hl
	ld	a,(hl)
	ld	l,a
	rla
	sbc	a, a
	ld	h,a
	ld	a,l
	add	a,#0xDF
	ld	l,a
	ld	a,h
	adc	a,#0xFF
	ld	h,a
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	ld	a,c
	rla
	sbc	a, a
	ld	b,a
	ld	a,c
	or	a, l
	ld	c,a
	ld	a,b
	or	a, h
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),c
	ld	hl,(_header_Rana)
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x0F
	rlca
	rlca
	rlca
	rlca
	and	a,#0xF0
	ld	c,a
	ld	hl,(_header_Rana)
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	l,a
	rla
	sbc	a, a
	ld	h,a
	ld	a,l
	add	a,#0xDF
	ld	l,a
	ld	a,h
	adc	a,#0xFF
	ld	h,a
	sra	h
	rr	l
	sra	h
	rr	l
	ld	a,c
	rla
	sbc	a, a
	ld	b,a
	ld	a,c
	or	a, l
	ld	c,a
	ld	a,b
	or	a, h
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),c
	ld	hl,(_header_Rana)
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x03
	rrca
	rrca
	and	a,#0xC0
	ld	c,a
	ld	hl,(_header_Rana)
	inc	hl
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	or	a, c
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	(hl),a
	ld	hl,#_header_Rana
	ld	a,(hl)
	add	a, #0x04
	ld	(hl),a
	inc	hl
	ld	a,(hl)
	adc	a, #0x00
	ld	(hl),a
;main.c:352: colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	a,(hl)
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	c,(hl)
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	b,(hl)
	push	de
	push	af
	inc	sp
	ld	a,c
	push	af
	inc	sp
	push	bc
	inc	sp
	call	_color565
	pop	af
	inc	sp
	pop	de
	ld	-12 (ix),l
	ld	-11 (ix),h
;main.c:353: rana[i][y] = colorAux;
	ld	hl,#_rana+0
	add	hl,de
	ld	c,l
	ld	b,h
	pop	hl
	push	hl
	add	hl, hl
	add	hl,bc
	ld	a,-12 (ix)
	ld	(hl),a
	inc	hl
	ld	a,-11 (ix)
	ld	(hl),a
;main.c:349: for(y = 0; y < 20; y++)
	inc	-21 (ix)
	jr	NZ,00220$
	inc	-20 (ix)
00220$:
	ld	a,-21 (ix)
	sub	a, #0x14
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00115$
;main.c:347: for(i = 0; i < 20; i++)
	ld	hl,#0x0028
	add	hl,de
	ex	de,hl
	inc	-16 (ix)
	jr	NZ,00221$
	inc	-15 (ix)
00221$:
	ld	a,-16 (ix)
	sub	a, #0x14
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00137$
;main.c:356: for(i = 0; i < 20; i++)
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	-12 (ix),#0x00
	ld	-11 (ix),#0x00
;main.c:358: for(y = 0; y < 20; y++)
00141$:
	ld	hl,#0x0000
	ex	(sp), hl
00119$:
;main.c:360: HEADER_PIXEL(header_Rana2,pixeles);
	ld	hl,(_header_Rana2)
	ld	-2 (ix),l
	ld	-1 (ix),h
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	a,(hl)
	add	a,#0xDF
	add	a, a
	add	a, a
	ld	e,a
	ld	hl,(_header_Rana2)
	inc	hl
	ld	a,(hl)
	ld	-2 (ix),a
	rla
	sbc	a, a
	ld	-1 (ix),a
	ld	a,-2 (ix)
	add	a,#0xDF
	ld	l,a
	ld	a,-1 (ix)
	adc	a,#0xFF
	ld	h,a
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	ld	a,e
	rla
	sbc	a, a
	ld	d,a
	ld	a,e
	or	a, l
	ld	e,a
	ld	a,d
	or	a, h
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),e
	ld	hl,(_header_Rana2)
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x0F
	rlca
	rlca
	rlca
	rlca
	and	a,#0xF0
	ld	c,a
	ld	hl,(_header_Rana2)
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	l,a
	rla
	sbc	a, a
	ld	h,a
	ld	a,l
	add	a,#0xDF
	ld	l,a
	ld	a,h
	adc	a,#0xFF
	ld	h,a
	sra	h
	rr	l
	sra	h
	rr	l
	ld	a,c
	rla
	sbc	a, a
	ld	b,a
	ld	a,c
	or	a, l
	ld	d,a
	ld	a,b
	or	a, h
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),d
	ld	hl,(_header_Rana2)
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x03
	rrca
	rrca
	and	a,#0xC0
	ld	d,a
	ld	iy,(_header_Rana2)
	ld	a,3 (iy)
	add	a,#0xDF
	or	a, d
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	(hl),a
	ld	hl,#_header_Rana2
	ld	a,(hl)
	add	a, #0x04
	ld	(hl),a
	inc	hl
	ld	a,(hl)
	adc	a, #0x00
	ld	(hl),a
;main.c:361: colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	b,(hl)
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	d,(hl)
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	a,(hl)
	ld	c, d
	push	bc
	push	af
	inc	sp
	call	_color565
	pop	af
	inc	sp
	ld	d,l
	ld	e,h
;main.c:362: rana2[i][y] = colorAux;
	ld	a,-12 (ix)
	add	a, #<(_rana2)
	ld	c,a
	ld	a,-11 (ix)
	adc	a, #>(_rana2)
	ld	b,a
	pop	hl
	push	hl
	add	hl, hl
	add	hl,bc
	ld	(hl),d
	inc	hl
	ld	(hl),e
;main.c:358: for(y = 0; y < 20; y++)
	inc	-21 (ix)
	jr	NZ,00222$
	inc	-20 (ix)
00222$:
	ld	a,-21 (ix)
	sub	a, #0x14
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00119$
;main.c:356: for(i = 0; i < 20; i++)
	ld	a,-12 (ix)
	add	a, #0x28
	ld	-12 (ix),a
	ld	a,-11 (ix)
	adc	a, #0x00
	ld	-11 (ix),a
	inc	-16 (ix)
	jr	NZ,00223$
	inc	-15 (ix)
00223$:
	ld	a,-16 (ix)
	sub	a, #0x14
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00141$
;main.c:365: for(i = 0; i < 20; i++)
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	-12 (ix),#0x00
	ld	-11 (ix),#0x00
;main.c:367: for(y = 0; y < 20; y++)
00145$:
	ld	hl,#0x0000
	ex	(sp), hl
00123$:
;main.c:369: HEADER_PIXEL(header_Rana3,pixeles);
	ld	hl,(_header_Rana3)
	ld	-2 (ix),l
	ld	-1 (ix),h
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	a,(hl)
	ld	-2 (ix), a
	add	a,#0xDF
	ld	-2 (ix), a
	add	a, a
	add	a, a
	ld	-2 (ix),a
	ld	hl,(_header_Rana3)
	ld	-10 (ix),l
	ld	-9 (ix),h
	ld	l,-10 (ix)
	ld	h,-9 (ix)
	inc	hl
	ld	a,(hl)
	ld	-10 (ix), a
	ld	-10 (ix), a
	rla
	sbc	a, a
	ld	-9 (ix),a
	ld	a,-10 (ix)
	add	a,#0xDF
	ld	-10 (ix),a
	ld	a,-9 (ix)
	adc	a,#0xFF
	ld	-9 (ix),a
	ld	l,-10 (ix)
	ld	h,-9 (ix)
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	ld	e,-2 (ix)
	ld	a,-2 (ix)
	rla
	sbc	a, a
	ld	d,a
	ld	a,e
	or	a, l
	ld	e,a
	ld	a,d
	or	a, h
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),e
	ld	hl,(_header_Rana3)
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x0F
	ld	-10 (ix), a
	rlca
	rlca
	rlca
	rlca
	and	a,#0xF0
	ld	-10 (ix),a
	ld	hl,(_header_Rana3)
	ld	-2 (ix),l
	ld	-1 (ix),h
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	-2 (ix), a
	ld	-2 (ix), a
	rla
	sbc	a, a
	ld	-1 (ix),a
	ld	a,-2 (ix)
	add	a,#0xDF
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a,#0xFF
	ld	-1 (ix),a
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	sra	h
	rr	l
	sra	h
	rr	l
	ld	e,-10 (ix)
	ld	a,-10 (ix)
	rla
	sbc	a, a
	ld	d,a
	ld	a,e
	or	a, l
	ld	e,a
	ld	a,d
	or	a, h
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),e
	ld	hl,(_header_Rana3)
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x03
	ld	-10 (ix), a
	rrca
	rrca
	and	a,#0xC0
	ld	-10 (ix),a
	ld	hl,(_header_Rana3)
	ld	-2 (ix),l
	ld	-1 (ix),h
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	inc	hl
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	or	a, -10 (ix)
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	(hl),a
	ld	hl,#_header_Rana3
	ld	a,(hl)
	add	a, #0x04
	ld	(hl),a
	inc	hl
	ld	a,(hl)
	adc	a, #0x00
	ld	(hl),a
;main.c:370: colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	b,(hl)
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	d,(hl)
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	a,(hl)
	ld	c, d
	push	bc
	push	af
	inc	sp
	call	_color565
	pop	af
	inc	sp
	ld	-9 (ix),h
	ld	-10 (ix),l
;main.c:371: rana3[i][y] = colorAux;
	ld	a,#<(_rana3)
	add	a, -12 (ix)
	ld	-2 (ix),a
	ld	a,#>(_rana3)
	adc	a, -11 (ix)
	ld	-1 (ix),a
	ld	a,-21 (ix)
	ld	-6 (ix),a
	ld	a,-20 (ix)
	ld	-5 (ix),a
	sla	-6 (ix)
	rl	-5 (ix)
	ld	a,-2 (ix)
	add	a, -6 (ix)
	ld	-6 (ix),a
	ld	a,-1 (ix)
	adc	a, -5 (ix)
	ld	-5 (ix),a
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	a,-10 (ix)
	ld	(hl),a
	inc	hl
	ld	a,-9 (ix)
	ld	(hl),a
;main.c:367: for(y = 0; y < 20; y++)
	inc	-21 (ix)
	jr	NZ,00226$
	inc	-20 (ix)
00226$:
	ld	a,-21 (ix)
	sub	a, #0x14
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00123$
;main.c:365: for(i = 0; i < 20; i++)
	ld	a,-12 (ix)
	add	a, #0x28
	ld	-12 (ix),a
	ld	a,-11 (ix)
	adc	a, #0x00
	ld	-11 (ix),a
	inc	-16 (ix)
	jr	NZ,00227$
	inc	-15 (ix)
00227$:
	ld	a,-16 (ix)
	sub	a, #0x14
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00145$
;main.c:374: for(i = 0; i < 20; i++)
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	de,#0x0000
;main.c:376: for(y = 0; y < 20; y++)
00149$:
	ld	hl,#0x0000
	ex	(sp), hl
00127$:
;main.c:378: HEADER_PIXEL(header_Rana4,pixeles);
	ld	hl,(_header_Rana4)
	ld	a,(hl)
	add	a,#0xDF
	add	a, a
	add	a, a
	ld	c,a
	ld	hl,(_header_Rana4)
	inc	hl
	ld	a,(hl)
	ld	l,a
	rla
	sbc	a, a
	ld	h,a
	ld	a,l
	add	a,#0xDF
	ld	l,a
	ld	a,h
	adc	a,#0xFF
	ld	h,a
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	sra	h
	rr	l
	ld	a,c
	rla
	sbc	a, a
	ld	b,a
	ld	a,c
	or	a, l
	ld	c,a
	ld	a,b
	or	a, h
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),c
	ld	hl,(_header_Rana4)
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x0F
	rlca
	rlca
	rlca
	rlca
	and	a,#0xF0
	ld	c,a
	ld	hl,(_header_Rana4)
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	l,a
	rla
	sbc	a, a
	ld	h,a
	ld	a,l
	add	a,#0xDF
	ld	l,a
	ld	a,h
	adc	a,#0xFF
	ld	h,a
	sra	h
	rr	l
	sra	h
	rr	l
	ld	a,c
	rla
	sbc	a, a
	ld	b,a
	ld	a,c
	or	a, l
	ld	c,a
	ld	a,b
	or	a, h
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),c
	ld	hl,(_header_Rana4)
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	and	a, #0x03
	rrca
	rrca
	and	a,#0xC0
	ld	b,a
	ld	hl,(_header_Rana4)
	inc	hl
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a,#0xDF
	or	a, b
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	(hl),a
	ld	hl,#_header_Rana4
	ld	a,(hl)
	add	a, #0x04
	ld	(hl),a
	inc	hl
	ld	a,(hl)
	adc	a, #0x00
	ld	(hl),a
;main.c:379: colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	a,(hl)
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	b,(hl)
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	c,(hl)
	push	de
	push	af
	inc	sp
	push	bc
	call	_color565
	pop	af
	inc	sp
	pop	de
	ld	-6 (ix),l
	ld	-5 (ix),h
;main.c:380: rana4[i][y] = colorAux;
	ld	hl,#_rana4+0
	add	hl,de
	ld	c,l
	ld	b,h
	pop	hl
	push	hl
	add	hl, hl
	add	hl,bc
	ld	a,-6 (ix)
	ld	(hl),a
	inc	hl
	ld	a,-5 (ix)
	ld	(hl),a
;main.c:376: for(y = 0; y < 20; y++)
	inc	-21 (ix)
	jr	NZ,00228$
	inc	-20 (ix)
00228$:
	ld	a,-21 (ix)
	sub	a, #0x14
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00127$
;main.c:374: for(i = 0; i < 20; i++)
	ld	hl,#0x0028
	add	hl,de
	ex	de,hl
	inc	-16 (ix)
	jr	NZ,00229$
	inc	-15 (ix)
00229$:
	ld	a,-16 (ix)
	sub	a, #0x14
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00149$
	ld	sp, ix
	pop	ix
	ret
_drawGraf_end::
;main.c:385: void pintaSprite(int _x, int _y, int spr)
;	---------------------------------
; Function pintaSprite
; ---------------------------------
_pintaSprite_start::
_pintaSprite:
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-12
	add	hl,sp
	ld	sp,hl
;main.c:389: if(spr == 1)
	ld	a,8 (ix)
	dec	a
	jp	NZ,00104$
	ld	a,9 (ix)
	or	a, a
	jp	NZ,00104$
;main.c:391: for(y = 0; y < 20; y++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	de,#0x0000
;main.c:393: for(x = 0; x < 20; x++)
00152$:
	ld	a,-12 (ix)
	add	a, 6 (ix)
	ld	-6 (ix),a
	ld	a,-11 (ix)
	adc	a, 7 (ix)
	ld	-5 (ix),a
	ld	bc,#0x0000
00129$:
;main.c:395: drawPixel(x+_x, y+_y, cuadroMorado[y][x]);
	ld	hl,#_cuadroMorado+0
	add	hl,de
	ld	-4 (ix),l
	ld	-3 (ix),h
	ld	l, c
	ld	h, b
	add	hl, hl
	ld	a,-4 (ix)
	add	a, l
	ld	l,a
	ld	a,-3 (ix)
	adc	a, h
	ld	h,a
	ld	a, (hl)
	inc	hl
	ld	h,(hl)
	ld	l,a
	push	hl
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	pop	iy
	pop	hl
	add	iy, bc
	push	bc
	push	de
	push	hl
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	push	hl
	push	iy
	call	_drawPixel
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
	pop	de
	pop	bc
;main.c:393: for(x = 0; x < 20; x++)
	inc	bc
	ld	a,c
	sub	a, #0x14
	ld	a,b
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00129$
;main.c:391: for(y = 0; y < 20; y++)
	ld	hl,#0x0028
	add	hl,de
	ex	de,hl
	inc	-12 (ix)
	jr	NZ,00259$
	inc	-11 (ix)
00259$:
	ld	a,-12 (ix)
	sub	a, #0x14
	ld	a,-11 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00152$
00104$:
;main.c:399: if(spr == 2)
	ld	a,8 (ix)
	sub	a, #0x02
	jp	NZ,00110$
	ld	a,9 (ix)
	or	a, a
	jp	NZ,00110$
;main.c:401: for(y = 0; y < 20; y++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-4 (ix),#0x00
	ld	-3 (ix),#0x00
;main.c:403: for(x = 0; x < 20; x++)
00157$:
	ld	a,-12 (ix)
	add	a, 6 (ix)
	ld	-6 (ix),a
	ld	a,-11 (ix)
	adc	a, 7 (ix)
	ld	-5 (ix),a
	ld	-10 (ix),#0x00
	ld	-9 (ix),#0x00
00133$:
;main.c:405: if(rana[y][x] != 0x0000)
	ld	hl,#_rana+0
	ld	e,-4 (ix)
	ld	d,-3 (ix)
	add	hl,de
	ld	a,-10 (ix)
	ld	-8 (ix),a
	ld	a,-9 (ix)
	ld	-7 (ix),a
	sla	-8 (ix)
	rl	-7 (ix)
	ld	e,-8 (ix)
	ld	d,-7 (ix)
	add	hl,de
	ld	c,(hl)
	inc	hl
	ld	b,(hl)
	ld	a,b
	or	a,c
	jr	Z,00134$
;main.c:406: drawPixel(x+_x, y+_y, rana[y][x]);
	ld	a,#<(_rana)
	add	a, -4 (ix)
	ld	-2 (ix),a
	ld	a,#>(_rana)
	adc	a, -3 (ix)
	ld	-1 (ix),a
	ld	a,-2 (ix)
	add	a, -8 (ix)
	ld	l,a
	ld	a,-1 (ix)
	adc	a, -7 (ix)
	ld	h,a
	ld	e,(hl)
	inc	hl
	ld	d,(hl)
	ld	a,4 (ix)
	add	a, -10 (ix)
	ld	l,a
	ld	a,5 (ix)
	adc	a, -9 (ix)
	ld	h,a
	push	de
	ld	c,-6 (ix)
	ld	b,-5 (ix)
	push	bc
	push	hl
	call	_drawPixel
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
00134$:
;main.c:403: for(x = 0; x < 20; x++)
	inc	-10 (ix)
	jr	NZ,00264$
	inc	-9 (ix)
00264$:
	ld	a,-10 (ix)
	sub	a, #0x14
	ld	a,-9 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00133$
;main.c:401: for(y = 0; y < 20; y++)
	ld	a,-4 (ix)
	add	a, #0x28
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	inc	-12 (ix)
	jr	NZ,00265$
	inc	-11 (ix)
00265$:
	ld	a,-12 (ix)
	sub	a, #0x14
	ld	a,-11 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00157$
00110$:
;main.c:410: if(spr == 3)
	ld	a,8 (ix)
	sub	a, #0x03
	jp	NZ,00116$
	ld	a,9 (ix)
	or	a, a
	jp	NZ,00116$
;main.c:412: for(y = 0; y < 20; y++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-2 (ix),#0x00
	ld	-1 (ix),#0x00
;main.c:414: for(x = 0; x < 20; x++)
00162$:
	ld	a,-12 (ix)
	add	a, 6 (ix)
	ld	-8 (ix),a
	ld	a,-11 (ix)
	adc	a, 7 (ix)
	ld	-7 (ix),a
	ld	-10 (ix),#0x00
	ld	-9 (ix),#0x00
00137$:
;main.c:416: if(rana2[y][x] != 0x0000)
	ld	hl,#_rana2+0
	ld	e,-2 (ix)
	ld	d,-1 (ix)
	add	hl,de
	ld	a,-10 (ix)
	ld	-4 (ix),a
	ld	a,-9 (ix)
	ld	-3 (ix),a
	sla	-4 (ix)
	rl	-3 (ix)
	ld	e,-4 (ix)
	ld	d,-3 (ix)
	add	hl,de
	ld	c,(hl)
	inc	hl
	ld	b,(hl)
	ld	a,b
	or	a,c
	jr	Z,00138$
;main.c:417: drawPixel(x+_x, y+_y, rana2[y][x]);
	ld	a,#<(_rana2)
	add	a, -2 (ix)
	ld	-6 (ix),a
	ld	a,#>(_rana2)
	adc	a, -1 (ix)
	ld	-5 (ix),a
	ld	a,-6 (ix)
	add	a, -4 (ix)
	ld	l,a
	ld	a,-5 (ix)
	adc	a, -3 (ix)
	ld	h,a
	ld	e,(hl)
	inc	hl
	ld	d,(hl)
	ld	a,4 (ix)
	add	a, -10 (ix)
	ld	l,a
	ld	a,5 (ix)
	adc	a, -9 (ix)
	ld	h,a
	push	de
	ld	c,-8 (ix)
	ld	b,-7 (ix)
	push	bc
	push	hl
	call	_drawPixel
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
00138$:
;main.c:414: for(x = 0; x < 20; x++)
	inc	-10 (ix)
	jr	NZ,00270$
	inc	-9 (ix)
00270$:
	ld	a,-10 (ix)
	sub	a, #0x14
	ld	a,-9 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00137$
;main.c:412: for(y = 0; y < 20; y++)
	ld	a,-2 (ix)
	add	a, #0x28
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a, #0x00
	ld	-1 (ix),a
	inc	-12 (ix)
	jr	NZ,00271$
	inc	-11 (ix)
00271$:
	ld	a,-12 (ix)
	sub	a, #0x14
	ld	a,-11 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00162$
00116$:
;main.c:421: if(spr == 4)
	ld	a,8 (ix)
	sub	a, #0x04
	jp	NZ,00122$
	ld	a,9 (ix)
	or	a, a
	jp	NZ,00122$
;main.c:423: for(y = 0; y < 20; y++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-2 (ix),#0x00
	ld	-1 (ix),#0x00
;main.c:425: for(x = 0; x < 20; x++)
00167$:
	ld	a,-12 (ix)
	add	a, 6 (ix)
	ld	-8 (ix),a
	ld	a,-11 (ix)
	adc	a, 7 (ix)
	ld	-7 (ix),a
	ld	-10 (ix),#0x00
	ld	-9 (ix),#0x00
00141$:
;main.c:427: if(rana3[y][x] != 0x0000)
	ld	hl,#_rana3+0
	ld	e,-2 (ix)
	ld	d,-1 (ix)
	add	hl,de
	ld	a,-10 (ix)
	ld	-4 (ix),a
	ld	a,-9 (ix)
	ld	-3 (ix),a
	sla	-4 (ix)
	rl	-3 (ix)
	ld	e,-4 (ix)
	ld	d,-3 (ix)
	add	hl,de
	ld	c,(hl)
	inc	hl
	ld	b,(hl)
	ld	a,b
	or	a,c
	jr	Z,00142$
;main.c:428: drawPixel(x+_x, y+_y, rana3[y][x]);
	ld	a,#<(_rana3)
	add	a, -2 (ix)
	ld	-6 (ix),a
	ld	a,#>(_rana3)
	adc	a, -1 (ix)
	ld	-5 (ix),a
	ld	a,-6 (ix)
	add	a, -4 (ix)
	ld	l,a
	ld	a,-5 (ix)
	adc	a, -3 (ix)
	ld	h,a
	ld	e,(hl)
	inc	hl
	ld	d,(hl)
	ld	a,4 (ix)
	add	a, -10 (ix)
	ld	l,a
	ld	a,5 (ix)
	adc	a, -9 (ix)
	ld	h,a
	push	de
	ld	c,-8 (ix)
	ld	b,-7 (ix)
	push	bc
	push	hl
	call	_drawPixel
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
00142$:
;main.c:425: for(x = 0; x < 20; x++)
	inc	-10 (ix)
	jr	NZ,00276$
	inc	-9 (ix)
00276$:
	ld	a,-10 (ix)
	sub	a, #0x14
	ld	a,-9 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00141$
;main.c:423: for(y = 0; y < 20; y++)
	ld	a,-2 (ix)
	add	a, #0x28
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a, #0x00
	ld	-1 (ix),a
	inc	-12 (ix)
	jr	NZ,00277$
	inc	-11 (ix)
00277$:
	ld	a,-12 (ix)
	sub	a, #0x14
	ld	a,-11 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00167$
00122$:
;main.c:432: if(spr == 5)
	ld	a,8 (ix)
	sub	a, #0x05
	jp	NZ,00149$
	ld	a,9 (ix)
	or	a, a
	jp	NZ,00149$
;main.c:434: for(y = 0; y < 20; y++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-2 (ix),#0x00
	ld	-1 (ix),#0x00
;main.c:436: for(x = 0; x < 20; x++)
00172$:
	ld	a,-12 (ix)
	add	a, 6 (ix)
	ld	-8 (ix),a
	ld	a,-11 (ix)
	adc	a, 7 (ix)
	ld	-7 (ix),a
	ld	-10 (ix),#0x00
	ld	-9 (ix),#0x00
00145$:
;main.c:438: if(rana4[y][x] != 0x0000)
	ld	a,#<(_rana4)
	add	a, -2 (ix)
	ld	-4 (ix),a
	ld	a,#>(_rana4)
	adc	a, -1 (ix)
	ld	-3 (ix),a
	pop	bc
	pop	de
	push	de
	push	bc
	sla	e
	rl	d
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	add	hl,de
	ld	c,(hl)
	inc	hl
	ld	a, (hl)
	or	a,c
	jr	Z,00146$
;main.c:439: drawPixel(x+_x, y+_y, rana4[y][x]);
	ld	hl,#_rana4+0
	ld	c,-2 (ix)
	ld	b,-1 (ix)
	add	hl,bc
	add	hl,de
	ld	e,(hl)
	inc	hl
	ld	d,(hl)
	ld	a,-10 (ix)
	add	a, 4 (ix)
	ld	l,a
	ld	a,-9 (ix)
	adc	a, 5 (ix)
	ld	h,a
	push	de
	ld	c,-8 (ix)
	ld	b,-7 (ix)
	push	bc
	push	hl
	call	_drawPixel
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
00146$:
;main.c:436: for(x = 0; x < 20; x++)
	inc	-10 (ix)
	jr	NZ,00282$
	inc	-9 (ix)
00282$:
	ld	a,-10 (ix)
	sub	a, #0x14
	ld	a,-9 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00145$
;main.c:434: for(y = 0; y < 20; y++)
	ld	a,-2 (ix)
	add	a, #0x28
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a, #0x00
	ld	-1 (ix),a
	inc	-12 (ix)
	jr	NZ,00283$
	inc	-11 (ix)
00283$:
	ld	a,-12 (ix)
	sub	a, #0x14
	ld	a,-11 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00172$
00149$:
	ld	sp, ix
	pop	ix
	ret
_pintaSprite_end::
;main.c:445: uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
;	---------------------------------
; Function color565
; ---------------------------------
_color565_start::
_color565:
	push	ix
	ld	ix,#0
	add	ix,sp
;main.c:446: return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
	ld	a,4 (ix)
	and	a, #0xF8
	ld	d,a
	ld	e,#0x00
	ld	a,5 (ix)
	and	a, #0xFC
	ld	l,a
	ld	h,#0x00
	add	hl, hl
	add	hl, hl
	add	hl, hl
	ld	a,l
	or	a, e
	ld	l,a
	ld	a,h
	or	a, d
	ld	h,a
	ld	a,6 (ix)
	rrca
	rrca
	rrca
	and	a,#0x1F
	ld	e,a
	ld	d,#0x00
	ld	a,l
	or	a, e
	ld	l,a
	ld	a,h
	or	a, d
	ld	h,a
	pop	ix
	ret
_color565_end::
;main.c:449: void init_pantalla()
;	---------------------------------
; Function init_pantalla
; ---------------------------------
_init_pantalla_start::
_init_pantalla:
;main.c:451: drawGraf();  //carga imagenes
	call	_drawGraf
;main.c:453: pintaAzul(); 
	call	_pintaAzul
;main.c:454: pintaNegro();
	call	_pintaNegro
;main.c:456: pintaBarra(0);
	ld	hl,#0x0000
	push	hl
	call	_pintaBarra
	pop	af
;main.c:457: pintaSprite(jug.x, jug.y, 2);
	ld	de, (#_jug + 4)
	ld	hl, (#_jug + 2)
	ld	bc,#0x0002
	push	bc
	push	de
	push	hl
	call	_pintaSprite
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
	ret
_init_pantalla_end::
;main.c:460: void pintaAzul()
;	---------------------------------
; Function pintaAzul
; ---------------------------------
_pintaAzul_start::
_pintaAzul:
;main.c:462: fillRect(0, 0, 240, 160, BLUE);
	ld	hl,#0x001F
	push	hl
	ld	l, #0xA0
	push	hl
	ld	l, #0xF0
	push	hl
	ld	l, #0x00
	push	hl
	ld	l, #0x00
	push	hl
	call	_fillRect
	ld	hl,#10
	add	hl,sp
	ld	sp,hl
	ret
_pintaAzul_end::
;main.c:465: void pintaNegro()
;	---------------------------------
; Function pintaNegro
; ---------------------------------
_pintaNegro_start::
_pintaNegro:
;main.c:467: fillRect(0, 160, 240, 320, BLACK);
	ld	hl,#0x0000
	push	hl
	ld	hl,#0x0140
	push	hl
	ld	hl,#0x00F0
	push	hl
	ld	l, #0xA0
	push	hl
	ld	l, #0x00
	push	hl
	call	_fillRect
	ld	hl,#10
	add	hl,sp
	ld	sp,hl
	ret
_pintaNegro_end::
;main.c:470: void pintaBarra(int i)
;	---------------------------------
; Function pintaBarra
; ---------------------------------
_pintaBarra_start::
_pintaBarra:
;main.c:472: if(i == 0)
	ld	hl, #2+1
	add	hl, sp
	ld	a, (hl)
	dec	hl
	or	a,(hl)
	ret	NZ
;main.c:473: for(i=0; i < 12; i++){
	ld	de,#0x0000
	ld	hl,#0x0000
00104$:
;main.c:474: pintaSprite(i*20, 280, 1);
	push	hl
	push	de
	ld	bc,#0x0001
	push	bc
	ld	bc,#0x0118
	push	bc
	push	hl
	call	_pintaSprite
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
	pop	de
	pop	hl
;main.c:475: pintaSprite(i*20, 160, 1);
	push	hl
	push	de
	ld	bc,#0x0001
	push	bc
	ld	bc,#0x00A0
	push	bc
	push	hl
	call	_pintaSprite
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
	pop	de
	pop	hl
;main.c:473: for(i=0; i < 12; i++){
	ld	bc,#0x0014
	add	hl,bc
	inc	de
	ld	a,e
	sub	a, #0x0C
	ld	a,d
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00104$
	ret
_pintaBarra_end::
;main.c:479: void repinta_rana(int i)
;	---------------------------------
; Function repinta_rana
; ---------------------------------
_repinta_rana_start::
_repinta_rana:
	push	ix
	ld	ix,#0
	add	ix,sp
;main.c:481: if(jug.altura <= 7)
	ld	hl, (#_jug + 0)
	ld	a,#0x07
	cp	a, l
	ld	a,#0x00
	sbc	a, h
	jp	PO, 00131$
	xor	a, #0x80
00131$:
	jp	M,00102$
;main.c:483: fillRect(jug.x, jug.y, 20, 20, BLUE);
	ld	de, (#_jug + 4)
	ld	hl, (#_jug + 2)
	ld	bc,#0x001F
	push	bc
	ld	bc,#0x0014
	push	bc
	ld	bc,#0x0014
	push	bc
	push	de
	push	hl
	call	_fillRect
	ld	hl,#10
	add	hl,sp
	ld	sp,hl
	jr	00103$
00102$:
;main.c:487: fillRect(jug.x, jug.y, 20, 20, BLACK);
	ld	de, (#_jug + 4)
	ld	hl, (#_jug + 2)
	ld	bc,#0x0000
	push	bc
	ld	bc,#0x0014
	push	bc
	ld	bc,#0x0014
	push	bc
	push	de
	push	hl
	call	_fillRect
	ld	hl,#10
	add	hl,sp
	ld	sp,hl
00103$:
;main.c:489: switch(jug.altura)
	ld	hl, (#_jug + 0)
	ld	a,l
	sub	a,#0x08
	jr	NZ,00132$
	or	a,h
	jr	Z,00105$
00132$:
	ld	a,l
	sub	a,#0x0E
	jr	NZ,00106$
	or	a,h
	jr	NZ,00106$
;main.c:492: case 14:
00105$:
;main.c:493: pintaSprite(jug.x, jug.y, 2);
	ld	de, (#_jug + 4)
	ld	hl, (#_jug + 2)
	ld	bc,#0x0002
	push	bc
	push	de
	push	hl
	call	_pintaSprite
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
;main.c:494: }
00106$:
;main.c:496: switch(i)
	ld	a,4 (ix)
	dec	a
	jr	NZ,00135$
	ld	a,5 (ix)
	or	a, a
	jr	Z,00108$
00135$:
	ld	a,4 (ix)
	sub	a, #0x02
	jr	NZ,00113$
	ld	a,5 (ix)
	or	a, a
	jr	NZ,00113$
;main.c:499: case 2:
00108$:
;main.c:500: if((jug.x + 20) <= 220)
	ld	de, (#(_jug + 0x0002) + 0)
	ld	hl,#0x0014
	add	hl,de
	ld	c,l
	ld	b,h
;main.c:503: pintaSprite(jug.x, jug.y, 3);
;main.c:500: if((jug.x + 20) <= 220)
	ld	a,#0xDC
	cp	a, c
	ld	a,#0x00
	sbc	a, b
	jp	PO, 00138$
	xor	a, #0x80
00138$:
	jp	M,00110$
;main.c:502: jug.x += 20;
	ld	((_jug + 0x0002)), bc
;main.c:503: pintaSprite(jug.x, jug.y, 3);
	ld	hl, (#(_jug + 0x0004) + 0)
	ld	de,#0x0003
	push	de
	push	hl
	push	bc
	call	_pintaSprite
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
	jr	00113$
00110$:
;main.c:507: pintaSprite(jug.x, jug.y, 3);
	ld	hl, (#(_jug + 0x0004) + 0)
	ld	bc,#0x0003
	push	bc
	push	hl
	push	de
	call	_pintaSprite
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
;main.c:510: }
00113$:
	pop	ix
	ret
_repinta_rana_end::
	.area _CODE
___str_0:
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!FQ$XFQ$XFQ$X!!!!FQ$X!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!FQ$X!!!!!!!!!!!!!!!!FQ$X!!!!!!!!!!!!FQ$XFQ$X!!!!!!!!"
	.ascii "FQ$X!!!!FQ$XFQ$XFQ$X!!!!!!!!FQ$XFQ$X!!!!FQ$X!!!!FQ$XFQ$XFQ$X"
	.ascii "!!!!FQ$XFQ$X!!!!!!!!FQ$X!!!!FQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!!!"
	.ascii "FQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X"
	.ascii "FQ$XFQ$XFQ$XFQ$X!!!!FQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$X!!$X"
	.ascii "FQ$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!$X`Q!!"
	.ascii "!!$XFQ$XFQ$X!!$X`Q!!!!$XFQ$XFQ$XFQ$X!!$X`Q!!!!$XFQ$XFQ$XFQ$X"
	.ascii "FQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$X"
	.ascii "!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X"
	.ascii "`Q!!FQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$X"
	.ascii "!!$XFQ$XFQ$XFQ$XFQ$X`Q!!!!$X`Q!!!!$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$X"
	.ascii "!!$X`Q!!!!$XFQ$X!!$X`Q!!!!$XFQ$XFQ$X!!$X`Q!!FQ$X!!$XFQ$XFQ$X"
	.ascii "FQ$XFQ$X!!$X`Q!!!!$XFQ$X!!$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$X!!$X`Q!!"
	.ascii "!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X"
	.ascii "FQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$X"
	.ascii "FQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X`Q!!FQ$XFQ$X!!$X`Q!!"
	.ascii "!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!$X`Q!!!!$XFQ$XFQ$XFQ$XFQ$XFQ$X"
	.ascii "!!!!FQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$X"
	.ascii "FQ$X!!$XFQ$XFQ$X!!$X`Q!!!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$X"
	.ascii "FQ$XFQ$XFQ$XFQ$XFQ$X!!$X`Q!!!!$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$X"
	.ascii "FQ$XFQ$X!!$X`Q!!!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$X"
	.ascii "FQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$X!!$XFQ$X!!!!!!!!FQ$XFQ$X!!$X"
	.ascii "FQ$XFQ$XFQ$XFQ$XFQ$X!!!!FQ$XFQ$X!!$X`Q!!!!$X!!!!!!!!FQ$XFQ$X"
	.ascii "!!!!!!!!FQ$XFQ$X!!!!FQ$X!!!!FQ$XFQ$XFQ$X!!!!FQ$XFQ$XFQ$X!!$X"
	.ascii "FQ$X!!!!!!!!!!!!FQ$X!!!!!!!!FQ$XFQ$X!!!!!!!!!!!!FQ$XFQ$X!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FQ$X"
	.db 0x00
___str_1:
	.ascii "!!!!!!!!!!!!'"
	.db 0x5C
	.ascii "5!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!'"
	.db 0x5C
	.ascii "5!!!!!!!!!!!!!!!!!!!!!(==!(^=!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!(^=!(==!!!!!!!!!!!!!'"
	.db 0x5C
	.ascii "M!)>]!(--!!!!!"
	.ascii "!!!!!!!![/E#C@!!1?-!``]!``]!``]!!!!!!!!!!!!!(--!)>]!'"
	.db 0x5C
	.ascii "M!!!!!"
	.ascii "!!!!F`]!&J5!(-Y!!!!!`Q$XWAX.7=E>A`I!Y`M!``]!H@Y!6]E>WAX.`Q$X"
	.ascii "!!!!(-Y!&J5!F`]!!!!!!!!!!!!!%HM!(NE!!!!!`Q$X`Q$X$-96A09!``]!"
	.ascii "``]!>`1!$=96`Q$X`Q$X!!!!(NE!%HM!!!!!!!!!!!!!!!!!%89!(=5!!!!!"
	.ascii "%'Q!)_9!(^]!FO]!``]!``]!F?]!(N]!)_9!%'Q!!!!!(=5!%89!!!!!!!!!"
	.ascii "!!!!!!!!!!!!F`]!$[=!=/5!U0]!W@I!"
	.db 0x5C
	.ascii "`Q!``]!``]!``]!``]!``]!=?5!"
	.ascii "$[=!F`]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!F`]!G>U!``]!S`=!``]!``]!"
	.ascii "``]!``]!``]!``]!F>Q!F`]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "F`]!``]!,_%!A?Q!``]!``]!``]!``]!``]!F`]!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!F`]!``]!(.Y!@?Q!``]!``]!`@]!``]!``]!F`]!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!F`]!N@5!=/M!BOU!``]!"
	.ascii "^`Y!``]!``]!S0=!F`]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "F`]!0_-!``]!E/Y!,_!!YPM!``]!ZPQ!0?-!F`]!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!%HA!%]A!)NY!9_A!E?]!``]!``]!PP5!6O9!(^Y!%]A!"
	.ascii "%HA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#UY!)>Y!)>M!&+9!!+9!<L9!``]!"
	.ascii "``]!:,9!!+9!&;9!)>M!)>Y!#UY!!!!!!!!!!!!!!!!!!!!!$WA!)>]!)>]!"
	.ascii ")>Y!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)>Y!)>]!)>]!$WA!!!!!!!!!"
	.ascii "!!!!!!!!%85!)>Y!)>Y!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii ")>Y!)>Y!%85!!!!!!!!!!!!!!!!!&J9!(MM!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!(MM!&J9!!!!!!!!!!!!!'"
	.db 0x5C
	.ascii "M!)>]!(--!F`]!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!F`]!(--!)>]!'"
	.db 0x5C
	.ascii "M!!!!!"
	.ascii "!!!!F`]!(==!(^=!F`]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "F`]!(^=!(==!F`]!!!!!!!!!!!!!$W5!'"
	.db 0x5C
	.ascii "5!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!'"
	.db 0x5C
	.ascii "5!$W5!!!!!!!!!"
	.db 0x00
___str_2:
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!F`]!'"
	.db 0x5C
	.ascii "M!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!F`]!'"
	.db 0x5C
	.ascii "M!!!!!!!!!$W5!(==!)>]!&J9!%85!"
	.ascii "$WA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%89!%HM!&J5!)>]!(==!!!!!"
	.ascii "'"
	.db 0x5C
	.ascii "5!(^=!(--!(MM!)>Y!)>]!#UY!!!!!!!!!!!!!!!!!!!!!!!!!F`]!(=5!"
	.ascii "(NE!(-Y!(--!(^=!'"
	.db 0x5C
	.ascii "5!!!!!F`]!F`]!!!!!)>Y!)>]!)>Y!%HA!!!!!!!!!"
	.ascii "!!!!!!!!F`]!$[=!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii ")>Y!)>M!%]A!F`]!F`]!F`]!F`]!G>U!=/5!%'Q!`Q$X`Q$X!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!&+9!)NY!0_-!N@5!``]!``]!``]!U0]!)_9!"
	.ascii "`Q$XWAX.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!+9!9_A!``]!=/M!"
	.ascii "(.Y!,_%!S`=!W@I!(^]!$-967=E>[/E#!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!<L9!E?]!E/Y!BOU!@?Q!A?Q!``]!"
	.db 0x5C
	.ascii "`Q!FO]!A09!A`I!C@!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!``]!``]!,_!!``]!``]!``]!``]!``]!``]!"
	.ascii "``]!Y`M!1?-!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!``]!``]!YPM!^`Y!"
	.ascii "``]!``]!``]!``]!``]!``]!``]!``]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!:,9!PP5!``]!``]!`@]!``]!``]!``]!F?]!>`1!H@Y!``]!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!+9!6O9!ZPQ!``]!``]!``]!``]!``]!(N]!"
	.ascii "$=966]E>``]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!&;9!(^Y!0?-!S0=!"
	.ascii "``]!``]!``]!``]!)_9!`Q$XWAX.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii ")>Y!)>M!%]A!F`]!F`]!F`]!F`]!F>Q!=?5!%'Q!`Q$X`Q$X!!!!!!!!!!!!"
	.ascii "!!!!F`]!F`]!!!!!)>Y!)>]!)>Y!%HA!!!!!!!!!!!!!!!!!F`]!$[=!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!'"
	.db 0x5C
	.ascii "5!(^=!(--!(MM!)>Y!)>]!#UY!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!F`]!(=5!(NE!(-Y!(--!(^=!'"
	.db 0x5C
	.ascii "5!$W5!(==!)>]!&J9!%85!"
	.ascii "$WA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%89!%HM!&J5!)>]!(==!!!!!"
	.ascii "!!!!F`]!'"
	.db 0x5C
	.ascii "M!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!F`]!'"
	.db 0x5C
	.ascii "M!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.db 0x00
___str_3:
	.ascii "!!!!!!!!$W5!'"
	.db 0x5C
	.ascii "5!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!'"
	.db 0x5C
	.ascii "5!$W5!!!!!!!!!!!!!F`]!(==!(^=!F`]!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!F`]!(^=!(==!F`]!!!!!!!!!'"
	.db 0x5C
	.ascii "M!)>]!(--!F`]!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!F`]!(--!)>]!'"
	.db 0x5C
	.ascii "M!!!!!"
	.ascii "!!!!!!!!&J9!(MM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!(MM!&J9!!!!!!!!!!!!!!!!!%85!)>Y!)>Y!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!)>Y!)>Y!%85!!!!!!!!!!!!!!!!!$WA!)>]!)>]!"
	.ascii ")>Y!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)>Y!)>]!)>]!$WA!!!!!!!!!"
	.ascii "!!!!!!!!!!!!#UY!)>Y!)>M!&;9!!+9!:,9!``]!``]!<L9!!+9!&+9!)>M!"
	.ascii ")>Y!#UY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%HA!%]A!(^Y!6O9!PP5!``]!"
	.ascii "``]!E?]!9_A!)NY!%]A!%HA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "F`]!0?-!ZPQ!``]!YPM!,_!!E/Y!``]!0_-!F`]!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!F`]!S0=!``]!``]!^`Y!``]!BOU!=/M!N@5!F`]!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!F`]!``]!``]!`@]!``]!"
	.ascii "``]!@?Q!(.Y!``]!F`]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "F`]!``]!``]!``]!``]!``]!A?Q!,_%!``]!F`]!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!F`]!F>Q!``]!``]!``]!``]!``]!``]!S`=!``]!G>U!"
	.ascii "F`]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!F`]!$[=!=?5!``]!``]!``]!``]!"
	.ascii "``]!"
	.db 0x5C
	.ascii "`Q!W@I!U0]!=/5!$[=!F`]!!!!!!!!!!!!!!!!!!!!!%89!(=5!!!!!"
	.ascii "%'Q!)_9!(N]!F?]!``]!``]!FO]!(^]!)_9!%'Q!!!!!(=5!%89!!!!!!!!!"
	.ascii "!!!!!!!!%HM!(NE!!!!!`Q$X`Q$X$=96>`1!``]!``]!A09!$-96`Q$X`Q$X"
	.ascii "!!!!(NE!%HM!!!!!!!!!!!!!F`]!&J5!(-Y!!!!!`Q$XWAX.6]E>H@Y!``]!"
	.ascii "Y`M!A`I!7=E>WAX.`Q$X!!!!(-Y!&J5!F`]!!!!!!!!!'"
	.db 0x5C
	.ascii "M!)>]!(--!!!!!"
	.ascii "!!!!!!!!``]!``]!``]!1?-!C@!![/E#!!!!!!!!!!!!(--!)>]!'"
	.db 0x5C
	.ascii "M!!!!!"
	.ascii "!!!!!!!!(==!(^=!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!(^=!(==!!!!!!!!!!!!!!!!!!!!!'"
	.db 0x5C
	.ascii "5!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!'"
	.db 0x5C
	.ascii "5!!!!!!!!!!!!!"
	.db 0x00
___str_4:
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!'"
	.db 0x5C
	.ascii "M!F`]!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!'"
	.db 0x5C
	.ascii "M!F`]!!!!!!!!!(==!)>]!&J5!%HM!"
	.ascii "%89!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!$WA!%85!&J9!)>]!(==!$W5!"
	.ascii "'"
	.db 0x5C
	.ascii "5!(^=!(--!(-Y!(NE!(=5!F`]!!!!!!!!!!!!!!!!!!!!!!!!!#UY!)>]!"
	.ascii ")>Y!(MM!(--!(^=!'"
	.db 0x5C
	.ascii "5!!!!!!!!!!!!!!!!!!!!!!!!!$[=!F`]!!!!!!!!!"
	.ascii "!!!!!!!!%HA!)>Y!)>]!)>Y!!!!!F`]!F`]!!!!!!!!!!!!!!!!!`Q$X`Q$X"
	.ascii "%'Q!=?5!F>Q!F`]!F`]!F`]!F`]!%]A!)>M!)>Y!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!WAX.`Q$X)_9!``]!``]!``]!``]!S0=!0?-!(^Y!&;9!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!``]!6]E>$=96(N]!``]!``]!``]!``]!"
	.ascii "``]!ZPQ!6O9!!+9!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!``]!H@Y!>`1!"
	.ascii "F?]!``]!``]!``]!`@]!``]!``]!PP5!:,9!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!``]!``]!``]!``]!``]!``]!``]!``]!^`Y!YPM!``]!``]!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!1?-!Y`M!``]!``]!``]!``]!``]!``]!"
	.ascii "``]!,_!!``]!``]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!C@!!A`I!A09!"
	.ascii "FO]!"
	.db 0x5C
	.ascii "`Q!``]!A?Q!@?Q!BOU!E/Y!E?]!<L9!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!![/E#7=E>$-96(^]!W@I!S`=!,_%!(.Y!=/M!``]!9_A!!+9!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!WAX.`Q$X)_9!U0]!``]!``]!``]!"
	.ascii "N@5!0_-!)NY!&+9!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!`Q$X`Q$X"
	.ascii "%'Q!=/5!G>U!F`]!F`]!F`]!F`]!%]A!)>M!)>Y!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!$[=!F`]!!!!!!!!!!!!!!!!!%HA!)>Y!)>]!"
	.ascii ")>Y!!!!!F`]!F`]!!!!!'"
	.db 0x5C
	.ascii "5!(^=!(--!(-Y!(NE!(=5!F`]!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!#UY!)>]!)>Y!(MM!(--!(^=!'"
	.db 0x5C
	.ascii "5!!!!!(==!)>]!&J5!%HM!"
	.ascii "%89!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!$WA!%85!&J9!)>]!(==!$W5!"
	.ascii "!!!!!!!!'"
	.db 0x5C
	.ascii "M!F`]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!'"
	.db 0x5C
	.ascii "M!F`]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.ascii "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	.db 0x00
	.area _INITIALIZER
__xinit__width:
	.dw #0x0014
__xinit__height:
	.dw #0x0014
__xinit__header_data:
	.dw ___str_0
__xinit__width_Rana:
	.dw #0x0014
__xinit__height_Rana:
	.dw #0x0014
__xinit__header_Rana:
	.dw ___str_1
__xinit__width_Rana2:
	.dw #0x0014
__xinit__height_Rana2:
	.dw #0x0014
__xinit__header_Rana2:
	.dw ___str_2
__xinit__width_Rana3:
	.dw #0x0014
__xinit__height_Rana3:
	.dw #0x0014
__xinit__header_Rana3:
	.dw ___str_3
__xinit__width_Rana4:
	.dw #0x0014
__xinit__height_Rana4:
	.dw #0x0014
__xinit__header_Rana4:
	.dw ___str_4
	.area _CABS (ABS)

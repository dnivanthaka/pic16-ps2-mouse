;/opt/microchip/mplabx/v4.10/mpasmx/templates/Object
;20Mhz crystal = 20/4 = 5Mhz
;1Mhz = 1us (One instruction)
;5Mhz = 0.2us
;**********************************************************************
;   PS/2 Mouse interface                                              *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Filename:      main.asm                                           *
;    Date:                                                            *
;    File Version:                                                    *
;                                                                     *
;    Author:                                                          *
;    Company:                                                         *
;                                                                     * 
;                                                                     *
;**********************************************************************
;                                                                     *
;    Files required: P16F876A.INC                                     *
;                                                                     *
;                                                                     *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes:                                                           *
;                                                                     *
;                                                                     *
;                                                                     *
;                                                                     *
;**********************************************************************

    list        p=16f876a   ; list directive to define processor
    #include    <p16f876a.inc>  ; processor specific variable definitions
    
    __CONFIG _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _HS_OSC & _WRT_OFF & _LVP_OFF & _CPD_OFF

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The labels following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
    
		; PS2 Ports and Pins
#define		PS2_CLK_PORT		PORTB			; Clock line port, config, and pin
#define		PS2_CLK_TRIS		TRISB
#define		PS2_CLK_PIN		2
#define		PS2_DAT_PORT		PORTB			; Data line port, config, and pin
#define		PS2_DAT_TRIS		TRISB
#define		PS2_DAT_PIN		1
#define		RS232_RTS_PORT		PORTB	
#define		RS232_RTS_TRIS		TRISB
#define         RS232_RTS_PIN           0
#define         PS2_CMD_RESET           0xFF
#define         PS2_CMD_RESEND          0xFE
#define         PS2_CMD_DEFAULT         0xF6		
#define		PS2_CMD_EN_DAT_RPT	0xF4		; Enable Data Reporting
#define		PS2_CMD_DIS_DAT_RPT	0xF5		; Disable Data Reporting
#define		PS2_CMD_SET_RATE	0xF3		
#define		PS2_CMD_GET_ID	        0xF2
#define		PS2_CMD_SET_DAT_REMOTE	0xF0
#define		PS2_CMD_SET_WRAP_MODE	0xEE
#define		PS2_CMD_RESET_WRAP_MODE	0xEC
#define		PS2_CMD_READ_DATA	0xEB
#define		PS2_CMD_SET_STREAM_MODE	0xEA
#define		PS2_CMD_STATUS_REQ	0xE9
#define		PS2_CMD_SET_RESOLUTION	0xE8
#define		PS2_CMD_SET_SCALE_2_1	0xE7
#define		PS2_CMD_SET_SCALE_1_1	0xE6

		
PS2_CLK_HI MACRO    
            banksel PS2_CLK_TRIS
            bsf     PS2_CLK_TRIS, PS2_CLK_PIN
	    ENDM
		    
PS2_CLK_LO MACRO
            banksel PS2_CLK_TRIS
            bcf     PS2_CLK_TRIS, PS2_CLK_PIN
	    banksel PS2_CLK_PORT
	    bcf     PS2_CLK_PORT, PS2_CLK_PIN
	    ENDM

;PS2_DAT_HI MACRO
;	    banksel PS2_DAT_TRIS
;           bsf     PS2_DAT_TRIS, PS2_DAT_PIN
;	    ENDM
	    
PS2_DAT_HI MACRO
	    banksel PS2_DAT_TRIS
            bcf     PS2_DAT_TRIS, PS2_DAT_PIN
	    banksel PS2_DAT_PORT
	    bsf     PS2_DAT_PORT, PS2_DAT_PIN
	    ENDM
		    
PS2_DAT_LO MACRO
            banksel PS2_DAT_TRIS
            bcf     PS2_DAT_TRIS, PS2_DAT_PIN
	    banksel PS2_DAT_PORT
	    bcf     PS2_DAT_PORT, PS2_DAT_PIN
	    ENDM
	    
PS2_WAIT_CLK_HI MACRO
	    ;banksel PS2_CLK_TRIS
            ;bsf     PS2_CLK_TRIS, PS2_CLK_PIN
	    banksel PS2_CLK_PORT
	    btfsc   PS2_CLK_PORT, PS2_CLK_PIN
	    goto    $-1
	    ENDM
	    
PS2_WAIT_CLK_LO MACRO
	    ;banksel PS2_CLK_TRIS
            ;bsf     PS2_CLK_TRIS, PS2_CLK_PIN
	    banksel PS2_CLK_PORT
	    btfss   PS2_CLK_PORT, PS2_CLK_PIN
	    goto    $-1
	    ENDM
	    
PS2_WAIT_DAT_HI MACRO
	    ;banksel PS2_CLK_TRIS
            ;bsf     PS2_CLK_TRIS, PS2_CLK_PIN
	    banksel PS2_DAT_PORT
	    btfsc   PS2_DAT_PORT, PS2_DAT_PIN
	    goto    $-1
	    ENDM
	    
PS2_WAIT_DAT_LO MACRO
	    ;banksel PS2_CLK_TRIS
            ;bsf     PS2_CLK_TRIS, PS2_CLK_PIN
	    banksel PS2_DAT_PORT
	    btfss   PS2_DAT_PORT, PS2_DAT_PIN
	    goto    $-1
	    ENDM
	    
PS2_ENABLE_COMM MACRO
 	    banksel PS2_CLK_TRIS
	    bsf     PS2_CLK_TRIS, PS2_CLK_PIN
	    
	    ;banksel PS2_DAT_TRIS
	    ;bsf     PS2_DAT_TRIS, PS2_DAT_PIN
	    ENDM
	    
PS2_DISABLE_COMM MACRO
 	    banksel PS2_CLK_PORT
	    bcf     PS2_CLK_PORT, PS2_CLK_PIN
	    
	    banksel PS2_CLK_TRIS
	    bcf     PS2_CLK_TRIS, PS2_CLK_PIN
	    ENDM
	    
DISABLE_INT MACRO
	    banksel INTCON
	    bcf     INTCON, GIE
	    ENDM
	    
ENABLE_INT MACRO
	    banksel INTCON
	    bsf     INTCON, GIE
	    ENDM
	    
		
;***** VARIABLE DEFINITIONS (examples)
	    
cblock 0x20 
time_scaler
parity
tdata
counter
temp
temp_2
temp_3
ms_btn
ms_x
ms_y
ms_ps2_initialized
ms_serial_initialized	    
endc

; example of using Shared Uninitialized Data Section
INT_VAR     UDATA_SHR
ms_x_inc       RES   2
ms_y_inc       RES   2
ms_send_bytes  RES   3


; example of using Uninitialized Data Section
TEMP_VAR    UDATA           ; explicit address specified is not required




; example of using Overlayed Uninitialized Data Section
; in this example both variables are assigned the same GPR location by linker
G_DATA      UDATA_OVR       ; explicit address can be specified


;**********************************************************************
RESET_VECTOR    CODE    0x0000  ; processor reset vector
    nop                         ; nop for icd
    pagesel start
    goto    start               ; go to beginning of program

INT_VECTOR      CODE    0x0004  ; interrupt vector location

INTERRUPT
      ; Check what triggered the interrupt
      banksel INTCON
      btfsc INTCON, INTF
      goto  _pin0_int
      goto  _int_done
      
_pin0_int
      banksel OPTION_REG
      btfsc     OPTION_REG, INTEDG  ;check the type, if falling edge int will fire, rts is pulled high
      goto _int_rts_stop
      
      banksel OPTION_REG
      bsf     OPTION_REG, INTEDG
      
      btfsc   ms_serial_initialized, 0
      goto    _int_done
      
      pagesel rs232_probe
      call    rs232_probe
      goto    _int_done
      
_int_rts_stop
      bcf ms_serial_initialized, 0
      banksel OPTION_REG
      bcf     OPTION_REG, INTEDG
      
_int_done
    banksel INTCON
    bcf     INTCON, INTF
    retfie              ; return from interrupt

MAIN_PROG       CODE

start
    pagesel device_init
    call    device_init
    
    ;movlw   .100
    ;pagesel delay_ms
    ;call    delay_ms
    ;call    delay_ms                   ; wait for device to settle
    
;    banksel PORTB
;    bcf     PORTB, RB2
    
    clrf ms_ps2_initialized
    clrf ms_serial_initialized
    
    pagesel  ms_init
    call     ms_init
    
    ;movlw PS2_CMD_DIS_DAT_RPT
    ;pagesel ms_write
    ;call    ms_write
    
    ;pagesel ms_read
    ;call    ms_read
    
    ;movlw PS2_CMD_EN_DAT_RPT
    movlw 0xF0
    pagesel ms_write
    call    ms_write
    
    pagesel ms_read
    call    ms_read
    
    PS2_DISABLE_COMM
    
    movlw .100
    pagesel delay_us
    call    delay_us
    
    btfss   ms_serial_initialized, 0
    goto    $-1
    
;    banksel PORTB
;    btfsc   PORTB, RB2
;    goto    $-1
    
;_probe_loop
;    pagesel  rs232_probe
;    call     rs232_probe
;    
;    banksel PORTB
;    btfss   PORTB, RB2
;    goto    _probe_loop
    
    movlw .100
    pagesel delay_ms
    call    delay_ms
    
    
;    movlw .200
;    pagesel delay_us
;    call    delay_us
    
        
;    movlw   0x0D
;    pagesel TXPoll
;    call    TXPoll
;    
;    movlw   0x0A
;    pagesel TXPoll
;    call    TXPoll
    
;    movlw 0xEB
;    pagesel ms_write
;    call    ms_write
;    
;    pagesel ms_read
;    call    ms_read
    
    ;pagesel uart_print_hex
    ;call    uart_print_hex
    
;    pagesel ms_read
;    call    ms_read
    
;    pagesel uart_print_hex
;    call    uart_print_hex
    
    ;movlw 0xEB
    ;pagesel ms_write
    ;call    ms_write
    
;    pagesel ms_read
;    call    ms_read
    
;    pagesel ms_read
;    call    ms_read
    
;    pagesel uart_print_hex
;    call    uart_print_hex
;    
;    pagesel ms_read
;    call    ms_read
;;    
;    pagesel uart_print_hex
;    call    uart_print_hex
    
    ;movf    ms_btn, w
    ;pagesel uart_print_hex
    ;call    uart_print_hex
    
;    movlw PS2_CMD_EN_DAT_RPT
;    pagesel ms_write
;    call    ms_write
;    
;    pagesel ms_read
;    call    ms_read
    
;    PS2_DISABLE_COMM
;    
;    movlw .100
;    pagesel delay_us
;    call    delay_us
    
    
read_loop   
    clrf ms_x_inc
    clrf ms_y_inc
    
    banksel PORTB
    bcf     PORTB, RB2
    
    movlw 0xEB
    pagesel ms_write
    call    ms_write
    
    pagesel ms_read
    call    ms_read
    
    ;pagesel ms_read_frame
    ;call    ms_read_frame
    
    pagesel ms_read
    call    ms_read
    movwf   ms_btn
    
    pagesel ms_read
    call    ms_read
    movwf   ms_x
    
    pagesel ms_read
    call    ms_read
    movwf   ms_y
    
;    banksel PORTB
;    bsf     PORTB, RB2
    
    ;movf    ms_btn, w
    ;pagesel uart_print_hex
    ;call    uart_print_hex
    
;    movlw ','
;    pagesel TXPoll
;    call    TXPoll
;    
;    movlw 'X'
;    pagesel TXPoll
;    call    TXPoll
;    
;    movlw '='
;    pagesel TXPoll
;    call    TXPoll
;    
;    movf  ms_x, w
;    movwf ms_x_inc
;    
;    btfsc ms_btn, 4
;    goto  neg_ms_x
;    goto  ms_x_done
    
;neg_ms_x:
;    movlw 0xff
;    movwf ms_x_inc + 1
;    movlw '-'
;    pagesel TXPoll
;    call    TXPoll
;    
;ms_x_done:
;    movf    ms_x_inc, w
;    pagesel uart_print_hex
;    call    uart_print_hex
;    
;    movlw ','
;    pagesel TXPoll
;    call    TXPoll
;    
;    movlw 'Y'
;    pagesel TXPoll
;    call    TXPoll
;    
;    movlw '='
;    pagesel TXPoll
;    call    TXPoll
;    
;    movf  ms_y, w
;    movwf ms_y_inc
;    
;    btfsc ms_btn, 5
;    goto  neg_ms_y
;    goto  ms_y_done
;    
;neg_ms_y:
;    movlw 0xff
;    movwf ms_y_inc + 1
;    movlw '-'
;    pagesel TXPoll
;    call    TXPoll
;    
;ms_y_done:
;    movf    ms_y_inc, w
;    pagesel uart_print_hex
;    call    uart_print_hex
    
    movf    ms_x, w
    movwf   ms_x_inc
    
    bcf STATUS, C
    btfsc ms_btn, 4
    bsf STATUS, C
    rrf ms_x_inc, f
    
    movf    ms_y, w
    movwf   ms_y_inc
    
    bcf STATUS, C
    btfsc ms_btn, 5
    bsf STATUS, C
    rrf ms_y_inc, f
    
    ;------------------------------------------------
    clrf    ms_send_bytes
    clrf    ms_send_bytes + 1
    clrf    ms_send_bytes + 2
    
    movlw   b'11000000'
    movwf   ms_send_bytes
    btfsc   ms_btn, 0                ; Left button
    bsf     ms_send_bytes, 5
    btfsc   ms_btn, 1                ; Right button
    bsf     ms_send_bytes, 4
    btfsc   ms_y_inc, 7
    bsf     ms_send_bytes, 3
    btfsc   ms_y_inc, 6
    bsf     ms_send_bytes, 2
    btfsc   ms_x_inc, 7
    bsf     ms_send_bytes, 1
    btfsc   ms_x_inc, 6
    bsf     ms_send_bytes, 0
    
    movlw   0x3f
    andwf   ms_x_inc, w
    movwf   ms_send_bytes + 1
    
    movlw   0x3f
    andwf   ms_y_inc, w
    movwf   ms_send_bytes + 2
    
    ; Send the bytes, should convert into 7N2 format
    movf ms_send_bytes, w
    pagesel TXPoll
    call    TXPoll
    
    movf ms_send_bytes + 1, w
    pagesel TXPoll
    call    TXPoll
    
    movf ms_send_bytes + 2, w
    pagesel TXPoll
    call    TXPoll
    ;------------------------------------------------
    
;    movlw   0x0D
;    pagesel TXPoll
;    call    TXPoll
;    
;    movlw   0x0A
;    pagesel TXPoll
;    call    TXPoll
    
    
    goto read_loop
    
    goto $
    
;------------------------------------------------------------------------------   
ms_init
    movlw   .200
    pagesel delay_ms
    call    delay_ms
    
    DISABLE_INT
    
    movlw   0xff
    pagesel ms_write
    call    ms_write
    
    pagesel ms_read
    call    ms_read
    
    pagesel ms_read
    call    ms_read
    
    pagesel ms_read
    call    ms_read
    
;    pagesel uart_print_hex
;    call    uart_print_hex
    
    movlw   PS2_CMD_SET_RATE
    pagesel ms_write
    call    ms_write
    
    pagesel ms_read
    call    ms_read
    
    movlw   0x64                        ;100
    pagesel ms_write
    call    ms_write
    
    pagesel ms_read
    call    ms_read
    
    movlw   PS2_CMD_SET_RESOLUTION                       
    pagesel ms_write
    call    ms_write
    
    pagesel ms_read
    call    ms_read
    
    movlw   0x02
    pagesel ms_write
    call    ms_write
    
    pagesel ms_read
    call    ms_read
    
    movlw   0xF2
    pagesel ms_write
    call    ms_write
    
    pagesel ms_read
    call    ms_read
    
    pagesel ms_read
    call    ms_read
    
    movlw   0xF6
    pagesel ms_write
    call    ms_write
    
    pagesel ms_read
    call    ms_read
    
    ENABLE_INT
    
    movlw   0x01
    movwf   ms_ps2_initialized
    
    return
    
rs232_probe
    movlw .14
    pagesel delay_ms
    call    delay_ms
    
    movlw  'M'
    pagesel TXPoll
    call    TXPoll
    
    movlw .100
    pagesel delay_ms
    call    delay_ms
    
    bsf  ms_serial_initialized, 0
    
    return
    
    
ms_write
    movwf   tdata
    movlw   0x01
    movwf   parity
    
    DISABLE_INT
    
    movlw   .8
    movwf   counter
    
    ;PS2_ENABLE_COMM
    banksel PS2_CLK_TRIS
    bsf     PS2_CLK_TRIS, PS2_CLK_PIN

    banksel PS2_DAT_TRIS
    bsf     PS2_DAT_TRIS, PS2_DAT_PIN
    
    movlw   .200
    pagesel delay_us
    call    delay_us
    
    PS2_CLK_LO
    
    movlw   .200
    pagesel delay_us
    call    delay_us 
    
    PS2_DAT_LO
    
    movlw   .5
    pagesel delay_us
    call    delay_us
    
    ;movlw   .10
    ;pagesel delay_us
    ;call    delay_us
    
    banksel PS2_CLK_PORT
    bsf     PS2_CLK_PORT, PS2_CLK_PIN 
    
    PS2_CLK_HI
    
    PS2_WAIT_CLK_HI              ; discard start clock pulse
    
w_tmp
    
    rrf   tdata, f
    banksel PS2_DAT_PORT
    btfss   STATUS, C
    goto    $+2
    goto    $+3
    bcf   PS2_DAT_PORT, PS2_DAT_PIN
    goto    w_done
    bsf   PS2_DAT_PORT, PS2_DAT_PIN
    movlw 0x1
    xorwf parity, f
w_done    
    
    PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    
    nop
    nop
    nop

    decfsz counter, f
    goto   w_tmp
    
    ; Send parity bit
    
    rrf  parity, f
    banksel PS2_DAT_PORT
    btfss   STATUS, C
    goto    $+2
    goto    $+3
    bcf   PS2_DAT_PORT, PS2_DAT_PIN
    goto    wp_done
    bsf   PS2_DAT_PORT, PS2_DAT_PIN
    
wp_done   
    
    PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    
    banksel PS2_DAT_TRIS
    bsf     PS2_DAT_TRIS, PS2_DAT_PIN
    
    ;stop bit
    
    PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    
    ;Wait for acknowledgemet by device
    PS2_WAIT_DAT_LO
    PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    PS2_WAIT_DAT_HI
    
    ENABLE_INT
    
    return
    
;--------------------------------------------------
; UART
;--------------------------------------------------
    
;----------------------------
RXPoll
	banksel	PIR1
	btfss	PIR1, RCIF
	;return
	goto	RXPoll
	; Reading value into w
	banksel	RCREG
	movf	RCREG, w

	return
;-----------------------------
TXPoll
	banksel	PIR1
	btfss	PIR1, TXIF
	goto	TXPoll
	; Writing value in w
	banksel	TXREG
	movwf	TXREG

	return
;-----------------------------
hex2ascii
	movwf	temp
	btfss	temp, 3
	goto	ztn
	xorlw	0x08
	btfsc	STATUS, Z
	goto	ztn
	nop
	movf	temp, w
	xorlw	0x09
	btfsc	STATUS, Z
	goto	ztn
	nop
	movlw	0x07
	addwf	temp, f
	
ztn
	movf	temp, w
	addlw	0x30
	call	TXPoll
	return
	
uart_print_hex
	movwf temp_2
	
	swapf   temp_2, w
	andlw 0x0f
	pagesel hex2ascii
	call    hex2ascii
	movf  temp_2, w
	andlw 0x0f
	pagesel hex2ascii
	call    hex2ascii
	
	return
	
hex2dec
	;movwf	temp
	btfss	temp, 3
	retlw	0x30
	movlw	0x06
	andwf	temp, w
	btfsc	STATUS, Z
	retlw	0x30
	retlw	0x41	

	return
;-----------------------------    
ms_read
    clrf    tdata
    clrf    parity
    
    ;banksel PS2_DAT_TRIS
    ;bsf     PS2_DAT_TRIS, PS2_DAT_PIN
    
    DISABLE_INT
    
    movlw   .8
    movwf   counter
    
    
    PS2_WAIT_CLK_HI              ; discard start clock pulse

    PS2_WAIT_CLK_LO
    ;movlw   .5
    ;pagesel delay_us
    ;call    delay_us
    ;PS2_WAIT_CLK_HI
    
r_bits
    ;PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    
    ;movlw   .5
    ;pagesel delay_us
    ;call    delay_us
    
    banksel PS2_DAT_PORT
    btfss   PS2_DAT_PORT, PS2_DAT_PIN
    goto    $+3
    bsf     STATUS, C
    goto    r_bit_done
    bcf     STATUS, C
    
r_bit_done
    rrf     tdata, f
    
    ;PS2_WAIT_CLK_HI
    PS2_WAIT_CLK_LO
    
    decfsz counter, f
    goto   r_bits
    
    ;PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    
    ;movlw   .5
    ;pagesel delay_us
    ;call    delay_us
    
    banksel PS2_DAT_PORT
    btfss   PS2_DAT_PORT, PS2_DAT_PIN
    goto    $+3
    bsf     parity, 0
    goto    r_parity_done
    bcf     parity, 0
    
r_parity_done
    PS2_WAIT_CLK_LO
    
    ; stop bit
    
    PS2_WAIT_CLK_HI
    
    ;banksel PS2_DAT_PORT
    ;btfsc   PS2_DAT_PORT, PS2_DAT_PIN
    ;goto    $-1
    
    PS2_WAIT_CLK_LO
    
    ENABLE_INT
    
    movf    tdata, w
    
    return
    
ms_read_frame
    PS2_ENABLE_COMM
    
    ;movlw   .50
    ;pagesel delay_us
    ;call    delay_us
    
    pagesel ms_read
    call    ms_read
    movwf   ms_btn
    
    pagesel ms_read
    call    ms_read
    movwf   ms_x
    
    pagesel ms_read
    call    ms_read
    movwf   ms_y
    
    PS2_DISABLE_COMM
    
    movlw   .100
    pagesel delay_us
    call    delay_us
    
    return
    
ms_wzero
    
    return
    
ms_wone
    
    return
;------------------------------------------------------------------------------
    
device_init
    banksel ADCON0
    bcf ADCON0, ADON            ;Disable ADC
    
    movlw b'10000111'              ; Setting OPTION_REG, Pullup disable PSA = 1:256
    ;movlw b'00000111'              ; Setting OPTION_REG, Pullup enable PSA = 1:256
    banksel OPTION_REG
    movwf   OPTION_REG
    
    ;banksel OPTION_REG
    ;bcf     OPTION_REG, 7   ; Enable portb pull ups
    
    banksel PS2_CLK_TRIS
    bsf     PS2_CLK_TRIS, PS2_CLK_PIN
    
    banksel PS2_DAT_TRIS
    bsf     PS2_DAT_TRIS, PS2_DAT_PIN
    
    ;UART
    banksel	TRISC
    bsf		TRISC, RC7
    bcf		TRISC, RC6
    
    banksel RCREG
    clrf    RCREG
    
    banksel TXREG
    clrf    TXREG
    
    ;movlw   0x81           ;9600 bps
    movlw   0xff              ;1200
    banksel SPBRG
    movwf   SPBRG
    
    ;movlw   0x24
    movlw   0x20             ;1200
    banksel TXSTA
    movwf   TXSTA
    
    movlw   0x90
    banksel RCSTA
    movwf   RCSTA
    ;---------------------------------------
    
    ;banksel PORTB
    ;bsf     PORTB, RB0  
    ;Testing
    banksel TRISB
    bsf     TRISB, RB2
;    
;    banksel PORTB
;    bcf     PORTB, RB2
    ; Enabling pin 0 interrupt
    
    banksel RS232_RTS_TRIS
    bsf     RS232_RTS_TRIS, RS232_RTS_PIN
    
    banksel INTCON
    bsf     INTCON, INTE
    banksel OPTION_REG
    bcf     OPTION_REG, INTEDG  ;falling edge trigger
    
    ;banksel INTCON
    ;bsf  INTCON, PEIE
    ;bsf     INTCON, GIE
    
    ENABLE_INT
    
    return
    
;------------------------------------------------------------------------------
    
delay_ms
    movwf time_scaler
    
w_inner    
    banksel TMR0
    clrf    TMR0
    
w_tmr0
    movf   TMR0, w
    xorlw  .200                  ; 0.2 * 256 = 51.2us, 51.2 * 200 = ~1ms
    btfss  STATUS, Z
    goto   w_tmr0
    
    decfsz time_scaler, f
    goto   w_inner
    
    return
    
delay_us
    movwf time_scaler
    
us_loop
    nop
    nop
    nop
    ;nop
    ;nop
    decfsz  time_scaler, f
    goto us_loop
    
    return
    
;------------------------------------------------------------------------------
    

    END                       ; directive 'end of program'

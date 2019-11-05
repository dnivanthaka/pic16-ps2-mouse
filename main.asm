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
#define		PS2_CLK_PIN		1
#define		PS2_DAT_PORT		PORTB			; Data line port, config, and pin
#define		PS2_DAT_TRIS		TRISB
#define		PS2_DAT_PIN		0
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
	    
		
;***** VARIABLE DEFINITIONS (examples)
	    
cblock 0x20 
time_scaler
parity
tdata
counter
temp
ms_btn
ms_x
ms_y	    
endc

; example of using Shared Uninitialized Data Section
INT_VAR     UDATA_SHR      


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

    retfie              ; return from interrupt

MAIN_PROG       CODE

start
    pagesel device_init
    call    device_init
    
    movlw   .100
    pagesel delay_ms
    call    delay_ms
    ;call    delay_ms                   ; wait for device to settle

loop
    banksel PORTB
    bcf     PORTB, RB2
    
    pagesel  ms_init
    call     ms_init
    
    
    ;PS2_CLK_LO
    movlw   0xff
    pagesel ms_write
    call    ms_write
    
    ;movlw .20
    ;pagesel delay_us
    ;call    delay_us
    
    ;movlw    0x99
    ;pagesel  uart_print_hex
    ;call uart_print_hex
    
    pagesel ms_read
    call    ms_read
    
    ;movlw    0x99
    ;pagesel  uart_print_hex
    ;call uart_print_hex
    
    ;PS2_DISABLE_COMM
    
    ;movlw .200
    ;pagesel delay_us
    ;call    delay_us
    ;pagesel TXPoll
    ;call    TXPoll
    
    ;PS2_ENABLE_COMM
    
    pagesel ms_read
    call    ms_read
    
    pagesel ms_read
    call    ms_read
    
    
    ;PS2_ENABLE_COMM
    
    movlw .200
    pagesel delay_us
    call    delay_us
    
    movlw PS2_CMD_EN_DAT_RPT
    pagesel ms_write
    call    ms_write
    
    ;movlw    0x99
    ;pagesel  uart_print_hex
    ;call uart_print_hex
    
    pagesel ms_read
    call    ms_read
    
    movlw .100
    pagesel delay_us
    call    delay_us
    
    ;movlw    0x99
    ;pagesel  uart_print_hex
    ;call uart_print_hex
    
    
read_loop    
    banksel PORTB
    bcf     PORTB, RB2
    
    ;pagesel ms_read
    ;call    ms_read
    
    ;pagesel ms_read
    ;call    ms_read
    
    ;pagesel ms_read
    ;call    ms_read
    
    pagesel ms_read_frame
    call    ms_read_frame
    
    ;pagesel  uart_print_hex
    ;call uart_print_hex
    
    ;PS2_ENABLE_COMM
    
    ;PS2_WAIT_CLK_HI 
    
    ;pagesel ms_read
    ;call    ms_read
    
    ;pagesel ms_read
    ;call    ms_read
    
    ;pagesel ms_read
    ;call    ms_read
    
    ;movlw    0xff
    ;pagesel  ms_write
    ;call     ms_write
    
    banksel PORTB
    bsf     PORTB, RB2
    
    movlw   .100
    pagesel delay_ms
    call    delay_ms 
    
    goto read_loop
    
    goto $
    
;------------------------------------------------------------------------------    

ms_init
    movlw   .200
    pagesel delay_ms
    call    delay_ms
    
    return
    
    
ms_write
    movwf   tdata
    movlw   0x01
    movwf   parity
    
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
    
    ;PS2_WAIT_CLK_LO
    
    PS2_WAIT_CLK_HI              ; discard start clock pulse
    
    ;PS2_WAIT_CLK_LO
    
w_tmp
    ;PS2_WAIT_CLK_LO
    ;PS2_WAIT_CLK_HI
    
    rrf   tdata, f
    banksel PS2_DAT_PORT
    ;banksel STATUS
    ;movf    C, 0
    ;xorwf   parity, f
    btfss   STATUS, C
    goto    $+2
    goto    $+3
    ;PS2_DAT_LO
    ;banksel PS2_DAT_PORT
    bcf   PS2_DAT_PORT, PS2_DAT_PIN
    ;movlw 0x0
    ;xorwf parity, f
    goto    w_done
    ;PS2_DAT_HI
    ;banksel PS2_DAT_PORT
    bsf   PS2_DAT_PORT, PS2_DAT_PIN
    movlw 0x1
    xorwf parity, f
w_done    
    
    ;PS2_WAIT_CLK_LO
    
    ;PS2_DAT_HI
    
    PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    ;PS2_WAIT_CLK_LO
    
    nop
    nop
    nop

    decfsz counter, f
    goto   w_tmp
    
    ;PS2_WAIT_CLK_LO    ; Send parity bit
    ;PS2_WAIT_CLK_HI    ; Send parity bit
    
    rrf  parity, f
    banksel PS2_DAT_PORT
    ;banksel STATUS
    btfss   STATUS, C
    goto    $+2
    goto    $+3
    bcf   PS2_DAT_PORT, PS2_DAT_PIN
    goto    wp_done
    bsf   PS2_DAT_PORT, PS2_DAT_PIN
    ;PS2_DAT_HI
    
wp_done   
    
    PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    ;PS2_WAIT_CLK_LO
    
    
    banksel PS2_DAT_TRIS
    bsf     PS2_DAT_TRIS, PS2_DAT_PIN
    
    ;stop bit
    ;PS2_WAIT_CLK_LO
    ;PS2_WAIT_CLK_HI

    
    ;banksel PS2_DAT_PORT
    ;btfsc   PS2_DAT_PORT, PS2_DAT_PIN
    ;goto    $-1
    
    PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    ;PS2_WAIT_CLK_LO
    
    ;Wait for acknowledgemet by device
    PS2_WAIT_DAT_LO
    PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    PS2_WAIT_DAT_HI
    
    ;PS2_DISABLE_COMM
    
    ;movlw   .60
    ;pagesel delay_ms
    ;call    delay_ms
    
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
	movwf temp
	
	swapf   temp, 0
	andlw   0x0f
	addlw   '0'
	
	
	pagesel hex2ascii
	call    hex2ascii
	
	movlw  0x0f
	andwf  temp, 0
	addlw  '0'
	
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
    
    movlw   .8
    movwf   counter
    
    
    PS2_WAIT_CLK_HI              ; discard start clock pulse
    
    PS2_WAIT_CLK_LO
    ;movlw   .5
    ;pagesel delay_us
    ;call    delay_us
    
r_bits
    ;PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    
    movlw   .2
    pagesel delay_us
    call    delay_us
    
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
    
    movlw   .2
    pagesel delay_us
    call    delay_us
    
    banksel PS2_DAT_PORT
    btfss   PS2_DAT_PORT, PS2_DAT_PIN
    goto    $+3
    bsf     parity, 0
    goto    r_parity_done
    bcf     parity, 0
    
    ;PS2_WAIT_CLK_HI
    PS2_WAIT_CLK_LO
    
r_parity_done
    ; stop bit
    ;PS2_WAIT_CLK_LO
    PS2_WAIT_CLK_HI
    
    ;banksel PS2_DAT_PORT
    ;btfsc   PS2_DAT_PORT, PS2_DAT_PIN
    ;goto    $-1
    
    ;PS2_WAIT_CLK_HI
    PS2_WAIT_CLK_LO
    
;    banksel PS2_CLK_TRIS
;    bcf     PS2_CLK_TRIS, PS2_CLK_PIN
;    
;    banksel PS2_CLK_PORT
;    bcf     PS2_CLK_PORT, PS2_CLK_PIN
    
    ;movlw   .60
    ;pagesel delay_ms
    ;call    delay_ms
    
    ;PS2_DISABLE_COMM
    
    movf    tdata, w
    
    return
    
ms_read_frame
    PS2_ENABLE_COMM
    
    pagesel ms_read
    call    ms_read
    
    pagesel ms_read
    call    ms_read
    
    pagesel ms_read
    call    ms_read
    
    PS2_DISABLE_COMM
    
    return
    
ms_wzero
    
    return
    
ms_wone
    
    return
;------------------------------------------------------------------------------
    
device_init
    banksel ADCON0
    bcf ADCON0, ADON            ;Disable ADC
    
    movlw b'10000111'              ; Setting OPTION_REG, Pullup enable PSA = 1:256
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
    
    movlw   0x81
    banksel SPBRG
    movwf   SPBRG
    
    movlw   0x24
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
    bcf     TRISB, RB2
    
    banksel PORTB
    bcf     PORTB, RB2
    
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
    nop
    nop
    decfsz  time_scaler, f
    goto us_loop
    
    return
    
;------------------------------------------------------------------------------
    

    END                       ; directive 'end of program'

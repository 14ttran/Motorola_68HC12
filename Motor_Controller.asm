;**************************************************************************************
;* Motor Controller code                                                             *
;**************************************************************************************
;* Summary:                                                                           *
;*  This code is designed for use with the Motorola 68HC12 microcontroller. This      *
;*  program controls a motor and interface to input and view motor conditions         *
;**************************************************************************************

;/------------------------------------------------------------------------------------\
;| Include all associated files                                                       |
;\------------------------------------------------------------------------------------/
; The following are external files to be included during assembly


;/------------------------------------------------------------------------------------\
;| External Definitions                                                               |
;\------------------------------------------------------------------------------------/
; All labels that are referenced by the linker need an external definition

              XDEF  main

;/------------------------------------------------------------------------------------\
;| External References                                                                |
;\------------------------------------------------------------------------------------/
; All labels from other files must have an external reference

              XREF  ENABLE_MOTOR, DISABLE_MOTOR
              XREF  STARTUP_MOTOR, UPDATE_MOTOR, CURRENT_MOTOR
              XREF  STARTUP_PWM, STARTUP_ATD0, STARTUP_ATD1
              XREF  OUTDACA, OUTDACB
              XREF  STARTUP_ENCODER, READ_ENCODER
              XREF  INITLCD, SETADDR, GETADDR, CURSOR_ON, CURSOR_OFF, DISP_OFF
              XREF  OUTCHAR, OUTCHAR_AT, OUTSTRING, OUTSTRING_AT
              XREF  INITKEY, LKEY_FLG, GETCHAR
              XREF  LCDTEMPLATE, UPDATELCD_L1, UPDATELCD_L2
              XREF  LVREF_BUF, LVACT_BUF, LERR_BUF,LEFF_BUF, LKP_BUF, LKI_BUF
              XREF  Entry, ISR_KEYPAD
            
;/------------------------------------------------------------------------------------\
;| Assembler Equates                                                                  |
;\------------------------------------------------------------------------------------/
; Constant values can be equated here

TIOS     EQU $0040
TEN      EQU $0046
TCR2     EQU $0049   ;Timer Control Register 2
TMR1     EQU $004C   ;Timer Mask Register 1
TFR1     EQU $004E   ;Timer Flag Register 1
Chan0    EQU $01

TCNTH    EQU $0044   ;Timer count register HIGH
TCNTL    EQU $0045   ;Timer count register LOW

TC0H     EQU $0050
TC0L     EQU $0051


;/------------------------------------------------------------------------------------\
;| Variables in RAM                                                                   |
;\------------------------------------------------------------------------------------/
; The following variables are located in unpaged ram

DEFAULT_RAM:  SECTION

;-------------------------------------ITCV------------------------------------; 
RUN        DS.B 1    ; TASK: timer, master
KEY_FLG    DS.B 1    ; key waiting TASK: master, keypad
ECHO       DS.B 1    ;define ECHO in RAM 1 byte TASK: master, LCD
BSPACE     DS.B 1    ;define backspace flag in RAM 1 byte TASK: master, LCD
DGAIN      DS.B 1    ;Display Prompt flag  TASK: master, LCD
CLR_TIME   DS.B 1    ;variable to indicate that the time on display needs to clear
RUN_MOTOR  DS.B 1    ;flag to indicate the motor = on
UPDATE_LCD DS.B 1    ;flag to indicate time to update lcd
VREF_FLG   DS.B 1    ;flag to indicate vref is chosen variable
DPROMPT    DS.B 1    ;flag to indicate prompts need to display
ENTRY_FLG  DS.B 1    ;flag to indicate and entry is occurring
LET_BUF    DS.B 1    ;buffer for the letter keypress
CURSOR_LOC DS.B 1    ;buffer for the current location of cursor
TEMPNUM_W  DS.W 1    ;temporary variable for BtBtA
DIGITREM   DS.B 1    ;counter for BtBtA
DIVISOR_W  DS.W 1    ;buffer for divisor in BtBtA
VREF_DISP  DS.B 1    ;flag to indicate vref prompt display
VREF_SIGN  DS.B 1    ;flag to indicate the sign of vref 0=+, 1=-
FKEY_FLG   DS.B 1    ;flag to indicate function keypress
CURRENT_SIGN DS.B 1  ;flag to indicate the current vector direction
SIGN_BUF   DS.B 1    ;buffer to store the current vector direction
DSYSTEM    DS.B 1    ;flag to indicate system parameters to display
MSGPTR_W   DS.W 1    ;pointer for system parameter messages
SYS_CURSOR DS.B 1    ;buffer for location to display certain messages
CURSOR_BUF DS.B 1    ;buffer for cursor location and immediate return
GAIN_FLG   DS.B 1    ;flag to indicate gain changes
LOOP_TYPE  DS.B 1    ;flag to indicate control loop type 0 is open, 1 is closed
CONV_COUNT DS.B 1    ;counter for display update variables
SETUP_DONE DS.B 1    ;flag to indicate display update variables can be converted



;-------------------------------------TASK 1 MM----------------------------------;
DONE_CONV DS.B 1    ;Completed ASCII to BCD Conversion
COUNT     DS.B   1  ;define count in RAM 1 byte
MAXCOUNT  DS.B   1
POINTER   DS.W   1  ;define pointer in RAM 1 word
BUFFER    DS.B   5  ;define buffer in RAM 3 bytes
RESULT    DS.W   1  ;define result in RAM 1 byte
RESULT_B  DS.B   1
ENTRY_DONE DS.B 1
VREF_BUF_W DS.W 1    ;buffer for vref hex value
KP_BUF_W   DS.W 1    ;buffer for kp hex value
KI_BUF_W   DS.W 1    ;buffer for ki hex value
VACT_BUF_W DS.W 1    ;buffer for vact hex value
ERR_BUF_W  DS.W 1    ;buffer for error hex value
EFF_BUF_W  DS.W 1    ;buffer for effort hex value
INPUTPTR_W DS.W 1    ;pointer for the input variable chosen
OUTPUTPTR_W DS.W 1   ;pointer for the output variable chosen

;-------------------------------LCD variables----------------------------------;
DPTR      DS.W   1  ;Display Pointer, address of the next character to be read
FIRSTCHAR DS.B   1  ;First character of message needs to be be written

;-------------------------------State Variables--------------------------------;
t1state  DS.B 1
t2state  DS.B 1
t3state  DS.B 1

;ISR VARS---------------------------------------------------------------------
ESUM_W           DS.W 1
E_W              DS.W 1
VREF_W           DS.W 1
VACT_W           DS.W 1
KIINPUT_W        DS.W 1
KPINPUT_W        DS.W 1    ;User input, actually equals 1024*Kp
A_W              DS.W 1
APRIME_W         DS.W 1
ASTAR_W          DS.W 1

ENCODER_COUNT_W  DS.W 1

ISR_UPDATE  DS.B 1
ISR_COUNTER DS.B 1

;END ISR VARS------------------------------------------------------------------
;/------------------------------------------------------------------------------------\
;|  Main Program Code                                                                 |
;\------------------------------------------------------------------------------------/
; This code implements Lab_1A for ME305

MyCode:       SECTION
main:   
        clr t1state
        clr t2state
        clr t3state
      
        
Top:    
        jsr TASK_1 ;MM
        jsr TASK_2 ;Keypad
        jsr TASK_3 ;LCD
        
        
        ;DO NOT DELAY!!!
        bra Top

;-------------------------------------------TASK_1 MASTER------------
TASK_1:        

        ldaa  t1state                    ;load accu A with the current state
        lbeq   t1state0                   ;init
        deca                             ;decrement accu A by 1
        lbeq   t1state1                   ;wait for prompt stage
        deca  
        lbeq   t1state2                   ;hub state
        deca
        lbeq  t1state3                   ;digit state
        deca
        lbeq  t1state4                   ;enter key state (ASCIItoBCD)
        deca 
        deca
        lbeq  t1state6                   ;backspace state
        deca
        lbeq  t1state7                   ;bspace waiting state
        deca
        lbeq  t1state8                   ;letter state
        deca
        lbeq  t1state9                   ;function key state
        deca
        lbeq  t1state10                  ;update
        
        rts                              ;return from subroutine

t1state0:                                ;init state
        
        clr   COUNT                      ;clear count
        clrw  RESULT                     ;clear result
        clr   RESULT_B
        clr   DONE_CONV
        clr   CLR_TIME
        clr   MAXCOUNT
        clrw  INPUTPTR_W
        clrw  KP_BUF_W
        clrw  VREF_BUF_W
        clrw  EFF_BUF_W
        clrw  ERR_BUF_W
        clrw  VACT_BUF_W
        clr   VREF_FLG
        clr   UPDATE_LCD
        clr   RUN_MOTOR
        clr   DPROMPT
        clr   ENTRY_FLG
        clr   LET_BUF
        clr   CURSOR_LOC
        clr   KI_BUF_W
        clr   OUTPUTPTR_W
        clr   VREF_DISP
        clr   FKEY_FLG
        clr   VREF_SIGN
        clr   SIGN_BUF
        clr   DSYSTEM
        clr   MSGPTR_W
        clr   SYS_CURSOR
        clr   CURSOR_BUF
        clr   GAIN_FLG
        clr   CONV_COUNT
        clr   ISR_UPDATE
        clr   ENTRY_DONE
        clr   SETUP_DONE
        clrw  VACT_W
        clrw  LKP_BUF
        clrw  LKP_BUF+2
        clr   LKP_BUF+4
        clrw  LKI_BUF
        clrw  LKI_BUF+2
        clr   LKI_BUF+4
        
        ;--------------------ISR init------------------------------------------------;
        jsr STARTUP_PWM
        jsr STARTUP_MOTOR
        jsr ENABLE_MOTOR
        jsr STARTUP_ENCODER
        
        clrw ESUM_W
        clrw VREF_W
        
        ;clr STOP_MOTOR
        ;clr OPEN_LOOP
        
        clr ISR_COUNTER
        clr ISR_UPDATE
        
        bset TIOS, Chan0      ;Enable timer channel 0 timer ourput compare interrupts
        bset TEN,  %10100000  ;Timer Enable and TSBCK Enable
        bset TCR2, %00000001  ;Set Toggel OCn output line every time C0F changes
        bset TFR1, %00000001  ;Clear C0F by writing a 1 to it
        jsr INITKEY           ;This enables maskable interrupts in the most lazily
        bset TMR1, %00000001  ;Enable C0I channel interupt
        ;Generate first interrupt
        clrx                  ;Add current counter value to INTERVAL
        addx TCNTH
        stx INTERVAL_INIT
        movw TC0H, INTERVAL_INIT   ;Load INTERVAL Value into TC0 compare register
        
        ;----------------------END ISR init----------------------------------;
        
        movw  #BUFFER, POINTER           ;put address of BUFFER into contents of POINTER
        movw  #10000, DIVISOR_W
        movb  #$05, DIGITREM
        movb  #$20, CURRENT_SIGN
        movb  #$01, LOOP_TYPE
        movb  #$01, t1state              ;set the next state to state1
        
        rts                              ;return from subroutine
        
t1state1:                                ;waiting for prompts state
        
        ldaa #$00
        tst  DPROMPT                        ;Check for flags to be set to zero
        bne t1s1_exit
        
        movb  #$02, t1state              ;set the next state to state 2 after pass of Top
        
        rts                              ;return from subroutine
        
t1s1_exit:

        rts

t1state2:                                ;hub state
        
        ;-----------------------code to implement display update---------------------;
        tst   UPDATE_LCD                 ;tst to see if the lcd can be updated
        lbeq   t1s2_resume               ;if no update then resume like normal
        tst   ISR_UPDATE                 ;test if ISR has given values 
        lbeq   t1s2_resume               ;if not then resume state 2
        ;bgnd
    conversion_routine:    
        
        ldd    TEMPNUM_W                 ;push all variables to stack
        pshd
        ldd    RESULT
        pshd
        ldd    OUTPUTPTR_W
        pshd
        ldab   MAXCOUNT
        pshb
        ldab   COUNT
        pshb
    
        clr    COUNT                     ;clear those values involved in conversions
        clrw   RESULT
        clrw   TEMPNUM_W
        
    conv_begin:    
        ldaa   CONV_COUNT
        ;bgnd
        lbeq   vact_setup
        deca
        lbeq   err_setup
        deca
        lbeq   eff_setup
        bra   conv_end
        
    conv_end:
        ;bgnd
        pulb
        stab   COUNT                      ;pull all the variables pushed to stack
        pulb
        stab   MAXCOUNT
        puld
        std    OUTPUTPTR_W
        puld
        std    RESULT
        puld
        std    TEMPNUM_W
        
        lbra   t1s2_resume
        
    vact_setup:                           ;setup for vactual conversion
        tst   SETUP_DONE
        lbne   conversion
        
        tst   VACT_W
        bmi   neg_vact
        movb  #$20, $1008
        bra   vact_continue
    neg_vact:
        negw  VACT_W
        movb  #$2D, $1008    
    vact_continue:    
        movw  VACT_W, TEMPNUM_W
        movw  VACT_W, RESULT
        movw  #LVACT_BUF+1, OUTPUTPTR_W    
        movb  #$03, MAXCOUNT
        
        movb  #$01, SETUP_DONE
        bra   conversion
    
    err_setup:                            ;setup for error conversion
        tst   SETUP_DONE
        bne   conversion
        
        tst   ERR_BUF_W
        bmi   neg_error
        movb  #$20, $100D
        bra   err_continue
    neg_error:
        negw  ERR_BUF_W
        movb  #$2D, $100D
    err_continue:    
        movw  ERR_BUF_W, TEMPNUM_W
        movw  ERR_BUF_W, RESULT
        movw  #LERR_BUF+1, OUTPUTPTR_W
        
        movb  #$03, MAXCOUNT
        movb  #$01, SETUP_DONE
        bra   conversion
    
    eff_setup:                             ;setup for effort conversion
        tst   SETUP_DONE
        bne   conversion
        tst   EFF_BUF_W
        bmi   neg_effort
        movb  #$20, $1012
        bra   eff_continue
    neg_effort:
        negw  EFF_BUF_W
        movb  #$2D, $1012
    eff_continue:        
        movw  EFF_BUF_W, TEMPNUM_W
        movw  EFF_BUF_W, RESULT
        movw  #LEFF_BUF+1, OUTPUTPTR_W
       
        movb  #$03, MAXCOUNT
        movb  #$01, SETUP_DONE
        bra   conversion
    
    conversion:
        ldd   TEMPNUM_W                    ;conversion routine for ISR values
        ldy   OUTPUTPTR_W
        
        jsr   BtBtA
        
        tst   DIGITREM
        beq   conversion_done
        lbra   conv_begin
    conversion_done:
        movw  #10000, DIVISOR_W
        movb  #$05, DIGITREM
        movb  #$00, COUNT
        clrw  RESULT
        inc   CONV_COUNT
        clr   SETUP_DONE
        lbra   conv_begin
        
                
    t1s2_resume:
        
        tst   KEY_FLG                    ;test key flag for value
        beq   t1s2_exit                  ;branch to exit if boolean zero (Z=1)
        tst   FKEY_FLG
        bne   t1s2_exit
        ;bgnd
        
        ldaa  FUN_TEST                   ;load accu A with contents of FUN_TEST ($F0)
        sba                              ;subtract contents of accu B to accu A and 
                                         ;store in accu A N=0 if not Fn key
        bcc   letter                     

        inca
        beq   t1s2_exit
        
        tst   ENTRY_FLG
        beq   t1s2_exit
        
        movb  #$09, t1state
        
        rts
        
    letter:
        
        ldaa  LET_TEST
        sba
        
        bcc   digit
        
        stab  LET_BUF              ;puts whatever in B to stack and 
                                   ;moves stack pointer by 1 byte
        movb  #$08, t1state
        
        rts
    
    digit:

        ;bgnd

        ldaa  DIGIT_TEST                 ;load accu A with $2F
        sba                              ;sub accu b from A and store in accu A
                                         ;C=0 if not digit
        bcc   ent_chk
        
        tst   ENTRY_FLG
        beq   t1s2_exit
        
        ldaa  COUNT                      ;load accu A with contents of COUNT
        cmpa  MAXCOUNT                   ;compare accu A to maxcount and set Z=1 if equal
        beq   t1s2_exit                  ;branch to exit state if count = maxcount (Z=1)

        movb  #$03, t1state              ;set next state to state 3 storing digits
        movb  #$01, ECHO                 ;sets boolean echo to 1
        
        rts
        
    ent_chk:
        
        ldaa  ENT_TEST                   ;load accu A with $09
        sba                              ;sub accu B from accu A and stor in A
                                         ;N=0 if not Ent key
        bcc   set_bspace                 ;branch to bspace set if N=0       
        
        tst   ENTRY_FLG
        beq   t1s2_exit
        
        movb  #$04, t1state              ;set state 5 for enter key state
        movw  #BUFFER, POINTER           ;put address of BUFFER into 
                                         ;contents of POINTER        
        
        rts 
        
    set_bspace:                              ;set the backspace state
        
        tst   COUNT
        beq   t1s2_exit
        
        movb  #$06, t1state              ;set state 7 for bspace state        
        rts 
                                  
    t1s2_exit:
        
        movb  #$00, KEY_FLG              ;turn off the KEY_FLG so T2 can go to S1
        rts                              ;return from subroutine

t1state3:                                ;store a digit state        
        ;bgnd
        
        ldx   POINTER                    ;load accu X with contents of pointer                                         ;(1st 2 bytes)
        stab  0,x                        ;store the contents accu B in accu x 
                                         ;(mem location of pointer which is buffer)
                                         ;need two arguments so use indexed addressing

        inc   COUNT                      ;increment COUNT to track number of digits
        inx                              ;increment the contents of accu X
        stx   POINTER                    ;store accu x to addr of INPUTPTR
        
        movb  #$02, t1state              ;set next state to state 2, hub
        movb  #$00, KEY_FLG              ;turn off the KEY_FLG so T2 can go to S1
        
        rts                              ;return from subroutine    

t1state4:                                ;enter handler state and ascii to bcd
        ;bgnd
        
        tst  ISR_UPDATE
        beq  t1s4res
        rts


    t1s4res:
        
        tst   COUNT
        beq   t1s4_exit                  ;branch to enter end state if Z=1
        

        jsr   ASCIItoBCD                 ;jump to subroutine ASCIItoBCD
        
        tst   COUNT
        beq   t1s4_t1s10                  ;branch to enter end state if Z=1

        rts
            
     
     t1s4_exit:
        ;bgnd
        movw  #BUFFER, POINTER
        clrw   RESULT
        movb  CURRENT_SIGN, $1003
        ;movb  SIGN_BUF, VREF_SIGN
        movb  #$00, VREF_SIGN
        movb  #$00, KEY_FLG
        movb  #$00, ENTRY_FLG
        movb  #$01, DGAIN
        movb #$00, GAIN_FLG
        movb  #$02, t1state
        
        ldaa  #$27                       ;hiding the cursor in an inconspicuous spot
        jsr   SETADDR
        
        rts   
     
     t1s4_t1s10:    ;Enter Handler
        ;bgnd
        ldd   RESULT                     ;load accu X with contents of RESULT
        
        std   TEMPNUM_W
        tst   VREF_SIGN
        bne   neg_num
        bra   cont
     neg_num:
        negw  RESULT
        ldd   RESULT
        negw  RESULT
     cont:   
        ldx   INPUTPTR_W
        std   0,x                        ;stores RESULT (accu D) to location of IR X
        
        
        movw  #BUFFER, POINTER           ;put address of BUFFER into contents of POINTER
        clr   DONE_CONV
        
        movb  #$01, ENTRY_DONE
        movb  #$0A, t1state              ;set next state to state 2, hub
        
        rts        

ASCIItoBCD:

        movb #$01, DONE_CONV             ;Done ASCIItoBCD flag set

        clra
        ldd   RESULT                     ;load accu D with contents of RESULT (2 bytes)
        ldy   multiplier                 ;load accu Y with the contents of multiplier 
                                         ;(bin 10, 16-bit) 
        EMUL                             ;extended Multiply accu D with IR Y 
                                         ;and store in accu D
        std   RESULT                     ;store contents of accu D to contents 
                                         ;of RESULT (2 bytes)

        clra                             ;clear accu A
        ldx   POINTER
        ldab  0,x                        ;load accu B with contents of 
                                         ;address x (indexed addressing)
                                         ;which is where the POINTER is from Top
        inx
        stx   POINTER
                                         
        subb  #$30                       ;subtract contents of accu B by 
                                         ;hex 30 (makes a BCD)
        addd  RESULT                     ;add contents of D to RESULT and store in D 
                                         ;set flags (C=1 if D > hex FFFF)
        std   RESULT                     ;store contents of accu D to 
                                         ;contents of RESULT (2 bytes)
        
        cpd   #$7FFF
        bhi   TOO_LARGE
        
        
        clry                             ;clear IR Y back to normal state
        
        ldaa  #$00                       ;load accu A with hex 0
        dec   COUNT                      ;dec count and set Z=1 if $00
                
        rts                              ;return from subroutine

     TOO_LARGE:

         movw  #32767, RESULT
         clr   COUNT
         
         rts
                                      ;return from subroutine                               

                             ;wait for error message to finish displaying

t1state6:                                ;backspace state
        ;bgnd
        movb  #$01, BSPACE               ;set backspace flag
        movb  #$00, KEY_FLG              ;reset key flag
        movb  #$07, t1state              ;set the state to the 
                                         ;backspace waiting state       
        rts
        
t1state7:                                ;bspace waiting state

        tst   BSPACE
        beq   t1s7_exit
        rts

t1s7_exit:
        movb  #$02, t1state              ;set state 2 as next state
        rts
        
t1state8:                                ;letter state

        ;bgnd
        ldab LET_BUF
        subb #$41
        lbeq  Apressed
        decb
        lbeq  Bpressed
        decb
        lbeq  Cpressed
        decb
        lbeq  Dpressed
        decb
        lbeq  Epressed
        ;Fpressed if no branch
    Fpressed:
        tst   GAIN_FLG
        bne   Fexit
        tst   LOOP_TYPE
        bne   open_loop
        
        movw  #CL, MSGPTR_W
        movb  #$01, LOOP_TYPE
        bra   Fcontinue
        
    open_loop:
        movw  #OL, MSGPTR_W
        movb  #$00, LOOP_TYPE
        movb #$01, DGAIN
        movw #0, KI_BUF_W
        movw KI_BUF_W, TEMPNUM_W
        movw KI_BUF_W, RESULT
        movw #LKI_BUF, OUTPUTPTR_W
        movb #$60, SYS_CURSOR
        movb #$01, DSYSTEM
        movb #$0A, t1state
        movb #$00, KEY_FLG
        rts    
    Fcontinue:
        movb  #$60, SYS_CURSOR
        movb  #$01, DSYSTEM
    Fexit:
        movb  #$00, KEY_FLG
        movb  #$02, t1state
        clrw  ESUM_W
        rts            
        
    Apressed:  ;run or stop the motor
        ;bgnd
        tst   GAIN_FLG
        bne   Aexit
        tst   RUN_MOTOR
        bne   stop_motor
        
        movw  #RUN_MSG, MSGPTR_W
        movb  #$01, RUN_MOTOR
        bra   Acontinue
        
    stop_motor:
        movw  #STP_MSG, MSGPTR_W
        movb  #$00, RUN_MOTOR
        
    Acontinue:    
        movb  #$5C, SYS_CURSOR
        movb  #$01, DSYSTEM
    Aexit:    
        movb  #$00, KEY_FLG
        movb  #$02, t1state
        clrw  ESUM_W
        
        rts
            
    Bpressed:  ;toggle DISP_UPDATE
        ;bgnd
        tst UPDATE_LCD
        bne update_off
        
        movb #$01, UPDATE_LCD
        movw #DON, MSGPTR_W
        movb  #$63, SYS_CURSOR
        movb #$01, DSYSTEM
        movb #$02, t1state
        movb  #$00, KEY_FLG              ;reset key flag
        
        rts
        
    update_off:
        
        movb #$00, UPDATE_LCD
        movw #DOF, MSGPTR_W
        movb  #$63, SYS_CURSOR
        movb #$01, DSYSTEM
        movb #$02, t1state
        movb  #$00, KEY_FLG              ;reset key flag
        
        rts    
        
    Cpressed:  ;change Vref but don't stop motor
        
        tst  ENTRY_FLG
        lbne  t1s8_exit
        
        movw #VREF_BUF_W, INPUTPTR_W      ;put the address of vref buf to 
                                          ;input ptr for digit storage
        movw #LVREF_BUF+1, OUTPUTPTR_W
        movb #$20, $1003
        ;movb #$01, ECHO
        movb #$02, VREF_FLG
        movb #$01, VREF_DISP
        movb #$03, MAXCOUNT
        
        ldaa #$40                        ;sets address to 7 leaving a space for negative
        jsr  SETADDR
        
        movb #$02, t1state
        movb  #$00, KEY_FLG              ;reset key flag
        movb  #$01, ENTRY_FLG
        clrw  ESUM_W
        rts
    Dpressed:  ;stop motor and change 1024*Kp
        
        tst  ENTRY_FLG
        lbne  t1s8_exit
        
        movb #$00, RUN_MOTOR
        movw #KP_BUF_W, INPUTPTR_W
        movw #LKP_BUF, OUTPUTPTR_W
        ;movb #$01, ECHO
        movb #$01, CLR_TIME
        movb #$05, MAXCOUNT
        movb #$01, GAIN_FLG
        
        ldaa #$48
        staa CURSOR_LOC
        ;jsr  SETADDR
        
        movw #STP_MSG, MSGPTR_W
        movb #$5C, SYS_CURSOR
        movb #$01, DSYSTEM
        movb #$00, RUN_MOTOR
        
        movb #$02, t1state
        movb  #$00, KEY_FLG              ;reset key flag
        movb  #$01, ENTRY_FLG
        clrw  ESUM_W
        rts
    Epressed:  ;stop motor and change 1024*Ki
        
        tst  ENTRY_FLG
        lbne  t1s8_exit
        tst  LOOP_TYPE
        beq  t1s8_exit
        
        movb #$00, RUN_MOTOR
        movw #KI_BUF_W, INPUTPTR_W
        movw #LKI_BUF, OUTPUTPTR_W
        ;movb #$01, ECHO
        movb #$01, CLR_TIME
        movb #$05, MAXCOUNT
        movb #$01, GAIN_FLG
        
        ldaa #$56
        staa CURSOR_LOC
        ;jsr  SETADDR
        
        movw #STP_MSG, MSGPTR_W
        movb #$5C, SYS_CURSOR
        movb #$01, DSYSTEM
        movb #$00, RUN_MOTOR
        
        movb #$02, t1state
        movb  #$00, KEY_FLG              ;reset key flag
        movb  #$01, ENTRY_FLG
        clrw  ESUM_W
        rts
    
    t1s8_exit:    
        
        movb #$02, t1state
        movb  #$00, KEY_FLG              ;reset key flag
        rts
        
t1state9:       ;alter vref state
        ;bgnd
        tst   VREF_FLG
        beq   t1s9_exit
        tst   VREF_DISP
        bne   t1s9_exit
        tst   COUNT
        bne   t1s9_exit
        tst   VREF_SIGN
        beq   set_neg
        
        ;movb  VREF_SIGN, SIGN_BUF
        movb  #$00, VREF_SIGN
        movb  $1003, CURRENT_SIGN
        movb  #$20, $1003
        
        movb  #$00, KEY_FLG              ;reset key flag
        movb  #$01, FKEY_FLG
        movb  #$00, KEY_FLG
        movb  #$02, t1state
        
        rts       

    set_neg:
    
        movb  #$01, VREF_SIGN
        movb  $1003, CURRENT_SIGN
        movb  #$2D, $1003
        movb  #$01, FKEY_FLG
        movb  #$00, KEY_FLG
        movb  #$02, t1state
        
        rts
        
    t1s9_exit:
        movb  #$00, KEY_FLG
        movb  #$02, t1state
        
        rts

t1state10:     ;binary to bcd to ascii state 
        ;bgnd
        ldd TEMPNUM_W
        ldy OUTPUTPTR_W
        ;bgnd
        jsr BtBtA
        
        tst DIGITREM
        beq t1s10_exit
        
        rts
     t1s10_exit:
        ;bgnd
        movw  #10000, DIVISOR_W
        movb  #$05, DIGITREM
        movb  #$00, COUNT
        movb  #$00, KEY_FLG              ;turn off the KEY_FLG so T2 can go to S1
        movb  #$00, ENTRY_FLG
        movb #$00, GAIN_FLG
        clrw  RESULT
        ;movb  #$01, DGAIN
        movb  #$02, t1state
        
        tst   ENTRY_DONE
        bne   gain_update
        bra   t1s10_continue
        
     gain_update:
        movb  #$01, DGAIN
        movb  #$00, ENTRY_DONE
           
     t1s10_continue:   
        ldaa  VREF_FLG
        cmpa  #$02
        beq   vref_set
        
        rts
     vref_set:
        movb  #$01, VREF_FLG
        movb  VREF_SIGN, SIGN_BUF
        movb  #$00, VREF_SIGN
        
        rts      
        
;-------------------------------------------TASK_2-----------------------------------------------------;
TASK_2:

        ldaa  t2state                    ;load accu A with the current state
        beq   t2state0                   ;branch to state0 if Z=1
        deca                             ;decrement accu A by 1
        beq   t2state1                   ;branch to state1 if Z=1
        deca  
        beq   t2state2
        
        rts                              ;return from subroutine

t2state0:
        ;jsr   INITKEY                    ;initialize keypad
        clr   KEY_FLG                    ;clear boolean key flag
        clr BUFFER
        clr BUFFER+1
        clr BUFFER+2
        movb  #$01, t2state              ;set next state to state 1
        rts                              ;return from subroutine
        
t2state1:
        
        tst   LKEY_FLG                   ;wait for a button press flag
        beq   t2s1_exit               ;branch to exit if no button pressed (Z=1)
        
        jsr   GETCHAR                    ;subroutine to get char store in accu B
        
        movb  #$01, KEY_FLG              ;set KEY_FLG to hex 1
        movb  #$02, t2state              ;set next state to state2 after 
                                         ;lkey flag triggered
        
        rts

     t2s1_exit:
        
        rts        
        
t2state2:                
        
        tst   KEY_FLG                    ;test KEY_FLG to see if cleared
        bne   t2s2_exit                  ;if not cleared (Z=0) then rts
        movb  #$01, t2state              ;reset state to state 1, wait for key        
        rts

     t2s2_exit:
        
        rts

;---------------------------------------TASK_3------------------------------
TASK_3:

        ldaa  t3state
        lbeq   t3state0                   ;lcd init state
        deca  
        lbeq   t3state1                   ;hub state
        deca
        lbeq   t3state2                   ;echo
        deca
        lbeq   t3state3                   ;bspace
        deca
        lbeq   t3state4                   ;gain prompts
        deca
        lbeq   t3state5                   ;vref prompts
        deca
        lbeq   t3state6                   ;initial motor condition prompts
        deca
        lbeq   t3state7
        deca
        lbeq   t3state8                   ;time display clear
        deca
        lbeq   t3state9                   ;system display
        deca
        deca
        lbeq   t3state11
        
        rts                              ;return from subroutine
        
t3state0:

        clr   ECHO                       ;clear boolean echo
        clr   BSPACE                     ;clear boolean backspace
        clr   DGAIN
        
        movb  #$01, FIRSTCHAR
        movb  #$01, DPROMPT   
        jsr   INITLCD                    ;initialize the lcd
        jsr   LCDTEMPLATE                ;display the LCD template
        ;bgnd
        jsr   CURSOR_ON
        
        ldaa  #$27
        jsr   SETADDR
        
        movb  #$01, t3state              ;set the next state to state 1
        
        rts                              ;return from subroutine
        
t3state1:                                ;lcd hub state
        
        tst   UPDATE_LCD
        beq   t3s1_resume
        ldaa  CONV_COUNT
        cmpa  #$03
        bne  t3s1_resume
        tst   ISR_UPDATE
        lbeq   t3s1_resume
        ;bgnd                
        jsr  GETADDR
        staa CURSOR_BUF
        
        ;jsr   LCDTEMPLATE
        jsr   UPDATELCD_L1
        
        ldaa CURSOR_BUF
        jsr  SETADDR
        movb #$00, CONV_COUNT
        movb #$00, ISR_UPDATE
        
    t3s1_resume:
        
        tst   DPROMPT
        bne   set_t3s6
        
        tst   DSYSTEM                       ;test to display system condition
        bne   set_t3s9                   ;branch if its not 0 (Z=0)
        
        tst   CLR_TIME                    
        bne   set_t3s8                   
        
        tst   DGAIN                      ;test for message display to be done (Z=1)
        bne   set_t3s4
        
        tst   VREF_DISP
        bne   set_t3s5
        
        ldaa  VREF_FLG
        cmpa  #$01
        beq   set_t3s11
        
        tst   ECHO                       ;test boolean echo
        bne   set_t3s2                   ;branch to set echo if Z=0
        
        tst   BSPACE                     ;test boolean bspace
        bne   set_t3s3                   ;set the bspace state
        
        tst   FKEY_FLG
        bne   set_t3s7
        
        rts
        
set_t3s2:
        movb  #$02,t3state                                        
        rts
        
set_t3s4:
        movb  #$04,t3state                                        
        rts

set_t3s5:
        movb  #$05, t3state
        rts

set_t3s6:
        movb  #$06, t3state
        rts

set_t3s7:
        movb  #$07, t3state
        rts        
        
set_t3s8:
        movb  #$08, t3state             
        rts                              

set_t3s3:
        movb  #$03, t3state             
        rts                             
        
set_t3s9:
        movb  #$09, t3state              
        rts                                             

set_t3s11:
        movb  #$0B, t3state
        rts

t3state4:   ;Display kp and ki prompts ----------------------------------- 
            ;bgnd
            tst FIRSTCHAR
            bne t3s4a               ;Display subsequent character
            jsr PUTCHAR
            tst FIRSTCHAR           ;After returning from PUTCHAR, if FIRSTCHAR=1, 
                                    ;done disp
            bne exit_t3s4
            rts
exit_t3s4:  ;bgnd
            movb #$00, DGAIN
            jsr  UPDATELCD_L2
            ldaa #$27
            jsr  SETADDR
            rts            
t3s4a:      
            jsr GETADDR
            staa CURSOR_BUF
            ldaa #$40               ;Starting location of first character for this MSG 
            movw #CONSTANTS, DPTR    ;Set DPTR to address of MESSAGE1
            lbra PUTCHAR_1ST

t3state5:   ;Display new vref ------------------------------------------------- 
            tst FIRSTCHAR
            bne t3s5a               ;Display subsequent character
            jsr PUTCHAR
            tst FIRSTCHAR           ;After returning from PUTCHAR, if FIRSTCHAR=1, 
                                    ;done disp
            bne exit_t3s5
            rts
exit_t3s5:  ;bgnd
            ldaa #$4B
            staa CURSOR_BUF
            jsr  SETADDR
            movb #$00, VREF_DISP
            rts            
t3s5a:      ldaa #$40               ;Starting location of first character for this MSG 
            movw #NEW_VREF, DPTR    ;Set DPTR to address of MESSAGE1
            lbra PUTCHAR_1ST

t3state6:   ;Display inital motor condition ------------------------------------ 
            tst FIRSTCHAR
            bne t3s6a               ;Display subsequent character
            jsr PUTCHAR
            tst FIRSTCHAR           ;After returning from PUTCHAR, if FIRSTCHAR=1, 
                                    ;done disp
            bne exit_t3s6
            rts
exit_t3s6:  ;bgnd
            movb #$00, DPROMPT
            ldaa #$27
            jsr  SETADDR
            rts            
t3s6a:      ldaa #$5C               ;Starting location of first character for this MSG 
            movw #INITPROMPT, DPTR    ;Set DPTR to address of MESSAGE1
            lbra PUTCHAR_1ST

t3state7:   ;display and toggle the sign of the input velocity

        ldaa #$4A
        jsr  SETADDR
        tst  VREF_SIGN
        bne  disp_neg
        
        ldab #$20
        jsr  OUTCHAR
        
        movb #$00, FKEY_FLG
        movb #$01, t3state
        
        rts

    disp_neg:
        ldab #$2D
        jsr  OUTCHAR
        
        movb #$00, FKEY_FLG
        movb #$01, t3state
        
        rts

t3state8:    ;clear the input for entry
        ;bgnd
        tst   CLR_TIME              ;test if the time needs to be cleared
        beq   t3s8_qexit            ;quick exit if it doesnt

        
        tst   FIRSTCHAR
        bne   t3s8a
        jsr   PUTCHAR
        tst   FIRSTCHAR
        bne   t3s8_exit
        
        rts
        
t3s8_exit:
        
        movb  #$01, t3state         ;go to state 1
        movb  #$00, CLR_TIME        ;time display has finished clearing
        movb  #$00, ECHO            ;reset echo so no keys get displayed  
                                    ;until new keypress
        ldaa  CURSOR_LOC              ;load beginning cursor location
        jsr   SETADDR               ;set the cursor location to accept new keypresses
        
        rts

t3s8_qexit:
        
        movb  #$02, t3state         ;go to state 2 so echo can happen
        rts

t3s8rts:
        rts
         
t3s8a:  
   
        ldaa  CURSOR_LOC              ;load beginning cursor location
        staa CURSOR_BUF
        jsr   SETADDR               ;set the cursor location to 
                                    ;accept new keypresses             
        movw  #TIME_CLR, DPTR
        lbra  PUTCHAR_1ST        

t3state2:        ;echo state
        ;bgnd
        ldx   POINTER                    ;load IR X with contents of 
                                         ;POINTER (next open byte of buffer)
        ldab  -1,x                       ;loac accu B with contents of 
                                         ;address of x minus 1 
                                         ;(previous bit of buffer)
        jsr   OUTCHAR                    ;Jump to outchar subroutine
        
        movb  #$00, ECHO                 ;set echo to boolean zero
        movb  #$01, t3state              ;set next state to state 1, hub
        
        rts

t3state3:     ;backspace state
        ;bgnd
        tst   COUNT
        beq   t3s3_exit

        jsr   GETADDR                    ;jump to getaddr and store address in accu A
        deca                             ;dec accu A by 1 set Z=1 if 0
        jsr   SETADDR                    ;jump to set addr and move cursor to new addr
        
        ldab  #$20                       ;load accu B with a hex space
        jsr   OUTCHAR                    ;print the space
        
        jsr   SETADDR                    ;put cursor back to new addr
        
        dec   COUNT                      ;dec COUNT by 1
        decw  POINTER                    ;dec POINTER by 1
        
        movb  #$00, BSPACE
        movb  #$01, t3state
        
        rts

t3s3_exit:
        
        movb  #$00, BSPACE
        movb  #$01, t3state
        
        rts
        
t3state9: ;-----------------------display system condition-----------------------;
            tst FIRSTCHAR
            bne t3s9a               ;Display subsequent character
            jsr PUTCHAR
            tst FIRSTCHAR           ;After returning from PUTCHAR, if FIRSTCHAR=1, 
                                    ;done disp
            bne exit_t3s9
            rts
            
exit_t3s9:  ;bgnd
            movb #$00, DSYSTEM         ;go to the delay state
            ldaa CURSOR_BUF
            jsr  SETADDR
            rts                
t3s9a:      
            jsr GETADDR
            staa CURSOR_BUF
            ldaa SYS_CURSOR           ;Starting location of intial time display location 
            movw MSGPTR_W, DPTR    
            lbra PUTCHAR_1ST

t3state11:   ;Display vref in top left --------------------------------------------- 
            tst FIRSTCHAR
            bne t3s11a               ;Display subsequent character
            jsr PUTCHAR
            tst FIRSTCHAR           ;After returning from PUTCHAR, if FIRSTCHAR=1, 
                                    ;done disp
            bne exit_t3s11
            rts
exit_t3s11:  ;bgnd
            movb #$00, VREF_FLG
            ldaa #$27
            jsr  SETADDR
            rts            
t3s11a:     ldaa #$06               ;Starting location of first character for this MSG 
            movw #LVREF_BUF, DPTR    ;Set DPTR to address of MESSAGE1
            lbra PUTCHAR_1ST

            

PUTCHAR_1ST:;Put the first character (FIRSTCHAR=1)
            
            jsr SETADDR             ;Set LCDAddr from A
            ldx DPTR
            ldab 0,x
            jsr OUTCHAR
            movb #$00, FIRSTCHAR    ;First character has been printed, 
                                    ;cursor moved to next loc
            inc DPTR+1              ;Increment low byte of DPTR
            bne EXIT_PC1
            inc DPTR                ;Increment high byte of DPTR if overflow
EXIT_PC1:   rts            
PUTCHAR:   ;Display subsequent characters of Message (FIRSTCHAR=0)
            ldx DPTR
            ldab 0,x
            tstb                    ;Test for ASCII null character, message printed
            beq DISP_DONE
            jsr OUTCHAR
            inc DPTR+1              ;Increment low byte of DPTR, if Z flag, overflowed
            bne EXIT_PC           ;Increment high byte of DPTR if inc DPTR+1 overflows
            inc DPTR
EXIT_PC:    rts
DISP_DONE:  ;Done displaying message
            movb #$01, FIRSTCHAR    ;First character of next message needs to be printed
            movb #$01, t3state      ;Go back to hubstate of LCD
            rts

        
;/------------------------------------------------------------------------------------\
;| Subroutines                                                                        |
;\------------------------------------------------------------------------------------/
; General purpose subroutines go here

; Add subroutines here:

;------------------------Binary to BCD to ASCII----------------------------;
BtBtA:
        ldx DIVISOR_W
        idiv
        beq zero
        bra continue
       
    zero:
        ;tst COUNT
        ;beq no_digit
        tstw RESULT
        beq jump
        cpd RESULT
        beq no_digit
        tst COUNT
        bne continue
    jump:    
        movb #$01, DIGITREM
       
    continue:
        std  TEMPNUM_W
        pshx                           ;put ir x value (remainder) on stack
        pulb                           ;pull stack to b (0)
        pulb                           ;pull stack to b (remainder)
        addb #$30                      ;convert to ASCII
        ldx  OUTPUTPTR_W                 
        stab 0,x
        incw OUTPUTPTR_W
        inc  COUNT                   ;increment count for each digit stored
        ldd  DIVISOR_W                 ;load accu D with divisor value
        ldx  #10                       ;load irx with '10'
        idiv                           ;divide divisor by '10'
        stx  DIVISOR_W                 ;store new divisor value
        dec  DIGITREM                ;decrement digits remaining until 0
        beq  finish
        
        rts     
    
    finish:
        
        ldaa COUNT
        suba MAXCOUNT
        beq  EXIT
        ldab #$20
        ldx  OUTPUTPTR_W
        stab 0,x
        incw OUTPUTPTR_W
        inca
        beq  EXIT
        ldab #$20
        ldx  OUTPUTPTR_W
        stab 0,x
        incw OUTPUTPTR_W
        inca
        beq  EXIT
        ldab #$20
        ldx  OUTPUTPTR_W
        stab 0,x
        incw OUTPUTPTR_W
        inca
        beq  EXIT
        ldab #$20
        ldx  OUTPUTPTR_W
        stab 0,x
        incw OUTPUTPTR_W
        inca
        beq  EXIT
        
    no_digit:
        std  TEMPNUM_W                     
        ldd  DIVISOR_W
        ldx  #10
        idiv
        stx  DIVISOR_W
        dec  DIGITREM
        
        rts
 EXIT:
        clr COUNT
        rts
 ;-----------------------END Binary to BCD to ASCII-----------------------------;
 
 ;------------------------ISR---------------------------------------------------;
 ISR:    
        
        ;---------------------set variables-------------------------------------;
        movw  VREF_BUF_W, VREF_W
        movw  KI_BUF_W, KIINPUT_W
        movw  KP_BUF_W, KPINPUT_W
        movw  E_W, ERR_BUF_W
        
        
        
        ;TIMER STUFF HERE------------------------
        ldx TC0H                   ;Load X with current timer value
        addx INTERVAL_INIT         ;Add Interval
        stx TC0H                   ;Store value in TC0
        bset TFR1, %00000001       ;Clear C0F by writing a 1 to it
        ;END TIMER STUFF-------------------------
        
        ;THIS CPU PRESERVES REGISTER STATE, HELL YES!!!
        
        ;V_ACT FROM ENCODER
        jsr READ_ENCODER
        pshd
        subd ENCODER_COUNT_W
        std VACT_W
        puld 
        std ENCODER_COUNT_W
        
        ldx VREF_W
        tst LOOP_TYPE
        beq ISR_OL
        subx VACT_W
        stx E_W                ;e=Vref-Vact
        bra ISR_CTD
        
ISR_OL:
        stx E_W        
        
ISR_CTD: ;SAT BD ADD
        pshx                  ;X->Y
        puly
        ldd ESUM_W
        jsr SatDoubByteAdd;
        std ESUM_W
        ;END SAT DB ADD
             
        ;Integral Action Here
        ldd ESUM_W
        ldy KIINPUT_W
        emuls
        ldx #1024
        edivs
        pshy
        ;End Integral Action
        
        ldd E_W
        ldy KPINPUT_W
        emuls                  ;product goes in Y:D
        ldx #1024
        edivs                  ;quotient in X, remainder in D 
        sty A_W                ;Restult goes into SDBA sum junction, ans is A.
        puld
        jsr SatDoubByteAdd
        ;std A_W
        
        ldx #2
        ;ldd A_W
        idivs
        stx APRIME_W
        
        ;Start Saturator
        cpx SAT512
        bgt SatHi
        blt SatLT          
SatHi:  movw #512, ASTAR_W
        bra SatEx
SatLT:  cpx SATNEG512
        blt SatLo
        movw APRIME_W, ASTAR_W
        bra SatEx
SatLo:  movw #-512, ASTAR_W
        bra SatEx        
SatEx:  ;End Saturator               

        tst RUN_MOTOR
        beq ISR_KILL_MOTOR
        bra ISR_UPDATE_MOTOR
ISR_KILL_MOTOR:                     
        ldd #$8000
        jsr UPDATE_MOTOR
        bra ISR_EXIT                              
ISR_UPDATE_MOTOR:
        ldd ASTAR_W
        jsr UPDATE_MOTOR
ISR_EXIT:
        jsr  DAC_DEBUG              ;Set DAC to (13*Vact)+2048
        jsr  EFF_CALC
        jsr  SatErr
        
        tst  UPDATE_LCD
        beq  ISR_RTI
        tst  ISR_UPDATE
        bne  ISR_RTI
        inc  ISR_COUNTER
        ldab ISR_COUNTER
        cmpb #255
        beq SetISRFlg

ISR_RTI:rti  

SetISRFlg:
        movb #$01, ISR_UPDATE
        clr ISR_COUNTER
        bra ISR_RTI
        
;----------------------------End ISR----------------------------------------;               

EFF_CALC:
    ldd ASTAR_W
    ldy #100
    emuls
    ldx #512
    edivs
    sty EFF_BUF_W
    tst RUN_MOTOR
    beq SetEff0
    rts
SetEff0:
    movw #0, EFF_BUF_W
    rts    
SatErr:
    ldd E_W
    cpd #999
    bgt SatErrHi
    cpd #-999
    blt SatErrLo
SER:rts
SatErrHi:
    movw #999, E_W
    bra SER
SatErrLo:
    movw #-999, E_W
    bra SER       
;-------------------------------------DAC DEBUG-----------------------------;
DAC_DEBUG:
        ldd VACT_W
        ldy #13
        emul
        addd #2048
        jsr OUTDACA
        rts
        
;------------------------------End DAC DEBUG-----------------------------;

;--------------------------Saturated Double Byte Add---------------------;
SatDoubByteAdd:
              pshy
              addd 0,SP
              bvc SDBAExit
              tst 0,SP
              bmi SDBANeg
              ldd #$7FFF
              bra SDBAExit
SDBANeg:      ldd #$8000
SDBAExit:     ins
              ins
              rts              
;-------------------------End Saturated Double byte Add----------------------------;
                
;/------------------------------------------------------------------------------------\
;| ASCII Messages and Constant Data  (FLASH)                                          |
;\------------------------------------------------------------------------------------/
; Any constants can be defined here

DIGIT_TEST DC.B   $2F                        ;define digit in flash
FUN_TEST   DC.B   $F0                        ;define fuction test  in flash
ENT_TEST   DC.B   $09                        ;define Enter test in Flash
BS_TEST    DC.B   $07                        ;define Backspace test in flash
LET_TEST   DC.B   $40                        ;define letter test in flash
multiplier DC.W   %0000000000001010
CONSTANTS  DC.B   '1024*Kp=      1024*Ki=      ',$00
TIME_CLR   DC.B   '     ',$00
NEW_VREF   DC.B   'new V_ref=                  ',$00
RUN_MSG    DC.B   'RUN ',$00
STP_MSG    DC.B   'STP ',$00
CL         DC.B   'CL ',$00                  ;closed loop display
OL         DC.B   'OL ',$00                  ;open loop display
DON        DC.B   'D_ON ',$00                  ;display update on
DOF        DC.B   'D_OFF',$00                  ;display update off
INITPROMPT DC.B   'STP CL D_OFF',$00
INTERVAL_INIT DC.W 20000    ;20000 clock ticks per 2ms
;INTERVAL_INIT DC.W 1000    ;1000 clock ticks per 0.1ms
SAT512 DC.W 512
SATNEG512 DC.W -512

;/------------------------------------------------------------------------------------\
;| Vectors                                                                            |
;\------------------------------------------------------------------------------------/
; Add interrupt and reset vectors here

  ORG   $FFFE                    ; reset vector address
  DC.W  Entry
  ORG   $FFCE                    ; Key Wakeup interrupt vector address [Port J]
  DC.W  ISR_KEYPAD
        
  ORG   $FFEE                    ; ISR for Timer
  DC.W  ISR                      

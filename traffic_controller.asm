;File Name: traffic_controller.asm
;Authors: Dimitriadis Stathis 8490 - Sahinis Alexandros 8906

.include "m16def.inc"

; storing the needed number of
; left shifts to turn on a particular
; traffic light
.equ B = 4
.equ E = 6
.equ A = 0
.equ D = 2
.equ C = 6
.equ F = 4
; storing the needed masks to clear
; previous state of the traffic lights
; with the use of and andi
.equ CLEAR_B = 0b11001111
.equ CLEAR_F = 0b11001111
.equ CLEAR_C = 0b00111111
.equ CLEAR_E = 0b00111111
.equ CLEAR_B_E = 0x0F
.equ CLEAR_A_D = 0xF0
.equ CLEAR_A_D_E = 0b00110000
.equ CLEAR_A_D_B = 0b11000000
; decoder input codes for every colour
.equ GREEN = 2
.equ ORANGE = 1
.equ RED = 0
; pedestrian lights to LED match
.equ PEDESTRIAN_EGNATIA = 0b0100_0000
.equ PEDESTRIAN_HELEXPO = 0b1000_0000

.def delay = r16            ; stores the desired delay for the next transition
.def counter = r17          ; stores the seconds that have passed since last transition
.def next_state = r18       ; stores the next state of the system (after the next transition)
.def curr_state = r19       ; stores the current state of the system
.def button_pressed = r20   ; flag that signals if a button has been pressed
.def temp = r21             ; use as auxillary register
.def end_state = r22        ; stores the state to which the traffic will be resumed after a turn state
.def led_counter = r23      ; stores the number of timer0 overflows, so we can caclulate a grater time
.def leds = r24             ; used for writing out values for the led control

.cseg
.org 0
rjmp init
.org 0x10 jmp OVR_handler
.org 0x16 jmp OVR0_handler

OVR_handler:
    ; for every interrupt, the handler increments the "counter" register
    push temp		; save temp state
    in temp, SREG
    push temp		; save SREG state
     inc counter
    pop temp
    out SREG, temp	; restore SREG state
    pop temp		; restore temp state
    reti

; handles the led patterns
; is called every 50 ms
; using an extra counter, led handling happens every 200 ms
OVR0_handler:
    push temp
    in temp, SREG
    push temp
    inc led_counter
    cpi led_counter, 4          ; 4*50 = 200 ms delay for pedestrian_lights call
    brlo skip_pedestrian_lights ; if 200 ms have not passed, exit the handler
    ldi led_counter, 0          ; else, re-initialize led_counter 
    rcall pedestrian_lights     ; and call pedestrian_lights handle the led patterns
    skip_pedestrian_lights:
        pop temp
        out SREG, temp
        pop temp
        reti

; writes out to the Pedestrian LEDs,
; depending on the state, the corresponding values
pedestrian_lights:
    in leds, PORTB
    cpi curr_state, 0
    brne xcheck_1
    sbr leds,  PEDESTRIAN_EGNATIA           ; turn off LED6 (egnatia)
    cbr leds, PEDESTRIAN_HELEXPO            ; turn on LED7 (helexpo)
    rjmp xfinish
    xcheck_1:
        cpi curr_state, 1
        brne xcheck_2
        sbr leds,  PEDESTRIAN_EGNATIA       ; turn off LED6 (egnatia)
        ldi temp, PEDESTRIAN_HELEXPO			
        eor leds, temp                      ; toggle LED7 (helexpo)
        rjmp xfinish
    xcheck_2:
        cpi curr_state, 2
        brne xcheck_3
        sbr leds, PEDESTRIAN_HELEXPO        ; turn off LED7 (helexpo)
        cbr leds,  PEDESTRIAN_EGNATIA       ; turn on LED6 (egnatia)
        rjmp xfinish
    xcheck_3:
        cpi curr_state, 3
        brne xcheck_else
        sbr leds, PEDESTRIAN_HELEXPO        ; turn off LED7 (helexpo)
        ldi temp,  PEDESTRIAN_EGNATIA			
        eor leds, temp                      ; toggle LED6 (egnatia)
        rjmp xfinish
    xcheck_else:                            ; on a turn, so leds are off
        sbr leds,  PEDESTRIAN_EGNATIA
        sbr leds, PEDESTRIAN_HELEXPO	
    xfinish:
    out PORTB, leds
    ret
    
init:
    ; initialize stack, PORTs, etc.
    ; initialize an interrupt timer with a period of 1sec

    ldi  temp,  low(RAMEND)     ; initialize stack for function calls
    out  SPL,  temp             ; 		
    ldi  temp,  high(RAMEND)    ; 				-
    out  SPH,  temp  

    ; PORTA first 4 bits output, next 4 input
    ldi temp, 0b11110000				 
    out DDRA, temp			
    ldi temp, 0x0F
    out PORTA, temp             ;initialize leds to be switched off

    ; PORTC output
    ser temp
    out DDRC, temp
    out PORTC, temp
    ; PORTB 7 & 8 output, rest input
    ldi temp, 0b11000000
    out DDRB, temp
    ; initial values for pedestrian lights
    ldi temp, 0b0100_0000
    out PORTB, temp

    ;initialize timer/counter ( ~ 1s)
    ldi temp, 0x00
    ldi temp, HIGH(65536 - 62500)
    out TCNT1H, temp

    ldi temp, LOW(65536 - 62500)
    out TCNT1L, temp
    ldi temp, 0x00
    out TCCR1A, temp
    ldi temp, 0b0000_0011       ;overflow mode + 64 prescaler
    out TCCR1B, temp
    
    ; initialize timer0/counter (0.25us * 1024 * 200 = ~ 50 ms)
    ldi temp, 56
    out TCNT0, temp
    ldi temp, 0b0000_0101       ; overflow mode + 1024 prescaler
    ldi led_counter, 0          ; initialize overflow counts to 0
    out TCCR0, temp
    
    ;timer overflow interrupt enable for both timer1 & timer0
    ldi temp, (1<<TOIE1) | (1<<TOIE0)
    out TIMSK, temp	
    sei

; GREEN on egnatia, RED on helexpo, RED on turn
; eg. We want this state to last for 10 secs
state_0:
   ldi curr_state, 0
   clr temp
   ori temp, (GREEN << B) | (GREEN << E)    ; egnatia
   ori temp, (RED << D) | (RED << A)        ; helexpo
   out PORTC, temp
   ldi temp, 0x0F
   ori temp, (RED << C) | (RED << F)        ; egnatia turn
   out PORTA, temp
   ldi delay, 10                            ; 10 sec delay for the next state
   ldi next_state, 1
   rjmp wait_for_transition

; ORANGE on egnatia, RED on helexpo, RED on turn
; eg. We want this state to last for 3 secs
state_1:
   ldi curr_state, 1
   in temp, PINC
   clr temp
   ori temp, (ORANGE << B) | (ORANGE << E)  ; egnatia
   ori temp, (RED << D) | (RED << A)        ; helexpo
   out PORTC, temp
   ; PORTA remains the same, so no operation is needed
   ldi delay, 3                             ; 3 sec delay for the next state
   ldi next_state, 2
   rjmp wait_for_transition

; RED on egnatia, GREEN on helexpo, RED on turn
; eg. We want this state to last for 5 secs
state_2:
   ldi curr_state, 2
   clr temp
   ori temp, (RED << B) | (RED << E)        ; egnatia
   ori temp, (GREEN << D) | (GREEN << A)    ; helexpo
   out PORTC, temp
   ; PORTA remains the same, so no operation is needed
   ldi delay, 5                             ; 5 sec delay for the next state
   ldi next_state, 3
   rjmp wait_for_transition

; RED on egnatia, ORANGE on helexpo, RED on turn
; eg. We want this state to remain true for 3 secs
state_3:
   ldi curr_state, 3
   clr temp
   ori temp, (RED << B) | (RED << E)        ; egnatia
   ori temp, (ORANGE << D) | (ORANGE << A)  ; helexpo
   out PORTC, temp
   ; PORTA remains the same, so no operation is needed
   ldi delay, 3                             ; 3 sec delay for the next state
   ldi next_state, 0
   rjmp wait_for_transition
   
turnF_A:	; F turn and initially helexpo on red
    ldi curr_state, 6
    ; phase 0: ORANGE on B and GREEN on E
    in temp, PINC
    andi temp, CLEAR_B_E
    ori temp, (ORANGE << B) | (GREEN << E)  ; egnatia
    out PORTC, temp
    ldi delay, 3                            ; phase 0 duration: 3s
    rcall wait_for_time_sec
    ; phase 1: RED on B, GREEN on F
    in temp, PINC
    andi temp, CLEAR_B
    ori temp, (RED << B)
    out PORTC, temp
    in temp, PINA
    andi temp, CLEAR_F
    ori temp, (GREEN << F)
    out PORTA, temp
    ldi delay, 3                            ; phase 1 duration: 3s
    rcall wait_for_time_sec
    ; phase 2: ORANGE on E, ORANGE on F
    in temp, PINC
    andi temp, CLEAR_E
    ori temp, (ORANGE << E)
    out PORTC, temp
    in temp, PINA
    andi temp, CLEAR_F
    ori temp, (ORANGE << F)
    out PORTA, temp
    ldi delay, 3                            ; phase 2 duration: 3s
    rcall wait_for_time_sec
    ; transition to end_state = 2 so normal flow can be resumed
    ldi temp, 0x0F                          ; RED on TURNs lights
    out PORTA, temp
    rjmp state_2

turnF_B:	; F turn and initially egnatia on red
    ldi curr_state, 7
    ; phase 0: ORANGE on A, ORANGE on D
    in temp, PINC
    andi temp, CLEAR_A_D
    ori temp, (ORANGE << D) | (ORANGE << A) ; helexpo
    out PORTC, temp
    ldi delay, 3                            ; phase 0 duration: 3s
    rcall wait_for_time_sec
    ; phase 1: RED on A & D, GREEN on E & F
    in temp, PINC
    andi temp, CLEAR_A_D_E
    ori temp, (RED << A) | (RED << D) | (GREEN << E)
    out PORTC, temp
    in temp, PINA
    andi temp, CLEAR_F
    ori temp, (GREEN << F)
    out PORTA, temp
    ldi delay, 3                            ; phase 1 duration: 3s
    rcall wait_for_time_sec
    ; phase 2: ORANGE on F
    in temp, PINA
    andi temp, CLEAR_F
    ori temp, (ORANGE << F)
    out PORTA, temp
    ldi delay, 3                            ; phase 2 duration: 3s
    rcall wait_for_time_sec
    ; transition to end_state = 0, so normal flow can be resumed
    ldi temp, 0x0F                          ; RED on TURNs lights
    out PORTA, temp
    rjmp state_0
    
turnC_A:	; C turn and initially helexpo on red
    ldi curr_state, 6
    ; phase 0: ORANGE on E and GREEN on B
    in temp, PINC
    andi temp, CLEAR_B_E
    ori temp, (ORANGE << E) | (GREEN << B)  ; egnatia
    out PORTC, temp
    ldi delay, 3                            ; phase 0 duration: 3s
    rcall wait_for_time_sec
    ; phase 1: RED on E, GREEN on C
    in temp, PINC
    andi temp, CLEAR_E
    ori temp, (RED << E)
    out PORTC, temp
    in temp, PINA
    andi temp, CLEAR_C
    ori temp, (GREEN << C)
    out PORTA, temp
    ldi delay, 3                            ; phase 1 duration: 3s
    rcall wait_for_time_sec
    ; phase 2: ORANGE on B, ORANGE on C
    in temp, PINC
    andi temp, CLEAR_B
    ori temp, (ORANGE << B)
    out PORTC, temp
    in temp, PINA
    andi temp, CLEAR_C
    ori temp, (ORANGE << C)
    out PORTA, temp
    ldi delay, 3                            ; phase 2 duration: 3s
    rcall wait_for_time_sec
    ; transition to end_state = 2 so normal flow can be resumed
    ldi temp, 0x0F                          ; RED on TURNs lights
    out PORTA, temp
    rjmp state_2

turnC_B:	; C turn and initially egnatia on red
    ldi curr_state, 7
    ; phase 0: ORANGE on A, ORANGE on D
    in temp, PINC
    andi temp, CLEAR_A_D
    ori temp, (ORANGE << D) | (ORANGE << A) ; helexpo
    out PORTC, temp
    ldi delay, 3                            ; phase 0 duration: 3s
    rcall wait_for_time_sec
    ; phase 1: RED on A & D, GREEN on B & C
    in temp, PINC
    andi temp, CLEAR_A_D_B
    ori temp, (RED << A) | (RED << D) | (GREEN << B)
    out PORTC, temp
    in temp, PINA
    andi temp, CLEAR_C
    ori temp, (GREEN << C)
    out PORTA, temp
    ldi delay, 3                            ; phase 1 duration: 3s
    rcall wait_for_time_sec
    ; phase 2: ORANGE on C
    in temp, PINA
    andi temp, CLEAR_C
    ori temp, (ORANGE << C)
    out PORTA, temp
    ldi delay, 3                            ; phase 2 duration: 3s
    rcall wait_for_time_sec
    ; transition to end_state = 0 so normal flow can be resumed
    ldi temp, 0x0F                          ; RED on TURNs lights
    out PORTA, temp
    rjmp state_0

; delay is previously set in order to  trigger
; the next transition at the right time
wait_for_transition:
   ldi button_pressed, 0
   ldi counter, 0
   loop:
        rcall check_button_press    ; custom handling for if a switch is pressed
        cpi button_pressed, 1       ; if a switch press has indeed occured,
        breq state_change           ; break the loop and proceed with the next state
        cp counter, delay           ; if time passed is less than the desired delay, loop
        brlo loop
    state_change:
        cpi next_state, 0
        brne check_1
        rjmp state_0
        check_1:
            cpi next_state, 1
            brne check_2
            rjmp state_1
        check_2:
            cpi next_state, 2
            brne check_3
            rjmp state_2
        check_3:
            cpi next_state, 3
            brne check_4
            rjmp state_3
        check_4:
            cpi next_state, 4       ; C turn and initially helexpo on red
            brne check_5
            rjmp turnC_A
        check_5:
            cpi next_state, 5       ; C turn and initially egnatia on red
            brne check_6
            rjmp turnC_B
        check_6:
            cpi next_state, 6       ; F turn and initially helexpo on red
            brne check_7
            rjmp turnF_A
        check_7:
            cpi next_state, 7       ; F turn and initially egnatia on red
            brne wait_for_transition
            rjmp turnF_B
        

check_button_press:
    sbic PINA, 0                    ; if button A pressed, set the next state accordingly
    rjmp check_button_press_B       ; else proceed with checking button B
    cpi curr_state, 2
    brne check_button_press_B       ; if D traffic lights are not green, there is no point of state change
    ldi next_state, 3               ; else, orange will appear on D & A traffic lights
    ldi button_pressed, 1           ; indicate that a button event that needs to be served has occured
    rjmp check_button_press_end
    check_button_press_B:
        sbic PINA, 1                ; if button B pressed, set the next state accordingly
        rjmp check_button_press_C   ; else proceed with checking button C
        cpi curr_state, 0
        brne check_button_press_C   ; if B & E traffic lights are not green, there is no point of state change
        ldi next_state, 1           ; else, orange will appear on B & E traffic lights
        ldi button_pressed, 1       ; indicate that a button event that needs to be served has occured
        rjmp check_button_press_end
    check_button_press_C:
        sbic PINA, 2
        rjmp check_button_press_F
        ldi button_pressed, 1           ; indicate that a button event that needs to be served has occured
        sbrs curr_state, 1              ; state 2 or 3 -> end_state = 0
        rjmp button_press_C_end_state_0
        ldi end_state, 2                ; state 0 or 1 -> end_state = 2
        ldi next_state, 5               ; initially egnatia on red
        rjmp button_press_C_end
        button_press_C_end_state_0:
            ldi end_state, 0
            ldi next_state, 4           ; initially helexpo on red
        button_press_C_end:
            rjmp check_button_press_end
    check_button_press_F:
        sbic PINA, 3
        rjmp check_button_press_end		
        ldi button_pressed, 1           ; indicate that a button event that needs to be served has occured
        sbrs curr_state, 1              ; state 2 or 3 -> end_state = 0
        rjmp button_press_F_end_state_0
        ldi end_state, 2                ; state 0 or 1 -> end_state = 2
        ldi next_state, 7               ; initially egnatia on red
        rjmp check_button_press_end
        button_press_F_end_state_0:
            ldi end_state, 0
            ldi next_state, 6           ; initially helexpo on red
    check_button_press_end:
        ret

; simply waits for time = delay seconds
wait_for_time_sec:
    ldi counter, 0
    loop_wait:
        cp counter, delay
        brlo loop_wait
    ret
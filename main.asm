;********************************************************  
; Universidad del Valle de Guatemala  
; IE2023: Programación de Microcontroladores  
; Test.asm  
;  
; Author : Willy Ulises Cuellar Bendfeldt   
; Proyecto: Reloj Digital    
; Hardware: ATMega328P  
; Creado: 16/03/2025   
; Modificado: 11/02/2025  
; Descipción: Este programa consiste en ......    
; Reloj Digital ATMega328P - 7 Estados
; (0: Hora, 1: Conf. Minutos, 2: Conf. Horas, 3: Fecha, 4: Conf. Día,
;  5: Conf. Mes, 6: Alarma)
; Versión para Arduino Nano (16MHz)
;
; LED1: PB5 (configuración), LED2: PC0 (horas/alarma), LED3: PC1 (fecha/mes)
;**********************************************************************
.include "m328pdef.inc"  ; Librería con datos del microcontrolador

;==================== DEFINICIONES ======================
.equ F_CPU = 16000000     ; Frecuencia del reloj (16MHz)
.equ EST_HORA = 0         ; Estado 0: Mostrar hora normal
.equ EST_CONF_MIN = 1     ; Estado 1: Ajustar minutos
.equ EST_CONF_HORA = 2    ; Estado 2: Ajustar horas
.equ EST_FECHA = 3        ; Estado 3: Mostrar fecha
.equ EST_CONF_DIA = 4     ; Estado 4: Ajustar día
.equ EST_CONF_MES = 5     ; Estado 5: Ajustar mes
.equ EST_ALARM = 6        ; Estado 6: Configurar alarma
.equ T0_CTC_VAL = 30      ; Valor para temporizador de ~2ms

;==================== MEMORIA RAM ========================
.dseg
.org 0x0100
current_state: .byte 1    ; ¿En qué modo estoy? (ej: hora, configuración)
hours: .byte 1            ; Horas actuales (0-23)
minutes: .byte 1          ; Minutos actuales (0-59)
seconds: .byte 1          ; Segundos (tic-tac)
day: .byte 1              ; Día del mes (1-31)
month: .byte 1            ; Mes (1-12)
alarm_h: .byte 1          ; Hora de la alarma
alarm_m: .byte 1          ; Minuto de la alarma
disp_index: .byte 1       ; ¿Qué dígito mostrar? (0-3)
alarm_enabled: .byte 1    ; ¿Alarma activada? (0 = no, 1 = sí)
alarm_active: .byte 1     ; ¿Alarma sonando? (0 = no, 1 = sí)
alarm_counter: .byte 1    ; Contador para apagar alarma después de 60s
beep_count: .byte 1       ; Contador para parpadear el sonido
tmr_counter: .byte 1      ; Contador principal de tiempo
tmr_counter2: .byte 1     ; Contador secundario para 1 segundo
oldPINB: .byte 1          ; Estado anterior del botón PB4
oldPINC: .byte 1          ; Estado anterior de los botones PC2-PC4
mode_debouncing: .byte 1  ; Bloqueo para botón de modo
pcint1_debouncing: .byte 1; Bloqueo para botones +/-/alarma
dp_count: .byte 1         ; Contador para parpadear el punto decimal
dp_state: .byte 1         ; ¿Punto decimal encendido? (0 = no, 1 = sí)
alarm_silenced: .byte 1   ; ¿Alarma silenciada manualmente?

;==================== INTERRUPCIONES ========================
.cseg
.org 0x0000
    jmp START             ; Salto al inicio del programa

; Interrupción por botón de modo (PB4)
.org PCI0addr
    jmp PCINT0_ISR

; Interrupción por botones +/- y alarma (PC2-PC4)
.org PCI1addr
    jmp PCINT1_ISR

; Interrupción cada ~2ms (actualización de display y tiempo)
.org OC0Aaddr
    jmp TIMER0_ISR

;----------------------------------------------------------
; Tablas en memoria Flash:
;----------------------------------------------------------
.org 0x0100
DigitsTable:               ; Patrones para display de 7 segmentos
    .db 0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F
MonthDays:                 ; Días que tiene cada mes (ene=31, feb=28, etc)
    .db 31,28,31,30,31,30,31,31,30,31,30,31

;***********************************************************
; Inicio del Programa:
;***********************************************************
.org 0x0200
START:
    cli                    ; Desactiva interrupciones
    ; Configuración de la pila
    ldi r16, LOW(RAMEND)
    out SPL, r16
    ldi r16, HIGH(RAMEND)
    out SPH, r16

    ; Puerto B: PB5 (LED1), PB0-PB3 (dígitos display)
    ldi r16, 0b00101111    ; PB5 salida, PB0-3 salida, PB4 entrada
    out DDRB, r16
    ldi r16, 0b00010000    ; Resistencia pull-up en PB4 (botón modo)
    out PORTB, r16

    ; Puerto C: PC0-1 (LED2-3), PC2-4 (botones), PC5 (buzzer)
    ldi r16, 0b00100111    ; PC0-1 y PC5 salidas, PC2-4 entradas
    out DDRC, r16
    ldi r16, 0b00011100    ; Resistencias pull-up en PC2-4 (botones)
    out PORTC, r16

    ; Puerto D: Todos salidas (segmentos del display)
    ldi r16, 0xFF
    out DDRD, r16
    ldi r16, 0
    out PORTD, r16         ; Apaga todos los segmentos

    ; Configuración del Timer0 (~2ms):
    ldi r16, (1<<WGM01)    ; Modo CTC
    out TCCR0A, r16
    ldi r16, (1<<CS02)|(1<<CS00) ; Prescaler 1024 (16MHz/1024=15625Hz)
    out TCCR0B, r16
    ldi r16, T0_CTC_VAL    ; Valor para 2ms (30 * 1/15625Hz ? 1.92ms)
    out OCR0A, r16
    ldi r16, (1<<OCIE0A)   ; Habilita interrupción por comparación
    sts TIMSK0, r16

    ; Configuración de interrupciones por botones:
    ldi r16, (1<<PCINT4)   ; Habilita PB4 (botón modo)
    sts PCMSK0, r16
    ldi r16, (1<<PCINT10)|(1<<PCINT11)|(1<<PCINT12) ; PC2-4 (botones +/-/alarma)
    sts PCMSK1, r16
    ldi r16, (1<<PCIE0)|(1<<PCIE1) ; Habilita interrupciones de botones
    sts PCICR, r16

    ; Inicialización de variables:
    clr r16
    sts current_state, r16 ; Estado inicial: mostrar hora
    sts alarm_enabled, r16 ; Alarma desactivada
    sts alarm_active, r16  ; Alarma no activa
    sts alarm_counter, r16 ; Contador de alarma a cero
    sts seconds, r16       ; 0 segundos
    sts minutes, r16       ; 0 minutos
    sts hours, r16         ; 0 horas
    ldi r16, 1
    sts day, r16           ; Día 1
    sts month, r16         ; Mes 1
    clr r16
    sts alarm_h, r16       ; Hora de alarma 0
    sts alarm_m, r16       ; Minuto de alarma 0
    sts beep_count, r16    ; Contador de beep a cero
    sts tmr_counter, r16   ; Contador de tiempo a cero
    sts tmr_counter2, r16
    sts disp_index, r16    ; Índice de display 0
    clr r16
    sts mode_debouncing, r16 ; Sin rebote en botón modo
    sts pcint1_debouncing, r16 ; Sin rebote en botones +/-/alarma
    clr r16
    sts dp_count, r16      ; Contador de punto decimal a cero
    sts dp_state, r16      ; Punto decimal apagado
    clr r16
    sts alarm_silenced, r16 ; Alarma no silenciada

    ; Guarda estados iniciales de los pines
    in  r16, PINB
    sts oldPINB, r16
    in  r16, PINC
    sts oldPINC, r16

    sei                    ; Habilita interrupciones

MAIN_LOOP:
    jmp MAIN_LOOP          ; Bucle infinito (todo se maneja por interrupciones)

;**********************************************************************
; PCINT0_ISR: Botón de modo (PB4)
;**********************************************************************
PCINT0_ISR:
    push r16               ; Guarda registros
    in r16, SREG
    push r16
    push r17
    push r18

    in   r17, PINB         ; Lee estado actual de PB4
    lds  r18, oldPINB      ; Compara con estado anterior

    ; Antirrebote: si ya está en modo debounce, espera a que se suelte
    lds r16, mode_debouncing
    tst  r16
    breq CHECK_MODE_FALLING ; Si no hay rebote, continua
    sbrs r17, PB4          ; Si PB4 está suelto, desbloquea
    rjmp MODE0_EXIT

CHECK_MODE_FALLING:
    ; Detecta flanco descendente (botón presionado)
    sbrc r18, PB4          ; Si antes estaba alto...
    sbrs r17, PB4          ; ...y ahora bajo, cambia estado
    rjmp MODO_FALLING
    rjmp MODE0_EXIT

MODO_FALLING:
    ; Cambia de estado (0 a 6, luego vuelve a 0)
    lds r16, current_state
    inc r16
    cpi r16, 7
    brlo ST_MODE
    clr r16
ST_MODE:
    sts current_state, r16 ; Actualiza estado
    rcall UPDATE_LEDS      ; Enciende LEDs correspondientes
    ldi r16, 1
    sts mode_debouncing, r16 ; Bloquea rebote

MODE0_EXIT:
    in   r16, PINB         ; Guarda estado actual de PB4
    sts oldPINB, r16
    pop  r18
    pop  r17
    pop  r16
    out SREG, r16
    pop r16
    reti                   ; Retorna de interrupción

;**********************************************************************
; PCINT1_ISR: Botones +/- y alarma (PC2-PC4)
;**********************************************************************
PCINT1_ISR:
    push r16
    in   r16, SREG
    push r16
    push r17
    push r18

    in   r17, PINC         ; Lee estado actual de botones
    lds  r18, oldPINC

    ; Antirrebote: verifica si hay rebote activo
    lds r16, pcint1_debouncing
    tst r16
    breq PCINT1_CHECK_FALLING ; Si no hay rebote, procesa botones
    ; Si hay rebote, verifica si todos los botones están sueltos
    in   r16, PINC
    sbrs r16, PC2          ; Botón "+" suelto?
    rjmp PCINT1_EXIT
    sbrs r16, PC3          ; Botón "-" suelto?
    rjmp PCINT1_EXIT
    sbrs r16, PC4          ; Botón "alarma" suelto?
    rjmp PCINT1_EXIT
    clr r16                ; Desbloquea rebote
    sts pcint1_debouncing, r16
    rjmp PCINT1_EXIT

PCINT1_CHECK_FALLING:
    ; Botón PC4 (apagar alarma):
    sbrc r18, PC4          ; Si antes estaba presionado...
    sbrs r17, PC4          ; ...y ahora suelto, activa
    rjmp ALARMA_OFF

    ; Botón PC2 (incrementar):
    sbrc r18, PC2          ; Verifica flanco descendente
    sbrs r17, PC2
    rjmp DO_INCREMENT

    ; Botón PC3 (decrementar):
    sbrc r18, PC3
    sbrs r17, PC3
    rjmp DO_DECREMENT

    rjmp PCINT1_EXIT       ; Si no hay flanco válido, sale

;------------------- Incrementar/decrementar según estado
DO_INCREMENT:
    lds r16, current_state ; ¿Qué estoy configurando?
    cpi r16, EST_CONF_MIN  ; Si es minutos...
    brne SKIP_MIN_INC
    rjmp CONFIG_MIN_INC    ; ...aumenta minutos
SKIP_MIN_INC:
    cpi r16, EST_CONF_HORA ; Si es horas...
    brne SKIP_HORA_INC
    rjmp CONFIG_HORA_INC   ; ...aumenta horas
SKIP_HORA_INC:
    cpi r16, EST_CONF_DIA  ; Si es día...
    brne SKIP_DIA_INC
    rjmp CONFIG_DIA_INC    ; ...aumenta día
SKIP_DIA_INC:
    cpi r16, EST_CONF_MES  ; Si es mes...
    brne SKIP_MES_INC
    rjmp CONFIG_MES_INC    ; ...aumenta mes
SKIP_MES_INC:
    cpi r16, EST_ALARM     ; Si es alarma...
    brne PCINT1_EXIT
    rjmp CONFIG_ALARMA_INC ; ...aumenta alarma
DO_DECREMENT:
    lds r16, current_state
    cpi r16, EST_CONF_MIN  ; Decrementa minutos
    brne SKIP_MIN_DEC
    rjmp CONFIG_MIN_DEC
SKIP_MIN_DEC:
    cpi r16, EST_CONF_HORA ; Decrementa horas
    brne SKIP_HORA_DEC
    rjmp CONFIG_HORA_DEC
SKIP_HORA_DEC:
    cpi r16, EST_CONF_DIA  ; Decrementa día
    brne SKIP_DIA_DEC
    rjmp CONFIG_DIA_DEC
SKIP_DIA_DEC:
    cpi r16, EST_CONF_MES  ; Decrementa mes
    brne SKIP_MES_DEC
    rjmp CONFIG_MES_DEC
SKIP_MES_DEC:
    cpi r16, EST_ALARM     ; Decrementa alarma
    brne PCINT1_EXIT
    rjmp CONFIG_ALARMA_DEC

;=====================================================================
; ALARMA_OFF: Silencia la alarma manualmente
;=====================================================================
ALARMA_OFF:
    clr r16                ; Desactiva alarma
    sts alarm_active, r16
    ldi r16, 1
    sts alarm_silenced, r16 ; Marca como silenciada
    in  r19, PORTC
    andi r19, 0xDF         ; Apaga buzzer (PC5=0)
    out PORTC, r19
    ldi r16, 1
    sts pcint1_debouncing, r16 ; Bloquea rebote
    rjmp PCINT1_EXIT

PCINT1_EXIT:
    in   r19, PINC         ; Guarda estado actual de botones
    sts oldPINC, r19
    pop  r18
    pop  r17
    pop  r16
    out SREG, r16
    pop r16
    reti

;**********************************************************************
; Subrutinas para incrementar/decrementar valores
;**********************************************************************
CONFIG_MIN_INC:
    lds r16, minutes       ; Carga minutos actuales
    inc r16               ; +1
    cpi r16, 60           ; ¿Llegó a 60?
    brlo STORE_MIN_INC    ; No: guarda
    ldi r16, 0            ; Sí: reinicia a 0
STORE_MIN_INC:
    sts minutes, r16
    rjmp CONFIG_MIN_EXIT  ; Sale

CONFIG_MIN_DEC:
    lds r16, minutes       ; Carga minutos
    cpi r16, 0            ; ¿Es 0?
    brne MIN_DEC_CONT     ; No: decrementa
    ldi r16, 59           ; Sí: vuelve a 59
    rjmp MIN_DEC_SAVE
MIN_DEC_CONT:
    dec r16               ; -1
MIN_DEC_SAVE:
    sts minutes, r16
    rjmp CONFIG_MIN_EXIT

CONFIG_MIN_EXIT:
    ldi r16, 1            ; Bloquea rebote
    sts pcint1_debouncing, r16
    rjmp PCINT1_EXIT

CONFIG_HORA_INC:
    lds r16, hours        ; Similar a minutos, pero para horas
    inc r16
    cpi r16, 24
    brlo STORE_HORA_INC
    ldi r16, 0
STORE_HORA_INC:
    sts hours, r16
    rjmp CONFIG_HORA_EXIT

CONFIG_HORA_DEC:
    lds r16, hours
    cpi r16, 0
    brne HORA_DEC_CONT
    ldi r16, 23
    rjmp HORA_DEC_SAVE
HORA_DEC_CONT:
    dec r16
HORA_DEC_SAVE:
    sts hours, r16
    rjmp CONFIG_HORA_EXIT

CONFIG_HORA_EXIT:
    ldi r16, 1
    sts pcint1_debouncing, r16
    rjmp PCINT1_EXIT

;=== Subrutina para obtener días del mes actual
GET_MAX_DAY:
    lds r16, month        ; Mes actual -1 (índice de tabla)
    dec r16
    ldi r18, low(MonthDays*2) ; Apunta a la tabla de días
    add r18, r16
    ldi r31, high(MonthDays*2)
    mov r30, r18
    lpm r17, Z           ; Carga días máximos del mes
    ret

CONFIG_DIA_INC:
    rcall GET_MAX_DAY    ; Obtiene días del mes
    lds r16, day         ; Día actual +1
    inc r16
    cp r16, r17          ; ¿Excede el máximo?
    brlo ST_DAY_OK       ; No: guarda
    breq ST_DAY_OK       ; Si es igual (ej: 31), también avanza
    ldi r16, 1           ; Reinicia a día 1
ST_DAY_OK:
    sts day, r16
    rjmp CONFIG_DIA_EXIT

CONFIG_DIA_DEC:
    rcall GET_MAX_DAY    ; Obtiene días del mes
    lds r16, day         ; Día actual -1
    cpi r16, 1          ; ¿Es 1? (no puede ser 0)
    brne DIA_DEC_CONT
    mov r16, r17         ; Vuelve al último día del mes
    rjmp STORE_DIA_DEC
DIA_DEC_CONT:
    dec r16
STORE_DIA_DEC:
    sts day, r16
    rjmp CONFIG_DIA_EXIT

CONFIG_DIA_EXIT:
    ldi r16, 1
    sts pcint1_debouncing, r16
    rjmp PCINT1_EXIT

CONFIG_MES_INC:
    lds r16, month       ; Mes actual +1
    inc r16
    cpi r16, 13          ; ¿Llegó a 13?
    brlo STORE_MES_INC  ; No: guarda
    ldi r16, 1           ; Sí: reinicia a enero
STORE_MES_INC:
    sts month, r16
    rjmp CONFIG_MES_EXIT

CONFIG_MES_DEC:
    lds r16, month       ; Mes actual -1
    cpi r16, 1          ; ¿Es enero?
    brne MES_DEC_CONT   ; No: decrementa
    ldi r16, 12         ; Sí: vuelve a diciembre
    rjmp MES_DEC_SAVE
MES_DEC_CONT:
    dec r16
MES_DEC_SAVE:
    sts month, r16
    rjmp CONFIG_MES_EXIT

CONFIG_MES_EXIT:
    ldi r16, 1
    sts pcint1_debouncing, r16
    rjmp PCINT1_EXIT

CONFIG_ALARMA_INC:
    lds r16, alarm_m     ; Minutos de alarma +1
    inc r16
    cpi r16, 60          ; Si llega a 60...
    brlo STORE_ALARM_INC
    ldi r16, 0           ; ...reinicia minutos
    lds r19, alarm_h     ; Horas de alarma +1
    inc r19
    cpi r19, 24          ; Si llega a 24...
    brlo STORE_ALARM_H
    ldi r19, 0           ; ...reinicia horas
STORE_ALARM_H:
    sts alarm_h, r19
STORE_ALARM_INC:
    sts alarm_m, r16
    ldi r16, 1
    sts alarm_enabled, r16 ; Activa alarma
    rjmp CONFIG_ALARMA_EXIT

CONFIG_ALARMA_DEC:
    lds r16, alarm_m     ; Minutos de alarma -1
    cpi r16, 0           ; Si es 0...
    brne ALARM_DEC_CONT
    ldi r16, 59          ; ...vuelve a 59
    lds r19, alarm_h     ; Horas de alarma -1 (si no es 0)
    tst r19
    breq ALARM_DEC_CONT
    dec r19
    sts alarm_h, r19
ALARM_DEC_CONT:
    dec r16
    sts alarm_m, r16
    ldi r16, 1
    sts alarm_enabled, r16
    rjmp CONFIG_ALARMA_EXIT

CONFIG_ALARMA_EXIT:
    ldi r16, 1
    sts pcint1_debouncing, r16
    rjmp PCINT1_EXIT

;**********************************************************************
; TIMER0_ISR: Rutina que se ejecuta cada ~2ms
;**********************************************************************
TIMER0_ISR:
    push r16
    push r17
    push r18
    push r19
    in   r16, SREG
    push r16

    ; (1) Actualiza display (multiplexación)
    lds r18, disp_index    ; ¿Qué dígito toca?
    rcall GET_DISPLAY_DIGIT ; Obtiene patrón del dígito
    out PORTD, r19         ; Muestra el patrón en los segmentos

    ; Controla qué dígito está activo (PB0-PB3)
    in  r16, PORTB
    andi r16, 0xF0         ; Apaga todos los dígitos
    cpi r18, 0             ; Dígito 0 (PB0)
    breq SET_D0
    cpi r18, 1             ; Dígito 1 (PB1)
    breq SET_D1
    cpi r18, 2             ; Dígito 2 (PB2)
    breq SET_D2
SET_D3:                    ; Dígito 3 (PB3)
    ori r16, 0x08
    rjmp UPDATE_DIG
SET_D0:
    ori r16, 0x01
    rjmp UPDATE_DIG
SET_D1:
    ori r16, 0x02
    rjmp UPDATE_DIG
SET_D2:
    ori r16, 0x04
UPDATE_DIG:
    out PORTB, r16         ; Activa el dígito correspondiente
    inc r18               ; Siguiente dígito
    cpi r18, 4
    brlo STORE_IDX
    ldi r18, 0            ; Reinicia a 0 si llegó a 4
STORE_IDX:
    sts disp_index, r18   ; Guarda índice

    ; (2) Contador de 1 segundo
    lds r16, tmr_counter
    inc r16
    sts tmr_counter, r16
    cpi r16, 250          ; 250 * 2ms = 500ms
    brlo SKIP_COUNT2
    clr r16
    sts tmr_counter, r16
    lds r17, tmr_counter2
    inc r17
    sts tmr_counter2, r17
    cpi r17, 2            ; 2 * 500ms = 1 segundo
    brlo SKIP_SECOND
    clr r17
    sts tmr_counter2, r17

    ; Actualiza reloj si está en modo HORA o FECHA
    lds r16, current_state
    cpi r16, EST_HORA
    breq UPDATE_CLOCK_LABEL
    cpi r16, EST_FECHA
    breq UPDATE_CLOCK_LABEL
    rjmp SKIP_SECOND

UPDATE_CLOCK_LABEL:
    rcall UPDATE_CLOCK    ; Actualiza tiempo
    ; Controla duración de alarma (60 segundos)
    lds r16, alarm_active
    tst r16
    breq SKIP_ALCNT
    lds r16, alarm_counter
    inc r16
    sts alarm_counter, r16
    cpi r16, 60           ; ¿Lleva 60 segundos sonando?
    brlo SKIP_ALCNT
    clr r16
    sts alarm_active, r16 ; Desactiva alarma
    in r17, PORTC
    andi r17, 0xDF        ; Apaga buzzer
    out PORTC, r17
SKIP_ALCNT:
    rcall ALARM_CHECK     ; Verifica si debe activar alarma

SKIP_SECOND:
SKIP_COUNT2:
    ; Parpadeo del buzzer cada 250ms (0.5s total)
    lds r16, alarm_active
    tst r16
    breq T0_END_BEEP
    lds r16, beep_count
    inc r16
    sts beep_count, r16
    cpi r16, 125          ; 125 * 2ms = 250ms
    brlo T0_END_BEEP2
    clr r16
    sts beep_count, r16
    in  r17, PORTC
    ldi r19, (1<<PC5)     ; Alterna estado del buzzer
    eor r17, r19
    out PORTC, r17

T0_END_BEEP2:
T0_END_BEEP:
    ; Parpadeo del punto decimal cada 500ms
    lds r16, dp_count
    inc r16
    sts dp_count, r16
    cpi r16, 250          ; 250 * 2ms = 500ms
    brlo T0_END_DP
    clr r16
    sts dp_count, r16
    lds r17, dp_state     ; Alterna estado del punto
    ldi r19, 1
    eor r17, r19
    sts dp_state, r17

T0_END_DP:
    pop r16
    out SREG, r16
    pop r19
    pop r18
    pop r17
    pop r16
    reti

;**********************************************************************
; GET_DISPLAY_DIGIT: Decide qué número mostrar en cada dígito
;**********************************************************************
GET_DISPLAY_DIGIT:
    push r20
    push r21
    push r22

    lds r20, current_state ; ¿En qué modo estoy?
    lds r22, disp_index    ; ¿Qué dígito toca?

    ; Si es modo fecha o configuración de día/mes, muestra DD/MM
    cpi r20, EST_FECHA
    breq FECHA_DIGIT
    cpi r20, EST_CONF_DIA
    breq FECHA_DIGIT
    cpi r20, EST_CONF_MES
    breq FECHA_DIGIT
    ; Si es modo alarma, muestra HH:MM de la alarma
    cpi r20, EST_ALARM
    breq ALARM_DIGIT

HORA_DIGIT:                ; Muestra hora normal (HH:MM)
    cpi r22, 2            ; Dígitos 2-3: horas
    brlo USE_MINUTES      ; Dígitos 0-1: minutos
    lds r21, hours        ; Carga horas
    rjmp SPLIT_VAL
USE_MINUTES:
    lds r21, minutes      ; Carga minutos
    rjmp SPLIT_VAL

FECHA_DIGIT:               ; Muestra fecha (DD/MM)
    cpi r22, 2            ; Dígitos 2-3: día
    brlo USE_MONTH        ; Dígitos 0-1: mes
    lds r21, day          ; Carga día
    rjmp SPLIT_VAL
USE_MONTH:
    lds r21, month        ; Carga mes
    rjmp SPLIT_VAL

ALARM_DIGIT:               ; Muestra alarma (HH:MM)
    cpi r22, 2            ; Dígitos 2-3: horas
    brlo USE_ALARM_MIN
    lds r21, alarm_h      ; Carga hora de alarma
    rjmp SPLIT_VAL
USE_ALARM_MIN:
    lds r21, alarm_m      ; Carga minutos de alarma

SPLIT_VAL:                 ; Separa unidades/decenas
    andi r22, 1           ; ¿Es unidad (0) o decena (1)?
    brne DO_TENS          ; Si es decena, usa GET_TENS
    mov r20, r21          ; Copia valor (ej: 45)
    rcall GET_UNITS       ; Obtiene 5 (unidades)
    rjmp DIGIT_OK
DO_TENS:
    mov r20, r21          ; Copia valor (ej: 45)
    rcall GET_TENS        ; Obtiene 4 (decenas)
DIGIT_OK:
    rcall DIGIT_PATTERN   ; Convierte a patrón de 7 segmentos

    ; Agrega punto decimal si dp_state=1 (modo configuración)
    lds r16, dp_state
    tst r16
    breq NO_DP            ; Si dp_state=0, no lo muestra
    ori r19, 0x80         ; Enciende el punto decimal (bit 7)
NO_DP:

    pop r22               ; Recupera registros
    pop r21
    pop r20
    ret

;**********************************************************************
; GET_UNITS: Obtiene unidades de un número (ej: 45 ? 5)
;**********************************************************************
GET_UNITS:
    push r0
U_LOOP:
    cpi r20, 10           ; ¿Es menor a 10?
    brlo U_DONE          ; Sí: ya es la unidad
    subi r20, 10         ; Resta 10 hasta llegar a unidades
    rjmp U_LOOP
U_DONE:
    pop r0
    ret

;**********************************************************************
; GET_TENS: Obtiene decenas de un número (ej: 45 ? 4)
;**********************************************************************
GET_TENS:
    push r0
    clr r0               ; Contador de decenas
T_LOOP:
    cpi r20, 10          ; ¿Es mayor o igual a 10?
    brlo T_DONE          ; No: termina
    subi r20, 10         ; Resta 10 y aumenta contador
    inc r0
    rjmp T_LOOP
T_DONE:
    mov r20, r0          ; Retorna la decena (ej: 4)
    pop r0
    ret

;**********************************************************************
; DIGIT_PATTERN: Convierte número a patrón de 7 segmentos
;**********************************************************************
DIGIT_PATTERN:
    cpi r20, 10          ; ¿Es válido? (0-9)
    brlo DP_OK
    ldi r20, 0          ; Si no, muestra 0
DP_OK:
    ldi r31, high(DigitsTable*2) ; Dirección de la tabla
    ldi r30, low(DigitsTable*2)
    add r30, r20        ; Apunta al patrón del número
    brcc NO_INC_ZH      ; Ajusta puntero si hay acarreo
    inc r31
NO_INC_ZH:
    lpm r19, Z          ; Carga el patrón (ej: 0x3F para 0)
    ret

;**********************************************************************
; ALARM_CHECK: Verifica si debe activar la alarma
;**********************************************************************
ALARM_CHECK:
    push r16
    push r17

    ; Si la alarma fue silenciada y la hora no cambió, no activar
    lds r16, alarm_silenced
    tst r16
    breq ALCHK_NORMAL    ; No está silenciada: continua
    ; Verifica si la hora actual sigue siendo igual a la alarma
    lds r16, hours
    lds r17, alarm_h
    cp  r16, r17         ; ¿Horas iguales?
    brne CLEAR_SILENCED  ; No: reactiva alarma
    lds r16, minutes
    lds r17, alarm_m
    cp  r16, r17         ; ¿Minutos iguales?
    brne CLEAR_SILENCED  ; No: reactiva alarma
    rjmp ALCHK_EXIT      ; Sí: no hace nada

CLEAR_SILENCED:          ; Reactiva alarma si la hora cambió
    clr r16
    sts alarm_silenced, r16

ALCHK_NORMAL:
    ; ¿La alarma está activada? (alarm_enabled=1)
    lds r16, alarm_enabled
    tst r16                ; Si alarm_enabled=0, no hace nada
    breq ALCHK_EXIT

    ; ¿La alarma ya está sonando? (alarm_active=1)
    lds r16, alarm_active
    tst r16                ; Si ya suena, no la reactiva
    brne ALCHK_EXIT

    ; Compara hora actual con la alarma
    lds r16, hours         ; Carga horas actuales
    lds r17, alarm_h       ; Carga hora de alarma
    cp  r16, r17           ; ¿Coinciden las horas?
    brne ALCHK_EXIT        ; No: sale
    lds r16, minutes       ; Carga minutos actuales
    lds r17, alarm_m       ; Carga minutos de alarma
    cp  r16, r17           ; ¿Coinciden los minutos?
    brne ALCHK_EXIT        ; No: sale

    ; ¡Activar alarma!
    ldi r16, 1
    sts alarm_active, r16  ; Marca alarma como activa
    clr r16
    sts alarm_counter, r16 ; Reinicia contador de 60s

INC_DATE:
    push r16               ; Guarda registros
    push r17
    push r18

    lds r16, day           ; Día actual +1
    inc r16
    lds r17, month         ; Mes actual -1 (para índice)
    dec r17
    ldi r18, low(MonthDays*2) ; Apunta a la tabla de días por mes
    add r18, r17
    brcc SKIP_ADD_CARRY    ; Ajusta puntero si hay acarreo
SKIP_ADD_CARRY:
    ldi r31, high(MonthDays*2)
    mov r30, r18
    lpm r17, Z             ; Carga días máximos del mes (ej: 31 para enero)
    cp   r16, r17          ; ¿El día supera el máximo?
    brlo ST_DAY            ; No: guarda día
    breq ST_DAY            ; Si es igual (ej: 31), también avanza
    ldi r16, 1             ; Reinicia a día 1
ST_DAY:
    sts day, r16           ; Actualiza día
    tst r16                ; Si es 0 (error), fuerza a 1
    brne ID_EXIT
    ldi r16, 1
    sts day, r16
    lds r16, month         ; Mes actual +1
    inc r16
    cpi r16, 13            ; ¿Llegó a 13?
    brlo ST_MON            ; No: guarda
    ldi r16, 1             ; Reinicia a enero
ST_MON:
    sts month, r16         ; Actualiza mes

ID_EXIT:
    pop r18                ; Recupera registros
    pop r17
    pop r16
    ret

UPDATE_CLOCK:
    lds r18, seconds       ; Segundos +1
    inc r18
    cpi r18, 60            ; ¿Llegó a 60?
    brlo SEC_OK            ; No: sigue
    ldi r18, 0             ; Reinicia segundos
    lds r19, minutes       ; Minutos +1
    inc r19
    cpi r19, 60            ; ¿Llegó a 60?
    brlo MIN_OK            ; No: sigue
    ldi r19, 0             ; Reinicia minutos
    lds r16, hours         ; Horas +1
    inc r16
    cpi r16, 24            ; ¿Llegó a 24?
    brlo HRS_OK            ; No: sigue
    ldi r16, 0             ; Reinicia horas
    rcall INC_DATE         ; ¡Nuevo día!
HRS_OK:
    sts hours, r16         ; Guarda horas
MIN_OK:
    sts minutes, r19       ; Guarda minutos
SEC_OK:
    sts seconds, r18       ; Guarda segundos
    ret

UPDATE_LEDS:
    push r16               ; Guarda registros
    push r17
    in r16, SREG
    push r16

    cbi PORTB, PB5         ; Apaga todos los LEDs
    cbi PORTC, PC0
    cbi PORTC, PC1

    lds r17, current_state ; Lee estado actual

    ; Estado 0: Hora normal ? LEDs apagados
    cpi r17, 0
    brne SKIP_LED0
    rjmp LED_DONE
SKIP_LED0:

    ; Estado 1: Configurar minutos ? LED1 (PB5)
    cpi r17, 1
    brne SKIP_LED1
    rjmp LED_CONF_MIN
SKIP_LED1:

    ; Estado 2: Configurar horas ? LED2 (PC0)
    cpi r17, 2
    brne SKIP_LED2
    rjmp LED_CONF_HORA
SKIP_LED2:

    ; Estado 3: Fecha ? LED1 + LED2 (PB5 y PC0)
    cpi r17, 3
    brne SKIP_LED3
    rjmp LED_FECHA
SKIP_LED3:

    ; Estado 4: Configurar día ? LED3 (PC1)
    cpi r17, 4
    brne SKIP_LED4
    rjmp LED_CONF_DIA
SKIP_LED4:

    ; Estado 5: Configurar mes ? LED1 + LED3 (PB5 y PC1)
    cpi r17, 5
    brne SKIP_LED5
    rjmp LED_CONF_MES
SKIP_LED5:

    ; Estado 6: Alarma ? LED2 + LED3 (PC0 y PC1)
    cpi r17, 6
    brne SKIP_LED6
    rjmp LED_ALARMA
SKIP_LED6:
    rjmp LED_DONE

; Enciende LEDs según el estado
LED_CONF_MIN:              ; LED1 (PB5)
    sbi PORTB, PB5
    rjmp LED_DONE
LED_CONF_HORA:             ; LED2 (PC0)
    sbi PORTC, PC0
    rjmp LED_DONE
LED_FECHA:                 ; LED1 + LED2 (PB5 y PC0)
    sbi PORTB, PB5
    sbi PORTC, PC0
    rjmp LED_DONE
LED_CONF_DIA:              ; LED3 (PC1)
    sbi PORTC, PC1
    rjmp LED_DONE
LED_CONF_MES:              ; LED1 + LED3 (PB5 y PC1)
    sbi PORTB, PB5
    sbi PORTC, PC1
    rjmp LED_DONE
LED_ALARMA:                ; LED2 + LED3 (PC0 y PC1)
    sbi PORTC, PC0
    sbi PORTC, PC1
    rjmp LED_DONE

LED_DONE:
    pop r16                ; Recupera registros
    out SREG, r16
    pop r17
    pop r16
    ret
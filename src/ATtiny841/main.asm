; battery level check and low voltage shutdown
; by TerraForce

; include register names for ATtiny 841
 .include "tn841def.inc"	

 .equ vtest = 246 ; >= 8.6V --> enter test mode
 .equ v100 = 227  ; >= 8.0V --> A segment green, green frame
 .equ v80 = 216	  ; >= 7.6V --> B segmant green, green frame
 .equ v60 = 205   ; >= 7.2V --> C segment green, green frame
 .equ v40 = 200   ; >= 7.0V --> D segment green, green frame
 .equ v20 = 193   ; >= 6.8V --> E segment green, green frame
 .equ v0 = 188    ; >= 6.6V --> E segment red, green frame
                  ;  < 6.6V --> E segment red power off
 
.cseg
 ; stack at end of RAM
 ldi r16, LOW(RAMEND)		
 out spl, r16
 ldi r16, HIGH(RAMEND)
 out sph, r16

 ; disable all interrupts
 cli

 ; disable unused peripheral clock signals
 ldi r16, 0b11111110
 sts prr, r16

 ; set data directions
 sbi ddra, 0 ; A0 = out LED D
 sbi ddra, 1 ; A1 = out LED C
 sbi ddra, 2 ; A2 = out LED F (red)
 cbi ddra, 3 ; A3 = in (ADC)
 sbi ddra, 4 ; A4 = out LED A
 sbi ddra, 6 ; A6 = out LED B
 sbi ddra, 7 ; A7 = out LED F (green)
 sbi ddrb, 0 ; B0 = out LED E (red)
 sbi ddrb, 1 ; B1 = out LED E (green)
 sbi ddrb, 2 ; B2 = out power off (L)

 ; disable all LEDs (inverted!!!)
 sbi porta, 0 ; LED D
 sbi porta, 1 ; LED C
 sbi porta, 2 ; LED F (red)
 sbi porta, 4 ; LED A
 sbi porta, 6 ; LED B
 sbi porta, 7 ; LED F (green)
 sbi portb, 0 ; LED E (red)
 sbi portb, 1 ; LED E (green)

 ; enable power supply
 sbi portb, 2

 ; set ADC pin
 ldi r16, 0b00000011
 out admuxa, r16

 ; disable ADC pin digital input
 ldi r16, 0b00001000
 sts didr0, r16

 ; set ADC voltage reference to 4.096V and gain to 1
 ldi r16, 0b01100000
 out admuxb, r16

 ; set ADC to be free running and result to be left adjusted
 ldi r16, 0b00001000
 out adcsrb, r16

 ; enable ADC, start free running conversions, enable interrupts and set clock division factor to 16 (500 kHz on 8 MHz CPU clock)
 ldi r16, 0b11111100
 out adcsra, r16

 ; wait for first ADC result
 wait:
 sbis adcsra, 4
 rjmp wait

 ; start of main loop
 loop:
 
 ; read ADC conversion result
 in r16, adch

 ; detect test mode
 cpi r16, vtest
 brsh test

 ; enable green LEDs (and disable red LED)
 cpi r16, v100
 brsh leda
 sbi porta, 4

 cpi r16, v80
 brsh ledb
 sbi porta, 6

 cpi r16, v60
 brsh ledc
 sbi porta, 1

 cpi r16, v40
 brsh ledd
 sbi porta, 0

 cpi r16, v20
 brsh ledeg
 sbi portb, 1
 cbi portb, 0

 cpi r16, v0
 brsh ledfg
 sbi porta, 7

 ; shutdown voltage converters
 cbi portb, 2

 ; disable ADC
 ldi r16, 0b11110100
 out adcsra, r16

 ; shutdown ADC
 ldi r16, 0b00000000
 sts prr, r16
 
 ; wait forever
 end:
 nop
 rjmp end

 ; set green LEDs (and clear red)
 leda:
 cbi porta, 4
 ledb:
 cbi porta, 6
 ledc:
 cbi porta, 1
 ledd:
 cbi porta, 0
 ledeg:
 cbi portb, 1
 sbi portb, 0
 ledfg:
 cbi porta, 7

 ; jump back to loop start
 rjmp loop

 ; start of test mode (showing conversion result in binary)
 test:

 ; copy port output states
 in r17, porta
 in r18, portb

 ; set LED F (green) to bit 7
 bst r16, 7
 bld r17, 7

 ; set LED A to bit 6
 bst r16, 6
 bld r17, 4

 ; set LED B to bit 5
 bst r16, 5
 bld r17, 6

 ; set LED C to bit 4
 bst r16, 4
 bld r17, 1

 ; set LED D to bit 3
 bst r16, 3
 bld r17, 0

 ; set LED E (green) to bit 2
 bst r16, 2
 bld r18, 1

 ; set LED E (red) to bit 1
 bst r16, 1
 bld r18, 0

 ; set LED F (red) to bit 0
 bst r16, 0
 bld r17, 2

 ; apply new port output states
 out porta, r17
 out portb, r18

 ; read ADC conversion result
 in r16, adch

 ; jump back to test mode start
 rjmp test
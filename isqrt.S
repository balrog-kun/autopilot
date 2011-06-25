;-----------------------------------------------------------------------------;
; Fast integer squareroot routines for avr-gcc project          (C)ChaN, 2008
; http://elm-chan.org/docs/avrlib/sqrt32.S
;-----------------------------------------------------------------------------;
; uint16_t isqrt32 (uint32_t n);
; uint8_t isqrt16 (uint16_t n);
; uint16_t ihypot (int16_t x, int16_t y);

;-----------------------------------------------------------------------------:
; 32bit integer squareroot
;-----------------------------------------------------------------------------;
;   uint16_t isqrt32 (
;     uint32_t n
;   );
;
; Return Value:
;   Squareroot of n.
;
; Size  = 53 words
; Clock = 532..548 cycles
; Stack = 0 byte

.global isqrt32
.func isqrt32

isqrt32:
	clr	r0
	clr	r18
	clr	r19
	clr	r20
	ldi	r21, 1
	clr	r27
	clr	r30
	clr	r31
	ldi	r26, 16
1:	lsl	r22
	rol	r23
	rol	r24
	rol	r25
	rol	r0
	rol	r18
	rol	r19
	rol	r20
	lsl	r22
	rol	r23
	rol	r24
	rol	r25
	rol	r0
	rol	r18
	rol	r19
	rol	r20
	brpl	2f
	add	r0, r21
	adc	r18, r27
	adc	r19, r30
	adc	r20, r31
	rjmp	3f
2:	sub	r0, r21
	sbc	r18, r27
	sbc	r19, r30
	sbc	r20, r31
3:	lsl	r21
	rol	r27
	rol	r30
	andi	r21, 0b11111000
	ori	r21, 0b00000101
	sbrc	r20, 7
	subi	r21, 2
	dec	r26
	brne	1b
	lsr	r30
	ror	r27
	ror	r21
	lsr	r30
	ror	r27
	ror	r21
	mov	r24, r21
	mov	r25, r27
	ret
.endfunc



;-----------------------------------------------------------------------------:
; 16bit integer squareroot
;-----------------------------------------------------------------------------;
;   uint8_t isqrt16 (
;     uint16_t n
;   );
;
; Return Value:
;   Squareroot of n.
;
; Size  = 33 words
; Clock = 181..189 cycles
; Stack = 0 byte

.global isqrt16
.func isqrt16

isqrt16:
	clr	r18
	clr	r19
	ldi	r20, 1
	clr	r21
	ldi	r22, 8
1:	lsl	r24
	rol	r25
	rol	r18
	rol	r19
	lsl	r24
	rol	r25
	rol	r18
	rol	r19
	brpl	2f
	add	r18, r20
	adc	r19, r21
	rjmp	3f
2:	sub	r18, r20
	sbc	r19, r21
3:	lsl	r20
	rol	r21
	andi	r20, 0b11111000
	ori	r20, 0b00000101
	sbrc	r19, 7
	subi	r20, 2
	dec	r22
	brne	1b
	lsr	r21
	ror	r20
	lsr	r21
	ror	r20
	mov	r24, r20
	ret
.endfunc



;-----------------------------------------------------------------------------:
; 16bit integer hypot (megaAVR is required)
;-----------------------------------------------------------------------------;
;   uint16_t ihypot (
;     int16_t x,
;     int16_t y
;   );
;
; Return Value:
;   Squareroot of (x*x + y*y)
;
; Size  = 42 words
; Clock = 581..597 cycles
; Stack = 0 byte

.global ihypot
.func ihypot

ihypot:
	clr	r26
	sbrs	r25, 7
	rjmp	1f
	com	r24
	com	r25
	adc	r24, r26
	adc	r25, r26
1:	sbrs	r23, 7
	rjmp	2f
	com	r22
	com	r23
	adc	r22, r26
	adc	r23, r26
2:	mul	r22, r22
	movw	r18, r0
	mul	r23, r23
	movw	r20, r0
	mul	r22, r23
	add	r19, r0
	adc	r20, r1
	adc	r21, r26
	add	r19, r0
	adc	r20, r1
	adc	r21, r26
	mul	r24, r24
	movw	r30, r0
	mul	r25, r25
	add	r18, r30
	adc	r19, r31
	adc	r20, r0
	adc	r21, r1
	mul	r24, r25
	add	r19, r0
	adc	r20, r1
	adc	r21, r26
	add	r19, r0
	adc	r20, r1
	adc	r21, r26
	movw	r24, r20
	movw	r22, r18
	clr	r1
	rjmp	isqrt32
.endfunc





// This is a replacement firmware for the Corona RP8D1 RC receiver.
//
// It has only been used on the 8-channel 35MHz version (RP8D1). If you want to use it
// on other frequency versions of the receiver then you need to modify the
// CH_STEP, CH0_FREQ, START_CHAN, END_CHAN #defines accordingly.
//
// Their are NO guarantees or warranties what so ever with this code, use at your own risk!

#include <ioavr.h>

#include "pins.h"

//#define		poly16 0x1021
#define		poly16 0x8005

//#define	poly32 0x04C11DB7
#define		poly32 0xEDB88320

PUBLIC		updateCrc8

PUBLIC		updateCrc16, calcBufCrc16
//PUBLIC		updateCrc32

RSEG	CODE
/*
; X8 + X5 + X4 + 1 .. 0x131
; http://pdfserv.maxim-ic.com/en/an/AN27.pdf
; crc = crcTable8[crc ^ b];
crcTable8:	db 0, 94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65
				db 157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220
				db 35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98
				db 190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255
				db 70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7
				db 219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154
				db 101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36
				db 248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185
				db 140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205
				db 17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80
				db 175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238
				db 50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115
				db 202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139
				db 87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22
				db 233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168
				db 116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53
*/
; X8 + X2 + X + 1 .. 0x107
; http://www.maxim-ic.com/appnotes.cfm/an_pk/3749 (DS1862 chip)
; crc = crcTable8[crc ^ b];
crcTable8:	db 0x00,0x07,0x0e,0x09,0x1c,0x1b,0x12,0x15,0x38,0x3f,0x36,0x31,0x24,0x23,0x2a,0x2d
				db 0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D
				db 0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD	
				db 0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD
				db 0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA
				db 0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A
				db 0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A
				db 0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A
				db 0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4
				db 0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4
				db 0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44
				db 0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34
				db 0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63
				db 0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13
				db 0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83
				db 0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3

crcTable16:	dw 0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf
				dw 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7
				dw 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e
				dw 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876
				dw 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd
				dw 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5
				dw 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c
				dw 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974
				dw 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb
				dw 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3
				dw 0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a
				dw 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72
				dw 0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9
				dw 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1
				dw 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738
				dw 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70
				dw 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7
				dw 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff
				dw 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036
				dw 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e
				dw 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5
				dw 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd
				dw 0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134
				dw 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c
				dw 0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3
				dw 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb
				dw 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232
				dw 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a
				dw 0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1
				dw 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9
				dw 0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330
				dw 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78

;*************************************
; uint8_t updateCrc8(uint8_t crc, uint8_t ch);

; crc = r16
; ch = r17

updateCrc8:		//	fast table method
	eor	r16,r17							;1
	clr	r17								;1
	ldi	ZL,low(crcTable8)				;1
	ldi	ZH,high(crcTable8)			;1
	add	ZL,r16							;1
	adc	ZH,r17							;1
	lpm	r16,Z								;3
	ret										;4
	
;*************************************
; uint16_t updateCrc16(uint16_t crc, uint8_t ch);

; crc = r16:r17
; ch = r18

updateCrc16:		//	fast table method
	eor	r16,r18							;1   crc = (crc >> 8) ^ crcTable16[(crc ^ b) & 0xff];
	clr	r18								;1
	lsl	r16								;1
	rol	r18								;1
	ldi	ZL,low(crcTable16)			;1
	ldi	ZH,high(crcTable16)			;1
	add	ZL,r16							;1
	adc	ZH,r18							;1
	mov	r16,r17							;1
	lpm	r17,Z+							;3
	eor	r16,r17							;1
	lpm	r17,Z								;3
	ret										;4
	
// *************************************
; uint16_t calcBufCrc16(uint16_t crc, void *buf, uint8_t length)
;
; crc = r16:r17
; buf = r18:r19
; length = r20

calcBufCrc16:	//	fast table method
	tst	r20								;1
	breq	calcBufCrc16b					;1/2 no data

	mov	r22,XH							;1   save XH:XL
	mov	r21,XL							;1
	
	movw	XH:XL,r19:r18					;2   buffer pointer

calcBufCrc16a:
	ld		r18,X+							;2   fetch buffer byte
	
	eor	r16,r18							;1   crc = (crc >> 8) ^ crcTable16[(crc ^ b) & 0xff];
	clr	r18								;1
	lsl	r16								;1
	rol	r18								;1
	ldi	ZL,low(crcTable16)			;1
	ldi	ZH,high(crcTable16)			;1
	add	ZL,r16							;1
	adc	ZH,r18							;1
	mov	r16,r17							;1
	lpm	r17,Z+							;3
	eor	r16,r17							;1
	lpm	r17,Z								;3

	dec	r20								;1
	brne	calcBufCrc16a					;1/2 next byte
	
	mov	XL,r21							;1   restore XH:XL
	mov	XH,r22							;1
	
calcBufCrc16b:
	ret										;4

// *************************************
; uint32_t updateCrc32(uint32_t crc, uint8_t b);
/*
updateCrc32:
	ldi		r21,byte1(poly32)			;1
	ldi		r22,byte2(poly32)			;1
	ldi		r23,byte3(poly32)			;1
	ldi		r24,byte4(poly32)			;1

	eor		r16,r20						;1

	ldi		r20,8							;1
updateCrc32a:
	lsr		r19							;1
	ror		r18							;1
	ror		r17							;1
	ror		r16							;1
	brcc		updateCrc32b				;1/2
	eor		r16,r21						;1
	eor		r17,r22						;1
	eor		r18,r23						;1
	eor		r19,r24						;1
updateCrc32b:
	dec		r20							;1
	brne		updateCrc32a				;1/2
	
	ret										;4
*/
END
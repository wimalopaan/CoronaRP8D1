
// This is a replacement firmware for the Corona RP8D1 RC receiver.
//
// It has only been used on the 8-channel 35MHz version (RP8D1). If you want to use it
// on other frequency versions of the receiver then you need to modify the
// CH_STEP, CH0_FREQ, START_CHAN, END_CHAN #defines accordingly.
//
// Their are NO guarantees or warranties what so ever with this code, use at your own risk!

#ifndef _TYPES_H_
#define _TYPES_H_

//#include <stdint.h>

typedef unsigned char		uint8_t;
typedef signed char			int8_t;
typedef unsigned int		uint16_t;
typedef signed int			int16_t;
typedef unsigned long		uint32_t;
typedef signed long			int32_t;
typedef unsigned char		bool;

#define true	1
#define false	0

#define null	0

#endif

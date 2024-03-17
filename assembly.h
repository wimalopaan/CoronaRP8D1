
// This is a replacement firmware for the Corona RP8D1 RC receiver.
//
// It has only been used on the 8-channel 35MHz version (RP8D1). If you want to use it
// on other frequency versions of the receiver then you need to modify the
// CH_STEP, CH0_FREQ, START_CHAN, END_CHAN #defines accordingly.
//
// Their are NO guarantees or warranties what so ever with this code, use at your own risk!

#ifndef _ASSEMBLY_H_
#define _ASSEMBLY_H_

#include "types.h"

extern uint8_t updateCrc8(uint8_t crc, uint8_t ch);

extern uint16_t updateCrc16(uint16_t crc, uint8_t ch);
extern uint16_t calcBufCrc16(uint16_t crc, void *buf, uint8_t length);

//extern uint32_t updateCrc32(uint32_t crc, uint8_t ch);

#endif


// This is a replacement firmware for the Corona RP8D1 RC receiver.
//
// It has only been used on the 8-channel 35MHz version (RP8D1). If you want to use it
// on other frequency versions of the receiver then you need to modify the
// CH_STEP, CH0_FREQ, START_CHAN, END_CHAN #defines accordingly.
//
// Their are NO guarantees or warranties what so ever with this code, use at your own risk!

#ifndef _MAIN_H_
#define _MAIN_H_

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// #include <ioavr.h>
// #include <inavr.h>
// #include <intrinsics.h>

#include "types.h"

//*************************************

#define low(word_reg)				((uint8_t)(word_reg))
#define high(word_reg)				((uint8_t)(word_reg >> 8))

#define byte1(dword_reg)			((uint8_t)(dword_reg))
#define byte2(dword_reg)			((uint8_t)(dword_reg >> 8))
#define byte3(dword_reg)			((uint8_t)(dword_reg >> 16))
#define byte4(dword_reg)			((uint8_t)(dword_reg >> 24))

//*************************************

#define FOSC						8000000ul	// cpu clock frequency (Hz)

//*************************************

#define UART_BAUDRATE				19200
//#define UART_BAUDRATE				38400
//#define UART_BAUDRATE				57600
//#define UART_BAUDRATE				76800
//#define UART_BAUDRATE				115200
//#define UART_BAUDRATE				230400
//#define UART_BAUDRATE				460800
//#define UART_BAUDRATE				921600

#define UART_RX_BUFFER_SIZE			32

//*************************************

enum {
	state_normal = 0,
	state_but_release,
	state_scan_option,
	state_disable_failsafe_option,
	state_enable_failsafe_option,
	state_enable_pwm_option,
	state_enable_ppm_option,
	state_defaults_option,
	state_last								// always end with 'state_last'
} t_state_option;

//*************************************

#define MIN_PWM_CHANNELS			2			//
#define MAX_PWM_CHANNELS			10			// although we only have 8 PWM individual outputs, we can still do as many channels as we like in PPM output mode

#define MIN_PPM_FRAME_LENGTH		((MIN_PWM_CHANNELS * 2000) + 8000)	// microseconds
#define MAX_PPM_FRAME_LENGTH		((MAX_PWM_CHANNELS * 2000) + 8000)	// microseconds

#define MIN_SYNC_WIDTH				4000		// microseconds
#define MAX_SYNC_WIDTH				24000		// microseconds

#define PPM_HIGH_WIDTH				450			// microseconds

#define MIN_PWM_WIDTH				800			// microseconds
#define MAX_PWM_WIDTH				2200		// microseconds

#define INTER_PWM_GAP				100			// microseconds

//*************************************



// UK 35MHz
// #define PLL_VCO_OFFSET		10700000ul	// Hz
// #define CH_STEP				   10000	// Hz .. 10kHz channel spacing
// #define CH0_FREQ			34400000ul	// Hz
// #define START_CHAN			      55	// UK min chan
// #define END_CHAN			      90	// UK max chan

// UK 40MHz
#define PLL_VCO_OFFSET		10700000ul	// Hz
#define CH_STEP				   10000	// Hz .. 10kHz channel spacing
#define CH0_FREQ			34015000ul	// Hz
#define START_CHAN			     665	// UK min chan
#define END_CHAN			     995	// UK max chan

// Aussie 36MHz
//#define PLL_VCO_OFFSET		10700000ul	// Hz
//#define CH_STEP				   10000	// Hz .. 10kHz channel spacing
//#define CH0_FREQ			30000000ul	// Hz
//#define START_CHAN			     601	// Aus min chan
//#define END_CHAN			     659	// Aus max chan

// NZ 35MHz
//#define PLL_VCO_OFFSET		10700000ul	// Hz
//#define CH_STEP				   10000	// Hz .. 10kHz channel spacing
//#define CH0_FREQ			30000000ul	// Hz
//#define START_CHAN			     500	// NZ min chan
//#define END_CHAN			     530	// NZ max chan

// NZ 36MHz
//#define PLL_VCO_OFFSET		10700000ul	// Hz
//#define CH_STEP				   10000	// Hz .. 10kHz channel spacing
//#define CH0_FREQ			30000000ul	// Hz
//#define START_CHAN			     625	// NZ min chan
//#define END_CHAN			     659	// NZ max chan

// NZ 40MHz
//#define PLL_VCO_OFFSET		10700000ul	// Hz
//#define CH_STEP				   10000	// Hz .. 10kHz channel spacing
//#define CH0_FREQ			36000000ul	// Hz
//#define START_CHAN			     451	// NZ min chan
//#define END_CHAN			     493	// NZ max chan





#define PLL_REF_XTAL		11155000ul	// Hz
#define PLL_REF_FREQ		    5000	// Hz

#define PLL_PT				(1ul << 16)
#define PLL_PR				(1ul << 16)
#define PLL_TON				(1ul << 17)
#define PLL_RON				(1ul << 17)
#define PLL_OFF				(1ul << 18)
#define PLL_PS				(1ul << 19)
#define PLL_T0				(1ul << 20)
#define PLL_T1				(1ul << 21)
#define PLL_ID0				(1ul << 22)
#define PLL_ID1				(1ul << 23)

#define PLL_LD0				(1ul << 18)
#define PLL_LD1				(1ul << 19)

// ************************************

#pragma pack(1)

typedef struct
{
	uint8_t  bad_byte;									// the 1st byte of eeprom is a none problem
	uint16_t rf_channel;								// the receivers RF channel number
	uint8_t  pwm_channels;								// the number of channels found in the PPM stream of the RC TX we bound too
	bool     pwm_out_mode;								// false = PPM output mode, true = PWM output mode
	bool     failsafe_enabled;							// false = no fail-safe. true = fail-safe allowed
	uint16_t failsafe_pwm[MAX_PWM_CHANNELS];			// microseconds
	uint16_t ppm_frame_length;							// microseconds
} T_EEPROM;

typedef struct
{
	uint16_t marker;
	uint16_t rf_channel;
	uint8_t  flags;
	uint8_t  pwm_channels;
	uint16_t pwm[MAX_PWM_CHANNELS];
	uint16_t crc;
} T_PACKET;

#pragma pack()

//*************************************

//extern __eeprom uint16_t ee_pwm_in_width[MAX_PWM_CHANNELS];

//*************************************

#endif

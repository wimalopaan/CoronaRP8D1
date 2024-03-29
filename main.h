/*
 * Copyright (C) 2024 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef F_CPU
# define F_CPU 8000000ul	// cpu clock frequency (Hz)
#endif

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>

// #define USART_DEBUG // anble debug messages via UART (then ch7 and ch8 are not usable)
#define MAX_FILTER_CHANNEL 4 // Channels above are not filtered
#define SWITCH_CHANNELS_FROM 4 // use chx (x >= SWITCH_CHANNLES_FROM) as switching outputs for multi-switch protocols
#define SWITCH_CHANNELS_DEFAULT false
#define MULTI_SWITCHES 8 // number of switches decoded
#define MULTI_SWITCH_CHANNEL 7 // incoming channel 8 use for multi-channel
// tbd
#define SENDER_ID_CHANNEL 8 // Channel 9 sould contain sender ID (8-bit time multiplex (4 frames, each 2 bits)
#define SBUS_OUT // sbus output on serial port

#define UART_BAUDRATE				19200
//#define UART_BAUDRATE				38400
//#define UART_BAUDRATE				57600
//#define UART_BAUDRATE				76800
//#define UART_BAUDRATE				115200
//#define UART_BAUDRATE				230400
//#define UART_BAUDRATE				460800
//#define UART_BAUDRATE				921600

#define UART_RX_BUFFER_SIZE			32

enum {
	state_normal = 0,
	state_but_release,
    state_scan_option, // 1 blink
    state_enable_filter_option, // 2 blink
    state_disable_filter_option, // 3 blink
    state_enable_switch_option, // 4
    state_disable_switch_option, // 5
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

#define MIN_FILTER_VALUE		    16						// no filtering
#define MAX_FILTER_VALUE		    (MIN_FILTER_VALUE * 80)	// max filtering

#define MAX_DIFF                    75
#define MIN_DIFF                    2

#define MULTI_SYNC_WIDTH            2050
#define MULTI_SYNC_HIGH             1750
#define MULTI_SYNC_LOW              1250

// UK 35MHz
// #define PLL_VCO_OFFSET		10700000ul	// Hz
// #define CH_STEP				   10000	// Hz .. 10kHz channel spacing
// #define CH0_FREQ			34400000ul	// Hz
// #define START_CHAN			      55	// UK min chan
// #define END_CHAN			      90	// UK max chan

// // UK 40MHz
// #define PLL_VCO_OFFSET		10700000ul	// Hz
// #define CH_STEP				   10000	// Hz .. 10kHz channel spacing
// #define CH0_FREQ			34015000ul	// Hz
// #define START_CHAN			     665	// UK min chan 40.665 MHz
// #define END_CHAN			     995	// UK max chan 43.965 MHz

// D 40MHz
#define PLL_VCO_OFFSET		10700000ul	// Hz
#define CH_STEP				   10000	// Hz .. 10kHz channel spacing
#define CH0_FREQ			34015000ul	// Hz
#define START_CHAN			     665	// UK min chan 40.665 MHz
#define END_CHAN			     697	// UK max chan 40.985 MHz

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

#define EEPROM_VERSION 44

typedef struct
{
    uint8_t  bad_byte_dont_use;  						// the 1st byte of eeprom is a none problem
    uint8_t  magic_number;
	uint16_t rf_channel;								// the receivers RF channel number
	uint8_t  pwm_channels;								// the number of channels found in the PPM stream of the RC TX we bound too
	bool     pwm_out_mode;								// false = PPM output mode, true = PWM output mode
	bool     failsafe_enabled;							// false = no fail-safe. true = fail-safe allowed
	uint16_t failsafe_pwm[MAX_PWM_CHANNELS];			// microseconds
	uint16_t ppm_frame_length;							// microseconds
    bool     filter_high_channels;                      // enable channels >= MAX_FILTER_CHANNEL
    bool     switchChannels;                            // use ch4-8 as switching channels / multi-channel
    uint8_t  multiSwitchChannel;                        // rc-channel to use for switching
    // not implemented yet
    bool     enable_sender_id;                          // enable sender identification (channel SENDER_ID_CHANNEL)
    uint8_t  sender_id;                                 // enable sender identification (channel SENDER_ID_CHANNEL)
} T_EEPROM;

#pragma pack()


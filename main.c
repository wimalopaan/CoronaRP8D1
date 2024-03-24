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

// This is a replacement firmware for the Corona RP8D1 RC receiver.
//
// It has only been used on the 8-channel 40MHz version (RP8D1). If you want to use it
// on other frequency versions of the receiver then you need to modify the
// CH_STEP, CH0_FREQ, START_CHAN, END_CHAN & PLL_VCO_OFFSET #defines accordingly.
//

#include <stdlib.h>		// for abs()

#include "main.h"
#include "pins.h"

uint8_t state;

volatile bool scanning;

uint8_t button_pressed_counter;
uint8_t button_released_counter;

uint8_t last_ppm_frame_received_timer;

uint16_t prev_icr1;

// values read from eeprom
volatile T_EEPROM ee;
T_EEPROM eeprom EEMEM; // generates valid eeprom-offset address

uint16_t pwm_in[MAX_PWM_CHANNELS];					// microseconds
uint16_t prev_pwm_in[MAX_PWM_CHANNELS];				// microseconds
int8_t pwm_in_index;
int8_t pwm_in_prev_chans;
uint8_t pwm_in_bad_pulses;
uint16_t average_diff;

// read/write from ISR
// attention: atomic rw, but non-atomic rmw
volatile bool failsafe_mode;
volatile uint8_t pwm_in_frames;
volatile uint8_t new_pwm_in_chans;

volatile uint8_t switchValues[MULTI_SWITCHES]; // multi-switch: switch 0-3, only on/off

// attention: non-atomic rmw, use ATOMIC_BLOCK outside ISR
volatile uint16_t new_pwm_in[MAX_PWM_CHANNELS];		// microseconds

uint16_t pwm_out[MAX_PWM_CHANNELS];					// microseconds
uint16_t pwm_out_total;								// microseconds
uint8_t pwm_out_index;

uint32_t rx_freq;									// Hz

uint16_t msCounter;

uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t uart_rx_buffer_wr;
uint8_t uart_rx_buffer_rd;

// ************************************

void stopUART(void);
void startUART(void);
void uartTxByteWait(uint8_t b);

// ************************************
// revert all settings back to defaults

void setDefaultSettings(void) {
#ifdef USART_DEBUG
    uartTxByteWait(0x01);
#endif
    ee.magic_number = EEPROM_VERSION;
    ee.rf_channel = START_CHAN;
    ee.pwm_channels = 0;
#ifdef USART_DEBUG
    ee.pwm_out_mode = false;
#else
    ee.pwm_out_mode = true;
#endif
    ee.failsafe_enabled = false;
    for (int8_t i = 0; i < MAX_PWM_CHANNELS; i++) {
        ee.failsafe_pwm[i] = 1500;
    }
    ee.ppm_frame_length = 20000;

    ee.filter_high_channels = false;

    ee.switchChannels = SWITCH_CHANNELS_DEFAULT;
    ee.multiSwitchChannel = MULTI_SWITCH_CHANNEL;

    ee.enable_sender_id = false;
    ee.sender_id = 0;

    // write the settings back to eeprom
    eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
    eeprom_busy_wait();

    stopUART();
}

// ************************************
// set all the PWM channel outputs LOW

void allOutputsLow(void) {
    CHAN_1_OUT_PORT &= ~(1 << CHAN_1_OUT_BIT);		// pin LOW
    CHAN_2_OUT_PORT &= ~(1 << CHAN_2_OUT_BIT);		// pin LOW
    CHAN_3_OUT_PORT &= ~(1 << CHAN_3_OUT_BIT);		// pin LOW
    CHAN_4_OUT_PORT &= ~(1 << CHAN_4_OUT_BIT);		// pin LOW
    CHAN_5_OUT_PORT &= ~(1 << CHAN_5_OUT_BIT);		// pin LOW
    CHAN_6_OUT_PORT &= ~(1 << CHAN_6_OUT_BIT);		// pin LOW
    if (ee.pwm_out_mode) {	// the UART is not using the pins
        CHAN_7_OUT_PORT &= ~(1 << CHAN_7_OUT_BIT);	// pin LOW
        CHAN_8_OUT_PORT &= ~(1 << CHAN_8_OUT_BIT);	// pin LOW
    }
}

// ************************************
// copy the fail-safe pwm values to the outputs and enable sail-safe mode

void useFailSafeValues(void) {
    uint8_t sreg = SREG;
    cli();
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        pwm_out[i] = ee.failsafe_pwm[i];
    }
    failsafe_mode = true;								// we are in fail-safe mode
    pwm_in_frames = 0;
    average_diff = 1000;
    SREG = sreg;
}

// ************************************
// feed the doggy

bool feedWatchdog(void) {
    if (WDTCSR & (1 << WDIE))
        return false;					// watchdog hasn't yet timed out

    // watchdog needs resetting
    uint8_t sreg = SREG;
    cli();
    wdt_reset();
    WDTCSR |= 1 << WDIE;			// re-enable interrupt
    SREG = sreg;

    return true;
}

// ************************************
// This generates the output PPM stream or the PWM output pulses
//
// This should be the ONLY interrupt in the whole system - this is to ensure
// nothing else upsets the output of PPM/PWM waveforms (timing is critical).

ISR(TIMER1_COMPA_vect) {
    if (
        (failsafe_mode && !ee.failsafe_enabled) || // we are in fail-safe mode but fail-safe outputs are disabled
        (ee.pwm_channels < MIN_PWM_CHANNELS) ||	   // we have not been bound to an RC Tx
        (ee.ppm_frame_length < MIN_PPM_FRAME_LENGTH && !ee.pwm_out_mode)	// we are in PPM output mode but the PPM frame length has not been set
        ) {
        allOutputsLow();								// set all PWM outputs LOW
        OCR1A = (uint16_t)OCR1A + 1000;					// come back here in 1ms
        pwm_out_index = 0;								// back to channel-1
        pwm_out_total = ee.ppm_frame_length;			//
    }
    else {
        if (!ee.pwm_out_mode) {	// PPM output mode
            if (pwm_out_index >= ((ee.pwm_channels + 1) << 1)) {
                pwm_out_index = 0;
                pwm_out_total = ee.ppm_frame_length;
            }

            if (!(pwm_out_index & 1)) {	// HIGH pulse
                CHAN_1_OUT_PORT |= 1 << CHAN_1_OUT_BIT;		// pin HIGH
                uint16_t pulse_width = PPM_HIGH_WIDTH;
                OCR1A = (uint16_t)OCR1A + pulse_width;
                pwm_out_total -= pulse_width;
                pwm_out_index++;
            }
            else {	// LOW pulse
                CHAN_1_OUT_PORT &= ~(1 << CHAN_1_OUT_BIT);	// pin LOW

                if (pwm_out_index >= (ee.pwm_channels << 1)) {	// SYNC pulse
                    OCR1A = (uint16_t)OCR1A + pwm_out_total;
                    pwm_out_index = 0;						// back to channel-1
                    pwm_out_total = ee.ppm_frame_length;
                }
                else {	// CHANNEL pulse
                    uint16_t pulse_width = pwm_out[pwm_out_index >> 1];
                    if (pulse_width < MIN_PWM_WIDTH) pulse_width = MIN_PWM_WIDTH;
                    else
                        if (pulse_width > MAX_PWM_WIDTH) pulse_width = MAX_PWM_WIDTH;
                    pulse_width -= PPM_HIGH_WIDTH;
                    OCR1A = (uint16_t)OCR1A + pulse_width;
                    pwm_out_total -= pulse_width;
                    pwm_out_index++;
                }
            }
        }
        else {	// PWM output mode
            if (pwm_out_index >= (ee.pwm_channels << 1))
                pwm_out_index = 0;

            const int chan = pwm_out_index >> 1;

            if (!(pwm_out_index & 1)) {	// Start of channel pulse
                if (ee.switchChannels && (chan >= SWITCH_CHANNELS_FROM)) {
                    const uint8_t sw = switchValues[chan - SWITCH_CHANNELS_FROM];
                    if (sw == 2) {
                        switch (chan) {
                        case 0: CHAN_1_OUT_PORT |= 1 << CHAN_1_OUT_BIT; break;	// pin HIGH
                        case 1: CHAN_2_OUT_PORT |= 1 << CHAN_2_OUT_BIT; break;	// pin HIGH
                        case 2: CHAN_3_OUT_PORT |= 1 << CHAN_3_OUT_BIT; break;	// pin HIGH
                        case 3: CHAN_4_OUT_PORT |= 1 << CHAN_4_OUT_BIT; break;	// pin HIGH
                        case 4: CHAN_5_OUT_PORT |= 1 << CHAN_5_OUT_BIT; break;	// pin HIGH
                        case 5: CHAN_6_OUT_PORT |= 1 << CHAN_6_OUT_BIT; break;	// pin HIGH
                        case 6: CHAN_7_OUT_PORT |= 1 << CHAN_7_OUT_BIT; break;	// pin HIGH
                        case 7: CHAN_8_OUT_PORT |= 1 << CHAN_8_OUT_BIT; break;	// pin HIGH
                        }
                    }
                    else {
                        switch (chan) {
                        case 0: CHAN_1_OUT_PORT &= ~(1 << CHAN_1_OUT_BIT); break;	// pin LOW
                        case 1: CHAN_2_OUT_PORT &= ~(1 << CHAN_2_OUT_BIT); break;	// pin LOW
                        case 2: CHAN_3_OUT_PORT &= ~(1 << CHAN_3_OUT_BIT); break;	// pin LOW
                        case 3: CHAN_4_OUT_PORT &= ~(1 << CHAN_4_OUT_BIT); break;	// pin LOW
                        case 4: CHAN_5_OUT_PORT &= ~(1 << CHAN_5_OUT_BIT); break;	// pin LOW
                        case 5: CHAN_6_OUT_PORT &= ~(1 << CHAN_6_OUT_BIT); break;	// pin LOW
                        case 6: CHAN_7_OUT_PORT &= ~(1 << CHAN_7_OUT_BIT); break;	// pin LOW
                        case 7: CHAN_8_OUT_PORT &= ~(1 << CHAN_8_OUT_BIT); break;	// pin LOW
                        }
                    }
                }
                else {
                    switch (chan) {
                    case 0: CHAN_1_OUT_PORT |= 1 << CHAN_1_OUT_BIT; break;	// pin HIGH
                    case 1: CHAN_2_OUT_PORT |= 1 << CHAN_2_OUT_BIT; break;	// pin HIGH
                    case 2: CHAN_3_OUT_PORT |= 1 << CHAN_3_OUT_BIT; break;	// pin HIGH
                    case 3: CHAN_4_OUT_PORT |= 1 << CHAN_4_OUT_BIT; break;	// pin HIGH
                    case 4: CHAN_5_OUT_PORT |= 1 << CHAN_5_OUT_BIT; break;	// pin HIGH
                    case 5: CHAN_6_OUT_PORT |= 1 << CHAN_6_OUT_BIT; break;	// pin HIGH
                    case 6: CHAN_7_OUT_PORT |= 1 << CHAN_7_OUT_BIT; break;	// pin HIGH
                    case 7: CHAN_8_OUT_PORT |= 1 << CHAN_8_OUT_BIT; break;	// pin HIGH
                    }
                }
                uint16_t pulse_width = pwm_out[chan];
                if (pulse_width < MIN_PWM_WIDTH) pulse_width = MIN_PWM_WIDTH;
                else
                    if (pulse_width > MAX_PWM_WIDTH) pulse_width = MAX_PWM_WIDTH;
                OCR1A = (uint16_t)OCR1A + pulse_width;
                pwm_out_index++;
            }
            else {	// inter pulse gap
                if (ee.switchChannels && (chan >= SWITCH_CHANNELS_FROM)) {

                }
                else {
                    switch (chan) {
                    case 0: CHAN_1_OUT_PORT &= ~(1 << CHAN_1_OUT_BIT); break;	// pin LOW
                    case 1: CHAN_2_OUT_PORT &= ~(1 << CHAN_2_OUT_BIT); break;	// pin LOW
                    case 2: CHAN_3_OUT_PORT &= ~(1 << CHAN_3_OUT_BIT); break;	// pin LOW
                    case 3: CHAN_4_OUT_PORT &= ~(1 << CHAN_4_OUT_BIT); break;	// pin LOW
                    case 4: CHAN_5_OUT_PORT &= ~(1 << CHAN_5_OUT_BIT); break;	// pin LOW
                    case 5: CHAN_6_OUT_PORT &= ~(1 << CHAN_6_OUT_BIT); break;	// pin LOW
                    case 6: CHAN_7_OUT_PORT &= ~(1 << CHAN_7_OUT_BIT); break;	// pin LOW
                    case 7: CHAN_8_OUT_PORT &= ~(1 << CHAN_8_OUT_BIT); break;	// pin LOW
                    }
                }
                uint16_t pulse_width = pwm_out[chan];
                if (pulse_width < MIN_PWM_WIDTH) pulse_width = MIN_PWM_WIDTH;
                else
                    if (pulse_width > MAX_PWM_WIDTH) pulse_width = MAX_PWM_WIDTH;
                pulse_width = (MAX_PWM_WIDTH + INTER_PWM_GAP) - pulse_width;
                OCR1A = (uint16_t)OCR1A + pulse_width;
                if (++pwm_out_index >= (ee.pwm_channels << 1))
                    pwm_out_index = 0;	// back to channel-1
            }
        }
    }

    if (ee.pwm_channels >= MIN_PWM_CHANNELS){		// we are bound to an RC Tx
        if (pwm_out_index == 0						// we are at the start of a frame
            && failsafe_mode						// we are in fail-safe mode
            && ee.failsafe_enabled					// fail-safe mode allowed
            && new_pwm_in_chans != ee.pwm_channels)	// we don't have any new RC PWM values
        {	// move servo outputs SLOWLY towards their fail-safe positions.

            for (uint8_t i = 0; i < ee.pwm_channels; i++) {
                int16_t in = ee.failsafe_pwm[i];
                uint16_t out = pwm_out[i];
                uint16_t diff = abs(in - out);
                if (diff > 5) diff = 5;	// limit the speed the servos move at
                if (out < in) out += diff;
                else
                    if (out > in) out -= diff;
                pwm_out[i] = out;
            }
        }
        else {
            const int chan = pwm_out_index >> 1;
            if (pwm_out_index & 1) {	// end of pulse we are currently outputting
                if (!scanning && new_pwm_in_chans == ee.pwm_channels) {	// fetch the new PWM values we should be outputting
                    for (uint8_t i = 0; i < ee.pwm_channels; i++)
                        pwm_out[i] = new_pwm_in[i];
                    new_pwm_in_chans = 0;
                    failsafe_mode = false;

                    if (ee.switchChannels && (chan >= SWITCH_CHANNELS_FROM)) {

                    }
                    else {
                        // set the unused channel outputs LOW
                        if (ee.pwm_channels < 1) CHAN_1_OUT_PORT &= ~(1 << CHAN_1_OUT_BIT);		// pin LOW
                        if (ee.pwm_channels < 2) CHAN_2_OUT_PORT &= ~(1 << CHAN_2_OUT_BIT);		// pin LOW
                        if (ee.pwm_channels < 3) CHAN_3_OUT_PORT &= ~(1 << CHAN_3_OUT_BIT);		// pin LOW
                        if (ee.pwm_channels < 4) CHAN_4_OUT_PORT &= ~(1 << CHAN_4_OUT_BIT);		// pin LOW
                        if (ee.pwm_channels < 5) CHAN_5_OUT_PORT &= ~(1 << CHAN_5_OUT_BIT);		// pin LOW
                        if (ee.pwm_channels < 6) CHAN_6_OUT_PORT &= ~(1 << CHAN_6_OUT_BIT);		// pin LOW
                        if (ee.pwm_out_mode) {	// the UART is not using the pins
                            if (ee.pwm_channels < 7) CHAN_7_OUT_PORT &= ~(1 << CHAN_7_OUT_BIT);	// pin LOW
                            if (ee.pwm_channels < 8) CHAN_8_OUT_PORT &= ~(1 << CHAN_8_OUT_BIT);	// pin LOW
                        }

                    }
                }
            }
        }
    }
}

// ************************************

ISR(USART0_RX_vect){
    register uint8_t stat = UCSR0A;						// get status
    register uint8_t data = UDR0;						// get received byte

    if (stat & (1 << FE0)) return;						// frame error (faulty stop bit)
    if (stat & (1 << DOR0)) return;						// data overrun (failed to read the previous data byte before the current one came in)
    if (stat & (1 << UPE0)) return;						// parity error

    register uint8_t i = uart_rx_buffer_wr;				// fetch the index
    uart_rx_buffer[i] = data;							// save rx'ed byte
    if (++i >= (uint8_t)sizeof(uart_rx_buffer)) i = 0;	// update the index
    uart_rx_buffer_wr = i;								// save the new index
}

// ************************************
// Process the PPM input stream capturing.
//
// Do this in the exec (rather than interrupt) so as not to interfere with the PPM/PWM waveform output generation.
// Just make sure you call this as often and quickly as possible.

void processInputCaptureScanning(int16_t microsecs) {
    // the received PPM stream is expected to be clean (very strong signal) .. so don't bother with any kind of noise reduction/detection

    if (microsecs > MAX_PWM_WIDTH || pwm_in_index < 0) {	// SYNC pulse, or waiting for a SYNC pulse
        if (pwm_in_index >= MIN_PWM_CHANNELS) {	// we have received channels

            if (pwm_in_frames < 1 || pwm_in_index != pwm_in_prev_chans) {
                pwm_in_frames = 1;
            }
            else {	// same number of channels
                last_ppm_frame_received_timer = 0;	// reset timer
                if (pwm_in_frames < 255) pwm_in_frames++;
            }

            // save the received channel pulse widths
            for (int8_t i = 0; i < pwm_in_index; i++) {
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    new_pwm_in[i] = pwm_in[i];
                }
            }
            new_pwm_in_chans = pwm_in_index;
            pwm_in_prev_chans = pwm_in_index;
        }

        if (microsecs < MIN_SYNC_WIDTH)
        {
            pwm_in_index = -1;								// pulse too short for a SYNC pulse
            pwm_in_frames = 0;
        }
        else
            pwm_in_index = 0;								// start receiving a pulse stream
    }
    else
    {	// CHANNEL pulse
        if (microsecs < MIN_PWM_WIDTH)
        {	// pulse width too small - assume it's noise or corrupted pulse
            pwm_in_index = -1;
            pwm_in_frames = 0;
        }
        else
        {	// a valid pulse width
            if (pwm_in_index < MAX_PWM_CHANNELS)
                pwm_in[pwm_in_index++] = microsecs;			// save the new pulse width value
        }
    }
}

bool processInputCaptureNormal(int16_t microsecs) {
    if (ee.pwm_channels < MIN_PWM_CHANNELS)
        return false;			// we have not yet been bound to an RC Tx .. ignore the input PPM stream

    if (microsecs < MIN_PWM_WIDTH){	// ignore unusually short pulses
        if (pwm_in_bad_pulses < 255) pwm_in_bad_pulses++;
        if (pwm_in_bad_pulses >= 5) {	// to many bad pulses
            pwm_in_index = -1;
            if (failsafe_mode) pwm_in_frames = 0;			// don't come out of fail-safe too easily
        }
        return false;
    }

    if (microsecs > MAX_PWM_WIDTH || pwm_in_index < 0) {	// SYNC pulse, or waiting for a SYNC pulse
#ifdef USART_DEBUG
        uartTxByteWait(0x05);
        uartTxByteWait(pwm_in_index);
        uartTxByteWait(pwm_in_frames);
#endif
        if (pwm_in_index == ee.pwm_channels) {	// same number of channels as the RC TX we are bound too
            if (pwm_in_frames < 255) pwm_in_frames++;
            if (new_pwm_in[0] <= 0) {
                for (int8_t i = 0; i < pwm_in_index; i++) {
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                        new_pwm_in[i] = pwm_in[i];
                    }
                }
            }

            uint16_t new_average_diff = 0;
            for (int8_t i = 0; i < pwm_in_index; i++) {
                const uint16_t diff = abs((int16_t)pwm_in[i] - (int16_t)prev_pwm_in[i]);
                new_average_diff += diff;
            }
            new_average_diff /= pwm_in_index;
            average_diff = (average_diff + new_average_diff) >> 1;

            if (average_diff < MIN_FILTER_VALUE) {
                average_diff = MIN_FILTER_VALUE;	// limit minimum filtering
            }
            else if (average_diff > MAX_FILTER_VALUE) {
                average_diff = MAX_FILTER_VALUE;	// limit maximum filtering
            }
            if (ee.switchChannels) {
                static uint8_t multiSyncCount = 0;
                static uint8_t multiSyncPulse = MULTI_SWITCHES;
                if (pwm_in[ee.multiSwitchChannel] >= MULTI_SYNC_WIDTH) {
                    ++multiSyncCount;
                    if (multiSyncCount == 2) {
                        multiSyncPulse = 0;
                    }
                }
                else {
                    if (multiSyncPulse < MULTI_SWITCHES) {
                        if (pwm_in[ee.multiSwitchChannel] > MULTI_SYNC_HIGH) {
                            switchValues[multiSyncPulse] = 2;
                        }
                        else if (pwm_in[ee.multiSwitchChannel] < MULTI_SYNC_LOW) {
                            switchValues[multiSyncPulse] = 0;
                        }
                        else {
                            switchValues[multiSyncPulse] = 1;
                        }
                        ++multiSyncPulse;
                        if (multiSyncPulse >= MULTI_SWITCHES) {
                            multiSyncCount = 0;
                        }
                    }
                }
            }
            for (int8_t i = 0; i < pwm_in_index; i++) {
                if ((i < MAX_FILTER_CHANNEL) || ee.filter_high_channels) {
                    uint16_t in = (4 * pwm_in[i] + 6 * prev_pwm_in[i]) / 10;
                    uint16_t out = new_pwm_in[i];

                    uint16_t diff = abs(in - out);
                    diff = (diff * MIN_FILTER_VALUE) / average_diff;

                    if (diff < MIN_DIFF) {
                        diff = 0;
                    }
                    else if (diff > MAX_DIFF) {
                        diff = MAX_DIFF;
                    }
                    if (out < in) {
                        out += diff;
                    }
                    else if (out > in) {
                        out -= diff;
                    }

                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                        new_pwm_in[i] = out;
                    }
                }
                else {
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                        new_pwm_in[i] = pwm_in[i];
                    }
                }
            }

            if (pwm_in_frames >= 4) {
                new_pwm_in_chans = pwm_in_index;			// let the pwm output routine use the new values
                failsafe_mode = false;
            }

            // remember this new block of pulse widths
            for (int8_t i = 0; i < pwm_in_index; i++)
                prev_pwm_in[i] = pwm_in[i];

            last_ppm_frame_received_timer = 0;				// reset PPM frame timer
        }
        else {	// wrong number of channels
            if (failsafe_mode && pwm_in_index > 0)
                pwm_in_frames = 0;
        }

        pwm_in_index = 0;

        if (microsecs >= MIN_SYNC_WIDTH) {	// a good SYNC pulse
            pwm_in_bad_pulses = 0;
        }
        else {	// a rather short SYNC pulse
            if (pwm_in_bad_pulses < 255) pwm_in_bad_pulses++;
            if (failsafe_mode) pwm_in_frames = 0;			// don't come out of fail-safe too easily
            return false;
        }
    }
    else {	// CHANNEL pulse
        if (pwm_in_index < MAX_PWM_CHANNELS)
            pwm_in[pwm_in_index++] = microsecs;				// save the new pulse width value
    }
    return true;
}

void processInputCapture(void) {
    const uint8_t flags = TIFR1 & ((1 << TOV1) | (1 << ICF1));
    if (!flags)
        return;										// nothing to process

    TIFR1 = flags;									// reset flags

    //	if (flags & (1 << TOV1))
    //	{	// timer-1 overflowed
    //	}

    if (!(flags & (1 << ICF1)))
        return;										// input capture not yet triggered

    const uint16_t icr1 = (uint16_t)ICR1;					// read the captured timer value
    // const uint16_t microsecs = (uint16_t)((int16_t)icr1 - prev_icr1);

    uint16_t microsecs;
    if (icr1 < prev_icr1)
        microsecs = icr1 + (65535u - prev_icr1) + 1u;
    else
        microsecs = icr1 - prev_icr1;
    prev_icr1 = icr1;

    if (scanning) {	// looking a new RC Tx to bind with
        processInputCaptureScanning(microsecs);
    }
    else {	// normal mode
        if (!processInputCaptureNormal(microsecs))
            return;
    }
    // prev_icr1 = icr1;
}

// ************************************

void setPLL(uint32_t data) {
    PLL_CLK_OUT_PORT &= ~(1 << PLL_CLK_OUT_BIT);			// pin LOW
    PLL_CE_OUT_PORT &= ~(1 << PLL_CE_OUT_BIT);				// pin LOW
    _delay_us(16);
    PLL_CE_OUT_PORT |= 1 << PLL_CE_OUT_BIT;					// pin HIGH
    _delay_us(10);
    for (int8_t i = 24; i > 0; i--) {
        const bool b = data & 1;
        data >>= 1;
        if (!b)
            PLL_DATA_OUT_PORT &= ~(1 << PLL_DATA_OUT_BIT);	// pin LOW
        else
            PLL_DATA_OUT_PORT |= 1 << PLL_DATA_OUT_BIT;		// pin HIGH
        _delay_us(8);
        PLL_CLK_OUT_PORT |= 1 << PLL_CLK_OUT_BIT;			// pin HIGH
        _delay_us(8);
        PLL_CLK_OUT_PORT &= ~(1 << PLL_CLK_OUT_BIT);		// pin LOW
        _delay_us(8);
    }
    _delay_us(8);
    PLL_DATA_OUT_PORT &= ~(1 << PLL_DATA_OUT_BIT);			// pin LOW
    PLL_CE_OUT_PORT &= ~(1 << PLL_CE_OUT_BIT);				// pin LOW
}

void setPLLChannel(uint16_t channel) {
    if (channel < START_CHAN) channel = START_CHAN;
    else
        if (channel > END_CHAN) channel = END_CHAN;

    ee.rf_channel = channel;

    uint16_t ref_div = (PLL_REF_XTAL / PLL_REF_FREQ) & 0x3fff;

    rx_freq = (uint32_t)CH0_FREQ + ((uint32_t)CH_STEP * channel);

    uint32_t vco_freq = rx_freq + PLL_VCO_OFFSET;
    uint16_t pll_div = vco_freq / PLL_REF_FREQ;

    // set RX reference divider
    setPLL(PLL_ID0 | PLL_ID1 | ref_div);

    // set RX programmable divider
    setPLL(PLL_OFF | PLL_ID0 | pll_div);

    // set TX reference divider
    setPLL(PLL_ID1 | ref_div);

    // set TX programmable divider
    setPLL(pll_div);
}

// ************************************

void enableAnaComp(void) {
    // this uses the AIN0 and ADC1 inputs - so we can't use the normal ADC while we are using the AC

    PRR0 &= ~(1 << PRADC);			// power up the adc

    // disable ADC
    ADCSRA = (0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (1<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);

    // Enable Analogue Comparator
    // Comparator Interrupt on Rising Output Edge
    // clear int flag (by setting it to one)
    ACSR = (0<<ACD)|(0<<ACBG)|(0<<ACO)|(1<<ACI)|(0<<ACIE)|(1<<ACIC)|(1<ACIS1)|(1<ACIS0);

    // disable AIN0 & AIN1 digital input buffers - they not used and just use up power
    DIDR1 = (1<<AIN1D) | (1<<AIN0D);

    // use ADC1 as negative input
    ADCSRB = (1<<ACME) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);
    ADMUX = (0<<REFS1) | (0<<REFS0) | (0<<ADLAR) | (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (1<<MUX0);

    // Enable Analogue Comparator Interrupt
    //	ACSR |= 1 << ACIE;
}

// ************************************

void enableADC(void) {
    PRR0 &= ~(1 << PRADC);		// power up the adc

    // disable Analogue Comparator
    ACSR = (1<<ACD) | (0<<ACBG) | (0<<ACO) | (1<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);

    // enable ADC
    // ADC clock = CPU clock / 64 = 8MHz/64 = 125kHz
    // clear the ADIF flag (by setting it to one)
    ADCSRA = (1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (1<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);

    // ADC0 as input
    // ADC reference = external voltage applied to AREF pin
    // Right adjusted ADC read value
    ADMUX = (0<<REFS1) | (0<<REFS0) | (0<<ADLAR) | (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (0<<MUX0);

    // disable ADC0 digital input
    DIDR0 = (0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (1<<ADC0D);

    // set ADC Auto Trigger Source to free running
    ADCSRB = (0<<ACME) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

    // start first conversion
    ADCSRA |= (1<<ADSC) | (1<<ADIF);
}

int16_t readADC(void) {
    if (!(ADCSRA & (1 << ADIF)))
        return -1;						// ADC is still sampling
    uint16_t adc = ADC;					// read the ADC register
    ADCSRA |= (1<<ADSC) | (1<<ADIF);	// start next conversion and clear the ADIF flag
    return (int16_t)adc;
}

// ************************************

void startTimer0(void) {
    uint8_t sreg = SREG;
    cli();

    PRR0 &= ~(1 << PRTIM0);		// power the timer

    TIMSK0 = 0;					// disable timer interrupts
    TCCR0B = 0;					// stop timer
    TCCR0A = 0;					// normal mode
    TCNT0 = 0;					// reset timer

    last_ppm_frame_received_timer = 255;		// reset counter

    // Normal mode
    // IOclk/1024 (128us)
    // 32 ms OVF
    TCCR0A = (0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
    TCCR0B = (0<<FOC0A) | (0<<FOC0B) | (0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);

    // clear all pending interrupt flags
    TIFR0 = (1<<OCF0B) | (1<<OCF0A) | (1<<TOV0);

    SREG = sreg;
}

// ************************************

void startTimer1(void) {
    uint8_t sreg = SREG;
    cli();

    PRR0 &= ~(1 << PRTIM1);		// power the timer

    TIMSK1 = 0;					// disable timer interrupts
    TCCR1C = 0;					//
    TCCR1B = 0;					// stop timer
    TCCR1A = 0;					// normal mode
    TCNT1 = 0;					// reset timer

    allOutputsLow();

    OCR1A = 2000;				// 2ms

    // Normal mode
    // IOclk/8 (1us)
    // Enable input capture noise canceler
    // Capture on positive edge of the input capture
    //	// Toggle OC1A pin on output compare
    //	TCCR1A = (0<<COM1A1) | (1<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
    TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B = (1<<ICNC1) | (1<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
    TCCR1C = (0<<FOC1A) | (0<<FOC1B);

    // clear all pending interrupt flags
    TIFR1 = (1<<ICF1) | (1<<OCF1B) | (1<<OCF1A) | (1<<TOV1);

    // enable output compare 'A' interrupt
    TIMSK1 = (0<<ICIE1) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);

    SREG = sreg;
}

// ************************************

void stopUART(void)
{
    uint8_t sreg = SREG;
    cli();

    if (!(PRR0 & (1 << PRUSART0)))
    {
        // disable interrupts, disable RX and TX
        UCSR0B = (0<<TXCIE0) | (0<<RXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02);

        // power down the UART module
        PRR0 |= 1 << PRUSART0;
    }

    UART_RXD_DIR |= 1 << UART_RXD_BIT;				// pin as output
    UART_RXD_PORT &= ~(1 << UART_RXD_BIT);			// pin LOW

    UART_TXD_DIR |= 1 << UART_TXD_BIT;				// pin as output
    UART_TXD_PORT &= ~(1 << UART_TXD_BIT);			// pin LOW

    SREG = sreg;
}

void startUART(void)
{

    uint8_t sreg = SREG;
    cli();

    // power up the UART module
    PRR0 &= ~(1 << PRUSART0);

    UART_RXD_DIR |= 1 << UART_RXD_BIT;				// pin as input
    UART_RXD_PORT |= 1 << UART_RXD_BIT;				// enable pull-up

    UART_TXD_DIR |= 1 << UART_TXD_BIT;				// pin as output
    UART_TXD_PORT |= 1 << UART_TXD_BIT;				// pin HIGH

    // set the baud rate, *1 speed, normal UART
    UBRR0 = (uint16_t)((F_CPU / (16.0 * UART_BAUDRATE)) - 0.5);
    UCSR0A = (0<<U2X0) | (0<<MPCM0);

    // set the baud rate, *2 speed, normal UART
    //	UBRR0 = (uint16_t)((FOSC / (8.0 * UART_BAUDRATE)) - 0.5);
    //	UCSR0A = (1<<U2X0) | (0<<MPCM0);

    // asynchronous USART, no parity, 1 stop bit, 8 data bits
    UCSR0C = (0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);

    // enable RX and TX, enable RX interrupt
    UCSR0B = (0<<TXCIE0) | (1<<RXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02);

    SREG = sreg;
}

inline void uartTxByteWait(uint8_t b)
{
    if (PRR0 & (1 << PRUSART0)) return;				// the UART is not enabled
    while (!(UCSR0A & (1 << UDRE0)));				// wait until previous byte sent
    UDR0 = b;										// send byte
}

inline bool uartTxByteNoWait(uint8_t b)
{
    if (PRR0 & (1 << PRUSART0)) return false;		// the UART is not enabled
    if (!(UCSR0A & (1 << UDRE0))) return false;		// previous byte has not yet been sent
    UDR0 = b;										// send byte
    return true;
}

// ************************************

void processExec(void)
{
    if (TIFR0 & (1 << TOV0))
    {	// timer-0 overflowed: 32ms
        TIFR0 |= 1 << TOV0;	// reset flag

        if (last_ppm_frame_received_timer < 255)
            last_ppm_frame_received_timer++;

        if (!(BUT_PIN & (1 << BUT_BIT)))
        {	// button pressed
            button_released_counter = 0;
            if (button_pressed_counter < 255) button_pressed_counter++;
        }
        else
        {	// button not pressed
            button_pressed_counter = 0;
            if (button_released_counter < 255) button_released_counter++;
        }

        msCounter++;
    }

    if (scanning)
        return;									// we are scanning for a valid RC Tx

    processInputCapture();

    if (!failsafe_mode									// we are not in fail-safe mode
        && ee.pwm_channels >= MIN_PWM_CHANNELS			// we are bound to an RC Tx
        && last_ppm_frame_received_timer >= 16)	    // not received any valid PPM frames in the last 512ms
    {											// enter fail-safe mode then
        setPLLChannel(ee.rf_channel);			// just incase the PLL has lost it's settings
        //		useFailSafeValues();					// fall back to the failsafe settings

        failsafe_mode = true;					// fall back to the failsafe settings
        pwm_in_frames = 0;
        pwm_in_index = -1;
#ifdef USART_DEBUG
        uartTxByteWait(0x03);
#endif
    }

    if (state != state_normal)
        return;									// we are not in Normal mode

    if (button_pressed_counter >= 3)
    {											// the button is pressed
        LED_PORT &= ~(1 << LED_BIT);			// led OFF
        return;
    }

    if (ee.pwm_channels < MIN_PWM_CHANNELS)
    {											// we are not bound to an RC Tx .. continuously flash the LED
        if ((msCounter & 0x001f) < 16)
            LED_PORT &= ~(1 << LED_BIT);		// led OFF
        else
            LED_PORT |= 1 << LED_BIT;			// led ON
#ifdef USART_DEBUG
        uartTxByteWait(0x02);
        uartTxByteWait(ee.pwm_channels);
#endif
        return;
    }

    // we are bound to an RC Tx

    if (failsafe_mode || pwm_in_frames < 2)
    {											// we are not receiving a valid signal
        if ((msCounter & 0x003f) < 2)
            LED_PORT |= 1 << LED_BIT;			// led ON ... blink the led breifly
        else
            LED_PORT &= ~(1 << LED_BIT);		// led OFF
        return;
    }

    // we're receiving a valid signal

    LED_PORT |= 1 << LED_BIT;					// led ON
}

// ************************************
// scan the band for the strongest valid RC TX
// if we find one then bind to it

void scan(void)
{
    scanning = true;

    // useFailSafeValues();								// use the fail-safe values while scanning for a transmitter
    failsafe_mode = true;

    LED_PORT &= ~(1 << LED_BIT);						// led OFF

    uint16_t prev_rf_chan = ee.rf_channel;

    uint16_t rf_chan = 0;
    uint32_t rf_rssi = 0;
    uint8_t pwm_chans = 0;

    uint16_t chan = START_CHAN;

    while (chan <= END_CHAN)
    {
        LED_PIN |= 1 << LED_BIT;						// led TOGGLE

        enableADC();									// we want to monitor the RSSI level

        setPLLChannel(chan);							// onto next channel
        if (chan <= START_CHAN)
            _delay_ms(30);	// wait 30ms .. give the PLL time to lock - takes around 25ms for large frequency change
        else
            _delay_ms(10);	// wait 10ms .. give the PLL time to lock - takes around 6ms for 10kHz change in frequency

        // get an average RSSI value over a 40ms period
        while (readADC() < 0);							// wait for first ADC conversion
        uint32_t rssi = 0;
        uint16_t rssi_num = 0;
        while (rssi_num < (uint16_t)(0.04 * (125000 / 11)))
        {
            int16_t adc = readADC();					// fetch adc value
            if (adc < 0) continue;
            rssi += adc;
            rssi_num++;
        }
        rssi /= rssi_num;

        // RSSI noise floor (no signal received) is around 270, a strong signal is around 620 (or more)
        if (rssi >= 580 && rssi > rf_rssi)
        {	// we've found a strong signal - stronger than the last one we found
            // we need to check the PPM stream to see if it's valid

            enableAnaComp();
            pwm_in_frames = 0;
            pwm_in_bad_pulses = 0;
            pwm_in_index = -1;
            pwm_in_prev_chans = -1;
            new_pwm_in_chans = 0;
            uint8_t ticks = 0;
            TCNT0 = 0;									// reset timer
            TIFR0 |= 1 << TOV0;							// reset flag
            while (ticks < 7)							// stay on this channel for upto 230ms - look for a valid PPM stream
            {
                processInputCapture();

                if (pwm_in_frames >= 6 && new_pwm_in_chans >= MIN_PWM_CHANNELS)
                {	// we are receiving a valid PPM stream, remember it
                    rf_chan = chan;
                    rf_rssi = rssi;
                    pwm_chans = new_pwm_in_chans;
                    break;
                }

                if (TIFR0 & (1 << TOV0))
                {	// this happens once every 32.864ms
                    TIFR0 |= 1 << TOV0;
                    ticks++;
                }
            }
        }

        chan++;												// next RF channel

        wdt_reset();
    }

    LED_PORT &= ~(1 << LED_BIT);							// led OFF

    if (rf_chan > 0 && pwm_chans >= MIN_PWM_CHANNELS)
    {	// we found an RC Tx to bind with, so lets do it!

        ee.rf_channel = rf_chan;							// the RF channel the TX is on

        ee.pwm_channels = pwm_chans;						// number of RC channels the RC TX has

        if (ee.pwm_channels != pwm_chans || rf_chan != prev_rf_chan)
            ee.failsafe_enabled = false;					// they will need to reset the fail-safe values

        // decide on a suitable PPM frame rate (depends on how many pwm channels the RC Tx has)
        uint16_t ppm_length = (pwm_chans * 2000) + 8000;
        if (ppm_length < MIN_PPM_FRAME_LENGTH) ppm_length = MIN_PPM_FRAME_LENGTH;
        else
            if (ppm_length > MAX_PPM_FRAME_LENGTH) ppm_length = MAX_PPM_FRAME_LENGTH;
        ee.ppm_frame_length = ppm_length;

        // save the new settings into eeprom
        eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
        eeprom_busy_wait();

#ifdef USART_DEBUG
        uartTxByteWait(0x04);
        uartTxByteWait(ee.pwm_channels);
        uartTxByteWait(ee.rf_channel);
#endif

        setPLLChannel(rf_chan);							// switch back to the channel with the strongest signal
        _delay_ms(30);
    }
    else
    {	// we didunt find an RC Tx to bind with .. leave things as they were
        setPLLChannel(prev_rf_chan);
        _delay_ms(30);
    }

    last_ppm_frame_received_timer = 255;

    pwm_in_frames = 0;
    pwm_in_bad_pulses = 0;
    pwm_in_index = -1;
    pwm_in_prev_chans = -1;
    new_pwm_in_chans = 0;
    average_diff = 1000;
    for (int i = 0; i < MAX_PWM_CHANNELS; i++)
        new_pwm_in[i] = 0;

    enableAnaComp();

    scanning = false;
}

// ************************************

bool processState(void)
{
    if (state == state_normal)
    {	// normal
        if (button_pressed_counter >= 32 * 2)		// button been pressed for 2 seconds?
        {											// yes
            state++;
            msCounter = 0;
            LED_PORT &= ~(1 << LED_BIT);			// led OFF
        }
        return true;
    }

    if (state == state_but_release)
    {	// wait for button release
        if (button_released_counter < 48)			// button been released for 1.5 second ?
        {											// not yet
            if (button_pressed_counter <= 0)		// button still pressed ?
            {										// no
                LED_PORT &= ~(1 << LED_BIT);		// led OFF
            }
            else
            {										// button is still pressed
                if (!(msCounter & 2))				// flash the LED until they release the button
                    LED_PORT |= 1 << LED_BIT;		// led ON
                else
                    LED_PORT &= ~(1 << LED_BIT);	// led OFF
            }
            return true;
        }
        state++;									// next config option
        msCounter = 0;
        LED_PORT |= 1 << LED_BIT;					// led ON
        return true;
    }

    // config option state

    if (msCounter < 6)								// wait until the LED has been on for a while
    {
        if (msCounter < 2)
            button_pressed_counter = 0;
        return true;
    }

    LED_PORT &= ~(1 << LED_BIT);					// led OFF

    if (button_pressed_counter < 3)					// button pressed?
    {												// not yet
        if (msCounter >= 48)							// 1.5 second time-out waiting for button press
        {
            state++;								// next config option
            msCounter = 0;
            if (state >= state_last)
                return false;						// back to normal mode
            LED_PORT |= 1 << LED_BIT;				// led ON
        }
        return true;
    }

    // button pressed on a config option

    switch (state)
    {
    case state_scan_option:						// scan option - bind to an RC Tx
        scan();
        break;
    case state_enable_filter_option:
        ee.filter_high_channels = true;
        eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
        eeprom_busy_wait();
        break;
    case state_disable_filter_option:
        ee.filter_high_channels = false;
        eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
        eeprom_busy_wait();
        break;
    case state_enable_switch_option:
        ee.switchChannels = true;
        eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
        eeprom_busy_wait();
        break;
    case state_disable_switch_option:
        ee.switchChannels = false;
        eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
        eeprom_busy_wait();
        break;
    case state_disable_failsafe_option:			// disable fail-safe option
        ee.failsafe_enabled = false;
        eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
        eeprom_busy_wait();
        break;
    case state_enable_failsafe_option:			// enable fail-safe option
        if (!failsafe_mode && ee.pwm_channels >= MIN_PWM_CHANNELS)
        {	// save the current output pulse widths as the fail-safe values
            for (int i = 0; i < ee.pwm_channels; i++) ee.failsafe_pwm[i] = pwm_out[i];
            for (int i = ee.pwm_channels; i < MAX_PWM_CHANNELS; i++) ee.failsafe_pwm[i] = 1000;
            ee.failsafe_enabled = true;
            eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
            eeprom_busy_wait();
        }
        else
        {	// we are not currently receiving a valid RC Tx
        }
        break;

    case state_enable_pwm_option:				// enable PWM output mode
        cli();
        stopUART();
        allOutputsLow();
        pwm_out_index = 0;					// back to channel-1
        ee.pwm_out_mode = true;
        eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
        eeprom_busy_wait();
        sei();
        break;

    case state_enable_ppm_option:				// enable PPM output mode
        cli();
        allOutputsLow();
        pwm_out_index = 0;					// back to channel-1
        pwm_out_total = ee.ppm_frame_length;
        ee.pwm_out_mode = false;
        eeprom_write_block((void*)(&ee), (void*)(&eeprom), sizeof(T_EEPROM));
        eeprom_busy_wait();
        startUART();
        sei();
        break;

    case state_defaults_option:					// restore defaults
        cli();
        allOutputsLow();
        setDefaultSettings();
        sei();
        break;
    }
    return false;
}

// ************************************

int main(void)
{
    cli();

    CLKPR = 1 << CLKPCE;					// Enable main clock prescaler change
    CLKPR = 0;								// prescaler = 1

    MCUCR = 1 << IVCE;						// enable interrupt vector change
    MCUCR = 0;								// ensure interrupt vectors go to start of flash (not the boot loader section of the flash)

    EIMSK = 0;								// disable external interrupts
    PCICR = 0;								// disable pin change interrupts

    GTCCR = (1 << PSRASY) | (1 << PSRSYNC);	// reset timer/counter prescalers

    SMCR = 0;								// disable sleep mode
    //	SMCR = (0<<SM2)|(0<<SM1)|(0<<SM0)|(1<<SE);	// set sleep mode to IDLE

    PCMSK0 = 0;								// disable PCINT 7-0
    PCMSK1 = 0;								// disable PCINT 14-8
    PCMSK2 = 0;								// disable PCINT 23-16

    // Turn off all modules (to save power) .. they will be turned back on as & when they are needed
    ACSR |= (1 << ACD);						// disable the analogue comparator
    PRR0 = ((1<<PRTWI0) | (1<<PRTIM2) | (1<<PRTIM0) | (1<<PRTIM1) | (1<<PRSPI0) | (1<<PRUSART0) | (1<<PRADC));

    // all PORT pins as inputs
    DDRB = 0;
    DDRC = 0;
    DDRD = 0;

    // *******************

    BUT_PORT |= 1 << BUT_BIT;						// enable pull-up

    //	UART_RXD_PORT |= 1 << UART_RXD_BIT;				// enable pull-up

    LED_DIR |= 1 << LED_BIT;						// pin as output
    LED_PORT &= ~(1 << LED_BIT);					// led off

    //	OC1A_DIR |= 1 << OC1A_BIT;						// pin as output
    //	OC1A_PORT &= ~(1 << OC1A_BIT);					// pin LOW

    PLL_CE_OUT_DIR |= 1 << PLL_CE_OUT_BIT;			// pin as output
    PLL_CE_OUT_PORT &= ~(1 << PLL_CE_OUT_BIT);		// pin LOW

    PLL_DATA_OUT_DIR |= 1 << PLL_DATA_OUT_BIT;		// pin as output
    PLL_DATA_OUT_PORT &= ~(1 << PLL_DATA_OUT_BIT);	// pin LOW

    PLL_CLK_OUT_DIR |= 1 << PLL_CLK_OUT_BIT;		// pin as output
    PLL_CLK_OUT_PORT &= ~(1 << PLL_CLK_OUT_BIT);	//  pin LOW

    CHAN_1_OUT_DIR |= 1 << CHAN_1_OUT_BIT;			// pin as output
    CHAN_2_OUT_DIR |= 1 << CHAN_2_OUT_BIT;			// pin as output
    CHAN_3_OUT_DIR |= 1 << CHAN_3_OUT_BIT;			// pin as output
    CHAN_4_OUT_DIR |= 1 << CHAN_4_OUT_BIT;			// pin as output
    CHAN_5_OUT_DIR |= 1 << CHAN_5_OUT_BIT;			// pin as output
    CHAN_6_OUT_DIR |= 1 << CHAN_6_OUT_BIT;			// pin as output
    CHAN_7_OUT_DIR |= 1 << CHAN_7_OUT_BIT;			// pin as output
    CHAN_8_OUT_DIR |= 1 << CHAN_8_OUT_BIT;			// pin as output

    allOutputsLow();

    // *******************
    // Check reset cause

    uint8_t resetFlags = MCUSR;				// read reset cause
    MCUSR = 0x00;							// clear reset cause

    if (resetFlags & (1 << BORF))
    {	// Brown-out reset
    }
    else
        if (resetFlags & (1 << WDRF))
        {	// Watchdog reset
        }
        else
            if (resetFlags & ((1 << PORF) | (1 << EXTRF)))
            {	// Power-on and/or External reset
            }

    // *******************
    // setup the watchdog timer to 1 second timeout

    wdt_reset();

    WDTCSR = (1<<WDCE) | (1<<WDE);
    WDTCSR = (1<<WDCE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
    //	WDTCSR |= (1<<WDIF) | (1<<WDIE);
    WDTCSR |= (1<<WDIF);

    // *******************
    // read the eeprom values .. and correct them if they are invalid
    // read the values
    eeprom_busy_wait();
    eeprom_read_block((void*)(&ee), (const void*)(&eeprom), sizeof(T_EEPROM));

#ifdef USART_DEBUG
    startUART();
    uartTxByteWait(0x00);
    uartTxByteWait(ee.rf_channel);
    uartTxByteWait(ee.pwm_channels);
#endif

    if (ee.magic_number != EEPROM_VERSION) {
        setDefaultSettings();		// an error was found in the values we read from eeprom .. revert to default settings
    }

    // *******************

    state = state_normal;

    rx_freq = 0;

    last_ppm_frame_received_timer = 255;

    failsafe_mode = true;

    msCounter = 0;

    scanning = false;

    button_pressed_counter = 0;
    button_released_counter = 0;

    prev_icr1 = 0;

    pwm_out_index = 0;
    pwm_out_total = ee.ppm_frame_length;

    uart_rx_buffer_rd = 0;
    uart_rx_buffer_wr = 0;

    pwm_in_frames = 0;
    pwm_in_bad_pulses = 0;
    pwm_in_index = -1;
    pwm_in_prev_chans = -1;
    new_pwm_in_chans = 0;
    average_diff = 1000;
    for (int i = 0; i < MAX_PWM_CHANNELS; i++)
        new_pwm_in[i] = 0;

    if (!ee.pwm_out_mode)
        startUART();		// we can use the UART pins - we are in PPM output mode

    // *******************

    useFailSafeValues();

    _delay_ms(50);

    enableAnaComp();

    startTimer0();
    startTimer1();

    // *******************

    // failsafe_mode = false;
    sei();

    while (true)
    {
        wdt_reset();

        // flash the led 2 times
        for (int i = 2; i > 0; i--)
        {
            LED_PORT |= 1 << LED_BIT;						// led ON
            _delay_ms(70);
            LED_PORT &= ~(1 << LED_BIT);					// led OFF
            _delay_ms(150);
        }

        setPLLChannel(ee.rf_channel);

        state = state_normal;
        msCounter = 0;

        while (true)
        {
            wdt_reset();

            //feedWatchdog();

            processExec();

            if (!processState())
                break;
        }
    }
    return 0;
}
